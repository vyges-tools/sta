// SPDX-License-Identifier: Apache-2.0
//! An exhaustive interleaving model (tokio-rs loom, as `loom_model`) of the parallel visit in
//! `vyges_sta::par::map` and its use by the passes: workers claim chunks from an atomic counter,
//! keep `(index, value)` parts, the caller joins them all (the wave barrier), merges with the real
//! [`vyges_sta::par::merge_by_index`] and applies in order; the next wave computes from what the
//! last stored.
//!
//! Properties, checked in every interleaving loom explores:
//! 1. no wave starts before the previous one is applied;
//! 2. the merged order does not depend on the schedule;
//! 3. every vertex is computed exactly once (none lost, none duplicated).
//!
//! Each property has a mutant of the model that loom must catch (`PAR_MODEL_MUTANT`).

use loom_model::sync::atomic::{AtomicUsize, Ordering};
use loom_model::sync::{Arc, Mutex};
use loom_model::thread;

#[derive(Clone, Copy, PartialEq)]
enum Mutant {
    None,
    /// The next wave is started before the workers of this one are joined.
    NoBarrier,
    /// Results taken in completion order instead of by index.
    CompletionOrder,
    /// A chunk claimed by a load then a store, not one `fetch_add`.
    SplitClaim,
}

const CHUNK: usize = 1;

/// A worker's `(index, value)` results.
type Part = Vec<(usize, usize)>;
/// A started wave: the worker's handle, the caller's own part, the values in completion order.
type Started = (thread::JoinHandle<Part>, Part, Arc<Mutex<Vec<usize>>>);

/// One wave of `n` items on the calling thread and one worker; item `i` computes
/// `10 * i + input[i]`, counting its computes in `computed`.
fn wave(n: usize, input: Arc<Vec<usize>>, computed: Arc<Vec<AtomicUsize>>, applied: Arc<AtomicUsize>, need_applied: usize, m: Mutant) -> Started {
    let next = Arc::new(AtomicUsize::new(0));
    let done = Arc::new(Mutex::new(Vec::new()));
    let work = {
        let (next, input, computed, applied, done) = (next.clone(), input.clone(), computed.clone(), applied.clone(), done.clone());
        move || {
            let mut part = Vec::new();
            loop {
                let from = if m == Mutant::SplitClaim {
                    let f = next.load(Ordering::SeqCst);
                    next.store(f + CHUNK, Ordering::SeqCst);
                    f
                } else {
                    next.fetch_add(CHUNK, Ordering::SeqCst)
                };
                if from >= n {
                    return part;
                }
                for i in from..(from + CHUNK).min(n) {
                    // Property 1: the previous wave is fully applied before any compute here.
                    assert_eq!(applied.load(Ordering::SeqCst), need_applied, "a wave started before the previous one was applied");
                    computed[i].fetch_add(1, Ordering::SeqCst);
                    part.push((i, 10 * i + input[i]));
                    done.lock().unwrap().push(10 * i + input[i]);
                }
            }
        }
    };
    let h = thread::spawn(work.clone());
    let mine = work();
    (h, mine, done)
}

fn model(m: Mutant) {
    let n = 2;
    let computed: Arc<Vec<AtomicUsize>> = Arc::new((0..2 * n).map(|_| AtomicUsize::new(0)).collect());
    let applied = Arc::new(AtomicUsize::new(0));
    let store = Arc::new(Mutex::new(vec![0usize; n]));
    // Wave 1: inputs 0.
    let (h1, mine1, done1) = wave(n, Arc::new(vec![0; n]), computed.clone(), applied.clone(), 0, m);
    let parts1 = if m == Mutant::NoBarrier {
        // Wave 2 is started from what is stored so far, before wave 1 is joined and applied.
        let early = Arc::new(store.lock().unwrap().clone());
        let c2 = Arc::new((0..n).map(|_| AtomicUsize::new(0)).collect::<Vec<_>>());
        let (h2, _m2, _d2) = wave(n, early, c2, applied.clone(), n, m);
        let p = vec![mine1, h1.join().unwrap()];
        h2.join().unwrap();
        p
    } else {
        vec![mine1, h1.join().unwrap()]
    };
    let merged: Vec<usize> = if m == Mutant::CompletionOrder { done1.lock().unwrap().clone() } else { vyges_sta::par::merge_by_index(n, parts1) };
    // Property 2: the merge is in index order, whatever the schedule.
    assert_eq!(merged, (0..n).map(|i| 10 * i).collect::<Vec<_>>(), "merge order depends on the schedule");
    for (i, x) in merged.into_iter().enumerate() {
        store.lock().unwrap()[i] = x;
        applied.fetch_add(1, Ordering::SeqCst);
    }
    // Property 3: each vertex computed exactly once.
    for i in 0..n {
        assert_eq!(computed[i].load(Ordering::SeqCst), 1, "vertex {i} lost or duplicated");
    }
    // Wave 2 reads wave 1's stores.
    let input2 = Arc::new(store.lock().unwrap().clone());
    let c2: Arc<Vec<AtomicUsize>> = Arc::new((0..n).map(|_| AtomicUsize::new(0)).collect());
    let (h2, mine2, _d2) = wave(n, input2, c2.clone(), applied.clone(), n, m);
    let merged2 = vyges_sta::par::merge_by_index(n, vec![mine2, h2.join().unwrap()]);
    assert_eq!(merged2, (0..n).map(|i| 20 * i).collect::<Vec<_>>());
}

fn check(m: Mutant) {
    let mut b = loom_model::model::Builder::new();
    b.preemption_bound = Some(3);
    b.check(move || model(m));
}

/// The model with no mutant — or, with `PAR_MODEL_MUTANT` set (`no-barrier`, `completion-order`,
/// `split-claim`), that mutant, which must fail: a panic inside a loom model aborts the process,
/// so a mutant is judged by the run's exit status and message, not by `#[should_panic]`.
#[test]
fn the_parallel_visit_holds_in_every_interleaving() {
    let m = match std::env::var("PAR_MODEL_MUTANT").as_deref() {
        Ok("no-barrier") => Mutant::NoBarrier,
        Ok("completion-order") => Mutant::CompletionOrder,
        Ok("split-claim") => Mutant::SplitClaim,
        Ok(other) => panic!("unknown mutant {other}"),
        Err(_) => Mutant::None,
    };
    check(m);
}
