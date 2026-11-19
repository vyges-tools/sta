// SPDX-License-Identifier: Apache-2.0
//! Timing passes on several threads, with results bit-identical at any thread count.
//!
//! The reference timer visits a pass level by level: every vertex of a level may run on any
//! thread, and the next level starts only when the whole level is done. Each visit reads lower
//! levels and writes its own vertex's results, so the values do not depend on the order inside a
//! level.
//!
//! Here a pass is a pure per-vertex COMPUTE (reads, writes nothing) and an APPLY (stores the
//! results), run in this order:
//! 1. [`waves`]: the serial visiting order is cut into waves. Each vertex declares the state cells
//!    its visit reads, writes, or sets (an idempotent flag such as "queued"). A vertex goes in the
//!    first wave after every earlier conflicting access, so every conflicting pair keeps its serial
//!    order. This holds across edges that levels do not order (latch D->Q, bidirect paths, broken
//!    loops), so the result equals the serial one by construction. On plain logic the waves are the
//!    levels.
//! 2. [`map`]: a wave's computes run on the threads. Each wave is a barrier: it returns only after
//!    every compute of the wave has finished.
//! 3. The caller applies the results one at a time, in wave order, which is the serial order.
//!
//! A thread count of 1 never calls [`waves`]: callers keep their serial loop. A wave with fewer
//! vertices than threads is computed on the calling thread, as the reference computes a small level.
//!
//! No `unsafe`: the compute borrows the timer shared (`&`), the apply borrows it exclusively
//! (`&mut`), and the borrow checker keeps the two phases apart.

use std::sync::atomic::{AtomicUsize, Ordering};

/// Vertices a worker claims at a time: the reference's chunk size (its runtime curve is flat
/// from there).
pub const CHUNK: usize = 8;

/// The reference's thread-count rule (`set_thread_count`): `<= 0` asks for every hardware thread,
/// more than the machine has is clamped to it.
pub fn resolve_thread_count(requested: i32) -> usize {
    let max = std::thread::available_parallelism().map_or(1, |n| n.get());
    if requested <= 0 {
        max
    } else {
        (requested as usize).min(max)
    }
}

/// The default thread count of a new timer: 1, or `VYGES_STA_THREADS` when it is set (a gating
/// override, resolved by [`resolve_thread_count`]).
pub fn default_thread_count() -> usize {
    match std::env::var("VYGES_STA_THREADS").ok().and_then(|s| s.trim().parse::<i32>().ok()) {
        Some(n) => resolve_thread_count(n),
        None => 1,
    }
}

/// `VYGES_STA_STAGE_TIMES=1`: each pass prints `sta stage <name> threads=<n> secs=<wall>` to
/// stderr when it ends (a measuring aid; nothing it prints feeds a result).
pub struct StageTimer {
    name: &'static str,
    threads: usize,
    started: Option<std::time::Instant>,
}

fn timing_on() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("VYGES_STA_STAGE_TIMES").is_some_and(|v| v != "0"))
}

/// Where a stage's time goes, accumulated while [`timing_on`] and printed by the [`StageTimer`]
/// that ends the stage: [`map`] calls (and how many ran on the calling thread alone because they
/// had fewer items than threads), the items they visited and their wall time; [`waves`]; and any
/// other named part ([`add_time`]). The rest of the stage is serial work on the calling thread.
#[derive(Default)]
struct Breakdown {
    maps: usize,
    serial_maps: usize,
    items: usize,
    map_secs: f64,
    parts: Vec<(&'static str, f64)>,
}

static BREAKDOWN: std::sync::Mutex<Breakdown> = std::sync::Mutex::new(Breakdown { maps: 0, serial_maps: 0, items: 0, map_secs: 0.0, parts: Vec::new() });

/// Adds `secs` to the named part of the running stage's breakdown (no-op unless timing is on).
pub fn add_time(part: &'static str, secs: f64) {
    if !timing_on() {
        return;
    }
    let mut b = BREAKDOWN.lock().unwrap_or_else(|e| e.into_inner());
    match b.parts.iter_mut().find(|(n, _)| *n == part) {
        Some(p) => p.1 += secs,
        None => b.parts.push((part, secs)),
    }
}

fn add_map(items: usize, serial: bool, secs: f64) {
    let mut b = BREAKDOWN.lock().unwrap_or_else(|e| e.into_inner());
    b.maps += 1;
    b.serial_maps += usize::from(serial);
    b.items += items;
    b.map_secs += secs;
}

impl StageTimer {
    pub fn start(name: &'static str, threads: usize) -> StageTimer {
        let on = timing_on();
        if on {
            *BREAKDOWN.lock().unwrap_or_else(|e| e.into_inner()) = Breakdown::default();
        }
        StageTimer { name, threads, started: on.then(std::time::Instant::now) }
    }
}

impl Drop for StageTimer {
    fn drop(&mut self) {
        if let Some(t) = self.started {
            let secs = t.elapsed().as_secs_f64();
            let b = std::mem::take(&mut *BREAKDOWN.lock().unwrap_or_else(|e| e.into_inner()));
            // a `*_count` part is a count, not seconds
            let parts: f64 = b.parts.iter().filter(|p| !p.0.ends_with("_count")).map(|p| p.1).sum();
            let named: String = b.parts.iter().map(|(n, s)| if n.ends_with("_count") { format!(" {n}={s}") } else { format!(" {n}_secs={s:.3}") }).collect();
            eprintln!(
                "sta stage {} threads={} secs={secs:.3} maps={} serial_maps={} items={} map_secs={:.3}{named} other_secs={:.3}",
                self.name, self.threads, b.maps, b.serial_maps, b.items, b.map_secs, secs - b.map_secs - parts
            );
        }
    }
}

/// One state access of a vertex visit, on a cell numbered by the caller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Access {
    /// Computing the visit reads the cell.
    Read(usize),
    /// Applying the visit overwrites the cell.
    Write(usize),
    /// Applying the visit sets the cell to a fixed value (idempotent, so two sets commute).
    Set(usize),
}

/// The first wave a vertex may join given its accesses and the waves of the earlier accesses
/// (`last_*[c]`: 1 + the last wave that read/wrote/set cell `c`, 0 for none).
///
/// - Read after a write or a set: a LATER wave (the value is stored at apply).
/// - Write after a read: the SAME wave may do (all computes of a wave finish before any apply,
///   so the earlier reader still sees the old value); after a write or a set: a later wave.
/// - Set after a read: the same wave; after a write: a later wave. Set after set: any wave.
fn first_wave(accesses: &[Access], last_r: &[usize], last_w: &[usize], last_s: &[usize]) -> usize {
    let mut w = 0;
    for &a in accesses {
        w = match a {
            Access::Read(c) => w.max(last_w[c]).max(last_s[c]),
            Access::Write(c) => w.max(last_w[c]).max(last_s[c]).max(last_r[c].saturating_sub(1)),
            Access::Set(c) => w.max(last_w[c]).max(last_r[c].saturating_sub(1)),
        };
    }
    w
}

/// The bookkeeping [`waves`] keeps over the serial order: per cell, 1 + the last wave that read,
/// wrote or set it.
pub struct WaveState {
    last_r: Vec<usize>,
    last_w: Vec<usize>,
    last_s: Vec<usize>,
}

impl WaveState {
    pub fn new(cells: usize) -> WaveState {
        WaveState { last_r: vec![0; cells], last_w: vec![0; cells], last_s: vec![0; cells] }
    }

    /// The wave of the next vertex in serial order, which makes `accesses`; they are recorded.
    pub fn place(&mut self, accesses: &[Access]) -> usize {
        let w = first_wave(accesses, &self.last_r, &self.last_w, &self.last_s);
        for &a in accesses {
            let (c, last) = match a {
                Access::Read(c) => (c, &mut self.last_r),
                Access::Write(c) => (c, &mut self.last_w),
                Access::Set(c) => (c, &mut self.last_s),
            };
            last[c] = last[c].max(w + 1);
        }
        w
    }
}

/// `order` grouped by wave (`wave[k]`: the wave of `order[k]`): group `w` holds the vertices of
/// wave `w` in `order` order.
pub fn group_by_wave(order: &[usize], wave: &[usize]) -> Vec<Vec<usize>> {
    let mut out: Vec<Vec<usize>> = Vec::new();
    for (k, &v) in order.iter().enumerate() {
        let w = wave[k];
        while out.len() <= w {
            out.push(Vec::new());
        }
        out[w].push(v);
    }
    out
}

/// The waves of `order` (each vertex once): wave `k` holds, in `order` order, the vertices whose
/// earliest legal wave is `k`. `access(v, out)` pushes the cells (below `cells`) the visit of `v`
/// touches.
pub fn waves(order: &[usize], cells: usize, access: impl FnMut(usize, &mut Vec<Access>)) -> Vec<Vec<usize>> {
    let t = timing_on().then(std::time::Instant::now);
    let w = waves_inner(order, cells, access);
    if let Some(t) = t {
        add_time("waves", t.elapsed().as_secs_f64());
        add_time("wave_count", w.len() as f64);
    }
    w
}

fn waves_inner(order: &[usize], cells: usize, mut access: impl FnMut(usize, &mut Vec<Access>)) -> Vec<Vec<usize>> {
    let mut st = WaveState::new(cells);
    let mut wave = Vec::with_capacity(order.len());
    let mut acc = Vec::new();
    for &v in order {
        acc.clear();
        access(v, &mut acc);
        wave.push(st.place(&acc));
    }
    group_by_wave(order, &wave)
}

/// Places each `(index, value)` at its index: the result is in `items` order whichever thread
/// computed what. ⛔ The order of the pairs must not matter (the schedule decides it).
pub fn merge_by_index<T>(n: usize, parts: Vec<Vec<(usize, T)>>) -> Vec<T> {
    let mut slots: Vec<Option<T>> = Vec::with_capacity(n);
    for _ in 0..n {
        slots.push(None);
    }
    for part in parts {
        for (i, t) in part {
            debug_assert!(slots[i].is_none(), "an item computed twice");
            slots[i] = Some(t);
        }
    }
    let mut out = Vec::with_capacity(n);
    for s in slots {
        out.push(s.expect("every item computed"));
    }
    out
}

/// `f` of every item, in `items` order, computed on up to `threads` threads (the calling thread
/// one of them). Workers claim chunks of [`CHUNK`] items in index order. Returns only when every
/// item is computed (the wave barrier). Fewer items than threads: computed here, serially.
pub fn map<T: Send>(items: &[usize], threads: usize, f: impl Fn(usize) -> T + Sync) -> Vec<T> {
    if timing_on() {
        let t = std::time::Instant::now();
        let r = map_inner(items, threads, f);
        add_map(items.len(), threads <= 1 || items.len() < threads, t.elapsed().as_secs_f64());
        return r;
    }
    map_inner(items, threads, f)
}

fn map_inner<T: Send>(items: &[usize], threads: usize, f: impl Fn(usize) -> T + Sync) -> Vec<T> {
    if threads <= 1 || items.len() < threads {
        return items.iter().map(|&v| f(v)).collect();
    }
    let next = AtomicUsize::new(0);
    let work = || {
        let mut part = Vec::new();
        loop {
            let from = next.fetch_add(CHUNK, Ordering::Relaxed);
            if from >= items.len() {
                return part;
            }
            for (i, &v) in items.iter().enumerate().take((from + CHUNK).min(items.len())).skip(from) {
                part.push((i, f(v)));
            }
        }
    };
    let parts = std::thread::scope(|s| {
        let handles: Vec<_> = (1..threads).map(|_| s.spawn(work)).collect();
        let mut parts = vec![work()];
        parts.extend(handles.into_iter().map(|h| h.join().expect("a timing worker panicked")));
        parts
    });
    merge_by_index(items.len(), parts)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Serial semantics of a toy pass: cell values; vertex `v` reads `reads[v]`, writes
    /// `writes[v]` with a value from what it read and its own id, and sets `sets[v]` to 1.
    struct Toy {
        reads: Vec<Vec<usize>>,
        writes: Vec<Vec<usize>>,
        sets: Vec<Vec<usize>>,
    }

    impl Toy {
        fn compute(&self, cells: &[u64], v: usize) -> u64 {
            self.reads[v].iter().fold(v as u64 + 1, |h, &c| h.wrapping_mul(31).wrapping_add(cells[c]))
        }
        fn apply(&self, cells: &mut [u64], v: usize, x: u64) {
            for &c in &self.writes[v] {
                cells[c] = x;
            }
            for &c in &self.sets[v] {
                cells[c] = 1;
            }
        }
        fn access(&self, v: usize, out: &mut Vec<Access>) {
            out.extend(self.reads[v].iter().map(|&c| Access::Read(c)));
            out.extend(self.writes[v].iter().map(|&c| Access::Write(c)));
            out.extend(self.sets[v].iter().map(|&c| Access::Set(c)));
        }
        fn serial(&self, n_cells: usize, order: &[usize]) -> Vec<u64> {
            let mut cells = vec![0u64; n_cells];
            for &v in order {
                let x = self.compute(&cells, v);
                self.apply(&mut cells, v, x);
            }
            cells
        }
        fn parallel(&self, n_cells: usize, order: &[usize], threads: usize) -> Vec<u64> {
            let mut cells = vec![0u64; n_cells];
            for wave in waves(order, n_cells, |v, out| self.access(v, out)) {
                let xs = map(&wave, threads, |v| self.compute(&cells, v));
                for (&v, x) in wave.iter().zip(xs) {
                    self.apply(&mut cells, v, x);
                }
            }
            cells
        }
    }

    /// A small pseudo-random generator, so the cases are fixed.
    fn lcg(s: &mut u64) -> usize {
        *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (*s >> 33) as usize
    }

    fn random_toy(seed: u64, n: usize, cells: usize) -> Toy {
        let mut s = seed;
        let mut pick = |k: usize| (0..lcg(&mut s) % (k + 1)).map(|_| lcg(&mut s) % cells).collect::<Vec<_>>();
        let reads = (0..n).map(|_| pick(3)).collect();
        let writes = (0..n).map(|_| pick(2)).collect();
        let sets = (0..n).map(|_| pick(1)).collect();
        Toy { reads, writes, sets }
    }

    /// 🔑 Rule: cutting the serial order into hazard waves, computing each wave on any number of
    /// threads and applying in wave order gives the serial result bit for bit — every
    /// read-after-write, write-after-write and set/read-write pair keeps its serial order.
    #[test]
    fn waves_on_any_thread_count_equal_the_serial_order() {
        // Fewer cases under Miri, which interprets every step.
        let seeds = if cfg!(miri) { 6 } else { 300 };
        for seed in 0..seeds {
            let (n, cells) = (40 + (seed as usize % 50), 12 + (seed as usize % 20));
            let toy = random_toy(seed, n, cells);
            let order: Vec<usize> = (0..n).collect();
            let want = toy.serial(cells, &order);
            for threads in [1, 2, 4, 16] {
                assert_eq!(toy.parallel(cells, &order, threads), want, "seed {seed} threads {threads}");
            }
        }
    }

    /// Every vertex lands in exactly one wave, and a wave keeps the serial order.
    /// Grouping covers every vertex exactly once, in its own wave, in serial order within it —
    /// checked for EVERY wave assignment of up to five vertices (Kani's model of the nested
    /// vectors outgrows 40 GB even at two, so the bounded check is exhaustive enumeration here).
    #[test]
    fn grouping_covers_every_vertex_once_in_order_exhaustively() {
        for n in 0..=5usize {
            let order: Vec<usize> = (0..n).collect();
            for code in 0..n.pow(n as u32).max(1) {
                let wave: Vec<usize> = (0..n).map(|k| code / n.pow(k as u32) % n).collect();
                let groups = group_by_wave(&order, &wave);
                let mut seen = vec![0usize; n];
                for (k, g) in groups.iter().enumerate() {
                    for (j, &v) in g.iter().enumerate() {
                        assert_eq!(wave[v], k, "vertex {v} in group {k}, wave {wave:?}");
                        seen[v] += 1;
                        assert!(j == 0 || g[j - 1] < v, "group {k} out of serial order: {g:?}");
                    }
                }
                assert!(seen.iter().all(|&c| c == 1), "covered {seen:?} for wave {wave:?}");
            }
        }
    }

    #[test]
    fn waves_partition_the_order() {
        let toy = random_toy(7, 200, 30);
        let order: Vec<usize> = (0..200).rev().collect();
        let ws = waves(&order, 30, |v, out| toy.access(v, out));
        let flat: Vec<usize> = ws.iter().flatten().copied().collect();
        let mut sorted = flat.clone();
        sorted.sort_unstable();
        assert_eq!(sorted, (0..200).collect::<Vec<_>>());
        for w in &ws {
            let pos: Vec<usize> = w.iter().map(|v| order.iter().position(|x| x == v).unwrap()).collect();
            assert!(pos.windows(2).all(|p| p[0] < p[1]));
        }
    }

    /// A chain reading what its predecessor wrote is one vertex per wave; independent vertices
    /// share wave 0; a write may share the wave of an earlier read.
    #[test]
    fn wave_rules() {
        let chain = Toy { reads: vec![vec![], vec![0], vec![1]], writes: vec![vec![0], vec![1], vec![2]], sets: vec![vec![]; 3] };
        assert_eq!(waves(&[0, 1, 2], 3, |v, o| chain.access(v, o)), vec![vec![0], vec![1], vec![2]]);
        let war = Toy { reads: vec![vec![0], vec![]], writes: vec![vec![], vec![0]], sets: vec![vec![]; 2] };
        assert_eq!(waves(&[0, 1], 1, |v, o| war.access(v, o)), vec![vec![0, 1]]);
        let sets = Toy { reads: vec![vec![], vec![], vec![0]], writes: vec![vec![]; 3], sets: vec![vec![0], vec![0], vec![]] };
        assert_eq!(waves(&[0, 1, 2], 1, |v, o| sets.access(v, o)), vec![vec![0, 1], vec![2]]);
    }

    /// `map` returns in item order at every thread count, chunk boundaries included.
    #[test]
    fn map_keeps_item_order() {
        let items: Vec<usize> = (0..1000).map(|i| i * 7 % 1000).collect();
        for threads in [1, 2, 3, 4, 16] {
            assert_eq!(map(&items, threads, |v| v * 2), items.iter().map(|v| v * 2).collect::<Vec<_>>());
        }
    }

    #[test]
    fn thread_count_rule() {
        let max = std::thread::available_parallelism().map_or(1, |n| n.get());
        assert_eq!(resolve_thread_count(0), max);
        assert_eq!(resolve_thread_count(-3), max);
        assert_eq!(resolve_thread_count(1), 1);
        assert_eq!(resolve_thread_count(i32::MAX), max);
    }
}

/// Kani proofs of the sequential core (`cargo kani`): bounded, every input of the size.
#[cfg(kani)]
mod proofs {
    use super::*;

    const N: usize = 3;
    const CELLS: usize = 2;

    fn any_access() -> Access {
        let c: usize = kani::any();
        kani::assume(c < CELLS);
        match kani::any::<u8>() % 3 {
            0 => Access::Read(c),
            1 => Access::Write(c),
            _ => Access::Set(c),
        }
    }

    fn cell(a: Access) -> usize {
        match a {
            Access::Read(c) | Access::Write(c) | Access::Set(c) => c,
        }
    }

    /// Every conflicting pair (one writes, or one sets and the other reads or writes) is placed
    /// in waves in serial order — strictly, except an earlier read before a later write, which
    /// may share the wave (computes finish before stores).
    #[kani::proof]
    #[kani::unwind(4)]
    fn waves_keep_conflicts_in_serial_order() {
        let acc: [[Access; 2]; N] = [[any_access(), any_access()], [any_access(), any_access()], [any_access(), any_access()]];
        let mut st = WaveState::new(CELLS);
        let wave = [st.place(&acc[0]), st.place(&acc[1]), st.place(&acc[2])];
        for i in 0..N {
            for j in (i + 1)..N {
                for a in acc[i] {
                    for b in acc[j] {
                        if cell(a) != cell(b) {
                            continue;
                        }
                        match (a, b) {
                            (Access::Read(_), Access::Read(_)) | (Access::Set(_), Access::Set(_)) => {}
                            (Access::Read(_), _) => assert!(wave[i] <= wave[j]),
                            _ => assert!(wave[i] < wave[j]),
                        }
                    }
                }
            }
        }
    }

    /// The merge places every item at its index whatever order the parts arrive in (two items:
    /// every split into two parts, in either order).
    #[kani::proof]
    #[kani::unwind(4)]
    fn merge_by_index_is_schedule_independent() {
        let split: usize = kani::any();
        kani::assume(split <= 2);
        let swap: bool = kani::any();
        let mut a: Vec<(usize, u8)> = Vec::with_capacity(2);
        let mut b: Vec<(usize, u8)> = Vec::with_capacity(2);
        for i in 0..2 {
            if i < split {
                a.push((i, 10 + i as u8));
            } else {
                b.push((i, 10 + i as u8));
            }
        }
        let parts = if swap { vec![b, a] } else { vec![a, b] };
        let m = merge_by_index(2, parts);
        assert!(m.len() == 2);
        assert!(m[0] == 10 && m[1] == 11);
    }
}
