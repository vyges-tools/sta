// SPDX-License-Identifier: Apache-2.0
//! Loop breaking as the reference's levelization does it (`Levelize::ensureRoots`,
//! `findBackEdges`, `findCycleBackEdges`, `recordLoop`): a depth-first search from the roots that
//! disables every edge closing a loop, so the graph the timer walks is acyclic.
//!
//! The module sees the graph only through [`LoopGraph`] — vertices in the reference's
//! `VertexIterator` order (vertex id), each vertex's out-edges in `VertexOutEdgeIterator` order
//! (newest first), the edge's ends and `Levelize::searchThru` without its disabled-loop term — and
//! answers [`LoopBreaking`]: the disabled edges in the order the search found them. So it can be
//! hosted by any timing graph that keeps the reference's ids and edge order.
//!
//! The functions below are the reference's, in its order, under its names; [`break_loops`] is
//! nothing but their call sequence.

use std::cmp::Ordering;
use std::collections::BTreeSet;

/// The graph as levelization reads it.
pub trait LoopGraph {
    /// Vertex ids are `0..vertex_count()`, in `VertexIterator` order. Since OpenSTA b395bb0 the
    /// iterator walks `Network::pinIterator()` (the top instance's pins, then each leaf's), the
    /// order a fresh build gives the ids, ports first (`Graph::build`). After an edit the
    /// reference walks the network while new vertices take new ids; this trait reads ids.
    fn vertex_count(&self) -> usize;
    /// Edge ids are `0..edge_count()` (a deleted edge's id may be one no vertex lists).
    fn edge_count(&self) -> usize;
    /// The `k`-th out-edge of `v` in `VertexOutEdgeIterator` order (the newest edge first).
    fn out_edge(&self, v: usize, k: usize) -> Option<usize>;
    /// The edges into `v` (any order: only whether one is searched through is asked).
    fn in_edges(&self, v: usize) -> &[usize];
    fn edge_from(&self, e: usize) -> usize;
    fn edge_to(&self, e: usize) -> usize;
    /// `Levelize::searchThru` WITHOUT `isDisabledLoop` (this module adds that term): not a timing
    /// check, not latch D->Q, not a disabled preset/clear arc, not a disabled bidirect inst path.
    fn search_thru(&self, e: usize) -> bool;
    /// `bidirectDrvrSlewFromLoad(pin) && !isBidirectDriver()`: a bidirect top port's load vertex,
    /// which levelization treats as having fanout to its driver. False while bidirect instance
    /// paths are off (the default).
    fn bidirect_load_fanout(&self, v: usize) -> bool;
    /// `VertexNameLess` (`Network::pathNameLess` of the two pins).
    fn name_cmp(&self, a: usize, b: usize) -> Ordering;
    /// The edge as the trace names it (`VYGES_STA_LOOP_TRACE`); its id by default.
    fn edge_label(&self, e: usize) -> String {
        (e + 1).to_string()
    }
}

/// What loop breaking decided.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LoopBreaking {
    /// `disabled_loop_edges_`, in the order `recordLoop` disabled them.
    pub disabled: Vec<usize>,
    /// `loop_edges_`: every edge of every recorded loop (`loopEdges`), the closing edge among them.
    /// Deleting one of these makes the reference levelize from scratch (`deleteEdgeBefore`).
    pub loop_edges: BTreeSet<usize>,
    /// `loops_`: each loop's edges, the closing edge last.
    pub loops: Vec<Vec<usize>>,
    /// `roots_` after loop breaking (`VertexSet`, id order): the roots, and the to-vertex of every
    /// back edge of a root-less cycle.
    pub roots: BTreeSet<usize>,
    /// An edge of `loop_edges` was deleted since: the reference's next levelization breaks the
    /// loops again from scratch (`Levelize::deleteEdgeBefore`).
    pub stale: bool,
}

struct State {
    visited: Vec<bool>,
    on_path: Vec<bool>,
    disabled: Vec<bool>,
    out: LoopBreaking,
    trace: bool,
}

impl State {
    /// `Levelize::searchThru`, its disabled-loop term included.
    fn search_thru(&self, g: &dyn LoopGraph, e: usize) -> bool {
        !self.disabled[e] && g.search_thru(e)
    }
}

/// `Levelize::ensureBackEdges` from a cleared levelization: `ensureRoots`, then `findBackEdges`.
pub fn break_loops(g: &dyn LoopGraph) -> LoopBreaking {
    let trace = std::env::var_os("VYGES_STA_LOOP_TRACE").is_some();
    let mut st = State { visited: vec![false; g.vertex_count()], on_path: vec![false; g.vertex_count()], disabled: vec![false; g.edge_count()], out: LoopBreaking::default(), trace };
    ensure_roots(g, &mut st);
    find_back_edges(g, &mut st);
    st.out
}

/// `Levelize::ensureRoots`: every vertex, in id order, that [`is_root`] — inserted into the
/// id-ordered `roots_`.
fn ensure_roots(g: &dyn LoopGraph, st: &mut State) {
    for v in 0..g.vertex_count() {
        if is_root(g, st, v) {
            st.out.roots.insert(v);
        }
    }
}

/// `Levelize::isRoot`: no searched-through edge enters the vertex.
fn is_root(g: &dyn LoopGraph, st: &State, v: usize) -> bool {
    !g.in_edges(v).iter().any(|&e| st.search_thru(g, e))
}

/// `Levelize::hasFanout`: a searched-through out-edge, or a bidirect top port's load.
fn has_fanout(g: &dyn LoopGraph, st: &State, v: usize) -> bool {
    let mut k = 0;
    while let Some(e) = g.out_edge(v, k) {
        if st.search_thru(g, e) {
            return true;
        }
        k += 1;
    }
    g.bidirect_load_fanout(v)
}

/// `Levelize::findBackEdges()`: visited / on-path cleared on every vertex; every root with
/// fanout (sorted, [`sorted_roots_with_fanout`]) marked visited and on path and pushed, in that
/// order, onto ONE stack — so the LAST root is searched first; then the root-less cycles.
fn find_back_edges(g: &dyn LoopGraph, st: &mut State) {
    st.visited.iter_mut().for_each(|x| *x = false);
    st.on_path.iter_mut().for_each(|x| *x = false);
    let roots = sorted_roots_with_fanout(g, st);
    if st.trace {
        for (i, &v) in roots.iter().enumerate() {
            eprintln!("VYGL|SR|{i}|{}", v + 1);
        }
    }
    let mut path = Vec::new();
    let mut stack = Vec::with_capacity(roots.len());
    for &v in &roots {
        st.visited[v] = true;
        st.on_path[v] = true;
        stack.push((v, 0usize));
    }
    find_back_edges_dfs(g, st, &mut path, &mut stack);
    find_cycle_back_edges(g, st);
}

/// `Levelize::sortedRootsWithFanout`: `roots_` (id order) with fanout, sorted by name
/// (`VertexNameLess`) only when there are fewer than 100 — the reference skips the sort on a
/// large design, so there the order is the id order.
fn sorted_roots_with_fanout(g: &dyn LoopGraph, st: &State) -> Vec<usize> {
    let mut roots: Vec<usize> = st.out.roots.iter().copied().filter(|&v| has_fanout(g, st, v)).collect();
    if roots.len() < 100 {
        roots.sort_by(|&a, &b| g.name_cmp(a, b));
    }
    roots
}

/// `Levelize::findBackEdges(path, stack)`: the iterative depth-first search. The top vertex's
/// next out-edge, if searched through: an unvisited to-vertex is marked visited and on path,
/// the edge pushed on the path and the vertex on the stack; a to-vertex on the path closes a
/// loop ([`record_loop`]); a visited one off the path is passed. An exhausted vertex is popped,
/// taken off the path, and the path loses its last edge (if any). Returns the back edges.
fn find_back_edges_dfs(g: &dyn LoopGraph, st: &mut State, path: &mut Vec<usize>, stack: &mut Vec<(usize, usize)>) -> Vec<usize> {
    let mut back_edges = Vec::new();
    while let Some(&(v, k)) = stack.last() {
        match g.out_edge(v, k) {
            Some(e) => {
                stack.last_mut().expect("top").1 = k + 1;
                let to = g.edge_to(e);
                if st.trace {
                    let branch = if !st.search_thru(g, e) { "nothru" } else if !st.visited[to] { "push" } else if st.on_path[to] { "back" } else { "seen" };
                    eprintln!("VYGL|E|{}|{}|{}|{branch}", v + 1, to + 1, e + 1);
                }
                if st.search_thru(g, e) {
                    if !st.visited[to] {
                        st.visited[to] = true;
                        st.on_path[to] = true;
                        path.push(e);
                        stack.push((to, 0));
                    } else if st.on_path[to] {
                        record_loop(g, st, e, path);
                        back_edges.push(e);
                    }
                }
            }
            None => {
                stack.pop();
                if st.trace {
                    eprintln!("VYGL|P|{}", v + 1);
                }
                st.on_path[v] = false;
                path.pop();
            }
        }
    }
    back_edges
}

/// `Levelize::findCycleBackEdges`: the cycles no root reaches. The vertices left unvisited
/// ([`find_unvisited_vertices`], name-sorted below 100); from each one not yet a START (only
/// starts are remembered — a vertex an earlier start's search visited starts a search of its
/// own all the same), a search with the vertex on the stack but NOT marked visited or on path —
/// so a cycle through it is walked until it closes at the vertex the second time round. Each
/// back edge's to-vertex becomes a root.
fn find_cycle_back_edges(g: &dyn LoopGraph, st: &mut State) {
    let mut unvisited = find_unvisited_vertices(st);
    if unvisited.len() < 100 {
        unvisited.sort_by(|&a, &b| g.name_cmp(a, b));
    }
    if st.trace {
        eprintln!("VYGL|U|{}", unvisited.len());
        for &v in &unvisited {
            eprintln!("VYGL|UV|{}", v + 1);
        }
    }
    let mut started = BTreeSet::new();
    for v in unvisited {
        if started.insert(v) {
            let mut path = Vec::new();
            let mut stack = vec![(v, 0usize)];
            if st.trace {
                eprintln!("VYGL|C|{}|{}", v + 1, u8::from(st.visited[v]));
            }
            for e in find_back_edges_dfs(g, st, &mut path, &mut stack) {
                st.out.roots.insert(g.edge_to(e));
            }
        }
    }
}

/// `Levelize::findUnvisitedVertices`: every vertex no search from a root visited, in id order.
fn find_unvisited_vertices(st: &State) -> Vec<usize> {
    st.visited.iter().enumerate().filter(|&(_, &v)| !v).map(|(i, _)| i).collect()
}

/// `Levelize::recordLoop`: the loop (`loopEdges`: the path from the edge leaving the loop's
/// head pin, then the closing edge) kept, and the closing edge disabled.
fn record_loop(g: &dyn LoopGraph, st: &mut State, e: usize, path: &[usize]) {
    let edges = loop_edges(g, path, e);
    st.out.loop_edges.extend(edges.iter().copied());
    st.out.loops.push(edges);
    st.out.disabled.push(e);
    st.disabled[e] = true;
    if st.trace {
        eprintln!("VYGL|L|{}|{}|{}|{}", e + 1, g.edge_from(e) + 1, g.edge_to(e) + 1, g.edge_label(e));
    }
}

/// `Levelize::loopEdges`: the path's edges from the first one leaving the closing edge's
/// to-vertex on, then the closing edge.
fn loop_edges(g: &dyn LoopGraph, path: &[usize], closing: usize) -> Vec<usize> {
    let head = g.edge_to(closing);
    let mut edges: Vec<usize> = match path.iter().position(|&e| g.edge_from(e) == head) {
        Some(i) => path[i..].to_vec(),
        None => Vec::new(),
    };
    edges.push(closing);
    edges
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A graph given as the reference prints it: vertex names by id (from 1), edges `(from, to)`
    /// by id (from 1). Each vertex's out-edges are kept in creation (id) order and iterated
    /// newest first, as `VertexOutEdgeIterator` walks the list `makeEdge` prepends to.
    struct Mock {
        names: Vec<String>,
        port: Vec<bool>,
        edges: Vec<(usize, usize)>,
        out: Vec<Vec<usize>>,
        inn: Vec<Vec<usize>>,
        off: Vec<usize>,
    }

    impl Mock {
        /// `ports`: the ids (from 1) of the top-level ports.
        fn new(names: &[&str], ports: &[usize], edges: &[(usize, usize)]) -> Mock {
            let n = names.len();
            let mut out = vec![Vec::new(); n];
            let mut inn = vec![Vec::new(); n];
            let edges: Vec<(usize, usize)> = edges.iter().map(|&(f, t)| (f - 1, t - 1)).collect();
            for (e, &(f, t)) in edges.iter().enumerate() {
                out[f].push(e);
                inn[t].push(e);
            }
            let port = (1..=n).map(|v| ports.contains(&v)).collect();
            Mock { names: names.iter().map(|s| s.to_string()).collect(), port, edges, out, inn, off: Vec::new() }
        }
    }

    impl LoopGraph for Mock {
        fn vertex_count(&self) -> usize {
            self.names.len()
        }
        fn edge_count(&self) -> usize {
            self.edges.len()
        }
        fn out_edge(&self, v: usize, k: usize) -> Option<usize> {
            let o = &self.out[v];
            (k < o.len()).then(|| o[o.len() - 1 - k])
        }
        fn in_edges(&self, v: usize) -> &[usize] {
            &self.inn[v]
        }
        fn edge_from(&self, e: usize) -> usize {
            self.edges[e].0
        }
        fn edge_to(&self, e: usize) -> usize {
            self.edges[e].1
        }
        fn search_thru(&self, e: usize) -> bool {
            !self.off.contains(&e)
        }
        fn bidirect_load_fanout(&self, _v: usize) -> bool {
            false
        }
        /// A top port before any instance pin, then by name (`pathNameLess` over a flat design).
        fn name_cmp(&self, a: usize, b: usize) -> Ordering {
            (!self.port[a], &self.names[a]).cmp(&(!self.port[b], &self.names[b]))
        }
    }

    /// Ids from 1, as the reference prints them.
    fn ids(v: &[usize]) -> Vec<usize> {
        v.iter().map(|x| x + 1).collect()
    }

    /// The reference's own run (instrumented levelization at the pin) on a two-input design with
    /// one loop the inputs reach (u1 <-> u2) and two root-less rings, the second fed from the
    /// first (r1 <-> r2 -> r3 <-> r4): it disabled edges 8 (u2/A -> u2/X), 1 (r1/A -> r1/Y) and 4
    /// (r3/B -> r3/X), in that order, and made r1/Y and r3/X roots. Vertex and edge ids are the
    /// reference's.
    fn reference_design() -> Mock {
        let names = ["r1/A", "r1/Y", "r2/A", "r2/Y", "r3/A", "r3/B", "r3/X", "r4/A", "r4/Y", "u1/A", "u1/B", "u1/X", "u2/A", "u2/B", "u2/X", "in1", "in2", "out1", "out2"];
        let edges = [
            (1, 2), (3, 4), (5, 7), (6, 7), (8, 9), (10, 12), (11, 12), (13, 15), (14, 15), (2, 5),
            (2, 3), (4, 1), (7, 8), (9, 6), (9, 19), (12, 13), (15, 11), (15, 18), (16, 10), (17, 14),
        ];
        Mock::new(&names, &[16, 17, 18, 19], &edges)
    }

    #[test]
    fn the_reference_runs_disables_the_same_edges_in_the_same_order() {
        let m = reference_design();
        let lb = break_loops(&m);
        assert_eq!(ids(&lb.disabled), [8, 1, 4]);
        // roots_: the two inputs, and the to-vertices of the root-less cycles' back edges (r1/Y,
        // r3/X); the outputs are entered, so are not roots.
        assert_eq!(ids(&lb.roots.iter().copied().collect::<Vec<_>>()), [2, 7, 16, 17]);
    }

    /// `findBackEdges()` pushes EVERY sorted root onto one stack before searching, so the LAST
    /// root is searched first: in2 (searched first) reaches u2/X through u2/B, goes round
    /// through u1, and closes the loop at u2/A -> u2/X. Searching in1 first would have closed
    /// it at u1/B -> u1/X (edge 7).
    #[test]
    fn the_last_sorted_root_is_searched_first() {
        let m = reference_design();
        assert_eq!(ids(&break_loops(&m).disabled)[0], 8);
        let mut first_in1 = reference_design();
        // in1 named after in2: now in1 is the last root, searched first
        first_in1.names[15] = "in3".into();
        assert_eq!(ids(&break_loops(&first_in1).disabled)[0], 7);
    }

    /// `findCycleBackEdges` starts each root-less cycle's search WITHOUT marking the start
    /// visited or on the path: from r1/A the search goes round r1 -> r2 -> r1/A (visiting r1/A
    /// again) and the loop closes on r1/A's own out-edge, r1/A -> r1/Y — not on r2/Y -> r1/A,
    /// where a search that marked its start would close it.
    #[test]
    fn a_rootless_cycle_closes_at_its_start_the_second_time_round() {
        let m = reference_design();
        let lb = break_loops(&m);
        assert_eq!(ids(&lb.disabled)[1], 1);
        // the loop (`loopEdges`): the path from the first edge leaving the closing edge's
        // to-vertex r1/Y — r1/Y -> r2/A, r2/A -> r2/Y, r2/Y -> r1/A — then the closing edge
        assert_eq!(ids(&lb.loops[1]), [11, 2, 12, 1]);
        // its back edge's to-vertex, r1/Y, is a root
        assert!(lb.roots.contains(&1));
    }

    /// `VertexOutEdgeIterator` walks newest first: parallel edges (a cell's conditional arcs from
    /// one pin) each close the loop, and are disabled newest first.
    #[test]
    fn parallel_loop_edges_are_disabled_newest_first() {
        // r -> y, y -> b, then four arcs b -> y
        let m = Mock::new(&["r", "y", "b"], &[1], &[(1, 2), (2, 3), (3, 2), (3, 2), (3, 2), (3, 2)]);
        assert_eq!(ids(&break_loops(&m).disabled), [6, 5, 4, 3]);
    }

    /// `sortedRootsWithFanout` (and the unvisited list) are name-sorted only below 100 entries;
    /// at 100 or more they stay in id order.
    #[test]
    fn roots_are_name_sorted_below_one_hundred_only() {
        for n in [99usize, 100] {
            // n roots named in reverse id order, each driving its own sink
            let mut names: Vec<String> = (0..n).map(|i| format!("p{:03}", n - i)).collect();
            names.extend((0..n).map(|i| format!("s{i:03}")));
            let refs: Vec<&str> = names.iter().map(|s| s.as_str()).collect();
            let edges: Vec<(usize, usize)> = (1..=n).map(|i| (i, n + i)).collect();
            let ports: Vec<usize> = (1..=n).collect();
            let m = Mock::new(&refs, &ports, &edges);
            let st = State { visited: vec![false; 2 * n], on_path: vec![false; 2 * n], disabled: vec![false; n], out: LoopBreaking { roots: (0..n).collect(), ..Default::default() }, trace: false };
            let roots = sorted_roots_with_fanout(&m, &st);
            let expect: Vec<usize> = if n < 100 { (0..n).rev().collect() } else { (0..n).collect() };
            assert_eq!(roots, expect, "{n} roots");
        }
    }

    /// `isRoot`: no edge levelization searches through enters it — an edge it does not search
    /// (a timing check, latch D -> Q, ...) leaves the vertex a root.
    #[test]
    fn a_vertex_entered_only_by_unsearched_edges_is_a_root() {
        let mut m = Mock::new(&["a", "b"], &[], &[(1, 2)]);
        assert_eq!(ids(&break_loops(&m).roots.iter().copied().collect::<Vec<_>>()), [1]);
        m.off.push(0);
        assert_eq!(ids(&break_loops(&m).roots.iter().copied().collect::<Vec<_>>()), [1, 2]);
    }
}
