// SPDX-License-Identifier: Apache-2.0
//! Incremental timing: the timer's state kept across netlist edits, and each update doing only
//! the reference's incremental work — so a value it does not recompute stays as it was.
//!
//! The graph is rebuilt from the netlist after edits; the state lives here, keyed by pin name, and
//! is imported into the new graph before an update and exported after it. Stages, in the order a
//! query runs them (`Sta::findDelays(level)`):
//! - [`IncTimer::relevelize`] — `Levelize::relevelize`: from each vertex an edit relevelized
//!   from, a fanout not above its fanin is raised to the fanin's level + 1 (levels never fall;
//!   the first levels are the full levelization's, spaced by 10); a vertex whose level changes is
//!   invalid (delay, arrival, required);
//! - [`IncTimer::find_delays`] — `GraphDelayCalc::findDelays(level)`: a level-ordered queue from
//!   the invalid vertices, up to the queried level (the rest stays queued). A driver is
//!   recomputed whole; it enqueues a load only if the load's slew changed beyond fuzzy equality.
//!   A load enqueues its fanout. Check edges at visited check pins are recomputed after the queue.
//!
//! The observer (`StaDelayCalcObserver`) turns delay changes into search invalidations: a changed
//! gate delay invalidates the required at the arc's from vertex and at the driver, and the
//! arrival at the driver; every recomputed driver re-merges its wire delays from the initial
//! values, so each of its loads' arrivals is invalid; a recomputed check edge invalidates the
//! required at its data pin.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use crate::graph::{EdgeKind, Graph, NetParasitics};
use crate::liberty::Role;
use crate::search::{CrprPath, Path, Prev, Search, Tag};

/// A gate edge by its ends and its arc set (cell, set index); a wire edge by its ends.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum EdgeKey {
    Gate { from: String, to: String, cell: String, set: usize },
    Wire { from: String, to: String },
}

/// The full levelization's spacing; incremental relevelization steps by 1 between.
pub const LEVEL_SPACE: i32 = 10;

#[derive(Debug, Default, Clone)]
pub struct IncTimer {
    /// Each vertex's level.
    pub levels: HashMap<String, i32>,
    slews: HashMap<String, [[f32; 2]; 2]>,
    delays: HashMap<EdgeKey, Vec<[f32; 2]>>,
    /// Vertices whose delays are invalid (`invalid_delays_`) and the delay queue's pending
    /// vertices (`iter_`), by name.
    pub invalid_delays: BTreeSet<String>,
    pub delay_queue: BTreeSet<String>,
    pub relevelize_from: BTreeSet<String>,
    pub invalid_check_edges: BTreeSet<EdgeKey>,
    /// Search invalidations raised by delay calculation and by edits.
    pub invalid_arrivals: BTreeSet<String>,
    pub invalid_requireds: BTreeSet<String>,
    pub delays_exist: bool,
    pub arrivals_exist: bool,
    pub requireds_exist: bool,
    /// What the last update visited, in order (`VYGDV` lines of the instrumented reference).
    pub visited: Vec<String>,
    /// Each vertex's stored paths, its path array's generation (a new array when its tag group
    /// changes), and each edge's generation (a remade edge is a new edge).
    paths: HashMap<String, Vec<SPath>>,
    path_gen: HashMap<String, u64>,
    pub edge_gen: HashMap<EdgeKey, u64>,
    next_gen: u64,
    /// The arrival and required queues' pending vertices.
    pub arrival_queue: BTreeSet<String>,
    pub required_queue: BTreeSet<String>,
    /// The last arrival / required visits: `(vertex, changed)` (`VYGAV` / `VYGRV`).
    pub arrival_visits: Vec<(String, bool)>,
    pub required_visits: Vec<(String, bool)>,
    /// Each instance's cell and each top port's direction (input: true), kept by the events.
    pub cells: HashMap<String, String>,
    pub port_input: HashMap<String, bool>,
}

/// What applying an edit event reads: the libraries and whether every clock is ideal
/// (`idealClockMode`).
pub struct EventCtx<'a> {
    pub libs: &'a [crate::liberty::Library],
    pub ideal_clock_mode: bool,
}

/// A tag with its CRPR clock vertex by name.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct STag {
    pub rf: usize,
    pub mm: usize,
    pub clk_edge: Option<usize>,
    pub is_clock: bool,
    pub crpr: Option<(String, usize, usize, usize)>,
}

/// Where a stored path came from — what the reference compares by pointer: the fanin vertex's
/// path (vertex, tag, the generation of its path array), the edge (and its generation), the arc.
#[derive(Debug, Clone, PartialEq)]
pub struct SPrev {
    pub vertex: String,
    pub tag: STag,
    pub vertex_gen: u64,
    pub edge: EdgeKey,
    pub edge_gen: u64,
    pub arc: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SPath {
    pub tag: STag,
    pub arrival: f32,
    pub required: f32,
    pub prev: Option<SPrev>,
}

impl IncTimer {
    pub fn edge_key(g: &Graph<'_>, e: usize) -> EdgeKey {
        let ed = &g.edges[e];
        let (from, to) = (g.vertices[ed.from].name.clone(), g.vertices[ed.to].name.clone());
        match ed.kind {
            EdgeKind::Gate { set } => EdgeKey::Gate { from, to, cell: g.vertices[ed.to].cell.clone().unwrap_or_default(), set },
            EdgeKind::Wire => EdgeKey::Wire { from, to },
        }
    }

    /// The stored slews and delays into a freshly built graph; a vertex or edge the state has
    /// never seen starts at 0.
    pub fn import(&self, g: &mut Graph<'_>) {
        for v in 0..g.vertices.len() {
            g.slew[v] = self.slews.get(&g.vertices[v].name).copied().unwrap_or([[0.0; 2]; 2]);
        }
        for e in 0..g.edges.len() {
            if let Some(d) = self.delays.get(&Self::edge_key(g, e)) {
                if d.len() == g.delay[e].len() {
                    g.delay[e] = d.clone();
                }
            }
        }
    }

    /// The graph's slews and delays back into the state.
    pub fn export(&mut self, g: &Graph<'_>) {
        self.slews = (0..g.vertices.len()).map(|v| (g.vertices[v].name.clone(), g.slew[v])).collect();
        self.delays = (0..g.edges.len()).map(|e| (Self::edge_key(g, e), g.delay[e].clone())).collect();
    }

    /// `Levelize::searchThru`: every edge but timing checks, latch D->Q and set/clear arcs.
    fn search_thru(g: &Graph<'_>, e: usize) -> bool {
        match g.edges[e].kind {
            EdgeKind::Gate { set } => !g.is_check(e) && !matches!(g.arc_set(e, set).role, Role::RegSetClr | Role::LatchDtoQ),
            EdgeKind::Wire => true,
        }
    }

    /// The full levelization (`findLevels`, levels spaced by [`LEVEL_SPACE`]) when there are no
    /// levels yet, else `relevelize`. A vertex whose level changes is invalid.
    pub fn relevelize(&mut self, g: &Graph<'_>) -> Result<(), String> {
        if self.levels.is_empty() {
            let l = g.levels()?;
            self.levels = (0..g.vertices.len()).map(|v| (g.vertices[v].name.clone(), l[v] * LEVEL_SPACE)).collect();
            self.relevelize_from.clear();
            return Ok(());
        }
        let index = g.pin_index();
        // A new vertex starts at level 0 (`makeVertex`); a deleted one is forgotten.
        let mut level: Vec<i32> = (0..g.vertices.len()).map(|v| self.levels.get(&g.vertices[v].name).copied().unwrap_or(0)).collect();
        let start = level.clone();
        for name in std::mem::take(&mut self.relevelize_from) {
            let Some(&v) = index.get(&name) else { continue };
            // `visit(vertex, level, 1)`: depth first; a fanout not above `level` is raised to
            // `level + 1` and visited from there. The final levels are the maxima over the paths,
            // whatever the visiting order.
            let mut stack = vec![(v, level[v])];
            while let Some((u, l)) = stack.pop() {
                level[u] = level[u].max(l);
                for &e in &g.out_edges[u] {
                    if !Self::search_thru(g, e) {
                        continue;
                    }
                    let to = g.edges[e].to;
                    if level[to] <= l {
                        level[to] = l + 1;
                        stack.push((to, l + 1));
                    }
                }
            }
        }
        for v in 0..g.vertices.len() {
            if level[v] != start[v] {
                // `levelChangedBefore`: delay, arrival and required invalid.
                let name = &g.vertices[v].name;
                if self.delays_exist {
                    self.invalid_delays.insert(name.clone());
                }
                if self.arrivals_exist {
                    self.invalid_arrivals.insert(name.clone());
                }
                if self.requireds_exist {
                    self.invalid_requireds.insert(name.clone());
                }
            }
        }
        self.levels = (0..g.vertices.len()).map(|v| (g.vertices[v].name.clone(), level[v])).collect();
        Ok(())
    }

    /// `GraphDelayCalc::findDelays(to_level)` (all levels with `None`), after `relevelize`.
    /// The graph must hold the imported state; it holds the updated one after.
    pub fn find_delays(&mut self, g: &mut Graph<'_>, parasitics: &HashMap<String, NetParasitics>, to_level: Option<i32>) -> Result<(), String> {
        self.relevelize(g)?;
        self.visited.clear();
        if !self.delays_exist {
            // Not incremental: every vertex from the roots (`seedRootSlews`).
            g.find_delays(parasitics, None)?;
            self.delays_exist = true;
            self.invalid_delays.clear();
            self.delay_queue.clear();
            self.invalid_check_edges.clear();
            return Ok(());
        }
        let index = g.pin_index();
        // `seedInvalidDelays`.
        self.delay_queue.extend(std::mem::take(&mut self.invalid_delays));
        // The levels, read while the observer writes the rest of the state.
        let levels = std::mem::take(&mut self.levels);
        let level_of = |name: &str| levels.get(name).copied().unwrap_or(0);
        // The queue by level (`BfsIterator`), names kept sorted within a level.
        let mut by_level: BTreeMap<i32, BTreeSet<String>> = BTreeMap::new();
        for n in std::mem::take(&mut self.delay_queue) {
            by_level.entry(level_of(&n)).or_default().insert(n);
        }
        while let Some((&lvl, _)) = by_level.iter().next() {
            if to_level.is_some_and(|t| lvl > t) {
                break;
            }
            let names = by_level.remove(&lvl).expect("a level");
            for name in names {
                let Some(&v) = index.get(&name) else { continue };
                self.visited.push(name.clone());
                let enqueue = |g: &Graph<'_>, to: usize, q: &mut BTreeMap<i32, BTreeSet<String>>| {
                    let n = g_name(g, to);
                    q.entry(level_of(&n)).or_default().insert(n);
                };
                let fanout: Vec<usize> = g.out_edges[v].iter().copied().filter(|&e| !g.is_check(e) && !is_latch_d_to_q(g, e)).map(|e| g.edges[e].to).collect();
                let is_root = level_of(&name) == 0;
                if is_root {
                    // `seedRootSlew`, then every fanout.
                    g.find_vertex_delays(v, parasitics, &index, None)?;
                    for to in fanout {
                        enqueue(g, to, &mut by_level);
                    }
                } else if g.vertices[v].is_driver {
                    let wires: Vec<usize> = g.out_edges[v].iter().copied().filter(|&e| matches!(g.edges[e].kind, EdgeKind::Wire)).collect();
                    let prev_load: Vec<[[f32; 2]; 2]> = wires.iter().map(|&w| g.slew[g.edges[w].to]).collect();
                    let gates: Vec<usize> = g.in_edges[v].iter().copied().filter(|&e| !g.is_check(e) && !is_latch_d_to_q(g, e) && matches!(g.edges[e].kind, EdgeKind::Gate { .. })).collect();
                    let prev_gate: Vec<Vec<[f32; 2]>> = gates.iter().map(|&e| g.delay[e].clone()).collect();
                    g.find_vertex_delays(v, parasitics, &index, None)?;
                    // The observer: a changed gate delay ([`gate_delay_changed`]), every load's
                    // wire delay.
                    let mut changed = false;
                    for (k, &e) in gates.iter().enumerate() {
                        let differs = g.delay[e].iter().zip(&prev_gate[k]).any(|(new, prev)| (0..2).any(|mm| gate_delay_changed(prev[mm], new[mm])));
                        if differs {
                            changed = true;
                            self.required_invalid(g_name(g, g.edges[e].from));
                            self.required_invalid(name.clone());
                        }
                    }
                    if changed {
                        self.arrival_invalid(name.clone());
                    }
                    for (k, &w) in wires.iter().enumerate() {
                        let load = g.edges[w].to;
                        self.arrival_invalid(g_name(g, load));
                        // `loadSlewChanged`: any of the four slews beyond fuzzy equality.
                        let now = g.slew[load];
                        let was = prev_load[k];
                        let differs = (0..2).any(|rf| (0..2).any(|mm| !crate::fuzzy::equal(now[rf][mm], was[rf][mm])));
                        if differs {
                            enqueue(g, load, &mut by_level);
                        }
                    }
                } else {
                    // A load: its slew comes from its driver; checks at it, then every fanout.
                    g.find_vertex_delays(v, parasitics, &index, None)?;
                    self.enqueue_check_edges(g, v);
                    for to in fanout {
                        enqueue(g, to, &mut by_level);
                    }
                }
            }
        }
        // Vertices above the queried level stay queued.
        for (_, names) in by_level {
            self.delay_queue.extend(names);
        }
        self.levels = levels;
        // Timing checks, after the slews they read.
        for key in std::mem::take(&mut self.invalid_check_edges) {
            if let Some(e) = (0..g.edges.len()).find(|&e| Self::edge_key(g, e) == key) {
                g.find_check_edge_delays(e);
                self.required_invalid(g_name(g, g.edges[e].to));
            }
        }
        Ok(())
    }

    /// `enqueueCheckEdges`: the check edges into a data pin and out of a check clock pin.
    fn enqueue_check_edges(&mut self, g: &Graph<'_>, v: usize) {
        for &e in g.in_edges[v].iter().chain(g.out_edges[v].iter()) {
            if g.is_check(e) {
                self.invalid_check_edges.insert(Self::edge_key(g, e));
            }
        }
    }

    pub fn arrival_invalid(&mut self, name: String) {
        if self.arrivals_exist {
            self.invalid_arrivals.insert(name);
        }
    }

    pub fn required_invalid(&mut self, name: String) {
        if self.requireds_exist {
            self.invalid_requireds.insert(name);
        }
    }

    pub fn delay_invalid(&mut self, name: String) {
        if self.delays_exist {
            self.invalid_delays.insert(name);
        }
    }
}

impl IncTimer {
    fn stag(g: &Graph<'_>, t: &Tag) -> STag {
        STag { rf: t.rf, mm: t.mm, clk_edge: t.clk_edge, is_clock: t.is_clock, crpr: t.crpr.map(|c| (g_name(g, c.vertex), c.rf, c.mm, c.clk_edge)) }
    }

    fn tag(index: &HashMap<String, usize>, t: &STag) -> Option<Tag> {
        let crpr = match &t.crpr {
            None => None,
            Some((n, rf, mm, clk_edge)) => Some(CrprPath { vertex: *index.get(n)?, rf: *rf, mm: *mm, clk_edge: *clk_edge }),
        };
        Some(Tag { rf: t.rf, mm: t.mm, clk_edge: t.clk_edge, is_clock: t.is_clock, crpr })
    }

    fn gen_of(&self, name: &str) -> u64 {
        self.path_gen.get(name).copied().unwrap_or(0)
    }

    /// A computed path as the state would store it, its prev linked to the fanin's current array.
    fn spath(&self, g: &Graph<'_>, p: &Path) -> SPath {
        let prev = p.prev.map(|q| {
            let edge = Self::edge_key(g, q.edge);
            let vertex = g_name(g, q.vertex);
            SPrev { vertex_gen: self.gen_of(&vertex), vertex, tag: Self::stag(g, &q.tag), edge_gen: self.edge_gen.get(&edge).copied().unwrap_or(0), edge, arc: q.arc }
        });
        SPath { tag: Self::stag(g, &p.tag), arrival: p.arrival, required: p.required, prev }
    }

    /// The stored paths into a search over a freshly built graph (a prev whose vertex or edge is
    /// gone is dropped).
    pub fn import_paths(&self, search: &mut Search<'_, '_>) {
        let g = search.graph;
        let index = g.pin_index();
        let edges: HashMap<EdgeKey, usize> = (0..g.edges.len()).map(|e| (Self::edge_key(g, e), e)).collect();
        for v in 0..g.vertices.len() {
            let Some(stored) = self.paths.get(&g.vertices[v].name) else { continue };
            search.paths[v] = stored
                .iter()
                .filter_map(|sp| {
                    let tag = Self::tag(&index, &sp.tag)?;
                    let prev = sp.prev.as_ref().and_then(|q| Some(Prev { vertex: *index.get(&q.vertex)?, tag: Self::tag(&index, &q.tag)?, edge: *edges.get(&q.edge)?, arc: q.arc }));
                    Some(Path { tag, arrival: sp.arrival, required: sp.required, prev })
                })
                .collect();
        }
    }

    /// `Search::arrivalsChanged`: a different tag group, or a tag whose arrival differs beyond
    /// fuzzy equality, or whose prev edge, arc or prev path is not the same object.
    fn arrivals_changed(stored: Option<&Vec<SPath>>, new: &[SPath]) -> bool {
        let Some(old) = stored.filter(|o| !o.is_empty()) else { return !new.is_empty() };
        if old.len() != new.len() {
            return true;
        }
        old.iter().any(|p1| match new.iter().find(|p2| p2.tag == p1.tag) {
            None => true,
            Some(p2) => !crate::fuzzy::equal(p1.arrival, p2.arrival) || p1.prev != p2.prev,
        })
    }

    /// `Search::findArrivals(to_level)` (all with `None`) over the imported paths: the queue from
    /// the invalid vertices, level by level; a vertex whose arrivals changed is stored (in place
    /// for the same tag group, else a new array) and its fanout enqueued; an unchanged one keeps
    /// what it had and enqueues nothing.
    pub fn find_arrivals(&mut self, search: &mut Search<'_, '_>, to_level: Option<i32>) -> Result<(), String> {
        let g = search.graph;
        self.arrival_visits.clear();
        if !self.arrivals_exist {
            search.find_arrivals()?;
            self.arrivals_exist = true;
            self.invalid_arrivals.clear();
            self.arrival_queue.clear();
            self.store_all(search);
            return Ok(());
        }
        let index = g.pin_index();
        self.arrival_queue.extend(std::mem::take(&mut self.invalid_arrivals));
        let mut by_level: BTreeMap<i32, BTreeSet<String>> = BTreeMap::new();
        for n in std::mem::take(&mut self.arrival_queue) {
            by_level.entry(self.levels.get(&n).copied().unwrap_or(0)).or_default().insert(n);
        }
        while let Some((&lvl, _)) = by_level.iter().next() {
            if to_level.is_some_and(|t| lvl > t) {
                break;
            }
            for name in by_level.remove(&lvl).expect("a level") {
                let Some(&v) = index.get(&name) else { continue };
                let new: Vec<Path> = search.arrival_paths(v);
                let snew: Vec<SPath> = new.iter().map(|p| self.spath(g, p)).collect();
                let changed = Self::arrivals_changed(self.paths.get(&name), &snew);
                self.arrival_visits.push((name.clone(), changed));
                if !changed {
                    continue;
                }
                for &e in &g.out_edges[v] {
                    if g.is_check(e) || is_latch_d_to_q(g, e) {
                        continue;
                    }
                    let to = g_name(g, g.edges[e].to);
                    by_level.entry(self.levels.get(&to).copied().unwrap_or(0)).or_default().insert(to);
                }
                // `setVertexArrivals`: the same tag group keeps its array, else a new one.
                let same_group = self.paths.get(&name).is_some_and(|o| o.len() == snew.len() && o.iter().all(|p| snew.iter().any(|q| q.tag == p.tag)));
                if !same_group {
                    self.next_gen += 1;
                    self.path_gen.insert(name.clone(), self.next_gen);
                }
                // A new path's required is 0 (`Path::Path`) until the required visit the change
                // invalidates — which compares against that 0.
                let mut stored = snew;
                for p in stored.iter_mut() {
                    p.required = 0.0;
                }
                search.paths[v] = new.into_iter().map(|mut p| {
                    p.required = 0.0;
                    p
                }).collect();
                self.paths.insert(name.clone(), stored);
                self.required_invalid(name.clone());
                // `constrainedRequiredsInvalid`: a clock arrival at a check clock pin changes the
                // required at the data pins it constrains.
                let is_clk = search.paths[v].iter().any(|p| p.tag.is_clock);
                if is_clk && !g.vertices[v].is_driver {
                    for &e in &g.out_edges[v] {
                        if g.is_check(e) {
                            self.required_invalid(g_name(g, g.edges[e].to));
                        }
                    }
                }
            }
        }
        for (_, names) in by_level {
            self.arrival_queue.extend(names);
        }
        Ok(())
    }

    /// `Search::findRequireds(level)` (all with `None`): the queue from the invalid vertices,
    /// highest level first down to `level`; each visited vertex's requireds are stored, and its
    /// fanin enqueued if one changed beyond fuzzy equality.
    pub fn find_requireds(&mut self, search: &mut Search<'_, '_>, down_to_level: Option<i32>) -> Result<(), String> {
        let g = search.graph;
        self.required_visits.clear();
        if !self.requireds_exist {
            search.find_requireds()?;
            self.requireds_exist = true;
            self.invalid_requireds.clear();
            self.required_queue.clear();
            self.store_all(search);
            return Ok(());
        }
        let index = g.pin_index();
        self.required_queue.extend(std::mem::take(&mut self.invalid_requireds));
        let mut by_level: BTreeMap<i32, BTreeSet<String>> = BTreeMap::new();
        for n in std::mem::take(&mut self.required_queue) {
            by_level.entry(self.levels.get(&n).copied().unwrap_or(0)).or_default().insert(n);
        }
        while let Some((&lvl, _)) = by_level.iter().next_back() {
            if down_to_level.is_some_and(|t| lvl < t) {
                break;
            }
            for name in by_level.remove(&lvl).expect("a level") {
                let Some(&v) = index.get(&name) else { continue };
                let req = search.required_values(v);
                let mut changed = false;
                for (p, r) in search.paths[v].iter_mut().zip(req) {
                    if !crate::fuzzy::equal(p.required, r) {
                        changed = true;
                    }
                    p.required = r;
                }
                if let Some(stored) = self.paths.get_mut(&name) {
                    for sp in stored.iter_mut() {
                        if let Some(p) = search.paths[v].iter().find(|p| Self::stag(g, &p.tag) == sp.tag) {
                            sp.required = p.required;
                        }
                    }
                }
                self.required_visits.push((name.clone(), changed));
                if changed {
                    for &e in &g.in_edges[v] {
                        if g.is_check(e) || is_latch_d_to_q(g, e) {
                            continue;
                        }
                        let from = g_name(g, g.edges[e].from);
                        by_level.entry(self.levels.get(&from).copied().unwrap_or(0)).or_default().insert(from);
                    }
                }
            }
        }
        for (_, names) in by_level {
            self.required_queue.extend(names);
        }
        Ok(())
    }

    /// Every vertex's paths into the state (after a full pass).
    fn store_all(&mut self, search: &Search<'_, '_>) {
        let g = search.graph;
        for v in 0..g.vertices.len() {
            let stored: Vec<SPath> = search.paths[v].iter().map(|p| self.spath(g, p)).collect();
            self.paths.insert(g_name(g, v), stored);
        }
    }
}

impl IncTimer {
    /// The instances' cells and the ports' directions as the netlist has them now.
    pub fn track_netlist(&mut self, nl: &crate::netlist::Netlist) {
        self.cells = nl.insts.iter().cloned().collect();
        self.port_input = nl.ports.iter().map(|(n, d)| (n.clone(), *d == crate::netlist::PortDir::Input)).collect();
    }

    fn cell<'l>(&self, cx: &EventCtx<'l>, name: &str) -> Option<&'l crate::liberty::Cell> {
        cx.libs.iter().find_map(|l| l.cells.get(name))
    }

    /// A pin's timing vertex: `(driver, load)`, `None` for a pin the timer has no vertex for.
    fn pin_kind(&self, cx: &EventCtx<'_>, pin: &str) -> Option<(bool, bool)> {
        use crate::liberty::Direction;
        if let Some((inst, port)) = pin.rsplit_once('/') {
            if let Some(cell) = self.cells.get(inst).and_then(|c| self.cell(cx, c)) {
                return match cell.port(port)?.direction {
                    Direction::Input => Some((false, true)),
                    Direction::Output | Direction::Tristate => Some((true, false)),
                    Direction::Bidirect => Some((true, true)),
                    _ => None,
                };
            }
        }
        self.port_input.get(pin).map(|&input| (input, !input))
    }

    fn drivers_loads(&self, cx: &EventCtx<'_>, pins: &str) -> (Vec<String>, Vec<String>) {
        let (mut d, mut l) = (Vec::new(), Vec::new());
        for p in pins.split(',').filter(|p| !p.is_empty()) {
            if let Some((drv, load)) = self.pin_kind(cx, p) {
                if drv {
                    d.push(p.to_string());
                }
                if load {
                    l.push(p.to_string());
                }
            }
        }
        (d, l)
    }

    /// `Search::deleteEdgeBefore` + `GraphDelayCalc` + `Levelize` for a deleted edge: `to`'s
    /// arrival and delay invalid, its paths' prev paths cleared, relevelized from; `from`'s
    /// required invalid.
    fn delete_edge(&mut self, from: &str, to: &str, key: EdgeKey) {
        self.arrival_invalid(to.to_string());
        self.required_invalid(from.to_string());
        if let Some(paths) = self.paths.get_mut(to) {
            for p in paths.iter_mut() {
                p.prev = None;
            }
        }
        self.delay_invalid(to.to_string());
        self.relevelize_from.insert(to.to_string());
        self.bump_edge(key);
    }

    /// A deleted or remade edge: the next one with this key is a different object.
    fn bump_edge(&mut self, key: EdgeKey) {
        self.next_gen += 1;
        self.edge_gen.insert(key, self.next_gen);
    }

    /// `Sta::delaysInvalidFrom(vertex)`.
    pub fn delays_invalid_from(&mut self, v: &str) {
        self.arrival_invalid(v.to_string());
        self.required_invalid(v.to_string());
        self.delay_invalid(v.to_string());
    }

    /// The instance's gate edges into `out`, by key (from its cell's arc sets).
    fn inst_edges(&self, cx: &EventCtx<'_>, inst: &str, cell_name: &str) -> Vec<(String, String, EdgeKey)> {
        let Some(cell) = self.cell(cx, cell_name) else { return Vec::new() };
        cell.arc_sets
            .iter()
            .enumerate()
            .filter(|(_, a)| a.role != Role::Other)
            .map(|(k, a)| {
                let (f, t) = (format!("{inst}/{}", a.from), format!("{inst}/{}", a.to));
                (f.clone(), t.clone(), EdgeKey::Gate { from: f, to: t, cell: cell_name.to_string(), set: k })
            })
            .collect()
    }

    /// Forget a deleted vertex (`deleteVertexBefore`).
    fn delete_vertex(&mut self, v: &str) {
        self.slews.remove(v);
        self.paths.remove(v);
        self.path_gen.remove(v);
        self.levels.remove(v);
        for q in [&mut self.invalid_delays, &mut self.delay_queue, &mut self.invalid_arrivals, &mut self.arrival_queue, &mut self.invalid_requireds, &mut self.required_queue, &mut self.relevelize_from] {
            q.remove(v);
        }
    }

    /// One edit event (`Db::edit_log_take`) as the reference's timer acts on it (`dbStaCbk`).
    pub fn apply_event(&mut self, cx: &EventCtx<'_>, ev: &str) -> Result<(), String> {
        let f: Vec<&str> = ev.split('|').collect();
        match f.as_slice() {
            ["iterm_connect" | "bterm_connect", pin, _net, pins] => self.connect_pin_after(cx, pin, pins),
            ["iterm_disconnect" | "bterm_disconnect", pin, _net, pins] => self.disconnect_pin_before(cx, pin, pins),
            ["iterm_destroy", pin, _net] => self.delete_vertex(pin),
            ["inst_create", inst, master] => {
                // `makeInstanceAfter`: new vertices (level 0) and instance edges.
                self.cells.insert(inst.to_string(), master.to_string());
                if let Some(cell) = self.cell(cx, master) {
                    for p in &cell.ports {
                        self.delete_vertex(&format!("{inst}/{}", p.name));
                    }
                }
                for (_, _, key) in self.inst_edges(cx, inst, master) {
                    self.bump_edge(key);
                }
            }
            ["inst_destroy", inst] => {
                self.cells.remove(*inst);
            }
            ["swap_before", inst, from, to, terms] => self.swap_before(cx, inst, from, to, terms)?,
            ["swap_after", inst, _terms] => {
                let _ = inst;
            }
            ["net_destroy", _net, pins] => {
                // `deleteNetBefore`: each pin still on the net disconnected.
                for p in pins.split(',').filter(|p| !p.is_empty()) {
                    self.disconnect_pin_before(cx, p, pins);
                }
            }
            ["net_create", ..] | ["net_merge", ..] | ["bterm_create", ..] | ["bterm_destroy", ..] => {}
            _ => return Err(format!("edit event not modelled: {ev}")),
        }
        Ok(())
    }

    /// `Sta::connectPinAfter` (+ `connectDrvrPinAfter` / `connectLoadPinAfter`).
    fn connect_pin_after(&mut self, cx: &EventCtx<'_>, pin: &str, pins: &str) {
        let Some((is_drv, is_load)) = self.pin_kind(cx, pin) else { return };
        self.arrival_invalid(pin.to_string());
        self.required_invalid(pin.to_string());
        let (drivers, loads) = self.drivers_loads(cx, pins);
        if is_drv {
            for l in loads.iter().filter(|l| *l != pin) {
                self.bump_edge(EdgeKey::Wire { from: pin.to_string(), to: l.clone() });
                self.arrival_invalid(l.clone());
            }
            self.delay_invalid(pin.to_string());
            self.required_invalid(pin.to_string());
            self.relevelize_from.insert(pin.to_string());
        }
        if is_load {
            for d in drivers.iter().filter(|d| *d != pin) {
                self.bump_edge(EdgeKey::Wire { from: d.clone(), to: pin.to_string() });
                self.delay_invalid(d.clone());
                self.required_invalid(d.clone());
                self.relevelize_from.insert(d.clone());
            }
            self.delay_invalid(pin.to_string());
            self.arrival_invalid(pin.to_string());
        }
    }

    /// `Sta::disconnectPinBefore`: the pin's wire edges deleted.
    fn disconnect_pin_before(&mut self, cx: &EventCtx<'_>, pin: &str, pins: &str) {
        let Some((is_drv, is_load)) = self.pin_kind(cx, pin) else { return };
        let (drivers, loads) = self.drivers_loads(cx, pins);
        if is_drv {
            for l in loads.iter().filter(|l| *l != pin) {
                self.delete_edge(pin, l, EdgeKey::Wire { from: pin.to_string(), to: l.clone() });
            }
        }
        if is_load {
            for d in drivers.iter().filter(|d| *d != pin) {
                self.delete_edge(d, pin, EdgeKey::Wire { from: d.clone(), to: pin.to_string() });
            }
        }
    }

    /// `dbStaCbk::inDbInstSwapMasterBefore`: equivalent arcs → `replaceEquivCellBefore` (inputs
    /// invalidated, arc sets replaced, outputs' delays invalid), else `replaceCellBefore` (inputs
    /// invalidated, the instance's edges deleted; `replaceCellAfter` makes them anew).
    fn swap_before(&mut self, cx: &EventCtx<'_>, inst: &str, from: &str, to: &str, terms: &str) -> Result<(), String> {
        use crate::liberty::Direction;
        let (Some(from_cell), Some(to_cell)) = (self.cell(cx, from), self.cell(cx, to)) else {
            self.cells.insert(inst.to_string(), to.to_string());
            return Ok(());
        };
        let equiv = equiv_cells_arcs(from_cell, to_cell);
        for t in terms.split(';').filter(|t| !t.is_empty()) {
            let mut parts = t.splitn(3, '=');
            let (Some(port), Some(_net), Some(pins)) = (parts.next(), parts.next(), parts.next()) else { continue };
            let Some(fp) = from_cell.port(port) else { continue };
            let pin = format!("{inst}/{port}");
            match fp.direction {
                Direction::Input => {
                    // `replaceCellPinInvalidate`.
                    let caps_equal = to_cell.port(port).is_some_and(|tp| tp.capacitance == fp.capacitance);
                    let to_is_clock = to_cell.port(port).is_some_and(|tp| tp.is_clock);
                    if to_cell.port(port).is_none() || (!caps_equal && !(to_is_clock && cx.ideal_clock_mode)) {
                        let (drivers, _) = self.drivers_loads(cx, pins);
                        for d in drivers.iter().filter(|d| **d != pin) {
                            self.delays_invalid_from(d);
                            self.required_invalid(d.clone());
                        }
                    } else {
                        self.delays_invalid_from(&pin);
                    }
                }
                Direction::Output | Direction::Tristate if equiv => self.delay_invalid(pin.clone()),
                _ => {}
            }
        }
        let old_edges = self.inst_edges(cx, inst, from);
        if equiv {
            // The arc sets are replaced in place: a path's prev arc is a different object.
            for (_, _, key) in old_edges {
                self.bump_edge(key);
            }
        } else {
            for (f, t, key) in old_edges {
                self.delete_edge(&f, &t, key);
            }
            for (_, _, key) in self.inst_edges(cx, inst, to) {
                self.bump_edge(key);
            }
        }
        self.cells.insert(inst.to_string(), to.to_string());
        Ok(())
    }
}

/// `sta::equivCellsArcs`: the same number of arc sets, each matching the other's by index — the
/// same ports, role, condition and arcs.
pub fn equiv_cells_arcs(a: &crate::liberty::Cell, b: &crate::liberty::Cell) -> bool {
    a.arc_sets.len() == b.arc_sets.len()
        && a.arc_sets.iter().zip(&b.arc_sets).all(|(x, y)| {
            x.from == y.from && x.to == y.to && x.role == y.role && x.cond == y.cond && x.arcs.len() == y.arcs.len() && x.arcs.iter().zip(&y.arcs).all(|(p, q)| p.from_rf == q.from_rf && p.to_rf == q.to_rf)
        })
}

/// `GraphDelayCalc::annotateDelaySlew`'s change test, at the tolerance the reference leaves
/// (`incremental_delay_tolerance_` 0): in `float`, a previous delay of 0 has ALWAYS changed (even
/// to 0 again), else `|new − prev| / prev > 0` — so a negative previous delay never has. Not
/// `new != prev`.
pub fn gate_delay_changed(prev: f32, new: f32) -> bool {
    prev == 0.0 || (new - prev).abs() / prev > 0.0
}

fn g_name(g: &Graph<'_>, v: usize) -> String {
    g.vertices[v].name.clone()
}

fn is_latch_d_to_q(g: &Graph<'_>, e: usize) -> bool {
    matches!(g.edges[e].kind, EdgeKind::Gate { set } if g.arc_set(e, set).role == Role::LatchDtoQ)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::liberty::Library;
    use crate::liberty_parse::parse;
    use crate::netlist::{Conn, Net, Netlist, PortDir};

    // A buffer whose output slew grows with its load capacitance.
    const LIB: &str = r#"library (t) {
      time_unit : "1ns"; capacitive_load_unit (1, pf);
      lu_table_template (c) { variable_1 : total_output_net_capacitance; index_1 ("0.0, 1.0"); }
      cell (buf) {
        pin (A) { direction : input; capacitance : 0.001; }
        pin (X) { direction : output; function : "A";
          timing () { related_pin : "A"; timing_sense : positive_unate;
            cell_rise (c) { values ("0.1, 1.1"); } rise_transition (c) { values ("0.05, 1.05"); }
            cell_fall (c) { values ("0.1, 1.1"); } fall_transition (c) { values ("0.05, 1.05"); } } }
      }
      cell (big) {
        pin (A) { direction : input; capacitance : 0.5; }
        pin (X) { direction : output; function : "A";
          timing () { related_pin : "A"; timing_sense : positive_unate;
            cell_rise (c) { values ("0.1, 1.1"); } rise_transition (c) { values ("0.05, 1.05"); }
            cell_fall (c) { values ("0.1, 1.1"); } fall_transition (c) { values ("0.05, 1.05"); } } }
      }
    }"#;

    /// in → b1 → b2 → b3 → out, b2 of `mid`.
    fn chain(mid: &str) -> Netlist {
        let inst = |i: usize, p: &str| Conn::Inst(i, p.to_string());
        Netlist {
            insts: vec![("b1".into(), "buf".into()), ("b2".into(), mid.into()), ("b3".into(), "buf".into())],
            ports: vec![("in".into(), PortDir::Input), ("out".into(), PortDir::Output)],
            nets: vec![
                Net { name: "n0".into(), pins: vec![inst(0, "A"), Conn::Port(0)] },
                Net { name: "n1".into(), pins: vec![inst(0, "X"), inst(1, "A")] },
                Net { name: "n2".into(), pins: vec![inst(1, "X"), inst(2, "A")] },
                Net { name: "n3".into(), pins: vec![inst(2, "X"), Conn::Port(1)] },
            ],
        }
    }

    fn libs() -> Vec<Library> {
        vec![Library::read(&parse(LIB).unwrap()).unwrap()]
    }

    // Rule (findVertexDelay): a recomputed driver whose loads' slews are unchanged enqueues
    // nothing — the incremental pass stops at it.
    #[test]
    fn an_unchanged_driver_stops_the_pass() {
        let libs = libs();
        let nl = chain("buf");
        let mut inc = IncTimer::default();
        let mut g = Graph::build(&libs, &nl).unwrap();
        inc.find_delays(&mut g, &HashMap::new(), None).unwrap();
        inc.export(&g);
        let mut g = Graph::build(&libs, &nl).unwrap();
        inc.import(&mut g);
        inc.invalid_delays.insert("b1/X".into());
        inc.find_delays(&mut g, &HashMap::new(), None).unwrap();
        assert_eq!(inc.visited, ["b1/X"]);
    }

    // Rule (loadSlewChanged): a driver whose load's slew changes enqueues that load, which
    // enqueues its fanout — the change propagates as far as slews keep changing.
    #[test]
    fn a_changed_load_slew_propagates() {
        let libs = libs();
        let mut inc = IncTimer::default();
        let nl = chain("buf");
        let mut g = Graph::build(&libs, &nl).unwrap();
        inc.find_delays(&mut g, &HashMap::new(), None).unwrap();
        inc.export(&g);
        // b2 becomes a cell with a heavier input: b1's load changes.
        let nl2 = chain("big");
        let mut g = Graph::build(&libs, &nl2).unwrap();
        inc.import(&mut g);
        inc.invalid_delays.insert("b1/X".into());
        inc.find_delays(&mut g, &HashMap::new(), None).unwrap();
        assert_eq!(inc.visited[..3], ["b1/X".to_string(), "b2/A".into(), "b2/X".into()]);
        // b2's output slew is unchanged (its load is the same), so b3 is not revisited.
        assert!(!inc.visited.iter().any(|v| v.starts_with("b3")), "{:?}", inc.visited);
    }

    // Rule (annotateDelaySlew, tolerance 0): the relative test, not inequality — and the two
    // differ on a zero or negative previous delay.
    #[test]
    fn a_gate_delay_change_is_the_relative_test() {
        assert!(gate_delay_changed(0.0, 0.0), "a previous 0 has always changed — `!=` says no");
        assert!(gate_delay_changed(-0.0, 0.0));
        assert!(!gate_delay_changed(-1e-12, -2e-12), "a negative previous never has — `!=` says yes");
        assert!(!gate_delay_changed(1e-10, 1e-10));
        assert!(gate_delay_changed(1e-10, f32::from_bits(1e-10f32.to_bits() + 1)), "one ulp is a change");
    }

    // Rule (BfsIterator::visitParallel(level)): a level-limited pass leaves the higher levels
    // queued for the next pass.
    #[test]
    fn a_level_limited_pass_keeps_the_rest_queued() {
        let libs = libs();
        let mut inc = IncTimer::default();
        let nl = chain("buf");
        let mut g = Graph::build(&libs, &nl).unwrap();
        inc.find_delays(&mut g, &HashMap::new(), None).unwrap();
        inc.export(&g);
        let nl2 = chain("big");
        let mut g = Graph::build(&libs, &nl2).unwrap();
        inc.import(&mut g);
        inc.invalid_delays.insert("b1/X".into());
        let b1 = inc.levels["b1/X"];
        inc.find_delays(&mut g, &HashMap::new(), Some(b1)).unwrap();
        assert_eq!(inc.visited, ["b1/X"]);
        assert!(inc.delay_queue.contains("b2/A"));
        inc.find_delays(&mut g, &HashMap::new(), None).unwrap();
        assert_eq!(inc.visited[..2], ["b2/A".to_string(), "b2/X".into()]);
    }
}
