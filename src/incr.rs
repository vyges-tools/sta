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
use crate::sdc::States;
use crate::search::{CrprPath, Path, Prev, Search, Tag};

/// A gate edge by its ends and its arc set (cell, set index); a wire edge by its ends.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum EdgeKey {
    Gate { from: String, to: String, cell: String, set: usize },
    Wire { from: String, to: String },
}

/// An edge's delays: gate arcs `[arc][min/max]`, wire edges `[rf][min/max]`.
type EdgeDelays = Vec<[f32; 2]>;

/// A name the timer's state is kept by (a pin's, an instance's, a cell's): held once in a
/// [`NamePool`] and shared by every map that keys by it.
pub type Name = std::sync::Arc<str>;

/// The names a timer's maps share. The reference's state hangs off the graph's objects, so a pin
/// is named once, by the network; ours is kept by name, and a `String` per map per scene made the
/// names most of the timer's memory on a design with long hierarchical names (each pin's name in
/// the levels, the slews, the delays, each edge's from-pin, per scene). The scenes' timers of one
/// design share one pool ([`IncTimer::share_names`]). Names are never let go: a deleted pin's
/// stays, as a repair deletes few.
#[derive(Debug, Default, Clone)]
pub struct NamePool(std::sync::Arc<std::sync::Mutex<std::collections::HashSet<Name>>>);

impl NamePool {
    /// `s`, held once: the pool's copy when it has one, else a new one it keeps.
    pub fn intern(&self, s: &str) -> Name {
        // a panic elsewhere while the lock was held leaves the set itself whole (an insert or a
        // lookup), so it is used as it is
        let mut pool = self.0.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
        if let Some(n) = pool.get(s) {
            return n.clone();
        }
        let n: Name = Name::from(s);
        pool.insert(n.clone());
        n
    }

    /// The same pool (one set of names), not a copy of it.
    pub fn same_as(&self, other: &NamePool) -> bool {
        std::sync::Arc::ptr_eq(&self.0, &other.0)
    }
}

/// An [`EdgeKey`] without its to-vertex.
#[derive(Debug, Clone, PartialEq, Eq)]
enum EdgeTail {
    /// `cell`: the to-vertex's cell name, shared with the graph's vertices.
    Gate { from: Name, cell: std::sync::Arc<str>, set: usize },
    Wire { from: Name },
}

impl EdgeTail {
    fn of(g: &Graph<'_>, e: usize, names: &NamePool) -> EdgeTail {
        let ed = &g.edges[e];
        let from = g.with_vname(ed.from, |n| names.intern(n));
        match ed.kind {
            EdgeKind::Gate { set } => EdgeTail::Gate { from, cell: g.vertices[ed.to].cell.clone().unwrap_or_default(), set },
            EdgeKind::Wire => EdgeTail::Wire { from },
        }
    }

    /// Is this edge `e` of `g` (whose to-vertex was matched already)?
    fn is(&self, g: &Graph<'_>, e: usize) -> bool {
        let ed = &g.edges[e];
        match (self, &ed.kind) {
            (EdgeTail::Gate { from, cell, set }, EdgeKind::Gate { set: s }) => set == s && g.vname_is(ed.from, from) && &**cell == g.vertices[ed.to].cell.as_deref().unwrap_or(""),
            (EdgeTail::Wire { from }, EdgeKind::Wire) => g.vname_is(ed.from, from),
            _ => false,
        }
    }
}

/// The full levelization's spacing; incremental relevelization steps by 1 between.
pub const LEVEL_SPACE: i32 = 10;

/// Which state a graph imported: a new one per timer, per copy of one and per [`IncTimer::export`]
/// — a copy goes its own way, so a graph that imported the original imports the copy whole.
#[derive(Debug)]
struct Stamp(u64);

impl Stamp {
    fn new() -> Stamp {
        static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
        Stamp(NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed))
    }
}

impl Default for Stamp {
    fn default() -> Stamp {
        Stamp::new()
    }
}

impl Clone for Stamp {
    fn clone(&self) -> Stamp {
        Stamp::new()
    }
}

#[derive(Debug, Default, Clone)]
pub struct IncTimer {
    /// The names every map below keys by ([`NamePool`]).
    names: NamePool,
    /// Each vertex's level.
    pub levels: HashMap<Name, i32>,
    slews: HashMap<Name, [[f32; 2]; 2]>,
    /// The vertices whose slew or in-edge delays were stored since the last [`IncTimer::export`],
    /// in order, under `stamp`: what a graph that imported before takes ([`IncTimer::import`]).
    written: Vec<Name>,
    /// The place of `written[0]`: places stay absolute when the head is let go
    /// ([`IncTimer::trim_written`]), so a graph's `state_seq` keeps meaning the same names.
    written_base: usize,
    stamp: Stamp,
    /// Each edge's delays by its [`EdgeKey`], held as to-vertex → the rest of the key, so an edge of
    /// a graph is found without making its key (a graph is imported whole after every edit).
    delays: HashMap<Name, Vec<(EdgeTail, EdgeDelays)>>,
    /// Vertices whose delays are invalid (`invalid_delays_`) and the delay queue's pending
    /// vertices (`iter_`), by name.
    pub invalid_delays: BTreeSet<String>,
    pub delay_queue: BTreeSet<String>,
    pub relevelize_from: BTreeSet<String>,
    /// Vertices made since (`makeVertex`: level 0) that `levels` may not hold yet: what the
    /// relevelize that visits only the raised vertices adds at 0.
    made: BTreeSet<String>,
    pub invalid_check_edges: BTreeSet<EdgeKey>,
    /// Latch D -> Q edges to re-time (`invalid_latch_edges_`): their D was visited.
    pub invalid_latch_edges: BTreeSet<EdgeKey>,
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
    /// `pending_arrivals_`: latch outputs postponed to the next full arrival pass.
    pub pending_arrivals: BTreeSet<String>,
    pub required_queue: BTreeSet<String>,
    /// The last arrival / required visits: `(vertex, changed)` (`VYGAV` / `VYGRV`).
    pub arrival_visits: Vec<(String, bool)>,
    pub required_visits: Vec<(String, bool)>,
    /// A diagnostic (`VYGES_STA_ARRIVAL_VALUES` set): each visited vertex's new paths beside the
    /// stored arrival of their tag, `vertex|rf|mm|clk|old|new` (float bits, hex; `-` when the tag
    /// is new) — the instrumented reference's `VYGAA` lines.
    pub arrival_values: Vec<String>,
    /// Each instance's cell and each top port's direction (input: true), kept by the events.
    pub cells: HashMap<Name, Name>,
    pub port_input: HashMap<String, bool>,
    /// Each net's drivers as the last connect or disconnect on it listed them: a load moved
    /// between nets (a fanout repair moves every load of a net) reads them here, not from a pass
    /// over every pin on the net (that was O(fanout) per event, O(fanout²) per repair). Dropped
    /// for a net when a driver of it is connected or disconnected, and whole on any other event.
    net_drivers: HashMap<String, Vec<String>>,
    /// This timer prints the graph-edit trace (`VYGES_STA_GRAPH_TRACE`): one timer per design
    /// does — the reference has ONE graph for every scene, a caller one timer per scene.
    pub trace_edits: bool,
    /// A scene timer that FOLLOWS the first scene's ([`IncTimer::follow`]): the reference has one
    /// graph, so one set of levels, cells and net drivers for every scene; a follower keeps only its
    /// scene's values (slews, delays, paths) and invalid sets, takes its events through
    /// [`apply_event_scenes`] from the leader's answers, and never relevelizes (every scene pass
    /// reads the first timer's levels).
    follower: bool,
    /// While the leader takes an event for its followers: each value-side effect, in order.
    effects: Option<Vec<Effect>>,
    /// The graph that HOLDS this timer's slews and delays ([`IncTimer::hold_in`]), by its stamp:
    /// the reference's one graph, whose arrays are the delay calculator's state. The name-keyed
    /// maps (`slews`, `delays`, `written`) are then empty and not written; only that graph is
    /// imported (a no-op), until [`IncTimer::release`] puts the values back in the maps.
    held_by: Option<u64>,
}

/// One value-side effect of an edit event on a scene timer, as the leader made it
/// ([`apply_event_scenes`]): what a follower repeats on its own state.
#[derive(Debug, Clone, PartialEq)]
enum Effect {
    Arrival(String),
    Required(String),
    Delay(String),
    ClearPrev(String),
    Bump(EdgeKey),
    DeleteVertex(String),
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
    pub states: States,
    pub latency: u32,
    pub segment_start: bool,
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
    /// Keep `other`'s names ([`NamePool`]): the timers of one design's scenes name the same pins.
    /// Before any state is kept (the maps' names would be another pool's otherwise; nothing
    /// depends on which pool a name came from, only memory does).
    pub fn share_names(&mut self, other: &IncTimer) {
        self.names = other.names.clone();
    }

    /// This timer follows `leader`, the first scene's ([`IncTimer::follower`]): its names, and none
    /// of the structure the leader keeps for every scene.
    pub fn follow(&mut self, leader: &IncTimer) {
        self.share_names(leader);
        self.follower = true;
        self.levels = HashMap::new();
        self.cells = HashMap::new();
        self.port_input = HashMap::new();
        self.net_drivers = HashMap::new();
        self.made.clear();
        self.relevelize_from.clear();
        self.trace_edits = false;
    }

    /// The leader's effects of one event repeated on this follower's values.
    fn repeat(&mut self, fx: &[Effect]) {
        for e in fx {
            match e {
                Effect::Arrival(n) => self.arrival_invalid(n.clone()),
                Effect::Required(n) => self.required_invalid(n.clone()),
                Effect::Delay(n) => self.delay_invalid(n.clone()),
                Effect::ClearPrev(n) => self.clear_prev(n),
                Effect::Bump(k) => self.bump_edge(k.clone()),
                Effect::DeleteVertex(n) => self.delete_vertex(n),
            }
        }
    }

    /// `s` as this timer's maps hold it.
    pub fn intern(&self, s: &str) -> Name {
        self.names.intern(s)
    }

    /// Vertex `v`'s name as this timer's maps hold it.
    fn name(&self, g: &Graph<'_>, v: usize) -> Name {
        g.with_vname(v, |n| self.names.intern(n))
    }

    pub fn edge_key(g: &Graph<'_>, e: usize) -> EdgeKey {
        let ed = &g.edges[e];
        let (from, to) = (g.vname(ed.from), g.vname(ed.to));
        match ed.kind {
            EdgeKind::Gate { set } => EdgeKey::Gate { from, to, cell: g.vertices[ed.to].cell.as_deref().unwrap_or_default().to_string(), set },
            EdgeKind::Wire => EdgeKey::Wire { from, to },
        }
    }

    /// The edge of `g` whose [`IncTimer::edge_key`] is `key` (the lowest id when two are: as a
    /// scan of every edge finds it), found among the in-edges of the vertices named as its head
    /// (`index`: `g`'s pin index). A key costs its head's fan-in, not the graph: after a repair
    /// the invalid check edges are every sink's, and a scan per key never ended on a large design.
    pub fn edge_of_key(g: &Graph<'_>, index: &HashMap<String, usize>, key: &EdgeKey) -> Option<usize> {
        let to = match key {
            EdgeKey::Gate { to, .. } | EdgeKey::Wire { to, .. } => to,
        };
        let &v = index.get(to)?;
        let mut found: Option<usize> = None;
        for u in g.vertices_named(v) {
            for &e in &g.in_edges[u] {
                if found.is_none_or(|f| e < f) && Self::edge_key(g, e) == *key {
                    found = Some(e);
                }
            }
        }
        found
    }

    /// The stored slews and delays into a graph; a vertex or edge the state has never seen
    /// starts at 0. A graph that imported this state before (and edited in place since: what it
    /// made new is 0, as here) takes only the vertices written after; one built, whole.
    /// The names written since the last export and still kept (a diagnostic counter).
    pub fn written_len(&self) -> usize {
        self.written.len()
    }

    /// The place a graph imported this state at (`None`: never, or another state's).
    pub fn written_place(&self, g: &Graph<'_>) -> Option<usize> {
        g.state_seq.filter(|&(stamp, _)| stamp == self.stamp.0).map(|(_, n)| n)
    }

    /// Lets go of the names before place `keep_from`: every graph that may still import this state
    /// stands at or past it. The reference keeps no such log (one graph: a value written is a
    /// value read); this keeps ours from growing with every edit of the repair. A graph behind
    /// the trim imports whole, which is the same values (it is never given a partial list).
    pub fn trim_written(&mut self, keep_from: usize) {
        let keep_from = keep_from.min(self.written_base + self.written.len());
        if keep_from > self.written_base {
            self.written.drain(..keep_from - self.written_base);
            self.written_base = keep_from;
        }
    }

    /// Graph `g` holds this timer's slews and delays from now on: the maps are let go. `g` must
    /// have imported the state as it stands (or be the graph that holds it already).
    pub fn hold_in(&mut self, g: &mut Graph<'_>) -> Result<(), String> {
        if self.holds(g) {
            return Ok(());
        }
        if self.held_by.is_some() {
            return Err("the timer's values are held by another graph".into());
        }
        if g.state_seq != Some((self.stamp.0, self.written_base + self.written.len())) {
            return Err("a graph that has not imported the timer's state cannot hold it".into());
        }
        self.slews = HashMap::new();
        self.delays = HashMap::new();
        self.written = Vec::new();
        self.written_base = 0;
        self.stamp = Stamp::new();
        self.held_by = Some(self.stamp.0);
        g.state_seq = Some((self.stamp.0, 0));
        Ok(())
    }

    /// Graph `g`, just timed whole, holds this timer's slews and delays from now on: the state
    /// [`IncTimer::export`], [`IncTimer::import`] and [`IncTimer::hold_in`] leave, without writing
    /// every value out by name and reading it back. The reference keeps one graph, timed once
    /// (`findDelays`), and edits it; a copy of its values by name is ours alone.
    pub fn hold_timed(&mut self, g: &mut Graph<'_>) -> Result<(), String> {
        if self.held_by.is_some() {
            return Err("the timer's values are held by another graph".into());
        }
        self.slews = HashMap::new();
        self.delays = HashMap::new();
        self.written = Vec::new();
        self.written_base = 0;
        self.stamp = Stamp::new();
        self.held_by = Some(self.stamp.0);
        g.state_seq = Some((self.stamp.0, 0));
        Ok(())
    }

    /// Does `g` hold this timer's values ([`IncTimer::hold_in`])?
    pub fn holds(&self, g: &Graph<'_>) -> bool {
        self.held_by.is_some() && g.state_seq.map(|(s, _)| s) == self.held_by
    }

    /// Is a graph holding this timer's values?
    pub fn is_held(&self) -> bool {
        self.held_by.is_some()
    }

    /// The values back from the graph that holds them into the maps, by name (as
    /// [`IncTimer::export`]): before that graph is let go, or another graph is to import them.
    /// A graph an edit left unusable is read under the netlist its vertices name
    /// ([`Graph::name_failed`]); its deleted edges (no delay rows) are passed over.
    pub fn release(&mut self, g: &mut Graph<'_>) -> Result<(), String> {
        if !self.holds(g) {
            return Err("release: this graph does not hold the timer's values".into());
        }
        self.held_by = None;
        self.export(g);
        g.state_seq = Some((self.stamp.0, 0));
        Ok(())
    }

    pub fn import(&self, g: &mut Graph<'_>) {
        if self.held_by.is_some() {
            // the graph holding the values has them; any other must wait for a release
            assert!(self.holds(g), "import: the timer's values are held by another graph (release it first)");
            return;
        }
        match g.state_seq {
            // one name, two vertices (a bidirect pin): the index holds one of them, so only a
            // bidirect top-level port's pair (both taken) keeps the import to what was written
            Some((stamp, n)) if stamp == self.stamp.0 && n >= self.written_base && n <= self.written_base + self.written.len() && (g.index().len() == g.vertices.len() || g.names_shared_only_by_bidirect_ports()) => {
                let index = g.pin_index();
                let mut seen = std::collections::HashSet::new();
                for name in &self.written[n - self.written_base..] {
                    if seen.insert(&**name) {
                        if let Some(&v) = index.get(&**name) {
                            for u in g.vertices_named(v) {
                                self.import_vertex(g, u);
                            }
                        }
                    }
                }
            }
            _ => {
                for v in 0..g.vertices.len() {
                    g.slew[v] = g.with_vname(v, |n| self.slews.get(n).copied()).unwrap_or([[0.0; 2]; 2]);
                }
                for e in 0..g.edges.len() {
                    self.import_delay(g, e);
                }
            }
        }
        g.state_seq = Some((self.stamp.0, self.written_base + self.written.len()));
    }

    /// Vertex `v`'s slew and its in-edges' delays.
    fn import_vertex(&self, g: &mut Graph<'_>, v: usize) {
        g.slew[v] = g.with_vname(v, |n| self.slews.get(n).copied()).unwrap_or([[0.0; 2]; 2]);
        for i in 0..g.in_edges[v].len() {
            self.import_delay(g, g.in_edges[v][i]);
        }
    }

    fn import_delay(&self, g: &mut Graph<'_>, e: usize) {
        if let Some(d) = self.delay_of(g, e) {
            if d.len() == g.delay[e].len() {
                g.delay[e] = d.clone();
            }
        }
    }

    /// The stored delays of edge `e` of `g`.
    fn delay_of(&self, g: &Graph<'_>, e: usize) -> Option<&Vec<[f32; 2]>> {
        g.with_vname(g.edges[e].to, |to| self.delays.get(to))?.iter().find(|(t, _)| t.is(g, e)).map(|(_, d)| d)
    }

    /// Store `g`'s delays of edge `e`, replacing what its key held.
    fn store_delay(&mut self, g: &Graph<'_>, e: usize) {
        if self.held_by.is_some() {
            return;
        }
        let d = g.delay[e].clone();
        let names = &self.names;
        let delays = &mut self.delays;
        match g.with_vname(g.edges[e].to, move |to| delays.get_mut(to)) {
            Some(bucket) => match bucket.iter_mut().find(|(t, _)| t.is(g, e)) {
                Some(slot) => slot.1 = d,
                None => bucket.push((EdgeTail::of(g, e, names), d)),
            },
            None => {
                let tail = EdgeTail::of(g, e, names);
                self.delays.insert(self.name(g, g.edges[e].to), vec![(tail, d)]);
            }
        }
    }

    /// The graph's slews and delays back into the state.
    pub fn export(&mut self, g: &Graph<'_>) {
        if self.held_by.is_some() {
            assert!(self.holds(g), "export: the timer's values are held by another graph");
            return;
        }
        self.slews = (0..g.vertices.len()).map(|v| (self.name(g, v), g.slew[v])).collect();
        self.delays.clear();
        self.written.clear();
        self.written_base = 0;
        self.stamp = Stamp::new();
        // a deleted edge (an unusable graph's free id) has no rows, and no key of its own
        for e in (0..g.edges.len()).filter(|&e| !g.delay[e].is_empty()) {
            self.store_delay(g, e);
        }
    }

    /// What re-timing vertex `v` can change, back into the state: its slews, its in- and out-edges'
    /// delays, and the slews at its fanout (a driver sets its loads'). Keeps the state current
    /// after a visit without exporting the whole graph.
    pub fn export_around(&mut self, g: &Graph<'_>, v: usize) {
        if self.held_by.is_some() {
            return;
        }
        let name = self.name(g, v);
        self.slews.insert(name.clone(), g.slew[v]);
        self.written.push(name);
        for &e in g.in_edges[v].iter().chain(g.out_edges[v].iter()) {
            self.store_delay(g, e);
        }
        for &e in &g.out_edges[v] {
            let to = g.edges[e].to;
            let name = self.name(g, to);
            self.slews.insert(name.clone(), g.slew[to]);
            self.written.push(name);
        }
    }

    /// `Levelize::searchThru`: every edge but timing checks, latch D->Q and set/clear arcs.
    /// The disabled loop edges are passed over too: the visit would go round a loop for ever.
    fn search_thru(g: &Graph<'_>, e: usize) -> bool {
        !g.is_disabled_loop(e)
            && match g.edges[e].kind {
                EdgeKind::Gate { set } => !g.is_check(e) && !matches!(g.arc_set(e, set).role, Role::RegSetClr | Role::LatchDtoQ),
                EdgeKind::Wire => true,
            }
    }

    /// The full levelization (`findLevels`, levels spaced by [`LEVEL_SPACE`]) when there are no
    /// levels yet, else `relevelize`. A vertex whose level changes is invalid.
    pub fn relevelize(&mut self, g: &Graph<'_>) -> Result<(), String> {
        if self.follower {
            return Ok(());
        }
        if self.levels.is_empty() {
            let l = g.levels()?;
            self.levels = (0..g.vertices.len()).map(|v| (self.name(g, v), l[v] * LEVEL_SPACE)).collect();
            self.relevelize_from.clear();
            self.made.clear();
            return Ok(());
        }
        // Nothing to relevelize from: every level stays (a new vertex reads 0 either way, as an
        // absent one does), so nothing is invalid — and the whole map need not be rebuilt.
        if self.relevelize_from.is_empty() {
            return Ok(());
        }
        let index = g.pin_index();
        // One vertex per name: only the vertices the visits raise change, so only they are read
        // and written. `VYGES_STA_CHECK_LEVELS=1`: against the whole pass, every call.
        if index.len() == g.vertices.len() || g.names_shared_only_by_bidirect_ports() {
            let full = std::env::var_os("VYGES_STA_CHECK_LEVELS").is_some().then(|| {
                let mut t = self.clone();
                t.relevelize_all(g, &index);
                t
            });
            self.relevelize_raised(g, &index);
            if let Some(t) = full {
                if t.levels != self.levels || t.invalid_delays != self.invalid_delays || t.invalid_arrivals != self.invalid_arrivals || t.invalid_requireds != self.invalid_requireds {
                    return Err("relevelize: the raised vertices' pass differs from the whole pass".to_string());
                }
            }
            return Ok(());
        }
        self.relevelize_all(g, &index);
        Ok(())
    }

    /// [`IncTimer::relevelize`] reading and writing only the vertices the visits raise; the map
    /// otherwise holds every vertex already (the new ones added at 0, the deleted ones forgotten
    /// by their events).
    fn relevelize_raised(&mut self, g: &Graph<'_>, index: &HashMap<String, usize>) {
        let levels = &self.levels;
        let at = |raised: &HashMap<usize, i32>, u: usize| raised.get(&u).copied().unwrap_or_else(|| g.with_vname(u, |n| levels.get(n).copied()).unwrap_or(0));
        let mut raised: HashMap<usize, i32> = HashMap::new();
        for name in std::mem::take(&mut self.relevelize_from) {
            let Some(&v) = index.get(&name) else { continue };
            // As the whole pass: a vertex is pushed at the level it was raised to, and levels only
            // rise, so it is never below its push. (A bidirect port's name: both its vertices.)
            let mut stack: Vec<(usize, i32)> = g.vertices_named(v).map(|u| (u, at(&raised, u))).collect();
            while let Some((u, l)) = stack.pop() {
                for &e in &g.out_edges[u] {
                    if !Self::search_thru(g, e) {
                        continue;
                    }
                    let to = g.edges[e].to;
                    if at(&raised, to) <= l {
                        raised.insert(to, l + 1);
                        stack.push((to, l + 1));
                    }
                }
            }
        }
        // Every raised vertex rose: `levelChangedBefore`.
        for &v in raised.keys() {
            let name = &g.vname(v);
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
        for (v, l) in raised {
            let name = self.name(g, v);
            self.levels.insert(name, l);
        }
        for name in std::mem::take(&mut self.made) {
            if index.contains_key(&name) && !self.levels.contains_key(name.as_str()) {
                let name = self.names.intern(&name);
                self.levels.insert(name, 0);
            }
        }
    }

    /// [`IncTimer::relevelize`] over every vertex.
    fn relevelize_all(&mut self, g: &Graph<'_>, index: &HashMap<String, usize>) {
        self.made.clear();
        // A new vertex starts at level 0 (`makeVertex`); a deleted one is forgotten.
        let mut level: Vec<i32> = (0..g.vertices.len()).map(|v| g.with_vname(v, |n| self.levels.get(n).copied()).unwrap_or(0)).collect();
        let start = level.clone();
        for name in std::mem::take(&mut self.relevelize_from) {
            let Some(&v) = index.get(&name) else { continue };
            // `visit(vertex, level, 1)`: depth first; a fanout not above `level` is raised to
            // `level + 1` and visited from there. The final levels are the maxima over the paths,
            // whatever the visiting order. (A bidirect port's name: both its vertices.)
            let mut stack: Vec<(usize, i32)> = g.vertices_named(v).map(|u| (u, level[u])).collect();
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
                let name = &g.vname(v);
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
        // The map every vertex's level makes, updated in place rather than made again: each
        // vertex's level (a new one inserted, a later one of a repeated name last), then any name no
        // longer in the graph dropped.
        for (v, &l) in level.iter().enumerate() {
            let levels = &mut self.levels;
            if g.with_vname(v, |n| levels.get_mut(n).map(|x| *x = l)).is_none() {
                let name = self.name(g, v);
                self.levels.insert(name, l);
            }
        }
        if self.levels.len() != g.vertices.len() {
            self.levels.retain(|k, _| index.contains_key(&**k));
        }
    }

    /// `GraphDelayCalc::findDelays(to_level)` (all levels with `None`), after `relevelize`.
    /// The graph must hold the imported state; it holds the updated one after.
    pub fn find_delays(&mut self, g: &mut Graph<'_>, parasitics: &HashMap<String, NetParasitics>, to_level: Option<i32>) -> Result<(), String> {
        find_delays_scenes(std::slice::from_mut(self), std::slice::from_mut(g), &[parasitics], to_level)
    }

    /// `enqueueCheckEdges`: the check edges into a data pin and out of a check clock pin.
    fn enqueue_check_edges(&mut self, g: &Graph<'_>, v: usize) {
        for &e in g.in_edges[v].iter().chain(g.out_edges[v].iter()) {
            if g.is_check(e) {
                self.invalid_check_edges.insert(Self::edge_key(g, e));
            }
        }
        // `isLatchData`: its D -> Q edges are re-timed after the pass (levelization does not
        // traverse them).
        for &e in &g.out_edges[v] {
            if is_latch_d_to_q(g, e) {
                self.invalid_latch_edges.insert(Self::edge_key(g, e));
            }
        }
    }

    pub fn arrival_invalid(&mut self, name: String) {
        if let Some(fx) = &mut self.effects {
            fx.push(Effect::Arrival(name.clone()));
        }
        if self.arrivals_exist {
            self.invalid_arrivals.insert(name);
        }
    }

    pub fn required_invalid(&mut self, name: String) {
        if let Some(fx) = &mut self.effects {
            fx.push(Effect::Required(name.clone()));
        }
        if self.requireds_exist {
            self.invalid_requireds.insert(name);
        }
    }

    pub fn delay_invalid(&mut self, name: String) {
        if let Some(fx) = &mut self.effects {
            fx.push(Effect::Delay(name.clone()));
        }
        if self.delays_exist {
            self.invalid_delays.insert(name);
        }
    }
}

impl IncTimer {
    fn stag(g: &Graph<'_>, t: &Tag) -> STag {
        STag { rf: t.rf, mm: t.mm, clk_edge: t.clk_edge, is_clock: t.is_clock, crpr: t.crpr.map(|c| (g_name(g, c.vertex), c.rf, c.mm, c.clk_edge)), states: t.states, latency: t.latency, segment_start: t.segment_start }
    }

    fn tag(index: &HashMap<String, usize>, t: &STag) -> Option<Tag> {
        let crpr = match &t.crpr {
            None => None,
            Some((n, rf, mm, clk_edge)) => Some(CrprPath { vertex: *index.get(n)?, rf: *rf, mm: *mm, clk_edge: *clk_edge }),
        };
        Some(Tag { rf: t.rf, mm: t.mm, clk_edge: t.clk_edge, is_clock: t.is_clock, crpr, states: t.states, latency: t.latency, segment_start: t.segment_start })
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
        for (v, paths) in search.paths.iter_mut().enumerate().take(g.vertices.len()) {
            let Some(stored) = self.paths.get(&g.vname(v)) else { continue };
            *paths = stored
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

    /// `Search::findArrivals(to_level)` (all with `None`) over the imported paths: see
    /// [`find_arrivals_scenes`].
    pub fn find_arrivals(&mut self, search: &mut Search<'_, '_>, to_level: Option<i32>) -> Result<(), String> {
        find_arrivals_scenes(std::slice::from_mut(self), std::slice::from_mut(search), to_level)
    }

    /// `Search::findRequireds(level)` (all with `None`): see [`find_requireds_scenes`].
    pub fn find_requireds(&mut self, search: &mut Search<'_, '_>, down_to_level: Option<i32>) -> Result<(), String> {
        find_requireds_scenes(std::slice::from_mut(self), std::slice::from_mut(search), down_to_level)
    }

    /// Every vertex's paths into the state (after a full pass).
    fn store_all(&mut self, search: &Search<'_, '_>) {
        let g = search.graph;
        // Index loop kept as written: the index is used beyond this container (order and bounds unchanged).
        #[allow(clippy::needless_range_loop)]
        for v in 0..g.vertices.len() {
            let stored: Vec<SPath> = search.paths[v].iter().map(|p| self.spath(g, p)).collect();
            self.paths.insert(g_name(g, v), stored);
        }
    }
}

/// The queue's levels: each name at its level (scene 0's levels; every scene's are alike — one
/// netlist, one levelization).
fn queue_by_level(levels: &HashMap<Name, i32>, names: impl IntoIterator<Item = String>) -> BTreeMap<i32, BTreeSet<String>> {
    let mut by_level: BTreeMap<i32, BTreeSet<String>> = BTreeMap::new();
    for n in names {
        by_level.entry(levels.get(n.as_str()).copied().unwrap_or(0)).or_default().insert(n);
    }
    by_level
}

/// `GraphDelayCalc::findDelays(to_level)` over every scene at once, after `relevelize`: ONE
/// level-ordered queue, as the reference's one graph holds every scene's delays and slews. A
/// visited vertex is recomputed in every scene. A driver enqueues a load if the load's slew
/// changed beyond fuzzy equality in ANY scene (`loadSlewChanged` over `slewCount()`, every
/// analysis point); the observer's invalidations are raised in each scene whose delay changed —
/// the union is the reference's per-vertex OR. A load enqueues its fanout. Check edges at
/// visited check pins are recomputed after the queue. Each graph must hold its imported state;
/// it holds the updated one after.
pub fn find_delays_scenes(incs: &mut [IncTimer], gs: &mut [Graph<'_>], parasitics: &[&HashMap<String, NetParasitics>], to_level: Option<i32>) -> Result<(), String> {
    for (inc, g) in incs.iter_mut().zip(gs.iter()) {
        inc.relevelize(g)?;
        inc.visited.clear();
    }
    if !incs[0].delays_exist {
        // Not incremental: every vertex from the roots (`seedRootSlews`).
        for ((inc, g), par) in incs.iter_mut().zip(gs.iter_mut()).zip(parasitics) {
            g.find_delays(par, None)?;
            inc.delays_exist = true;
            inc.invalid_delays.clear();
            inc.delay_queue.clear();
            inc.invalid_check_edges.clear();
            inc.invalid_latch_edges.clear();
        }
        return Ok(());
    }
    let indexes: Vec<std::sync::Arc<HashMap<String, usize>>> = gs.iter().map(|g| g.pin_index()).collect();
    // `seedInvalidDelays`: every scene's invalid vertices (alike: one netlist's edits).
    let mut seed = BTreeSet::new();
    for inc in incs.iter_mut() {
        seed.extend(std::mem::take(&mut inc.invalid_delays));
        seed.extend(std::mem::take(&mut inc.delay_queue));
    }
    // The levels, read while the observer writes the rest of the state.
    let levels = std::mem::take(&mut incs[0].levels);
    let level_of = |name: &str| levels.get(name).copied().unwrap_or(0);
    // The queue by level (`BfsIterator`), names kept sorted within a level.
    let mut by_level = queue_by_level(&levels, seed);
    let enqueue = |n: String, q: &mut BTreeMap<i32, BTreeSet<String>>| {
        q.entry(level_of(&n)).or_default().insert(n);
    };
    while let Some((&lvl, _)) = by_level.iter().next() {
        if to_level.is_some_and(|t| lvl > t) {
            break;
        }
        let names = by_level.remove(&lvl).expect("a level");
        for name in names {
            let fanout_names = visit_vertex(incs, gs, &indexes, parasitics, &name, level_of(&name) == 0)?;
            for n in fanout_names {
                enqueue(n, &mut by_level);
            }
        }
    }
    // Vertices above the queried level stay queued.
    let rest: BTreeSet<String> = by_level.into_values().flatten().collect();
    for inc in incs.iter_mut() {
        inc.delay_queue.extend(rest.iter().cloned());
    }
    incs[0].levels = levels;
    // Timing checks, after the slews they read.
    for (inc, g) in incs.iter_mut().zip(gs.iter_mut()) {
        let index = g.pin_index();
        for key in std::mem::take(&mut inc.invalid_check_edges) {
            if let Some(e) = IncTimer::edge_of_key(g, &index, &key) {
                g.find_check_edge_delays(e);
                inc.store_delay(g, e);
                inc.required_invalid(g_name(g, g.edges[e].to));
            }
        }
    }
    // Latch D -> Q edges, after the checks (`invalid_latch_edges_`); a changed delay invalidates
    // the Q's arrivals (`delayChangedTo`).
    for ((inc, g), par) in incs.iter_mut().zip(gs.iter_mut()).zip(parasitics) {
        let index = g.pin_index();
        for key in std::mem::take(&mut inc.invalid_latch_edges) {
            if let Some(e) = IncTimer::edge_of_key(g, &index, &key) {
                let changed = g.find_latch_edge_delays(e, par, &index);
                inc.store_delay(g, e);
                if changed {
                    inc.arrival_invalid(g_name(g, g.edges[e].to));
                }
            }
        }
    }
    Ok(())
}

/// One vertex re-timed in every scene, as the queue visits it (`FindVertexDelays::visit`): a root
/// seeds its slews and enqueues every fanout; a driver is recomputed whole and enqueues each load
/// whose slew changed beyond fuzzy equality (`loadSlewChanged`); a load is re-timed, enqueues its
/// checks, then every fanout. Returns the fanouts to enqueue.
fn visit_vertex(incs: &mut [IncTimer], gs: &mut [Graph<'_>], indexes: &[std::sync::Arc<HashMap<String, usize>>], parasitics: &[&HashMap<String, NetParasitics>], name: &str, is_root: bool) -> Result<BTreeSet<String>, String> {
    let mut fanout_names: BTreeSet<String> = BTreeSet::new();
    for k in 0..gs.len() {
        let Some(&v) = indexes[k].get(name) else { continue };
        let (inc, g) = (&mut incs[k], &mut gs[k]);
        inc.visited.push(name.to_string());
        let fanout: Vec<usize> = g.out_edges[v].iter().copied().filter(|&e| !g.is_check(e) && !is_latch_d_to_q(g, e)).map(|e| g.edges[e].to).collect();
        if is_root {
            // `seedRootSlew`, then every fanout.
            g.find_vertex_delays(v, parasitics[k], &indexes[k], None)?;
            inc.export_around(g, v);
            fanout_names.extend(fanout.iter().map(|&to| g_name(g, to)));
        } else if g.vertices[v].is_driver {
            let wires: Vec<usize> = g.out_edges[v].iter().copied().filter(|&e| matches!(g.edges[e].kind, EdgeKind::Wire)).collect();
            let prev_load: Vec<[[f32; 2]; 2]> = wires.iter().map(|&w| g.slew[g.edges[w].to]).collect();
            let gates: Vec<usize> = g.in_edges[v].iter().copied().filter(|&e| !g.is_check(e) && !is_latch_d_to_q(g, e) && matches!(g.edges[e].kind, EdgeKind::Gate { .. })).collect();
            let prev_gate: Vec<Vec<[f32; 2]>> = gates.iter().map(|&e| g.delay[e].clone()).collect();
            g.find_vertex_delays(v, parasitics[k], &indexes[k], None)?;
            inc.export_around(g, v);
            // The observer: a changed gate delay ([`gate_delay_changed`]), every load's
            // wire delay.
            let mut changed = false;
            for (j, &e) in gates.iter().enumerate() {
                let differs = g.delay[e].iter().zip(&prev_gate[j]).any(|(new, prev)| (0..2).any(|mm| gate_delay_changed(prev[mm], new[mm])));
                if differs {
                    changed = true;
                    inc.required_invalid(g_name(g, g.edges[e].from));
                    inc.required_invalid(name.to_string());
                }
            }
            if changed {
                inc.arrival_invalid(name.to_string());
            }
            for (j, &w) in wires.iter().enumerate() {
                let load = g.edges[w].to;
                inc.arrival_invalid(g_name(g, load));
                // `loadSlewChanged`: any of the slews beyond fuzzy equality — in this
                // scene; a load any scene enqueues is visited in every scene.
                let now = g.slew[load];
                let was = prev_load[j];
                let differs = (0..2).any(|rf| (0..2).any(|mm| !crate::fuzzy::equal(now[rf][mm], was[rf][mm])));
                if differs {
                    fanout_names.insert(g_name(g, load));
                }
            }
            // `visitFanouts` also reaches a GATE fanout of the driver (an output-to-output
            // arc, a full adder's CON -> SN). `loadSlewChanged` looks its pin up in the load
            // index map, where `operator[]` inserts it at index 0: its slews are compared
            // with the previous slews of the net's FIRST load (in the reference's out-edge
            // order, which follows edit history) — or, with no loads, it is enqueued.
            for &e in g.out_edges[v].iter().filter(|&&e| matches!(g.edges[e].kind, EdgeKind::Gate { .. }) && !g.is_check(e) && !is_latch_d_to_q(g, e)) {
                let to = g.edges[e].to;
                let now = g.slew[to];
                let verdicts: BTreeSet<bool> = prev_load.iter().map(|was| (0..2).any(|rf| (0..2).any(|mm| !crate::fuzzy::equal(now[rf][mm], was[rf][mm])))).collect();
                match verdicts.len() {
                    0 => {
                        fanout_names.insert(g_name(g, to));
                    }
                    1 => {
                        if verdicts.contains(&true) {
                            fanout_names.insert(g_name(g, to));
                        }
                    }
                    _ => return Err(format!("{}: whether gate fanout {} is re-timed depends on the first load's previous slew (edit-history order): not modelled", name, g_name(g, to))),
                }
            }
        } else {
            // A load: its slew comes from its driver; checks at it, then every fanout.
            g.find_vertex_delays(v, parasitics[k], &indexes[k], None)?;
            inc.export_around(g, v);
            inc.enqueue_check_edges(g, v);
            fanout_names.extend(fanout.iter().map(|&to| g_name(g, to)));
        }
    }
    Ok(fanout_names)
}

/// `GraphDelayCalc::findDelays(drvr)`: ONE vertex re-timed (`findVertexDelay`) in every scene,
/// from the slews the graphs hold — no queue is run. The fanouts it would enqueue stay queued for
/// the next `findDelays(level)`; nothing else is recomputed.
pub fn find_vertex_delay_scenes(incs: &mut [IncTimer], gs: &mut [Graph<'_>], parasitics: &[&HashMap<String, NetParasitics>], name: &str) -> Result<(), String> {
    for (inc, g) in incs.iter_mut().zip(gs.iter()) {
        inc.relevelize(g)?;
    }
    let indexes: Vec<std::sync::Arc<HashMap<String, usize>>> = gs.iter().map(|g| g.pin_index()).collect();
    let is_root = incs[0].levels.get(name).copied().unwrap_or(0) == 0;
    let fanout = visit_vertex(incs, gs, &indexes, parasitics, name, is_root)?;
    for inc in incs.iter_mut() {
        inc.delay_queue.extend(fanout.iter().cloned());
    }
    Ok(())
}

/// One scene's recomputed arrivals at a vertex: its index there, the paths, as stored.
type SceneArrivals = (usize, Vec<Path>, Vec<SPath>);

/// `Search::findArrivals(to_level)` (all with `None`) over every scene's imported paths at once:
/// ONE queue from the invalid vertices, level by level, as the reference's vertex holds every
/// scene's paths in one tag group. A vertex's arrivals changed if they did in ANY scene
/// (`arrivalsChanged` over the whole group); then every scene's are stored — the same tag group
/// keeps its array, else ONE new array for the vertex (a new generation in every scene) — and the
/// fanout enqueued; an unchanged vertex keeps what it had and enqueues nothing.
pub fn find_arrivals_scenes(incs: &mut [IncTimer], searches: &mut [Search<'_, '_>], to_level: Option<i32>) -> Result<(), String> {
    for inc in incs.iter_mut() {
        inc.arrival_visits.clear();
        inc.arrival_values.clear();
    }
    if !incs[0].arrivals_exist {
        for (inc, search) in incs.iter_mut().zip(searches.iter_mut()) {
            search.find_arrivals()?;
            inc.arrivals_exist = true;
            inc.invalid_arrivals.clear();
            inc.arrival_queue.clear();
            inc.pending_arrivals.clear();
            inc.store_all(search);
        }
        return Ok(());
    }
    let indexes: Vec<std::sync::Arc<HashMap<String, usize>>> = searches.iter().map(|s| s.graph.pin_index()).collect();
    let mut seed = BTreeSet::new();
    for inc in incs.iter_mut() {
        seed.extend(std::mem::take(&mut inc.invalid_arrivals));
        seed.extend(std::mem::take(&mut inc.arrival_queue));
    }
    let levels = incs[0].levels.clone();
    let mut by_level = queue_by_level(&levels, seed);
    // `findAllArrivals(thru_latches)`: a pass, then the postponed latch outputs visited with their
    // D -> Q fanin, again while any was postponed. A pass to a level leaves them postponed.
    loop {
        while let Some((&lvl, _)) = by_level.iter().next() {
            if to_level.is_some_and(|t| lvl > t) {
                break;
            }
            for name in by_level.remove(&lvl).expect("a level") {
                // `ArrivalVisitor::visit(vertex)`: a latch output is postponed.
                if indexes[0].get(&name).is_some_and(|&v| searches[0].is_latch_output(v)) {
                    for inc in incs.iter_mut() {
                        inc.pending_arrivals.insert(name.clone());
                    }
                    continue;
                }
                visit_arrivals(incs, searches, &indexes, &levels, &mut by_level, &name);
            }
        }
        if to_level.is_some() || incs[0].pending_arrivals.is_empty() {
            break;
        }
        // The postponed vertices in vertex id order (`VertexSet`).
        let mut pending: Vec<String> = std::mem::take(&mut incs[0].pending_arrivals).into_iter().collect();
        for inc in incs.iter_mut().skip(1) {
            inc.pending_arrivals.clear();
        }
        pending.sort_by_key(|n| indexes[0].get(n).map_or(usize::MAX, |&v| searches[0].vertex_id[v]));
        for name in pending {
            visit_arrivals(incs, searches, &indexes, &levels, &mut by_level, &name);
        }
    }
    let rest: BTreeSet<String> = by_level.into_values().flatten().collect();
    for inc in incs.iter_mut() {
        inc.arrival_queue.extend(rest.iter().cloned());
    }
    Ok(())
}

fn arrival_values_on() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("VYGES_STA_ARRIVAL_VALUES").is_some())
}

/// `ArrivalVisitor::visit(vertex, with_latch_edges)` over every scene: the new paths; when they
/// changed, stored and the fanout enqueued — a latch data pin's latch outputs postponed instead.
fn visit_arrivals(incs: &mut [IncTimer], searches: &mut [Search<'_, '_>], indexes: &[std::sync::Arc<HashMap<String, usize>>], levels: &HashMap<Name, i32>, by_level: &mut BTreeMap<i32, BTreeSet<String>>, name: &str) {
    let name = name.to_string();
    // Every scene's new paths, then the vertex's verdict.
    let mut news: Vec<Option<SceneArrivals>> = Vec::with_capacity(searches.len());
    let mut changed = false;
    let mut same_group = true;
    for k in 0..searches.len() {
        let Some(&v) = indexes[k].get(&name) else {
            news.push(None);
            continue;
        };
        let g = searches[k].graph;
        let new: Vec<Path> = searches[k].arrival_paths(v);
        let snew: Vec<SPath> = new.iter().map(|p| incs[k].spath(g, p)).collect();
        changed |= IncTimer::arrivals_changed(incs[k].paths.get(&name), &snew);
        if arrival_values_on() {
            let stored = incs[k].paths.get(&name);
            for p in &snew {
                let old = stored.and_then(|o| o.iter().find(|q| q.tag == p.tag)).map_or_else(|| "-".to_string(), |q| format!("{:08x}", q.arrival.to_bits()));
                let line = format!("{name}|{}|{}|{}|{old}|{:08x}", p.tag.rf, p.tag.mm, u8::from(p.tag.is_clock), p.arrival.to_bits());
                incs[k].arrival_values.push(line);
            }
        }
        // `setVertexArrivals`: the same tag group keeps its array.
        same_group &= incs[k].paths.get(&name).is_some_and(|o| o.len() == snew.len() && o.iter().all(|p| snew.iter().any(|q| q.tag == p.tag)));
        news.push(Some((v, new, snew)));
    }
    for inc in incs.iter_mut() {
        inc.arrival_visits.push((name.clone(), changed));
    }
    if !changed {
        return;
    }
    if !same_group {
        let generation = incs.iter().map(|i| i.next_gen).max().unwrap_or(0) + 1;
        for inc in incs.iter_mut() {
            inc.next_gen = generation;
            inc.path_gen.insert(name.clone(), generation);
        }
    }
    let mut fanout_names: BTreeSet<String> = BTreeSet::new();
    for (k, item) in news.into_iter().enumerate() {
        let Some((v, new, snew)) = item else { continue };
        let g = searches[k].graph;
        for &e in &g.out_edges[v] {
            if g.is_check(e) || is_latch_d_to_q(g, e) {
                continue;
            }
            fanout_names.insert(g_name(g, g.edges[e].to));
        }
        // A new path's required is 0 (`Path::Path`) until the required visit the change
        // invalidates — which compares against that 0.
        let mut stored = snew;
        for p in stored.iter_mut() {
            p.required = 0.0;
        }
        searches[k].paths[v] = new.into_iter().map(|mut p| {
            p.required = 0.0;
            p
        }).collect();
        let inc = &mut incs[k];
        inc.paths.insert(name.clone(), stored);
        inc.required_invalid(name.clone());
        // `constrainedRequiredsInvalid`: a clock arrival at a check clock pin changes the
        // required at the data pins it constrains.
        let is_clk = searches[k].paths[v].iter().any(|p| p.tag.is_clock);
        if is_clk && !g.vertices[v].is_driver {
            for &e in &g.out_edges[v] {
                if g.is_check(e) {
                    inc.required_invalid(g_name(g, g.edges[e].to));
                }
            }
        }
    }
    // `postponeLatchDataOutputs`.
    if let Some(&v) = indexes[0].get(&name) {
        for q in searches[0].latch_outputs_of(v) {
            let q = g_name(searches[0].graph, q);
            for inc in incs.iter_mut() {
                inc.pending_arrivals.insert(q.clone());
            }
        }
    }
    for n in fanout_names {
        by_level.entry(levels.get(n.as_str()).copied().unwrap_or(0)).or_default().insert(n);
    }
}

/// `Search::findRequireds(level)` (all with `None`) over every scene at once: ONE queue from the
/// invalid vertices, highest level first down to `level`; each visited vertex's requireds are
/// stored in every scene, and its fanin enqueued if one changed beyond fuzzy equality in ANY
/// scene (the vertex's whole tag group).
pub fn find_requireds_scenes(incs: &mut [IncTimer], searches: &mut [Search<'_, '_>], down_to_level: Option<i32>) -> Result<(), String> {
    for inc in incs.iter_mut() {
        inc.required_visits.clear();
    }
    if !incs[0].requireds_exist {
        for (inc, search) in incs.iter_mut().zip(searches.iter_mut()) {
            search.find_requireds()?;
            inc.requireds_exist = true;
            inc.invalid_requireds.clear();
            inc.required_queue.clear();
            inc.store_all(search);
        }
        return Ok(());
    }
    let indexes: Vec<std::sync::Arc<HashMap<String, usize>>> = searches.iter().map(|s| s.graph.pin_index()).collect();
    let mut seed = BTreeSet::new();
    for inc in incs.iter_mut() {
        seed.extend(std::mem::take(&mut inc.invalid_requireds));
        seed.extend(std::mem::take(&mut inc.required_queue));
    }
    let levels = incs[0].levels.clone();
    let mut by_level = queue_by_level(&levels, seed);
    while let Some((&lvl, _)) = by_level.iter().next_back() {
        if down_to_level.is_some_and(|t| lvl < t) {
            break;
        }
        for name in by_level.remove(&lvl).expect("a level") {
            let mut changed = false;
            let mut fanin_names: BTreeSet<String> = BTreeSet::new();
            for k in 0..searches.len() {
                let Some(&v) = indexes[k].get(&name) else { continue };
                let search = &mut searches[k];
                let g = search.graph;
                let req = search.required_values(v);
                for (i, r) in req.into_iter().enumerate() {
                    // `requiredsSave`: a register clock pin's ideal clock path is not a change.
                    if !crate::fuzzy::equal(search.paths[v][i].required, r) && !search.is_ideal_reg_clk_path(v, &search.paths[v][i].tag) {
                        changed = true;
                    }
                    search.paths[v][i].required = r;
                }
                if let Some(stored) = incs[k].paths.get_mut(&name) {
                    for sp in stored.iter_mut() {
                        if let Some(p) = search.paths[v].iter().find(|p| IncTimer::stag(g, &p.tag) == sp.tag) {
                            sp.required = p.required;
                        }
                    }
                }
                for &e in &g.in_edges[v] {
                    if g.is_check(e) || is_latch_d_to_q(g, e) {
                        continue;
                    }
                    fanin_names.insert(g_name(g, g.edges[e].from));
                }
            }
            for inc in incs.iter_mut() {
                inc.required_visits.push((name.clone(), changed));
            }
            if changed {
                for n in fanin_names {
                    by_level.entry(levels.get(n.as_str()).copied().unwrap_or(0)).or_default().insert(n);
                }
            }
        }
    }
    let rest: BTreeSet<String> = by_level.into_values().flatten().collect();
    for inc in incs.iter_mut() {
        inc.required_queue.extend(rest.iter().cloned());
    }
    Ok(())
}

impl IncTimer {
    /// The instances' cells and the ports' directions as the netlist has them now.
    pub fn track_netlist(&mut self, nl: &crate::netlist::Netlist) {
        if self.follower {
            return;
        }
        self.cells = nl.insts.iter().map(|i| (self.names.intern(&i.0), self.names.intern(&i.1))).collect();
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
        self.clear_prev(to);
        self.delay_invalid(to.to_string());
        self.relevelize_from_pin(to.to_string());
        if self.tracing() {
            eprintln!("VYGGD|{from}|{to}");
        }
        self.bump_edge(key);
    }

    /// The graph-edit trace is on for this timer.
    fn tracing(&self) -> bool {
        self.trace_edits && graph_trace()
    }

    /// `relevelizeFrom(vertex)` (traced while levelized, as the reference queues it only then).
    fn relevelize_from_pin(&mut self, pin: String) {
        if self.tracing() && !self.levels.is_empty() {
            eprintln!("VYGGR|{pin}");
        }
        self.relevelize_from.insert(pin);
    }

    /// `to`'s paths lose their prev paths (an edge into it is deleted).
    fn clear_prev(&mut self, to: &str) {
        if let Some(fx) = &mut self.effects {
            fx.push(Effect::ClearPrev(to.to_string()));
        }
        if let Some(paths) = self.paths.get_mut(to) {
            for p in paths.iter_mut() {
                p.prev = None;
            }
        }
    }

    /// A deleted or remade edge: the next one with this key is a different object.
    fn bump_edge(&mut self, key: EdgeKey) {
        if let Some(fx) = &mut self.effects {
            fx.push(Effect::Bump(key.clone()));
        }
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
            .map(|(k, a)| {
                let (f, t) = (format!("{inst}/{}", a.from), format!("{inst}/{}", a.to));
                (f.clone(), t.clone(), EdgeKey::Gate { from: f, to: t, cell: cell_name.to_string(), set: k })
            })
            .collect()
    }

    /// Forget a deleted vertex (`deleteVertexBefore`).
    fn delete_vertex(&mut self, v: &str) {
        if let Some(fx) = &mut self.effects {
            fx.push(Effect::DeleteVertex(v.to_string()));
        }
        self.slews.remove(v);
        if self.held_by.is_none() {
            self.written.push(self.names.intern(v));
        }
        self.paths.remove(v);
        self.path_gen.remove(v);
        self.levels.remove(v);
        for q in [&mut self.invalid_delays, &mut self.delay_queue, &mut self.invalid_arrivals, &mut self.arrival_queue, &mut self.invalid_requireds, &mut self.required_queue, &mut self.relevelize_from, &mut self.made] {
            q.remove(v);
        }
    }

    /// One edit event (`Db::edit_log_take`) as the reference's timer acts on it (`dbStaCbk`).
    pub fn apply_event(&mut self, cx: &EventCtx<'_>, ev: &str) -> Result<(), String> {
        if self.follower {
            return Err("a following scene timer takes its events from its leader (apply_event_scenes)".into());
        }
        let f: Vec<&str> = ev.split('|').collect();
        if self.tracing() {
            trace_callback(&f);
        }
        if !matches!(f.first(), Some(&("iterm_connect" | "bterm_connect" | "iterm_disconnect" | "bterm_disconnect"))) {
            self.net_drivers.clear();
        }
        match f.as_slice() {
            ["iterm_connect" | "bterm_connect", pin, net, pins] => self.connect_pin_after(cx, pin, net, pins),
            ["iterm_disconnect" | "bterm_disconnect", pin, net, pins] => self.disconnect_pin_before(cx, pin, net, pins),
            ["iterm_destroy", pin, _net] => self.delete_vertex(pin),
            ["inst_create", inst, master] => {
                // `makeInstanceAfter`: new vertices (level 0) and instance edges.
                self.cells.insert(self.names.intern(inst), self.names.intern(master));
                if let Some(cell) = self.cell(cx, master) {
                    for p in &cell.ports {
                        if self.tracing() {
                            eprintln!("VYGGV|{inst}/{}", p.name);
                        }
                        let v = format!("{inst}/{}", p.name);
                        self.delete_vertex(&v);
                        self.made.insert(v);
                    }
                }
                for (from, to, key) in self.inst_edges(cx, inst, master) {
                    if self.tracing() {
                        eprintln!("VYGGE|0|{from}|{to}");
                    }
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
                    self.disconnect_pin_before(cx, p, "", pins);
                }
            }
            // A port's vertex: made at level 0, or forgotten with its level.
            ["bterm_create", port, ..] => {
                self.made.insert(port.to_string());
            }
            ["bterm_destroy", port, ..] => {
                self.levels.remove(*port);
                self.made.remove(*port);
            }
            ["net_create", ..] | ["net_merge", ..] => {}
            _ => return Err(format!("edit event not modelled: {ev}")),
        }
        Ok(())
    }

    /// The drivers among `pins`, the pins on `net` (an event's list): kept per net while only
    /// loads come and go ([`IncTimer::net_drivers`]); a driver pin's own event reads them whole.
    /// `VYGES_STA_CHECK_EVENTS`: each kept answer against the pass.
    fn net_drivers_of(&mut self, cx: &EventCtx<'_>, pin_is_driver: bool, net: &str, pins: &str) -> (Vec<String>, Option<Vec<String>>) {
        if pin_is_driver || net.is_empty() {
            self.net_drivers.remove(net);
            let (d, l) = self.drivers_loads(cx, pins);
            return (d, Some(l));
        }
        if let Some(d) = self.net_drivers.get(net) {
            if std::env::var_os("VYGES_STA_CHECK_EVENTS").is_some() {
                assert_eq!(*d, self.drivers_loads(cx, pins).0, "kept drivers of {net} stale");
            }
            return (d.clone(), None);
        }
        let (d, _) = self.drivers_loads(cx, pins);
        self.net_drivers.insert(net.to_string(), d.clone());
        (d, None)
    }

    /// `Sta::connectPinAfter` (+ `connectDrvrPinAfter` / `connectLoadPinAfter`).
    fn connect_pin_after(&mut self, cx: &EventCtx<'_>, pin: &str, net: &str, pins: &str) {
        let Some((is_drv, is_load)) = self.pin_kind(cx, pin) else { return };
        self.arrival_invalid(pin.to_string());
        self.required_invalid(pin.to_string());
        let (drivers, loads) = self.net_drivers_of(cx, is_drv, net, pins);
        let loads = loads.unwrap_or_default();
        if is_drv {
            for l in loads.iter().filter(|l| *l != pin) {
                if self.tracing() {
                    eprintln!("VYGGE|1|{pin}|{l}");
                }
                self.bump_edge(EdgeKey::Wire { from: pin.to_string(), to: l.clone() });
                self.arrival_invalid(l.clone());
            }
            self.delay_invalid(pin.to_string());
            self.required_invalid(pin.to_string());
            self.relevelize_from_pin(pin.to_string());
        }
        if is_load {
            for d in drivers.iter().filter(|d| *d != pin) {
                if self.tracing() {
                    eprintln!("VYGGE|1|{d}|{pin}");
                }
                self.bump_edge(EdgeKey::Wire { from: d.clone(), to: pin.to_string() });
                self.delay_invalid(d.clone());
                self.required_invalid(d.clone());
                self.relevelize_from_pin(d.clone());
            }
            self.delay_invalid(pin.to_string());
            self.arrival_invalid(pin.to_string());
        }
    }

    /// `Sta::disconnectPinBefore`: the pin's wire edges deleted.
    fn disconnect_pin_before(&mut self, cx: &EventCtx<'_>, pin: &str, net: &str, pins: &str) {
        let Some((is_drv, is_load)) = self.pin_kind(cx, pin) else { return };
        let (drivers, loads) = self.net_drivers_of(cx, is_drv, net, pins);
        let loads = loads.unwrap_or_default();
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
            self.cells.insert(self.names.intern(inst), self.names.intern(to));
            return Ok(());
        };
        let equiv = equiv_cells_arcs(from_cell, to_cell);
        if self.tracing() {
            TRACE_SWAP_EQUIV.with(|e| e.set(equiv));
            eprintln!("VYGGC|{}|{inst}|{to}", if equiv { "replaceEquivCellBefore" } else { "replaceCellBefore" });
        }
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
        self.cells.insert(self.names.intern(inst), self.names.intern(to));
        Ok(())
    }
}

/// One edit event on every scene's timer: the reference's ONE graph takes it once. The first
/// timer (the leader) acts on it; each follower ([`IncTimer::follow`]) repeats the leader's value-side
/// effects — its invalidations, deleted vertices and remade edges are the leader's, read from the
/// one netlist — and keeps no structure of its own. A timer that does not follow acts on the event
/// itself, as before.
pub fn apply_event_scenes(incs: &mut [IncTimer], cx: &EventCtx<'_>, ev: &str) -> Result<(), String> {
    let Some((leader, rest)) = incs.split_first_mut() else { return Ok(()) };
    if rest.iter().any(|i| i.follower) {
        leader.effects = Some(Vec::new());
    }
    let done = leader.apply_event(cx, ev);
    let fx = leader.effects.take().unwrap_or_default();
    done?;
    for inc in rest.iter_mut() {
        if inc.follower {
            inc.repeat(&fx);
        } else {
            inc.apply_event(cx, ev)?;
        }
    }
    Ok(())
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
    g.vname(v)
}

fn is_latch_d_to_q(g: &Graph<'_>, e: usize) -> bool {
    matches!(g.edges[e].kind, EdgeKind::Gate { set } if g.arc_set(e, set).role == Role::LatchDtoQ)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
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
            insts: vec![crate::netlist::inst("b1", "buf"), crate::netlist::inst("b2", mid), crate::netlist::inst("b3", "buf")],
            ports: vec![("in".into(), PortDir::Input), ("out".into(), PortDir::Output)],
            nets: vec![
                Net { name: "n0".into(), pins: vec![inst(0, "A"), Conn::Port(0)] }.into(),
                Net { name: "n1".into(), pins: vec![inst(0, "X"), inst(1, "A")] }.into(),
                Net { name: "n2".into(), pins: vec![inst(1, "X"), inst(2, "A")] }.into(),
                Net { name: "n3".into(), pins: vec![inst(2, "X"), Conn::Port(1)] }.into(),
            ],
        }
    }

    fn libs() -> Vec<Library> {
        vec![Library::read(&parse(LIB).unwrap()).unwrap()]
    }

    // The reference has ONE graph, so a value written is a value read; ours holds several graph
    // sets beside the timer. A graph that imported the state takes, at its next import, every
    // vertex written since — slew and in-edge delays — and only those; a copy of the timer is
    // another state, imported whole.
    #[test]
    fn a_graph_that_imported_takes_what_was_written_since() {
        let libs = libs();
        let mut inc = IncTimer::default();
        let nl0 = chain("buf");
        let mut g = Graph::build(&libs, &nl0).unwrap();
        inc.find_delays(&mut g, &HashMap::new(), None).unwrap();
        inc.export(&g);
        let nl = chain("big");
        let (mut a, mut b) = (Graph::build(&libs, &nl).unwrap(), Graph::build(&libs, &nl).unwrap());
        inc.import(&mut a);
        inc.import(&mut b);
        inc.invalid_delays.insert("b1/X".into());
        // to the driver's level: the load's new slew is written by its driver, the load queued
        let level = inc.levels["b1/X"];
        inc.find_delays(&mut a, &HashMap::new(), Some(level)).unwrap();
        assert_eq!(inc.visited, ["b1/X"]);
        let port = b.index()["in"];
        b.slew[port] = [[7.0; 2]; 2];
        inc.import(&mut b);
        let load = a.index()["b2/A"];
        assert_ne!(a.slew[load], [[0.0; 2]; 2]);
        for v in (0..a.vertices.len()).filter(|&v| v != port) {
            assert_eq!(a.slew[v], b.slew[v], "{}", a.vname(v));
        }
        assert_eq!(a.delay, b.delay);
        assert_eq!(b.slew[port], [[7.0; 2]; 2], "not written since: left as it was");
        inc.clone().import(&mut b);
        assert_eq!(b.slew[port], a.slew[port], "a copy: imported whole");
    }

    // The reference's state hangs off its one graph's objects, so a pin is named once, by the
    // network. Ours is kept by name per scene: the scenes' timers share ONE pool of names, so a
    // pin's name is one allocation for every map of every scene (levels, slews, delays and each
    // edge's from-pin), and what the state holds is unchanged — the same values under the same
    // names as a timer keeping its own.
    #[test]
    fn the_scenes_timers_hold_each_name_once() {
        let libs = libs();
        let nl = chain("buf");
        let mut g = Graph::build(&libs, &nl).unwrap();
        let mut own = IncTimer::default();
        own.find_delays(&mut g, &HashMap::new(), None).unwrap();
        own.relevelize(&g).unwrap();
        own.export(&g);
        let (mut s0, mut s1) = (IncTimer::default(), IncTimer::default());
        s1.share_names(&s0);
        assert!(s0.names.same_as(&s1.names) && !s0.names.same_as(&own.names));
        for s in [&mut s0, &mut s1] {
            s.relevelize(&g).unwrap();
            s.export(&g);
        }
        let key = |m: &HashMap<Name, i32>, n: &str| m.get_key_value(n).map(|(k, _)| k.clone()).expect("a vertex's level");
        for v in 0..g.vertices.len() {
            let n = g.vname(v);
            let (a, b) = (key(&s0.levels, &n), key(&s1.levels, &n));
            assert!(std::sync::Arc::ptr_eq(&a, &b), "{n}: one name for both scenes");
            let (sk, _) = s0.slews.get_key_value(n.as_str()).expect("a vertex's slew");
            assert!(std::sync::Arc::ptr_eq(&a, sk), "{n}: one name for the levels and the slews");
            assert!(!std::sync::Arc::ptr_eq(&a, &key(&own.levels, &n)), "{n}: another pool, another name");
        }
        // an edge's from-pin is the pool's name too
        for (to, tails) in &s0.delays {
            for (t, _) in tails {
                let (EdgeTail::Gate { from, .. } | EdgeTail::Wire { from }) = t;
                assert!(std::sync::Arc::ptr_eq(from, &key(&s0.levels, from)), "{to}: from {from}");
            }
        }
        // the same state as a timer with its own names
        assert_eq!((&s0.levels, &s0.slews, &s0.delays), (&own.levels, &own.slews, &own.delays));
        assert_eq!((&s1.levels, &s1.slews, &s1.delays), (&own.levels, &own.slews, &own.delays));
    }

    // Rule (the reference's ONE graph takes each edit once, and every scene pass reads its one set
    // of levels): a scene timer that follows the first takes each event as the leader's effects
    // and keeps no levels, cells or net drivers, and the scenes then time exactly as timers that
    // each took every event themselves — the same invalid sets and edge generations after the
    // events, the same visits, slews and delays after the next delay pass.
    #[test]
    fn a_following_scene_timer_times_as_one_that_takes_every_event() {
        let libs = libs();
        let (nl0, nl1) = (chain("buf"), chain("big"));
        let start = |follow: bool| {
            let mut incs = vec![IncTimer::default(), IncTimer::default()];
            let (a, b) = incs.split_at_mut(1);
            if follow {
                b[0].follow(&a[0]);
            } else {
                b[0].share_names(&a[0]);
            }
            let mut gs = vec![Graph::build(&libs, &nl0).unwrap(), Graph::build(&libs, &nl0).unwrap()];
            for inc in incs.iter_mut() {
                inc.track_netlist(&nl0);
            }
            find_delays_scenes(&mut incs, &mut gs, &[&HashMap::new(), &HashMap::new()], None).unwrap();
            for (inc, g) in incs.iter_mut().zip(&gs) {
                inc.relevelize(g).unwrap();
                inc.export(g);
                // as after a search: every invalidation is kept
                inc.arrivals_exist = true;
                inc.requireds_exist = true;
            }
            incs
        };
        let (mut each, mut led) = (start(false), start(true));
        assert!(led[1].levels.is_empty() && led[1].cells.is_empty() && !led[0].levels.is_empty());
        let cx = EventCtx { libs: &libs, ideal_clock_mode: false };
        let terms = "A=n1=b1/X,b2/A;X=n2=b2/X,b3/A";
        for ev in [format!("swap_before|b2|buf|big|{terms}"), format!("swap_after|b2|{terms}")] {
            for inc in each.iter_mut() {
                inc.apply_event(&cx, &ev).unwrap();
            }
            apply_event_scenes(&mut led, &cx, &ev).unwrap();
        }
        let sets = |i: &IncTimer| (i.invalid_delays.clone(), i.invalid_arrivals.clone(), i.invalid_requireds.clone(), i.edge_gen.clone(), i.next_gen);
        assert_eq!(sets(&led[1]), sets(&each[1]), "the follower's invalid sets and edge generations");
        assert_eq!(sets(&led[0]), sets(&each[0]));
        assert!(!led[1].invalid_delays.is_empty() && !led[1].invalid_arrivals.is_empty() && !led[1].invalid_requireds.is_empty(), "the swap invalidated something");
        let pars = [&HashMap::new(), &HashMap::new()];
        let mut ge = vec![Graph::build(&libs, &nl1).unwrap(), Graph::build(&libs, &nl1).unwrap()];
        let mut gl = vec![Graph::build(&libs, &nl1).unwrap(), Graph::build(&libs, &nl1).unwrap()];
        for (inc, g) in each.iter().zip(ge.iter_mut()).chain(led.iter().zip(gl.iter_mut())) {
            inc.import(g);
        }
        find_delays_scenes(&mut each, &mut ge, &pars, None).unwrap();
        find_delays_scenes(&mut led, &mut gl, &pars, None).unwrap();
        for k in 0..2 {
            assert_eq!((&led[k].visited, &led[k].slews, &led[k].delays), (&each[k].visited, &each[k].slews, &each[k].delays), "scene {k}");
            assert_eq!((&gl[k].slew, &gl[k].delay), (&ge[k].slew, &ge[k].delay), "scene {k}");
        }
        assert!(!led[1].visited.is_empty(), "the swap re-timed something");
        assert!(led[1].apply_event(&cx, "net_create|x").is_err(), "a follower takes events only from its leader");
    }

    // Rule (the reference's delay calculator keeps its slews and arc delays in its ONE graph):
    // a graph that HOLDS the timer's values times an in-place edit exactly as the name-keyed maps
    // do (the same visits, slews and delays), with nothing kept by name; released, the maps hold
    // what it held.
    #[test]
    fn a_graph_holding_the_values_times_as_the_maps_do() {
        let libs = libs();
        let (nl0, nl1) = (Arc::new(chain("buf")), Arc::new(chain("big")));
        let cx = EventCtx { libs: &libs, ideal_clock_mode: false };
        let terms = "A=n1=b1/X,b2/A;X=n2=b2/X,b3/A";
        let evs = [format!("swap_before|b2|buf|big|{terms}"), format!("swap_after|b2|{terms}")];
        let run = |hold: bool| {
            let mut inc = IncTimer::default();
            inc.track_netlist(&nl0);
            let mut g = Graph::build_shared(&libs, nl0.clone(), None).unwrap();
            inc.find_delays(&mut g, &HashMap::new(), None).unwrap();
            inc.relevelize(&g).unwrap();
            inc.export(&g);
            inc.import(&mut g);
            if hold {
                inc.hold_in(&mut g).unwrap();
            }
            for ev in &evs {
                inc.apply_event(&cx, ev).unwrap();
            }
            g.apply_edits(&evs, nl1.clone(), None).unwrap();
            inc.import(&mut g);
            inc.find_delays(&mut g, &HashMap::new(), None).unwrap();
            (inc, g)
        };
        let ((maps, gm), (mut held, mut gh)) = (run(false), run(true));
        assert!(!maps.visited.is_empty(), "the swap re-timed something");
        assert_eq!(held.visited, maps.visited);
        assert_eq!((&gh.slew, &gh.delay), (&gm.slew, &gm.delay));
        assert!(held.slews.is_empty() && held.delays.is_empty() && held.written.is_empty(), "nothing kept by name");
        held.release(&mut gh).unwrap();
        assert!(!held.slews.is_empty() && !held.is_held());
        let (mut fa, mut fb) = (Graph::build_shared(&libs, nl1.clone(), None).unwrap(), Graph::build_shared(&libs, nl1.clone(), None).unwrap());
        maps.import(&mut fa);
        held.import(&mut fb);
        assert_eq!((&fb.slew, &fb.delay), (&fa.slew, &fa.delay), "released: a fresh graph imports the same values");
    }

    /// Rule (OpenSTA `GraphDelayCalc::findCheckEdgeDelays` is handed the `Edge*` itself; we hold
    /// keys by name): a key finds the edge a scan of every edge finds, for every edge of a graph,
    /// and a key naming no edge finds none.
    #[test]
    fn an_edge_key_finds_the_edge_a_scan_finds() {
        let libs = libs();
        let nl = chain("buf");
        let g = Graph::build(&libs, &nl).unwrap();
        let index = g.pin_index();
        assert!(g.edges.len() > 2, "a graph with edges to find");
        for e in 0..g.edges.len() {
            let key = IncTimer::edge_key(&g, e);
            let scan = (0..g.edges.len()).find(|&f| IncTimer::edge_key(&g, f) == key);
            assert_eq!(IncTimer::edge_of_key(&g, &index, &key), scan, "edge {e} {key:?}");
        }
        let none = EdgeKey::Wire { from: "nowhere".into(), to: "nothing".into() };
        assert_eq!(IncTimer::edge_of_key(&g, &index, &none), None);
    }

    /// The graph the repair just timed whole holds the values as it is (`hold_timed`), as the
    /// reference's one graph does after `findDelays()`: the same state, and the same timing after
    /// an edit, as writing every value out by name, reading it back and holding it.
    #[test]
    fn a_graph_timed_whole_holds_the_values_as_an_export_import_would() {
        let libs = libs();
        let (nl0, nl1) = (Arc::new(chain("buf")), Arc::new(chain("big")));
        let cx = EventCtx { libs: &libs, ideal_clock_mode: false };
        let terms = "A=n1=b1/X,b2/A;X=n2=b2/X,b3/A";
        let evs = [format!("swap_before|b2|buf|big|{terms}"), format!("swap_after|b2|{terms}")];
        let run = |direct: bool| {
            let mut inc = IncTimer::default();
            inc.track_netlist(&nl0);
            let mut g = Graph::build_shared(&libs, nl0.clone(), None).unwrap();
            g.find_delays(&HashMap::new(), None).unwrap();
            inc.relevelize(&g).unwrap();
            // as the repair after its whole pass
            inc.delays_exist = true;
            if direct {
                inc.hold_timed(&mut g).unwrap();
            } else {
                inc.export(&g);
                inc.import(&mut g);
                inc.hold_in(&mut g).unwrap();
            }
            assert!(inc.holds(&g));
            let before = (g.slew.clone(), g.delay.clone());
            for ev in &evs {
                inc.apply_event(&cx, ev).unwrap();
            }
            g.apply_edits(&evs, nl1.clone(), None).unwrap();
            inc.import(&mut g);
            inc.find_delays(&mut g, &HashMap::new(), None).unwrap();
            (inc, g, before)
        };
        let ((a, ga, ba), (b, gb, bb)) = (run(false), run(true));
        assert_eq!(bb, ba, "held as timed: the values an export and import give");
        assert!(!a.visited.is_empty(), "the swap re-timed something");
        assert_eq!(b.visited, a.visited);
        assert_eq!((&gb.slew, &gb.delay), (&ga.slew, &ga.delay));
        assert!(b.slews.is_empty() && b.delays.is_empty() && b.written.is_empty(), "nothing kept by name");
        let mut again = b.clone();
        assert!(again.hold_timed(&mut Graph::build_shared(&libs, nl1.clone(), None).unwrap()).is_err(), "held by one graph at a time");
    }

    // An edit the holding graph cannot make in place (here an edge id left free) leaves it
    // part-edited; kept (`hold_failed`) and named by the netlist its vertices then refer to, its
    // values come back by name, and a fresh graph imports what the maps would have given it.
    #[test]
    fn a_failed_edit_of_the_holding_graph_gives_its_values_back_by_name() {
        let libs = libs();
        let mut next = chain("buf");
        Arc::make_mut(&mut next.nets[2]).pins.pop();
        let next = Arc::new(next);
        let ev = ["iterm_disconnect|b3/A|n2|b2/X,b3/A".to_string()];
        let mut inc = IncTimer::default();
        let mut g = Graph::build_shared(&libs, Arc::new(chain("buf")), None).unwrap();
        inc.find_delays(&mut g, &HashMap::new(), None).unwrap();
        inc.export(&g);
        let mut by_maps = Graph::build_shared(&libs, next.clone(), None).unwrap();
        inc.clone().import(&mut by_maps);
        inc.import(&mut g);
        inc.hold_in(&mut g).unwrap();
        g.hold_failed = true;
        assert_eq!(g.apply_edits(&ev, next.clone(), None), Err(crate::graph::NotInPlace::Holes(1)));
        inc.release(&mut g).unwrap();
        let mut fresh = Graph::build_shared(&libs, next, None).unwrap();
        inc.import(&mut fresh);
        assert!(fresh.slew.iter().any(|s| *s != [[0.0; 2]; 2]), "values came back");
        assert_eq!((&fresh.slew, &fresh.delay), (&by_maps.slew, &by_maps.delay));
    }

    /// Only the holding graph imports while a graph holds the values: any other would read
    /// nothing (the maps are empty), so it is a fault, never a silent zero.
    #[test]
    #[should_panic(expected = "held by another graph")]
    fn a_graph_not_holding_the_values_cannot_import_them() {
        let libs = libs();
        let nl = chain("buf");
        let mut inc = IncTimer::default();
        let mut g = Graph::build(&libs, &nl).unwrap();
        inc.find_delays(&mut g, &HashMap::new(), None).unwrap();
        inc.export(&g);
        inc.import(&mut g);
        inc.hold_in(&mut g).unwrap();
        let mut other = Graph::build(&libs, &nl).unwrap();
        inc.import(&mut other);
    }

    // Our written log (the reference has none): trimmed to the oldest place a live graph stands
    // at, a graph at or past the trim still takes only what was written since; one behind it is
    // imported whole — never from a partial list — and reads the same values.
    #[test]
    fn a_trimmed_log_keeps_every_graph_at_or_past_the_trim_current() {
        let libs = libs();
        let mut inc = IncTimer::default();
        let nl0 = chain("buf");
        let mut g = Graph::build(&libs, &nl0).unwrap();
        inc.find_delays(&mut g, &HashMap::new(), None).unwrap();
        inc.export(&g);
        let nl = chain("big");
        let (mut a, mut b, mut behind) = (Graph::build(&libs, &nl).unwrap(), Graph::build(&libs, &nl).unwrap(), Graph::build(&libs, &nl).unwrap());
        inc.import(&mut a);
        inc.import(&mut behind);
        inc.invalid_delays.insert("b1/X".into());
        let level = inc.levels["b1/X"];
        inc.find_delays(&mut a, &HashMap::new(), Some(level)).unwrap();
        assert!(inc.written_len() > 0);
        // b imports at the log's end, then the log is trimmed to there
        inc.import(&mut b);
        inc.trim_written(inc.written_place(&b).unwrap());
        assert_eq!(inc.written_len(), 0, "every name before b's place let go");
        assert!(inc.written_place(&behind).unwrap() < inc.written_place(&b).unwrap(), "behind the trim");
        let port = behind.index()["in"];
        behind.slew[port] = [[7.0; 2]; 2];
        inc.import(&mut behind);
        assert_eq!(behind.slew[port], b.slew[port], "behind the trim: imported whole, not from a partial list");
        assert_eq!(behind.slew, b.slew);
        assert_eq!(behind.delay, b.delay);
        let before = b.slew.clone();
        inc.import(&mut b);
        assert_eq!(b.slew, before, "at the trim: nothing written since, nothing changed");
    }

    // Rule (relevelize): from each vertex an edit relevelized from, a fanout not above the level
    // is raised to level + 1 and invalid (`levelChangedBefore`); a made vertex starts at 0. Only
    // the raised vertices change, so visiting only them gives what the whole pass gives.
    #[test]
    fn relevelizing_only_the_raised_vertices_is_the_whole_pass() {
        let libs = libs();
        let mut inc = IncTimer::default();
        let nl0 = chain("buf");
        let g0 = Graph::build(&libs, &nl0).unwrap();
        inc.relevelize(&g0).unwrap();
        // b4 inserted between b1 and b2: n1 → b4 → n4 → b2.
        let mut nl = chain("buf");
        nl.insts.push(crate::netlist::inst("b4", "buf"));
        // b5 made and left unconnected: no visit reaches it, it is added at 0 all the same
        nl.insts.push(crate::netlist::inst("b5", "buf"));
        std::sync::Arc::make_mut(&mut nl.nets[1]).pins = vec![Conn::Inst(0, "X".into()), Conn::Inst(3, "A".into())];
        nl.nets.push(Net { name: "n4".into(), pins: vec![Conn::Inst(3, "X".into()), Conn::Inst(1, "A".into())] }.into());
        let g = Graph::build(&libs, &nl).unwrap();
        inc.delays_exist = true;
        inc.arrivals_exist = true;
        inc.made = ["b4/A", "b4/X", "b5/A", "b5/X", "gone/A"].map(String::from).into();
        inc.relevelize_from = ["b1/X".to_string()].into();
        let mut all = inc.clone();
        all.relevelize_all(&g, &g.pin_index());
        inc.relevelize_raised(&g, &g.pin_index());
        assert_eq!(inc.levels, all.levels);
        assert_eq!(inc.invalid_delays, all.invalid_delays);
        assert_eq!(inc.invalid_arrivals, all.invalid_arrivals);
        assert_eq!(inc.levels.len(), g.vertices.len(), "the made vertices in, a made name not in the graph left out");
        assert_eq!(inc.levels["b5/X"], 0);
        // the full levels are spaced, so the raise stops at b4: b2 is already above it
        assert_eq!(inc.invalid_delays, ["b4/A", "b4/X"].map(String::from).into());
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

    // Rule (loadSlewChanged over `slewCount()`: every scene's analysis points): one queue serves
    // every scene, so a load whose slew changed in ANY scene is re-timed in all of them — a scene
    // timed alone would stop where its own slews did not change.
    #[test]
    fn a_load_slew_change_in_one_scene_retimes_every_scene() {
        let la = libs();
        // Scene b: `big` has `buf`'s input capacitance, so b1's load does not change there.
        let lb = vec![Library::read(&parse(&LIB.replace("capacitance : 0.5;", "capacitance : 0.001;")).unwrap()).unwrap()];
        let (nl, nl2) = (chain("buf"), chain("big"));
        let par = HashMap::new();
        let mut incs = vec![IncTimer::default(), IncTimer::default()];
        let mut gs = vec![Graph::build(&la, &nl).unwrap(), Graph::build(&lb, &nl).unwrap()];
        find_delays_scenes(&mut incs, &mut gs, &[&par, &par], None).unwrap();
        let mut alone = incs[1].clone();
        for (inc, g) in incs.iter_mut().zip(&gs) {
            inc.export(g);
        }
        alone.export(&gs[1]);
        let mut gs = vec![Graph::build(&la, &nl2).unwrap(), Graph::build(&lb, &nl2).unwrap()];
        for (inc, g) in incs.iter_mut().zip(gs.iter_mut()) {
            inc.import(g);
            inc.invalid_delays.insert("b1/X".into());
        }
        find_delays_scenes(&mut incs, &mut gs, &[&par, &par], None).unwrap();
        assert_eq!(incs[1].visited[..3], ["b1/X".to_string(), "b2/A".into(), "b2/X".into()], "scene b follows scene a's change");
        let mut gb = Graph::build(&lb, &nl2).unwrap();
        alone.import(&mut gb);
        alone.invalid_delays.insert("b1/X".into());
        alone.find_delays(&mut gb, &par, None).unwrap();
        assert_eq!(alone.visited, ["b1/X"], "timed alone, scene b stops at b1");
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

    // Rule (`connectPinAfter` / `disconnectPinBefore` read the net's drivers at the event): the
    // drivers kept per net while only loads come and go answer as a pass over the event's pins;
    // a driver's own event reads them again, and any other event drops every kept net.
    #[test]
    fn the_kept_drivers_of_a_net_answer_as_the_pass() {
        let libs = libs();
        let cx = EventCtx { libs: &libs, ideal_clock_mode: false };
        let mut inc = IncTimer::default();
        for i in ["b1", "b2", "b3", "b4"] {
            inc.cells.insert(i.into(), "buf".into());
        }
        let pass = |inc: &IncTimer, pins: &str| inc.drivers_loads(&cx, pins).0;
        let pins = "b1/X,b2/A,b3/A";
        assert_eq!(inc.net_drivers_of(&cx, false, "n1", pins).0, pass(&inc, pins));
        assert!(inc.net_drivers.contains_key("n1"), "a load's event keeps them");
        // loads leave one at a time (a fanout repair): each answer from the kept list
        for pins in ["b1/X,b3/A", "b1/X"] {
            assert_eq!(inc.net_drivers_of(&cx, false, "n1", pins).0, pass(&inc, pins));
        }
        // the driver moves: its own event reads the net again; the next load sees the new one
        let (d, loads) = inc.net_drivers_of(&cx, true, "n1", "b4/X");
        assert_eq!((d, loads), (vec!["b4/X".to_string()], Some(vec![])));
        let pins = "b4/X,b2/A";
        assert_eq!(inc.net_drivers_of(&cx, false, "n1", pins).0, ["b4/X"]);
        inc.apply_event(&cx, "net_create|n9").unwrap();
        assert!(inc.net_drivers.is_empty(), "another event drops every kept net");
    }
}

/// `VYGES_STA_GRAPH_TRACE`: print, in call order, each netlist-edit callback the timer applies and
/// the graph edits it implies (stderr) — the reference's graph-edit trace, by pin name.
fn graph_trace() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("VYGES_STA_GRAPH_TRACE").is_some())
}

thread_local! {
    /// The last traced swap's arcs were equivalent (`swap_master_arcs_equiv_`).
    static TRACE_SWAP_EQUIV: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// The reference callback an edit event is delivered to, as the graph-edit trace names it.
fn trace_callback(f: &[&str]) {
    let line = match f {
        ["iterm_connect" | "bterm_connect", pin, net, ..] => format!("connectPinAfter|{pin}|{net}"),
        ["iterm_disconnect" | "bterm_disconnect", pin, net, ..] => format!("disconnectPinBefore|{pin}|{net}"),
        ["iterm_destroy", pin, ..] => format!("deletePinBefore|{pin}|"),
        ["inst_create", inst, ..] => format!("makeInstanceAfter|{inst}|"),
        ["inst_destroy", inst] => format!("deleteInstanceBefore|{inst}|"),
        // the after half of the swap `swap_before` named (`swap_master_arcs_equiv_`)
        ["swap_after", inst, ..] => {
            let equiv = TRACE_SWAP_EQUIV.with(|e| e.get());
            format!("{}|{inst}|", if equiv { "replaceEquivCellAfter" } else { "replaceCellAfter" })
        }
        ["net_destroy", net, ..] => format!("deleteNetBefore|{net}|"),
        _ => return,
    };
    eprintln!("VYGGC|{line}");
}
