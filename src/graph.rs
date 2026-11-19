// SPDX-License-Identifier: Apache-2.0
//! The timing graph and delay calculation over it.
//!
//! Rules:
//! - a root driver (an input port with no drive) has slew 0; its loads' wire delays and slews come
//!   from the input-port delay (the Elmore delay at the load's thresholds), set — not merged;
//! - a driver pin starts at the min/max INITIAL value (`max`: −1e30, `min`: +1e30) and takes, over
//!   every arc of every in-edge, the slew that is fuzzily worse (`max`: greater); its loads' slews
//!   and wire delays merge the same way per output transition; a transition no arc drives is
//!   zeroed (slew 0, wire delays 0, load slews 0);
//! - a gate arc's input slew is its from-pin's slew for the arc's from transition and this
//!   min/max — except a register clock-to-Q edge from an IDEAL clock's network, which reads the
//!   ideal clock slew (0) instead, as a timing check's clock side does ([`Graph::ideal_clock`]);
//! - the parasitic is the pi model reduced from the net's network at the pin caps of this
//!   transition and min/max; it is dropped when its total is under the net's pin cap, and without
//!   one the gate is lumped at that pin cap (wire delay 0, load slew = driver slew);
//! - a timing check's delay reads the clock pin's slew from the OPPOSITE min/max (OCV) and the
//!   data pin's from its own.
//! - every merge compares FUZZILY ([`crate::fuzzy`]), which is not transitive — so a driver's
//!   in-edges are visited in a fixed order: newest first (edges are prepended to the vertex's
//!   list), min then max, then each arc in its set.
//!
//! A merge only combines the arcs of one driver, so any topological order serves for levels.

use std::collections::HashMap;
use std::sync::Arc;

use crate::dcalc::{dspf_wire_delay_slew, threshold_adjust, Dmp, LibThresholds, Thresholds};
use crate::liberty::{Cell, Direction, Library, Model, Role, FALL, MAX, MIN, RISE};
use crate::netlist::{Conn, Netlist, PortDir};
use crate::parasitics::{reduce_to_pi_elmore, Network, NodePins};

/// Whether delay calculation leaves vertex `v`'s `mm` slews as they are ([`Graph::slew_blocked`]);
/// by field, so a caller may write the graph's slews while asking.
fn slew_blocked_by(blocked: &Option<std::sync::Arc<HashMap<usize, [bool; 2]>>>, annotated: &HashMap<usize, f32>, v: usize, mm: usize) -> bool {
    match blocked {
        Some(b) => b.get(&v).is_some_and(|x| x[mm]),
        None => mm == MAX && annotated.contains_key(&v),
    }
}

/// The structure behind `a`, to edit: only where this view is its one holder. A structure shared by
/// scene views is edited through the scene set, which holds it alone while it edits; a second
/// holder here is a bug, not a copy to make.
pub(crate) fn uniq<T>(a: &mut Arc<T>) -> &mut T {
    Arc::get_mut(a).expect("a graph structure shared by scene views is edited only through its scene set")
}

mod edit;
pub use edit::{NotInPlace, SlotDelta};

/// A vertex: one pin (a bidirect pin is two). As the reference's `Vertex` holds its `Pin*` and
/// no name, a vertex holds its pin (`conn`); its name is the pin's ([`Graph::vname`]) and its
/// port the pin's ([`Vertex::port`]).
#[derive(Debug, Clone, PartialEq)]
pub struct Vertex {
    pub conn: Conn,
    pub is_driver: bool,
    /// The library and cell of an instance pin; the cell name is shared by every vertex of it.
    pub lib: Option<usize>,
    pub cell: Option<std::sync::Arc<str>>,
}

impl Vertex {
    /// The instance pin's port, `None` for a top-level port.
    pub fn port(&self) -> Option<&str> {
        match &self.conn {
            Conn::Inst(_, p) => Some(p),
            Conn::Port(_) => None,
        }
    }
}

#[derive(Debug, Clone)]
pub enum EdgeKind {
    /// An arc set of the instance's cell (index into `Cell::arc_sets`).
    Gate { set: usize },
    Wire,
}

#[derive(Debug, Clone)]
pub struct Edge {
    pub from: usize,
    pub to: usize,
    pub kind: EdgeKind,
}

/// A net's estimated RC network, nodes named by pin or point.
#[derive(Debug, Clone, Default)]
pub struct NetParasitics {
    pub node_names: Vec<String>,
    pub network: Network,
    /// The ports' `set_load -pin_load` in force when the estimator REDUCED this net, per
    /// `[rf][min/max]` — `None`: reduced against the loads in force now.
    ///
    /// Rule: the placement estimator reduces each net to its pi model as soon as it builds the
    /// network, and keeps only the pi model; a later `set_load` on a port changes the pin cap the
    /// delay calculation compares the pi model against, never the pi model itself.
    pub port_pin_caps: Option<HashMap<String, [[Option<f32>; 2]; 2]>>,
    /// A pi model and Elmore delays SET on the net rather than reduced from `network`
    /// (`Sta::makePiElmore` + `setElmore`): `elmore` indexes `node_names`. When present it is the
    /// net's parasitic as the delay calculator finds it, unchanged by any pin capacitance.
    pub pi: Option<crate::parasitics::PiElmore>,
    /// The set pi model is for RISING transitions only (`makePiElmore(pin, RiseFall::rise(), …)`):
    /// a falling transition finds no parasitic for the net at all.
    pub pi_rise_only: bool,
}

/// The netlist a graph was built over: borrowed from its caller, or shared with it — a graph
/// that outlives the netlist generation it was built from (it is edited in place) holds the
/// generation it is current with.
#[derive(Debug, Clone)]
pub enum NetlistRef<'a> {
    Borrowed(&'a Netlist),
    Shared(std::sync::Arc<Netlist>),
}

impl std::ops::Deref for NetlistRef<'_> {
    type Target = Netlist;
    fn deref(&self) -> &Netlist {
        match self {
            NetlistRef::Borrowed(n) => n,
            NetlistRef::Shared(n) => n,
        }
    }
}

/// `DeratingFactorsGlobal`'s factors, `[type][clock/data][rf][early/late]`: the type
/// [`DERATE_CELL_DELAY`], [`DERATE_CELL_CHECK`] or [`DERATE_NET_DELAY`] (`TimingDerateType`
/// order), [`DERATE_CLK`] or [`DERATE_DATA`] (`PathClkOrData`); early is MIN, late MAX.
pub type DerateFactors = [[[[Option<f32>; 2]; 2]; 2]; 3];
pub const DERATE_CELL_DELAY: usize = 0;
pub const DERATE_CELL_CHECK: usize = 1;
pub const DERATE_NET_DELAY: usize = 2;
pub const DERATE_CLK: usize = 0;
pub const DERATE_DATA: usize = 1;

pub struct Graph<'a> {
    pub libs: &'a [Library],
    pub netlist: NetlistRef<'a>,
    /// 🔑 The STRUCTURE (vertices, edges, edge lists, nets, loop breaking, the name index) is held
    /// by `Arc`: the scenes of one design share it, as the reference's scenes share its one
    /// `Graph` and keep only their slews and arc delays apart (`ap_index`). Edited only where no
    /// other view holds it ([`uniq`]: a shared structure is never copied behind the caller's back).
    pub vertices: Arc<Vec<Vertex>>,
    pub edges: Arc<Vec<Edge>>,
    pub in_edges: Arc<Vec<Vec<usize>>>,
    pub out_edges: Arc<Vec<Vec<usize>>>,
    /// The net each vertex is on, if any.
    pub vertex_net: Arc<Vec<Option<usize>>>,
    /// `slew[v][rf][min/max]`.
    pub slew: Vec<[[f32; 2]; 2]>,
    /// Per edge: gate arcs `[arc][min/max]`; wire edges `[rf][min/max]`.
    pub delay: Vec<Vec<[f32; 2]>>,
    /// Load vertices whose max slew is held at a limit when it is exceeded (an annotated slew,
    /// as a resizer sets on loads it will repair, so the excess does not propagate).
    pub slew_limit: HashMap<usize, f32>,
    /// The loads [`Graph::find_delays`] held at their limit, in the order it timed them.
    pub clamped: Vec<usize>,
    /// Threads the delay calculation and the search run on ([`crate::par`]; 1: serial). Set
    /// with [`Graph::set_thread_count`]. Every result is the same at any count.
    pub thread_count: usize,
    /// Annotated max slews (`setAnnotatedSlew`): the vertex's max slew on both transitions IS the
    /// value, whatever the delay calculation finds — applied before its fanout reads it.
    pub slew_annotated: HashMap<usize, f32>,
    /// Which of a vertex's slews delay calculation may not overwrite, per min/max
    /// (`Vertex::slewAnnotated(rf, min_max)`), when the graph is one scene of several. `None`: this
    /// scene's own max annotations ([`Graph::slew_annotated`]) are the blocked ones.
    ///
    /// 🔑 The reference keeps ONE set of annotation bits per vertex for every scene: the bit is
    /// `ap_index * 2 + rf` with any `ap_index > 1` folded to 0 (`Vertex::setSlewAnnotated`), and a
    /// scene's max index is `scene * 2 + 1`. So a max annotation in the FIRST scene sets the max
    /// bit (that load's max slew is then kept in EVERY scene), and one in any later scene sets the
    /// MIN bit (its min slews are kept in every scene, and its own max is not protected at all).
    /// It looks unintended (the bits predate scenes), but it decides results: transcribed as is
    /// (asked upstream: parallaxsw/OpenSTA#581).
    pub slew_blocked: Option<std::sync::Arc<HashMap<usize, [bool; 2]>>>,
    /// The SDC environment the delay calculation reads; empty unless a caller sets it.
    pub sdc: SdcEnv,
    /// Per vertex, the (library, cell) its pin capacitance is read from instead of its own — a
    /// master swapped under a timer whose graph keeps the old arcs (a scratch characterization
    /// timer that is not notified of `swapMaster`).
    pub pin_cap_cell: HashMap<usize, (usize, String)>,
    /// Vertices on an IDEAL clock's network (`ClkNetwork::isIdealClock`). A register's clock-to-Q
    /// edge and a timing check read their clock-side slew from such a vertex as the ideal clock
    /// slew, 0 (`edgeFromSlew`, `checkEdgeClkSlew`); every other edge reads the graph's slew.
    /// Empty unless a caller sets it — a propagated clock has none.
    pub ideal_clock: std::collections::HashSet<usize>,
    /// `ClkNetwork::idealClkSlew` at each pin of [`Graph::ideal_clock`] where a clock reaching it
    /// has a `set_clock_transition`, `[rf][min/max]`; a pin not here takes 0.
    pub ideal_clock_slew: HashMap<usize, [[f32; 2]; 2]>,
    /// Per instance, the library and cell whose arc sets its OUTPUT-to-output edges still carry:
    /// `replaceEquivCellBefore` swaps the arc sets of the edges leaving an instance's INPUT pins
    /// only, so after an equivalent-arcs replacement an edge from one output to another (a half
    /// adder's CON -> SN) keeps the cell it was made with. See [`Graph::set_stale_out_arcs`].
    pub stale_out_arcs: HashMap<String, (usize, String)>,
    /// The edges from a bidirect pin's driver vertex into its instance (`isBidirectInstPath`,
    /// whole graph only): levelization and the activity search skip them
    /// (`bidirect_inst_paths_enabled` is off by default); delay calculation reads them.
    pub bidirect_inst_paths: Arc<std::collections::HashSet<usize>>,
    /// The wire edge from a bidirect top port's load vertex to its driver vertex
    /// (`isBidirectPortPath`, whole graph only): levelization skips it, so the driver is a root;
    /// the activity search crosses it.
    pub bidirect_port_paths: Arc<std::collections::HashSet<usize>>,
    /// `Levelize::disabled_loop_edges_`: the edges loop breaking disabled ([`Graph::break_loops`],
    /// at the build) — every edge closing a combinational loop. Levelization, delay calculation
    /// and the searches pass over them as the reference's predicates do (`isDisabledLoop`).
    pub disabled_loops: Arc<std::collections::HashSet<usize>>,
    /// The whole loop breaking: the disabled edges in the order found, the loops, `loop_edges_`.
    pub loops: Arc<crate::levelize::LoopBreaking>,
    /// Every vertex's index by name, built on first use: the vertices never change once the graph
    /// is built, and [`Graph::load_cap`] and its kin are called per driver by a repair loop.
    /// Shared, so a caller holding it ([`Graph::pin_index`]) does not copy every name.
    index_cache: std::sync::OnceLock<std::sync::Arc<HashMap<String, usize>>>,
    /// Each net's slot in [`Graph::netlist`] by name, kept across in-place edits (an edit replaces
    /// nets in their slots or appends them): [`Graph::apply_edits`] finds the nets its edits named
    /// without a pass over every net. Checked by name at each slot read; made again on a miss.
    net_slots: Arc<Option<HashMap<String, usize>>>,
    /// A scene view's arc sets ([`crate::scenes`]): the structure's edges carry the LINK cell's
    /// set index, and this scene times through its own cell's set of the same key
    /// (`TimingArc::sceneArc`). `None`: the graph's sets are its own (a one-scene graph, the link view).
    pub(crate) scene_sets: Option<Arc<crate::scenes::SceneSets>>,
    /// While a scene set edits its structure through this view: each per-scene array write, in
    /// order, for the other views to replay ([`crate::scenes::apply_edits`]).
    pub(crate) array_ops: Option<Vec<crate::scenes::ArrayOp>>,
    /// The caller's: how far into its log of netlist edits this graph is ([`Graph::apply_edits`]
    /// brings it on). `None` from a build — not known, so a caller builds again.
    pub edit_seq: Option<usize>,
    /// The timer's: the state it imported, as its stamp and how far into its log of written names
    /// ([`crate::incr::IncTimer::import`] brings it on). `None` from a build.
    pub state_seq: Option<(u64, usize)>,
    /// An edit this graph cannot make in place keeps its vertices, named by the netlist they then
    /// refer to, so the values it holds can still be read by name
    /// ([`crate::incr::IncTimer::release`]) before it is let go. Set by a caller whose graph holds
    /// a timer's values; such a graph is not recycled.
    pub hold_failed: bool,
}

/// [`Graph::parasitic_load`]: `(pin_cap, wire_cap, the pi model and its node names)`.
type ParasiticLoad<'p> = (f32, f32, Option<(crate::parasitics::PiElmore, &'p [String])>);
/// What one delay visit stores ([`Graph::vertex_delays`]): whole slew rows of vertices and whole
/// delay rows of edges, whether the vertex enters `clamped`, and its trace lines.
#[derive(Default, Debug)]
pub struct VertexDelays {
    pub slews: Vec<(usize, [[f32; 2]; 2])>,
    pub delays: Vec<(usize, Vec<[f32; 2]>)>,
    pub clamped: bool,
    pub trace: Vec<String>,
}

/// The state a driver's delay visit writes, as copies: its own slews, each distinct load's slews
/// (`slot[i]`: wire `i`'s load), each wire's delay row and each fanin edge's delay row (copied
/// on first write). A read of a slew the visit already wrote — a fanin from-vertex that is also
/// one of its loads, on a broken loop — reads the copy, as the in-place visit did.
struct Overlay {
    v: usize,
    sv: [[f32; 2]; 2],
    loads: Vec<usize>,
    ls: Vec<[[f32; 2]; 2]>,
    slot: Vec<usize>,
    wire_delay: Vec<Vec<[f32; 2]>>,
    fanin_delay: Vec<(usize, Vec<[f32; 2]>)>,
    /// (fanin from-vertex, load slot) for a from-vertex among the loads.
    alias: Vec<(usize, usize)>,
}

impl Overlay {
    fn new(g: &Graph<'_>, v: usize, wires: &[usize], fanin: &[usize]) -> Overlay {
        let mut loads: Vec<usize> = Vec::with_capacity(wires.len());
        let mut slot = Vec::with_capacity(wires.len());
        let mut by_load: HashMap<usize, usize> = HashMap::new();
        for &w in wires {
            let load = g.edges[w].to;
            let k = if wires.len() <= 32 { loads.iter().position(|&l| l == load) } else { by_load.get(&load).copied() };
            let k = k.unwrap_or_else(|| {
                loads.push(load);
                if wires.len() > 32 {
                    by_load.insert(load, loads.len() - 1);
                }
                loads.len() - 1
            });
            slot.push(k);
        }
        let ls = loads.iter().map(|&l| g.slew[l]).collect();
        let wire_delay = wires.iter().map(|&w| g.delay[w].clone()).collect();
        let mut alias = Vec::new();
        for &e in fanin {
            let from = g.edges[e].from;
            let k = if wires.len() <= 32 { loads.iter().position(|&l| l == from) } else { by_load.get(&from).copied() };
            if let Some(k) = k {
                alias.push((from, k));
            }
        }
        Overlay { v, sv: g.slew[v], loads, ls, slot, wire_delay, fanin_delay: Vec::new(), alias }
    }

    /// Vertex `x`'s slews as the visit sees them so far.
    fn slew_of(&self, g: &Graph<'_>, x: usize) -> [[f32; 2]; 2] {
        if x == self.v {
            return self.sv;
        }
        match self.alias.iter().find(|a| a.0 == x) {
            Some(&(_, k)) => self.ls[k],
            None => g.slew[x],
        }
    }

    /// Fanin edge `e`'s delay row, copied on first use.
    fn fanin_row(&mut self, g: &Graph<'_>, e: usize) -> &mut Vec<[f32; 2]> {
        let k = match self.fanin_delay.iter().position(|r| r.0 == e) {
            Some(k) => k,
            None => {
                self.fanin_delay.push((e, g.delay[e].clone()));
                self.fanin_delay.len() - 1
            }
        };
        &mut self.fanin_delay[k].1
    }

    fn into_delays(self, wires: &[usize], trace: Vec<String>) -> VertexDelays {
        let mut slews = Vec::with_capacity(1 + self.loads.len());
        slews.push((self.v, self.sv));
        slews.extend(self.loads.into_iter().zip(self.ls));
        let mut delays = self.fanin_delay;
        delays.extend(wires.iter().copied().zip(self.wire_delay));
        VertexDelays { slews, delays, clamped: false, trace }
    }
}


/// A reduced parasitic's loads by name (`ReducedParasitic`'s pin-to-Elmore map, read by
/// `findElmore(load_pin)`): the first entry for a name, as a pass over the list finds it. Built
/// only for a net with more loads than a pass is worth: one pass per load was O(fanout²) per
/// driver — minutes on a net of 10^4 loads.
struct LoadElmores<'p> {
    by_name: Option<HashMap<&'p str, f32>>,
}

impl<'p> LoadElmores<'p> {
    const PASS_MAX: usize = 16;

    fn new(pe: Option<&(crate::parasitics::PiElmore, &'p [String])>) -> Self {
        let by_name = pe.filter(|(p, _)| p.elmore.len() > Self::PASS_MAX).map(|(p, names)| {
            let mut m: HashMap<&'p str, f32> = HashMap::with_capacity(p.elmore.len());
            for &(n, el) in &p.elmore {
                m.entry(names[n].as_str()).or_insert(el);
            }
            m
        });
        LoadElmores { by_name }
    }

    /// The load's Elmore delay, `None` when the parasitic has none for it.
    fn get(&self, p: &crate::parasitics::PiElmore, names: &[String], load: &str) -> Option<f32> {
        match &self.by_name {
            Some(m) => m.get(load).copied(),
            None => p.elmore.iter().find(|(n, _)| names[*n] == load).map(|&(_, el)| el),
        }
    }
}

/// The SDC loads and drives delay calculation reads (`Sdc::connectedCap`, `seedDrvrSlew`).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SdcEnv {
    /// The design-wide `set_timing_derate` factors (`DeratingFactorsGlobal`), where any is set.
    /// None: every factor 1.
    pub derates: Option<Box<DerateFactors>>,
    /// `sta::set_crpr_enabled 0` (`Variables::crprEnabled` false; `repair_timing -skip_crpr` sets
    /// it for the repair): CRPR is not active ([`crate::search::Search::crpr_active`]). Default
    /// false — CRPR enabled, as the reference starts.
    pub crpr_disabled: bool,
    /// `set_load` on a net (`setNetWireCap`), keyed — as `drvr_pin_wire_cap_map_` is — by each of
    /// the net's DRIVER pins when it was set: per min/max, the wire cap and `-subtract_pin_load`.
    pub net_wire_cap: HashMap<String, [Option<(f32, bool)>; 2]>,
    /// `set_load -pin_load` on a port (`portExtCap`'s pin cap), per `[rf][min/max]`.
    pub port_pin_cap: HashMap<String, [[Option<f32>; 2]; 2]>,
    /// `set_input_transition` on an input port (`InputDrive::slew`), per `[rf][min/max]`.
    pub input_slew: HashMap<String, [[Option<f32>; 2]; 2]>,
    /// `set_driving_cell` on an input port (both min and max, rise and fall).
    pub input_drive: HashMap<String, InputDrive>,
    /// The pins the logic simulation holds constant (`Sim::isConstant`: a signal pin on a power or
    /// ground net, and what propagates from it), by name, with their values. Delay calculation
    /// takes no arc FROM one (`DcalcPred::searchFrom`); the search neither leaves nor enters one
    /// (`SearchPred0`), and on an instance with one it disables conditional arcs and arcs by their
    /// sense under the constants (`Sim::findDisabledEdges`).
    pub constants: HashMap<String, bool>,
    /// The pins a `set_case_analysis rising` / `falling` holds, and what propagates from them, by
    /// name: only that transition (`RISE` / `FALL`) is searched from or to them
    /// (`searchThruSimEdge`).
    pub sim_transitions: HashMap<String, usize>,
    /// `set_clock_transition` (`Clock::slews_`) per clock that takes one (not virtual: Warning
    /// 419), in clock order: its source pins and its `[rf][min/max]` slews where set. A timer
    /// with ideal clocks folds these into [`Graph::ideal_clock_slew`] over each clock's network
    /// (`ClkNetwork::idealClkSlew`); a clock with none is absent.
    pub clock_slews: Vec<ClockSlew>,
}

/// One clock's `set_clock_transition` values ([`SdcEnv::clock_slews`]).
#[derive(Debug, Clone, PartialEq)]
pub struct ClockSlew {
    pub name: String,
    pub sources: Vec<String>,
    /// `[rf][min/max]`, in the timer's unit; `None` where no command set it (`Clock::slew`: 0).
    pub slew: [[Option<f32>; 2]; 2],
}

/// One `set_driving_cell`: the cell (the first library holding it), the port it drives from
/// (`-pin`), the port its arcs start at (`-from_pin`, else `driveCellDefaultFromPort`: of the
/// arc sets into `to_port`, the from-port first in the cell's port order), and the slews at that
/// port per from-transition.
#[derive(Debug, Clone, PartialEq)]
pub struct InputDrive {
    pub cell: String,
    pub from_port: Option<String>,
    pub to_port: String,
    pub from_slews: [f32; 2],
}

/// `Graph::makePinVertices` over an instance's pins: its cell's ports in the master's terminal
/// order (`pins`, the database's) when known, else the liberty's; a port the master lacks has none.
fn instance_ports<'c>(c: &'c Cell, has: Option<&Vec<String>>) -> Vec<&'c crate::liberty::Port> {
    match has {
        Some(h) => h.iter().filter_map(|n| c.port(n)).collect(),
        None => c.ports.iter().collect(),
    }
}

fn cell<'a>(libs: &'a [Library], lib: usize, name: &str) -> &'a Cell {
    &libs[lib].cells[name]
}

/// A recycled graph's allocations, kept for the next build on this thread ([`Graph::recycle`]).
#[derive(Default)]
struct Spare {
    vertices: Vec<Vertex>,
    edges: Vec<Edge>,
    in_edges: Vec<Vec<usize>>,
    out_edges: Vec<Vec<usize>>,
    delay: Vec<Vec<[f32; 2]>>,
    index: Option<std::sync::Arc<HashMap<String, usize>>>,
    /// The recycled graph's instances and ports, which its vertices' pins index: what names them.
    insts: Vec<std::sync::Arc<(String, String)>>,
    ports: Vec<(String, PortDir)>,
}

impl Spare {
    /// A recycled vertex's pin name.
    fn name(&self, v: &Vertex) -> String {
        match &v.conn {
            Conn::Inst(i, p) => format!("{}/{p}", self.insts[*i].0),
            Conn::Port(i) => self.ports[*i].0.clone(),
        }
    }
}

/// How many recycled graphs a thread keeps.
const MAX_SPARES: usize = 4;

thread_local! {
    static SPARES: std::cell::RefCell<Vec<Spare>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// The largest recycled graph on this thread, if any.
fn take_spare() -> Option<Spare> {
    SPARES.with(|p| {
        let mut p = p.borrow_mut();
        let k = (0..p.len()).max_by_key(|&k| p[k].vertices.len())?;
        Some(p.swap_remove(k))
    })
}

impl<'a> Graph<'a> {
    /// Done with this graph: its vertices, edge lists and name index are kept for the next build
    /// on this thread, which takes an instance's vertices from them unchanged where it can and
    /// updates the index rather than making it again. A loop that rebuilds the graph after each
    /// edit (a repair) pays for what the edit changed, not for every pin's name. The next graph is
    /// the one a fresh build makes (`VYGES_STA_CHECK_BUILD=1` checks it).
    pub fn recycle(self) {
        // its vertices may be a failed edit's: never handed to a build
        if self.hold_failed {
            return;
        }
        let (insts, ports) = (self.netlist.insts.clone(), self.netlist.ports.clone());
        let Graph { vertices, edges, in_edges, out_edges, delay, index_cache, .. } = self;
        // a structure another scene view still holds is that view's to recycle: nothing is taken
        let (Ok(vertices), Ok(edges), Ok(in_edges), Ok(out_edges)) = (Arc::try_unwrap(vertices), Arc::try_unwrap(edges), Arc::try_unwrap(in_edges), Arc::try_unwrap(out_edges)) else { return };
        // a view that let go of its structure (a failed scene-set edit) has nothing to keep
        if vertices.is_empty() {
            return;
        }
        let mut ic = index_cache;
        let spare = Spare { vertices, edges, in_edges, out_edges, delay, index: ic.take(), insts, ports };
        SPARES.with(|p| {
            let mut p = p.borrow_mut();
            if p.len() < MAX_SPARES {
                p.push(spare);
            }
        });
    }

    /// Vertices for every signal pin of every instance and every port, gate edges for every arc
    /// set between two of an instance's vertices, wire edges from each driver to each load of a
    /// net.
    pub fn build(libs: &'a [Library], netlist: &'a Netlist) -> Result<Graph<'a>, String> {
        Graph::build_with_pins(libs, netlist, None)
    }

    /// [`Graph::build`] where an instance's pins are the ones its master HAS (`pins`: master name →
    /// its signal terminals, as the database lists them; a master not listed keeps every liberty
    /// port): a liberty port the master lacks gets no vertex (the reference's graph is made from
    /// the instance's pins). A bidirect pin is two
    /// vertices — a driver (arcs INTO it) and a load (arcs FROM it); one on a net is not modelled.
    pub fn build_with_pins(libs: &'a [Library], netlist: &'a Netlist, pins: Option<&HashMap<String, Vec<String>>>) -> Result<Graph<'a>, String> {
        Graph::build_keeping(libs, netlist, NetlistRef::Borrowed(netlist), pins)
    }

    /// [`Graph::build_with_pins`] with a vertex for every signal pin of an instance whose cell no
    /// library holds (`unlinked`: master name → its signal terminals in order, each with its
    /// direction from the database; `Graph::makePinVertices` makes a vertex for every pin of every
    /// leaf instance, linked or not). Such an instance has no gate edge, only wire edges: an
    /// output is a root driver, an input a load whose capacitance is 0. Its vertices have no
    /// library (`lib: None`) — a top-level port is told apart by its connection, not by that.
    ///
    /// The graph is the reference's WHOLE graph in its bidirect rules too: a bidirect pin or port
    /// is two vertices, its load then its driver (same name); a combinational arc to it reaches
    /// the driver, a check the load, and a bidirect from pin's driver reaches the to pin as well;
    /// a net wires every driver to every load but the same pin (several drivers allowed; ten or
    /// more fanin-free ones into fanout-free loads are an isolated net, not wired); a bidirect
    /// port's load reaches its driver.
    pub fn build_with_unlinked(libs: &'a [Library], netlist: &'a Netlist, pins: Option<&HashMap<String, Vec<String>>>, unlinked: &HashMap<String, Vec<(String, Direction)>>) -> Result<Graph<'a>, String> {
        let g = Graph::build_by_name(libs, netlist, NetlistRef::Borrowed(netlist), pins, Some(unlinked))?;
        if std::env::var_os("VYGES_STA_BUILD_TRACE").is_some() {
            g.trace_build();
        }
        Ok(g)
    }

    /// [`Graph::build_with_pins`] over a netlist the graph SHARES (it may outlive the caller's
    /// generation of it, and be moved to a later one).
    pub fn build_shared(libs: &'a [Library], netlist: std::sync::Arc<Netlist>, pins: Option<&HashMap<String, Vec<String>>>) -> Result<Graph<'a>, String> {
        let read = netlist.clone();
        Graph::build_keeping(libs, &read, NetlistRef::Shared(netlist), pins)
    }

    /// The build over `netlist`, the graph keeping `keep` (the same netlist).
    fn build_keeping(libs: &'a [Library], netlist: &Netlist, keep: NetlistRef<'a>, pins: Option<&HashMap<String, Vec<String>>>) -> Result<Graph<'a>, String> {
        let g = Graph::build_with_pins_untraced(libs, netlist, keep, pins)?;
        // `VYGES_STA_BUILD_TRACE`: every graph built, as the reference's graph-edit tracer prints
        // its build — ids from 1 (the reference's object tables reserve id 0); a diagnostic for
        // small designs
        if std::env::var_os("VYGES_STA_BUILD_TRACE").is_some() {
            g.trace_build();
        }
        Ok(g)
    }

    /// The build trace: each vertex in id order, then each edge in id order.
    fn trace_build(&self) {
        for (i, v) in self.vertices.iter().enumerate() {
            eprintln!("VYGGV|{}|{}|0", i + 1, self.netlist.pin_name(&v.conn));
        }
        for (i, e) in self.edges.iter().enumerate() {
            let wire = u8::from(matches!(e.kind, EdgeKind::Wire));
            eprintln!("VYGGE|{}|{}|{}|{wire}|{}|{}", i + 1, e.from + 1, e.to + 1, self.vname(e.from), self.vname(e.to));
        }
        eprintln!("VYGGBUILT");
    }

    fn build_with_pins_untraced(libs: &'a [Library], netlist: &Netlist, keep: NetlistRef<'a>, pins: Option<&HashMap<String, Vec<String>>>) -> Result<Graph<'a>, String> {
        static CHECK: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        match Graph::build_by_cell(libs, netlist, keep.clone(), pins) {
            // `VYGES_STA_CHECK_BUILD=1`: every template build is checked against the by-name one.
            Some(g) if *CHECK.get_or_init(|| std::env::var_os("VYGES_STA_CHECK_BUILD").is_some()) => {
                let by_name = Graph::build_by_name(libs, netlist, keep, pins, None)?;
                if g.shape() != by_name.shape() {
                    return Err("the template build differs from the by-name build".into());
                }
                // a name index carried from a recycled graph must be the one a fresh build makes
                if let Some(ix) = g.index_cache.get() {
                    if ix.len() != g.vertices.len() || (0..g.vertices.len()).any(|i| ix.get(&g.vname(i)) != Some(&i)) {
                        return Err("the recycled name index differs from a fresh one".into());
                    }
                }
                Ok(g)
            }
            Some(g) => Ok(g),
            None => Graph::build_by_name(libs, netlist, keep, pins, None),
        }
    }

    /// The graph's structure as comparable text: every vertex, edge, edge list, net and delay row
    /// count.
    fn shape(&self) -> String {
        format!("{:?}|{:?}|{:?}|{:?}|{:?}|{:?}", self.vertices, self.edges, self.in_edges, self.out_edges, self.vertex_net, self.delay.iter().map(Vec::len).collect::<Vec<_>>())
    }

    /// [`Graph::build_with_pins`] from one template per cell: the vertices an instance of it makes
    /// and the gate edges between them, laid out from the instance's first vertex — so a net's
    /// pin is found by its instance and port, not by its name. The same graph as
    /// [`Graph::build_by_name`] whenever a pin's name is its alone, which is checked: `None`
    /// (build by name) when an instance or port name repeats, a port or liberty port name holds a
    /// `/` (an instance pin's name could then be another's), or a cell has a bidirect pin.
    fn build_by_cell(libs: &'a [Library], netlist: &Netlist, keep: NetlistRef<'a>, pins: Option<&HashMap<String, Vec<String>>>) -> Option<Graph<'a>> {
        /// One cell's instance layout: per vertex its port, whether it drives, and its delay-row
        /// count's source; the first vertex of each port; the gate edges `(from, to, set, arcs)`.
        struct Template<'l> {
            lib: usize,
            cname: std::sync::Arc<str>,
            verts: Vec<(&'l str, bool)>,
            first: HashMap<&'l str, usize>,
            edges: Vec<(usize, usize, usize, usize)>,
        }
        let mut names = std::collections::HashSet::with_capacity(netlist.insts.len() + netlist.ports.len());
        if netlist.ports.iter().any(|(n, _)| n.contains('/') || !names.insert(n.as_str())) {
            return None;
        }
        names.clear();
        if !netlist.insts.iter().all(|i| names.insert(i.0.as_str())) {
            return None;
        }
        let mut templates: HashMap<&str, Option<Template<'a>>> = HashMap::new();
        let mut inst_tpl: Vec<Option<&str>> = Vec::with_capacity(netlist.insts.len());
        for (_, cname) in netlist.insts.iter().map(|i| &**i) {
            if !templates.contains_key(cname.as_str()) {
                let t = match libs.iter().position(|l| l.cells.contains_key(cname)) {
                    // A physical-only cell (tap, fill, decap) has no liberty cell and nothing to time.
                    None => None,
                    Some(lib) => {
                        let c = cell(libs, lib, cname);
                        let has = pins.and_then(|m| m.get(cname));
                        let mut t = Template { lib, cname: std::sync::Arc::from(cname.as_str()), verts: Vec::new(), first: HashMap::new(), edges: Vec::new() };
                        // The vertex an arc set goes TO and the one it comes FROM: the same for
                        // every pin but a bidirect one, which is refused here.
                        let mut at: HashMap<&str, usize> = HashMap::new();
                        // the master's terminal order (`pins`), not the liberty's: vertex ids follow it
                        for p in instance_ports(c, has) {
                            if has.is_some_and(|h| !h.contains(&p.name)) {
                                continue;
                            }
                            let is_driver = match p.direction {
                                Direction::Input => false,
                                Direction::Output | Direction::Tristate => true,
                                Direction::Bidirect => return None,
                                _ => continue,
                            };
                            if p.name.contains('/') {
                                return None;
                            }
                            let v = t.verts.len();
                            t.first.entry(p.name.as_str()).or_insert(v);
                            at.insert(p.name.as_str(), v);
                            t.verts.push((p.name.as_str(), is_driver));
                        }
                        for (k, set) in c.arc_sets.iter().enumerate() {
                            let tristate = matches!(set.role, Role::TristateEnable | Role::TristateDisable);
                            // `makePortInstanceEdges`: an edge per arc set, the checks timed
                            // nowhere here (`Other`: pulse width, period, non-sequential) included —
                            // they take ids, and a pin's width edge is fanout of it.
                            if set.role != Role::Other && set.arcs.is_empty() && !tristate {
                                continue;
                            }
                            if let (Some(&f), Some(&to)) = (at.get(set.from.as_str()), at.get(set.to.as_str())) {
                                t.edges.push((f, to, k, set.arcs.len()));
                            }
                        }
                        Some(t)
                    }
                };
                templates.insert(cname.as_str(), t);
            }
            inst_tpl.push(templates[cname.as_str()].as_ref().map(|_| cname.as_str()));
        }
        let tpl = |i: usize| inst_tpl[i].and_then(|c| templates[c].as_ref());
        // A recycled graph's allocations (`Graph::recycle`): an instance found there with the same
        // cell and pins takes its vertices as they are (moved, its index updated), every other
        // vertex is made. The result is the graph a fresh build makes, vertex for vertex.
        let mut spare = take_spare();
        let mut old: Vec<Option<Vertex>> = spare.as_mut().map(|s| std::mem::take(&mut s.vertices).into_iter().map(Some).collect()).unwrap_or_default();
        // The spare's instance vertex blocks `(start, len)` in its order, and its port vertices.
        let mut blocks: Vec<(usize, usize)> = Vec::new();
        let mut old_ports: Vec<usize> = Vec::new();
        let mut k = 0;
        while k < old.len() {
            match old[k].as_ref().map(|v| &v.conn) {
                Some(Conn::Inst(oi, _)) => {
                    let (oi, start) = (*oi, k);
                    while k < old.len() && matches!(old[k].as_ref().map(|v| &v.conn), Some(Conn::Inst(o, _)) if *o == oi) {
                        k += 1;
                    }
                    blocks.push((start, k - start));
                }
                _ => {
                    old_ports.push(k);
                    k += 1;
                }
            }
        }
        // The instance a block's vertices belong to: its first vertex's instance in the spare.
        let block_inst = |old: &[Option<Vertex>], (start, _): (usize, usize)| -> Option<&str> {
            match &old[start].as_ref()?.conn {
                Conn::Inst(oi, _) => spare.as_ref().map(|s| s.insts[*oi].0.as_str()),
                Conn::Port(_) => None,
            }
        };
        let mut reused_from: Vec<Option<usize>> = Vec::new();
        let mut cursor = 0usize;
        let mut by_inst: Option<HashMap<String, usize>> = None;
        let mut vertices = Vec::with_capacity(old.len().max(netlist.ports.len() + 4 * netlist.insts.len()));
        let mut base = vec![0usize; netlist.insts.len()];
        for (i, (name, cname)) in netlist.insts.iter().map(|i| &**i).enumerate() {
            let Some(t) = tpl(i) else { continue };
            base[i] = vertices.len();
            // The spare's block for this instance: next in order, else (an instance removed or
            // moved) looked up by name.
            let found = if cursor >= blocks.len() {
                None
            } else if block_inst(&old, blocks[cursor]) == Some(name.as_str()) {
                Some(cursor)
            } else {
                let map = by_inst.get_or_insert_with(|| blocks.iter().enumerate().filter_map(|(b, &blk)| block_inst(&old, blk).map(|n| (n.to_string(), b))).collect());
                map.get(name.as_str()).copied().filter(|&b| old[blocks[b].0].is_some())
            };
            let fits = found.is_some_and(|b| {
                let (start, len) = blocks[b];
                len == t.verts.len()
                    && t.verts.iter().enumerate().all(|(j, &(port, is_driver))| {
                        old[start + j].as_ref().is_some_and(|v| v.port() == Some(port) && v.is_driver == is_driver && v.lib == Some(t.lib) && v.cell.as_deref() == Some(cname.as_str()))
                    })
            });
            if let Some(b) = found {
                cursor = b + 1;
            }
            match found.filter(|_| fits) {
                Some(b) => {
                    let (start, len) = blocks[b];
                    for (k, slot) in old.iter_mut().enumerate().skip(start).take(len) {
                        let mut v = slot.take().expect("a fitting block");
                        if let Conn::Inst(ii, _) = &mut v.conn {
                            *ii = i;
                        }
                        reused_from.push(Some(k));
                        vertices.push(v);
                    }
                }
                None => {
                    for &(port, is_driver) in &t.verts {
                        vertices.push(Vertex { conn: Conn::Inst(i, port.to_string()), is_driver, lib: Some(t.lib), cell: Some(t.cname.clone()) });
                        reused_from.push(None);
                    }
                }
            }
        }
        // Each port's first vertex: a bidirect port is two (`makePinVertices`: its load, then its
        // driver), as in the by-name build.
        let mut port_at: Vec<usize> = Vec::with_capacity(netlist.ports.len());
        let mut pc = 0usize;
        for (i, (name, dir)) in netlist.ports.iter().enumerate() {
            port_at.push(vertices.len());
            let kinds: &[bool] = match dir {
                PortDir::Inout => &[false, true],
                PortDir::Input => &[true],
                _ => &[false],
            };
            for &is_driver in kinds {
                let reuse = old_ports.get(pc).copied().filter(|&k| old[k].as_ref().is_some_and(|v| v.is_driver == is_driver && matches!(v.conn, Conn::Port(oi) if spare.as_ref().is_some_and(|s| s.ports[oi].0 == *name))));
                match reuse {
                    Some(k) => {
                        let mut v = old[k].take().expect("a matching port");
                        v.conn = Conn::Port(i);
                        reused_from.push(Some(k));
                        vertices.push(v);
                        pc += 1;
                    }
                    None => {
                        vertices.push(Vertex { conn: Conn::Port(i), is_driver, lib: None, cell: None });
                        reused_from.push(None);
                    }
                }
            }
        }
        let mut delay_pool: Vec<Vec<[f32; 2]>> = spare.as_mut().map(|s| std::mem::take(&mut s.delay)).unwrap_or_default();
        let mut row = |n: usize| {
            let mut d = delay_pool.pop().unwrap_or_default();
            d.clear();
            d.resize(n, [0.0f32; 2]);
            d
        };
        let mut edges = spare.as_mut().map(|s| std::mem::take(&mut s.edges)).unwrap_or_default();
        edges.clear();
        let mut delay = Vec::new();
        for (i, &b) in base.iter().enumerate() {
            let Some(t) = tpl(i) else { continue };
            for &(f, to, set, arcs) in &t.edges {
                edges.push(Edge { from: b + f, to: b + to, kind: EdgeKind::Gate { set } });
                delay.push(row(arcs));
            }
        }
        // Each net's vertices in its pin order (flat, `net_at[n]..net_at[n + 1]`); one driver
        // each, else build by name.
        let mut vertex_net = vec![None; vertices.len()];
        let mut net_vs: Vec<usize> = Vec::new();
        let mut net_at: Vec<usize> = Vec::with_capacity(netlist.nets.len() + 1);
        for (n, net) in netlist.nets.iter().enumerate() {
            net_at.push(net_vs.len());
            let first = net_vs.len();
            for c in &net.pins {
                match c {
                    Conn::Inst(i, p) => net_vs.extend(tpl(*i).and_then(|t| t.first.get(p.as_str())).map(|&l| base[*i] + l)),
                    // a bidirect port is both: its load and its driver vertex
                    Conn::Port(i) if netlist.ports[*i].1 == PortDir::Inout => net_vs.extend([port_at[*i], port_at[*i] + 1]),
                    Conn::Port(i) => net_vs.push(port_at[*i]),
                }
            }
            if net_vs[first..].iter().filter(|&&v| vertices[v].is_driver).count() > 1 {
                return None;
            }
            for &v in &net_vs[first..] {
                vertex_net[v] = Some(n);
            }
        }
        net_at.push(net_vs.len());
        // `makeWireEdges`: the drivers in pin order — each leaf instance's pins, then the top
        // instance's (the ports) — each to its net's loads in the net's pin order but its own pin;
        // then, at a bidirect port, its load-to-driver path (`makeInstDrvrWireEdges` on the top
        // instance, `setIsBidirectPortPath`).
        let mut bidirect_port_paths = std::collections::HashSet::new();
        for d in (0..vertices.len()).filter(|&v| vertices[v].is_driver) {
            if let Some(n) = vertex_net[d] {
                for &l in net_vs[net_at[n]..net_at[n + 1]].iter().filter(|&&v| !vertices[v].is_driver && vertices[v].conn != vertices[d].conn) {
                    edges.push(Edge { from: d, to: l, kind: EdgeKind::Wire });
                    delay.push(row(2));
                }
            }
            if let Conn::Port(i) = vertices[d].conn {
                if netlist.ports[i].1 == PortDir::Inout {
                    bidirect_port_paths.insert(edges.len());
                    edges.push(Edge { from: d - 1, to: d, kind: EdgeKind::Wire });
                    delay.push(row(2));
                }
            }
        }
        let n = vertices.len();
        let (mut in_edges, mut out_edges) = spare.as_mut().map(|s| (std::mem::take(&mut s.in_edges), std::mem::take(&mut s.out_edges))).unwrap_or_default();
        for lists in [&mut in_edges, &mut out_edges] {
            lists.truncate(n);
            lists.iter_mut().for_each(Vec::clear);
            lists.resize_with(n, Vec::new);
        }
        for (e, ed) in edges.iter().enumerate() {
            in_edges[ed.to].push(e);
            out_edges[ed.from].push(e);
        }
        // The spare's name index, made this graph's: a vertex left behind is removed first (a
        // remade instance's vertices carry the same names), then each moved or made one set.
        let index_cache = std::sync::OnceLock::new();
        if let Some((sp, mut map)) = spare.and_then(|mut s| s.index.take().and_then(|a| std::sync::Arc::try_unwrap(a).ok()).map(|m| (s, m))) {
            for v in old.iter().flatten() {
                map.remove(sp.name(v).as_str());
            }
            for (p, (v, from)) in vertices.iter().zip(&reused_from).enumerate() {
                match from {
                    Some(q) if *q == p => {}
                    _ => {
                        let name = netlist.pin_name(&v.conn);
                        match map.get_mut(name.as_str()) {
                            Some(x) => *x = p,
                            None => {
                                map.insert(name, p);
                            }
                        }
                    }
                }
            }
            if map.len() == n {
                let _ = index_cache.set(std::sync::Arc::new(map));
            }
        }
        Some(Graph { libs, netlist: keep, vertices: Arc::new(vertices), edges: Arc::new(edges), in_edges: Arc::new(in_edges), out_edges: Arc::new(out_edges), vertex_net: Arc::new(vertex_net), slew: vec![[[0.0; 2]; 2]; n], delay, slew_limit: HashMap::new(), clamped: Vec::new(), slew_annotated: HashMap::new(), slew_blocked: None, sdc: SdcEnv::default(), pin_cap_cell: HashMap::new(), ideal_clock: std::collections::HashSet::new(), ideal_clock_slew: HashMap::new(), stale_out_arcs: HashMap::new(), bidirect_inst_paths: Default::default(), bidirect_port_paths: Arc::new(bidirect_port_paths), disabled_loops: Default::default(), loops: Default::default(), index_cache, net_slots: Default::default(), scene_sets: None, array_ops: None, edit_seq: None, state_seq: None, hold_failed: false, thread_count: crate::par::default_thread_count() }.with_loops_broken())
    }

    /// [`Graph::build_with_pins`] with each pin found by its name.
    fn build_by_name(libs: &'a [Library], netlist: &Netlist, keep: NetlistRef<'a>, pins: Option<&HashMap<String, Vec<String>>>, unlinked: Option<&HashMap<String, Vec<(String, Direction)>>>) -> Result<Graph<'a>, String> {
        let find_cell = |name: &str| libs.iter().position(|l| l.cells.contains_key(name));
        let mut vertices = Vec::new();
        let mut index: HashMap<String, usize> = HashMap::new();
        // Per instance: the vertex an arc set goes TO, and the one it comes FROM (they differ only
        // for a bidirect pin).
        let mut inst_vertices: Vec<HashMap<String, usize>> = vec![HashMap::new(); netlist.insts.len()];
        let mut inst_from: Vec<HashMap<String, usize>> = vec![HashMap::new(); netlist.insts.len()];
        let mut bidirect: Vec<String> = Vec::new();
        // each cell's name once, shared by its vertices
        let mut cell_names: HashMap<&str, std::sync::Arc<str>> = HashMap::new();
        // The whole graph (`unlinked` given): a bidirect pin or port is its load vertex, then its
        // driver vertex (`Graph::makePinVertices`), and may sit on a net.
        let whole = unlinked.is_some();
        for (i, (name, cname)) in netlist.insts.iter().map(|i| &**i).enumerate() {
            // A physical-only cell (tap, fill, decap) has no liberty cell and nothing to time.
            let Some(lib) = find_cell(cname) else {
                for (port, dir) in unlinked.and_then(|u| u.get(cname)).map_or(&[][..], Vec::as_slice) {
                    let kinds: &[bool] = match dir {
                        Direction::Input => &[false],
                        Direction::Output | Direction::Tristate => &[true],
                        Direction::Bidirect => &[false, true],
                        _ => return Err(format!("{name}/{port}: a {dir:?} pin of a cell no library holds is not modelled")),
                    };
                    let vname = format!("{name}/{port}");
                    index.entry(vname.clone()).or_insert(vertices.len());
                    for &is_driver in kinds {
                        vertices.push(Vertex { conn: Conn::Inst(i, port.clone()), is_driver, lib: None, cell: Some(cell_names.entry(cname.as_str()).or_insert_with(|| cname.as_str().into()).clone()) });
                    }
                }
                continue;
            };
            let has = pins.and_then(|m| m.get(cname));
            let c = cell(libs, lib, cname);
            // An instance's vertices in the network's pin order (the master's terminals,
            // `makePinVertices` over the instance's pins), not the liberty's — when it is known.
            for p in instance_ports(c, has) {
                if has.is_some_and(|h| !h.contains(&p.name)) {
                    continue;
                }
                let kinds: &[bool] = match p.direction {
                    Direction::Input => &[false],
                    Direction::Output | Direction::Tristate => &[true],
                    Direction::Bidirect if whole => &[false, true],
                    Direction::Bidirect => &[true, false],
                    _ => continue,
                };
                let vname = format!("{name}/{}", p.name);
                if p.direction == Direction::Bidirect {
                    bidirect.push(vname.clone());
                }
                for &is_driver in kinds {
                    let v = vertices.len();
                    index.entry(vname.clone()).or_insert(v);
                    if is_driver || p.direction != Direction::Bidirect {
                        inst_vertices[i].insert(p.name.clone(), v);
                    }
                    if !is_driver || p.direction != Direction::Bidirect {
                        inst_from[i].insert(p.name.clone(), v);
                    }
                    vertices.push(Vertex { conn: Conn::Inst(i, p.name.clone()), is_driver, lib: Some(lib), cell: Some(cell_names.entry(cname.as_str()).or_insert_with(|| cname.as_str().into()).clone()) });
                }
            }
        }
        for net in netlist.nets.iter().filter(|_| !whole) {
            if let Some(b) = net.pins.iter().map(|c| netlist.pin_name(c)).find(|n| bidirect.contains(n)) {
                return Err(format!("{b}: a bidirect pin on a net (net {}) is not modelled", net.name));
            }
        }
        for (i, (name, dir)) in netlist.ports.iter().enumerate() {
            if *dir == PortDir::Inout {
                index.insert(name.clone(), vertices.len());
                for is_driver in [false, true] {
                    vertices.push(Vertex { conn: Conn::Port(i), is_driver, lib: None, cell: None });
                }
                continue;
            }
            let v = vertices.len();
            index.insert(name.clone(), v);
            vertices.push(Vertex { conn: Conn::Port(i), is_driver: *dir == PortDir::Input, lib: None, cell: None });
        }
        // A pin's vertices: its first (the load of a bidirect pin), then a bidirect pin's driver.
        let pin_vertices = |first: usize| -> std::ops::Range<usize> {
            let two = vertices.get(first + 1).is_some_and(|w: &Vertex| w.conn == vertices[first].conn);
            first..first + 1 + usize::from(two)
        };
        let mut edges = Vec::new();
        let mut bidirect_inst_paths = std::collections::HashSet::new();
        for (i, (_, cname)) in netlist.insts.iter().map(|i| &**i).enumerate() {
            let Some(lib) = find_cell(cname) else { continue };
            for (k, set) in cell(libs, lib, cname).arc_sets.iter().enumerate() {
                let tristate = matches!(set.role, Role::TristateEnable | Role::TristateDisable);
                if set.role != Role::Other && set.arcs.is_empty() && !tristate {
                    continue;
                }
                if !whole {
                    if let (Some(&f), Some(&t)) = (inst_from[i].get(&set.from), inst_vertices[i].get(&set.to)) {
                        edges.push(Edge { from: f, to: t, kind: EdgeKind::Gate { set: k } });
                    }
                    continue;
                }
                // `makePortInstanceEdges`: from the from pin's (load) vertex; a combinational arc
                // to a bidirect pin goes to its driver vertex, a check to its load; and a bidirect
                // from pin's driver vertex reaches the to pin's (load) vertex too.
                let (Some(&f), Some(&t)) = (inst_from[i].get(&set.from), inst_from[i].get(&set.to)) else { continue };
                let check = set.is_check_between();
                let to_drvr = pin_vertices(t).nth(1);
                match to_drvr {
                    Some(d) if !check => edges.push(Edge { from: f, to: d, kind: EdgeKind::Gate { set: k } }),
                    _ => edges.push(Edge { from: f, to: t, kind: EdgeKind::Gate { set: k } }),
                }
                if let Some(fd) = pin_vertices(f).nth(1) {
                    bidirect_inst_paths.insert(edges.len());
                    edges.push(Edge { from: fd, to: t, kind: EdgeKind::Gate { set: k } });
                }
            }
        }
        // Each net's vertices in its pin order (flat, `net_at[n]..net_at[n + 1]`); a bidirect pin
        // is both (its driver vertex drives, its load vertex loads).
        let mut vertex_net = vec![None; vertices.len()];
        let mut net_vs: Vec<usize> = Vec::new();
        let mut net_at: Vec<usize> = Vec::with_capacity(netlist.nets.len() + 1);
        let mut buf = String::new();
        for (n, net) in netlist.nets.iter().enumerate() {
            net_at.push(net_vs.len());
            let first = net_vs.len();
            for c in &net.pins {
                if let Some(&v) = index.get(netlist.pin_name_into(c, &mut buf)) {
                    // a bidirect port is both, in every build; an instance's only in the whole one
                    if whole || matches!(c, Conn::Port(i) if netlist.ports[*i].1 == PortDir::Inout) {
                        net_vs.extend(pin_vertices(v));
                    } else {
                        net_vs.push(v);
                    }
                }
            }
            if !whole && net_vs[first..].iter().filter(|&&v| vertices[v].is_driver).count() > 1 {
                return Err(format!("net {}: multiple drivers are not modelled", net.name));
            }
            for &v in &net_vs[first..] {
                vertex_net[v] = Some(n);
            }
        }
        net_at.push(net_vs.len());
        // `makeWireEdges`: `makeInstDrvrWireEdges` over each leaf instance, then the top instance
        // — a pin at a time, in pin order (vertex order): a driver not yet reached makes its whole
        // net's wire edges (`makeWireEdgesFromPin`; the net's drivers are then reached), and a
        // bidirect PORT then its load-to-driver path.
        let mut net_done = vec![false; netlist.nets.len()];
        let mut bidirect_port_paths = std::collections::HashSet::new();
        for v in 0..vertices.len() {
            // a pin's first vertex (the load of a bidirect pin) stands for the pin
            if v > 0 && vertices[v - 1].conn == vertices[v].conn {
                continue;
            }
            let net = pin_vertices(v).find(|&d| vertices[d].is_driver).and_then(|d| vertex_net[d]);
            if let Some(n) = net.filter(|&n| !net_done[n]) {
                net_done[n] = true;
                let vs = &net_vs[net_at[n]..net_at[n + 1]];
                if !whole {
                    for &d in vs.iter().filter(|&&x| vertices[x].is_driver) {
                        for &l in vs.iter().filter(|&&x| !vertices[x].is_driver && vertices[x].conn != vertices[d].conn) {
                            edges.push(Edge { from: d, to: l, kind: EdgeKind::Wire });
                        }
                    }
                } else {
                    let drivers: Vec<usize> = vs.iter().copied().filter(|&x| vertices[x].is_driver).collect();
                    let loads: Vec<usize> = vs.iter().copied().filter(|&x| !vertices[x].is_driver).collect();
                    // `isIsolatedNet`: ten or more drivers, none a port or with fanin, and loads
                    // with no fanout or check: a pad ring's tied pins, given no edge.
                    let fanin = |x: usize| edges.iter().any(|e: &Edge| e.to == x);
                    let is_check = |e: &Edge| match e.kind {
                        EdgeKind::Gate { set } => {
                            let x = &vertices[e.to];
                            // `hasChecks`: set by a check BETWEEN pins only (not a width or period)
                            x.lib.is_some_and(|l| cell(libs, l, x.cell.as_deref().unwrap_or("")).arc_sets[set].is_check_between())
                        }
                        EdgeKind::Wire => false,
                    };
                    let fanout_or_check = |x: usize| edges.iter().any(|e: &Edge| e.from == x || (e.to == x && is_check(e)));
                    let isolated = drivers.len() >= 10 && drivers.iter().all(|&d| !matches!(vertices[d].conn, Conn::Port(_)) && !fanin(d)) && loads.iter().all(|&l| !fanout_or_check(l));
                    // `isPassiveBidirectDrvr` (OpenSTA fc6ed303): an instance pin's bidirect
                    // driver vertex with no fanin (a pad with no arc into it) drives nothing on a
                    // net that has an active driver. (Wire edges only reach loads, so a driver's
                    // fanin is the same before and after.)
                    let passive = |d: usize| {
                        let bidirect = d > 0 && vertices[d - 1].conn == vertices[d].conn;
                        bidirect && !matches!(vertices[d].conn, Conn::Port(_)) && !fanin(d)
                    };
                    let passives: Vec<bool> = if isolated { Vec::new() } else { drivers.iter().map(|&d| passive(d)).collect() };
                    let has_active_drvr = passives.iter().any(|&p| !p);
                    for (&d, &is_passive) in drivers.iter().zip(&passives) {
                        if has_active_drvr && is_passive {
                            continue;
                        }
                        for &l in &loads {
                            if vertices[l].conn != vertices[d].conn {
                                edges.push(Edge { from: d, to: l, kind: EdgeKind::Wire });
                            }
                        }
                    }
                }
            }
            // `makeInstDrvrWireEdges` on the top instance: a bidirect port's load vertex reaches
            // its driver vertex (a bidirect port path).
            if let Conn::Port(i) = vertices[v].conn {
                if netlist.ports[i].1 == PortDir::Inout {
                    bidirect_port_paths.insert(edges.len());
                    edges.push(Edge { from: v, to: v + 1, kind: EdgeKind::Wire });
                }
            }
        }
        let mut in_edges = vec![Vec::new(); vertices.len()];
        let mut out_edges = vec![Vec::new(); vertices.len()];
        for (e, ed) in edges.iter().enumerate() {
            in_edges[ed.to].push(e);
            out_edges[ed.from].push(e);
        }
        let n = vertices.len();
        let delay = edges
            .iter()
            .map(|e| match e.kind {
                EdgeKind::Gate { set } => {
                    let v = &vertices[e.to];
                    vec![[0.0f32; 2]; cell(libs, v.lib.unwrap(), v.cell.as_deref().unwrap()).arc_sets[set].arcs.len()]
                }
                EdgeKind::Wire => vec![[0.0f32; 2]; 2],
            })
            .collect();
        Ok(Graph { libs, netlist: keep, vertices: Arc::new(vertices), edges: Arc::new(edges), in_edges: Arc::new(in_edges), out_edges: Arc::new(out_edges), vertex_net: Arc::new(vertex_net), slew: vec![[[0.0; 2]; 2]; n], delay, slew_limit: HashMap::new(), clamped: Vec::new(), slew_annotated: HashMap::new(), slew_blocked: None, sdc: SdcEnv::default(), pin_cap_cell: HashMap::new(), ideal_clock: std::collections::HashSet::new(), ideal_clock_slew: HashMap::new(), stale_out_arcs: HashMap::new(), bidirect_inst_paths: Arc::new(bidirect_inst_paths), bidirect_port_paths: Arc::new(bidirect_port_paths), disabled_loops: Default::default(), loops: Default::default(), index_cache: std::sync::OnceLock::new(), net_slots: Default::default(), scene_sets: None, array_ops: None, edit_seq: None, state_seq: None, hold_failed: false, thread_count: crate::par::default_thread_count() }.with_loops_broken())
    }

    /// Whether vertex `v` is a top-level port's (an instance's vertex may have no library too:
    /// [`Graph::build_with_unlinked`]).
    pub fn is_port(&self, v: usize) -> bool {
        matches!(self.vertices[v].conn, Conn::Port(_))
    }

    /// `TimingRole::isTimingCheck`: every check — the ones timed here (setup, hold, recovery,
    /// removal) and the ones only carried (`Other`: width, period, non-sequential, …), which no
    /// search, levelization or delay calculation passes through.
    pub fn is_check(&self, e: usize) -> bool {
        match self.edges[e].kind {
            EdgeKind::Gate { set } => self.arc_set(e, set).role.is_timing_check(),
            EdgeKind::Wire => false,
        }
    }

    /// A check timed here: setup, hold, recovery or removal — what a check END is made of
    /// (`visitCheckEnd`); the `Other` checks carry no arcs, so end no path.
    pub fn is_timed_check(&self, e: usize) -> bool {
        match self.edges[e].kind {
            EdgeKind::Gate { set } => matches!(self.arc_set(e, set).role, Role::Setup | Role::Hold | Role::Recovery | Role::Removal),
            EdgeKind::Wire => false,
        }
    }

    /// The arc set of a gate edge as the graph holds it: an output-to-output edge of an instance
    /// replaced by an equivalent-arcs cell keeps its OLD cell's set ([`Graph::stale_out_arcs`]).
    /// Borrowed from the libraries (`'a`), not from the graph: a caller editing delays and slews
    /// holds the arcs without copying them.
    pub fn arc_set(&self, e: usize, set: usize) -> &'a crate::liberty::ArcSet {
        let v = &self.vertices[self.edges[e].to];
        if !self.stale_out_arcs.is_empty() {
            let from = &self.vertices[self.edges[e].from];
            if let (Conn::Inst(fi, _), Conn::Inst(ti, _)) = (&from.conn, &v.conn) {
                if fi == ti && from.is_driver {
                    if let Some((lib, name)) = self.stale_out_arcs.get(&self.netlist.insts[*ti].0) {
                        return &cell(self.libs, *lib, name).arc_sets[self.scene_set(*lib, name, set)];
                    }
                }
            }
        }
        let (lib, name) = (v.lib.unwrap(), v.cell.as_deref().unwrap());
        &cell(self.libs, lib, name).arc_sets[self.scene_set(lib, name, set)]
    }

    /// The index in THIS graph's cell `name` (library `lib`) of the arc set an edge carries as
    /// `set`: the same index, but in a scene view whose cell lists its sets in another order than
    /// the link cell (`LibertyLibrary::makeSceneMap`, [`crate::scenes`]).
    pub fn scene_set(&self, lib: usize, name: &str, set: usize) -> usize {
        match &self.scene_sets {
            Some(m) => m.get(lib, name, set),
            None => set,
        }
    }

    /// A view of this graph's structure for another scene ([`crate::scenes::scene_views`]): the
    /// structure, netlist and name index SHARED (no copy), the scene's libraries and arc-set map, and
    /// per-scene arrays as a fresh build leaves them (slews 0, each edge's delay row of its arcs, 0).
    pub(crate) fn scene_view(&self, libs: &'a [Library], scene_sets: Option<Arc<crate::scenes::SceneSets>>) -> Graph<'a> {
        let index_cache = std::sync::OnceLock::new();
        if let Some(ix) = self.index_cache.get() {
            let _ = index_cache.set(ix.clone());
        }
        Graph {
            libs,
            netlist: self.netlist.clone(),
            vertices: self.vertices.clone(),
            edges: self.edges.clone(),
            in_edges: self.in_edges.clone(),
            out_edges: self.out_edges.clone(),
            vertex_net: self.vertex_net.clone(),
            slew: vec![[[0.0; 2]; 2]; self.vertices.len()],
            delay: self.delay.iter().map(|r| vec![[0.0; 2]; r.len()]).collect(),
            slew_limit: HashMap::new(),
            clamped: Vec::new(),
            thread_count: self.thread_count,
            slew_annotated: HashMap::new(),
            slew_blocked: None,
            sdc: SdcEnv::default(),
            pin_cap_cell: HashMap::new(),
            ideal_clock: std::collections::HashSet::new(),
            ideal_clock_slew: HashMap::new(),
            stale_out_arcs: HashMap::new(),
            bidirect_inst_paths: self.bidirect_inst_paths.clone(),
            bidirect_port_paths: self.bidirect_port_paths.clone(),
            disabled_loops: self.disabled_loops.clone(),
            loops: self.loops.clone(),
            index_cache,
            net_slots: self.net_slots.clone(),
            scene_sets,
            array_ops: None,
            edit_seq: None,
            state_seq: None,
            hold_failed: false,
        }
    }

    /// This view lets go of the shared structure, netlist and name index (a scene set edits them
    /// through one view, which must hold them alone); [`Graph::reattach`] takes them back.
    pub(crate) fn detach(&mut self) {
        static NONE: std::sync::OnceLock<Netlist> = std::sync::OnceLock::new();
        self.vertices = Arc::default();
        self.edges = Arc::default();
        self.in_edges = Arc::default();
        self.out_edges = Arc::default();
        self.vertex_net = Arc::default();
        self.bidirect_inst_paths = Arc::default();
        self.bidirect_port_paths = Arc::default();
        self.disabled_loops = Arc::default();
        self.loops = Arc::default();
        self.net_slots = Arc::default();
        self.index_cache = std::sync::OnceLock::new();
        self.netlist = NetlistRef::Borrowed(NONE.get_or_init(Netlist::default));
    }

    /// The structure, netlist and name index of `from` (the view that edited them), shared again.
    pub(crate) fn reattach(&mut self, from: &Graph<'a>) {
        self.netlist = from.netlist.clone();
        self.vertices = from.vertices.clone();
        self.edges = from.edges.clone();
        self.in_edges = from.in_edges.clone();
        self.out_edges = from.out_edges.clone();
        self.vertex_net = from.vertex_net.clone();
        self.bidirect_inst_paths = from.bidirect_inst_paths.clone();
        self.bidirect_port_paths = from.bidirect_port_paths.clone();
        self.disabled_loops = from.disabled_loops.clone();
        self.loops = from.loops.clone();
        self.net_slots = from.net_slots.clone();
        self.index_cache = std::sync::OnceLock::new();
        if let Some(ix) = from.index_cache.get() {
            let _ = self.index_cache.set(ix.clone());
        }
    }

    /// [`Graph::stale_out_arcs`] from instance → cell name, each cell found in the graph's libraries
    /// (the first holding it); an instance whose cell is in none is left with its own.
    pub fn set_stale_out_arcs(&mut self, cells: &std::collections::BTreeMap<String, String>) {
        self.stale_out_arcs = cells.iter().filter_map(|(inst, name)| self.libs.iter().position(|l| l.cells.contains_key(name)).map(|lib| (inst.clone(), (lib, name.clone())))).collect();
    }

    /// The build's loop breaking ([`Graph::break_loops`]), for the builders' struct literals.
    fn with_loops_broken(mut self) -> Self {
        self.break_loops();
        self
    }

    /// `Levelize::ensureBackEdges` over the whole graph, from scratch: the edges closing loops
    /// are found ([`crate::levelize::break_loops`]) and disabled. Made at the build — the
    /// reference levelizes once and keeps the disabled edges until a loop edge is deleted.
    pub fn break_loops(&mut self) {
        let lb = crate::levelize::break_loops(&LevelizeView(self));
        self.disabled_loops = Arc::new(lb.disabled.iter().copied().collect());
        self.loops = Arc::new(lb);
    }

    /// `Levelize::isDisabledLoop`.
    pub fn is_disabled_loop(&self, e: usize) -> bool {
        self.disabled_loops.contains(&e)
    }

    /// `Levelize::findLevels` on an acyclic graph: a root — no searched-through edge into it —
    /// is level 0; every other vertex is one more than the highest level among the vertices
    /// that reach it through a searched-through edge (`Levelize::searchThru`: every edge but
    /// timing checks, latch D->Q, and the register set/clear arcs the reference leaves disabled
    /// by default). ⚠️ Arc sets of role `Other` have no edge here (tristate enable/disable among
    /// them, which the reference DOES search through) — a caller that needs exact levels refuses
    /// cells carrying one. A vertex no
    /// root reaches is 0. Levels are in steps of 1 (the reference's are a constant multiple,
    /// which orders the same).
    pub fn levels(&self) -> Result<Vec<i32>, String> {
        let thru = |e: usize| self.levelize_thru(e);
        let order = self.topo_order()?;
        let mut level = vec![-1i32; self.vertices.len()];
        for (v, l) in level.iter_mut().enumerate() {
            if !self.in_edges[v].iter().any(|&e| thru(e)) {
                *l = 0;
            }
        }
        for &v in &order {
            if level[v] == -1 {
                continue;
            }
            for &e in &self.out_edges[v] {
                if thru(e) {
                    let to = self.edges[e].to;
                    level[to] = level[to].max(level[v] + 1);
                }
            }
        }
        Ok(level.into_iter().map(|l| l.max(0)).collect())
    }

    /// `Levelize::searchThru`: the edges levelization follows — every edge but timing checks,
    /// latch D->Q, the register set/clear arcs the reference leaves disabled by default and the
    /// bidirect instance/port paths. A latch D->Q edge is left out so a loop through a latch
    /// (a clock-gate latch whose D is fed from the clock it gates) levelizes; its Q is timed by
    /// the search's latch passes ([`crate::search::Search::find_arrivals`]).
    pub fn levelize_thru(&self, e: usize) -> bool {
        self.levelize_thru_unlooped(e) && !self.disabled_loops.contains(&e)
    }

    /// [`Graph::levelize_thru`] without its disabled-loop term — what loop breaking itself asks
    /// ([`crate::levelize::LoopGraph::search_thru`]).
    pub fn levelize_thru_unlooped(&self, e: usize) -> bool {
        match self.edges[e].kind {
            EdgeKind::Gate { set } => !self.is_check(e) && !matches!(self.arc_set(e, set).role, Role::RegSetClr | Role::LatchDtoQ) && !self.bidirect_inst_paths.contains(&e),
            EdgeKind::Wire => !self.bidirect_port_paths.contains(&e),
        }
    }

    /// Topological order over the edges levelization follows ([`Graph::levelize_thru`]).
    pub fn topo_order(&self) -> Result<Vec<usize>, String> {
        let n = self.vertices.len();
        let mut indeg = vec![0usize; n];
        for (e, ed) in self.edges.iter().enumerate() {
            if self.levelize_thru(e) {
                indeg[ed.to] += 1;
            }
        }
        let mut stack: Vec<usize> = (0..n).rev().filter(|&v| indeg[v] == 0).collect();
        let mut order = Vec::with_capacity(n);
        while let Some(v) = stack.pop() {
            order.push(v);
            for &e in &self.out_edges[v] {
                if !self.levelize_thru(e) {
                    continue;
                }
                let t = self.edges[e].to;
                indeg[t] -= 1;
                if indeg[t] == 0 {
                    stack.push(t);
                }
            }
        }
        if order.len() != n {
            return Err(format!("the timing graph has a loop — not modelled: {}", self.loop_names(&indeg).join(" -> ")));
        }
        Ok(order)
    }

    /// One loop among the vertices `topo_order` could not order (`indeg` > 0), for the refusal:
    /// walk back along a propagating in-edge from another unordered vertex until a vertex
    /// repeats; each is named with its cell.
    fn loop_names(&self, indeg: &[usize]) -> Vec<String> {
        let mut in_edges = vec![Vec::new(); self.vertices.len()];
        for (e, ed) in self.edges.iter().enumerate() {
            if self.levelize_thru(e) && indeg[ed.from] > 0 && indeg[ed.to] > 0 {
                in_edges[ed.to].push(ed.from);
            }
        }
        let Some(mut v) = (0..indeg.len()).find(|&v| !in_edges[v].is_empty()) else { return Vec::new() };
        let mut seen = vec![usize::MAX; indeg.len()];
        let mut path = Vec::new();
        while seen[v] == usize::MAX {
            seen[v] = path.len();
            path.push(v);
            v = in_edges[v][0];
        }
        let mut cycle: Vec<usize> = path[seen[v]..].to_vec();
        cycle.reverse();
        cycle.iter().map(|&v| {
            let x = &self.vertices[v];
            match &x.cell { Some(c) => format!("{} ({c})", self.netlist.pin_name(&x.conn)), None => self.netlist.pin_name(&x.conn) }
        }).collect()
    }

    fn lib_thresholds(&self, lib: usize, rf: usize) -> LibThresholds {
        let l = &self.libs[lib];
        LibThresholds { output: l.output_threshold[rf], input: l.input_threshold[rf], lower: l.slew_lower_threshold[rf], upper: l.slew_upper_threshold[rf], derate: l.slew_derate }
    }

    /// `GraphDelayCalc::findInputDriverDelay` for an input port with a driving cell: per min/max
    /// and port transition, each of the cell's arcs from `from_port` into `to_port` ending in that
    /// transition, at the slew given for its from-transition (`findInputArcDelay`): the gate at
    /// the port's load (`parasiticLoad`: DMP on its pi, else lumped), and at no load (the
    /// intrinsic delay); the port's slew is SET to the gate's, and each load's slew and wire delay
    /// are SET from the gate's load waveforms, the wire delay plus the load-dependent part of the
    /// gate delay (`gate − intrinsic`, in float). A later arc of the same transition overwrites.
    fn time_input_drive(&self, o: &mut Overlay, wires: &[usize], drive: &InputDrive, parasitics: &HashMap<String, NetParasitics>, index: &HashMap<String, usize>) -> Result<(), String> {
        let v = o.v;
        let li = self.libs.iter().position(|l| l.cells.contains_key(&drive.cell)).ok_or_else(|| format!("set_driving_cell: cell {} not found", drive.cell))?;
        let cell = &self.libs[li].cells[&drive.cell];
        let from_port = match &drive.from_port {
            Some(f) => f.clone(),
            None => {
                let pos = |p: &str| cell.ports.iter().position(|x| x.name == p).unwrap_or(usize::MAX);
                cell.arc_sets.iter().filter(|s| s.to == drive.to_port).min_by_key(|s| pos(&s.from)).map(|s| s.from.clone()).ok_or_else(|| format!("set_driving_cell: no arc into {}/{}", drive.cell, drive.to_port))?
            }
        };
        let arcs: Vec<crate::liberty::Arc> = cell.arc_sets.iter().filter(|s| s.from == from_port && s.to == drive.to_port && !s.role.is_timing_check()).flat_map(|s| s.arcs.iter().cloned()).collect();
        for mm in [MIN, MAX] {
            for rf in [RISE, FALL] {
                let (pin_cap, wire_cap, pe) = self.parasitic_load(v, rf, mm, parasitics, index);
                let elmores = LoadElmores::new(pe.as_ref());
                let load_cap = pin_cap + wire_cap;
                for arc in arcs.iter().filter(|a| a.to_rf == rf) {
                    let Model::Gate(model) = &arc.model else { continue };
                    let from_slew = drive.from_slews[arc.from_rf];
                    let (intrinsic, _) = model.gate_delay(from_slew, 0.0);
                    let l = &self.libs[li];
                    let th = Thresholds { vth: l.output_threshold[rf], vl: l.slew_lower_threshold[rf], vh: l.slew_upper_threshold[rf], slew_derate: l.slew_derate };
                    let (gate_delay, drvr_slew, mut dmp) = match &pe {
                        Some((p, _)) => {
                            let mut d = Dmp::new(model, &th, from_slew, p.c2, p.rpi, p.c1);
                            let (gd, ds) = d.gate_delay_slew();
                            (gd, ds, Some(d))
                        }
                        None => {
                            let (gd, ds) = model.gate_delay(from_slew, load_cap);
                            (f64::from(gd), f64::from(ds), None)
                        }
                    };
                    o.sv[rf][mm] = drvr_slew as f32;
                    let load_delay = gate_delay as f32 - intrinsic;
                    for (i, &w) in wires.iter().enumerate() {
                        let load = self.edges[w].to;
                        let (mut wire_delay, mut load_slew) = match (&mut dmp, &pe) {
                            (Some(d), Some((p, names))) => match elmores.get(p, names, &self.vname(load)) {
                                Some(el) => d.load_delay_slew(f64::from(el)),
                                None => (0.0, drvr_slew),
                            },
                            _ => (0.0, drvr_slew),
                        };
                        let ll = self.threshold_library(load);
                        threshold_adjust(ll == li, &self.lib_thresholds(li, rf), &self.lib_thresholds(ll, rf), rf == RISE, &mut wire_delay, &mut load_slew);
                        if !slew_blocked_by(&self.slew_blocked, &self.slew_annotated, load, mm) {
                            o.ls[o.slot[i]][rf][mm] = load_slew as f32;
                        } else if let (Some(&a), MAX) = (self.slew_annotated.get(&load), mm) {
                            o.ls[o.slot[i]][rf][mm] = a;
                        }
                        o.wire_delay[i][rf][mm] = wire_delay as f32 + load_delay;
                    }
                }
            }
        }
        Ok(())
    }

    /// The library whose thresholds a load uses: a port's is the default (first read) library.
    fn threshold_library(&self, v: usize) -> usize {
        self.vertices[v].lib.unwrap_or(0)
    }

    /// A pin's capacitance as `Sdc::pinCaps` and `ReduceToPi::pinCapacitance` read it: an
    /// instance pin's liberty port capacitance; a top-level port's `set_load -pin_load` (0 without).
    fn pin_cap(&self, v: usize, rf: usize, mm: usize) -> f32 {
        // A cell swapped in the database the timer was not told about: its pins' capacitances are
        // read through the network (the new master's), its arcs stay the graph's.
        if let (Some((l, c)), Some(p)) = (self.pin_cap_cell.get(&v), self.vertices[v].port()) {
            return cell(self.libs, *l, c).port(p).map_or(0.0, |p| p.capacitance(rf, mm));
        }
        match (&self.vertices[v].lib, &self.vertices[v].cell, self.vertices[v].port()) {
            (Some(l), Some(c), Some(p)) => cell(self.libs, *l, c).port(p).map_or(0.0, |p| p.capacitance(rf, mm)),
            _ => match self.vertices[v].conn {
                Conn::Port(_) => self.sdc_at(&self.sdc.port_pin_cap, v).and_then(|c| c[rf][mm]).unwrap_or(0.0),
                Conn::Inst(..) => 0.0,
            },
        }
    }

    /// `Sdc::connectedCap(drvr, rf, scene, min_max)`: the net's pin cap, then the driver's net
    /// `set_load` — which zeroes the pin cap with `-subtract_pin_load` and adds its wire cap.
    /// `(pin_cap, wire_cap, has_net_load)`.
    fn connected_cap(&self, drvr: usize, rf: usize, mm: usize, index: &HashMap<String, usize>) -> (f32, f32, bool) {
        let mut pin_cap = self.net_pin_cap(drvr, rf, mm, index);
        let mut wire_cap = 0.0f32;
        let net_load = self.sdc_at(&self.sdc.net_wire_cap, drvr).and_then(|c| c[mm]);
        if let Some((cap, subtract_pin_cap)) = net_load {
            if subtract_pin_cap {
                pin_cap = 0.0;
            }
            wire_cap += cap;
        }
        (pin_cap, wire_cap, net_load.is_some())
    }

    /// `GraphDelayCalc::parasiticLoad(drvr, rf, scene, min_max)`: the connected cap, and the pi
    /// model `findParasitic` gives — none when the driver pin has a net `set_load` at all
    /// (`drvrPinHasWireCap`: set_load net has precedence over parasitics; the gate is lumped at
    /// the SDC's load). Otherwise a pi model at least the pin cap adds `pi − pin`; a smaller one is
    /// dropped. `(pin_cap, wire_cap, the pi model)`.
    fn parasitic_load<'p>(&self, drvr: usize, rf: usize, mm: usize, parasitics: &'p HashMap<String, NetParasitics>, index: &HashMap<String, usize>) -> ParasiticLoad<'p> {
        let (pin_cap, mut wire_cap, has_net_load) = self.connected_cap(drvr, rf, mm, index);
        let mut parasitic = if self.sdc_at(&self.sdc.net_wire_cap, drvr).is_some() { None } else { self.reduced(drvr, rf, mm, parasitics, index) };
        if !has_net_load {
            if let Some((p, _)) = &parasitic {
                let parasitic_cap = p.c1 + p.c2;
                if parasitic_cap >= pin_cap {
                    wire_cap = parasitic_cap - pin_cap;
                } else {
                    wire_cap = 0.0;
                    parasitic = None;
                }
            }
        }
        (pin_cap, wire_cap, parasitic)
    }

    /// The net's pin capacitance: every pin on the driver's net, in the net's order, summed in `f32`.
    fn net_pin_cap(&self, drvr: usize, rf: usize, mm: usize, index: &HashMap<String, usize>) -> f32 {
        let Some(n) = self.vertex_net[drvr] else { return self.pin_cap(drvr, rf, mm) };
        let mut sum = 0.0f32;
        let mut buf = String::new();
        for c in &self.netlist.nets[n].pins {
            if let Some(&v) = index.get(self.netlist.pin_name_into(c, &mut buf)) {
                sum += self.pin_cap(v, rf, mm);
            }
        }
        sum
    }

    /// The pi-Elmore model of a driver's net, for a transition and min/max — the one delay
    /// calculation and [`Graph::load_cap`] both read.
    fn reduced<'p>(&self, drvr: usize, rf: usize, mm: usize, parasitics: &'p HashMap<String, NetParasitics>, index: &HashMap<String, usize>) -> Option<(crate::parasitics::PiElmore, &'p [String])> {
        let n = self.vertex_net[drvr]?;
        let np = parasitics.get(&self.netlist.nets[n].name)?;
        let d = np.node_names.iter().position(|x| self.vname_is(drvr, x))?;
        let names = &np.node_names;
        if let Some(p) = &np.pi {
            if np.pi_rise_only && rf == FALL {
                return None;
            }
            return Some((p.clone(), names));
        }
        let cap = |i: usize| {
            index.get(&names[i]).map_or(0.0, |&v| match (&np.port_pin_caps, &self.vertices[v].conn) {
                (Some(m), Conn::Port(_)) => m.get(&names[i]).and_then(|c| c[rf][mm]).unwrap_or(0.0),
                _ => self.pin_cap(v, rf, mm),
            })
        };
        let load = |i: usize| index.get(&names[i]).is_some_and(|&v| !self.vertices[v].is_driver);
        Some((reduce_to_pi_elmore(&np.network, d, &NodePins { pin_cap: &cap, is_load: &load }), names))
    }

    /// `ArcDelayCalc::findParasitic(drvr_pin, rf, scene, min_max)`: the driver net's reduced pi model
    /// `(c2, rpi, c1)` as the delay calculator finds it — no `parasiticLoad` rule applied (a net
    /// `set_load`, a pi model under the pin cap): `None` without parasitics for the net.
    pub fn reduced_pi(&self, drvr: usize, rf: usize, mm: usize, parasitics: &HashMap<String, NetParasitics>) -> Option<(f32, f32, f32)> {
        self.reduced(drvr, rf, mm, parasitics, self.index()).map(|(p, _)| (p.c2, p.rpi, p.c1))
    }

    /// `GraphDelayCalc::loadCap(drvr_pin, rf, scene, max, pin_cap, wire_cap)` for one transition,
    /// and whether the driver has a reduced pi model at all (`findPiElmore`).
    pub fn load_cap_parts(&self, drvr: usize, parasitics: &HashMap<String, NetParasitics>, rf: usize) -> (f32, f32, bool) {
        let index = self.index();
        let (pin_cap, wire_cap, _) = self.parasitic_load(drvr, rf, MAX, parasitics, index);
        (pin_cap, wire_cap, self.reduced(drvr, rf, MAX, parasitics, index).is_some())
    }

    /// `GraphDelayCalc::loadCap(drvr_pin, scene, max)`: over rise then fall, the larger of
    /// `pin_cap + wire_cap` ([`Graph::parasitic_load`]) — the pin capacitance of the driver's net
    /// (`connectedCap`, with the SDC's loads) plus, when the reduced pi model's total is at least
    /// that, the total MINUS the pin cap (so the sum is `pin + (pi − pin)` in float, not the pi
    /// total); a smaller pi model is ignored (wire cap 0); a net `set_load` replaces the pi model.
    pub fn load_cap(&self, drvr: usize, parasitics: &HashMap<String, NetParasitics>) -> f32 {
        self.load_cap_mm(drvr, parasitics, MAX)
    }

    /// `GraphDelayCalc::loadCap(drvr_pin, scene, min_max)`: over rise then fall, the larger
    /// load at max, the smaller at min (`MinMax::minMax` from its init value) — each one
    /// transition's [`Graph::load_cap_rf`] at that min/max.
    pub fn load_cap_mm(&self, drvr: usize, parasitics: &HashMap<String, NetParasitics>, mm: usize) -> f32 {
        let index = self.index();
        let mut load_cap = if mm == MAX { -1e30f32 } else { 1e30f32 };
        for rf in [RISE, FALL] {
            let (pin_cap, wire_cap, _) = self.parasitic_load(drvr, rf, mm, parasitics, index);
            let cap = pin_cap + wire_cap;
            if (mm == MAX && cap > load_cap) || (mm != MAX && cap < load_cap) {
                load_cap = cap;
            }
        }
        load_cap
    }

    /// `GraphDelayCalc::loadCap(drvr_pin, rf, scene, min_max)`: `pin_cap + wire_cap` of one
    /// transition at one min/max ([`Graph::load_cap`]'s per-transition term).
    pub fn load_cap_rf(&self, drvr: usize, parasitics: &HashMap<String, NetParasitics>, rf: usize, mm: usize) -> f32 {
        let (pin_cap, wire_cap, _) = self.parasitic_load(drvr, rf, mm, parasitics, self.index());
        pin_cap + wire_cap
    }

    /// Delay calculation over the whole graph. With `trace`, one line per DMP gate call
    /// (`dcalc|gate|drvr|rf pair|in_slew|c2|rpi|c1|alg|rd|t0|dt|ceff|valid|delay|slew`, bits in hex)
    /// and per load (`dcalc|load|drvr|load|wire delay|load slew`).
    pub fn find_delays(&mut self, parasitics: &HashMap<String, NetParasitics>, mut trace: Option<&mut Vec<String>>) -> Result<(), String> {
        let _time = crate::par::StageTimer::start("find_delays", self.thread_count);
        let order = self.topo_order()?;
        self.clamped.clear();
        // the shared map, not a copy of it
        let index = std::sync::Arc::clone(self.shared_index());
        let reached = self.dcalc_reached(&order);
        if self.thread_count <= 1 {
            for &v in &order {
                if let Some(d) = self.delay_visit(v, reached[v], parasitics, &index, trace.is_some())? {
                    self.store_vertex_delays(v, d, trace.as_deref_mut());
                }
            }
        } else {
            self.find_delays_parallel(&order, &reached, parasitics, &index, trace)?;
        }
        // Timing-check delays, after every slew is known.
        for e in 0..self.edges.len() {
            if self.is_check(e) {
                self.find_check_edge_delays(e);
            }
        }
        // `invalid_latch_edges_`: every latch D -> Q edge, after the checks.
        for e in 0..self.edges.len() {
            if matches!(self.edges[e].kind, EdgeKind::Gate { set } if self.arc_set(e, set).role == Role::LatchDtoQ) {
                self.find_latch_edge_delays(e, parasitics, &index);
            }
        }
        Ok(())
    }

    /// The delay pass on [`Graph::thread_count`] threads ([`crate::par`]): the topological order
    /// cut into hazard waves — a visit reads the slews of its fanin's from-vertices and writes its
    /// own slew and its loads' — each wave computed on the threads, then stored in topological
    /// order. The trace lines and `clamped` come out in topological position, as the serial pass
    /// emits them. Equal to the serial pass bit for bit.
    fn find_delays_parallel(&mut self, order: &[usize], reached: &[bool], parasitics: &HashMap<String, NetParasitics>, index: &HashMap<String, usize>, trace: Option<&mut Vec<String>>) -> Result<(), String> {
        let threads = self.thread_count;
        let want_trace = trace.is_some();
        let waves = {
            let g = &*self;
            crate::par::waves(order, g.vertices.len(), |v, out| {
                for &e in g.in_edges[v].iter() {
                    if !g.is_check(e) {
                        out.push(crate::par::Access::Read(g.edges[e].from));
                    }
                }
                out.push(crate::par::Access::Write(v));
                for &e in g.out_edges[v].iter() {
                    if matches!(g.edges[e].kind, EdgeKind::Wire) {
                        out.push(crate::par::Access::Write(g.edges[e].to));
                    }
                }
            })
        };
        let mut pos = vec![0usize; self.vertices.len()];
        for (i, &v) in order.iter().enumerate() {
            pos[v] = i;
        }
        let mut side: Vec<(usize, Vec<String>, bool)> = Vec::new();
        for wave in &waves {
            let results = {
                let g = &*self;
                crate::par::map(wave, threads, |v| g.delay_visit(v, reached[v], parasitics, index, want_trace))
            };
            for (&v, r) in wave.iter().zip(results) {
                if let Some(mut d) = r? {
                    let lines = std::mem::take(&mut d.trace);
                    let clamped = d.clamped;
                    d.clamped = false;
                    self.store_vertex_delays(v, d, None);
                    if !lines.is_empty() || clamped {
                        side.push((pos[v], lines, clamped));
                    }
                }
            }
        }
        side.sort_by_key(|s| s.0);
        let mut trace = trace;
        for (p, lines, clamped) in side {
            if clamped {
                self.clamped.push(order[p]);
            }
            if let Some(t) = trace.as_deref_mut() {
                t.extend(lines);
            }
        }
        Ok(())
    }

    /// One vertex of the delay pass, computed, not stored: a vertex the search reaches is timed
    /// ([`Graph::vertex_delays`]); an unreached load with a slew limit is held at it.
    fn delay_visit(&self, v: usize, reached: bool, parasitics: &HashMap<String, NetParasitics>, index: &HashMap<String, usize>, trace: bool) -> Result<Option<VertexDelays>, String> {
        if reached {
            return self.vertex_delays(v, parasitics, index, trace).map(Some);
        }
        if !self.vertices[v].is_driver && self.slew_limit.contains_key(&v) && !self.slew_annotated.contains_key(&v) {
            // 🔑 A load the search does not reach (its driver held constant: `searchFrom`)
            // still has the slews its driver's arcs gave it, and the limit is read off them
            // all the same (repair_design's forward pass reads `graph_->slew` of EVERY load).
            let mut d = VertexDelays::default();
            if let Some(s) = self.held_at_limit(v, self.slew[v]) {
                d.slews.push((v, s));
                d.clamped = true;
            }
            return Ok(Some(d));
        }
        Ok(None)
    }

    /// Stores one visit's slews and delays, then its `clamped` entry and trace lines.
    fn store_vertex_delays(&mut self, v: usize, d: VertexDelays, trace: Option<&mut Vec<String>>) {
        for (x, s) in d.slews {
            self.slew[x] = s;
        }
        for (e, row) in d.delays {
            self.delay[e] = row;
        }
        if d.clamped {
            self.clamped.push(v);
        }
        if let Some(t) = trace {
            t.extend(d.trace);
        }
    }

    /// `GraphDelayCalc::findLatchEdgeDelays`: a latch D -> Q edge's arcs at the D slew and the Q
    /// load — the gate delays only (`annotateDelaySlew` merges no Q slew from them,
    /// `annotateDelaysSlews` annotates no load). Whether a delay changed (the reference's change
    /// test, [`crate::incr::gate_delay_changed`]).
    pub fn find_latch_edge_delays(&mut self, e: usize, parasitics: &HashMap<String, NetParasitics>, index: &HashMap<String, usize>) -> bool {
        let EdgeKind::Gate { set } = self.edges[e].kind else { return false };
        let (from, v) = (self.edges[e].from, self.edges[e].to);
        if self.is_constant(from) {
            return false;
        }
        let Some(drvr_lib) = self.vertices[v].lib else { return false };
        let arcs = &self.arc_set(e, set).arcs;
        let mut changed = false;
        for mm in [MIN, MAX] {
            for (k, arc) in arcs.iter().enumerate() {
                let Model::Gate(model) = &arc.model else { continue };
                let rf = arc.to_rf;
                let in_slew = self.slew[from][arc.from_rf][mm];
                let (pin_cap, wire_cap, pe) = self.parasitic_load(v, rf, mm, parasitics, index);
                let l = &self.libs[drvr_lib];
                let th = Thresholds { vth: l.output_threshold[rf], vl: l.slew_lower_threshold[rf], vh: l.slew_upper_threshold[rf], slew_derate: l.slew_derate };
                let gate_delay = match &pe {
                    Some((p, _)) => Dmp::new(model, &th, in_slew, p.c2, p.rpi, p.c1).gate_delay_slew().0,
                    None => f64::from(model.gate_delay(in_slew, pin_cap + wire_cap).0),
                };
                let (prev, new) = (self.delay[e][k][mm], gate_delay as f32);
                changed |= crate::incr::gate_delay_changed(prev, new);
                self.delay[e][k][mm] = new;
            }
        }
        changed
    }

    /// Whether the simulation holds vertex `v` constant ([`SdcEnv::constants`]).
    /// The vertices the delay calculation's search visits (`findDelays`: from the levelization
    /// roots, `DcalcPred`): it leaves no constant pin (`searchFrom`) and crosses no timing check,
    /// disabled set/clear arc or bidirect instance/port path (`searchThru`). A vertex it never
    /// reaches keeps its slews as made, 0 — e.g. a cell output whose every input is a constant,
    /// where a timed driver would hold the init values (±1e30) no arc replaced.
    fn dcalc_reached(&self, order: &[usize]) -> Vec<bool> {
        let thru = |e: usize| match self.edges[e].kind {
            EdgeKind::Gate { set } => !self.is_check(e) && self.arc_set(e, set).role != Role::RegSetClr && !self.bidirect_inst_paths.contains(&e) && !self.disabled_loops.contains(&e),
            EdgeKind::Wire => !self.bidirect_port_paths.contains(&e),
        };
        let levels = |e: usize| self.levelize_thru(e);
        let mut reached = vec![false; self.vertices.len()];
        for &v in order {
            reached[v] = !self.in_edges[v].iter().any(|&e| levels(e))
                || self.in_edges[v].iter().any(|&e| {
                    let from = self.edges[e].from;
                    thru(e) && reached[from] && !self.is_constant(from)
                });
        }
        reached
    }

    /// A load with a slew limit ([`Graph::slew_limit`]): its driver set its slew; before its
    /// fanout reads it, an excess on either transition holds BOTH max slews at the limit — the
    /// held slews, `None` when nothing is held (`clamped` takes the vertex).
    fn held_at_limit(&self, v: usize, mut slew: [[f32; 2]; 2]) -> Option<[[f32; 2]; 2]> {
        let &limit = self.slew_limit.get(&v)?;
        if self.in_edges[v].iter().all(|&e| self.is_check(e)) {
            return None;
        }
        if slew[RISE][MAX] > limit || slew[FALL][MAX] > limit {
            slew[RISE][MAX] = limit;
            slew[FALL][MAX] = limit;
            return Some(slew);
        }
        None
    }

    /// `set_thread_count`: `<= 0` every hardware thread, above the machine's count clamped to it
    /// ([`crate::par::resolve_thread_count`]). Returns the count set.
    pub fn set_thread_count(&mut self, requested: i32) -> usize {
        self.thread_count = crate::par::resolve_thread_count(requested);
        self.thread_count
    }

    pub fn is_constant(&self, v: usize) -> bool {
        self.sdc_at(&self.sdc.constants, v).is_some()
    }

    /// Every vertex's index by name (the parasitics name their nodes by pin).
    pub fn pin_index(&self) -> std::sync::Arc<HashMap<String, usize>> {
        self.index_cache.get_or_init(|| std::sync::Arc::new((0..self.vertices.len()).map(|i| (self.vname(i), i)).collect())).clone()
    }

    /// Whether every name shared by two vertices is a bidirect top-level port's (its load and its
    /// driver, adjacent): the name index then holds one vertex of each, and
    /// [`Graph::vertices_named`] gives both.
    pub fn names_shared_only_by_bidirect_ports(&self) -> bool {
        let ports: Vec<&str> = self.netlist.ports.iter().filter(|p| p.1 == PortDir::Inout).map(|p| p.0.as_str()).collect();
        let index = self.index();
        index.len() + ports.len() == self.vertices.len() && ports.iter().all(|p| index.get(*p).is_some_and(|&v| matches!(self.vertices[v].conn, Conn::Port(_)) && self.vertices_named(v).count() == 2))
    }

    /// The vertices of `v`'s pin: `v`, and the other of a bidirect pin's two (adjacent, same name
    /// and connection).
    pub fn vertices_named(&self, v: usize) -> std::ops::RangeInclusive<usize> {
        let same = |a: usize, b: usize| self.vertices[a].conn == self.vertices[b].conn;
        let lo = if v > 0 && same(v - 1, v) { v - 1 } else { v };
        let hi = if v + 1 < self.vertices.len() && same(v, v + 1) { v + 1 } else { v };
        lo..=hi
    }

    /// [`Graph::pin_index`], built once and borrowed.
    pub fn index(&self) -> &HashMap<String, usize> {
        self.shared_index()
    }

    /// [`Graph::index`] as its shared handle: a holder that must not borrow the graph takes this,
    /// never a copy of the map (one name per vertex).
    pub fn shared_index(&self) -> &std::sync::Arc<HashMap<String, usize>> {
        self.index_cache.get_or_init(|| std::sync::Arc::new((0..self.vertices.len()).map(|i| (self.vname(i), i)).collect()))
    }

    /// Vertex `v`'s name: its pin's (`Network::pathName` of the vertex's pin).
    pub fn vname(&self, v: usize) -> String {
        self.netlist.pin_name(&self.vertices[v].conn)
    }

    /// Is vertex `v`'s name `s`? Compared against the pin; no name is made.
    pub fn vname_is(&self, v: usize, s: &str) -> bool {
        self.netlist.pin_name_is(&self.vertices[v].conn, s)
    }

    /// `f` of vertex `v`'s name, borrowed: the name is written into a per-thread buffer, so a
    /// lookup by name makes no string of its own. Reentrant: a nested call takes a fresh buffer.
    pub fn with_vname<R>(&self, v: usize, f: impl FnOnce(&str) -> R) -> R {
        thread_local! {
            static BUF: std::cell::Cell<String> = const { std::cell::Cell::new(String::new()) };
        }
        let mut buf = BUF.with(std::cell::Cell::take);
        let r = f(self.netlist.pin_name_into(&self.vertices[v].conn, &mut buf));
        BUF.with(|b| b.set(buf));
        r
    }

    /// The SDC entry keyed by vertex `v`'s name: an empty map is not looked in, a full one by
    /// the borrowed name. The same entry as `map.get(&self.vname(v))`.
    fn sdc_at<'m, T>(&self, map: &'m HashMap<String, T>, v: usize) -> Option<&'m T> {
        if map.is_empty() {
            return None;
        }
        self.with_vname(v, |n| map.get(n))
    }

    /// One vertex of the delay calculation (`findVertexDelay`): a load takes its root slew, its
    /// annotation or its limit; an input port its drive; a driver every arc of every in-edge,
    /// its slew and its loads' slews and wire delays merged from the initial values.
    pub fn find_vertex_delays(&mut self, v: usize, parasitics: &HashMap<String, NetParasitics>, index: &HashMap<String, usize>, trace: Option<&mut Vec<String>>) -> Result<(), String> {
        let d = self.vertex_delays(v, parasitics, index, trace.is_some())?;
        self.store_vertex_delays(v, d, trace);
        Ok(())
    }

    /// [`Graph::find_vertex_delays`] computed, not stored: the visit runs on copies of the slews
    /// and delay rows it writes ([`Overlay`]) and returns them, so visits of one wave can run on
    /// several threads ([`crate::par`]). With `trace`, the trace lines it would push.
    pub fn vertex_delays(&self, v: usize, parasitics: &HashMap<String, NetParasitics>, index: &HashMap<String, usize>, trace: bool) -> Result<VertexDelays, String> {
        let mut lines: Vec<String> = Vec::new();
        const INIT: [f32; 2] = [1e30, -1e30];
        // Fuzzily worse for this min/max, so the merge order matters.
        let worse = |mm: usize, a: f32, b: f32| if mm == MAX { crate::fuzzy::greater(a, b) } else { crate::fuzzy::less(a, b) };
        let fanin: Vec<usize> = self.in_edges[v].iter().copied().filter(|&e| !self.is_check(e)).collect();
        let wires: Vec<usize> = self.out_edges[v].iter().copied().filter(|&e| matches!(self.edges[e].kind, EdgeKind::Wire)).collect();
        if !self.vertices[v].is_driver {
            let mut d = VertexDelays::default();
            if fanin.is_empty() {
                // A root load's slew is 0.
                d.slews.push((v, [[0.0; 2]; 2]));
            } else if let Some(&value) = self.slew_annotated.get(&v) {
                // Kept only while it is blocked: an annotation the bits do not protect
                // ([`Graph::slew_blocked`]) was overwritten by its driver, and visiting the load
                // (`findVertexDelay`) writes no slew.
                if slew_blocked_by(&self.slew_blocked, &self.slew_annotated, v, MAX) {
                    let mut s = self.slew[v];
                    s[RISE][MAX] = value;
                    s[FALL][MAX] = value;
                    d.slews.push((v, s));
                }
            } else if let Some(s) = self.held_at_limit(v, self.slew[v]) {
                d.slews.push((v, s));
                d.clamped = true;
            }
            return Ok(d);
        }
        let mut o = Overlay::new(self, v, &wires, &fanin);
        // `findVertexDelay`: a root, or a top-level bidirect port's driver vertex (its load's wire
        // into it aside), is seeded as an input.
        if fanin.is_empty() || self.is_port(v) {
            if let Some(drive) = self.sdc_at(&self.sdc.input_drive, v).cloned() {
                self.time_input_drive(&mut o, &wires, &drive, parasitics, index)?;
                return Ok(o.into_delays(&wires, lines));
            }
            // An input port (`seedNoDrvrCellSlew` / `seedNoDrvrSlew`): its slew is the
            // `set_input_transition`, else 0; its loads by the input-port delay at that slew
            // over the `parasiticLoad` pi model — a load with no Elmore delay takes the slew.
            // An ANNOTATED port slew (`setAnnotatedSlew` on the driver vertex) is kept as the
            // driver's slew and COPIED to every load (`annotateLoadDelays`: "Copy the driver slew
            // to the load if it is annotated"); the wire delay is still the input-port delay at the
            // seeded slew — `default_slew`, 0, without a `set_input_transition`.
            let annotated = self.slew_annotated.get(&v).copied();
            for rf in [RISE, FALL] {
                for mm in [MIN, MAX] {
                    let in_slew = self.sdc_at(&self.sdc.input_slew, v).and_then(|s| s[rf][mm]).unwrap_or(0.0);
                    o.sv[rf][mm] = annotated.unwrap_or(in_slew);
                    let (_, _, pe) = self.parasitic_load(v, rf, mm, parasitics, index);
                    let elmores = LoadElmores::new(pe.as_ref());
                    for (i, &e) in wires.iter().enumerate() {
                        let load = self.edges[e].to;
                        let mut wire_delay = 0.0f64;
                        let mut load_slew = f64::from(in_slew);
                        let elmore = pe.as_ref().and_then(|(p, names)| elmores.get(p, names, &self.vname(load)));
                        let ll = self.threshold_library(load);
                        if let Some(el) = elmore {
                            let l = &self.libs[ll];
                            let th = Thresholds { vth: l.input_threshold[rf], vl: l.slew_lower_threshold[rf], vh: l.slew_upper_threshold[rf], slew_derate: l.slew_derate };
                            (wire_delay, load_slew) = dspf_wire_delay_slew(f64::from(in_slew), el, &th);
                        }
                        threshold_adjust(ll == 0, &self.lib_thresholds(0, rf), &self.lib_thresholds(ll, rf), rf == RISE, &mut wire_delay, &mut load_slew);
                        // A load's own annotation stands (`annotateLoadDelays`), as from a cell driver.
                        if !slew_blocked_by(&self.slew_blocked, &self.slew_annotated, load, mm) {
                            o.ls[o.slot[i]][rf][mm] = annotated.unwrap_or(load_slew as f32);
                        } else if let (Some(&a), MAX) = (self.slew_annotated.get(&load), mm) {
                            o.ls[o.slot[i]][rf][mm] = a;
                        }
                        o.wire_delay[i][rf][mm] = wire_delay as f32;
                    }
                }
            }
            return Ok(o.into_delays(&wires, lines));
        }
        // Driver delays: init, then every arc of every in-edge.
        let drvr_lib = self.vertices[v].lib.expect("an instance driver");
        // `annotateLoadDelays` merges into a load only where its slew is not annotated: an annotated
        // load (`setAnnotatedSlew`, max rise and fall) keeps its annotation through its driver's
        // timing, not just after its own visit.
        // (`initLoadSlews`: a blocked slew is not reset — [`Graph::slew_blocked`].)
        for (i, &e) in wires.iter().enumerate() {
            let load = self.edges[e].to;
            for mm in [MIN, MAX] {
                if !slew_blocked_by(&self.slew_blocked, &self.slew_annotated, load, mm) {
                    o.ls[o.slot[i]][RISE][mm] = INIT[mm];
                    o.ls[o.slot[i]][FALL][mm] = INIT[mm];
                } else if let (Some(&a), MAX) = (self.slew_annotated.get(&load), mm) {
                    o.ls[o.slot[i]][RISE][MAX] = a;
                    o.ls[o.slot[i]][FALL][MAX] = a;
                }
            }
            o.wire_delay[i] = vec![[INIT[MIN], INIT[MAX]]; 2];
        }
        o.sv = [[INIT[MIN], INIT[MAX]], [INIT[MIN], INIT[MAX]]];
        let mut exists = [false; 2];
        // `parasiticLoad` depends on the driver, the transition and min/max only, not on the arc:
        // reduced once per pair, on first use, rather than once per arc.
        let mut loads: [[Option<ParasiticLoad>; 2]; 2] = Default::default();
        let mut elmore_maps: [[Option<LoadElmores>; 2]; 2] = Default::default();
        // Edges are PREPENDED to a vertex's in-edge list, so they are visited newest first —
        // the reverse of the cell's arc-set order.
        for &e in fanin.iter().rev() {
            let EdgeKind::Gate { set } = self.edges[e].kind else { continue };
            let arcs = &self.arc_set(e, set).arcs;
            let from = self.edges[e].from;
            // `findDriverEdgeDelays`: no arc from a constant pin (`DcalcPred::searchFrom`), and
            // none through a register set/clear arc (`DcalcPred::searchThru`: disabled while
            // `preset_clr_arcs_enabled` is false, its default) — so its slew never reaches the
            // driver: a dfrtp's Q takes CLK->Q's fall slew even where RESET_B->Q's is larger.
            // Nor through a disabled loop edge (`DcalcPred::searchThru`: `isDisabledLoop`).
            if self.is_constant(from) || self.arc_set(e, set).role == Role::RegSetClr || self.disabled_loops.contains(&e) {
                continue;
            }
            let ideal_clk_to_q = matches!(self.arc_set(e, set).role, Role::RegClkToQ | Role::LatchEnToQ) && self.ideal_clock.contains(&from);
            // `findDriverDelays1` skips a latch D -> Q edge: it is timed on its own
            // ([`Graph::find_latch_edge_delays`]).
            if self.arc_set(e, set).role == Role::LatchDtoQ {
                continue;
            }
            for mm in [MIN, MAX] {
                for (k, arc) in arcs.iter().enumerate() {
                    let Model::Gate(model) = &arc.model else { continue };
                    let rf = arc.to_rf;
                    // `edgeFromSlew`: an ideal clock's slew for a register clk->q edge
                    let in_slew = if ideal_clk_to_q { self.ideal_clk_slew(from, arc.from_rf, mm) } else { o.slew_of(self, from)[arc.from_rf][mm] };
                    if loads[rf][mm].is_none() {
                        loads[rf][mm] = Some(self.parasitic_load(v, rf, mm, parasitics, index));
                    }
                    let (pin_cap, wire_cap, pe) = loads[rf][mm].as_ref().expect("set above");
                    let elmores = elmore_maps[rf][mm].get_or_insert_with(|| LoadElmores::new(pe.as_ref()));
                    let load_cap = pin_cap + wire_cap;
                    let l = &self.libs[drvr_lib];
                    let th = Thresholds { vth: l.output_threshold[rf], vl: l.slew_lower_threshold[rf], vh: l.slew_upper_threshold[rf], slew_derate: l.slew_derate };
                    let (gate_delay, drvr_slew, mut dmp) = match &pe {
                        Some((p, _)) => {
                            let mut d = Dmp::new(model, &th, in_slew, p.c2, p.rpi, p.c1);
                            let (gd, ds) = d.gate_delay_slew();
                            (gd, ds, Some(d))
                        }
                        None => {
                            let (gd, ds) = model.gate_delay(in_slew, load_cap);
                            (f64::from(gd), f64::from(ds), None)
                        }
                    };
                    exists[rf] = true;
                    if let (true, Some(d), Some((p, _))) = (trace, &dmp, &pe) {
                        let (rd, t0, dt, ceff) = d.state();
                        let alg = match d.alg {
                            crate::dcalc::Alg::Cap => "cap",
                            crate::dcalc::Alg::Pi => "Pi",
                            crate::dcalc::Alg::ZeroC2 => "c2=0",
                        };
                        let rfc = |r: usize| if r == RISE { '^' } else { 'v' };
                        lines.push(format!("dcalc|gate|{}|{}{}|{:08x}|{:08x}|{:08x}|{:08x}|{alg}|{:016x}|{:016x}|{:016x}|{:016x}|{}|{:016x}|{:016x}", self.vname(v), rfc(arc.from_rf), rfc(rf), in_slew.to_bits(), p.c2.to_bits(), p.rpi.to_bits(), p.c1.to_bits(), rd.to_bits(), t0.to_bits(), dt.to_bits(), ceff.to_bits(), i32::from(d.driver_valid()), gate_delay.to_bits(), drvr_slew.to_bits()));
                    }
                    let (gd32, ds32) = (gate_delay as f32, drvr_slew as f32);
                    if worse(mm, ds32, o.sv[rf][mm]) {
                        o.sv[rf][mm] = ds32;
                    }
                    o.fanin_row(self, e)[k][mm] = gd32;
                    for (i, &w) in wires.iter().enumerate() {
                        let load = self.edges[w].to;
                        let (mut wire_delay, mut load_slew) = match (&mut dmp, &pe) {
                            (Some(d), Some((p, names))) => match elmores.get(p, names, &self.vname(load)) {
                                Some(el) => d.load_delay_slew(f64::from(el)),
                                None => (0.0, drvr_slew),
                            },
                            _ => (0.0, f64::from(ds32)),
                        };
                        if let (true, Some(_)) = (trace, &dmp) {
                            lines.push(format!("dcalc|load|{}|{}|{:016x}|{:016x}", self.vname(v), self.vname(load), wire_delay.to_bits(), load_slew.to_bits()));
                        }
                        let ll = self.threshold_library(load);
                        threshold_adjust(ll == drvr_lib, &self.lib_thresholds(drvr_lib, rf), &self.lib_thresholds(ll, rf), rf == RISE, &mut wire_delay, &mut load_slew);
                        if worse(mm, load_slew as f32, o.ls[o.slot[i]][rf][mm]) && !slew_blocked_by(&self.slew_blocked, &self.slew_annotated, load, mm) {
                            o.ls[o.slot[i]][rf][mm] = load_slew as f32;
                        }
                        if worse(mm, wire_delay as f32, o.wire_delay[i][rf][mm]) {
                            o.wire_delay[i][rf][mm] = wire_delay as f32;
                        }
                    }
                }
            }
        }
        for rf in [RISE, FALL] {
            if !exists[rf] {
                for mm in [MIN, MAX] {
                    o.sv[rf][mm] = INIT[mm];
                    for (i, &w) in wires.iter().enumerate() {
                        o.wire_delay[i][rf][mm] = 0.0;
                        let load = self.edges[w].to;
                        if !slew_blocked_by(&self.slew_blocked, &self.slew_annotated, load, mm) {
                            o.ls[o.slot[i]][rf][mm] = 0.0;
                        }
                    }
                }
            }
        }
        Ok(o.into_delays(&wires, lines))
    }

    /// `Search::timingDerate`: the factor on edge `e`'s arc `k` for a clock (`is_clk`) or data
    /// path of min/max `mm` (early/late) — a wire's the net delay factor of the arc's TO
    /// transition, a timing check's the cell check factor of its TO transition, any other arc's
    /// the cell delay factor of its FROM transition; 1 where unset (`timingDerateNet` /
    /// `timingDerateInstance` with no net, instance or cell factor).
    pub fn timing_derate(&self, e: usize, k: usize, is_clk: bool, mm: usize) -> f32 {
        let Some(d) = &self.sdc.derates else { return 1.0 };
        let clk_data = if is_clk { DERATE_CLK } else { DERATE_DATA };
        let (kind, rf) = match self.edges[e].kind {
            // a wire's arc `k` carries transition `k` through unchanged
            EdgeKind::Wire => (DERATE_NET_DELAY, k),
            EdgeKind::Gate { set } => {
                let arc_set = self.arc_set(e, set);
                let arc = &arc_set.arcs[k];
                if matches!(arc_set.role, Role::Setup | Role::Hold | Role::Recovery | Role::Removal) {
                    (DERATE_CELL_CHECK, arc.to_rf)
                } else {
                    (DERATE_CELL_DELAY, arc.from_rf)
                }
            }
        };
        d[kind][clk_data][rf][mm].unwrap_or(1.0)
    }

    /// `ClkNetwork::idealClkSlew(pin, rf, min_max)`: the slew of the ideal clocks at the pin.
    fn ideal_clk_slew(&self, v: usize, rf: usize, mm: usize) -> f32 {
        self.ideal_clock_slew.get(&v).map_or(0.0, |s| s[rf][mm])
    }

    /// One timing check edge's delays (`findCheckEdgeDelays`), from the slews at both ends.
    pub fn find_check_edge_delays(&mut self, e: usize) {
        let EdgeKind::Gate { set } = self.edges[e].kind else { return };
        let arcs = &self.arc_set(e, set).arcs;
        let (from, to) = (self.edges[e].from, self.edges[e].to);
        for (k, arc) in arcs.iter().enumerate() {
            let Model::Check(table) = &arc.model else { continue };
            for mm in [MIN, MAX] {
                // `checkEdgeClkSlew`: under OCV the opposite min/max (`checkClkSlewMinMax`)
                let clk_slew = if self.ideal_clock.contains(&from) { self.ideal_clk_slew(from, arc.from_rf, 1 - mm) } else { self.slew[from][arc.from_rf][1 - mm] };
                let data_slew = self.slew[to][arc.to_rf][mm];
                let pick = |a: usize| match table.axes.get(a).map(|x| x.var) {
                    Some(crate::table::AxisVar::RelatedPinTransition) => clk_slew,
                    Some(crate::table::AxisVar::ConstrainedPinTransition) => data_slew,
                    _ => 0.0,
                };
                self.delay[e][k][mm] = table.find_value(pick(0), pick(1), pick(2));
            }
        }
    }
}

/// The graph as loop breaking reads it ([`crate::levelize::LoopGraph`]): vertex and edge ids are
/// the indices (the reference's ids less one), a vertex's out-edges newest first.
struct LevelizeView<'g, 'a>(&'g Graph<'a>);

impl crate::levelize::LoopGraph for LevelizeView<'_, '_> {
    fn vertex_count(&self) -> usize {
        self.0.vertices.len()
    }
    fn edge_count(&self) -> usize {
        self.0.edges.len()
    }
    fn out_edge(&self, v: usize, k: usize) -> Option<usize> {
        let o = &self.0.out_edges[v];
        (k < o.len()).then(|| o[o.len() - 1 - k])
    }
    fn in_edges(&self, v: usize) -> &[usize] {
        &self.0.in_edges[v]
    }
    fn edge_from(&self, e: usize) -> usize {
        self.0.edges[e].from
    }
    fn edge_to(&self, e: usize) -> usize {
        self.0.edges[e].to
    }
    fn search_thru(&self, e: usize) -> bool {
        self.0.levelize_thru_unlooped(e)
    }
    /// `bidirect_inst_paths_enabled` is off (its default; no job here sets it).
    fn bidirect_load_fanout(&self, _v: usize) -> bool {
        false
    }
    /// `from -> to` and the arc set's index on its cell, for the loop trace.
    fn edge_label(&self, e: usize) -> String {
        let ed = &self.0.edges[e];
        let set = match ed.kind {
            EdgeKind::Gate { set } => set.to_string(),
            EdgeKind::Wire => "wire".to_string(),
        };
        format!("{} -> {} {set}", self.0.vname(ed.from), self.0.vname(ed.to))
    }
    /// `Network::pathNameCmp` over a flat netlist: a top port (the top instance, an empty path)
    /// before any instance pin; then the instance name, then the port name, bytewise.
    fn name_cmp(&self, a: usize, b: usize) -> std::cmp::Ordering {
        fn key(port: bool, name: &str) -> (u8, &str, &str) {
            if port {
                (0u8, "", name)
            } else {
                match name.rfind('/') {
                    Some(i) => (1, &name[..i], &name[i + 1..]),
                    None => (1, "", name),
                }
            }
        }
        let (name_a, name_b) = (self.0.vname(a), self.0.vname(b));
        key(self.0.is_port(a), &name_a).cmp(&key(self.0.is_port(b), &name_b))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::netlist::{Conn, Net, Netlist, PortDir};

    /// Rule (`ReducedParasitic` keeps a load's Elmore delay in a pin map; `findElmore` finds it or
    /// reports none): a load's delay by name is the first entry a pass over the list finds, on a
    /// net small enough for the pass and on one large enough for the map, and none for a pin the
    /// parasitic does not hold.
    #[test]
    fn a_loads_elmore_by_name_is_the_one_the_pass_finds() {
        for n in [3usize, 40] {
            let names: Vec<String> = (0..n).map(|k| format!("u{}/A", k % (n - 1))).collect();
            let p = crate::parasitics::PiElmore { c2: 0.0, rpi: 0.0, c1: 0.0, elmore: (0..n).map(|k| (k, k as f32)).collect() };
            let pe = (p.clone(), names.as_slice());
            let ix = LoadElmores::new(Some(&pe));
            assert_eq!(ix.by_name.is_some(), n > LoadElmores::PASS_MAX);
            for name in names.iter().map(String::as_str).chain(["nowhere"]) {
                let pass = p.elmore.iter().find(|(k, _)| names[*k] == name).map(|&(_, e)| e);
                assert_eq!(ix.get(&p, &names, name), pass, "{name} of {n}");
            }
            assert_eq!(ix.get(&p, &names, "u0/A"), Some(0.0), "the repeated name: its first entry");
        }
    }

    const LIB: &str = r#"library (t) {
      time_unit : "1ns"; capacitive_load_unit (1, pf);
      cell (and2) {
        pin (A) { direction : input; capacitance : 0.001; }
        pin (B) { direction : input; capacitance : 0.001; }
        pin (Y) { direction : output; function : "A&B";
          timing () { related_pin : "A"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("0.1"); } rise_transition (scalar) { values ("1.0"); }
            cell_fall (scalar) { values ("0.1"); } fall_transition (scalar) { values ("1.0"); } }
          timing () { related_pin : "B"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("0.1"); } rise_transition (scalar) { values ("1.0000005"); }
            cell_fall (scalar) { values ("0.1"); } fall_transition (scalar) { values ("1.0000005"); } }
        }
      }
    }"#;

    // Rule (`Graph::makePinVertices` over every leaf instance, `dbNetwork::direction` from the
    // terminal when no liberty port): an instance whose cell no library holds still has a vertex
    // per signal pin, in instance then terminal order, with wire edges and no gate edge — its
    // output is a root driver, and a vertex without a library is not a top-level port. Without
    // the unlinked map the instance has no vertex at all.
    #[test]
    fn an_unlinked_instance_has_vertices_and_wires_only() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        let nl = Netlist {
            insts: vec![crate::netlist::inst("x", "blk"), crate::netlist::inst("u1", "and2")],
            ports: vec![("a".into(), PortDir::Input), ("y".into(), PortDir::Output)],
            nets: vec![
                Net { name: "a".into(), pins: vec![pin(0, "A"), pin(1, "B"), Conn::Port(0)] }.into(),
                Net { name: "n".into(), pins: vec![pin(0, "Y"), pin(1, "A")] }.into(),
                Net { name: "y".into(), pins: vec![pin(1, "Y"), Conn::Port(1)] }.into(),
            ],
        };
        let unlinked: HashMap<String, Vec<(String, Direction)>> = [("blk".to_string(), vec![("A".to_string(), Direction::Input), ("Y".to_string(), Direction::Output)])].into();
        let g = Graph::build_with_unlinked(&libs, &nl, None, &unlinked).unwrap();
        let names: Vec<String> = (0..g.vertices.len()).map(|v| g.vname(v)).collect();
        let names: Vec<&str> = names.iter().map(String::as_str).collect();
        assert_eq!(names, ["x/A", "x/Y", "u1/A", "u1/B", "u1/Y", "a", "y"]);
        let y = 1;
        assert!(g.vertices[y].is_driver && g.in_edges[y].is_empty(), "a root driver");
        assert!(!g.is_port(y) && g.vertices[y].lib.is_none());
        let edges: Vec<(&str, &str, bool)> = g.edges.iter().map(|e| (names[e.from], names[e.to], matches!(e.kind, EdgeKind::Wire))).collect();
        // wire edges by driver PIN order (`makeWireEdges`: leaf instances, then the ports), each
        // to its net's loads in the net's pin order
        assert_eq!(edges, [("u1/A", "u1/Y", false), ("u1/B", "u1/Y", false), ("x/Y", "u1/A", true), ("u1/Y", "y", true), ("a", "x/A", true), ("a", "u1/B", true)]);
        let plain = Graph::build(&libs, &nl).unwrap();
        assert!((0..plain.vertices.len()).all(|v| !plain.vname(v).starts_with("x/")));
        // the master's terminal order, not the liberty's — in every build given the terminals
        let pins: HashMap<String, Vec<String>> = [("and2".to_string(), vec!["Y".to_string(), "B".to_string(), "A".to_string()])].into();
        let g = Graph::build_with_unlinked(&libs, &nl, Some(&pins), &unlinked).unwrap();
        let names: Vec<String> = (0..g.vertices.len()).map(|v| g.vname(v)).collect();
        let names: Vec<&str> = names.iter().map(String::as_str).collect();
        assert_eq!(names, ["x/A", "x/Y", "u1/Y", "u1/B", "u1/A", "a", "y"]);
        let plain = Graph::build_with_pins(&libs, &nl, Some(&pins)).unwrap();
        let names: Vec<String> = (0..plain.vertices.len()).map(|v| plain.vname(v)).collect();
        let names: Vec<&str> = names.iter().map(String::as_str).collect();
        assert_eq!(&names[..3], ["u1/Y", "u1/B", "u1/A"]);
        let built = Graph::build(&libs, &nl).unwrap();
        assert_eq!((0..built.vertices.len()).position(|v| built.vname(v) == "u1/A"), Some(0), "no terminals: the liberty's order");
    }

    // Rule (`Graph::makePinVertices` iterates the instance's pins — the master's terminals in the
    // database — so vertex ids follow the LEF order, not the liberty's): the per-cell template and
    // the by-name build lay an instance out alike when given the terminals.
    #[test]
    fn both_builds_take_the_terminal_order() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        let nl = Netlist {
            insts: vec![crate::netlist::inst("u1", "and2"), crate::netlist::inst("u2", "and2")],
            ports: vec![("a".into(), PortDir::Input), ("y".into(), PortDir::Output)],
            nets: vec![
                Net { name: "a".into(), pins: vec![pin(0, "A"), pin(0, "B"), pin(1, "B"), Conn::Port(0)] }.into(),
                Net { name: "n".into(), pins: vec![pin(0, "Y"), pin(1, "A")] }.into(),
                Net { name: "y".into(), pins: vec![pin(1, "Y"), Conn::Port(1)] }.into(),
            ],
        };
        let pins: HashMap<String, Vec<String>> = [("and2".to_string(), vec!["Y".to_string(), "A".to_string(), "B".to_string()])].into();
        let keep = || NetlistRef::Borrowed(&nl);
        let by_cell = Graph::build_by_cell(&libs, &nl, keep(), Some(&pins)).expect("a template build");
        let by_name = Graph::build_by_name(&libs, &nl, keep(), Some(&pins), None).unwrap();
        for g in [&by_cell, &by_name] {
            let names: Vec<String> = (0..g.vertices.len()).map(|v| g.vname(v)).collect();
        let names: Vec<&str> = names.iter().map(String::as_str).collect();
            assert_eq!(&names[..6], ["u1/Y", "u1/A", "u1/B", "u2/Y", "u2/A", "u2/B"]);
        }
        let ends = |g: &Graph| g.edges.iter().map(|e| (e.from, e.to)).collect::<Vec<_>>();
        assert_eq!(ends(&by_cell), ends(&by_name));
        // Rule (`Graph::makeWireEdges`: `makeInstDrvrWireEdges` over the leaf instances, then the
        // top instance — each driver PIN in pin order makes its net's wire edges, so wire ids
        // follow the drivers, not the nets): net `y` (driven by u2) is listed after `n` (u1) and
        // `a` (a port) first, yet its edge comes second and the port's last.
        for g in [&by_cell, &by_name] {
            let owned: Vec<String> = (0..g.vertices.len()).map(|v| g.vname(v)).collect();
            let name = |v: usize| owned[v].as_str();
            let wires: Vec<(&str, &str)> = g.edges.iter().filter(|e| matches!(e.kind, EdgeKind::Wire)).map(|e| (name(e.from), name(e.to))).collect();
            assert_eq!(wires, [("u1/Y", "u2/A"), ("u2/Y", "y"), ("a", "u1/A"), ("a", "u1/B"), ("a", "u2/B")]);
        }
    }

    const DFF_LIB: &str = r#"library (t) {
      time_unit : "1ns"; capacitive_load_unit (1, pf);
      cell (dffsr) {
        ff (IQ, IQN) { clocked_on : "CLK"; next_state : "D"; clear : "!RN"; preset : "!SN"; }
        pin (CLK) { direction : input; clock : true; capacitance : 0.001;
          timing () { related_pin : "CLK"; timing_type : min_pulse_width;
            rise_constraint (scalar) { values ("0.1"); } fall_constraint (scalar) { values ("0.1"); } } }
        pin (D) { direction : input; capacitance : 0.001;
          timing () { related_pin : "CLK"; timing_type : setup_rising;
            rise_constraint (scalar) { values ("0.05"); } fall_constraint (scalar) { values ("0.05"); } } }
        pin (RN) { direction : input; capacitance : 0.001;
          timing () { related_pin : "SN"; timing_type : non_seq_setup_rising;
            rise_constraint (scalar) { values ("0.05"); } fall_constraint (scalar) { values ("0.05"); } } }
        pin (SN) { direction : input; capacitance : 0.001; }
        pin (Q) { direction : output; function : "IQ";
          timing () { related_pin : "CLK"; timing_type : rising_edge;
            cell_rise (scalar) { values ("0.1"); } rise_transition (scalar) { values ("0.1"); }
            cell_fall (scalar) { values ("0.1"); } fall_transition (scalar) { values ("0.1"); } } }
      }
    }"#;

    // Rule (`Graph::makePortInstanceEdges`: an edge for EVERY arc set, whatever its role — the
    // checks timed nowhere here, pulse width and non-sequential, included; `isTimingCheck` keeps
    // every search off them; `setHasChecks` only for a check BETWEEN pins,
    // `isTimingCheckBetween` = a check but not width or period): the width check is a CLK->CLK
    // self-edge and the non-sequential one SN->RN; both take edge ids, neither is timed.
    #[test]
    fn untimed_checks_take_edges_but_are_not_timed() {
        let libs = [Library::read(&crate::liberty_parse::parse(DFF_LIB).unwrap()).unwrap()];
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        let nl = Netlist {
            insts: vec![crate::netlist::inst("r", "dffsr")],
            ports: vec![("clk".into(), PortDir::Input), ("d".into(), PortDir::Input), ("rn".into(), PortDir::Input), ("sn".into(), PortDir::Input), ("q".into(), PortDir::Output)],
            nets: vec![
                Net { name: "clk".into(), pins: vec![pin(0, "CLK"), Conn::Port(0)] }.into(),
                Net { name: "d".into(), pins: vec![pin(0, "D"), Conn::Port(1)] }.into(),
                Net { name: "rn".into(), pins: vec![pin(0, "RN"), Conn::Port(2)] }.into(),
                Net { name: "sn".into(), pins: vec![pin(0, "SN"), Conn::Port(3)] }.into(),
                Net { name: "q".into(), pins: vec![pin(0, "Q"), Conn::Port(4)] }.into(),
            ],
        };
        for g in [Graph::build(&libs, &nl).unwrap(), Graph::build_by_name(&libs, &nl, NetlistRef::Borrowed(&nl), None, None).unwrap()] {
            let owned: Vec<String> = (0..g.vertices.len()).map(|v| g.vname(v)).collect();
            let name = |v: usize| owned[v].as_str();
            let gate: Vec<(&str, &str, bool, bool)> = (0..g.edges.len())
                .filter(|&e| matches!(g.edges[e].kind, EdgeKind::Gate { .. }))
                .map(|e| (name(g.edges[e].from), name(g.edges[e].to), g.is_check(e), g.is_timed_check(e)))
                .collect();
            assert!(gate.contains(&("r/CLK", "r/CLK", true, false)), "{gate:?}");
            assert!(gate.contains(&("r/SN", "r/RN", true, false)), "{gate:?}");
            assert!(gate.contains(&("r/CLK", "r/D", true, true)), "{gate:?}");
            assert!(gate.contains(&("r/CLK", "r/Q", false, false)), "{gate:?}");
            let set_of = |e: usize| match g.edges[e].kind { EdgeKind::Gate { set } => g.arc_set(e, set), EdgeKind::Wire => unreachable!() };
            let width = (0..g.edges.len()).find(|&e| name(g.edges[e].from) == "r/CLK" && name(g.edges[e].to) == "r/CLK").unwrap();
            let nonseq = (0..g.edges.len()).find(|&e| name(g.edges[e].from) == "r/SN").unwrap();
            assert!(!set_of(width).is_check_between(), "a width check is no check between pins");
            assert!(set_of(nonseq).is_check_between());
        }
    }

    // Rule (`LibertyBuilder::makeTimingArcs`, OpenSTA at the 4155ac3 pin): `nochange_*`,
    // `retaining_time` and an unknown `timing_type` make NO arc set (the builder returns null;
    // `timingTypeIsCheck` still lists nochange, but only to pick a check model for `intrinsic_*`).
    // So `makePortInstanceEdges` makes no edge for them and no search sees them: the graph is the
    // one the same cell without those groups makes, edge for edge.
    #[test]
    fn a_nochange_arc_makes_no_arc_set_and_no_edge() {
        let with = DFF_LIB.replace(
            "pin (SN) { direction : input; capacitance : 0.001; }",
            "pin (SN) { direction : input; capacitance : 0.001;
          timing () { related_pin : \"D\"; timing_type : nochange_high_high;
            rise_constraint (scalar) { values (\"0.05\"); } fall_constraint (scalar) { values (\"0.05\"); } }
          timing () { related_pin : \"CLK\"; timing_type : nochange_low_low;
            rise_constraint (scalar) { values (\"0.05\"); } fall_constraint (scalar) { values (\"0.05\"); } }
          timing () { related_pin : \"CLK\"; timing_type : retaining_time;
            rise_constraint (scalar) { values (\"0.05\"); } fall_constraint (scalar) { values (\"0.05\"); } }
          timing () { related_pin : \"D\"; timing_type : no_such_type;
            rise_constraint (scalar) { values (\"0.05\"); } fall_constraint (scalar) { values (\"0.05\"); } } }",
        );
        assert_ne!(with, DFF_LIB, "the groups were added");
        let lib_with = [Library::read(&crate::liberty_parse::parse(&with).unwrap()).unwrap()];
        let lib_without = [Library::read(&crate::liberty_parse::parse(DFF_LIB).unwrap()).unwrap()];
        let types = |l: &Library| l.cells["dffsr"].arc_sets.iter().map(|a| a.timing_type.clone()).collect::<Vec<_>>();
        assert_eq!(types(&lib_with[0]), types(&lib_without[0]), "no arc set for nochange, retaining_time or an unknown type");
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        let nl = Netlist {
            insts: vec![crate::netlist::inst("r", "dffsr")],
            ports: vec![("clk".into(), PortDir::Input), ("d".into(), PortDir::Input), ("rn".into(), PortDir::Input), ("sn".into(), PortDir::Input), ("q".into(), PortDir::Output)],
            nets: vec![
                Net { name: "clk".into(), pins: vec![pin(0, "CLK"), Conn::Port(0)] }.into(),
                Net { name: "d".into(), pins: vec![pin(0, "D"), Conn::Port(1)] }.into(),
                Net { name: "rn".into(), pins: vec![pin(0, "RN"), Conn::Port(2)] }.into(),
                Net { name: "sn".into(), pins: vec![pin(0, "SN"), Conn::Port(3)] }.into(),
                Net { name: "q".into(), pins: vec![pin(0, "Q"), Conn::Port(4)] }.into(),
            ],
        };
        let ends = |g: &Graph| g.edges.iter().map(|e| (e.from, e.to)).collect::<Vec<_>>();
        let (a, b) = (Graph::build(&lib_with, &nl).unwrap(), Graph::build(&lib_without, &nl).unwrap());
        assert_eq!(ends(&a), ends(&b));
        assert!(!a.edges.iter().any(|e| a.vname(e.to) == "r/SN" && matches!(e.kind, EdgeKind::Gate { .. })), "no edge into SN");
    }

    // Rule (`Graph::isPassiveBidirectDrvr` / `hasActiveDrvr`, OpenSTA fc6ed303 #525): an
    // instance pin's bidirect driver vertex with no fanin — a pad cell no library holds — makes no
    // wire edge on a net with an active driver, here the inout port. So the port's load vertex
    // has no levelized in-edge and is a root (zerosoc_pads' we_pad ports: seeded load AND driver).
    #[test]
    fn a_passive_bidirect_pad_drives_nothing_on_a_driven_net() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = Netlist {
            insts: vec![crate::netlist::inst("x", "gpio")],
            ports: vec![("b".into(), PortDir::Inout)],
            nets: vec![Net { name: "b".into(), pins: vec![Conn::Inst(0, "PAD".into()), Conn::Port(0)] }.into()],
        };
        let unlinked: HashMap<String, Vec<(String, Direction)>> = [("gpio".to_string(), vec![("PAD".to_string(), Direction::Bidirect)])].into();
        let g = Graph::build_with_unlinked(&libs, &nl, None, &unlinked).unwrap();
        let label = |v: usize| format!("{}{}", g.vname(v), if g.vertices[v].is_driver { "+" } else { "" });
        let edges: Vec<String> = g.edges.iter().map(|e| format!("{}>{}", label(e.from), label(e.to))).collect();
        assert_eq!(edges, ["b+>x/PAD", "b>b+"]);
    }

    // Rules (`makePinVertices`, `makePortInstanceEdges`, `makeWireEdgesFromPin`,
    // `makeInstDrvrWireEdges`), whole graph only: a bidirect pin or port is its load vertex then
    // its driver vertex; a combinational arc into it reaches the driver, and a bidirect from pin's
    // driver also reaches the to pin; a net wires every driver to every load but the same pin; a
    // bidirect port's load reaches its driver, an edge levelization skips. The plain build still
    // refuses a bidirect on a net.
    #[test]
    fn a_bidirect_pin_is_a_load_then_a_driver() {
        let lib = r#"library (p) { time_unit : "1ns"; capacitive_load_unit (1, pf);
          cell (pad) {
            pin (c2p) { direction : input; capacitance : 0.001; }
            pin (pad) { direction : inout; capacitance : 0.002; function : "c2p";
              timing () { related_pin : "c2p"; timing_sense : positive_unate;
                cell_rise (scalar) { values ("1"); } rise_transition (scalar) { values ("1"); }
                cell_fall (scalar) { values ("1"); } fall_transition (scalar) { values ("1"); } } }
            pin (p2c) { direction : output; function : "pad";
              timing () { related_pin : "pad"; timing_sense : positive_unate;
                cell_rise (scalar) { values ("1"); } rise_transition (scalar) { values ("1"); }
                cell_fall (scalar) { values ("1"); } fall_transition (scalar) { values ("1"); } } } } }"#;
        let libs = [Library::read(&crate::liberty_parse::parse(lib).unwrap()).unwrap()];
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        let nl = Netlist {
            insts: vec![crate::netlist::inst("u", "pad"), crate::netlist::inst("w", "pad")],
            ports: vec![("p".into(), PortDir::Input), ("b".into(), PortDir::Inout)],
            nets: vec![
                Net { name: "p".into(), pins: vec![pin(0, "pad"), Conn::Port(0)] }.into(),
                Net { name: "b".into(), pins: vec![pin(1, "pad"), Conn::Port(1)] }.into(),
            ],
        };
        let g = Graph::build_with_unlinked(&libs, &nl, None, &HashMap::new()).unwrap();
        let label = |v: usize| format!("{}{}", g.vname(v), if g.vertices[v].is_driver { "+" } else { "" });
        let names: Vec<String> = (0..g.vertices.len()).map(label).collect();
        assert_eq!(names, ["u/c2p", "u/pad", "u/pad+", "u/p2c+", "w/c2p", "w/pad", "w/pad+", "w/p2c+", "p+", "b", "b+"]);
        let edges: Vec<String> = g.edges.iter().map(|e| format!("{}>{}", label(e.from), label(e.to))).collect();
        assert_eq!(
            edges,
            [
                "u/c2p>u/pad+", "u/pad>u/p2c+", "u/pad+>u/p2c+",
                "w/c2p>w/pad+", "w/pad>w/p2c+", "w/pad+>w/p2c+",
                "p+>u/pad", "w/pad+>b", "b+>w/pad", "b>b+",
            ]
        );
        let mut inst_paths: Vec<usize> = g.bidirect_inst_paths.iter().copied().collect();
        inst_paths.sort_unstable();
        assert_eq!(inst_paths, [2, 5], "the pad drivers' paths into p2c");
        let level = g.levels().unwrap();
        assert_eq!(level[3], level[1] + 1, "p2c levels from the pad's load, not through its driver");
        // `isBidirectPortPath`: levelization skips the port's load -> driver edge, so b+ is a root.
        assert_eq!(g.bidirect_port_paths.iter().copied().collect::<Vec<_>>(), [9]);
        assert!(level[9] > 0, "b is loaded by w/pad+");
        assert_eq!(level[10], 0, "b+ levels as a root, not past b");
        assert!(!g.names_shared_only_by_bidirect_ports(), "instance bidirect pins share names too");
        assert!(Graph::build(&libs, &nl).is_err());
    }

    // Rules (OpenSTA at the 4155ac3 pin): `Graph::makePinVertices` gives a bidirect TOP-LEVEL
    // port two vertices, its load then its driver; `makeWireEdgesFromPin` wires the port's driver
    // to its net's loads but itself; and `makeInstDrvrWireEdges` on the top instance then makes
    // the port's load-to-driver edge (`setIsBidirectPortPath`), which levelization skips. Every
    // build does this, not only the whole one: a design's inout pads (a DDR `dq` bus) reach the
    // repairer's graph, alone on their nets or driving a cell.
    #[test]
    fn a_bidirect_port_is_two_vertices_in_every_build() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        let nl = Netlist {
            insts: vec![crate::netlist::inst("u1", "and2")],
            ports: vec![("a".into(), PortDir::Input), ("io1".into(), PortDir::Inout), ("io2".into(), PortDir::Inout), ("y".into(), PortDir::Output)],
            nets: vec![
                Net { name: "a".into(), pins: vec![pin(0, "A"), Conn::Port(0)] }.into(),
                Net { name: "io1".into(), pins: vec![Conn::Port(1)] }.into(),
                Net { name: "io2".into(), pins: vec![pin(0, "B"), Conn::Port(2)] }.into(),
                Net { name: "y".into(), pins: vec![pin(0, "Y"), Conn::Port(3)] }.into(),
            ],
        };
        let pins: HashMap<String, Vec<String>> = [("and2".to_string(), vec!["Y".to_string(), "A".to_string(), "B".to_string()])].into();
        let keep = || NetlistRef::Borrowed(&nl);
        let by_cell = Graph::build_by_cell(&libs, &nl, keep(), Some(&pins)).expect("a template build");
        let by_name = Graph::build_by_name(&libs, &nl, keep(), Some(&pins), None).unwrap();
        for g in [&by_cell, &by_name] {
            let label = |v: usize| format!("{}{}", g.vname(v), if g.vertices[v].is_driver { "+" } else { "" });
            let names: Vec<String> = (0..g.vertices.len()).map(label).collect();
            assert_eq!(names, ["u1/Y+", "u1/A", "u1/B", "a+", "io1", "io1+", "io2", "io2+", "y"]);
            let wires: Vec<String> = g.edges.iter().filter(|e| matches!(e.kind, EdgeKind::Wire)).map(|e| format!("{}>{}", label(e.from), label(e.to))).collect();
            assert_eq!(wires, ["u1/Y+>y", "a+>u1/A", "io1>io1+", "io2+>u1/B", "io2>io2+"]);
            let paths: Vec<String> = g.bidirect_port_paths.iter().map(|&e| format!("{}>{}", label(g.edges[e].from), label(g.edges[e].to))).collect::<std::collections::BTreeSet<_>>().into_iter().collect();
            assert_eq!(paths, ["io1>io1+", "io2>io2+"]);
            let level = g.levels().unwrap();
            assert_eq!((level[5], level[7]), (0, 0), "a bidirect port's driver is a root, not past its load");
            // the incremental timer keeps to the vertices an edit touched: only the ports share names
            assert!(g.names_shared_only_by_bidirect_ports());
            assert_eq!(g.vertices_named(g.index()["io2"]), 6..=7);
        }
        let ends = |g: &Graph| g.edges.iter().map(|e| (e.from, e.to)).collect::<Vec<_>>();
        assert_eq!(ends(&by_cell), ends(&by_name));
        assert!(Graph::build_with_pins(&libs, &nl, Some(&pins)).is_ok(), "the repairer's build takes them");
    }

    /// Two `and2` in a chain: ports a, b into u1; u1/Y and port c into u2; u2/Y out to y.
    fn chain() -> Netlist {
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        Netlist {
            insts: vec![crate::netlist::inst("u1", "and2"), crate::netlist::inst("u2", "and2")],
            ports: vec![("a".into(), PortDir::Input), ("b".into(), PortDir::Input), ("c".into(), PortDir::Input), ("y".into(), PortDir::Output)],
            nets: vec![
                Net { name: "a".into(), pins: vec![pin(0, "A"), Conn::Port(0)] }.into(),
                Net { name: "b".into(), pins: vec![pin(0, "B"), Conn::Port(1)] }.into(),
                Net { name: "n1".into(), pins: vec![pin(0, "Y"), pin(1, "A")] }.into(),
                Net { name: "c".into(), pins: vec![pin(1, "B"), Conn::Port(2)] }.into(),
                Net { name: "y".into(), pins: vec![pin(1, "Y"), Conn::Port(3)] }.into(),
            ],
        }
    }

    /// Rule (Levelize::findLevels): roots 0; each vertex one past the highest of its fanin
    /// through searched edges — so u2/B, reached straight from port c, is 1 while u2/A is 3.
    #[test]
    fn levels_are_the_longest_path_from_a_root() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let g = Graph::build(&libs, &nl).unwrap();
        let lv = g.levels().unwrap();
        let at = |n: &str| lv[(0..g.vertices.len()).position(|v| g.vname(v) == n).unwrap()];
        assert_eq!([at("a"), at("u1/A"), at("u1/Y"), at("u2/A"), at("u2/B"), at("u2/Y"), at("y")], [0, 1, 2, 3, 1, 4, 5]);
    }

    /// Rule (`GraphDelayCalc::findDelays` under `DcalcPred`): the search leaves no constant pin,
    /// so a cell whose every input is constant is never visited and its output keeps the slew it
    /// was made with, 0 — a timed driver with no arc would hold the init values (max −1e30).
    /// u2, reached from port c, is timed as before.
    #[test]
    fn a_driver_the_delay_search_never_reaches_keeps_slew_zero() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let mut g = Graph::build(&libs, &nl).unwrap();
        g.sdc.constants.insert("u1/A".into(), false);
        g.sdc.constants.insert("u1/B".into(), false);
        g.find_delays(&HashMap::new(), None).unwrap();
        let at = |n: &str| (0..g.vertices.len()).position(|v| g.vname(v) == n).unwrap();
        assert_eq!(g.slew[at("u1/Y")], [[0.0; 2]; 2], "u1 is never visited");
        assert!(g.slew[at("u2/Y")][RISE][MAX] > 0.0, "u2 is timed from port c");
    }

    /// A combinational loop (u1/Y back into u1/A) is broken as the reference's levelization
    /// breaks it (`findBackEdges`): from root b the search goes u1/B -> u1/Y -> u1/A, and u1/A ->
    /// u1/Y, closing on the path, is disabled. Levelization, delay calculation and the searches
    /// then pass over it, and the graph orders.
    #[test]
    fn a_loop_is_broken_at_the_edge_that_closes_it() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        let nl = Netlist {
            insts: vec![crate::netlist::inst("u1", "and2")],
            ports: vec![("b".into(), PortDir::Input), ("y".into(), PortDir::Output)],
            nets: vec![
                Net { name: "b".into(), pins: vec![pin(0, "B"), Conn::Port(0)] }.into(),
                Net { name: "y".into(), pins: vec![pin(0, "Y"), pin(0, "A"), Conn::Port(1)] }.into(),
            ],
        };
        let g = Graph::build(&libs, &nl).unwrap();
        let name = |e: usize| format!("{} -> {}", g.vname(g.edges[e].from), g.vname(g.edges[e].to));
        let disabled: Vec<String> = g.loops.disabled.iter().map(|&e| name(e)).collect();
        assert_eq!(disabled, ["u1/A -> u1/Y"]);
        assert!(g.topo_order().is_ok());
        let e = g.loops.disabled[0];
        assert!(!g.levelize_thru(e) && g.levelize_thru_unlooped(e));
    }

    /// Rule (an annotated load slew): a load whose max slew exceeds its limit on either
    /// transition holds BOTH at the limit and is recorded; a limit above its slew changes nothing.
    #[test]
    fn a_load_over_its_limit_is_held_at_it() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let mut g = Graph::build(&libs, &nl).unwrap();
        let u2a = (0..g.vertices.len()).position(|v| g.vname(v) == "u2/A").unwrap();
        let u2b = (0..g.vertices.len()).position(|v| g.vname(v) == "u2/B").unwrap();
        g.slew_limit.insert(u2a, 0.5e-9);
        g.slew_limit.insert(u2b, 0.5e-9);
        g.find_delays(&HashMap::new(), None).unwrap();
        assert_eq!(g.clamped, vec![u2a], "u2/B (slew 0 from an undriven port) is under its limit");
        assert_eq!([g.slew[u2a][RISE][MAX], g.slew[u2a][FALL][MAX]], [0.5e-9, 0.5e-9]);
        // u1/Y takes the B arc's slew (newest first, see below); the lumped load inherits it.
        assert_eq!(g.slew[u2a][RISE][MIN], 1.000_000_5f32 * 1e-9, "only max is held");
    }

    /// Rule (`RepairDesign::repairDesign`'s forward pass reads `graph_->slew` of every load;
    /// `findDriverArcDelays` sets a load's slew from its driver's arcs whether or not the search
    /// goes on from the driver): u1/Y held constant (u1/A = 0, the AND's output 0) is still timed
    /// through u1/B, so its load u2/A, which the search never reaches, is held at its limit too.
    #[test]
    fn a_load_of_a_constant_driver_over_its_limit_is_held_at_it() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let mut g = Graph::build(&libs, &nl).unwrap();
        for p in ["u1/A", "u1/Y", "u2/A"] {
            g.sdc.constants.insert(p.into(), false);
        }
        let u2a = (0..g.vertices.len()).position(|v| g.vname(v) == "u2/A").unwrap();
        g.slew_limit.insert(u2a, 0.5e-9);
        g.find_delays(&HashMap::new(), None).unwrap();
        assert_eq!(g.clamped, vec![u2a]);
        assert_eq!([g.slew[u2a][RISE][MAX], g.slew[u2a][FALL][MAX]], [0.5e-9, 0.5e-9]);
    }

    /// Rule (an annotated slew): the annotation IS the max slew, even below what the calculation
    /// finds.
    #[test]
    fn an_annotated_load_slew_is_the_value() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let mut g = Graph::build(&libs, &nl).unwrap();
        let u2a = (0..g.vertices.len()).position(|v| g.vname(v) == "u2/A").unwrap();
        g.slew_annotated.insert(u2a, 2.0e-9);
        g.find_delays(&HashMap::new(), None).unwrap();
        assert_eq!([g.slew[u2a][RISE][MAX], g.slew[u2a][FALL][MAX]], [2.0e-9, 2.0e-9]);
        // `annotateLoadDelays` never merges into it: re-timing the DRIVER alone (as
        // `findDelays(drvr)` does, the load not visited) keeps the annotation; min is computed.
        let u1y = (0..g.vertices.len()).position(|v| g.vname(v) == "u1/Y").unwrap();
        let index = g.pin_index();
        g.find_vertex_delays(u1y, &HashMap::new(), &index, None).unwrap();
        assert_eq!([g.slew[u2a][RISE][MAX], g.slew[u2a][FALL][MAX]], [2.0e-9, 2.0e-9]);
        assert_eq!(g.slew[u2a][RISE][MIN], 1.000_000_5f32 * 1e-9, "only max is annotated");
        // The same from an input port driving the load (its own path through the port's drive).
        let (a, u1a) = ((0..g.vertices.len()).position(|v| g.vname(v) == "a" && g.vertices[v].is_driver).unwrap(), (0..g.vertices.len()).position(|v| g.vname(v) == "u1/A").unwrap());
        g.slew_annotated.insert(u1a, 3.0e-9);
        g.find_vertex_delays(a, &HashMap::new(), &index, None).unwrap();
        assert_eq!([g.slew[u1a][RISE][MAX], g.slew[u1a][FALL][MAX]], [3.0e-9, 3.0e-9], "kept from a port driver too");
    }

    /// Rule (`Vertex::setSlewAnnotated` / `slewAnnotated(rf, min_max)`): the annotation bits are
    /// shared by every scene, and a scene past the first folds onto the MIN bit. With scenes
    /// wc, tc, bc (the reference, `set_assigned_transition -corner <c> -max` on a load, then a
    /// load change): annotated in wc, the load's MAX slew in tc and bc stays as it was while their
    /// min slews are re-timed; annotated in bc (or tc), its bc MAX slew is re-timed over the
    /// annotation and the MIN slews of every scene stay.
    #[test]
    fn annotation_bits_are_shared_across_scenes() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let mut g = Graph::build(&libs, &nl).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let u2a = (0..g.vertices.len()).position(|v| g.vname(v) == "u2/A").unwrap();
        let u1y = (0..g.vertices.len()).position(|v| g.vname(v) == "u1/Y").unwrap();
        let timed = g.slew[u2a];
        let index = g.pin_index();
        let stale = [[7.0e-12, 8.0e-12], [9.0e-12, 6.0e-12]];
        // A later scene (tc) of a load annotated in the FIRST scene only: the max bit — its max
        // slews are left as they were, its min slews re-timed.
        g.slew_blocked = Some(std::sync::Arc::new(HashMap::from([(u2a, [false, true])])));
        g.slew[u2a] = stale;
        g.find_vertex_delays(u1y, &HashMap::new(), &index, None).unwrap();
        assert_eq!([g.slew[u2a][RISE][MAX], g.slew[u2a][FALL][MAX]], [stale[RISE][MAX], stale[FALL][MAX]]);
        assert_eq!([g.slew[u2a][RISE][MIN], g.slew[u2a][FALL][MIN]], [timed[RISE][MIN], timed[FALL][MIN]]);
        // The scene that annotated it, a later one (bc): the min bit — the driver writes the max
        // slew over the annotation, the min slews stay; visiting the load does not restore it.
        g.slew_blocked = Some(std::sync::Arc::new(HashMap::from([(u2a, [true, false])])));
        g.slew_annotated.insert(u2a, 2.0e-9);
        g.slew[u2a] = stale;
        g.find_vertex_delays(u1y, &HashMap::new(), &index, None).unwrap();
        g.find_vertex_delays(u2a, &HashMap::new(), &index, None).unwrap();
        assert_eq!([g.slew[u2a][RISE][MAX], g.slew[u2a][FALL][MAX]], [timed[RISE][MAX], timed[FALL][MAX]]);
        assert_eq!([g.slew[u2a][RISE][MIN], g.slew[u2a][FALL][MIN]], [stale[RISE][MIN], stale[FALL][MIN]]);
        // Unset (one scene): this scene's max annotation is the blocked one, as before.
        g.slew_blocked = None;
        g.find_vertex_delays(u1y, &HashMap::new(), &index, None).unwrap();
        assert_eq!([g.slew[u2a][RISE][MAX], g.slew[u2a][FALL][MAX]], [2.0e-9, 2.0e-9]);
    }

    /// Rules (Sdc::connectedCap, parasiticLoad): a net `set_load` is keyed by its DRIVER pin and
    /// adds its wire cap to the pin cap (`-subtract_pin_load` zeroes the pin cap); a port's
    /// `set_load -pin_load` is that port's pin cap.
    #[test]
    fn sdc_loads_enter_the_load_cap() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let mut g = Graph::build(&libs, &nl).unwrap();
        let v = |g: &Graph, n: &str| (0..g.vertices.len()).position(|x| g.vname(x) == n).unwrap();
        let (u1y, u2y) = (v(&g, "u1/Y"), v(&g, "u2/Y"));
        let pins = g.load_cap(u1y, &HashMap::new());
        g.sdc.net_wire_cap.insert("u1/Y".into(), [Some((0.5e-12, false)); 2]);
        assert_eq!(g.load_cap(u1y, &HashMap::new()), pins + 0.5e-12);
        g.sdc.net_wire_cap.insert("u1/Y".into(), [Some((0.5e-12, true)); 2]);
        assert_eq!(g.load_cap(u1y, &HashMap::new()), 0.5e-12);
        let before = g.load_cap(u2y, &HashMap::new());
        g.sdc.port_pin_cap.insert("y".into(), [[Some(0.2e-12); 2]; 2]);
        assert_eq!(g.load_cap(u2y, &HashMap::new()), before + 0.2e-12);
    }

    /// Rule (GraphDelayCalc::loadCap(drvr, scene, min_max)): over rise and fall, the LARGER load
    /// at max and the SMALLER at min (MinMax::minMax from its init value), each one
    /// transition's pin + wire cap at that min/max.
    #[test]
    fn the_min_load_cap_takes_the_smaller_transition() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let mut g = Graph::build(&libs, &nl).unwrap();
        let u2y = (0..g.vertices.len()).position(|x| g.vname(x) == "u2/Y").unwrap();
        // Port y's pin load: 0.5 pF rising, 0.1 pF falling, at both min and max ([rf][mm]).
        g.sdc.port_pin_cap.insert("y".into(), [[Some(0.5e-12); 2], [Some(0.1e-12); 2]]);
        let none = HashMap::new();
        let (rise_min, fall_min) = (g.load_cap_rf(u2y, &none, RISE, MIN), g.load_cap_rf(u2y, &none, FALL, MIN));
        let (rise_max, fall_max) = (g.load_cap_rf(u2y, &none, RISE, MAX), g.load_cap_rf(u2y, &none, FALL, MAX));
        assert!(fall_min < rise_min && fall_max < rise_max, "rise {rise_min} {rise_max} fall {fall_min} {fall_max}");
        assert_eq!(g.load_cap_mm(u2y, &none, MIN), fall_min);
        assert_eq!(g.load_cap_mm(u2y, &none, MAX), rise_max);
        assert_eq!(g.load_cap(u2y, &none), rise_max);
    }

    /// Rule (LumpedCapDelayCalc::findParasitic): a driver with a net `set_load` has NO parasitic —
    /// its gate is lumped at the SDC load, so a net load of 0 times the gate at 0 whatever the
    /// estimate says.
    #[test]
    fn a_net_load_drops_the_parasitic() {
        use crate::parasitics::Network;
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let mut g = Graph::build(&libs, &nl).unwrap();
        let mut par = HashMap::new();
        let net = Network { node_caps: vec![0.0, 1e-12], resistors: vec![(0, 1, 1000.0)] };
        par.insert("y".to_string(), NetParasitics { node_names: vec!["u2/Y".into(), "y".into()], network: net, ..Default::default() });
        let u2y = (0..g.vertices.len()).position(|v| g.vname(v) == "u2/Y").unwrap();
        let with_pi = g.load_cap(u2y, &par);
        assert!(with_pi > 0.9e-12);
        g.sdc.net_wire_cap.insert("u2/Y".into(), [Some((0.0, false)); 2]);
        g.find_delays(&par, None).unwrap();
        let lumped = g.slew[u2y][RISE][MAX];
        let mut g0 = Graph::build(&libs, &nl).unwrap();
        g0.find_delays(&HashMap::new(), None).unwrap();
        assert_eq!(lumped, g0.slew[u2y][RISE][MAX], "lumped at the pin cap + 0");
    }

    /// Rules (the estimator reduces at estimate time; GraphDelayCalc::parasiticLoad): a port load
    /// set AFTER the estimate is not in the pi model, which keeps the caps of the moment it was
    /// reduced; the delay calculation compares that pi model against the pin cap in force NOW and,
    /// when the pin cap is the larger, drops the pi model and times the gate lumped at the pin cap.
    #[test]
    fn a_port_load_set_after_the_estimate_is_not_in_the_pi_model() {
        use crate::parasitics::Network;
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let net = Network { node_caps: vec![0.05e-15, 0.05e-15], resistors: vec![(0, 1, 5.0)] };
        let np = |caps| NetParasitics { node_names: vec!["u2/Y".into(), "y".into()], network: net.clone(), port_pin_caps: caps, pi: None, pi_rise_only: false };
        let mut g = Graph::build(&libs, &nl).unwrap();
        g.sdc.port_pin_cap.insert("y".into(), [[Some(2e-12); 2]; 2]);
        let u2y = (0..g.vertices.len()).position(|v| g.vname(v) == "u2/Y").unwrap();
        // Estimated with no port load: the pi model is the wire alone, under the 2 pF pin cap —
        // dropped, so the load is the pin cap and no wire.
        let estimated_before: HashMap<String, NetParasitics> = [("y".to_string(), np(Some(HashMap::new())))].into();
        assert_eq!(g.load_cap_parts(u2y, &estimated_before, RISE).1, 0.0);
        assert_eq!(g.load_cap(u2y, &estimated_before), g.load_cap(u2y, &HashMap::new()));
        // Reduced against the load in force now: the pi model holds the port load and is kept,
        // adding its wire.
        let reduced_now: HashMap<String, NetParasitics> = [("y".to_string(), np(None))].into();
        assert!(g.load_cap_parts(u2y, &reduced_now, RISE).1 > 0.0);
    }

    /// Rules (the reference's graph): pins are the instance's own (a liberty-only port has no
    /// vertex); a bidirect pin is a DRIVER vertex (arcs into it) and a LOAD vertex (arcs from it);
    /// a tristate enable arc set is an edge levelization searches through.
    #[test]
    fn pad_cell_pins_bidirect_and_tristate_edges() {
        let lib = r#"library (p) { time_unit : "1ns"; capacitive_load_unit (1, pf);
          lu_table_template (t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; index_1 ("0, 1"); index_2 ("0, 1"); }
          cell (PADC) {
            pin (DATA) { direction : input; capacitance : 0.001; }
            pin (EN) { direction : input; capacitance : 0.001; }
            pin (PAD) { direction : inout; function : "DATA"; three_state : "EN";
              timing () { related_pin : "DATA"; timing_type : combinational; timing_sense : positive_unate;
                cell_rise (t) { values ("1, 1", "1, 1"); } rise_transition (t) { values ("1, 1", "1, 1"); }
                cell_fall (t) { values ("1, 1", "1, 1"); } fall_transition (t) { values ("1, 1", "1, 1"); } }
              timing () { related_pin : "EN"; timing_type : three_state_enable; } }
            pin (Y) { direction : output; function : "PAD";
              timing () { related_pin : "PAD"; timing_type : combinational; timing_sense : positive_unate;
                cell_rise (t) { values ("1, 1", "1, 1"); } rise_transition (t) { values ("1, 1", "1, 1"); }
                cell_fall (t) { values ("1, 1", "1, 1"); } fall_transition (t) { values ("1, 1", "1, 1"); } } }
            pin (NDOUT) { direction : output; function : "PAD"; } } }"#;
        let libs = [Library::read(&crate::liberty_parse::parse(lib).unwrap()).unwrap()];
        let nl = Netlist {
            insts: vec![crate::netlist::inst("p1", "PADC")],
            ports: vec![("a".into(), PortDir::Input)],
            nets: vec![Net { name: "a".into(), pins: vec![Conn::Port(0), Conn::Inst(0, "DATA".into())] }.into()],
        };
        let pins: HashMap<String, Vec<String>> = [("PADC".to_string(), vec!["DATA".into(), "EN".into(), "PAD".into(), "Y".into()])].into();
        let g = Graph::build_with_pins(&libs, &nl, Some(&pins)).unwrap();
        assert!(!(0..g.vertices.len()).any(|v| g.vname(v) == "p1/NDOUT"), "a liberty-only port has no vertex");
        let pad: Vec<usize> = (0..g.vertices.len()).filter(|&v| g.vname(v) == "p1/PAD").collect();
        assert_eq!(pad.len(), 2);
        let (drv, load) = if g.vertices[pad[0]].is_driver { (pad[0], pad[1]) } else { (pad[1], pad[0]) };
        let v = |n: &str| (0..g.vertices.len()).position(|x| g.vname(x) == n).unwrap();
        let level = g.levels().unwrap();
        assert_eq!((level[v("a")], level[v("p1/DATA")], level[drv], level[load], level[v("p1/Y")]), (0, 1, 2, 0, 1));
        assert!(g.in_edges[drv].iter().any(|&e| g.edges[e].from == v("p1/EN")), "the tristate enable edge");
        assert!(g.out_edges[load].iter().any(|&e| g.edges[e].to == v("p1/Y")));
        // A bidirect pin on a net is refused.
        let mut nl2 = nl.clone();
        nl2.nets.push(Net { name: "b".into(), pins: vec![Conn::Inst(0, "PAD".into())] }.into());
        assert!(Graph::build_with_pins(&libs, &nl2, Some(&pins)).is_err());
    }

    /// Rules (findInputArcDelay): an input port with a driving cell takes that cell's slew at the
    /// port's load — here lumped, no parasitics — and its loads take the same slew; each wire edge
    /// carries the load-dependent part of the cell's delay (`gate − intrinsic`).
    #[test]
    fn a_driving_cell_times_the_input_port() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let mut g = Graph::build(&libs, &nl).unwrap();
        g.sdc.input_drive.insert("a".into(), InputDrive { cell: "and2".into(), from_port: Some("A".into()), to_port: "Y".into(), from_slews: [0.1e-9, 0.1e-9] });
        g.find_delays(&HashMap::new(), None).unwrap();
        let v = |n: &str| (0..g.vertices.len()).position(|x| g.vname(x) == n).unwrap();
        let (a, u1a) = (v("a"), v("u1/A"));
        let cell = &libs[0].cells["and2"];
        let arc = cell.arc_sets.iter().find(|s| s.from == "A" && s.to == "Y").unwrap().arcs.iter().find(|x| x.to_rf == RISE).unwrap();
        let Model::Gate(m) = &arc.model else { panic!() };
        let load = g.load_cap(a, &HashMap::new());
        let (gate, slew) = m.gate_delay(0.1e-9, load);
        let (intrinsic, _) = m.gate_delay(0.1e-9, 0.0);
        assert_eq!(g.slew[a][RISE][MAX], slew);
        assert_eq!(g.slew[u1a][RISE][MAX], slew);
        let e = g.out_edges[a].iter().copied().find(|&e| g.edges[e].to == u1a).unwrap();
        assert_eq!(g.delay[e][RISE][MAX], gate - intrinsic);
    }

    /// Rule (seedNoDrvrCellSlew, inputPortDelay): an input port's slew is its
    /// `set_input_transition`, and a load with no Elmore delay takes that slew.
    #[test]
    fn an_input_transition_is_the_port_and_lumped_load_slew() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let mut g = Graph::build(&libs, &nl).unwrap();
        g.sdc.input_slew.insert("a".into(), [[Some(0.3e-9); 2]; 2]);
        g.find_delays(&HashMap::new(), None).unwrap();
        let v = |n: &str| (0..g.vertices.len()).position(|x| g.vname(x) == n).unwrap();
        assert_eq!(g.slew[v("a")][RISE][MAX], 0.3e-9);
        assert_eq!(g.slew[v("u1/A")][FALL][MAX], 0.3e-9);
        assert_eq!(g.slew[v("u1/B")][RISE][MAX], 0.0);
    }

    /// Rule (edges prepend to a vertex's in-edge list; merges are fuzzy): the B arc set, created
    /// SECOND, is visited FIRST, and A's slew — within 1 ppm — replaces it for
    /// neither min nor max. Visiting in creation order would keep A's 1.0 ns instead.
    #[test]
    fn a_drivers_in_edges_merge_newest_first() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let a = |p: &str| Conn::Inst(0, p.into());
        let netlist = Netlist {
            insts: vec![crate::netlist::inst("u1", "and2")],
            ports: vec![("a".into(), PortDir::Input), ("b".into(), PortDir::Input), ("y".into(), PortDir::Output)],
            nets: vec![
                Net { name: "a".into(), pins: vec![a("A"), Conn::Port(0)] }.into(),
                Net { name: "b".into(), pins: vec![a("B"), Conn::Port(1)] }.into(),
                Net { name: "y".into(), pins: vec![a("Y"), Conn::Port(2)] }.into(),
            ],
        };
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let y = (0..g.vertices.len()).position(|v| g.vname(v) == "u1/Y").unwrap();
        let b_slew = 1.000_000_5f32 * 1e-9;
        assert_ne!(b_slew, 1e-9);
        for rf in [RISE, FALL] {
            assert_eq!(g.slew[y][rf], [b_slew, b_slew], "rf {rf}");
        }
    }

    fn shape(g: &Graph<'_>) -> String {
        g.shape()
    }

    /// The template build is the by-name build: on the chain, with a physical-only instance, a
    /// net pin of a port the master lacks or the cell does not have, a master filter, and the
    /// order of instances and nets shuffled. Where a name could be shared (a repeated instance,
    /// a `/` in a port name) it hands over to the by-name build.
    #[test]
    fn the_template_build_is_the_by_name_build() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let mut nl = chain();
        nl.insts.insert(1, crate::netlist::inst("tap0", "tapcell"));
        for n in &mut nl.nets {
            for c in &mut std::sync::Arc::make_mut(n).pins {
                if let Conn::Inst(i, _) = c {
                    if *i >= 1 {
                        *i += 1;
                    }
                }
            }
        }
        nl.nets.push(Net { name: "x".into(), pins: vec![Conn::Inst(1, "VPWR".into()), Conn::Inst(0, "Q".into()), Conn::Inst(2, "Y".into())] }.into());
        nl.nets.swap(0, 3);
        let filter: HashMap<String, Vec<String>> = [("and2".to_string(), vec!["A".to_string(), "Y".to_string()])].into();
        for pins in [None, Some(&filter)] {
            let fast = Graph::build_by_cell(&libs, &nl, NetlistRef::Borrowed(&nl), pins).expect("names are unique");
            let slow = Graph::build_by_name(&libs, &nl, NetlistRef::Borrowed(&nl), pins, None).unwrap();
            assert_eq!(shape(&fast), shape(&slow));
        }
        let mut dup = chain();
        std::sync::Arc::make_mut(&mut dup.insts[1]).0 = "u1".into();
        assert!(Graph::build_by_cell(&libs, &dup, NetlistRef::Borrowed(&dup), None).is_none());
        let mut slash = chain();
        slash.ports[0].0 = "u1/A".into();
        assert!(Graph::build_by_cell(&libs, &slash, NetlistRef::Borrowed(&slash), None).is_none());
        // There the port takes the pin's name, and the by-name build's answer stands.
        assert_eq!(Graph::build(&libs, &slash).err(), Graph::build_by_name(&libs, &slash, NetlistRef::Borrowed(&slash), None, None).err());
        assert!(Graph::build(&libs, &slash).is_err());
    }

    /// A graph built from a recycled one's allocations is the graph a fresh build makes — vertex
    /// for vertex, edge for edge, with the same name index — whether the edit appended an
    /// instance (a buffer), removed one ahead of others, or changed one's cell.
    #[test]
    fn a_recycled_build_is_a_fresh_build() {
        let lib = LIB.replace("cell (and2) {", r#"cell (buf) { pin (A) { direction : input; capacitance : 0.001; }
            pin (Y) { direction : output; function : "A";
              timing () { related_pin : "A"; timing_sense : positive_unate;
                cell_rise (scalar) { values ("0.1"); } rise_transition (scalar) { values ("1.0"); }
                cell_fall (scalar) { values ("0.1"); } fall_transition (scalar) { values ("1.0"); } } } }
          cell (and2x) { pin (A) { direction : input; capacitance : 0.002; } pin (B) { direction : input; capacitance : 0.002; }
            pin (Y) { direction : output; function : "A&B"; } }
          cell (and2) {"#);
        let libs = [Library::read(&crate::liberty_parse::parse(&lib).unwrap()).unwrap()];
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        let fresh = |nl: &Netlist| {
            SPARES.with(|p| p.borrow_mut().clear());
            let g = Graph::build(&libs, nl).unwrap();
            (g.shape(), (0..g.vertices.len()).map(|i| (g.vname(i), i)).collect::<HashMap<_, _>>())
        };
        let check = |nl: &Netlist, prev: Graph<'_>| {
            let want = fresh(nl);
            prev.index();
            prev.recycle();
            let g = Graph::build(&libs, nl).unwrap();
            assert!(g.index_cache.get().is_some(), "the index came with the spare");
            assert_eq!((g.shape(), g.index().clone()), want);
        };
        // a buffer appended between u1 and u2
        let nl1 = chain();
        let mut nl2 = chain();
        nl2.insts.push(crate::netlist::inst("b1", "buf"));
        nl2.nets[2] = Net { name: "n1".into(), pins: vec![pin(0, "Y"), pin(2, "A")] }.into();
        nl2.nets.push(Net { name: "n2".into(), pins: vec![pin(2, "Y"), pin(1, "A")] }.into());
        check(&nl2, Graph::build(&libs, &nl1).unwrap());
        // u1 removed (every later instance moves up) and u2's cell changed
        let nl3 = Netlist {
            insts: vec![crate::netlist::inst("u2", "and2x"), crate::netlist::inst("b1", "buf")],
            ports: nl2.ports.clone(),
            nets: vec![
                Net { name: "n2".into(), pins: vec![pin(1, "Y"), pin(0, "A")] }.into(),
                Net { name: "c".into(), pins: vec![pin(0, "B"), Conn::Port(2)] }.into(),
                Net { name: "y".into(), pins: vec![pin(0, "Y"), Conn::Port(3)] }.into(),
                Net { name: "a".into(), pins: vec![pin(1, "A"), Conn::Port(0)] }.into(),
            ],
        };
        check(&nl3, Graph::build(&libs, &nl2).unwrap());
        // and back: a recycled graph larger than the next
        check(&nl1, Graph::build(&libs, &nl3).unwrap());
    }
}
