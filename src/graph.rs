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

use crate::dcalc::{dspf_wire_delay_slew, threshold_adjust, Dmp, LibThresholds, Thresholds};
use crate::liberty::{Cell, Direction, Library, Model, Role, FALL, MAX, MIN, RISE};
use crate::netlist::{Conn, Netlist, PortDir};
use crate::parasitics::{reduce_to_pi_elmore, Network, NodePins};

mod edit;
pub use edit::NotInPlace;

/// A vertex: one pin (a bidirect pin is refused).
#[derive(Debug, Clone)]
pub struct Vertex {
    pub name: String,
    pub conn: Conn,
    pub is_driver: bool,
    /// The library and cell of an instance pin.
    pub lib: Option<usize>,
    pub cell: Option<String>,
    pub port: Option<String>,
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

pub struct Graph<'a> {
    pub libs: &'a [Library],
    pub netlist: NetlistRef<'a>,
    pub vertices: Vec<Vertex>,
    pub edges: Vec<Edge>,
    pub in_edges: Vec<Vec<usize>>,
    pub out_edges: Vec<Vec<usize>>,
    /// The net each vertex is on, if any.
    pub vertex_net: Vec<Option<usize>>,
    /// `slew[v][rf][min/max]`.
    pub slew: Vec<[[f32; 2]; 2]>,
    /// Per edge: gate arcs `[arc][min/max]`; wire edges `[rf][min/max]`.
    pub delay: Vec<Vec<[f32; 2]>>,
    /// Load vertices whose max slew is held at a limit when it is exceeded (an annotated slew,
    /// as a resizer sets on loads it will repair, so the excess does not propagate).
    pub slew_limit: HashMap<usize, f32>,
    /// The loads [`Graph::find_delays`] held at their limit, in the order it timed them.
    pub clamped: Vec<usize>,
    /// Annotated max slews (`setAnnotatedSlew`): the vertex's max slew on both transitions IS the
    /// value, whatever the delay calculation finds — applied before its fanout reads it.
    pub slew_annotated: HashMap<usize, f32>,
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
    /// Per instance, the library and cell whose arc sets its OUTPUT-to-output edges still carry:
    /// `replaceEquivCellBefore` swaps the arc sets of the edges leaving an instance's INPUT pins
    /// only, so after an equivalent-arcs replacement an edge from one output to another (a half
    /// adder's CON -> SN) keeps the cell it was made with. See [`Graph::set_stale_out_arcs`].
    pub stale_out_arcs: HashMap<String, (usize, String)>,
    /// The edges from a bidirect pin's driver vertex into its instance (`isBidirectInstPath`,
    /// whole graph only): levelization and the activity search skip them
    /// (`bidirect_inst_paths_enabled` is off by default); delay calculation reads them.
    pub bidirect_inst_paths: std::collections::HashSet<usize>,
    /// The wire edge from a bidirect top port's load vertex to its driver vertex
    /// (`isBidirectPortPath`, whole graph only): levelization skips it, so the driver is a root;
    /// the activity search crosses it.
    pub bidirect_port_paths: std::collections::HashSet<usize>,
    /// Every vertex's index by name, built on first use: the vertices never change once the graph
    /// is built, and [`Graph::load_cap`] and its kin are called per driver by a repair loop.
    /// Shared, so a caller holding it ([`Graph::pin_index`]) does not copy every name.
    index_cache: std::sync::OnceLock<std::sync::Arc<HashMap<String, usize>>>,
    /// Each net's slot in [`Graph::netlist`] by name, kept across in-place edits (an edit replaces
    /// nets in their slots or appends them): [`Graph::apply_edits`] finds the nets its edits named
    /// without a pass over every net. Checked by name at each slot read; made again on a miss.
    net_slots: Option<HashMap<String, usize>>,
    /// The caller's: how far into its log of netlist edits this graph is ([`Graph::apply_edits`]
    /// brings it on). `None` from a build — not known, so a caller builds again.
    pub edit_seq: Option<usize>,
    /// The timer's: the state it imported, as its stamp and how far into its log of written names
    /// ([`crate::incr::IncTimer::import`] brings it on). `None` from a build.
    pub state_seq: Option<(u64, usize)>,
}

/// [`Graph::parasitic_load`]: `(pin_cap, wire_cap, the pi model and its node names)`.
type ParasiticLoad<'p> = (f32, f32, Option<(crate::parasitics::PiElmore, &'p [String])>);

/// The SDC loads and drives delay calculation reads (`Sdc::connectedCap`, `seedDrvrSlew`).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SdcEnv {
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
        let Graph { vertices, edges, in_edges, out_edges, delay, index_cache, .. } = self;
        let mut ic = index_cache;
        let spare = Spare { vertices, edges, in_edges, out_edges, delay, index: ic.take() };
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
            eprintln!("VYGGV|{}|{}|0", i + 1, v.name);
        }
        for (i, e) in self.edges.iter().enumerate() {
            let wire = u8::from(matches!(e.kind, EdgeKind::Wire));
            eprintln!("VYGGE|{}|{}|{}|{wire}|{}|{}", i + 1, e.from + 1, e.to + 1, self.vertices[e.from].name, self.vertices[e.to].name);
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
                    if ix.len() != g.vertices.len() || g.vertices.iter().enumerate().any(|(i, v)| ix.get(&v.name) != Some(&i)) {
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
                        let mut t = Template { lib, verts: Vec::new(), first: HashMap::new(), edges: Vec::new() };
                        // The vertex an arc set goes TO and the one it comes FROM: the same for
                        // every pin but a bidirect one, which is refused here.
                        let mut at: HashMap<&str, usize> = HashMap::new();
                        for p in &c.ports {
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
                            if set.role == Role::Other || (set.arcs.is_empty() && !tristate) {
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
        // The instance a block's vertices belong to: its first vertex's name less `/port`.
        fn block_inst(old: &[Option<Vertex>], (start, _): (usize, usize)) -> Option<&str> {
            let v = old[start].as_ref()?;
            let p = v.port.as_deref()?;
            v.name.len().checked_sub(p.len() + 1).and_then(|n| v.name.get(..n))
        }
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
                        old[start + j].as_ref().is_some_and(|v| v.port.as_deref() == Some(port) && v.is_driver == is_driver && v.lib == Some(t.lib) && v.cell.as_deref() == Some(cname.as_str()))
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
                        vertices.push(Vertex { name: format!("{name}/{port}"), conn: Conn::Inst(i, port.to_string()), is_driver, lib: Some(t.lib), cell: Some(cname.clone()), port: Some(port.to_string()) });
                        reused_from.push(None);
                    }
                }
            }
        }
        let port_base = vertices.len();
        let mut pc = 0usize;
        for (i, (name, dir)) in netlist.ports.iter().enumerate() {
            if *dir == PortDir::Inout {
                return None;
            }
            let is_driver = *dir == PortDir::Input;
            let reuse = old_ports.get(pc).copied().filter(|&k| old[k].as_ref().is_some_and(|v| v.name == *name && v.is_driver == is_driver && matches!(v.conn, Conn::Port(_))));
            match reuse {
                Some(k) => {
                    let mut v = old[k].take().expect("a matching port");
                    v.conn = Conn::Port(i);
                    reused_from.push(Some(k));
                    vertices.push(v);
                    pc += 1;
                }
                None => {
                    vertices.push(Vertex { name: name.clone(), conn: Conn::Port(i), is_driver, lib: None, cell: None, port: None });
                    reused_from.push(None);
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
        let mut vertex_net = vec![None; vertices.len()];
        let mut vs = Vec::new();
        for (n, net) in netlist.nets.iter().enumerate() {
            vs.clear();
            vs.extend(net.pins.iter().filter_map(|c| match c {
                Conn::Inst(i, p) => tpl(*i).and_then(|t| t.first.get(p.as_str())).map(|&l| base[*i] + l),
                Conn::Port(i) => Some(port_base + i),
            }));
            for &v in &vs {
                vertex_net[v] = Some(n);
            }
            let mut drivers = vs.iter().copied().filter(|&v| vertices[v].is_driver);
            let Some(d) = drivers.next() else { continue };
            if drivers.next().is_some() {
                return None;
            }
            for &l in vs.iter().filter(|&&v| !vertices[v].is_driver) {
                edges.push(Edge { from: d, to: l, kind: EdgeKind::Wire });
                delay.push(row(2));
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
        if let Some(mut map) = spare.and_then(|s| s.index).and_then(|a| std::sync::Arc::try_unwrap(a).ok()) {
            for v in old.iter().flatten() {
                map.remove(v.name.as_str());
            }
            for (p, (v, from)) in vertices.iter().zip(&reused_from).enumerate() {
                match from {
                    Some(q) if *q == p => {}
                    _ => match map.get_mut(v.name.as_str()) {
                        Some(x) => *x = p,
                        None => {
                            map.insert(v.name.clone(), p);
                        }
                    },
                }
            }
            if map.len() == n {
                let _ = index_cache.set(std::sync::Arc::new(map));
            }
        }
        Some(Graph { libs, netlist: keep, vertices, edges, in_edges, out_edges, vertex_net, slew: vec![[[0.0; 2]; 2]; n], delay, slew_limit: HashMap::new(), clamped: Vec::new(), slew_annotated: HashMap::new(), sdc: SdcEnv::default(), pin_cap_cell: HashMap::new(), ideal_clock: std::collections::HashSet::new(), stale_out_arcs: HashMap::new(), bidirect_inst_paths: std::collections::HashSet::new(), bidirect_port_paths: std::collections::HashSet::new(), index_cache, net_slots: None, edit_seq: None, state_seq: None })
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
                        vertices.push(Vertex { name: vname.clone(), conn: Conn::Inst(i, port.clone()), is_driver, lib: None, cell: Some(cname.clone()), port: Some(port.clone()) });
                    }
                }
                continue;
            };
            let has = pins.and_then(|m| m.get(cname));
            let c = cell(libs, lib, cname);
            // The whole graph makes an instance's vertices in the network's pin order (the
            // master's terminals, `makePinVertices` over the instance's pins), not the liberty's.
            let ordered: Vec<&crate::liberty::Port> = match has.filter(|_| whole) {
                Some(h) => h.iter().filter_map(|n| c.port(n)).collect(),
                None => c.ports.iter().collect(),
            };
            for p in ordered {
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
                    vertices.push(Vertex { name: vname.clone(), conn: Conn::Inst(i, p.name.clone()), is_driver, lib: Some(lib), cell: Some(cname.clone()), port: Some(p.name.clone()) });
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
                if !whole {
                    return Err(format!("port {name}: bidirect ports are not modelled"));
                }
                index.insert(name.clone(), vertices.len());
                for is_driver in [false, true] {
                    vertices.push(Vertex { name: name.clone(), conn: Conn::Port(i), is_driver, lib: None, cell: None, port: None });
                }
                continue;
            }
            let v = vertices.len();
            index.insert(name.clone(), v);
            vertices.push(Vertex { name: name.clone(), conn: Conn::Port(i), is_driver: *dir == PortDir::Input, lib: None, cell: None, port: None });
        }
        // A pin's vertices: its first (the load of a bidirect pin), then a bidirect pin's driver.
        let pin_vertices = |first: usize| -> std::ops::Range<usize> {
            let two = vertices.get(first + 1).is_some_and(|w: &Vertex| w.name == vertices[first].name && w.conn == vertices[first].conn);
            first..first + 1 + usize::from(two)
        };
        let mut edges = Vec::new();
        let mut bidirect_inst_paths = std::collections::HashSet::new();
        for (i, (_, cname)) in netlist.insts.iter().map(|i| &**i).enumerate() {
            let Some(lib) = find_cell(cname) else { continue };
            for (k, set) in cell(libs, lib, cname).arc_sets.iter().enumerate() {
                let tristate = matches!(set.role, Role::TristateEnable | Role::TristateDisable);
                if set.role == Role::Other || (set.arcs.is_empty() && !tristate) {
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
                let check = matches!(set.role, Role::Setup | Role::Hold | Role::Recovery | Role::Removal);
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
        let mut vertex_net = vec![None; vertices.len()];
        for (n, net) in netlist.nets.iter().enumerate().filter(|_| whole) {
            // `makeWireEdgesFromPin`: every driver to every load of the net but itself; a
            // bidirect pin is both (its driver vertex drives, its load vertex loads).
            let mut buf = String::new();
            let vs: Vec<usize> = net.pins.iter().filter_map(|c| index.get(netlist.pin_name_into(c, &mut buf)).copied()).flat_map(pin_vertices).collect();
            for &v in &vs {
                vertex_net[v] = Some(n);
            }
            let drivers: Vec<usize> = vs.iter().copied().filter(|&v| vertices[v].is_driver).collect();
            let loads: Vec<usize> = vs.iter().copied().filter(|&v| !vertices[v].is_driver).collect();
            // `isIsolatedNet`: ten or more drivers, none a port or with fanin, and loads with
            // no fanout or check: a pad ring's tied pins, given no edge.
            let fanin = |v: usize| edges.iter().any(|e: &Edge| e.to == v);
            let is_check = |e: &Edge| match e.kind {
                EdgeKind::Gate { set } => {
                    let x = &vertices[e.to];
                    x.lib.is_some_and(|l| matches!(cell(libs, l, x.cell.as_deref().unwrap_or("")).arc_sets[set].role, Role::Setup | Role::Hold | Role::Recovery | Role::Removal))
                }
                EdgeKind::Wire => false,
            };
            let fanout_or_check = |v: usize| edges.iter().any(|e: &Edge| e.from == v || (e.to == v && is_check(e)));
            if drivers.len() >= 10 && drivers.iter().all(|&d| !matches!(vertices[d].conn, Conn::Port(_)) && !fanin(d)) && loads.iter().all(|&l| !fanout_or_check(l)) {
                continue;
            }
            for &d in &drivers {
                for &l in &loads {
                    if vertices[l].conn != vertices[d].conn {
                        edges.push(Edge { from: d, to: l, kind: EdgeKind::Wire });
                    }
                }
            }
        }
        // `makeInstDrvrWireEdges` on the top instance: a bidirect port's load vertex reaches its
        // driver vertex (a bidirect port path).
        let mut bidirect_port_paths = std::collections::HashSet::new();
        for (i, (name, dir)) in netlist.ports.iter().enumerate().filter(|_| whole) {
            if *dir == PortDir::Inout {
                let first = index[name];
                debug_assert_eq!(vertices[first].conn, Conn::Port(i));
                bidirect_port_paths.insert(edges.len());
                edges.push(Edge { from: first, to: first + 1, kind: EdgeKind::Wire });
            }
        }
        for (n, net) in netlist.nets.iter().enumerate().filter(|_| !whole) {
            let mut buf = String::new();
            let vs: Vec<usize> = net.pins.iter().filter_map(|c| index.get(netlist.pin_name_into(c, &mut buf)).copied()).collect();
            for &v in &vs {
                vertex_net[v] = Some(n);
            }
            let drivers: Vec<usize> = vs.iter().copied().filter(|&v| vertices[v].is_driver).collect();
            if drivers.len() > 1 {
                return Err(format!("net {}: multiple drivers are not modelled", net.name));
            }
            for &d in &drivers {
                for &l in vs.iter().filter(|&&v| !vertices[v].is_driver) {
                    edges.push(Edge { from: d, to: l, kind: EdgeKind::Wire });
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
        Ok(Graph { libs, netlist: keep, vertices, edges, in_edges, out_edges, vertex_net, slew: vec![[[0.0; 2]; 2]; n], delay, slew_limit: HashMap::new(), clamped: Vec::new(), slew_annotated: HashMap::new(), sdc: SdcEnv::default(), pin_cap_cell: HashMap::new(), ideal_clock: std::collections::HashSet::new(), stale_out_arcs: HashMap::new(), bidirect_inst_paths, bidirect_port_paths, index_cache: std::sync::OnceLock::new(), net_slots: None, edit_seq: None, state_seq: None })
    }

    /// Whether vertex `v` is a top-level port's (an instance's vertex may have no library too:
    /// [`Graph::build_with_unlinked`]).
    pub fn is_port(&self, v: usize) -> bool {
        matches!(self.vertices[v].conn, Conn::Port(_))
    }

    pub fn is_check(&self, e: usize) -> bool {
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
                        return &cell(self.libs, *lib, name).arc_sets[set];
                    }
                }
            }
        }
        &cell(self.libs, v.lib.unwrap(), v.cell.as_deref().unwrap()).arc_sets[set]
    }

    /// [`Graph::stale_out_arcs`] from instance → cell name, each cell found in the graph's libraries
    /// (the first holding it); an instance whose cell is in none is left with its own.
    pub fn set_stale_out_arcs(&mut self, cells: &std::collections::BTreeMap<String, String>) {
        self.stale_out_arcs = cells.iter().filter_map(|(inst, name)| self.libs.iter().position(|l| l.cells.contains_key(name)).map(|lib| (inst.clone(), (lib, name.clone())))).collect();
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
            match &x.cell { Some(c) => format!("{} ({c})", x.name), None => x.name.clone() }
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
    fn time_input_drive(&mut self, v: usize, wires: &[usize], drive: &InputDrive, parasitics: &HashMap<String, NetParasitics>, index: &HashMap<String, usize>) -> Result<(), String> {
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
                    self.slew[v][rf][mm] = drvr_slew as f32;
                    let load_delay = gate_delay as f32 - intrinsic;
                    for &w in wires {
                        let load = self.edges[w].to;
                        let (mut wire_delay, mut load_slew) = match (&mut dmp, &pe) {
                            (Some(d), Some((p, names))) => match p.elmore.iter().find(|(n, _)| names[*n] == self.vertices[load].name) {
                                Some(&(_, el)) => d.load_delay_slew(f64::from(el)),
                                None => (0.0, drvr_slew),
                            },
                            _ => (0.0, drvr_slew),
                        };
                        let ll = self.threshold_library(load);
                        threshold_adjust(ll == li, &self.lib_thresholds(li, rf), &self.lib_thresholds(ll, rf), rf == RISE, &mut wire_delay, &mut load_slew);
                        self.slew[load][rf][mm] = match self.slew_annotated.get(&load) {
                            Some(&a) if mm == MAX => a,
                            _ => load_slew as f32,
                        };
                        self.delay[w][rf][mm] = wire_delay as f32 + load_delay;
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
        if let (Some((l, c)), Some(p)) = (self.pin_cap_cell.get(&v), &self.vertices[v].port) {
            return cell(self.libs, *l, c).port(p).map_or(0.0, |p| p.capacitance(rf, mm));
        }
        match (&self.vertices[v].lib, &self.vertices[v].cell, &self.vertices[v].port) {
            (Some(l), Some(c), Some(p)) => cell(self.libs, *l, c).port(p).map_or(0.0, |p| p.capacitance(rf, mm)),
            _ => match self.vertices[v].conn {
                Conn::Port(_) => self.sdc.port_pin_cap.get(&self.vertices[v].name).and_then(|c| c[rf][mm]).unwrap_or(0.0),
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
        let net_load = self.sdc.net_wire_cap.get(&self.vertices[drvr].name).and_then(|c| c[mm]);
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
        let mut parasitic = if self.sdc.net_wire_cap.contains_key(&self.vertices[drvr].name) { None } else { self.reduced(drvr, rf, mm, parasitics, index) };
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
        let d = np.node_names.iter().position(|x| x == &self.vertices[drvr].name)?;
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
        let index = self.index();
        let mut load_cap = -1e30f32;
        for rf in [RISE, FALL] {
            let (pin_cap, wire_cap, _) = self.parasitic_load(drvr, rf, MAX, parasitics, index);
            let cap = pin_cap + wire_cap;
            if cap > load_cap {
                load_cap = cap;
            }
        }
        load_cap
    }

    /// Delay calculation over the whole graph. With `trace`, one line per DMP gate call
    /// (`dcalc|gate|drvr|rf pair|in_slew|c2|rpi|c1|alg|rd|t0|dt|ceff|valid|delay|slew`, bits in hex)
    /// and per load (`dcalc|load|drvr|load|wire delay|load slew`).
    pub fn find_delays(&mut self, parasitics: &HashMap<String, NetParasitics>, mut trace: Option<&mut Vec<String>>) -> Result<(), String> {
        let order = self.topo_order()?;
        self.clamped.clear();
        let index = self.index().clone();
        let reached = self.dcalc_reached(&order);
        for &v in &order {
            if reached[v] {
                self.find_vertex_delays(v, parasitics, &index, trace.as_deref_mut())?;
            }
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
            EdgeKind::Gate { set } => !self.is_check(e) && self.arc_set(e, set).role != Role::RegSetClr && !self.bidirect_inst_paths.contains(&e),
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

    pub fn is_constant(&self, v: usize) -> bool {
        !self.sdc.constants.is_empty() && self.sdc.constants.contains_key(&self.vertices[v].name)
    }

    /// Every vertex's index by name (the parasitics name their nodes by pin).
    pub fn pin_index(&self) -> std::sync::Arc<HashMap<String, usize>> {
        self.index_cache.get_or_init(|| std::sync::Arc::new(self.vertices.iter().enumerate().map(|(i, v)| (v.name.clone(), i)).collect())).clone()
    }

    /// [`Graph::pin_index`], built once and borrowed.
    pub fn index(&self) -> &HashMap<String, usize> {
        self.index_cache.get_or_init(|| std::sync::Arc::new(self.vertices.iter().enumerate().map(|(i, v)| (v.name.clone(), i)).collect()))
    }

    /// One vertex of the delay calculation (`findVertexDelay`): a load takes its root slew, its
    /// annotation or its limit; an input port its drive; a driver every arc of every in-edge,
    /// its slew and its loads' slews and wire delays merged from the initial values.
    pub fn find_vertex_delays(&mut self, v: usize, parasitics: &HashMap<String, NetParasitics>, index: &HashMap<String, usize>, mut trace: Option<&mut Vec<String>>) -> Result<(), String> {
        const INIT: [f32; 2] = [1e30, -1e30];
        // Fuzzily worse for this min/max, so the merge order matters.
        let worse = |mm: usize, a: f32, b: f32| if mm == MAX { crate::fuzzy::greater(a, b) } else { crate::fuzzy::less(a, b) };
        let fanin: Vec<usize> = self.in_edges[v].iter().copied().filter(|&e| !self.is_check(e)).collect();
        let wires: Vec<usize> = self.out_edges[v].iter().copied().filter(|&e| matches!(self.edges[e].kind, EdgeKind::Wire)).collect();
        if !self.vertices[v].is_driver {
            if fanin.is_empty() {
                // A root load's slew is 0.
                self.slew[v] = [[0.0; 2]; 2];
            } else if let Some(&value) = self.slew_annotated.get(&v) {
                self.slew[v][RISE][MAX] = value;
                self.slew[v][FALL][MAX] = value;
            } else if let Some(&limit) = self.slew_limit.get(&v) {
                // Its driver set its slew; before its fanout reads it, an excess on either
                // transition holds BOTH max slews at the limit.
                if self.slew[v][RISE][MAX] > limit || self.slew[v][FALL][MAX] > limit {
                    self.slew[v][RISE][MAX] = limit;
                    self.slew[v][FALL][MAX] = limit;
                    self.clamped.push(v);
                }
            }
            return Ok(());
        }
        // `findVertexDelay`: a root, or a top-level bidirect port's driver vertex (its load's wire
        // into it aside), is seeded as an input.
        if fanin.is_empty() || self.is_port(v) {
            if let Some(drive) = self.sdc.input_drive.get(&self.vertices[v].name).cloned() {
                self.time_input_drive(v, &wires, &drive, parasitics, index)?;
                return Ok(());
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
                    let in_slew = self.sdc.input_slew.get(&self.vertices[v].name).and_then(|s| s[rf][mm]).unwrap_or(0.0);
                    self.slew[v][rf][mm] = annotated.unwrap_or(in_slew);
                    let (_, _, pe) = self.parasitic_load(v, rf, mm, parasitics, index);
                    for &e in &wires {
                        let load = self.edges[e].to;
                        let mut wire_delay = 0.0f64;
                        let mut load_slew = f64::from(in_slew);
                        let elmore = pe.as_ref().and_then(|(p, names)| p.elmore.iter().find(|(n, _)| names[*n] == self.vertices[load].name).map(|(_, e)| *e));
                        let ll = self.threshold_library(load);
                        if let Some(el) = elmore {
                            let l = &self.libs[ll];
                            let th = Thresholds { vth: l.input_threshold[rf], vl: l.slew_lower_threshold[rf], vh: l.slew_upper_threshold[rf], slew_derate: l.slew_derate };
                            (wire_delay, load_slew) = dspf_wire_delay_slew(f64::from(in_slew), el, &th);
                        }
                        threshold_adjust(ll == 0, &self.lib_thresholds(0, rf), &self.lib_thresholds(ll, rf), rf == RISE, &mut wire_delay, &mut load_slew);
                        // A load's own annotation stands (`annotateLoadDelays`), as from a cell driver.
                        self.slew[load][rf][mm] = match self.slew_annotated.get(&load) {
                            Some(&a) if mm == MAX => a,
                            _ => annotated.unwrap_or(load_slew as f32),
                        };
                        self.delay[e][rf][mm] = wire_delay as f32;
                    }
                }
            }
            return Ok(());
        }
        // Driver delays: init, then every arc of every in-edge.
        let drvr_lib = self.vertices[v].lib.expect("an instance driver");
        // `annotateLoadDelays` merges into a load only where its slew is not annotated: an annotated
        // load (`setAnnotatedSlew`, max rise and fall) keeps its annotation through its driver's
        // timing, not just after its own visit.
        for &e in &wires {
            let load = self.edges[e].to;
            self.slew[load] = [[INIT[MIN], INIT[MAX]], [INIT[MIN], INIT[MAX]]];
            if let Some(&a) = self.slew_annotated.get(&load) {
                self.slew[load][RISE][MAX] = a;
                self.slew[load][FALL][MAX] = a;
            }
            self.delay[e] = vec![[INIT[MIN], INIT[MAX]]; 2];
        }
        self.slew[v] = [[INIT[MIN], INIT[MAX]], [INIT[MIN], INIT[MAX]]];
        let mut exists = [false; 2];
        // `parasiticLoad` depends on the driver, the transition and min/max only, not on the arc:
        // reduced once per pair, on first use, rather than once per arc.
        let mut loads: [[Option<ParasiticLoad>; 2]; 2] = Default::default();
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
            if self.is_constant(from) || self.arc_set(e, set).role == Role::RegSetClr {
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
                    let in_slew = if ideal_clk_to_q { 0.0 } else { self.slew[from][arc.from_rf][mm] };
                    if loads[rf][mm].is_none() {
                        loads[rf][mm] = Some(self.parasitic_load(v, rf, mm, parasitics, index));
                    }
                    let (pin_cap, wire_cap, pe) = loads[rf][mm].as_ref().expect("set above");
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
                    if let (Some(t), Some(d), Some((p, _))) = (trace.as_deref_mut(), &dmp, &pe) {
                        let (rd, t0, dt, ceff) = d.state();
                        let alg = match d.alg {
                            crate::dcalc::Alg::Cap => "cap",
                            crate::dcalc::Alg::Pi => "Pi",
                            crate::dcalc::Alg::ZeroC2 => "c2=0",
                        };
                        let rfc = |r: usize| if r == RISE { '^' } else { 'v' };
                        t.push(format!("dcalc|gate|{}|{}{}|{:08x}|{:08x}|{:08x}|{:08x}|{alg}|{:016x}|{:016x}|{:016x}|{:016x}|{}|{:016x}|{:016x}", self.vertices[v].name, rfc(arc.from_rf), rfc(rf), in_slew.to_bits(), p.c2.to_bits(), p.rpi.to_bits(), p.c1.to_bits(), rd.to_bits(), t0.to_bits(), dt.to_bits(), ceff.to_bits(), i32::from(d.driver_valid()), gate_delay.to_bits(), drvr_slew.to_bits()));
                    }
                    let (gd32, ds32) = (gate_delay as f32, drvr_slew as f32);
                    if worse(mm, ds32, self.slew[v][rf][mm]) {
                        self.slew[v][rf][mm] = ds32;
                    }
                    self.delay[e][k][mm] = gd32;
                    for &w in &wires {
                        let load = self.edges[w].to;
                        let (mut wire_delay, mut load_slew) = match (&mut dmp, &pe) {
                            (Some(d), Some((p, names))) => match p.elmore.iter().find(|(n, _)| names[*n] == self.vertices[load].name) {
                                Some(&(_, el)) => d.load_delay_slew(f64::from(el)),
                                None => (0.0, drvr_slew),
                            },
                            _ => (0.0, f64::from(ds32)),
                        };
                        if let (Some(t), Some(_)) = (trace.as_deref_mut(), &dmp) {
                            t.push(format!("dcalc|load|{}|{}|{:016x}|{:016x}", self.vertices[v].name, self.vertices[load].name, wire_delay.to_bits(), load_slew.to_bits()));
                        }
                        let ll = self.threshold_library(load);
                        threshold_adjust(ll == drvr_lib, &self.lib_thresholds(drvr_lib, rf), &self.lib_thresholds(ll, rf), rf == RISE, &mut wire_delay, &mut load_slew);
                        if worse(mm, load_slew as f32, self.slew[load][rf][mm]) && !(mm == MAX && self.slew_annotated.contains_key(&load)) {
                            self.slew[load][rf][mm] = load_slew as f32;
                        }
                        if worse(mm, wire_delay as f32, self.delay[w][rf][mm]) {
                            self.delay[w][rf][mm] = wire_delay as f32;
                        }
                    }
                }
            }
        }
        for rf in [RISE, FALL] {
            if !exists[rf] {
                for mm in [MIN, MAX] {
                    self.slew[v][rf][mm] = INIT[mm];
                    for &w in &wires {
                        self.delay[w][rf][mm] = 0.0;
                        let load = self.edges[w].to;
                        if !(mm == MAX && self.slew_annotated.contains_key(&load)) {
                            self.slew[load][rf][mm] = 0.0;
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// One timing check edge's delays (`findCheckEdgeDelays`), from the slews at both ends.
    pub fn find_check_edge_delays(&mut self, e: usize) {
        let EdgeKind::Gate { set } = self.edges[e].kind else { return };
        let arcs = &self.arc_set(e, set).arcs;
        let (from, to) = (self.edges[e].from, self.edges[e].to);
        for (k, arc) in arcs.iter().enumerate() {
            let Model::Check(table) = &arc.model else { continue };
            for mm in [MIN, MAX] {
                let clk_slew = if self.ideal_clock.contains(&from) { 0.0 } else { self.slew[from][arc.from_rf][1 - mm] };
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::netlist::{Conn, Net, Netlist, PortDir};

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
        let names: Vec<&str> = g.vertices.iter().map(|v| v.name.as_str()).collect();
        assert_eq!(names, ["x/A", "x/Y", "u1/A", "u1/B", "u1/Y", "a", "y"]);
        let y = 1;
        assert!(g.vertices[y].is_driver && g.in_edges[y].is_empty(), "a root driver");
        assert!(!g.is_port(y) && g.vertices[y].lib.is_none());
        let edges: Vec<(&str, &str, bool)> = g.edges.iter().map(|e| (names[e.from], names[e.to], matches!(e.kind, EdgeKind::Wire))).collect();
        assert_eq!(edges, [("u1/A", "u1/Y", false), ("u1/B", "u1/Y", false), ("a", "x/A", true), ("a", "u1/B", true), ("x/Y", "u1/A", true), ("u1/Y", "y", true)]);
        let plain = Graph::build(&libs, &nl).unwrap();
        assert!(plain.vertices.iter().all(|v| !v.name.starts_with("x/")));
        // the master's terminal order, not the liberty's (the plain build keeps the liberty's)
        let pins: HashMap<String, Vec<String>> = [("and2".to_string(), vec!["Y".to_string(), "B".to_string(), "A".to_string()])].into();
        let g = Graph::build_with_unlinked(&libs, &nl, Some(&pins), &unlinked).unwrap();
        let names: Vec<&str> = g.vertices.iter().map(|v| v.name.as_str()).collect();
        assert_eq!(names, ["x/A", "x/Y", "u1/Y", "u1/B", "u1/A", "a", "y"]);
        let plain = Graph::build_with_pins(&libs, &nl, Some(&pins)).unwrap();
        assert_eq!(plain.vertices.iter().position(|v| v.name == "u1/A"), Some(0));
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
        let label = |v: usize| format!("{}{}", g.vertices[v].name, if g.vertices[v].is_driver { "+" } else { "" });
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
        assert!(Graph::build(&libs, &nl).is_err());
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
        let at = |n: &str| lv[g.vertices.iter().position(|v| v.name == n).unwrap()];
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
        let at = |n: &str| g.vertices.iter().position(|v| v.name == n).unwrap();
        assert_eq!(g.slew[at("u1/Y")], [[0.0; 2]; 2], "u1 is never visited");
        assert!(g.slew[at("u2/Y")][RISE][MAX] > 0.0, "u2 is timed from port c");
    }

    /// A combinational loop (u1/Y back into u1/A) is refused, and the refusal names the loop's
    /// pins in edge order with their cells.
    #[test]
    fn a_loop_is_refused_naming_its_pins() {
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
        let err = g.topo_order().unwrap_err();
        assert!(err.starts_with("the timing graph has a loop — not modelled: "), "{err}");
        let cycle: Vec<&str> = err.rsplit(": ").next().unwrap().split(" -> ").collect();
        assert_eq!(cycle.len(), 2, "{err}");
        assert!(cycle.contains(&"u1/A (and2)") && cycle.contains(&"u1/Y (and2)"), "{err}");
    }

    /// Rule (an annotated load slew): a load whose max slew exceeds its limit on either
    /// transition holds BOTH at the limit and is recorded; a limit above its slew changes nothing.
    #[test]
    fn a_load_over_its_limit_is_held_at_it() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let mut g = Graph::build(&libs, &nl).unwrap();
        let u2a = g.vertices.iter().position(|v| v.name == "u2/A").unwrap();
        let u2b = g.vertices.iter().position(|v| v.name == "u2/B").unwrap();
        g.slew_limit.insert(u2a, 0.5e-9);
        g.slew_limit.insert(u2b, 0.5e-9);
        g.find_delays(&HashMap::new(), None).unwrap();
        assert_eq!(g.clamped, vec![u2a], "u2/B (slew 0 from an undriven port) is under its limit");
        assert_eq!([g.slew[u2a][RISE][MAX], g.slew[u2a][FALL][MAX]], [0.5e-9, 0.5e-9]);
        // u1/Y takes the B arc's slew (newest first, see below); the lumped load inherits it.
        assert_eq!(g.slew[u2a][RISE][MIN], 1.000_000_5f32 * 1e-9, "only max is held");
    }

    /// Rule (an annotated slew): the annotation IS the max slew, even below what the calculation
    /// finds.
    #[test]
    fn an_annotated_load_slew_is_the_value() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let mut g = Graph::build(&libs, &nl).unwrap();
        let u2a = g.vertices.iter().position(|v| v.name == "u2/A").unwrap();
        g.slew_annotated.insert(u2a, 2.0e-9);
        g.find_delays(&HashMap::new(), None).unwrap();
        assert_eq!([g.slew[u2a][RISE][MAX], g.slew[u2a][FALL][MAX]], [2.0e-9, 2.0e-9]);
        // `annotateLoadDelays` never merges into it: re-timing the DRIVER alone (as
        // `findDelays(drvr)` does, the load not visited) keeps the annotation; min is computed.
        let u1y = g.vertices.iter().position(|v| v.name == "u1/Y").unwrap();
        let index = g.pin_index();
        g.find_vertex_delays(u1y, &HashMap::new(), &index, None).unwrap();
        assert_eq!([g.slew[u2a][RISE][MAX], g.slew[u2a][FALL][MAX]], [2.0e-9, 2.0e-9]);
        assert_eq!(g.slew[u2a][RISE][MIN], 1.000_000_5f32 * 1e-9, "only max is annotated");
        // The same from an input port driving the load (its own path through the port's drive).
        let (a, u1a) = (g.vertices.iter().position(|v| v.name == "a" && v.is_driver).unwrap(), g.vertices.iter().position(|v| v.name == "u1/A").unwrap());
        g.slew_annotated.insert(u1a, 3.0e-9);
        g.find_vertex_delays(a, &HashMap::new(), &index, None).unwrap();
        assert_eq!([g.slew[u1a][RISE][MAX], g.slew[u1a][FALL][MAX]], [3.0e-9, 3.0e-9], "kept from a port driver too");
    }

    /// Rules (Sdc::connectedCap, parasiticLoad): a net `set_load` is keyed by its DRIVER pin and
    /// adds its wire cap to the pin cap (`-subtract_pin_load` zeroes the pin cap); a port's
    /// `set_load -pin_load` is that port's pin cap.
    #[test]
    fn sdc_loads_enter_the_load_cap() {
        let libs = [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let nl = chain();
        let mut g = Graph::build(&libs, &nl).unwrap();
        let v = |g: &Graph, n: &str| g.vertices.iter().position(|x| x.name == n).unwrap();
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
        let u2y = g.vertices.iter().position(|v| v.name == "u2/Y").unwrap();
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
        let u2y = g.vertices.iter().position(|v| v.name == "u2/Y").unwrap();
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
        assert!(!g.vertices.iter().any(|v| v.name == "p1/NDOUT"), "a liberty-only port has no vertex");
        let pad: Vec<usize> = (0..g.vertices.len()).filter(|&v| g.vertices[v].name == "p1/PAD").collect();
        assert_eq!(pad.len(), 2);
        let (drv, load) = if g.vertices[pad[0]].is_driver { (pad[0], pad[1]) } else { (pad[1], pad[0]) };
        let v = |n: &str| g.vertices.iter().position(|x| x.name == n).unwrap();
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
        let v = |n: &str| g.vertices.iter().position(|x| x.name == n).unwrap();
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
        let v = |n: &str| g.vertices.iter().position(|x| x.name == n).unwrap();
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
        let y = g.vertices.iter().position(|v| v.name == "u1/Y").unwrap();
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
            (g.shape(), g.vertices.iter().enumerate().map(|(i, v)| (v.name.clone(), i)).collect::<HashMap<_, _>>())
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
