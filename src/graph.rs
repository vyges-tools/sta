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
//!   min/max (propagated clocks: never an ideal clock slew here);
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
}

pub struct Graph<'a> {
    pub libs: &'a [Library],
    pub netlist: &'a Netlist,
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
}

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

impl<'a> Graph<'a> {
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
        let find_cell = |name: &str| libs.iter().position(|l| l.cells.contains_key(name));
        let mut vertices = Vec::new();
        let mut index: HashMap<String, usize> = HashMap::new();
        // Per instance: the vertex an arc set goes TO, and the one it comes FROM (they differ only
        // for a bidirect pin).
        let mut inst_vertices: Vec<HashMap<String, usize>> = vec![HashMap::new(); netlist.insts.len()];
        let mut inst_from: Vec<HashMap<String, usize>> = vec![HashMap::new(); netlist.insts.len()];
        let mut bidirect: Vec<String> = Vec::new();
        for (i, (name, cname)) in netlist.insts.iter().enumerate() {
            // A physical-only cell (tap, fill, decap) has no liberty cell and nothing to time.
            let Some(lib) = find_cell(cname) else { continue };
            let has = pins.and_then(|m| m.get(cname));
            for p in &cell(libs, lib, cname).ports {
                if has.is_some_and(|h| !h.contains(&p.name)) {
                    continue;
                }
                let kinds: &[bool] = match p.direction {
                    Direction::Input => &[false],
                    Direction::Output | Direction::Tristate => &[true],
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
        for net in &netlist.nets {
            if let Some(b) = net.pins.iter().map(|c| netlist.pin_name(c)).find(|n| bidirect.contains(n)) {
                return Err(format!("{b}: a bidirect pin on a net (net {}) is not modelled", net.name));
            }
        }
        for (i, (name, dir)) in netlist.ports.iter().enumerate() {
            if *dir == PortDir::Inout {
                return Err(format!("port {name}: bidirect ports are not modelled"));
            }
            let v = vertices.len();
            index.insert(name.clone(), v);
            vertices.push(Vertex { name: name.clone(), conn: Conn::Port(i), is_driver: *dir == PortDir::Input, lib: None, cell: None, port: None });
        }
        let mut edges = Vec::new();
        for (i, (_, cname)) in netlist.insts.iter().enumerate() {
            let Some(lib) = find_cell(cname) else { continue };
            for (k, set) in cell(libs, lib, cname).arc_sets.iter().enumerate() {
                let tristate = matches!(set.role, Role::TristateEnable | Role::TristateDisable);
                if set.role == Role::Other || (set.arcs.is_empty() && !tristate) {
                    continue;
                }
                if let (Some(&f), Some(&t)) = (inst_from[i].get(&set.from), inst_vertices[i].get(&set.to)) {
                    edges.push(Edge { from: f, to: t, kind: EdgeKind::Gate { set: k } });
                }
            }
        }
        let mut vertex_net = vec![None; vertices.len()];
        for (n, net) in netlist.nets.iter().enumerate() {
            let vs: Vec<usize> = net.pins.iter().filter_map(|c| index.get(&netlist.pin_name(c)).copied()).collect();
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
        Ok(Graph { libs, netlist, vertices, edges, in_edges, out_edges, vertex_net, slew: vec![[[0.0; 2]; 2]; n], delay, slew_limit: HashMap::new(), clamped: Vec::new(), slew_annotated: HashMap::new(), sdc: SdcEnv::default() })
    }

    pub fn is_check(&self, e: usize) -> bool {
        match self.edges[e].kind {
            EdgeKind::Gate { set } => matches!(self.arc_set(e, set).role, Role::Setup | Role::Hold | Role::Recovery | Role::Removal),
            EdgeKind::Wire => false,
        }
    }

    pub(crate) fn arc_set(&self, e: usize, set: usize) -> &crate::liberty::ArcSet {
        let v = &self.vertices[self.edges[e].to];
        &cell(self.libs, v.lib.unwrap(), v.cell.as_deref().unwrap()).arc_sets[set]
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
        let thru = |e: usize| -> bool {
            match self.edges[e].kind {
                EdgeKind::Gate { set } => !self.is_check(e) && !matches!(self.arc_set(e, set).role, Role::RegSetClr | Role::LatchDtoQ),
                EdgeKind::Wire => true,
            }
        };
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

    /// Topological order over the propagating edges (every edge but timing checks).
    pub(crate) fn topo_order(&self) -> Result<Vec<usize>, String> {
        let n = self.vertices.len();
        let mut indeg = vec![0usize; n];
        for (e, ed) in self.edges.iter().enumerate() {
            if !self.is_check(e) {
                indeg[ed.to] += 1;
            }
        }
        let mut stack: Vec<usize> = (0..n).rev().filter(|&v| indeg[v] == 0).collect();
        let mut order = Vec::with_capacity(n);
        while let Some(v) = stack.pop() {
            order.push(v);
            for &e in &self.out_edges[v] {
                if self.is_check(e) {
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
            return Err("the timing graph has a loop — not modelled".into());
        }
        Ok(order)
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
                        self.slew[load][rf][mm] = load_slew as f32;
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
    fn parasitic_load(&self, drvr: usize, rf: usize, mm: usize, parasitics: &HashMap<String, NetParasitics>, index: &HashMap<String, usize>) -> (f32, f32, Option<(crate::parasitics::PiElmore, Vec<String>)>) {
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
        for c in &self.netlist.nets[n].pins {
            if let Some(&v) = index.get(&self.netlist.pin_name(c)) {
                sum += self.pin_cap(v, rf, mm);
            }
        }
        sum
    }

    /// The pi-Elmore model of a driver's net, for a transition and min/max — the one delay
    /// calculation and [`Graph::load_cap`] both read.
    fn reduced(&self, drvr: usize, rf: usize, mm: usize, parasitics: &HashMap<String, NetParasitics>, index: &HashMap<String, usize>) -> Option<(crate::parasitics::PiElmore, Vec<String>)> {
        let n = self.vertex_net[drvr]?;
        let np = parasitics.get(&self.netlist.nets[n].name)?;
        let d = np.node_names.iter().position(|x| x == &self.vertices[drvr].name)?;
        let names = &np.node_names;
        let cap = |i: usize| index.get(&names[i]).map_or(0.0, |&v| self.pin_cap(v, rf, mm));
        let load = |i: usize| index.get(&names[i]).is_some_and(|&v| !self.vertices[v].is_driver);
        Some((reduce_to_pi_elmore(&np.network, d, &NodePins { pin_cap: &cap, is_load: &load }), names.clone()))
    }

    /// `GraphDelayCalc::loadCap(drvr_pin, rf, scene, max, pin_cap, wire_cap)` for one transition,
    /// and whether the driver has a reduced pi model at all (`findPiElmore`).
    pub fn load_cap_parts(&self, drvr: usize, parasitics: &HashMap<String, NetParasitics>, rf: usize) -> (f32, f32, bool) {
        let index: HashMap<String, usize> = self.vertices.iter().enumerate().map(|(i, v)| (v.name.clone(), i)).collect();
        let (pin_cap, wire_cap, _) = self.parasitic_load(drvr, rf, MAX, parasitics, &index);
        (pin_cap, wire_cap, self.reduced(drvr, rf, MAX, parasitics, &index).is_some())
    }

    /// `GraphDelayCalc::loadCap(drvr_pin, scene, max)`: over rise then fall, the larger of
    /// `pin_cap + wire_cap` ([`Graph::parasitic_load`]) — the pin capacitance of the driver's net
    /// (`connectedCap`, with the SDC's loads) plus, when the reduced pi model's total is at least
    /// that, the total MINUS the pin cap (so the sum is `pin + (pi − pin)` in float, not the pi
    /// total); a smaller pi model is ignored (wire cap 0); a net `set_load` replaces the pi model.
    pub fn load_cap(&self, drvr: usize, parasitics: &HashMap<String, NetParasitics>) -> f32 {
        let index: HashMap<String, usize> = self.vertices.iter().enumerate().map(|(i, v)| (v.name.clone(), i)).collect();
        let mut load_cap = -1e30f32;
        for rf in [RISE, FALL] {
            let (pin_cap, wire_cap, _) = self.parasitic_load(drvr, rf, MAX, parasitics, &index);
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
        let index: HashMap<String, usize> = self.vertices.iter().enumerate().map(|(i, v)| (v.name.clone(), i)).collect();
        const INIT: [f32; 2] = [1e30, -1e30];
        // Fuzzily worse for this min/max, so the merge order matters.
        let worse = |mm: usize, a: f32, b: f32| if mm == MAX { crate::fuzzy::greater(a, b) } else { crate::fuzzy::less(a, b) };
        for &v in &order {
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
                continue;
            }
            if fanin.is_empty() {
                if let Some(drive) = self.sdc.input_drive.get(&self.vertices[v].name).cloned() {
                    self.time_input_drive(v, &wires, &drive, parasitics, &index)?;
                    continue;
                }
                // An input port (`seedNoDrvrCellSlew` / `seedNoDrvrSlew`): its slew is the
                // `set_input_transition`, else 0; its loads by the input-port delay at that slew
                // over the `parasiticLoad` pi model — a load with no Elmore delay takes the slew.
                for rf in [RISE, FALL] {
                    for mm in [MIN, MAX] {
                        let in_slew = self.sdc.input_slew.get(&self.vertices[v].name).and_then(|s| s[rf][mm]).unwrap_or(0.0);
                        self.slew[v][rf][mm] = in_slew;
                        let (_, _, pe) = self.parasitic_load(v, rf, mm, parasitics, &index);
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
                            self.slew[load][rf][mm] = load_slew as f32;
                            self.delay[e][rf][mm] = wire_delay as f32;
                        }
                    }
                }
                continue;
            }
            // Driver delays: init, then every arc of every in-edge.
            let drvr_lib = self.vertices[v].lib.expect("an instance driver");
            for &e in &wires {
                self.slew[self.edges[e].to] = [[INIT[MIN], INIT[MAX]], [INIT[MIN], INIT[MAX]]];
                self.delay[e] = vec![[INIT[MIN], INIT[MAX]]; 2];
            }
            self.slew[v] = [[INIT[MIN], INIT[MAX]], [INIT[MIN], INIT[MAX]]];
            let mut exists = [false; 2];
            // Edges are PREPENDED to a vertex's in-edge list, so they are visited newest first —
            // the reverse of the cell's arc-set order.
            for &e in fanin.iter().rev() {
                let EdgeKind::Gate { set } = self.edges[e].kind else { continue };
                let arcs = self.arc_set(e, set).arcs.clone();
                if self.arc_set(e, set).role == Role::LatchDtoQ {
                    return Err("latch D->Q arcs are not modelled".into());
                }
                let from = self.edges[e].from;
                for mm in [MIN, MAX] {
                    for (k, arc) in arcs.iter().enumerate() {
                        let Model::Gate(model) = &arc.model else { continue };
                        let rf = arc.to_rf;
                        let in_slew = self.slew[from][arc.from_rf][mm];
                        let (pin_cap, wire_cap, pe) = self.parasitic_load(v, rf, mm, parasitics, &index);
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
                            if worse(mm, load_slew as f32, self.slew[load][rf][mm]) {
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
                            self.slew[self.edges[w].to][rf][mm] = 0.0;
                        }
                    }
                }
            }
        }
        // Timing-check delays, after every slew is known.
        for e in 0..self.edges.len() {
            if !self.is_check(e) {
                continue;
            }
            let EdgeKind::Gate { set } = self.edges[e].kind else { continue };
            let arcs = self.arc_set(e, set).arcs.clone();
            let (from, to) = (self.edges[e].from, self.edges[e].to);
            for (k, arc) in arcs.iter().enumerate() {
                let Model::Check(table) = &arc.model else { continue };
                for mm in [MIN, MAX] {
                    let clk_slew = self.slew[from][arc.from_rf][1 - mm];
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
        Ok(())
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

    /// Two `and2` in a chain: ports a, b into u1; u1/Y and port c into u2; u2/Y out to y.
    fn chain() -> Netlist {
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        Netlist {
            insts: vec![("u1".into(), "and2".into()), ("u2".into(), "and2".into())],
            ports: vec![("a".into(), PortDir::Input), ("b".into(), PortDir::Input), ("c".into(), PortDir::Input), ("y".into(), PortDir::Output)],
            nets: vec![
                Net { name: "a".into(), pins: vec![pin(0, "A"), Conn::Port(0)] },
                Net { name: "b".into(), pins: vec![pin(0, "B"), Conn::Port(1)] },
                Net { name: "n1".into(), pins: vec![pin(0, "Y"), pin(1, "A")] },
                Net { name: "c".into(), pins: vec![pin(1, "B"), Conn::Port(2)] },
                Net { name: "y".into(), pins: vec![pin(1, "Y"), Conn::Port(3)] },
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
        par.insert("y".to_string(), NetParasitics { node_names: vec!["u2/Y".into(), "y".into()], network: net });
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
            insts: vec![("p1".into(), "PADC".into())],
            ports: vec![("a".into(), PortDir::Input)],
            nets: vec![Net { name: "a".into(), pins: vec![Conn::Port(0), Conn::Inst(0, "DATA".into())] }],
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
        nl2.nets.push(Net { name: "b".into(), pins: vec![Conn::Inst(0, "PAD".into())] });
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
            insts: vec![("u1".into(), "and2".into())],
            ports: vec![("a".into(), PortDir::Input), ("b".into(), PortDir::Input), ("y".into(), PortDir::Output)],
            nets: vec![
                Net { name: "a".into(), pins: vec![a("A"), Conn::Port(0)] },
                Net { name: "b".into(), pins: vec![a("B"), Conn::Port(1)] },
                Net { name: "y".into(), pins: vec![a("Y"), Conn::Port(2)] },
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
}
