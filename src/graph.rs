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
}

fn cell<'a>(libs: &'a [Library], lib: usize, name: &str) -> &'a Cell {
    &libs[lib].cells[name]
}

impl<'a> Graph<'a> {
    /// Vertices for every signal pin of every instance and every port, gate edges for every arc
    /// set between two of an instance's vertices, wire edges from each driver to each load of a
    /// net.
    pub fn build(libs: &'a [Library], netlist: &'a Netlist) -> Result<Graph<'a>, String> {
        let find_cell = |name: &str| libs.iter().position(|l| l.cells.contains_key(name));
        let mut vertices = Vec::new();
        let mut index: HashMap<String, usize> = HashMap::new();
        let mut inst_vertices: Vec<HashMap<String, usize>> = vec![HashMap::new(); netlist.insts.len()];
        for (i, (name, cname)) in netlist.insts.iter().enumerate() {
            // A physical-only cell (tap, fill, decap) has no liberty cell and nothing to time.
            let Some(lib) = find_cell(cname) else { continue };
            for p in &cell(libs, lib, cname).ports {
                let is_driver = match p.direction {
                    Direction::Input => false,
                    Direction::Output | Direction::Tristate => true,
                    Direction::Bidirect => return Err(format!("{name}/{}: bidirect pins are not modelled", p.name)),
                    _ => continue,
                };
                let v = vertices.len();
                let vname = format!("{name}/{}", p.name);
                index.insert(vname.clone(), v);
                inst_vertices[i].insert(p.name.clone(), v);
                vertices.push(Vertex { name: vname, conn: Conn::Inst(i, p.name.clone()), is_driver, lib: Some(lib), cell: Some(cname.clone()), port: Some(p.name.clone()) });
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
                if set.role == Role::Other || set.arcs.is_empty() {
                    continue;
                }
                if let (Some(&f), Some(&t)) = (inst_vertices[i].get(&set.from), inst_vertices[i].get(&set.to)) {
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
        Ok(Graph { libs, netlist, vertices, edges, in_edges, out_edges, vertex_net, slew: vec![[[0.0; 2]; 2]; n], delay })
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

    /// The library whose thresholds a load uses: a port's is the default (first read) library.
    fn threshold_library(&self, v: usize) -> usize {
        self.vertices[v].lib.unwrap_or(0)
    }

    fn pin_cap(&self, v: usize, rf: usize, mm: usize) -> f32 {
        match (&self.vertices[v].lib, &self.vertices[v].cell, &self.vertices[v].port) {
            (Some(l), Some(c), Some(p)) => cell(self.libs, *l, c).port(p).map_or(0.0, |p| p.capacitance(rf, mm)),
            _ => 0.0,
        }
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

    /// Delay calculation over the whole graph. With `trace`, one line per DMP gate call
    /// (`dcalc|gate|drvr|rf pair|in_slew|c2|rpi|c1|alg|rd|t0|dt|ceff|valid|delay|slew`, bits in hex)
    /// and per load (`dcalc|load|drvr|load|wire delay|load slew`).
    pub fn find_delays(&mut self, parasitics: &HashMap<String, NetParasitics>, mut trace: Option<&mut Vec<String>>) -> Result<(), String> {
        let order = self.topo_order()?;
        let index: HashMap<String, usize> = self.vertices.iter().enumerate().map(|(i, v)| (v.name.clone(), i)).collect();
        const INIT: [f32; 2] = [1e30, -1e30];
        // Fuzzily worse for this min/max, so the merge order matters.
        let worse = |mm: usize, a: f32, b: f32| if mm == MAX { crate::fuzzy::greater(a, b) } else { crate::fuzzy::less(a, b) };
        // The pi-Elmore model of a driver's net, for a transition and min/max.
        let reduce = |g: &Graph, drvr: usize, rf: usize, mm: usize| -> Option<(crate::parasitics::PiElmore, Vec<String>)> {
            let n = g.vertex_net[drvr]?;
            let np = parasitics.get(&g.netlist.nets[n].name)?;
            let d = np.node_names.iter().position(|x| x == &g.vertices[drvr].name)?;
            let names = &np.node_names;
            let cap = |i: usize| index.get(&names[i]).map_or(0.0, |&v| g.pin_cap(v, rf, mm));
            let load = |i: usize| index.get(&names[i]).is_some_and(|&v| !g.vertices[v].is_driver);
            Some((reduce_to_pi_elmore(&np.network, d, &NodePins { pin_cap: &cap, is_load: &load }), names.clone()))
        };
        for &v in &order {
            let fanin: Vec<usize> = self.in_edges[v].iter().copied().filter(|&e| !self.is_check(e)).collect();
            let wires: Vec<usize> = self.out_edges[v].iter().copied().filter(|&e| matches!(self.edges[e].kind, EdgeKind::Wire)).collect();
            if !self.vertices[v].is_driver {
                if fanin.is_empty() {
                    // A root load's slew is 0.
                    self.slew[v] = [[0.0; 2]; 2];
                }
                continue;
            }
            if fanin.is_empty() {
                // An input port with no drive — slew 0, its loads by the input-port delay.
                self.slew[v] = [[0.0; 2]; 2];
                for rf in [RISE, FALL] {
                    for mm in [MIN, MAX] {
                        let pe = reduce(self, v, rf, mm);
                        for &e in &wires {
                            let load = self.edges[e].to;
                            let mut wire_delay = 0.0f64;
                            let mut load_slew = 0.0f64;
                            let elmore = pe.as_ref().and_then(|(p, names)| p.elmore.iter().find(|(n, _)| names[*n] == self.vertices[load].name).map(|(_, e)| *e));
                            let ll = self.threshold_library(load);
                            if let Some(el) = elmore {
                                let l = &self.libs[ll];
                                let th = Thresholds { vth: l.input_threshold[rf], vl: l.slew_lower_threshold[rf], vh: l.slew_upper_threshold[rf], slew_derate: l.slew_derate };
                                (wire_delay, load_slew) = dspf_wire_delay_slew(0.0, el, &th);
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
                        let pin_cap = self.net_pin_cap(v, rf, mm, &index);
                        let pe = reduce(self, v, rf, mm).filter(|(p, _)| p.c1 + p.c2 >= pin_cap);
                        let l = &self.libs[drvr_lib];
                        let th = Thresholds { vth: l.output_threshold[rf], vl: l.slew_lower_threshold[rf], vh: l.slew_upper_threshold[rf], slew_derate: l.slew_derate };
                        let (gate_delay, drvr_slew, mut dmp) = match &pe {
                            Some((p, _)) => {
                                let mut d = Dmp::new(model, &th, in_slew, p.c2, p.rpi, p.c1);
                                let (gd, ds) = d.gate_delay_slew();
                                (gd, ds, Some(d))
                            }
                            None => {
                                let (gd, ds) = model.gate_delay(in_slew, pin_cap);
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
