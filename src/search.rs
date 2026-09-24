// SPDX-License-Identifier: Apache-2.0
//! Arrival, required and slack search over a delay-calculated graph, for one propagated clock
//! with input and output delays, setup checks and common-path pessimism removal (CRPR).
//!
//! Stages in order: [`Search::find_arrivals`] (per vertex in level order: fanin paths, CRPR
//! pruning, seeds), [`Search::find_requireds`] (per vertex in reverse level order: fanout paths,
//! then the endpoint's path ends), and the slack reads [`Search::vertex_slack`] /
//! [`Search::net_slack`].
//!
//! Rules kept here:
//! - a vertex holds one PATH per TAG; a tag is (transition, min/max, clock edge, is-clock, CRPR
//!   clock vertex). Paths are ordered by transition, min/max, clock-edge index
//!   (none first), data before clock, then the CRPR clock vertex's id — and every fold over a
//!   vertex's paths runs in that order, because every comparison is fuzzy ([`crate::fuzzy`]);
//! - arrivals are `float` sums `from + arc delay`; requireds `float` differences `to − arc delay`;
//!   a path's slack is `required − arrival` (max);
//! - a clock tag entering a register clock pin over an arc whose min and max delays are fuzzily
//!   equal records the DRIVING path as its CRPR clock path, and a clock-to-Q launch keeps it — so launched data tags are distinguished by the clock net's driver, not by each flop;
//! - a setup check's required gains the CRPR credit: the launch and capture clock paths backed up
//!   (deeper first, by level) to their common pin, the smaller of the two `|max − min|` there.

use std::collections::HashMap;

use crate::graph::{EdgeKind, Graph};
use crate::liberty::{Role, MAX, MIN};
use crate::sdc::{setup_required_time, Sdc};

/// The min/max initial value (`INF` = 1e30).
const INF: f32 = 1e30;

/// A tag's CRPR clock path: the vertex (its id orders tags) and that path's own tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CrprPath {
    pub vertex: usize,
    pub rf: usize,
    pub mm: usize,
    pub clk_edge: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Tag {
    pub rf: usize,
    pub mm: usize,
    /// The clock edge (index = its transition: one clock), none for an unclocked path.
    pub clk_edge: Option<usize>,
    pub is_clock: bool,
    pub crpr: Option<CrprPath>,
}

impl Tag {
    /// `TagMatchLess(match_crpr_clk_pin = true)` key: the CRPR path compares by its vertex's id.
    fn key(&self, vertex_id: &[usize]) -> (usize, usize, i64, bool, i64) {
        (self.rf, self.mm, self.clk_edge.map_or(-1, |e| e as i64), self.is_clock, self.crpr.map_or(-1, |c| vertex_id[c.vertex] as i64))
    }

    /// Tags equal, the CRPR clock vertex included.
    fn matches(&self, other: &Tag) -> bool {
        self.rf == other.rf && self.mm == other.mm && self.clk_edge == other.clk_edge && self.is_clock == other.is_clock && self.crpr.map(|c| c.vertex) == other.crpr.map(|c| c.vertex)
    }

    /// Everything but the CRPR clock pin.
    fn matches_no_crpr(&self, other: &Tag) -> bool {
        self.rf == other.rf && self.mm == other.mm && self.clk_edge == other.clk_edge && self.is_clock == other.is_clock
    }

    /// Transition, clock edge, is-clock — NOT min/max.
    fn matches_crpr(&self, other: &Tag) -> bool {
        self.rf == other.rf && self.clk_edge == other.clk_edge && self.is_clock == other.is_clock
    }
}

/// Where a path's arrival came from: the fanin vertex's path (by tag), the edge and the arc.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Prev {
    pub vertex: usize,
    pub tag: Tag,
    pub edge: usize,
    pub arc: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Path {
    pub tag: Tag,
    pub arrival: f32,
    pub required: f32,
    pub prev: Option<Prev>,
}

/// One arc of an edge as the search walks it.
struct ArcRef {
    index: usize,
    to_rf: usize,
}

/// Paths keyed by tag, in insertion order until sorted.
#[derive(Default)]
struct Bldr {
    paths: Vec<Path>,
}

impl Bldr {
    fn find(&self, tag: &Tag) -> Option<usize> {
        self.paths.iter().position(|p| p.tag.matches(tag))
    }
}

/// The no-CRPR shadow builder (`tag_bldr_no_crpr_`): keyed by the tag without its CRPR pin, it
/// keeps the latest winning tag and its arrival.
#[derive(Default)]
struct NoCrprBldr {
    paths: Vec<(Tag, f32)>,
}

impl NoCrprBldr {
    fn find(&self, tag: &Tag) -> Option<usize> {
        self.paths.iter().position(|(t, _)| t.matches_no_crpr(tag))
    }
}

pub struct Search<'g, 'a> {
    pub graph: &'g Graph<'a>,
    pub sdc: &'g Sdc,
    /// Each vertex's paths, in tag order.
    pub paths: Vec<Vec<Path>>,
    /// Vertex id order (the order vertices were created in), which orders CRPR tags.
    vertex_id: Vec<usize>,
    input_delay: HashMap<String, usize>,
    output_delay: HashMap<String, usize>,
    /// A vertex whose out-edges include a clock-to-Q arc.
    is_reg_clk: Vec<bool>,
    /// `Levelize`: roots 0, else the longest fanin level + 1 over non-check edges.
    level: Vec<usize>,
}

fn is_check_role(r: Role) -> bool {
    matches!(r, Role::Setup | Role::Hold | Role::Recovery | Role::Removal)
}

impl<'g, 'a> Search<'g, 'a> {
    /// `vertex_id[v]`: the creation-order id of vertex `v`.
    pub fn new(graph: &'g Graph<'a>, sdc: &'g Sdc, vertex_id: Vec<usize>) -> Search<'g, 'a> {
        let n = graph.vertices.len();
        let mut is_reg_clk = vec![false; n];
        for (e, ed) in graph.edges.iter().enumerate() {
            if let EdgeKind::Gate { set } = ed.kind {
                if matches!(graph.arc_set(e, set).role, Role::RegClkToQ | Role::LatchEnToQ) {
                    is_reg_clk[ed.from] = true;
                }
            }
        }
        let input_delay = sdc.input_delays.iter().enumerate().map(|(i, d)| (d.port.clone(), i)).collect();
        let output_delay = sdc.output_delays.iter().enumerate().map(|(i, d)| (d.port.clone(), i)).collect();
        let mut level = vec![0usize; n];
        if let Ok(order) = graph.topo_order() {
            for v in order {
                for &e in &graph.out_edges[v] {
                    let to = graph.edges[e].to;
                    let check = matches!(graph.edges[e].kind, EdgeKind::Gate { set } if is_check_role(graph.arc_set(e, set).role));
                    if !check {
                        level[to] = level[to].max(level[v] + 1);
                    }
                }
            }
        }
        Search { graph, sdc, paths: vec![Vec::new(); n], vertex_id, input_delay, output_delay, is_reg_clk, level }
    }

    fn role(&self, e: usize) -> Option<Role> {
        match self.graph.edges[e].kind {
            EdgeKind::Gate { set } => Some(self.graph.arc_set(e, set).role),
            EdgeKind::Wire => None,
        }
    }

    fn is_check(&self, e: usize) -> bool {
        self.role(e).is_some_and(is_check_role)
    }

    /// The (at most two) arcs from `from_rf`, in arc order.
    fn arcs_from(&self, e: usize, from_rf: usize) -> Vec<ArcRef> {
        match self.graph.edges[e].kind {
            EdgeKind::Wire => vec![ArcRef { index: from_rf, to_rf: from_rf }],
            EdgeKind::Gate { set } => self.graph.arc_set(e, set).arcs.iter().enumerate().filter(|(_, a)| a.from_rf == from_rf).map(|(k, a)| ArcRef { index: k, to_rf: a.to_rf }).collect(),
        }
    }

    fn arc_delay(&self, e: usize, arc: usize, mm: usize) -> f32 {
        self.graph.delay[e][arc][mm]
    }

    /// For the roles this subset has: the tag a path takes through
    /// one arc, the arc's delay, and the arrival it arrives with.
    fn visit_from_path(&self, from_v: usize, from: &Path, e: usize, arc: &ArcRef) -> Option<(Tag, f32, f32)> {
        let to_v = self.graph.edges[e].to;
        let mm = from.tag.mm;
        let delay = self.arc_delay(e, arc.index, mm);
        let role = self.role(e);
        match role {
            Some(Role::RegClkToQ) => {
                // Only clocked clock paths launch, keeping the clock path's CRPR
                // path — or taking this clock path when it has none.
                if !from.tag.is_clock || from.tag.clk_edge.is_none() {
                    return None;
                }
                let crpr = from.tag.crpr.or(Some(CrprPath { vertex: from_v, rf: from.tag.rf, mm, clk_edge: from.tag.clk_edge.unwrap() }));
                let tag = Tag { rf: arc.to_rf, mm, clk_edge: from.tag.clk_edge, is_clock: false, crpr };
                Some((tag, delay, from.arrival + delay))
            }
            Some(Role::LatchDtoQ | Role::LatchEnToQ) => None,
            _ if from.tag.is_clock => {
                // A clock path through a clock-network arc.
                let to_is_clk = matches!(role, None | Some(Role::Combinational));
                let opp = self.arc_delay(e, arc.index, 1 - mm);
                let min_max_eq = crate::fuzzy::equal(delay, opp);
                let from_is_reg_clk = self.is_reg_clk[from_v];
                let crpr = if (!to_is_clk && !from_is_reg_clk) || (self.is_reg_clk[to_v] && min_max_eq) {
                    Some(CrprPath { vertex: from_v, rf: from.tag.rf, mm, clk_edge: from.tag.clk_edge.unwrap() })
                } else {
                    from.tag.crpr
                };
                let tag = Tag { rf: arc.to_rf, mm, clk_edge: from.tag.clk_edge, is_clock: to_is_clk, crpr };
                Some((tag, delay, from.arrival + delay))
            }
            _ => {
                // A data path keeps its tag.
                let tag = Tag { rf: arc.to_rf, ..from.tag };
                Some((tag, delay, from.arrival + delay))
            }
        }
    }

    /// Every vertex in level order.
    pub fn find_arrivals(&mut self) -> Result<(), String> {
        for v in self.graph.topo_order()? {
            self.arrival_visit(v);
        }
        Ok(())
    }

    /// One vertex's arrivals.
    fn arrival_visit(&mut self, v: usize) {
        let mut bldr = Bldr::default();
        let mut no_crpr = NoCrprBldr::default();
        let has_fanin_one = self.graph.in_edges[v].len() == 1;
        self.visit_fanin_paths(v, &mut bldr, &mut no_crpr, has_fanin_one);
        if self.sdc.clock.propagated && !has_fanin_one && bldr.paths.iter().any(|p| p.tag.clk_edge.is_some()) {
            self.prune_crpr_arrivals(&mut bldr, &no_crpr);
        }
        self.seed_arrivals(v, &mut bldr);
        let vid = &self.vertex_id;
        bldr.paths.sort_by_key(|p| p.tag.key(vid));
        self.paths[v] = bldr.paths;
    }

    /// In-edges newest first (edges are prepended), each fanin path in tag order, each arc from its
    /// transition.
    fn visit_fanin_paths(&self, v: usize, bldr: &mut Bldr, no_crpr: &mut NoCrprBldr, has_fanin_one: bool) {
        for &e in self.graph.in_edges[v].iter().rev() {
            if self.is_check(e) || self.role(e) == Some(Role::LatchDtoQ) {
                continue;
            }
            let from_v = self.graph.edges[e].from;
            for from in &self.paths[from_v] {
                for arc in self.arcs_from(e, from.tag.rf) {
                    let Some((to_tag, _delay, to_arrival)) = self.visit_from_path(from_v, from, e, &arc) else { continue };
                    let mm = to_tag.mm;
                    let better = |new: f32, old: f32| if mm == MAX { crate::fuzzy::greater(new, old) } else { crate::fuzzy::less(new, old) };
                    let prev = Some(Prev { vertex: from_v, tag: from.tag, edge: e, arc: arc.index });
                    let m = bldr.find(&to_tag);
                    if m.is_none_or(|i| better(to_arrival, bldr.paths[i].arrival)) {
                        let path = Path { tag: to_tag, arrival: to_arrival, required: 0.0, prev };
                        match m {
                            Some(i) => bldr.paths[i] = path,
                            None => bldr.paths.push(path),
                        }
                        if !has_fanin_one && to_tag.crpr.is_some() && !to_tag.is_clock {
                            let n = no_crpr.find(&to_tag);
                            if n.is_none_or(|i| better(to_arrival, no_crpr.paths[i].1)) {
                                match n {
                                    Some(i) => no_crpr.paths[i] = (to_tag, to_arrival),
                                    None => no_crpr.paths.push((to_tag, to_arrival)),
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// `|late − early|` arrival of the tag's CRPR clock path
    /// (the other min/max arrival: the first path of the target min/max — min, for a max path — and the
    /// same transition that matches ignoring min/max; none → the path's own arrival).
    fn max_crpr(&self, tag: &Tag) -> f32 {
        let Some(c) = tag.crpr else { return 0.0 };
        let Some(p) = self.crpr_clk_path(&c) else { return 0.0 };
        self.crpr_arrival_diff(c.vertex, p)
    }

    /// Drop a data tag whose arrival is beaten, by more than
    /// the most CRPR could restore, by the no-CRPR winner of the same tag.
    fn prune_crpr_arrivals(&self, bldr: &mut Bldr, no_crpr: &NoCrprBldr) {
        bldr.paths.retain(|p| {
            let tag = p.tag;
            if tag.is_clock || tag.crpr.is_none() {
                return true;
            }
            let Some(i) = no_crpr.find(&tag) else { return true };
            let (win_tag, max_arrival) = no_crpr.paths[i];
            if win_tag.matches(&tag) {
                return true;
            }
            let max_crpr = self.max_crpr(&win_tag);
            let beaten = if tag.mm == MAX { crate::fuzzy::greater(max_arrival - max_crpr, p.arrival) } else { crate::fuzzy::less(max_arrival + max_crpr, p.arrival) };
            !(beaten && win_tag.crpr.map(|c| c.mm) == tag.crpr.map(|c| c.mm))
        });
    }

    /// The clock's source port, and input ports with a delay.
    fn seed_arrivals(&self, v: usize, bldr: &mut Bldr) {
        let name = &self.graph.vertices[v].name;
        let clock = &self.sdc.clock;
        if self.graph.vertices[v].lib.is_some() {
            return;
        }
        if *name == clock.source {
            // Per min/max and transition, the edge of that transition at
            // `insertion + edge time`.
            for mm in [MIN, MAX] {
                for rf in [0, 1] {
                    let tag = Tag { rf, mm, clk_edge: Some(rf), is_clock: true, crpr: None };
                    set_arrival(bldr, tag, 0.0 + clock.edge_time(rf));
                }
            }
        } else if let Some(&d) = self.input_delay.get(name) {
            // The clock's rise edge + the delay.
            for mm in [MIN, MAX] {
                let clk_arrival = clock.edge_time(0);
                for rf in [0, 1] {
                    let tag = Tag { rf, mm, clk_edge: Some(0), is_clock: false, crpr: None };
                    set_arrival(bldr, tag, clk_arrival + self.sdc.input_delays[d].delay[rf][mm]);
                }
            }
        }
    }

    /// Every vertex in reverse level order.
    pub fn find_requireds(&mut self) -> Result<(), String> {
        let order = self.graph.topo_order()?;
        for &v in order.iter().rev() {
            self.required_visit(v);
        }
        Ok(())
    }

    /// Requireds from the fanout, then the endpoint's path ends.
    fn required_visit(&mut self, v: usize) {
        let mut req: Vec<f32> = self.paths[v].iter().map(|p| if p.tag.mm == MAX { INF } else { -INF }).collect();
        self.visit_fanout_paths(v, &mut req);
        self.visit_path_ends(v, &mut req);
        for (p, r) in self.paths[v].iter_mut().zip(req) {
            p.required = r;
        }
    }

    /// A MAX path's required: the fuzzily smaller.
    fn required_set(req: &mut [f32], i: usize, value: f32) {
        if crate::fuzzy::less(value, req[i]) {
            req[i] = value;
        }
    }

    /// Requireds back from the fanout (max paths).
    fn visit_fanout_paths(&self, v: usize, req: &mut [f32]) {
        for &e in self.graph.out_edges[v].iter().rev() {
            if self.is_check(e) || self.role(e) == Some(Role::LatchDtoQ) {
                continue;
            }
            let to_v = self.graph.edges[e].to;
            for (i, from) in self.paths[v].iter().enumerate() {
                if from.tag.mm != MAX {
                    continue;
                }
                for arc in self.arcs_from(e, from.tag.rf) {
                    let Some((to_tag, delay, _)) = self.visit_from_path(v, from, e, &arc) else { continue };
                    let to_paths = &self.paths[to_v];
                    let to_required = match to_paths.iter().find(|p| p.tag.matches(&to_tag)) {
                        Some(p) => Some(p.required),
                        // The first same-transition path matching
                        // all but the CRPR pin.
                        None => to_paths.iter().find(|p| p.tag.mm == to_tag.mm && p.tag.rf == to_tag.rf && p.tag.matches_no_crpr(&to_tag)).map(|p| p.required),
                    };
                    if let Some(r) = to_required {
                        Self::required_set(req, i, r - delay);
                    }
                }
            }
        }
    }

    /// The endpoint's path ends, for max paths: an output delay
    /// end if the port has one, else a setup check end.
    fn visit_path_ends(&self, v: usize, req: &mut [f32]) {
        let name = &self.graph.vertices[v].name;
        let od = if self.graph.vertices[v].lib.is_none() { self.output_delay.get(name).copied() } else { None };
        let clock = &self.sdc.clock;
        for (i, path) in self.paths[v].iter().enumerate() {
            if path.tag.mm != MAX {
                continue;
            }
            let Some(src_edge) = path.tag.clk_edge else { continue };
            if let Some(d) = od {
                // Target clock time (the delay's rise edge) − the delay.
                let tgt_time = setup_required_time(clock, src_edge, 0);
                let margin = self.sdc.output_delays[d].delay[path.tag.rf][MAX];
                Self::required_set(req, i, (tgt_time + (0.0 + 0.0)) - margin);
                continue;
            }
            for &e in self.graph.in_edges[v].iter().rev() {
                if self.role(e) != Some(Role::Setup) {
                    continue;
                }
                let EdgeKind::Gate { set } = self.graph.edges[e].kind else { continue };
                let tgt_v = self.graph.edges[e].from;
                for (k, arc) in self.graph.arc_set(e, set).arcs.iter().enumerate() {
                    if arc.to_rf != path.tag.rf {
                        continue;
                    }
                    for tgt in &self.paths[tgt_v] {
                        // The target clock of a max path under OCV: the MIN clock path.
                        if tgt.tag.mm != MIN || tgt.tag.rf != arc.from_rf || !tgt.tag.is_clock {
                            continue;
                        }
                        let tgt_edge = tgt.tag.clk_edge.expect("a clock path has an edge");
                        // The check's required time.
                        let latency = (tgt.arrival - clock.edge_time(tgt_edge)) - 0.0;
                        let tgt_clk_arrival = (0.0 + latency) + setup_required_time(clock, src_edge, tgt_edge);
                        let margin = self.arc_delay(e, k, MAX);
                        let crpr = self.check_crpr(path, tgt_v, tgt);
                        Self::required_set(req, i, (tgt_clk_arrival - (margin + 0.0)) + crpr);
                    }
                }
            }
        }
    }

    /// The clock path a tag's CRPR clock path names.
    fn crpr_clk_path(&self, c: &CrprPath) -> Option<&Path> {
        self.paths[c.vertex].iter().find(|p| p.tag.is_clock && p.tag.rf == c.rf && p.tag.mm == c.mm && p.tag.clk_edge == Some(c.clk_edge))
    }

    /// The fanin path the arrival came from.
    fn prev_path(&self, p: &Path) -> Option<(usize, &Path)> {
        let prev = p.prev?;
        self.paths[prev.vertex].iter().find(|q| q.tag.matches(&prev.tag)).map(|q| (prev.vertex, q))
    }

    /// `|arrival − otherMinMaxArrival|`.
    fn crpr_arrival_diff(&self, v: usize, p: &Path) -> f32 {
        let other = self.paths[v].iter().find(|o| o.tag.mm == 1 - p.tag.mm && o.tag.rf == p.tag.rf && o.tag.matches_crpr(&p.tag)).map_or(p.arrival, |o| o.arrival);
        (p.arrival - other).abs()
    }

    /// The check CRPR for a data path ending at a check against the target clock path
    /// `tgt` (at `tgt_v`): from the data tag's CRPR clock path and the target clock path, back up
    /// the deeper one (by level) until they meet; the credit is the smaller of the two
    /// paths' `|max − min|` there. No common pin, or no CRPR clock path (a path from
    /// an input): 0.
    fn check_crpr(&self, path: &Path, tgt_v: usize, tgt: &Path) -> f32 {
        let Some(c) = path.tag.crpr else { return 0.0 };
        let Some(src) = self.crpr_clk_path(&c) else { return 0.0 };
        if src.tag.mm == tgt.tag.mm || !self.sdc.clock.propagated {
            return 0.0;
        }
        let (mut sv, mut sp) = (c.vertex, src);
        let (mut tv, mut tp) = (tgt_v, tgt);
        let (mut sl, mut tl) = (self.level[sv], self.level[tv]);
        while sv != tv {
            let diff = sl as i64 - tl as i64;
            if diff >= 0 {
                let Some((v, p)) = self.prev_path(sp) else { return 0.0 };
                (sv, sp, sl) = (v, p, self.level[v]);
            }
            if diff <= 0 {
                let Some((v, p)) = self.prev_path(tp) else { return 0.0 };
                (tv, tp, tl) = (v, p, self.level[v]);
            }
        }
        self.crpr_arrival_diff(sv, sp).min(self.crpr_arrival_diff(tv, tp))
    }

    /// The fuzzily least `required − arrival` over max paths, in tag
    /// order (`INF` with none).
    pub fn vertex_slack(&self, v: usize) -> f32 {
        let mut slack = INF;
        for p in &self.paths[v] {
            if p.tag.mm == MAX {
                let s = p.required - p.arrival;
                if crate::fuzzy::less(s, slack) {
                    slack = s;
                }
            }
        }
        slack
    }

    /// The fuzzily least slack over the net's LOAD pins, in the net's pin
    /// order.
    pub fn net_slack(&self, loads: &[usize]) -> f32 {
        let mut slack = INF;
        for &v in loads {
            let s = self.vertex_slack(v);
            if crate::fuzzy::less(s, slack) {
                slack = s;
            }
        }
        slack
    }
}

/// A seed sets (or replaces) its tag's arrival.
fn set_arrival(bldr: &mut Bldr, tag: Tag, arrival: f32) {
    let path = Path { tag, arrival, required: 0.0, prev: None };
    match bldr.find(&tag) {
        Some(i) => bldr.paths[i] = path,
        None => bldr.paths.push(path),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::liberty::{Library, FALL, RISE};
    use crate::netlist::{Conn, Net, Netlist, PortDir};
    use crate::sdc::{Clock, PortDelay};

    const LIB: &str = r#"library (t) {
      time_unit : "1ns"; capacitive_load_unit (1, pf);
      cell (buf) {
        pin (A) { direction : input; capacitance : 0.001; }
        pin (X) { direction : output; function : "A";
          timing () { related_pin : "A"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("0.1"); } rise_transition (scalar) { values ("0.05"); }
            cell_fall (scalar) { values ("0.1"); } fall_transition (scalar) { values ("0.05"); } } }
      }
      cell (dff) {
        ff (IQ, IQN) { clocked_on : "CLK"; next_state : "D"; }
        pin (CLK) { direction : input; capacitance : 0.001; clock : true; }
        pin (D) { direction : input; capacitance : 0.001;
          timing () { related_pin : "CLK"; timing_type : setup_rising;
            rise_constraint (scalar) { values ("0.05"); } fall_constraint (scalar) { values ("0.06"); } } }
        pin (Q) { direction : output; function : "IQ";
          timing () { related_pin : "CLK"; timing_type : rising_edge;
            cell_rise (scalar) { values ("0.3"); } rise_transition (scalar) { values ("0.1"); }
            cell_fall (scalar) { values ("0.35"); } fall_transition (scalar) { values ("0.1"); } } }
      }
    }"#;

    /// clk -> b1 -> {f1, f2}/CLK; in -> f1/D; f1/Q -> f2/D; f2/Q -> out. Period 1 ns, io 0.2 ns.
    fn design() -> (Vec<Library>, Netlist, Sdc) {
        let libs = vec![Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let i = |k: usize, p: &str| Conn::Inst(k, p.into());
        let net = |n: &str, pins: Vec<Conn>| Net { name: n.into(), pins };
        let netlist = Netlist {
            insts: vec![("b1".into(), "buf".into()), ("f1".into(), "dff".into()), ("f2".into(), "dff".into())],
            ports: vec![("clk".into(), PortDir::Input), ("in".into(), PortDir::Input), ("out".into(), PortDir::Output)],
            nets: vec![
                net("clk", vec![i(0, "A"), Conn::Port(0)]),
                net("cb", vec![i(0, "X"), i(1, "CLK"), i(2, "CLK")]),
                net("in", vec![i(1, "D"), Conn::Port(1)]),
                net("q1", vec![i(1, "Q"), i(2, "D")]),
                net("out", vec![i(2, "Q"), Conn::Port(2)]),
            ],
        };
        let sdc = Sdc {
            clock: Clock::new("c", 1e-9, "clk", true),
            input_delays: vec![PortDelay::uniform("in", 0.2e-9)],
            output_delays: vec![PortDelay::uniform("out", 0.2e-9)],
        };
        (libs, netlist, sdc)
    }

    fn timed<R>(f: impl FnOnce(&Search, &dyn Fn(&str) -> usize) -> R) -> R {
        let (libs, netlist, sdc) = design();
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let ids = (0..g.vertices.len()).collect();
        let mut s = Search::new(&g, &sdc, ids);
        s.find_arrivals().unwrap();
        s.find_requireds().unwrap();
        let v = |n: &str| g.vertices.iter().position(|x| x.name == n).unwrap();
        f(&s, &v)
    }

    fn path(s: &Search, v: usize, rf: usize, is_clock: bool) -> Path {
        *s.paths[v].iter().find(|p| p.tag.rf == rf && p.tag.mm == MAX && p.tag.is_clock == is_clock).unwrap()
    }

    /// Rule: a clock tag entering a register clock pin over
    /// an arc with equal min/max delay takes the DRIVING path as its CRPR clock path, and the
    /// launched data tag keeps it — the clock buffer's output, not the flop's clock pin.
    #[test]
    fn launched_data_tags_carry_the_clock_nets_driver_as_crpr_path() {
        timed(|s, v| {
            let ck = path(s, v("f1/CLK"), RISE, true);
            assert_eq!(ck.tag.crpr.map(|c| c.vertex), Some(v("b1/X")));
            let q = path(s, v("f1/Q"), RISE, false);
            assert_eq!(q.tag.crpr.map(|c| c.vertex), Some(v("b1/X")));
            assert_eq!(q.tag.clk_edge, Some(RISE));
        });
    }

    /// Arrivals are float sums along the path: clock edge + buffer + clock-to-Q; an input's is
    /// the rise edge + its delay.
    #[test]
    fn arrivals_are_float_sums() {
        timed(|s, v| {
            let b = 0.1f32 * 1e-9;
            assert_eq!(path(s, v("f1/CLK"), RISE, true).arrival, 0.0 + b);
            assert_eq!(path(s, v("f1/Q"), RISE, false).arrival, (0.0 + b) + 0.3f32 * 1e-9);
            assert_eq!(path(s, v("f1/D"), FALL, false).arrival, 0.0 + 0.2e-9f32);
        });
    }

    /// Rule: ((min capture clock arrival − edge time) +
    /// cycle time) − setup; and an output's is the cycle time − its delay.
    #[test]
    fn requireds_at_checks_and_output_delays() {
        timed(|s, v| {
            let b = 0.1f32 * 1e-9;
            let t = 1e-9f32;
            assert_eq!(path(s, v("f2/D"), RISE, false).required, ((0.0 + ((0.0 + b) - 0.0 - 0.0)) + t) - (0.05f32 * 1e-9 + 0.0));
            assert_eq!(path(s, v("f2/D"), FALL, false).required, ((0.0 + ((0.0 + b) - 0.0 - 0.0)) + t) - (0.06f32 * 1e-9 + 0.0));
            assert_eq!(path(s, v("out"), RISE, false).required, (t + 0.0) - 0.2e-9f32);
        });
    }

    /// A path's slack is required − arrival; a net's is the least over its LOAD pins.
    #[test]
    fn slacks_read_required_minus_arrival() {
        timed(|s, v| {
            let d = path(s, v("f2/D"), RISE, false);
            let dv = v("f2/D");
            assert!(s.vertex_slack(dv) <= d.required - d.arrival);
            assert_eq!(s.net_slack(&[dv]), s.vertex_slack(dv));
            assert_eq!(s.net_slack(&[]), INF);
        });
    }
}
