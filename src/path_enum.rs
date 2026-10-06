// SPDX-License-Identifier: Apache-2.0
//! The k worst paths to one endpoint: `findPathEnds` to a single `-to` pin with
//! `endpoint_path_count > 1` — `PathGroups::makeGroupPathEnds` (`MakePathEndsAll`), then
//! `enumPathEnds` over a `PathEnum`.
//!
//! A `-to` alone needs no filtered search: the arrivals already found are the ones enumerated.
//!
//! `PathEnum` keeps a priority queue of diversions in [`cmp_ends`] order (the least slack first,
//! every tie broken). Popping one hands its path end to the caller, after `makeDiversions` has
//! queued every alternative that merges into it from its diversion point back to its start: at
//! each pin, each fanin path whose arc into the pin is not the path's own and whose tag through
//! that arc matches the pin's (`Tag::matchNoCrpr`). A diverted path is the original from its
//! endpoint down to the pin, then the fanin path's own chain; its arrivals are summed again from
//! the fanin path forward (`updatePathHeadDelays`), and its slack is the endpoint's required time
//! less the new arrival.
//!
//! Modelled for an ideal clock and one scene, without latches; the caller refuses the rest.

use std::cmp::Ordering;

use crate::fuzzy;
use crate::graph::EdgeKind;
use crate::liberty::{Role, MAX};
use crate::search::{Path, Search, Tag, TagKey};

/// One pin of an enumerated path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Node {
    pub vertex: usize,
    pub tag: Tag,
    pub arrival: f32,
    /// The edge and arc into this pin from the next node (none at the start).
    pub prev: Option<(usize, usize)>,
}

/// One path end the enumeration returned: its path from the endpoint back to its start, and its
/// slack.
#[derive(Debug, Clone, PartialEq)]
pub struct PathEnd {
    /// Endpoint first.
    pub nodes: Vec<Node>,
    /// The endpoint's required time for this path (its path end's).
    pub required: f32,
    pub slack: f32,
}

impl PathEnd {
    /// `dataArrivalTime`.
    pub fn arrival(&self) -> f32 {
        self.nodes[0].arrival
    }
}

/// A queued alternative: its path end, and the node diversions are made from when it is popped.
#[derive(Debug, Clone)]
struct Diversion {
    end: PathEnd,
    div: usize,
}

/// The identity of a timing arc (`TimingArc*`): every wire edge shares the two wire arcs; a gate
/// arc is its cell's arc set and its index there.
fn arc_identity(s: &Search<'_, '_>, e: usize, arc: usize) -> (bool, usize, usize) {
    match s.graph.edges[e].kind {
        EdgeKind::Wire => (true, 0, arc),
        EdgeKind::Gate { set } => (false, set, arc),
    }
}

/// `PathEnd::cmp(.., cmp_slack = true)`: `cmpSlack` (fuzzy), then `Path::cmpPinTrClk` of the
/// ends' paths (one endpoint here: the transition, then the clock edge), then `Path::cmpAll` —
/// back from the endpoint, pin by pin, by vertex id then tag. The target clock paths are the
/// endpoint's and compare equal.
///
/// ⚠️ The tag order is a proxy for the reference's `TagIndex` (the order its tag table grew):
/// rise before fall, then this search's tag key. It decides only between paths whose slacks are
/// fuzzily equal and that differ first by tag at a common pin.
pub fn cmp_ends(s: &Search<'_, '_>, a: &PathEnd, b: &PathEnd) -> Ordering {
    if !fuzzy::equal(a.slack, b.slack) {
        return if fuzzy::less(a.slack, b.slack) { Ordering::Less } else { Ordering::Greater };
    }
    let (ta, tb) = (&a.nodes[0].tag, &b.nodes[0].tag);
    let pin_tr_clk = ta.rf.cmp(&tb.rf).then(ta.clk_edge.map_or(-1, |e| e as i64).cmp(&tb.clk_edge.map_or(-1, |e| e as i64)));
    if pin_tr_clk != Ordering::Equal {
        return pin_tr_clk;
    }
    // `Path::cmpAll`.
    for (p1, p2) in a.nodes.iter().zip(&b.nodes) {
        let c = s.vertex_id[p1.vertex].cmp(&s.vertex_id[p2.vertex]).then_with(|| tag_order(s, &p1.tag).cmp(&tag_order(s, &p2.tag)));
        if c != Ordering::Equal {
            return c;
        }
    }
    a.nodes.len().cmp(&b.nodes.len())
}

/// The proxy for `TagIndex` (see [`cmp_ends`]).
fn tag_order(s: &Search<'_, '_>, t: &Tag) -> (usize, TagKey) {
    (t.rf, t.key(&s.vertex_id))
}

/// The enumerator's state.
struct Enum<'s, 'g, 'a> {
    s: &'s Search<'g, 'a>,
    group_path_count: usize,
    endpoint_path_count: usize,
    queue: Vec<Diversion>,
    /// Paths returned so far (one endpoint: `path_counts_[vertex]`).
    path_count: usize,
}

impl Enum<'_, '_, '_> {
    /// `PathEnum::insert(Diversion*)`: queued; above twice the group count, pruned.
    fn insert(&mut self, div: Diversion) {
        self.queue.push(div);
        if self.queue.len() > self.group_path_count * 2 {
            self.prune();
        }
    }

    /// `pruneDiversionQueue`: the best `group_path_count` diversions kept, at most
    /// `endpoint_path_count` for the endpoint.
    fn prune(&mut self) {
        let s = self.s;
        self.queue.sort_by(|a, b| cmp_ends(s, &a.end, &b.end));
        self.queue.truncate(self.group_path_count.min(self.endpoint_path_count));
    }

    /// The top of the queue: the least by [`cmp_ends`] (the first queued of equals).
    fn pop(&mut self) -> Option<Diversion> {
        let s = self.s;
        let mut best = 0;
        for i in 1..self.queue.len() {
            if cmp_ends(s, &self.queue[i].end, &self.queue[best].end) == Ordering::Less {
                best = i;
            }
        }
        (!self.queue.is_empty()).then(|| self.queue.remove(best))
    }

    /// `findNext`: the next path end, its diversions queued.
    fn next(&mut self) -> Option<PathEnd> {
        while let Some(div) = self.pop() {
            self.path_count += 1;
            if self.path_count <= self.endpoint_path_count {
                self.make_diversions(&div.end, div.div);
                return Some(div.end);
            }
        }
        None
    }

    /// `makeDiversions`: from `before` back to the start, the fanin diversions at each pin; not
    /// past a register's (or latch's) clock-to-output arc.
    fn make_diversions(&mut self, end: &PathEnd, before: usize) {
        let s = self.s;
        let mut i = before;
        while i + 1 < end.nodes.len() {
            let (e, arc) = end.nodes[i].prev.expect("a node with a next one has its arc");
            self.visit_fanin_paths_thru(end, i, (e, arc));
            if matches!(s.role(e), Some(Role::LatchDtoQ | Role::RegClkToQ)) {
                break;
            }
            i += 1;
        }
    }

    /// `PathEnumFaninVisitor::visitFaninPathsThru`: each in-edge of the pin (newest first), each
    /// fanin path of the pin's min/max, each of its arcs to the pin's transition.
    fn visit_fanin_paths_thru(&mut self, end: &PathEnd, bd: usize, prev_arc: (usize, usize)) {
        let s = self.s;
        let before = end.nodes[bd];
        let v = before.vertex;
        let prev_id = arc_identity(s, prev_arc.0, prev_arc.1);
        for &e in s.graph.in_edges[v].iter().rev() {
            // `EnumPred`: no timing check, no latch D to Q (and no disabled loop edge — none here).
            if s.is_check(e) || s.role(e) == Some(Role::LatchDtoQ) {
                continue;
            }
            let from_v = s.graph.edges[e].from;
            for from in s.paths[from_v].iter().filter(|p| p.tag.mm == before.tag.mm) {
                for arc in s.arcs_from(e, from.tag.rf) {
                    if arc.to_rf != before.tag.rf {
                        continue;
                    }
                    let Some((to_tag, _, _)) = s.visit_from_path(from_v, from, e, &arc) else { continue };
                    if arc_identity(s, e, arc.index) == prev_id || !to_tag.matches_no_crpr(&before.tag) {
                        continue;
                    }
                    let div = self.make_diverted_path(end, bd, from_v, from, e, arc.index);
                    self.insert(div);
                }
            }
        }
    }

    /// `makeDivertedPath` + `updatePathHeadDelays`: the end's nodes down to `bd`, then the fanin
    /// path's own chain; arrivals summed forward from `clkPathArrival(after_div)`.
    fn make_diverted_path(&self, end: &PathEnd, bd: usize, from_v: usize, from: &Path, e: usize, arc: usize) -> Diversion {
        let s = self.s;
        let mut nodes: Vec<Node> = end.nodes[..=bd].to_vec();
        nodes[bd].prev = Some((e, arc));
        let div = nodes.len();
        nodes.extend(chain(s, from_v, from));
        let mut arrival = clk_path_arrival(s, from_v, from);
        for i in (0..=bd).rev() {
            let (pe, pa) = nodes[i].prev.expect("set above");
            arrival += s.graph.delay[pe][pa][nodes[i].tag.mm];
            nodes[i].arrival = arrival;
        }
        let slack = end.required - nodes[0].arrival;
        Diversion { end: PathEnd { nodes, required: end.required, slack }, div }
    }
}

/// `Search::clkPathArrival`: an IDEAL clock path at a register clock pin launches at its edge
/// (`(insertion + edge time) + latency`, both 0 here); any other path at its arrival.
fn clk_path_arrival(s: &Search<'_, '_>, v: usize, p: &Path) -> f32 {
    match p.tag.clk_edge {
        Some(edge) if s.is_reg_clk(v) && p.tag.is_clock && !s.sdc.clock.propagated => (0.0 + s.sdc.clock.edge_time(edge)) + 0.0,
        _ => p.arrival,
    }
}

/// A search path and its fanin chain back to its start, as nodes.
pub fn chain(s: &Search<'_, '_>, v: usize, p: &Path) -> Vec<Node> {
    let mut out = Vec::new();
    let (mut v, mut p) = (v, p);
    loop {
        out.push(Node { vertex: v, tag: p.tag, arrival: p.arrival, prev: p.prev.map(|x| (x.edge, x.arc)) });
        match s.prev_path(p) {
            Some((pv, pp)) => (v, p) = (pv, pp),
            None => break,
        }
    }
    out
}

impl Search<'_, '_> {
    /// `findPathEnds(to = end, group_path_count, endpoint_path_count, unique_pins = false,
    /// unique_edges = false, setup)`: the endpoint's max path ends with a required time, sorted
    /// (`MakePathEndsAll`: by slack, at most `endpoint_path_count`), enumerated
    /// (`enumPathEnds`: at most `group_path_count`), and returned sorted by [`cmp_ends`].
    pub fn endpoint_path_ends(&self, end: usize, group_path_count: usize, endpoint_path_count: usize) -> Vec<PathEnd> {
        let mut seeds: Vec<PathEnd> = self.paths[end]
            .iter()
            .filter(|p| p.tag.mm == MAX && p.tag.clk_edge.is_some() && p.required < crate::search::INF)
            .map(|p| PathEnd { nodes: chain(self, end, p), required: p.required, slack: p.required - p.arrival })
            .collect();
        seeds.sort_by(|a, b| cmp_ends(self, a, b));
        seeds.truncate(endpoint_path_count);
        let mut en = Enum { s: self, group_path_count, endpoint_path_count, queue: Vec::new(), path_count: 0 };
        for seed in seeds {
            // `PathEnum::insert(PathEnd*)`: queued without pruning.
            en.queue.push(Diversion { end: seed, div: 0 });
        }
        let mut out = Vec::new();
        while out.len() < group_path_count {
            let Some(pe) = en.next() else { break };
            out.push(pe);
        }
        out.sort_by(|a, b| cmp_ends(self, a, b));
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::Graph;
    use crate::liberty::{Library, FALL, RISE};
    use crate::netlist::{Conn, Net, Netlist, PortDir};
    use crate::sdc::{Clock, PortDelay, Sdc};
    use std::collections::HashMap;

    const LIB: &str = r#"library (t) {
      time_unit : "1ns"; capacitive_load_unit (1, pf);
      cell (and2) {
        pin (A) { direction : input; capacitance : 0.001; }
        pin (B) { direction : input; capacitance : 0.001; }
        pin (Y) { direction : output; function : "A&B";
          timing () { related_pin : "A"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("0.1"); } rise_transition (scalar) { values ("0.05"); }
            cell_fall (scalar) { values ("0.1"); } fall_transition (scalar) { values ("0.05"); } }
          timing () { related_pin : "B"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("0.2"); } rise_transition (scalar) { values ("0.05"); }
            cell_fall (scalar) { values ("0.2"); } fall_transition (scalar) { values ("0.05"); } } }
      }
    }"#;

    /// a, b -> u1 (and2) -> y; ideal clock, period 1 ns; input and output delays 0.2 ns.
    fn enumerate(endpoint_path_count: usize) -> Vec<(Vec<String>, usize, f32)> {
        let libs = vec![Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let i = |p: &str| Conn::Inst(0, p.into());
        let net = |n: &str, pins: Vec<Conn>| Net { name: n.into(), pins };
        let netlist = Netlist {
            insts: vec![("u1".into(), "and2".into())],
            ports: vec![("clk".into(), PortDir::Input), ("a".into(), PortDir::Input), ("b".into(), PortDir::Input), ("y".into(), PortDir::Output)],
            nets: vec![net("clk", vec![Conn::Port(0)]), net("a", vec![i("A"), Conn::Port(1)]), net("b", vec![i("B"), Conn::Port(2)]), net("y", vec![i("Y"), Conn::Port(3)])],
        };
        let sdc = Sdc {
            clock: Clock::new("c", 1e-9, "clk", false),
            input_delays: vec![PortDelay::uniform("a", 0.2e-9), PortDelay::uniform("b", 0.2e-9)],
            output_delays: vec![PortDelay::uniform("y", 0.2e-9)],
            path_delays: Vec::new(),
        };
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let mut s = Search::in_graph_order(&g, &sdc);
        s.find_arrivals().unwrap();
        s.find_requireds().unwrap();
        let y = g.vertices.iter().position(|x| x.name == "y").unwrap();
        s.endpoint_path_ends(y, 100, endpoint_path_count)
            .iter()
            .map(|pe| (pe.nodes.iter().map(|n| g.vertices[n.vertex].name.clone()).collect(), pe.nodes[0].tag.rf, pe.slack))
            .collect()
    }

    /// Rule (`PathEnum`): the seeds are the endpoint's own worst paths (one per transition); each
    /// popped path queues the fanin paths merging into it — here u1/A's into u1/Y, its arrival
    /// summed again from a (0.2 + 0.1). Equal slacks order by transition, rise first; slack is
    /// the endpoint's required (1 − 0.2) less the arrival.
    #[test]
    fn enumerates_every_fanin_alternative_in_slack_order() {
        let ends = enumerate(100);
        let via = |v: &[String]| v.iter().any(|n| n == "b");
        assert_eq!(ends.len(), 4);
        assert_eq!((via(&ends[0].0), ends[0].1), (true, RISE));
        assert_eq!((via(&ends[1].0), ends[1].1), (true, FALL));
        assert_eq!((via(&ends[2].0), ends[2].1), (false, RISE));
        assert_eq!((via(&ends[3].0), ends[3].1), (false, FALL));
        assert_eq!(ends[2].0, vec!["y", "u1/Y", "u1/A", "a"]);
        let req = 1e-9f32 - 0.2e-9;
        assert_eq!(ends[0].2, req - ((0.2e-9f32 + 0.0) + 0.2e-9 + 0.0));
        assert_eq!(ends[2].2, req - ((0.2e-9f32 + 0.0) + 0.1e-9 + 0.0));
    }

    /// Rule (`findNext`): at most `endpoint_path_count` path ends for the endpoint.
    #[test]
    fn stops_at_the_endpoint_path_count() {
        assert_eq!(enumerate(3).len(), 3);
    }
}
