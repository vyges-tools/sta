// SPDX-License-Identifier: Apache-2.0
//! Path groups: which group a path end is in, and a named group's path ends — `PathGroups` for
//! the groups `group_path -name` makes (`GroupPath`).
//!
//! A group path moves no slack. It rides on the tags as any exception does (the search's
//! `exceptionFromStates` / `mutateTag`), so a vertex keeps a path per group state, and at an
//! endpoint [`crate::sdc::Sdc::group_paths_to`] says which groups the path's end is in.
//!
//! [`Search::group_path_ends`] is `findPathEnds(to = none, group_names = {name},
//! endpoint_path_count = 1, unique_pins, sort_by_slack)`: `MakePathEnds1` keeps, per endpoint,
//! the group's worst end (`PathEndLess`), then the group's ends sorted by [`cmp_group_ends`].
//! Named groups only, of one scene; the default groups (clocks, `asynchronous`, `gated clock`,
//! `path delay`, `unconstrained`) are not modelled here.

use std::cmp::Ordering;

use crate::fuzzy;
use crate::liberty::MAX;
use crate::search::{Path, Search};

/// A named group's path end at one endpoint: the vertex, the index of its path in
/// [`Search::paths`], and its slack.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroupEnd {
    pub vertex: usize,
    pub path: usize,
    pub slack: f32,
}

/// A path's slack at its end: required less arrival for max, arrival less required for min.
fn path_slack(p: &Path) -> f32 {
    if p.tag.mm == MAX {
        p.required - p.arrival
    } else {
        p.arrival - p.required
    }
}

/// `PathEnd::cmp(cmp_slack = true)` between two group ends: `cmpSlack` (fuzzy), then
/// `Path::cmpPinTrClk` — the pins by name (`pathNameLess`), at one pin the transition, then the
/// clock edge. The rest (`cmpAll` back along the paths) is not reached by ends of one group with
/// one end per pin.
pub fn cmp_group_ends(s: &Search<'_, '_>, a: &GroupEnd, b: &GroupEnd) -> Ordering {
    if !fuzzy::equal(a.slack, b.slack) {
        return if fuzzy::less(a.slack, b.slack) { Ordering::Less } else { Ordering::Greater };
    }
    let (pa, pb) = (&s.paths[a.vertex][a.path], &s.paths[b.vertex][b.path]);
    if a.vertex != b.vertex {
        return s.graph.vertices[a.vertex].name.cmp(&s.graph.vertices[b.vertex].name);
    }
    pa.tag.rf.cmp(&pb.tag.rf).then(pa.tag.clk_edge.map_or(-1, |e| e as i64).cmp(&pb.tag.clk_edge.map_or(-1, |e| e as i64)))
}

impl Search<'_, '_> {
    /// `PathGroups::inPathGroupNamed(path_end, name)` for a named group: the path's end at `v` is
    /// in it when one of the group paths [`crate::sdc::Sdc::group_paths_to`] finds is of that name
    /// (`GroupPaths` have precedence; a named group's ends are only these).
    pub fn path_in_group(&self, v: usize, p: &Path, name: &str) -> bool {
        let clocked = p.tag.clk_edge.is_some() && self.has_target_clock(v);
        self.sdc.group_paths_to(p.tag.states, &self.graph.vertices[v].name, clocked).into_iter().any(|id| self.sdc.path_delays[id].group.as_deref() == Some(name))
    }

    /// `Sta::findPathEnds(to = none, group_names = {name}, group_path_count, endpoint_path_count =
    /// 1, unique_pins, slack_min, slack_max, sort_by_slack)` for setup (`mm` = max) or hold: over
    /// the endpoints in vertex id order (`Search::endpoints`, a `VertexSet`), each path of `mm`
    /// with a required time (a constrained end) and not a segment start, in the group — the
    /// worst kept per endpoint, a later path only when strictly less (`MakePathEnds1`), and only
    /// when saveable (slack within `[slack_min, slack_max]`, `PathGroup::saveable`); then sorted
    /// ([`cmp_group_ends`]) and the first `group_path_count` kept (`PathGroup::prune`).
    pub fn group_path_ends(&self, name: &str, mm: usize, group_path_count: usize, slack_min: f32, slack_max: f32) -> Vec<GroupEnd> {
        let mut endpoints: Vec<usize> = (0..self.graph.vertices.len()).filter(|&v| self.is_endpoint(v)).collect();
        endpoints.sort_by_key(|&v| self.vertex_id[v]);
        let mut ends: Vec<GroupEnd> = Vec::new();
        for v in endpoints {
            let mut worst: Option<GroupEnd> = None;
            for (i, p) in self.paths[v].iter().enumerate() {
                if p.tag.mm != mm || p.tag.segment_start || p.required.abs() >= crate::search::INF || !self.path_in_group(v, p, name) {
                    continue;
                }
                let slack = path_slack(p);
                if !(fuzzy::less_equal(slack, slack_max) && !fuzzy::less(slack, slack_min)) {
                    continue;
                }
                let end = GroupEnd { vertex: v, path: i, slack };
                let replace = match worst {
                    None => true,
                    Some(w) => cmp_group_ends(self, &end, &w) == Ordering::Less,
                };
                if replace {
                    worst = Some(end);
                }
            }
            ends.extend(worst);
        }
        ends.sort_by(|a, b| cmp_group_ends(self, a, b));
        ends.truncate(group_path_count);
        ends
    }
}

/// `Sta::findRegisterClkPins` (`data` false) / `findRegisterDataPins` (`data` true) with no
/// clocks, rise and fall, registers and latches — `all_registers -clock_pins` / `-data_pins`.
/// Without clocks `FindRegVisitor::visitRegs` takes every register clock vertex
/// (`Graph::regClkVertices`: a pin some `reg_clk_to_q` / `latch_en_to_q` arc leaves) and, the
/// search being specific to neither kind nor edge, visits its instance whatever its sequentials
/// (`visitReg`): each of the instance's pins that matches. A clock pin matches when such an arc
/// leaves it (`FindRegClkPins::matchPin`); a data pin when a timing check ends at it
/// (`Vertex::hasChecks`) and its port has neither `min_period` nor `min_pulse_width_high` /
/// `_low` (`FindRegDataPins::matchPin`). Vertices, in the graph's order (a set: the order is not
/// the reference's, nor read).
pub fn register_pins(g: &crate::graph::Graph<'_>, data: bool) -> Vec<usize> {
    use crate::graph::EdgeKind;
    use crate::liberty::Role;
    let reg_clk = |v: usize| {
        g.out_edges[v].iter().any(|&e| match g.edges[e].kind {
            EdgeKind::Gate { set } => matches!(g.arc_set(e, set).role, Role::RegClkToQ | Role::LatchEnToQ),
            EdgeKind::Wire => false,
        })
    };
    let inst_of = |v: usize| match &g.vertices[v].conn {
        crate::netlist::Conn::Inst(i, _) => Some(*i),
        crate::netlist::Conn::Port(_) => None,
    };
    let regs: std::collections::BTreeSet<usize> = (0..g.vertices.len()).filter(|&v| reg_clk(v)).filter_map(inst_of).collect();
    let port_timed = |v: usize| {
        let x = &g.vertices[v];
        let port = x.lib.zip(x.cell.as_deref()).zip(x.port.as_deref()).and_then(|((l, c), p)| g.libs[l].cells.get(c).and_then(|c| c.port(p)));
        port.is_some_and(|p| p.min_period.is_some() || p.min_pulse_width.iter().any(Option::is_some))
    };
    (0..g.vertices.len())
        .filter(|&v| inst_of(v).is_some_and(|i| regs.contains(&i)))
        .filter(|&v| if data { !g.vertices[v].is_driver && g.in_edges[v].iter().any(|&e| g.is_check(e)) && !port_timed(v) } else { reg_clk(v) })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::Graph;
    use crate::liberty::Library;
    use crate::netlist::{Conn, Net, Netlist, PortDir};
    use crate::sdc::{Clock, PathDelay, PortDelay, Sdc};
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

    /// a, b -> u1 (and2: A 0.1 ns, B 0.2 ns) -> y; ideal clock, period 1 ns; input and output
    /// delays 0.2 ns; one group path `ga -from a -to y`. Its ends at y between `slack_min` and
    /// `slack_max` (ns), as (pin, slack in ns).
    fn ends(slack_min: f32, slack_max: f32) -> Vec<(String, f32)> {
        let libs = vec![Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let i = |p: &str| Conn::Inst(0, p.into());
        let net = |n: &str, pins: Vec<Conn>| Net { name: n.into(), pins }.into();
        let netlist = Netlist {
            insts: vec![crate::netlist::inst("u1", "and2")],
            ports: vec![("clk".into(), PortDir::Input), ("a".into(), PortDir::Input), ("b".into(), PortDir::Input), ("y".into(), PortDir::Output)],
            nets: vec![net("clk", vec![Conn::Port(0)]), net("a", vec![i("A"), Conn::Port(1)]), net("b", vec![i("B"), Conn::Port(2)]), net("y", vec![i("Y"), Conn::Port(3)])],
        };
        let ga = PathDelay { from_pins: vec!["a".into()], from_clock: false, to_pins: vec!["y".into()], to_clock: false, min_max: MAX, ignore_clk_latency: false, break_path: false, delay: 0.0, false_path: false, multicycle: None, thrus: Vec::new(), group: Some("ga".into()) };
        let sdc = Sdc {
            clocks: vec![Clock::new("c", 1e-9, "clk", false)],
            input_delays: vec![PortDelay::uniform("a", 0.2e-9), PortDelay::uniform("b", 0.2e-9)],
            output_delays: vec![PortDelay::uniform("y", 0.2e-9)],
            path_delays: vec![ga],
        };
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let mut s = Search::in_graph_order(&g, &sdc);
        s.find_arrivals().unwrap();
        s.find_requireds().unwrap();
        s.group_path_ends("ga", MAX, usize::MAX, slack_min * 1e-9, slack_max * 1e-9).iter().map(|e| (g.vertices[e.vertex].name.clone(), e.slack * 1e9)).collect()
    }

    /// Rules (`MakePathEnds1`, `PathGroup::saveable`): a group's end at an endpoint is the worst of
    /// the GROUP's paths there — via a (slack 1 − 0.2 − 0.2 − 0.1 = 0.5 ns), not the endpoint's
    /// worst path, which comes from b (0.4 ns) and is in no group — and it is kept only when its
    /// slack is within `[slack_min, slack_max]` (fuzzily).
    #[test]
    fn a_group_end_is_the_groups_worst_path_within_the_slack_window() {
        let all = ends(-1.0, 1.0);
        assert_eq!(all.len(), 1);
        assert_eq!(all[0].0, "y");
        assert!((all[0].1 - 0.5).abs() < 1e-4, "the path via a, not via b: {all:?}");
        assert!(ends(-1.0, 0.45).is_empty(), "above slack_max");
        assert!(ends(0.55, 1.0).is_empty(), "under slack_min");
        assert_eq!(ends(0.5, 0.5).len(), 1, "the bounds are inclusive");
    }

    const SEQ: &str = r#"library (s) {
      time_unit : "1ns"; capacitive_load_unit (1, pf);
      cell (dff) {
        ff (IQ, IQN) { clocked_on : "CK"; next_state : "D"; clear : "!RN"; }
        pin (CK) { direction : input; clock : true; capacitance : 0.001; }
        pin (D) { direction : input; capacitance : 0.001;
          timing () { related_pin : "CK"; timing_type : setup_rising;
            rise_constraint (scalar) { values ("0.05"); } fall_constraint (scalar) { values ("0.05"); } } }
        pin (RN) { direction : input; capacitance : 0.001; min_pulse_width_low : 0.1;
          timing () { related_pin : "CK"; timing_type : recovery_rising;
            rise_constraint (scalar) { values ("0.05"); } } }
        pin (SI) { direction : input; capacitance : 0.001;
          timing () { related_pin : "CK"; timing_type : hold_rising;
            rise_constraint (scalar) { values ("0.01"); } fall_constraint (scalar) { values ("0.01"); } } }
        pin (Q) { direction : output; function : "IQ";
          timing () { related_pin : "CK"; timing_type : rising_edge;
            cell_rise (scalar) { values ("0.1"); } rise_transition (scalar) { values ("0.05"); }
            cell_fall (scalar) { values ("0.1"); } fall_transition (scalar) { values ("0.05"); } } }
      }
      cell (buf) {
        pin (A) { direction : input; capacitance : 0.001; }
        pin (X) { direction : output; function : "A";
          timing () { related_pin : "A"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("0.1"); } rise_transition (scalar) { values ("0.05"); }
            cell_fall (scalar) { values ("0.1"); } fall_transition (scalar) { values ("0.05"); } } }
      }
    }"#;

    /// Rules (`FindRegClkPins::matchPin`, `FindRegDataPins::matchPin`, `visitReg`): a register's
    /// clock pin is the one a clock-to-Q arc leaves; its data pins are the pins a timing check ends
    /// at — any check, so a hold-only SI too — less a pin whose port has a min pulse width (RN:
    /// `min_pulse_width_low`); a buffer has neither.
    #[test]
    fn a_registers_clock_and_data_pins() {
        let libs = vec![Library::read(&crate::liberty_parse::parse(SEQ).unwrap()).unwrap()];
        let i = |k: usize, p: &str| Conn::Inst(k, p.into());
        let net = |n: &str, pins: Vec<Conn>| Net { name: n.into(), pins }.into();
        let netlist = Netlist {
            insts: vec![crate::netlist::inst("r", "dff"), crate::netlist::inst("b", "buf")],
            ports: vec![("clk".into(), PortDir::Input), ("d".into(), PortDir::Input), ("q".into(), PortDir::Output)],
            nets: vec![net("clk", vec![i(0, "CK"), Conn::Port(0)]), net("d", vec![i(0, "D"), i(0, "RN"), i(0, "SI"), Conn::Port(1)]), net("n", vec![i(0, "Q"), i(1, "A")]), net("q", vec![i(1, "X"), Conn::Port(2)])],
        };
        let g = Graph::build(&libs, &netlist).unwrap();
        let names = |data: bool| {
            let mut v: Vec<String> = register_pins(&g, data).into_iter().map(|v| g.vertices[v].name.clone()).collect();
            v.sort();
            v
        };
        assert_eq!(names(false), ["r/CK"]);
        assert_eq!(names(true), ["r/D", "r/SI"]);
    }
}
