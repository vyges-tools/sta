// SPDX-License-Identifier: Apache-2.0
//! Arrival, required and slack search over a delay-calculated graph, for one clock — propagated,
//! or ideal (launched at its edge, captured with no network latency) — with input and output delays, setup checks and common-path pessimism removal (CRPR).
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
use crate::sdc::{PortDelay, Sdc, States, ACCT_HOLD, ACCT_LATCH_SETUP, ACCT_SETUP};

/// The min/max initial value (`INF` = 1e30).
pub(crate) const INF: f32 = 1e30;

/// A slack with no path: [`INF`].
pub const INF_SLACK: f32 = INF;

/// A tag's CRPR clock path: the vertex (its id orders tags) and that path's own tag.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CrprPath {
    pub vertex: usize,
    pub rf: usize,
    pub mm: usize,
    pub clk_edge: usize,
}

/// [`Tag::key`]: transition, min/max, clock edge, is-clock, segment start, CRPR vertex id, states.
pub(crate) type TagKey = (usize, usize, i64, bool, bool, i64, (u32, u64));

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Tag {
    pub rf: usize,
    pub mm: usize,
    /// The clock edge (index = its transition: one clock), none for an unclocked path.
    pub clk_edge: Option<usize>,
    pub is_clock: bool,
    pub crpr: Option<CrprPath>,
    /// The path delays the path is under (`Tag::states`).
    pub states: States,
    /// `ClkInfo::latency` as bits: an ideal clock's network latency set on a pin of its network
    /// (`set_clock_latency`), carried from that pin on. Part of the tag's identity (`Tag::cmp`),
    /// never of its match (`Tag::matchCmp`): paths that differ only in it merge.
    pub latency: u32,
    /// `Tag::isSegmentStart`: an unclocked path seeded at a register clock pin no clock reaches
    /// (it alone launches through the register's clock-to-Q).
    pub segment_start: bool,
}

impl Tag {
    /// `TagMatchLess(match_crpr_clk_pin = true)` key: the CRPR path compares by its vertex's id;
    /// the exception states last (`Tag::stateCmp`).
    pub(crate) fn key(&self, vertex_id: &[usize]) -> TagKey {
        (self.rf, self.mm, self.clk_edge.map_or(-1, |e| e as i64), self.is_clock, self.segment_start, self.crpr.map_or(-1, |c| vertex_id[c.vertex] as i64), self.states.key())
    }

    /// Tags equal, the CRPR clock vertex and the states included.
    fn matches(&self, other: &Tag) -> bool {
        self.rf == other.rf && self.mm == other.mm && self.clk_edge == other.clk_edge && self.is_clock == other.is_clock && self.segment_start == other.segment_start && self.crpr.map(|c| c.vertex) == other.crpr.map(|c| c.vertex) && self.states == other.states
    }

    /// Everything but the CRPR clock pin (`Tag::matchNoCrpr`: the states included, the segment
    /// start not).
    pub(crate) fn matches_no_crpr(&self, other: &Tag) -> bool {
        self.rf == other.rf && self.mm == other.mm && self.clk_edge == other.clk_edge && self.is_clock == other.is_clock && self.states == other.states
    }

    /// Transition, clock edge, is-clock — NOT min/max, and not the states: `stateEqualCrpr`
    /// compares loop states only, and there are none here.
    fn matches_crpr(&self, other: &Tag) -> bool {
        self.rf == other.rf && self.clk_edge == other.clk_edge && self.is_clock == other.is_clock && self.segment_start == other.segment_start
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
pub(crate) struct ArcRef {
    pub(crate) index: usize,
    pub(crate) to_rf: usize,
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
    pub(crate) vertex_id: Vec<usize>,
    /// Each port's input delays, one per clock (indices into [`Sdc::input_delays`]), in the
    /// order they were made.
    input_delay: HashMap<String, Vec<usize>>,
    /// Each port's output delays, one per clock, likewise.
    output_delay: HashMap<String, Vec<usize>>,
    /// A vertex whose out-edges include a clock-to-Q arc.
    is_reg_clk: Vec<bool>,
    /// `Levelize`: roots 0, else the longest fanin level + 1 over non-check edges.
    level: Vec<usize>,
    /// `Sdc::isPathDelayInternalFrom` / `…FromBreak` / `…To` / `…ToBreak`: a path delay's
    /// `-from` pin that is not an exception startpoint (a top input or a register clock pin), its
    /// `-to` pin with no timing check that is not a top port; "break" unless `-probe`.
    internal_from: Vec<bool>,
    internal_from_break: Vec<bool>,
    internal_to: Vec<bool>,
    internal_to_break: Vec<bool>,
    /// Per edge, `Sim::isDisabledCond` and `Sim::simTimingSense` on instances with a constant pin.
    sim: SimEdges,
    /// Per latch D -> Q edge, the enable vertex and edge (`LibertyCell::latchEnable`).
    latch_en: HashMap<usize, (usize, usize)>,
    /// A latch's data pin (`LibertyPort::isLatchData`): it has a D -> Q edge with an enable.
    latch_data: Vec<bool>,
    /// `ClkNetwork::isClock`: the vertices the clocks reach from their sources over wires and
    /// combinational arcs.
    clk_network: Vec<bool>,
    /// `Sdc::isLeafPinClock`: a clock's source pin — a port or any instance pin, both vertices of
    /// a bidirect one.
    clock_leaf: Vec<bool>,
    /// `GatedClk::isGatedClkEnable`: an enable pin, its gate's clock pin and whether the gate's
    /// active value is 1 (AND-like) or 0 (OR-like) — when the clock pin reaches a register clock
    /// pin (`hasDownstreamClkPin`).
    gated: HashMap<usize, (usize, bool)>,
}

/// `ClkNetwork`: from every clock's source, over wires and combinational arcs, never into another
/// clock's source.
fn clock_network(graph: &Graph<'_>, sdc: &Sdc) -> Vec<bool> {
    let n = graph.vertices.len();
    let mut seen = vec![false; n];
    let sources: Vec<usize> = sdc.clocks.iter().filter(|c| !c.source.is_empty()).filter_map(|c| graph.vertices.iter().position(|v| v.name == c.source)).collect();
    let mut queue: std::collections::VecDeque<usize> = sources.iter().copied().collect();
    for &v in &sources {
        seen[v] = true;
    }
    while let Some(v) = queue.pop_front() {
        for &e in &graph.out_edges[v] {
            let thru = match graph.edges[e].kind {
                EdgeKind::Wire => true,
                EdgeKind::Gate { set } => graph.arc_set(e, set).role == Role::Combinational,
            };
            let to = graph.edges[e].to;
            if thru && !sources.contains(&to) && !seen[to] {
                seen[to] = true;
                queue.push_back(to);
            }
        }
    }
    seen
}

/// `GatedClk::isClkGatingFunc`: under its outer negations the function is an AND (active value 1)
/// or an OR (0) whose operands — the maximal subexpressions of another operator — include the
/// clock port, or its negation (which flips the active value), and an operand naming the enable
/// port. Returns the active value.
fn clk_gating_func(func: &crate::func_expr::FuncExpr, enable: &str, clk: &str) -> Option<bool> {
    use crate::func_expr::FuncExpr as F;
    let mut f = func;
    while let F::Not(a) = f {
        f = a;
    }
    let mut one = match f {
        F::And(..) => true,
        F::Or(..) => false,
        _ => return None,
    };
    fn operands<'e>(root: &F, e: &'e F, out: &mut Vec<&'e F>) {
        match (root, e) {
            (F::And(..), F::And(a, b)) | (F::Or(..), F::Or(a, b)) => {
                operands(root, a, out);
                operands(root, b, out);
            }
            _ => out.push(e),
        }
    }
    let mut ops = Vec::new();
    if let F::And(a, b) | F::Or(a, b) = f {
        operands(f, a, &mut ops);
        operands(f, b, &mut ops);
    }
    let mut need = false;
    for e in &ops {
        match e {
            F::Not(a) if matches!(a.as_ref(), F::Port(p) if p == clk) => {
                need = true;
                one = !one;
            }
            F::Port(p) if p == clk => need = true,
            _ => {}
        }
    }
    if need && ops.iter().any(|e| {
        let mut ports = Vec::new();
        func_ports(e, &mut ports);
        ports.iter().any(|p| p == enable)
    }) {
        return Some(one);
    }
    None
}

/// `Sim::functionSense`'s answer for an edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimSense {
    Unknown,
    Positive,
    Negative,
    NonUnate,
    None,
}

/// The edges `Sim::findDisabledEdges` annotates.
#[derive(Debug, Clone, Default)]
struct SimEdges {
    disabled_cond: Vec<bool>,
    sense: Vec<SimSense>,
    /// What could not be annotated (too many free inputs to enumerate).
    error: Option<String>,
}

/// `LibertyCell::makeLatchEnables` → `latchEnable(d_to_q)` for every latch D -> Q edge: the latch
/// EN -> Q arc set to the same Q (one whose `when` matches the D -> Q's preferred), its from port as
/// the enable and its edge (`rising_edge` / `falling_edge`); and each such D pin.
fn latch_enables(graph: &Graph<'_>) -> (HashMap<usize, (usize, usize)>, Vec<bool>) {
    let mut en = HashMap::new();
    let mut data = vec![false; graph.vertices.len()];
    let d_q: Vec<usize> = (0..graph.edges.len()).filter(|&e| matches!(graph.edges[e].kind, EdgeKind::Gate { set } if graph.arc_set(e, set).role == Role::LatchDtoQ)).collect();
    if d_q.is_empty() {
        return (en, data);
    }
    let index = graph.pin_index();
    for e in d_q {
        let EdgeKind::Gate { set } = graph.edges[e].kind else { continue };
        let to = &graph.vertices[graph.edges[e].to];
        let (Some(lib), Some(cell_name), crate::netlist::Conn::Inst(i, _)) = (to.lib, to.cell.as_deref(), &to.conn) else { continue };
        let cell = &graph.libs[lib].cells[cell_name];
        let dq = graph.arc_set(e, set);
        let ens: Vec<&crate::liberty::ArcSet> = cell.arc_sets.iter().filter(|a| a.role == Role::LatchEnToQ && a.to == dq.to).collect();
        let Some(en_to_q) = ens.iter().find(|a| a.cond == dq.cond).or(ens.last()) else { continue };
        let en_rf = match en_to_q.timing_type.as_str() {
            "rising_edge" => crate::liberty::RISE,
            "falling_edge" => crate::liberty::FALL,
            _ => continue,
        };
        let Some(&en_v) = index.get(&format!("{}/{}", graph.netlist.insts[*i].0, en_to_q.from)) else { continue };
        en.insert(e, (en_v, en_rf));
        data[graph.edges[e].from] = true;
    }
    (en, data)
}

/// A function's value with every port given (`value`).
fn eval_func(e: &crate::func_expr::FuncExpr, value: &dyn Fn(&str) -> bool) -> bool {
    use crate::func_expr::FuncExpr as F;
    match e {
        F::Port(p) => value(p),
        F::One => true,
        F::Zero => false,
        F::Not(a) => !eval_func(a, value),
        F::And(a, b) => eval_func(a, value) && eval_func(b, value),
        F::Or(a, b) => eval_func(a, value) || eval_func(b, value),
        F::Xor(a, b) => eval_func(a, value) != eval_func(b, value),
    }
}

fn func_ports(e: &crate::func_expr::FuncExpr, out: &mut Vec<String>) {
    use crate::func_expr::FuncExpr as F;
    match e {
        F::Port(p) => {
            if !out.contains(p) {
                out.push(p.clone());
            }
        }
        F::Not(a) => func_ports(a, out),
        F::And(a, b) | F::Or(a, b) | F::Xor(a, b) => {
            func_ports(a, out);
            func_ports(b, out);
        }
        F::One | F::Zero => {}
    }
}

/// Every assignment of the ports `fixed` leaves free (but `skip`), as a lookup.
const MAX_FREE: usize = 16;

fn free_ports(e: &crate::func_expr::FuncExpr, fixed: &dyn Fn(&str) -> Option<bool>, skip: Option<&str>) -> Result<Vec<String>, String> {
    let mut ports = Vec::new();
    func_ports(e, &mut ports);
    ports.retain(|p| fixed(p).is_none() && Some(p.as_str()) != skip);
    if ports.len() > MAX_FREE {
        return Err(format!("{} free inputs", ports.len()));
    }
    Ok(ports)
}

/// `Sim::evalExpr`: the function's value under the constants — 0 or 1 when every assignment of the
/// free ports agrees (the BDD is a constant), else unknown.
fn eval_exact(e: &crate::func_expr::FuncExpr, fixed: &dyn Fn(&str) -> Option<bool>) -> Result<Option<bool>, String> {
    let free = free_ports(e, fixed, None)?;
    let mut seen = [false; 2];
    for bits in 0..1u32 << free.len() {
        let v = eval_func(e, &|p| free.iter().position(|f| f == p).map_or_else(|| fixed(p).unwrap_or(false), |k| (bits >> k) & 1 == 1));
        seen[usize::from(v)] = true;
    }
    Ok(match seen {
        [true, false] => Some(false),
        [false, true] => Some(true),
        _ => None,
    })
}

/// `Sim::functionSense`: increasing and decreasing in `input` over every assignment of the other
/// free ports (`Cudd_Increasing` / `Cudd_Decreasing`); both — independent — is none.
fn function_sense(e: &crate::func_expr::FuncExpr, input: &str, fixed: &dyn Fn(&str) -> Option<bool>) -> Result<SimSense, String> {
    let free = free_ports(e, fixed, Some(input))?;
    let (mut increasing, mut decreasing) = (true, true);
    for bits in 0..1u32 << free.len() {
        let at = |x: bool| eval_func(e, &|p| if p == input { x } else { free.iter().position(|f| f == p).map_or_else(|| fixed(p).unwrap_or(false), |k| (bits >> k) & 1 == 1) });
        let (lo, hi) = (at(false), at(true));
        increasing &= !lo || hi;
        decreasing &= lo || !hi;
    }
    Ok(match (increasing, decreasing) {
        (true, true) => SimSense::None,
        (true, false) => SimSense::Positive,
        (false, true) => SimSense::Negative,
        (false, false) => SimSense::NonUnate,
    })
}

/// `Sim::findDisabledEdges` over the instances with a constant pin: per gate edge, its sense
/// (`functionSense`: none from a constant; the to-port function's sense in the from-port when the
/// function names it; else unknown) and, unless none, `isDisabledCond` (its `when` evaluates to 0;
/// a default arc set — no `when` — when another arc set between the same ports evaluates to 1).
fn sim_edges(graph: &Graph<'_>) -> SimEdges {
    let n = graph.edges.len();
    let mut sim = SimEdges { disabled_cond: vec![false; n], sense: vec![SimSense::Unknown; n], error: None };
    let constants = &graph.sdc.constants;
    if constants.is_empty() {
        return sim;
    }
    let inst_name = |v: usize| match &graph.vertices[v].conn {
        crate::netlist::Conn::Inst(i, _) => Some(graph.netlist.insts[*i].0.as_str()),
        crate::netlist::Conn::Port(_) => None,
    };
    let annotated: std::collections::HashSet<&str> = graph.vertices.iter().enumerate().filter(|(_, x)| constants.contains_key(&x.name)).filter_map(|(v, _)| inst_name(v)).collect();
    for (e, ed) in graph.edges.iter().enumerate() {
        let EdgeKind::Gate { set } = ed.kind else { continue };
        let Some(inst) = inst_name(ed.to) else { continue };
        if !annotated.contains(inst) {
            continue;
        }
        let to = &graph.vertices[ed.to];
        let (Some(lib), Some(cell_name)) = (to.lib, to.cell.as_deref()) else { continue };
        let cell = &graph.libs[lib].cells[cell_name];
        let arc_set = &cell.arc_sets[set];
        let fixed = |p: &str| constants.get(&format!("{inst}/{p}")).copied();
        let parse = |f: &str| crate::func_expr::FuncExpr::parse(f).ok();
        let sense = if graph.is_constant(ed.from) {
            Ok(SimSense::None)
        } else {
            match cell.ports.iter().find(|p| p.name == arc_set.to).and_then(|p| p.function.as_deref()).and_then(parse) {
                Some(f) => {
                    let mut ports = Vec::new();
                    func_ports(&f, &mut ports);
                    if ports.contains(&arc_set.from) {
                        function_sense(&f, &arc_set.from, &fixed)
                    } else {
                        Ok(SimSense::Unknown)
                    }
                }
                None => Ok(SimSense::Unknown),
            }
        };
        let sense = match sense {
            Ok(s) => s,
            Err(why) => {
                sim.error.get_or_insert(format!("{inst}/{}: {why} under constants", arc_set.to));
                continue;
            }
        };
        sim.sense[e] = sense;
        if sense == SimSense::None {
            continue;
        }
        let cond_is = |c: &str, want: bool| -> Result<bool, String> { Ok(parse(c).map(|f| eval_exact(&f, &fixed)).transpose()?.flatten() == Some(want)) };
        let disabled = match arc_set.cond.as_deref() {
            Some(c) => cond_is(c, false),
            None => cell.arc_sets.iter().filter(|o| o.from == arc_set.from && o.to == arc_set.to).filter_map(|o| o.cond.as_deref()).try_fold(false, |d, c| Ok::<bool, String>(d || cond_is(c, true)?)),
        };
        match disabled {
            Ok(d) => sim.disabled_cond[e] = d,
            Err(why) => {
                sim.error.get_or_insert(format!("{inst}: a when condition: {why} under constants"));
            }
        }
    }
    sim
}

fn is_check_role(r: Role) -> bool {
    matches!(r, Role::Setup | Role::Hold | Role::Recovery | Role::Removal)
}

impl<'g, 'a> Search<'g, 'a> {
    /// Search in the graph's own vertex order — the order vertices were created in: instances in
    /// netlist order (each cell's pins in library order), then the top ports.
    pub fn in_graph_order(graph: &'g Graph<'a>, sdc: &'g Sdc) -> Search<'g, 'a> {
        Search::new(graph, sdc, (0..graph.vertices.len()).collect())
    }

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
        let input_delay = port_delays(&sdc.input_delays);
        let output_delay = port_delays(&sdc.output_delays);
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
        let (mut internal_from, mut internal_from_break, mut internal_to, mut internal_to_break) = (vec![false; n], vec![false; n], vec![false; n], vec![false; n]);
        if !sdc.path_delays.is_empty() {
            // the graph's own name index (built once; the last vertex of a repeated name, as a
            // collect here would keep)
            let index = graph.index();
            let has_check = |v: usize| graph.in_edges[v].iter().any(|&e| matches!(graph.edges[e].kind, EdgeKind::Gate { set } if is_check_role(graph.arc_set(e, set).role)));
            for pd in &sdc.path_delays {
                // `recordPathDelayInternalFrom`: not `isExceptionStartpoint`.
                for &v in pd.from_pins.iter().filter_map(|p| index.get(p.as_str())) {
                    let top_input = graph.is_port(v) && graph.vertices[v].is_driver;
                    if !(top_input || is_reg_clk[v]) {
                        internal_from[v] = true;
                        internal_from_break[v] |= pd.break_path;
                    }
                }
                // `recordPathDelayInternalTo`: not `hasLibertyCheckTo` nor a top port.
                for &v in pd.to_pins.iter().filter_map(|p| index.get(p.as_str())) {
                    if !(has_check(v) || graph.is_port(v)) {
                        internal_to[v] = true;
                        internal_to_break[v] |= pd.break_path;
                    }
                }
            }
        }
        let sim = sim_edges(graph);
        let (latch_en, latch_data) = latch_enables(graph);
        let clk_network = clock_network(graph, sdc);
        let clock_leaf = graph.vertices.iter().map(|v| sdc.clocks.iter().any(|c| !c.source.is_empty() && v.name == c.source)).collect();
        let mut search = Search { graph, sdc, paths: vec![Vec::new(); n], vertex_id, input_delay, output_delay, is_reg_clk, level, internal_from, internal_from_break, internal_to, internal_to_break, sim, latch_en, latch_data, clk_network, clock_leaf, gated: HashMap::new() };
        search.gated = search.gated_clk_enables();
        search
    }

    /// `GatedClk::isGatedClkEnable` for every instance input pin (gated clock checks are on by
    /// default): not in the clock network, searched from; a combinational arc to an output in the
    /// clock network whose function gates another port's clock (`isClkGatingFunc`, ports in cell
    /// order) — that clock pin in the clock network; kept when the clock pin has a register clock
    /// pin downstream in the clock network (`hasDownstreamClkPin`).
    fn gated_clk_enables(&self) -> HashMap<usize, (usize, bool)> {
        let g = self.graph;
        let mut out = HashMap::new();
        if self.sdc.clocks.is_empty() {
            return out;
        }
        let index = g.pin_index();
        for (v, vx) in g.vertices.iter().enumerate() {
            let (Some(lib), Some(cell_name), Some(port), crate::netlist::Conn::Inst(i, _)) = (vx.lib, vx.cell.as_deref(), vx.port.as_deref(), &vx.conn) else { continue };
            let cell = &g.libs[lib].cells[cell_name];
            if vx.is_driver || self.clk_network[v] || !self.search_from(v) || cell.port(port).is_none_or(|p| p.direction != crate::liberty::Direction::Input) {
                continue;
            }
            let inst = &g.netlist.insts[*i].0;
            'edges: for &e in &g.out_edges[v] {
                let EdgeKind::Gate { set } = g.edges[e].kind else { continue };
                let gclk = g.edges[e].to;
                if g.arc_set(e, set).role != Role::Combinational || !self.search_to(gclk) || !self.search_thru(e) || !self.clk_network[gclk] {
                    continue;
                }
                let Some(f) = g.vertices[gclk].port.as_deref().and_then(|op| cell.port(op)).and_then(|p| p.function.as_deref()).and_then(|f| crate::func_expr::FuncExpr::parse(f).ok()) else { continue };
                let mut fports = Vec::new();
                func_ports(&f, &mut fports);
                for clk_port in cell.ports.iter().map(|p| p.name.as_str()).filter(|p| fports.iter().any(|f| f == p) && *p != port) {
                    let Some(one) = clk_gating_func(&f, port, clk_port) else { continue };
                    let Some(&clk_v) = index.get(&format!("{inst}/{clk_port}")) else { continue };
                    if self.clk_network[clk_v] {
                        if self.has_downstream_clk_pin(clk_v) {
                            out.insert(v, (clk_v, one));
                        }
                        break 'edges;
                    }
                }
            }
        }
        out
    }

    /// `Vertex::hasDownstreamClkPin`: a register clock pin reachable in the clock network.
    fn has_downstream_clk_pin(&self, v: usize) -> bool {
        let mut seen = vec![false; self.graph.vertices.len()];
        let mut stack = vec![v];
        while let Some(u) = stack.pop() {
            if self.is_reg_clk[u] {
                return true;
            }
            for &e in &self.graph.out_edges[u] {
                let to = self.graph.edges[e].to;
                if self.clk_network[to] && !seen[to] && !self.is_check(e) {
                    seen[to] = true;
                    stack.push(to);
                }
            }
        }
        false
    }

    /// Whether `v` is a gated clock enable pin (an end: `Search::isEndpoint`).
    pub fn is_gated_clk_enable(&self, v: usize) -> bool {
        self.gated.contains_key(&v)
    }

    /// `VisitPathEnds::visitGatedClkEnd` → `PathEndGatedClock` (no `set_clock_gating_check`:
    /// margin 0): a clocked data path at a gated clock enable, against the gate's clock pin clock
    /// paths of the target min/max and the active value's transition (`gatedClkActiveTrans`: its
    /// leading edge for max, the trailing for min); required by the gated clock setup (setup's
    /// cycle) or hold (same cycle as the setup) accounting, + the target's network delay, ± CRPR.
    fn gated_clk_ends(&self, i: usize, path: &Path, src_edge: usize, clk_v: usize, one: bool, req: &mut [f32]) {
        let mm = path.tag.mm;
        let leading = if one { crate::liberty::RISE } else { crate::liberty::FALL };
        let clk_rf = if mm == MAX { leading } else { 1 - leading };
        for tgt in &self.paths[clk_v] {
            let Some(tgt_edge) = tgt.tag.clk_edge else { continue };
            if tgt.tag.mm != 1 - mm || tgt.tag.rf != clk_rf || !tgt.tag.is_clock {
                continue;
            }
            let acct = self.accting(src_edge, tgt_edge);
            let latency = self.tgt_clk_delay(tgt);
            let crpr = self.check_crpr(path, clk_v, tgt);
            // `clockGatingMargin`: no `set_clock_gating_check`, 0.
            let margin = 0.0f32;
            if mm == MAX {
                let tgt_clk_arrival = (0.0 + latency) + acct.required[ACCT_SETUP];
                Self::required_set(req, i, (tgt_clk_arrival - margin) + crpr, MAX);
            } else {
                let tgt_clk_arrival = (0.0 + latency) + acct.required[crate::sdc::ACCT_GCLK_HOLD];
                Self::required_set(req, i, (tgt_clk_arrival + margin) + (0.0 - crpr), MIN);
            }
        }
    }

    /// `SearchThru::searchTo`: fanin paths are broken at a path delay's internal `-from` pin, and
    /// a constant is never entered (`SearchPred0::searchTo`).
    /// `EvalPred::searchTo`: not a constant, not a clock's source pin (its arrival is the clock's
    /// own) unless a path delay ends there, not a breaking path delay `-from` pin.
    fn search_to(&self, v: usize) -> bool {
        !self.internal_from_break[v] && !self.graph.is_constant(v) && !(self.clock_leaf[v] && !self.internal_to[v])
    }

    /// `SearchPred0::searchFrom`: nothing leaves a constant.
    fn search_from(&self, v: usize) -> bool {
        !self.graph.is_constant(v)
    }

    /// `hasFanin(vertex, search_thru_)`: searched to, over a non-check edge.
    fn has_fanin(&self, v: usize) -> bool {
        self.search_to(v) && self.graph.in_edges[v].iter().any(|&e| self.search_thru(e) && self.search_from(self.graph.edges[e].from))
    }

    /// `SearchPred0::searchThru`: not a check, not disabled by a `when` under the constants, not of
    /// sense none, and not a register's preset/clear arc — those are disabled by default
    /// (`preset_clr_arcs_enabled` false, which no job here changes), so no path runs from an
    /// asynchronous reset through to the register's output.
    fn search_thru(&self, e: usize) -> bool {
        !self.is_check(e) && !self.sim.disabled_cond[e] && self.sim.sense[e] != SimSense::None && self.role(e) != Some(Role::RegSetClr)
    }

    /// `searchThruTimingSense`: the arc's transitions against the edge's sense under constants.
    fn search_thru_sense(&self, e: usize, from_rf: usize, to_rf: usize) -> bool {
        match self.sim.sense[e] {
            SimSense::Positive => from_rf == to_rf,
            SimSense::Negative => from_rf != to_rf,
            SimSense::None => false,
            SimSense::Unknown | SimSense::NonUnate => true,
        }
    }

    /// `SearchPred0::searchFrom` / `searchTo` as the resizer's cone search asks them: not a
    /// constant (no `set_disable_timing` pin is modelled). Unlike the arrival search's `searchTo`,
    /// no clock source or path delay pin is excluded.
    pub fn pred0_from(&self, v: usize) -> bool {
        !self.graph.is_constant(v)
    }

    pub fn pred0_to(&self, v: usize) -> bool {
        !self.graph.is_constant(v)
    }

    /// `SearchPredCombLogic::searchThru` (the resizer's fan-in / fan-out cones): `SearchPred1` —
    /// `SearchPred0::searchThru` (not a check, not disabled under the constants, not of sense none,
    /// not a preset/clear arc, not a bidirect instance path while those are off by default) and no
    /// disabled loop (a levelized graph has none) — and not a register clock-to-Q or latch D-to-Q
    /// edge, so a cone stops at sequential cells.
    pub fn comb_logic_thru(&self, e: usize) -> bool {
        self.search_thru(e)
            && !self.graph.bidirect_inst_paths.contains(&e)
            && !matches!(self.role(e), Some(Role::RegClkToQ) | Some(Role::LatchDtoQ))
    }

    /// Whether a gate edge into `v` is a register clock-to-Q arc (the resizer's `isRegOutput`).
    pub fn is_reg_output(&self, v: usize) -> bool {
        self.graph.in_edges[v].iter().any(|&e| self.role(e) == Some(Role::RegClkToQ))
    }

    /// `hasFanout(vertex, search_thru_)`: a non-check edge to a vertex searched to.
    pub fn has_fanout(&self, v: usize) -> bool {
        self.search_from(v) && self.graph.out_edges[v].iter().any(|&e| self.search_thru(e) && self.search_to(self.graph.edges[e].to))
    }

    /// `Search::isEndpoint`: with fanin, and a timing check, or a constrained end (a top output, an
    /// output delay), or no fanout, or a path delay's internal `-to` pin.
    pub fn is_endpoint(&self, v: usize) -> bool {
        let vx = &self.graph.vertices[v];
        let constrained = self.graph.is_port(v) && (!vx.is_driver || self.output_delay.contains_key(&vx.name));
        let checks = self.graph.in_edges[v].iter().any(|&e| self.is_check(e));
        self.has_fanin(v) && (checks || constrained || !self.has_fanout(v) || self.internal_to[v] || self.gated.contains_key(&v))
    }

    /// Whether every searched fanout edge of `v` is a latch D -> Q edge — across which no required
    /// time propagates (`RequiredVisitor::visitFromToPath`), so an end there takes its slack from
    /// its own checks alone.
    pub fn fanout_only_latch_d_to_q(&self, v: usize) -> bool {
        self.graph.out_edges[v].iter().filter(|&&e| self.search_thru(e) && self.search_to(self.graph.edges[e].to)).all(|&e| self.role(e) == Some(Role::LatchDtoQ))
    }

    /// `LibertyPort::isLatchOutput`: the Q of a latch D -> Q edge with an enable.
    pub fn is_latch_output(&self, v: usize) -> bool {
        self.latch_en.keys().any(|&e| self.graph.edges[e].to == v)
    }

    /// The latch outputs a latch data pin drives through its D -> Q edges.
    pub fn latch_outputs_of(&self, v: usize) -> Vec<usize> {
        let mut out: Vec<usize> = self.latch_en.keys().filter(|&&e| self.graph.edges[e].from == v).map(|&e| self.graph.edges[e].to).collect();
        out.sort_unstable();
        out.dedup();
        out
    }

    /// Whether `v` is a path delay's internal `-to` pin whose fanout the search breaks.
    pub fn is_path_delay_internal_to_break(&self, v: usize) -> bool {
        self.internal_to_break[v]
    }

    /// `ClkNetwork::isClock`: the vertex is on a clock's network.
    pub fn is_clock(&self, v: usize) -> bool {
        self.clk_network[v]
    }

    /// `Sta::vertexWorstSlackPath(vertex, min_max)`: of the vertex's paths of that min/max, in tag
    /// order, the first with the fuzzily least slack — strictly less than `INF`, so an
    /// unconstrained vertex has none.
    pub fn worst_slack_path(&self, v: usize, mm: usize) -> Option<&Path> {
        let mut worst = None;
        let mut min_slack = INF;
        for p in self.paths[v].iter().filter(|p| p.tag.mm == mm) {
            let s = if mm == MAX { p.required - p.arrival } else { p.arrival - p.required };
            if crate::fuzzy::less(s, min_slack) {
                min_slack = s;
                worst = Some(p);
            }
        }
        worst
    }

    /// `Sta::vertexWorstArrivalPath(vertex, min_max)`: the first path of that min/max with the
    /// fuzzily latest arrival (earliest, for min), from the min/max's initial value.
    pub fn worst_arrival_path(&self, v: usize, mm: usize) -> Option<&Path> {
        let mut worst = None;
        let mut worst_arrival = if mm == MAX { -INF } else { INF };
        for p in self.paths[v].iter().filter(|p| p.tag.mm == mm) {
            let later = if mm == MAX { crate::fuzzy::greater(p.arrival, worst_arrival) } else { crate::fuzzy::less(p.arrival, worst_arrival) };
            if later {
                worst_arrival = p.arrival;
                worst = Some(p);
            }
        }
        worst
    }

    /// `Sta::arrival(vertex, rf, scenes, min_max)` for one transition: the latest arrival of its
    /// paths (the earliest, for min); the min/max's initial value (±`INF`) with none.
    pub fn arrival_rf(&self, v: usize, rf: usize, mm: usize) -> f32 {
        let mut arrival = if mm == MAX { -INF } else { INF };
        for p in self.paths[v].iter().filter(|p| p.tag.mm == mm && p.tag.rf == rf) {
            let later = if mm == MAX { crate::fuzzy::greater(p.arrival, arrival) } else { crate::fuzzy::less(p.arrival, arrival) };
            if later {
                arrival = p.arrival;
            }
        }
        arrival
    }

    /// `Sta::required(vertex, rf, scenes, min_max)` for one transition: the earliest required of
    /// its max paths (the latest, for min) — the OPPOSITE min/max's order; its initial value
    /// (±`INF`) with none.
    pub fn required_rf(&self, v: usize, rf: usize, mm: usize) -> f32 {
        let mut required = if mm == MAX { INF } else { -INF };
        for p in self.paths[v].iter().filter(|p| p.tag.mm == mm && p.tag.rf == rf) {
            let first = if mm == MAX { crate::fuzzy::less(p.required, required) } else { crate::fuzzy::greater(p.required, required) };
            if first {
                required = p.required;
            }
        }
        required
    }

    /// `PathExpanded::size()` (generated clock sources not expanded): the path and every path
    /// before it, back to the first — through the launching clock's own path. Before the start is
    /// found (a register clock-to-Q or latch enable-to-Q arc), a latch D-to-Q arc ends it after
    /// the latch's D path.
    pub fn expanded_size(&self, p: &Path) -> usize {
        let mut n = 0;
        let mut found_start = false;
        let mut cur = Some(p);
        while let Some(q) = cur {
            let prev = self.prev_path(q);
            if let (false, Some(_), Some(pv)) = (found_start, prev, q.prev) {
                match self.role(pv.edge) {
                    Some(Role::RegClkToQ) | Some(Role::LatchEnToQ) => found_start = true,
                    Some(Role::LatchDtoQ) => return n + 2,
                    _ => {}
                }
            }
            n += 1;
            cur = prev.map(|(_, q)| q);
        }
        n
    }

    /// Whether a CLOCK arrival reaches `v` (`Search::clocks(vertex)` is not empty): a data path's
    /// tag carries its launching clock too, and does not count.
    pub fn has_clock_arrival(&self, v: usize) -> bool {
        self.paths[v].iter().any(|p| p.tag.is_clock)
    }

    /// `Power::findClk`: the fastest clock among the vertex's arrivals — strictly faster, so of
    /// equal periods the first path's clock.
    pub fn find_clk(&self, v: usize) -> Option<&crate::sdc::Clock> {
        let mut clk: Option<&crate::sdc::Clock> = None;
        for p in &self.paths[v] {
            if let Some(e) = p.tag.clk_edge {
                let c = self.edge_clock(e);
                if clk.is_none_or(|k| c.period < k.period) {
                    clk = Some(c);
                }
            }
        }
        clk
    }

    /// The clock of clock edge `e` (`ClockEdge::index`: clock × 2 + transition).
    pub fn edge_clock(&self, e: usize) -> &crate::sdc::Clock {
        &self.sdc.clocks[e / 2]
    }

    /// Whether clock edge `e`'s clock is propagated (`ClkInfo::isPropagated`).
    pub fn propagated(&self, e: usize) -> bool {
        self.sdc.clocks.get(e / 2).is_some_and(|c| c.propagated)
    }

    /// The time of clock edge `e`.
    pub fn edge_time(&self, e: usize) -> f32 {
        self.sdc.clocks.get(e / 2).map_or(0.0, |c| c.edge_time(e % 2))
    }

    /// `CycleAccting` from source clock edge `src` to target clock edge `tgt`.
    fn accting(&self, src: usize, tgt: usize) -> crate::sdc::Accting {
        crate::sdc::cycle_accting(self.edge_clock(src), src % 2, self.edge_clock(tgt), tgt % 2)
    }

    pub(crate) fn role(&self, e: usize) -> Option<Role> {
        match self.graph.edges[e].kind {
            EdgeKind::Gate { set } => Some(self.graph.arc_set(e, set).role),
            EdgeKind::Wire => None,
        }
    }

    pub(crate) fn is_check(&self, e: usize) -> bool {
        self.role(e).is_some_and(is_check_role)
    }

    /// The (at most two) arcs from `from_rf`, in arc order.
    pub(crate) fn arcs_from(&self, e: usize, from_rf: usize) -> Vec<ArcRef> {
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
    pub(crate) fn visit_from_path(&self, from_v: usize, from: &Path, e: usize, arc: &ArcRef) -> Option<(Tag, f32, f32)> {
        let to_v = self.graph.edges[e].to;
        if !self.search_from(from_v) || !self.search_to(to_v) || !self.search_thru(e) || !self.search_thru_sense(e, from.tag.rf, arc.to_rf) {
            return None;
        }
        let mm = from.tag.mm;
        let delay = self.arc_delay(e, arc.index, mm);
        let role = self.role(e);
        match role {
            // A latch's EN -> Q launches as a register's CLK -> Q (`genericRole() == regClkToQ`).
            Some(Role::RegClkToQ | Role::LatchEnToQ) => {
                // Only clocked clock paths launch, keeping the clock path's CRPR
                // path — or taking this clock path when it has none. ⛔ `ClkInfo` keeps a CRPR
                // path only for a PROPAGATED clock (`crpr_clk_path_(is_propagated ? … :
                // nullptr)`): under an ideal clock every register's launch shares one tag.
                // `(clk_edge == nullptr && isSegmentStart())`: an unclocked register clock pin's
                // seed launches too, unclocked, at its arrival (`clkPathArrival` of a non-clock path).
                if from.tag.clk_edge.is_none() && from.tag.segment_start {
                    let states = self.sdc.exception_from_states(&self.graph.vertices[from_v].name, false, mm);
                    let states = self.mutate_states(states, from_v, to_v, mm);
                    let tag = Tag { rf: arc.to_rf, mm, clk_edge: None, is_clock: false, crpr: None, states, latency: 0, segment_start: false };
                    return Some((tag, delay, from.arrival + delay));
                }
                if !from.tag.is_clock || from.tag.clk_edge.is_none() {
                    return None;
                }
                let crpr = from.tag.crpr.or(Some(CrprPath { vertex: from_v, rf: from.tag.rf, mm, clk_edge: from.tag.clk_edge.unwrap() })).filter(|c| self.propagated(c.clk_edge));
                // `fromRegClkTag`: the states of the path delays from the clock pin (or the
                // clock), then `thruTag` over the clock-to-Q edge.
                let states = self.sdc.exception_from_states(&self.graph.vertices[from_v].name, true, mm);
                let states = self.mutate_states(states, from_v, to_v, mm);
                let tag = Tag { rf: arc.to_rf, mm, clk_edge: from.tag.clk_edge, is_clock: false, crpr, states, latency: from.tag.latency, segment_start: false };
                // `clkPathArrival`: an IDEAL clock launches at its edge + its latency (no insertion
                // set), whatever the clock network's delays.
                let launch = if self.propagated(from.tag.clk_edge.unwrap()) { from.arrival } else { self.ideal_clk_arrival(&from.tag) };
                Some((tag, delay, launch + delay))
            }
            // `Latches::latchOutArrival`: max data paths only, clocked.
            Some(Role::LatchDtoQ) => {
                if mm != MAX || from.tag.clk_edge.is_none() {
                    return None;
                }
                self.latch_out_arrival(from_v, from, e, arc)
            }
            _ if from.tag.is_clock => {
                // A clock path through a clock-network arc.
                let to_is_clk = matches!(role, None | Some(Role::Combinational));
                let opp = self.arc_delay(e, arc.index, 1 - mm);
                let min_max_eq = crate::fuzzy::equal(delay, opp);
                let from_is_reg_clk = self.is_reg_clk[from_v];
                // `thruClkInfo`: the CRPR path set here is dropped again by `ClkInfo` for a clock
                // that is not propagated.
                let crpr = if (!to_is_clk && !from_is_reg_clk) || (self.is_reg_clk[to_v] && min_max_eq) {
                    Some(CrprPath { vertex: from_v, rf: from.tag.rf, mm, clk_edge: from.tag.clk_edge.unwrap() })
                } else {
                    from.tag.crpr
                }
                .filter(|c| self.propagated(c.clk_edge));
                let states = self.mutate_states(from.tag.states, from_v, to_v, mm);
                // `thruClkInfo`: a latency on the pin (for the clock EDGE's transition) replaces
                // the one carried (pin before hierarchical pin; no hierarchy here).
                let edge = from.tag.clk_edge.unwrap();
                let latency = self.edge_clock(edge).pin_latency(&self.graph.vertices[to_v].name, edge % 2, mm).map_or(from.tag.latency, f32::to_bits);
                let tag = Tag { rf: arc.to_rf, mm, clk_edge: from.tag.clk_edge, is_clock: to_is_clk, crpr, states, latency, segment_start: false };
                Some((tag, delay, from.arrival + delay))
            }
            // `visitFromPath`: a data arc out of an internal `-to` pin is broken (and into an
            // internal `-from` pin, above).
            _ if self.internal_to_break[from_v] => None,
            _ => {
                // A data path keeps its tag, its states mutated.
                let states = self.mutate_states(from.tag.states, from_v, to_v, mm);
                let tag = Tag { rf: arc.to_rf, states, ..from.tag };
                Some((tag, delay, from.arrival + delay))
            }
        }
    }

    /// `Search::mutateTag`'s states over an edge `from_v` → `to_v` (no `-through`, no false paths):
    /// unchanged unless a path delay completes at `to_v`; then, every state but those complete at
    /// `from_v` — so a path delay's tag is dropped on the edge LEAVING its `-to` pin only when
    /// that edge's own `to` completes one too (the reference's two passes test different pins).
    fn mutate_states(&self, states: States, from_v: usize, to_v: usize, mm: usize) -> States {
        if states == States::NONE {
            return states;
        }
        let to_pin = &self.graph.vertices[to_v].name;
        if !states.ids().any(|id| self.sdc.is_complete_to_pin(id, to_pin, mm)) {
            return states;
        }
        let from_pin = &self.graph.vertices[from_v].name;
        States(states.ids().filter(|&id| !self.sdc.is_complete_to_pin(id, from_pin, mm)).fold(0, |m, id| m | 1 << id))
    }

    /// Whether every path delay is of the form this search models, else what is not: `-from` pins
    /// or the clock, `-to` pins or the clock, never an output delay; a max delay without
    /// `-ignore_clock_latency` only with no clock; an internal `-from` to a checked pin only with no
    /// clock; at most 64.
    pub fn constraints_modelled(&self) -> Result<(), String> {
        if let Some(why) = &self.sim.error {
            return Err(format!("{why}: not modelled"));
        }
        if self.sdc.path_delays.len() > 64 {
            return Err(format!("{} path delays: not modelled (64 are)", self.sdc.path_delays.len()));
        }
        let find = |pin: &str| self.graph.vertices.iter().position(|x| x.name == pin);
        for pd in &self.sdc.path_delays {
            let cmd = if pd.min_max == MAX { "set_max_delay" } else { "set_min_delay" };
            // `-from` / `-to` "the clock" names no clock among several.
            if (pd.from_clock || pd.to_clock) && self.sdc.clocks.len() > 1 {
                return Err(format!("{cmd} -from / -to a clock with several clocks: not modelled"));
            }
            // A max delay measured from the clock edge is witnessed only with no clock.
            if pd.min_max == MAX && !pd.ignore_clk_latency && !self.sdc.clocks.is_empty() {
                return Err("set_max_delay without -ignore_clock_latency on a clocked design: not modelled".into());
            }
            if (pd.from_pins.is_empty() && !pd.from_clock) || (pd.to_pins.is_empty() && !pd.to_clock) || (pd.from_clock && !pd.from_pins.is_empty()) || (pd.to_clock && !pd.to_pins.is_empty()) {
                return Err(format!("{cmd} without -from / -to, or -from / -to both pins and the clock: not modelled"));
            }
            // A path delay ending at an output delay (`visitOutputDelayEnd1`) is not modelled.
            if (pd.to_clock && !self.sdc.output_delays.is_empty()) || pd.to_pins.iter().any(|p| self.output_delay.contains_key(p)) {
                return Err(format!("{cmd} -to an output delay: not modelled"));
            }
            let mut internal_from = false;
            for pin in &pd.from_pins {
                let Some(v) = find(pin) else { return Err(format!("{cmd} -from {pin}: no such pin")) };
                internal_from |= self.internal_from[v];
            }
            for pin in &pd.to_pins {
                let Some(v) = find(pin) else { return Err(format!("{cmd} -to {pin}: no such pin")) };
                // An unclocked path into a clocked check would read the target clock's latency.
                let check = self.graph.in_edges[v].iter().any(|&e| self.is_check(e));
                if internal_from && check && !self.sdc.clocks.is_empty() {
                    return Err(format!("{cmd} -from an internal pin -to the checked pin {pin} on a clocked design: not modelled"));
                }
            }
        }
        Ok(())
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
        self.paths[v] = self.arrival_paths(v);
    }

    /// The paths a visit of `v` builds from its fanin's paths as they stand (`ArrivalVisitor`:
    /// fanin paths, CRPR pruning, seeds), in tag order — not stored.
    pub fn arrival_paths(&self, v: usize) -> Vec<Path> {
        let mut bldr = Bldr::default();
        let mut no_crpr = NoCrprBldr::default();
        let has_fanin_one = self.graph.in_edges[v].len() == 1;
        self.visit_fanin_paths(v, &mut bldr, &mut no_crpr, has_fanin_one);
        // `tag_bldr_->hasPropagatedClk()`.
        if !has_fanin_one && bldr.paths.iter().any(|p| p.tag.clk_edge.is_some_and(|e| self.propagated(e))) {
            self.prune_crpr_arrivals(&mut bldr, &no_crpr);
        }
        // `seedArrivals`: a register clock pin with no clock path (`!hasClkTag()`, read before any
        // seed) — `makeUnclkedPaths(vertex, is_segment_start = true)`: per min/max and transition an
        // unclocked segment start at 0, with the path delays from the pin.
        let has_clk_tag = bldr.paths.iter().any(|p| p.tag.is_clock);
        self.seed_arrivals(v, &mut bldr);
        if self.is_reg_clk[v] && !has_clk_tag {
            for mm in [MIN, MAX] {
                let states = self.sdc.exception_from_states(&self.graph.vertices[v].name, false, mm);
                for rf in [0, 1] {
                    set_arrival(&mut bldr, Tag { rf, mm, clk_edge: None, is_clock: false, crpr: None, states, latency: 0, segment_start: true }, 0.0);
                }
            }
        }
        if self.internal_from[v] {
            // `makeUnclkedPaths(vertex, false, require_exception = true)`: per min/max and
            // transition, an unclocked path at 0 carrying the delays from the pin — none without.
            for mm in [MIN, MAX] {
                let states = self.sdc.exception_from_states(&self.graph.vertices[v].name, false, mm);
                if states == States::NONE {
                    continue;
                }
                for rf in [0, 1] {
                    set_arrival(&mut bldr, Tag { rf, mm, clk_edge: None, is_clock: false, crpr: None, states, latency: 0, segment_start: false }, 0.0);
                }
            }
        }
        let vid = &self.vertex_id;
        bldr.paths.sort_by_key(|p| p.tag.key(vid));
        bldr.paths
    }

    /// In-edges newest first (edges are prepended), each fanin path in tag order, each arc from its
    /// transition.
    fn visit_fanin_paths(&self, v: usize, bldr: &mut Bldr, no_crpr: &mut NoCrprBldr, has_fanin_one: bool) {
        for &e in self.graph.in_edges[v].iter().rev() {
            if self.is_check(e) {
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

    /// The clock's source port, input ports with a delay, and — `seedInputArrival` with no
    /// `set_input_delay` and the default arrival clock off (`use_default_arrival_clock_` is false
    /// and nothing sets it) — every other input port, UNCLOCKED at 0.
    fn seed_arrivals(&self, v: usize, bldr: &mut Bldr) {
        let name = &self.graph.vertices[v].name;
        // `isLeafPinClock`: a clock seeds on its source pin wherever it is — a port or an
        // instance pin; the input arrivals below seed on top ports only.
        let sources: Vec<usize> = self.sdc.clocks.iter().enumerate().filter(|(_, c)| !c.source.is_empty() && *name == c.source).map(|(k, _)| k).collect();
        if !sources.is_empty() {
            // `seedClkArrivals`: each clock on the pin; per min/max and transition, the edge of
            // that transition at `insertion + edge time`.
            for &c in &sources {
                for mm in [MIN, MAX] {
                    // `seedClkArrival` → `exceptionFromClkStates`: the path delays from the clock's
                    // pin or from the clock ride on its clock tags too.
                    let states = self.sdc.exception_from_states(name, true, mm);
                    for rf in [0, 1] {
                        let edge = 2 * c + rf;
                        // a latency on the clock's own pin (no clock-wide latency here)
                        let latency = self.sdc.clocks[c].pin_latency(name, rf, mm).map_or(0, f32::to_bits);
                        let tag = Tag { rf, mm, clk_edge: Some(edge), is_clock: true, crpr: None, states, latency, segment_start: false };
                        set_arrival(bldr, tag, 0.0 + self.edge_time(edge));
                    }
                }
            }
        }
        if !self.graph.is_port(v) {
            return;
        }
        let delays: Vec<usize> = self.input_delay.get(name).map(|l| l.iter().copied().filter(|&d| self.sdc.input_delays[d].clock < self.sdc.clocks.len()).collect()).unwrap_or_default();
        if !delays.is_empty() {
            // `seedInputArrival1`: an arrival per input delay, one per clock — but none for a
            // delay relative to a clock defined on the pin itself (its arrival there is the
            // clock's own). Each at its clock's rise edge + the delay.
            for d in delays {
                let clock = self.sdc.input_delays[d].clock;
                if sources.contains(&clock) {
                    continue;
                }
                let edge = 2 * clock;
                for mm in [MIN, MAX] {
                    let clk_arrival = self.edge_time(edge);
                    // `inputDelayTag` → `exceptionFromStates(pin, clk)`: from the port, or its clock.
                    let states = self.sdc.exception_from_states(name, true, mm);
                    for rf in [0, 1] {
                        let tag = Tag { rf, mm, clk_edge: Some(edge), is_clock: false, crpr: None, states, latency: 0, segment_start: false };
                        set_arrival(bldr, tag, clk_arrival + self.sdc.input_delays[d].delay[rf][mm]);
                    }
                }
            }
        } else if sources.is_empty() && self.graph.vertices[v].is_driver {
            // `seedInputDelayArrival(input_delay = null)` → `inputDelayTag` with no clock edge:
            // the states of the path delays from the port alone.
            for mm in [MIN, MAX] {
                let states = self.sdc.exception_from_states(name, false, mm);
                for rf in [0, 1] {
                    let tag = Tag { rf, mm, clk_edge: None, is_clock: false, crpr: None, states, latency: 0, segment_start: false };
                    set_arrival(bldr, tag, 0.0);
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
        let req = self.required_values(v);
        for (p, r) in self.paths[v].iter_mut().zip(req) {
            p.required = r;
        }
    }

    /// The requireds a visit of `v` computes for its paths (`RequiredVisitor`: from the fanout's
    /// requireds as they stand, then the endpoint's path ends) — not stored.
    pub fn required_values(&self, v: usize) -> Vec<f32> {
        let mut req: Vec<f32> = self.paths[v].iter().map(|p| if p.tag.mm == MAX { INF } else { -INF }).collect();
        self.visit_fanout_paths(v, &mut req);
        self.visit_path_ends(v, &mut req);
        req
    }

    /// Whether `v` drives a register's clock-to-output arc (`isRegClk`).
    pub fn is_reg_clk(&self, v: usize) -> bool {
        self.is_reg_clk[v]
    }

    /// `RequiredCmp::requiredSet` with the path's opposite min/max: a MAX path's required is the
    /// fuzzily smaller, a MIN path's the fuzzily greater.
    fn required_set(req: &mut [f32], i: usize, value: f32, mm: usize) {
        let better = if mm == MAX { crate::fuzzy::less(value, req[i]) } else { crate::fuzzy::greater(value, req[i]) };
        if better {
            req[i] = value;
        }
    }

    /// Requireds back from the fanout (max and min paths).
    fn visit_fanout_paths(&self, v: usize, req: &mut [f32]) {
        for &e in self.graph.out_edges[v].iter().rev() {
            if self.is_check(e) || self.role(e) == Some(Role::LatchDtoQ) {
                continue;
            }
            let to_v = self.graph.edges[e].to;
            for (i, from) in self.paths[v].iter().enumerate() {
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
                        Self::required_set(req, i, r - delay, from.tag.mm);
                    }
                }
            }
        }
    }

    /// The endpoint's path ends: an output delay end if the port has one, else a check end —
    /// setup for a max path, hold for a min path.
    fn visit_path_ends(&self, v: usize, req: &mut [f32]) {
        let name = &self.graph.vertices[v].name;
        let ods: &[usize] = if self.graph.is_port(v) { self.output_delay.get(name).map_or(&[], Vec::as_slice) } else { &[] };
        let checks = self.graph.in_edges[v].iter().any(|&e| self.is_check(e));
        let gated = self.gated.get(&v).copied();
        for (i, path) in self.paths[v].iter().enumerate() {
            // `visitGatedClkEnd`, besides any other end.
            if let (Some((clk_v, one)), Some(src_edge)) = (gated, path.tag.clk_edge) {
                if !path.tag.is_clock {
                    self.gated_clk_ends(i, path, src_edge, clk_v, one, req);
                }
            }
            // `visitClkedPathEnds`: no output delay and no check — a path delay `-to` the pin
            // (`pathDelayTo`, no target clock), whatever the path's clock.
            if ods.is_empty() && !checks {
                if let Some(id) = self.sdc.path_delay_to(path.tag.states, name, false, path.tag.mm) {
                    Self::required_set(req, i, self.path_delay_required(path, id, 0.0, 0.0, 0.0), path.tag.mm);
                }
                continue;
            }
            let Some(src_edge) = path.tag.clk_edge else {
                // An unclocked path at a check (`visitCheckEndUnclked`): a path delay `-to` the
                // pin with the check's arc as margin and no target clock path.
                if ods.is_empty() {
                    self.unclocked_check_ends(v, i, path, req);
                }
                continue;
            };
            if path.tag.mm == MIN {
                self.hold_path_ends(v, ods, i, path, src_edge, req);
                continue;
            }
            if !ods.is_empty() {
                // `visitOutputDelayEnd`: an end per output delay (one per clock). Target clock
                // time (the delay's rise edge) − the delay; none when the delay sets no max
                // value for this transition.
                for &d in ods {
                    if self.sdc.output_delays[d].exists[path.tag.rf][MAX] {
                        let tgt_time = self.accting(src_edge, 2 * self.sdc.output_delays[d].clock).required[ACCT_SETUP];
                        let margin = self.sdc.output_delays[d].delay[path.tag.rf][MAX];
                        Self::required_set(req, i, (tgt_time + (0.0 + 0.0)) - margin, MAX);
                    }
                }
                continue;
            }
            // Setup and recovery checks alike: the reference visits every check arc
            // (`visitCheckEnd`), and recovery takes setup's cycle accounting (`setSetupAccting`).
            // An asynchronous reset's release is checked here, as a data pin is.
            for &e in self.graph.in_edges[v].iter().rev() {
                if !matches!(self.role(e), Some(Role::Setup | Role::Recovery)) {
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
                        // `visitCheckEnd`: a path delay completing here overrides the check —
                        // `PathEndPathDelay` with the check's arc as its margin.
                        let tgt_edge = tgt.tag.clk_edge.expect("a clock path has an edge");
                        // `PathEndLatchCheck`: at a latch's data pin, the check's target is the
                        // DISABLE path; the required is `latchRequired`'s, the check arc the margin.
                        if self.latch_data[v] && self.sdc.path_delay_to(path.tag.states, name, true, MAX).is_none() {
                            let enable = self.latch_other_path(tgt_v, tgt);
                            let (required, _, _) = self.latch_required(path, tgt_v, enable, Some(tgt), self.arc_delay(e, k, MAX));
                            Self::required_set(req, i, required, MAX);
                            continue;
                        }
                        if let Some(id) = self.sdc.path_delay_to(path.tag.states, name, true, MAX) {
                            let latency = self.tgt_clk_delay(tgt);
                            let required = self.path_delay_required(path, id, self.arc_delay(e, k, MAX), latency, self.check_crpr(path, tgt_v, tgt));
                            Self::required_set(req, i, required, MAX);
                            continue;
                        }
                        // The check's required time.
                        // An ideal clock's target has no network latency (`targetClkDelay`).
                        let latency = self.tgt_clk_delay(tgt);
                        let tgt_clk_arrival = (0.0 + latency) + self.accting(src_edge, tgt_edge).required[ACCT_SETUP];
                        let margin = self.arc_delay(e, k, MAX);
                        let crpr = self.check_crpr(path, tgt_v, tgt);
                        Self::required_set(req, i, (tgt_clk_arrival - (margin + 0.0)) + crpr, MAX);
                    }
                }
            }
        }
    }

    /// `Latches::latchEnableOtherPath`: at the enable vertex, of the same min/max, the clock path of
    /// the opposite transition and the opposite clock edge.
    fn latch_other_path(&self, en_v: usize, p: &Path) -> Option<&Path> {
        let edge = p.tag.clk_edge?;
        self.paths[en_v].iter().find(|o| o.tag.is_clock && o.tag.mm == p.tag.mm && o.tag.rf == 1 - p.tag.rf && o.tag.clk_edge == Some(edge ^ 1))
    }

    /// `PathEnd::checkTgtClkDelay` with no insertion set: a propagated clock path's network delay
    /// (its arrival less its edge's time), else (ideal) the path's latency.
    fn tgt_clk_delay(&self, p: &Path) -> f32 {
        match p.tag.clk_edge {
            Some(e) if self.propagated(e) => (p.arrival - self.edge_time(e)) - 0.0,
            Some(_) => f32::from_bits(p.tag.latency),
            None => 0.0,
        }
    }

    /// `Search::clkPathArrival` for an ideal clock (no insertion set): `(0 + edge time) + latency`.
    fn ideal_clk_arrival(&self, tag: &Tag) -> f32 {
        (0.0 + self.edge_time(tag.clk_edge.expect("a clocked path"))) + f32::from_bits(tag.latency)
    }

    /// `Latches::latchRequired` with no exception (no multicycle, no path delay, no uncertainty, no
    /// borrow limit): `(required, borrow, adjusted data arrival)`. With the enable and disable clock
    /// paths: the enable's arrival = the latch-setup cycle's required + its network delay + the
    /// open CRPR; data at or before it needs nothing borrowed; else it borrows, up to the enable's
    /// pulse width less (the latency and CRPR differences + the margin) — the required then the
    /// data's arrival, or the enable plus the borrow limit; the data leaves shifted from the data
    /// clock's cycle to the enable's. With the disable path alone: its clock arrival − the margin.
    fn latch_required(&self, data: &Path, en_v: usize, enable: Option<&Path>, disable: Option<&Path>, margin: f32) -> (f32, f32, f32) {
        let data_arrival = data.arrival;
        match (enable, disable, data.tag.clk_edge) {
            (Some(en), Some(dis), Some(data_edge)) => {
                let en_edge = en.tag.clk_edge.expect("a clock path");
                // `latchBorrowInfo`.
                let nom_pulse_width = crate::sdc::pulse_width(self.edge_clock(en_edge), en_edge % 2);
                let open_crpr = self.check_crpr(data, en_v, en);
                let close_crpr = self.check_crpr(data, en_v, dis);
                let crpr_diff = open_crpr - close_crpr;
                let open_latency = self.tgt_clk_delay(en);
                let latency_diff = open_latency - self.tgt_clk_delay(dis);
                let max_borrow = nom_pulse_width - ((latency_diff + crpr_diff) + margin);
                let acct = self.accting(data_edge, en_edge);
                let tgt_clk_time = acct.required[ACCT_LATCH_SETUP];
                let enable_arrival = ((0.0 + tgt_clk_time + 0.0 + 0.0) + open_latency) + open_crpr;
                if crate::fuzzy::less_equal(data_arrival, enable_arrival) {
                    (enable_arrival, 0.0, data_arrival)
                } else {
                    let mut borrow = data_arrival - enable_arrival;
                    let required = if crate::fuzzy::less_equal(borrow, max_borrow) {
                        data_arrival
                    } else {
                        borrow = max_borrow;
                        enable_arrival + max_borrow
                    };
                    let shift = acct.shift(ACCT_LATCH_SETUP, self.edge_clock(data_edge), self.edge_clock(en_edge));
                    (required, borrow, required + shift)
                }
            }
            (None, Some(dis), ..) => {
                let disable_arrival = match dis.tag.clk_edge {
                    Some(e) if !self.propagated(e) => self.ideal_clk_arrival(&dis.tag),
                    _ => dis.arrival,
                };
                ((0.0 + disable_arrival) - margin, 0.0, data_arrival)
            }
            _ => (0.0, 0.0, data_arrival),
        }
    }

    /// `Latches::latchSetupMargin`: the setup check into the data pin from the enable whose arcs go
    /// to the data's transition from the disable's — its delay at the disable path's min/max.
    fn latch_setup_margin(&self, d_v: usize, data_rf: usize, en_v: usize, disable: Option<&Path>) -> f32 {
        let Some(dis) = disable else { return 0.0 };
        for &e in self.graph.in_edges[d_v].iter().rev() {
            if self.role(e) != Some(Role::Setup) || self.graph.edges[e].from != en_v || self.sim.disabled_cond[e] {
                continue;
            }
            let EdgeKind::Gate { set } = self.graph.edges[e].kind else { continue };
            for (k, arc) in self.graph.arc_set(e, set).arcs.iter().enumerate() {
                if arc.to_rf == data_rf && arc.from_rf == dis.tag.rf {
                    return self.arc_delay(e, k, dis.tag.mm);
                }
            }
        }
        0.0
    }

    /// `Latches::latchOutArrival` with the latch enabled: the first enable clock path (of the
    /// target clock min/max — MIN for a max path); a path delay to the D pin stops it; with data
    /// arriving while the latch is transparent (borrow > 0) it leaves at the adjusted arrival + the
    /// D -> Q arc's delay, under the ENABLE's clock edge (its CRPR clock path the enable path when
    /// propagated), its states from the D pin and from the enable's pin or clock — then `thruTag`.
    fn latch_out_arrival(&self, from_v: usize, from: &Path, e: usize, arc: &ArcRef) -> Option<(Tag, f32, f32)> {
        let &(en_v, en_rf) = self.latch_en.get(&e)?;
        let d_pin = &self.graph.vertices[from_v].name;
        let en = self.paths[en_v].iter().find(|p| p.tag.mm == MIN && p.tag.rf == en_rf && p.tag.is_clock)?;
        if self.sdc.path_delay_to(from.tag.states, d_pin, true, MAX).is_some() {
            return None;
        }
        let disable = self.latch_other_path(en_v, en);
        let margin = self.latch_setup_margin(from_v, from.tag.rf, en_v, disable);
        let (_, borrow, adjusted) = self.latch_required(from, en_v, Some(en), disable, margin);
        if !crate::fuzzy::greater(borrow, 0.0) {
            return None;
        }
        let delay = self.arc_delay(e, arc.index, MAX);
        let en_edge = en.tag.clk_edge.expect("a clock path");
        let crpr = self.propagated(en_edge).then_some(CrprPath { vertex: en_v, rf: en.tag.rf, mm: en.tag.mm, clk_edge: en_edge });
        let en_pin = &self.graph.vertices[en_v].name;
        let states = States(self.sdc.exception_from_states(d_pin, false, MAX).0 | self.sdc.exception_from_states(en_pin, true, MAX).0);
        let states = self.mutate_states(states, from_v, self.graph.edges[e].to, MAX);
        let tag = Tag { rf: arc.to_rf, mm: MAX, clk_edge: en.tag.clk_edge, is_clock: false, crpr, states, latency: en.tag.latency, segment_start: false };
        Some((tag, delay, adjusted + delay))
    }

    /// `visitCheckEndUnclked`: each enabled check of the path's min/max role, each arc to its
    /// transition — a path delay complete here makes a `PathEndPathDelay` with that arc's margin.
    fn unclocked_check_ends(&self, v: usize, i: usize, path: &Path, req: &mut [f32]) {
        let role = if path.tag.mm == MAX { Role::Setup } else { Role::Hold };
        for &e in self.graph.in_edges[v].iter().rev() {
            if self.role(e) != Some(role) {
                continue;
            }
            let EdgeKind::Gate { set } = self.graph.edges[e].kind else { continue };
            for (k, arc) in self.graph.arc_set(e, set).arcs.iter().enumerate() {
                if arc.to_rf != path.tag.rf {
                    continue;
                }
                if let Some(id) = self.sdc.path_delay_to(path.tag.states, &self.graph.vertices[v].name, false, path.tag.mm) {
                    Self::required_set(req, i, self.path_delay_required(path, id, self.arc_delay(e, k, path.tag.mm), 0.0, 0.0), path.tag.mm);
                }
            }
        }
    }

    /// `PathEndPathDelay::requiredTime` at a check, `latency` the target clock path's network
    /// delay (`targetClkDelay`), `crpr` the check's credit. Under `-ignore_clock_latency`: the
    /// source clock path's arrival + the delay; else the target clock arrival WITHOUT its edge
    /// time (`targetClkArrivalNoCrpr`: delay + uncertainty) + the check's CRPR (negated for
    /// hold, `checkCrpr`) + the delay − the source offset (−the source edge's time, which the
    /// arrival includes and the delay does not). Then less a max path's margin, plus a min's.
    fn path_delay_required(&self, path: &Path, id: usize, margin: f32, latency: f32, crpr: f32) -> f32 {
        let pd = &self.sdc.path_delays[id];
        let with_delay = if pd.ignore_clk_latency {
            self.path_clk_path_arrival(path) + pd.delay
        } else {
            let check_crpr = if path.tag.mm == MAX { crpr } else { 0.0 - crpr };
            let tgt_clk_arrival = (latency + 0.0) + check_crpr;
            let src_clk_offset = path.tag.clk_edge.map_or(0.0, |e| -self.edge_time(e));
            (tgt_clk_arrival + pd.delay) - src_clk_offset
        };
        if path.tag.mm == MAX {
            with_delay - margin
        } else {
            with_delay + margin
        }
    }

    /// `Search::pathClkPathArrival`: a propagated clock's source clock path arrival — back along
    /// the path to its first clock path, or to the path a clock-to-Q edge left (`clkPathArrival`:
    /// its arrival); else (ideal, or an input's clock) the clock edge's time + the latency.
    fn path_clk_path_arrival(&self, path: &Path) -> f32 {
        if path.tag.clk_edge.is_some_and(|e| self.propagated(e)) {
            let mut p = path;
            loop {
                if p.tag.is_clock {
                    return p.arrival;
                }
                let Some((_, prev)) = self.prev_path(p) else { break };
                if p.prev.is_some_and(|pr| matches!(self.role(pr.edge), Some(Role::RegClkToQ | Role::LatchEnToQ))) {
                    return prev.arrival;
                }
                p = prev;
            }
        }
        // `clk_edge->time() + clk_info->latency()`
        path.tag.clk_edge.map_or(0.0, |e| self.edge_time(e) + f32::from_bits(path.tag.latency))
    }

    /// A min path's ends (`PathEndOutputDelay` / `PathEndCheck` under the hold role): the
    /// output delay's hold end, `target + −min delay`; else each hold check, against the LATE
    /// (max) target clock path (`tgtClkEarlyLate`), `(target + margin) − crpr`. The target time is
    /// the hold cycle accounting's.
    fn hold_path_ends(&self, v: usize, ods: &[usize], i: usize, path: &Path, src_edge: usize, req: &mut [f32]) {
        if !ods.is_empty() {
            for &d in ods {
                if self.sdc.output_delays[d].exists[path.tag.rf][MIN] {
                    let tgt_time = self.accting(src_edge, 2 * self.sdc.output_delays[d].clock).required[ACCT_HOLD];
                    let margin = -self.sdc.output_delays[d].delay[path.tag.rf][MIN];
                    Self::required_set(req, i, (tgt_time + (0.0 + 0.0)) + margin, MIN);
                }
            }
            return;
        }
        // Hold and removal checks alike (removal takes hold's cycle accounting, `setHoldAccting`).
        for &e in self.graph.in_edges[v].iter().rev() {
            if !matches!(self.role(e), Some(Role::Hold | Role::Removal)) {
                continue;
            }
            let EdgeKind::Gate { set } = self.graph.edges[e].kind else { continue };
            let tgt_v = self.graph.edges[e].from;
            for (k, arc) in self.graph.arc_set(e, set).arcs.iter().enumerate() {
                if arc.to_rf != path.tag.rf {
                    continue;
                }
                for tgt in &self.paths[tgt_v] {
                    if tgt.tag.mm != MAX || tgt.tag.rf != arc.from_rf || !tgt.tag.is_clock {
                        continue;
                    }
                    let tgt_edge = tgt.tag.clk_edge.expect("a clock path has an edge");
                    let latency = self.tgt_clk_delay(tgt);
                    // `visitCheckEnd`: a path delay completing here overrides the check.
                    if let Some(id) = self.sdc.path_delay_to(path.tag.states, &self.graph.vertices[v].name, true, MIN) {
                        let required = self.path_delay_required(path, id, self.arc_delay(e, k, MIN), latency, self.check_crpr(path, tgt_v, tgt));
                        Self::required_set(req, i, required, MIN);
                        continue;
                    }
                    let tgt_clk_arrival = (0.0 + latency) + self.accting(src_edge, tgt_edge).required[ACCT_HOLD];
                    let margin = self.arc_delay(e, k, MIN);
                    let crpr = self.check_crpr(path, tgt_v, tgt);
                    Self::required_set(req, i, (tgt_clk_arrival + (margin - 0.0)) + (0.0 - crpr), MIN);
                }
            }
        }
    }

    /// The clock path a tag's CRPR clock path names.
    fn crpr_clk_path(&self, c: &CrprPath) -> Option<&Path> {
        self.paths[c.vertex].iter().find(|p| p.tag.is_clock && p.tag.rf == c.rf && p.tag.mm == c.mm && p.tag.clk_edge == Some(c.clk_edge))
    }

    /// The fanin path the arrival came from.
    pub fn prev_path(&self, p: &Path) -> Option<(usize, &Path)> {
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
        // `checkCrpr1`: both clock infos propagated, the same clock (`crprPossible`, no generated
        // clocks here), the CRPR clock path of the other min/max.
        let (Some(se), Some(te)) = (src.tag.clk_edge, tgt.tag.clk_edge) else { return 0.0 };
        if src.tag.mm == tgt.tag.mm || !self.propagated(se) || !self.propagated(te) || se / 2 != te / 2 {
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

    /// `Sta::slack(vertex, rf, min_max)`: over the paths of that min/max (and transition, when
    /// given), the fuzzily least slack in tag order — `Path::slack`: `required − arrival` for a
    /// max path, `arrival − required` for a min path; `INF` with none.
    pub fn slack_of(&self, v: usize, mm: usize, rf: Option<usize>) -> f32 {
        let mut slack = INF;
        for p in self.paths[v].iter().filter(|p| p.tag.mm == mm && rf.is_none_or(|r| p.tag.rf == r)) {
            let s = if mm == MAX { p.required - p.arrival } else { p.arrival - p.required };
            if crate::fuzzy::less(s, slack) {
                slack = s;
            }
        }
        slack
    }

    /// The fuzzily least `required − arrival` over max paths, in tag
    /// order (`INF` with none).
    /// `Search::wnsSlacks`: the slack an endpoint counts toward the worst and the total negative
    /// slack, the max analysis (setup) when `max`, else the min (hold). A vertex with fanout takes its OWN path ends' slacks only — its paths'
    /// required times include the ends downstream (an asynchronous reset's recovery check, and the
    /// clear arc on to the register's output), which are counted where they end; any other vertex
    /// its paths' slacks.
    pub fn wns_slack(&self, v: usize, max: bool) -> f32 {
        let mm = if max { MAX } else { MIN };
        let mut req: Vec<f32> = self.paths[v].iter().map(|p| if p.tag.mm == MAX { INF } else { -INF }).collect();
        if self.has_fanout(v) {
            self.visit_path_ends(v, &mut req);
        } else {
            for (r, p) in req.iter_mut().zip(&self.paths[v]) {
                *r = p.required;
            }
        }
        let mut slack = INF;
        for (p, r) in self.paths[v].iter().zip(req) {
            if p.tag.mm == mm {
                let s = if mm == MAX { r - p.arrival } else { p.arrival - r };
                if crate::fuzzy::less(s, slack) {
                    slack = s;
                }
            }
        }
        slack
    }

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

/// Each port's delays, one per clock in the order first made (`Sdc::findInputDelay` keys a delay
/// by pin and clock edge). A later entry for the same port and clock replaces the earlier one in
/// its place, as `setInputDelay` without `-add_delay` overwrites the delay it finds.
fn port_delays(list: &[PortDelay]) -> HashMap<String, Vec<usize>> {
    let mut map: HashMap<String, Vec<usize>> = HashMap::new();
    for (i, d) in list.iter().enumerate() {
        let on = map.entry(d.port.clone()).or_default();
        match on.iter().position(|&k| list[k].clock == d.clock) {
            Some(k) => on[k] = i,
            None => on.push(i),
        }
    }
    map
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
            rise_constraint (scalar) { values ("0.05"); } fall_constraint (scalar) { values ("0.06"); } }
          timing () { related_pin : "CLK"; timing_type : hold_rising;
            rise_constraint (scalar) { values ("0.02"); } fall_constraint (scalar) { values ("0.03"); } } }
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
        let net = |n: &str, pins: Vec<Conn>| Net { name: n.into(), pins }.into();
        let netlist = Netlist {
            insts: vec![crate::netlist::inst("b1", "buf"), crate::netlist::inst("f1", "dff"), crate::netlist::inst("f2", "dff")],
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
            clocks: vec![Clock::new("c", 1e-9, "clk", true)],
            input_delays: vec![PortDelay::uniform("in", 0.2e-9)],
            output_delays: vec![PortDelay::uniform("out", 0.2e-9)],
            path_delays: Vec::new(),
        };
        (libs, netlist, sdc)
    }

    fn timed<R>(f: impl FnOnce(&Search, &dyn Fn(&str) -> usize) -> R) -> R {
        timed_with(|_| {}, f)
    }

    /// [`timed`] with the constraints edited first.
    fn timed_with<R>(edit: impl FnOnce(&mut Sdc), f: impl FnOnce(&Search, &dyn Fn(&str) -> usize) -> R) -> R {
        let (libs, netlist, mut sdc) = design();
        edit(&mut sdc);
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let mut s = Search::in_graph_order(&g, &sdc);
        s.find_arrivals().unwrap();
        s.find_requireds().unwrap();
        let v = |n: &str| g.vertices.iter().position(|x| x.name == n).unwrap();
        f(&s, &v)
    }

    fn path(s: &Search, v: usize, rf: usize, is_clock: bool) -> Path {
        *s.paths[v].iter().find(|p| p.tag.rf == rf && p.tag.mm == MAX && p.tag.is_clock == is_clock).unwrap()
    }

    /// A register reset asynchronously: `RESET_B` with a rising recovery check and the clear arc on
    /// to Q, reset from an input, Q to an output.
    const LIB_RST: &str = r#"library (r) {
      time_unit : "1ns"; capacitive_load_unit (1, pf);
      cell (buf) {
        pin (A) { direction : input; capacitance : 0.001; }
        pin (X) { direction : output; function : "A";
          timing () { related_pin : "A"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("0.1"); } rise_transition (scalar) { values ("0.05"); }
            cell_fall (scalar) { values ("0.1"); } fall_transition (scalar) { values ("0.05"); } } }
      }
      cell (dffr) {
        ff (IQ, IQN) { clocked_on : "CLK"; next_state : "D"; clear : "!RESET_B"; }
        pin (CLK) { direction : input; capacitance : 0.001; clock : true; }
        pin (D) { direction : input; capacitance : 0.001; }
        pin (RESET_B) { direction : input; capacitance : 0.001;
          timing () { related_pin : "CLK"; timing_type : recovery_rising; rise_constraint (scalar) { values ("0.11"); } } }
        pin (Q) { direction : output; function : "IQ";
          timing () { related_pin : "CLK"; timing_type : rising_edge;
            cell_rise (scalar) { values ("0.3"); } rise_transition (scalar) { values ("0.1"); }
            cell_fall (scalar) { values ("0.35"); } fall_transition (scalar) { values ("0.1"); } }
          timing () { related_pin : "RESET_B"; timing_sense : positive_unate; timing_type : clear;
            cell_fall (scalar) { values ("0.25"); } fall_transition (scalar) { values ("0.1"); } } }
      }
    }"#;

    /// Rules: a recovery check is a path end as a setup check is (`visitCheckEnd`; setup's cycle
    /// accounting, `setSetupAccting`). RESET_B has NO searched fanout — its clear arc to Q is
    /// disabled by default (`SearchPred0::searchThru`, `preset_clr_arcs_enabled` false) — so its
    /// slack is its recovery end's alone. RESET_B: clock 1 ns + 0.1 ns tree − 0.11 recovery − 0.2
    /// input delay.
    #[test]
    fn a_recovery_check_ends_a_path_and_counts_by_its_own_end() {
        let libs = vec![Library::read(&crate::liberty_parse::parse(LIB_RST).unwrap()).unwrap()];
        let i = |k: usize, p: &str| Conn::Inst(k, p.into());
        let net = |n: &str, pins: Vec<Conn>| Net { name: n.into(), pins }.into();
        let netlist = Netlist {
            insts: vec![crate::netlist::inst("b1", "buf"), crate::netlist::inst("f1", "dffr")],
            ports: vec![("clk".into(), PortDir::Input), ("rst".into(), PortDir::Input), ("out".into(), PortDir::Output)],
            nets: vec![
                net("clk", vec![i(0, "A"), Conn::Port(0)]),
                net("cb", vec![i(0, "X"), i(1, "CLK")]),
                net("rst", vec![i(1, "RESET_B"), Conn::Port(1)]),
                net("out", vec![i(1, "Q"), Conn::Port(2)]),
            ],
        };
        let sdc = Sdc {
            clocks: vec![Clock::new("c", 1e-9, "clk", true)],
            input_delays: vec![PortDelay::uniform("rst", 0.2e-9)],
            output_delays: vec![PortDelay::uniform("out", 0.2e-9)],
            path_delays: Vec::new(),
        };
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let mut s = Search::in_graph_order(&g, &sdc);
        s.find_arrivals().unwrap();
        s.find_requireds().unwrap();
        let rb = g.vertices.iter().position(|x| x.name == "f1/RESET_B").unwrap();
        assert!(s.is_endpoint(rb) && !s.has_fanout(rb));
        let own = s.wns_slack(rb, true);
        assert!((own - 0.79e-9).abs() < 1e-15, "recovery slack {own}");
        assert_eq!(s.vertex_slack(rb), own, "no clear-arc end downstream");
    }

    /// Rule: a register's preset/clear arc is not searched by default (`SearchPred0::searchThru`:
    /// `role == regSetClr && !presetClrArcsEnabled()`), so a late reset never reaches Q. Q falls
    /// at the clock's 0.1 ns tree + 0.35 clock-to-Q only; through the clear arc it would be the
    /// 0.6 ns input delay + 0.25 = 0.85 ns.
    #[test]
    fn a_clear_arc_is_not_searched_by_default() {
        let libs = vec![Library::read(&crate::liberty_parse::parse(LIB_RST).unwrap()).unwrap()];
        let i = |k: usize, p: &str| Conn::Inst(k, p.into());
        let net = |n: &str, pins: Vec<Conn>| Net { name: n.into(), pins }.into();
        let netlist = Netlist {
            insts: vec![crate::netlist::inst("b1", "buf"), crate::netlist::inst("f1", "dffr")],
            ports: vec![("clk".into(), PortDir::Input), ("rst".into(), PortDir::Input), ("out".into(), PortDir::Output)],
            nets: vec![
                net("clk", vec![i(0, "A"), Conn::Port(0)]),
                net("cb", vec![i(0, "X"), i(1, "CLK")]),
                net("rst", vec![i(1, "RESET_B"), Conn::Port(1)]),
                net("out", vec![i(1, "Q"), Conn::Port(2)]),
            ],
        };
        let sdc = Sdc {
            clocks: vec![Clock::new("c", 1e-9, "clk", true)],
            input_delays: vec![PortDelay::uniform("rst", 0.6e-9)],
            output_delays: vec![PortDelay::uniform("out", 0.2e-9)],
            path_delays: Vec::new(),
        };
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let mut s = Search::in_graph_order(&g, &sdc);
        s.find_arrivals().unwrap();
        s.find_requireds().unwrap();
        let q = g.vertices.iter().position(|x| x.name == "f1/Q").unwrap();
        let falls: Vec<f32> = s.paths[q].iter().filter(|p| p.tag.mm == MAX && p.tag.rf == 1).map(|p| p.arrival).collect();
        assert_eq!(falls.len(), 1, "one max fall path at Q, the clock's: {falls:?}");
        assert!((falls[0] - 0.45e-9).abs() < 1e-15, "Q fall {}", falls[0]);
    }

    /// Rule: the delay calculation skips a register's preset/clear arc too (`DcalcPred::searchThru`,
    /// the same `presetClrArcsEnabled` test), so its slew never reaches Q: with the clear arc's
    /// fall transition at 0.4 ns, Q's max fall slew stays clock-to-Q's 0.1 ns.
    #[test]
    fn a_clear_arcs_slew_does_not_reach_its_output() {
        let lib = LIB_RST.replacen(
            r#"cell_fall (scalar) { values ("0.25"); } fall_transition (scalar) { values ("0.1"); }"#,
            r#"cell_fall (scalar) { values ("0.25"); } fall_transition (scalar) { values ("0.4"); }"#,
            1,
        );
        assert_ne!(lib, LIB_RST);
        let libs = vec![Library::read(&crate::liberty_parse::parse(&lib).unwrap()).unwrap()];
        let i = |k: usize, p: &str| Conn::Inst(k, p.into());
        let net = |n: &str, pins: Vec<Conn>| Net { name: n.into(), pins }.into();
        let netlist = Netlist {
            insts: vec![crate::netlist::inst("b1", "buf"), crate::netlist::inst("f1", "dffr")],
            ports: vec![("clk".into(), PortDir::Input), ("rst".into(), PortDir::Input), ("out".into(), PortDir::Output)],
            nets: vec![
                net("clk", vec![i(0, "A"), Conn::Port(0)]),
                net("cb", vec![i(0, "X"), i(1, "CLK")]),
                net("rst", vec![i(1, "RESET_B"), Conn::Port(1)]),
                net("out", vec![i(1, "Q"), Conn::Port(2)]),
            ],
        };
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let q = g.vertices.iter().position(|x| x.name == "f1/Q").unwrap();
        assert_eq!(g.slew[q][FALL][MAX], 0.1e-9, "Q fall slew is clock-to-Q's");
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

    /// Rule: `ClkInfo` keeps a CRPR clock path only for a PROPAGATED clock
    /// (`crpr_clk_path_(is_propagated ? crpr_clk_path : nullptr)`) — under an ideal clock neither
    /// the clock tag nor the launched data tag carries one, so every register's launch shares a
    /// tag (one merged arrival, one fuzzy change test, where per-register tags left ~1 fs drift).
    #[test]
    fn an_ideal_clock_carries_no_crpr_path() {
        timed_with(
            |sdc| sdc.clocks[0].propagated = false,
            |s, v| {
                assert_eq!(path(s, v("f1/CLK"), RISE, true).tag.crpr, None);
                assert_eq!(path(s, v("f1/Q"), RISE, false).tag.crpr, None);
            },
        );
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

    /// Rules (clkPathArrival, targetClkDelay): an IDEAL clock launches its registers at the edge
    /// and captures them with no network latency — the clock buffer's 0.1 ns counts for neither.
    #[test]
    fn an_ideal_clock_launches_and_captures_at_its_edge() {
        let (libs, netlist, mut sdc) = design();
        sdc.clocks[0].propagated = false;
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let mut s = Search::in_graph_order(&g, &sdc);
        s.find_arrivals().unwrap();
        s.find_requireds().unwrap();
        let v = |n: &str| g.vertices.iter().position(|x| x.name == n).unwrap();
        assert_eq!(path(&s, v("f1/Q"), RISE, false).arrival, 0.0 + 0.3f32 * 1e-9, "launched at the edge");
        let d = path(&s, v("f2/D"), RISE, false);
        assert_eq!(d.required, (0.0 + 1e-9f32) - 0.05e-9, "captured at the edge, less the setup");
    }

    /// Rule (`thruClkInfo` → `ClkInfo::latency`; `clkPathArrival`; `checkTgtClkDelay`): an ideal
    /// clock's network latency set on a register's clock pin rides on the clock path from that
    /// pin — the register launches at edge + ITS latency, a check captures at edge + the CAPTURING
    /// register's latency — and data paths that differ only in it still merge (`Tag::matchCmp`).
    #[test]
    fn an_ideal_clock_takes_each_register_pin_latency() {
        let (libs, netlist, mut sdc) = design();
        sdc.clocks[0].propagated = false;
        let lat = |pin: &str, d: f32| crate::sdc::PinLatency { pin: pin.into(), specific: true, delay: [[Some(d); 2]; 2] };
        sdc.clocks[0].pin_latencies = vec![lat("f1/CLK", 0.2e-9), lat("f2/CLK", 0.07e-9)];
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let mut s = Search::in_graph_order(&g, &sdc);
        s.find_arrivals().unwrap();
        s.find_requireds().unwrap();
        let v = |n: &str| g.vertices.iter().position(|x| x.name == n).unwrap();
        let q = path(&s, v("f1/Q"), RISE, false);
        assert_eq!(q.arrival, (0.0 + 0.0) + 0.2e-9f32 + 0.3f32 * 1e-9, "launched at the edge + f1's latency");
        assert_eq!(q.tag.latency, 0.2e-9f32.to_bits());
        let d = path(&s, v("f2/D"), RISE, false);
        assert_eq!(d.required, ((0.0 + 0.07e-9f32) + 1e-9) - (0.05e-9 + 0.0), "captured at the edge + f2's latency, less the setup");
        // a pin given no latency for this clock: none
        assert_eq!(sdc.clocks[0].pin_latency("f3/CLK", RISE, MAX), None);
    }

    /// Rule (`ArrivalVisitor::seedArrivals`: `isRegClk() && !hasClkTag()` → `makeUnclkedPaths(…,
    /// is_segment_start = true)`; `visitFromPath`: `clk_edge == nullptr && isSegmentStart()` crosses
    /// clock-to-Q): with no clock, each register clock pin seeds an unclocked segment start at 0
    /// and launches an unclocked path at its clock-to-Q delay — no required, so no slack.
    #[test]
    fn an_unclocked_register_launches_from_a_segment_start() {
        let (libs, netlist, mut sdc) = design();
        sdc.clocks.clear();
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let mut s = Search::in_graph_order(&g, &sdc);
        s.constraints_modelled().unwrap();
        s.find_arrivals().unwrap();
        s.find_requireds().unwrap();
        let v = |n: &str| g.vertices.iter().position(|x| x.name == n).unwrap();
        let ck: Vec<&Path> = s.paths[v("f1/CLK")].iter().filter(|p| p.tag.segment_start).collect();
        assert_eq!(ck.len(), 4, "a segment start per transition and min/max");
        assert!(ck.iter().all(|p| p.arrival == 0.0 && p.tag.clk_edge.is_none()));
        let q = s.paths[v("f1/Q")].iter().find(|p| p.tag.mm == MAX && p.tag.rf == RISE).expect("launched");
        let prev = q.prev.expect("from the clock pin");
        assert_eq!((prev.vertex, prev.tag.segment_start, q.tag.segment_start, q.tag.clk_edge), (v("f1/CLK"), true, false, None));
        assert_eq!(q.arrival, 0.0 + g.delay[prev.edge][prev.arc][MAX]);
    }

    #[test]
    fn a_comb_logic_cone_stops_at_registers() {
        // `SearchPredCombLogic`: a cell arc is searched through, a register's clock-to-Q is not,
        // and neither is a timing check.
        timed(|s, v| {
            let edge = |a: &str, b: &str| (0..s.graph.edges.len()).find(|&e| s.graph.edges[e].from == v(a) && s.graph.edges[e].to == v(b)).unwrap();
            assert!(s.comb_logic_thru(edge("b1/A", "b1/X")));
            assert!(s.search_thru(edge("f1/CLK", "f1/Q")), "the arrival search goes through it");
            assert!(!s.comb_logic_thru(edge("f1/CLK", "f1/Q")));
            assert!(!s.comb_logic_thru(edge("f1/CLK", "f1/D")));
            assert!(s.is_reg_output(v("f1/Q")) && !s.is_reg_output(v("b1/X")) && !s.is_reg_output(v("out")));
        });
    }

    #[test]
    fn the_worst_slack_path_expands_back_to_the_clock_source() {
        // `vertexWorstSlackPath` + `PathExpanded::size`: f2/D's worst path is f1/Q's launch, which
        // runs back through f1/CLK, b1/X, b1/A to the clock port — six paths.
        timed(|s, v| {
            let p = s.worst_slack_path(v("f2/D"), MAX).expect("constrained");
            let slack = p.required - p.arrival;
            assert!(s.paths[v("f2/D")].iter().filter(|q| q.tag.mm == MAX).all(|q| q.required - q.arrival >= slack));
            assert_eq!(s.expanded_size(p), 6);
            let (pv, _) = s.prev_path(p).unwrap();
            assert_eq!(pv, v("f1/Q"));
        });
        // No required time (no output delay): no worst-slack path, as for an unconstrained end.
        timed_with(|sdc| sdc.output_delays.clear(), |s, v| assert!(s.worst_slack_path(v("out"), MAX).is_none()));
    }

    #[test]
    fn per_transition_arrival_and_required_take_the_worst_of_their_paths() {
        timed(|s, v| {
            let d = v("f2/D");
            for rf in [RISE, FALL] {
                let of = |f: fn(&Path) -> f32| s.paths[d].iter().filter(|p| p.tag.mm == MAX && p.tag.rf == rf).map(f).collect::<Vec<f32>>();
                assert_eq!(s.arrival_rf(d, rf, MAX), of(|p| p.arrival).into_iter().fold(-INF, f32::max));
                assert_eq!(s.required_rf(d, rf, MAX), of(|p| p.required).into_iter().fold(INF, f32::min));
            }
            let a = s.worst_arrival_path(d, MAX).unwrap().arrival;
            assert_eq!(a, s.arrival_rf(d, RISE, MAX).max(s.arrival_rf(d, FALL, MAX)));
            // A vertex no path reaches keeps the initial values.
            let none = (0..s.graph.vertices.len()).find(|&x| s.paths[x].is_empty());
            if let Some(x) = none {
                assert_eq!((s.arrival_rf(x, RISE, MAX), s.required_rf(x, RISE, MAX)), (-INF, INF));
            }
        });
    }

    #[test]
    fn only_a_clock_tag_is_a_clock_arrival() {
        // A register clock pin a clock reaches has a clock arrival; its Q's data tag names the
        // launching clock but is not one; with no clock, the CLK pin's segment start is not one.
        timed(|s, v| assert!(s.has_clock_arrival(v("f1/CLK")) && !s.has_clock_arrival(v("f1/Q"))));
        timed_with(|sdc| sdc.clocks.clear(), |s, v| assert!(!s.has_clock_arrival(v("f1/CLK")) && !s.paths[v("f1/CLK")].is_empty()));
    }

    /// A second, virtual clock `v` (rising 0.3 ns into the same 1 ns period).
    fn with_virtual_clock(sdc: &mut Sdc) {
        let mut v = Clock::new("v", 1e-9, "", false);
        v.waveform = [0.3e-9, 0.8e-9];
        sdc.clocks.push(v);
    }

    /// Rule (`seedInputArrival1`): a port with input delays on two clocks gets an arrival per
    /// delay, each tagged with its own clock's edge — not one delay replacing the other.
    #[test]
    fn input_delays_on_two_clocks_seed_an_arrival_each() {
        timed_with(
            |sdc| {
                with_virtual_clock(sdc);
                sdc.input_delays.push(PortDelay { clock: 1, ..PortDelay::uniform("in", 0.1e-9) });
            },
            |s, v| {
                let edges = |rf| s.paths[v("f1/D")].iter().filter(|p| p.tag.mm == MAX && p.tag.rf == rf).map(|p| (p.tag.clk_edge, p.arrival)).collect::<Vec<_>>();
                assert_eq!(edges(RISE), [(Some(0), 0.0 + 0.2e-9f32), (Some(2), 0.3e-9f32 + 0.1e-9)]);
            },
        );
    }

    /// Rule (`visitOutputDelayEnd`): a port with output delays on two clocks ends a path once per
    /// delay; the path's required is the tighter of the two ends.
    #[test]
    fn output_delays_on_two_clocks_end_a_path_each() {
        let req = |delays: &'static [(usize, f32)]| {
            timed_with(
                move |sdc| {
                    with_virtual_clock(sdc);
                    sdc.output_delays = delays.iter().map(|&(clock, d)| PortDelay { clock, ..PortDelay::uniform("out", d) }).collect();
                },
                |s, v| path(s, v("out"), RISE, false).required,
            )
        };
        let (c, vr) = (req(&[(0, 0.2e-9)]), req(&[(1, 0.4e-9)]));
        assert!(vr < c, "the virtual clock's end is the tighter: {vr} vs {c}");
        assert_eq!(req(&[(0, 0.2e-9), (1, 0.4e-9)]), vr);
        assert_eq!(req(&[(1, 0.4e-9), (0, 0.2e-9)]), vr);
        // The hold ends likewise: a min path's required is the larger of the two.
        let hold = |delays: &'static [(usize, f32)]| {
            timed_with(
                move |sdc| {
                    with_virtual_clock(sdc);
                    sdc.output_delays = delays.iter().map(|&(clock, d)| PortDelay { clock, ..PortDelay::uniform("out", d) }).collect();
                },
                |s, v| s.paths[v("out")].iter().find(|p| p.tag.rf == RISE && p.tag.mm == MIN && !p.tag.is_clock).unwrap().required,
            )
        };
        let (hc, hv) = (hold(&[(0, 0.2e-9)]), hold(&[(1, 0.4e-9)]));
        assert_ne!(hc, hv);
        assert_eq!(hold(&[(0, 0.2e-9), (1, 0.4e-9)]), hc.max(hv));
        assert_eq!(hold(&[(1, 0.4e-9), (0, 0.2e-9)]), hc.max(hv));
    }

    /// Rule (`findInputDelay` keys a delay by pin and clock edge; `setInputDelay` without
    /// `-add_delay` overwrites the one it finds): a second entry for the same port and clock
    /// replaces the first in its place.
    #[test]
    fn a_second_delay_on_the_same_clock_replaces_the_first() {
        let list = [PortDelay::uniform("a", 1.0), PortDelay { clock: 1, ..PortDelay::uniform("a", 2.0) }, PortDelay::uniform("a", 3.0)];
        assert_eq!(port_delays(&list)["a"], [2, 1]);
    }

    /// Rules (`ArrivalVisitor::seedArrivals` → `seedClkArrivals` on `isLeafPinClock`, any pin;
    /// `EvalPred::searchTo` stops at a clock's leaf pin): a clock on an instance's output pin
    /// launches there at its edge times, nothing from the pin's fanin arrives over it, and the
    /// registers it drives read that clock.
    #[test]
    fn a_clock_on_an_instance_pin_seeds_there_and_blocks_its_fanin() {
        timed_with(
            |sdc| sdc.clocks[0].source = "b1/X".into(),
            |s, v| {
                let x = v("b1/X");
                assert!(s.paths[x].iter().all(|p| p.tag.is_clock && p.tag.clk_edge.is_some()));
                assert_eq!(path(s, x, RISE, true).arrival, 0.0);
                assert_eq!(path(s, x, FALL, true).arrival, 0.5e-9);
                assert_eq!(s.find_clk(v("f1/CLK")).map(|c| c.name.as_str()), Some("c"));
                // the port `clk`, no longer a clock, arrives unclocked at b1/A and stops there
                assert!(s.paths[v("b1/A")].iter().all(|p| !p.tag.is_clock));
            },
        );
    }

    /// Rule (`seedInputArrival1`): an input delay on a clock's own source port seeds no arrival
    /// for that clock, but one relative to another clock does, beside the clock arrivals.
    #[test]
    fn an_input_delay_on_a_clock_port_seeds_only_other_clocks() {
        let data = |clock: usize| {
            timed_with(
                move |sdc| {
                    with_virtual_clock(sdc);
                    sdc.input_delays.push(PortDelay { clock, ..PortDelay::uniform("clk", 0.1e-9) });
                },
                |s, v| s.paths[v("clk")].iter().filter(|p| !p.tag.is_clock).map(|p| p.tag.clk_edge).collect::<Vec<_>>(),
            )
        };
        assert!(data(0).is_empty());
        assert_eq!(data(1), [Some(2); 4]);
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

    /// Rules (PathEndCheck::requiredTimeNoCrpr / checkCrpr under the hold role; RequiredCmp with
    /// the min path's opposite, max): a min path's required at a hold check is (the LATE capture
    /// clock arrival + the hold cycle's target time) + the hold margin − crpr; at an output delay,
    /// the hold target time + −its min delay; back through the fanout, the greatest.
    #[test]
    fn min_paths_take_hold_requireds() {
        timed(|s, v| {
            let b = 0.1f32 * 1e-9;
            let min = |vx: usize, rf: usize| *s.paths[vx].iter().find(|p| p.tag.rf == rf && p.tag.mm == MIN && !p.tag.is_clock).unwrap();
            let d = min(v("f2/D"), RISE);
            assert_eq!(d.required, (((0.0 + ((0.0 + b) - 0.0 - 0.0)) + 0.0) + (0.02f32 * 1e-9 - 0.0)) + (0.0 - 0.0));
            assert_eq!(min(v("f2/D"), FALL).required, (((0.0 + ((0.0 + b) - 0.0 - 0.0)) + 0.0) + (0.03f32 * 1e-9 - 0.0)) + (0.0 - 0.0));
            assert_eq!(min(v("out"), RISE).required, (0.0 + (0.0 + 0.0)) + -0.2e-9f32);
            // f1/Q drives f2/D over a wire with no delay: the same required.
            assert_eq!(min(v("f1/Q"), RISE).required, d.required);
        });
    }

    /// Rule (RiseFallMinMax::value exists): an output delay set for min only makes a hold end
    /// and no setup end — the max required stays unconstrained.
    #[test]
    fn a_min_only_output_delay_constrains_hold_alone() {
        let (libs, netlist, mut sdc) = design();
        sdc.output_delays[0].exists = [[true, false], [true, false]];
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let mut s = Search::in_graph_order(&g, &sdc);
        s.find_arrivals().unwrap();
        s.find_requireds().unwrap();
        let out = g.vertices.iter().position(|x| x.name == "out").unwrap();
        assert_eq!(s.slack_of(out, MAX, None), INF);
        let hold = *s.paths[out].iter().find(|p| p.tag.rf == RISE && p.tag.mm == MIN).unwrap();
        assert_eq!(hold.required, (0.0 + (0.0 + 0.0)) + -0.2e-9f32);
    }

    /// Rule (Path::slack, Sta::slack): a min path's slack is arrival − required; the vertex's is
    /// the least over the min paths (per transition when asked).
    #[test]
    fn a_min_slack_is_arrival_less_required() {
        timed(|s, v| {
            let d = v("f2/D");
            let min = |rf: usize| *s.paths[d].iter().find(|p| p.tag.rf == rf && p.tag.mm == MIN && !p.tag.is_clock).unwrap();
            let (r, f) = (min(RISE), min(FALL));
            assert_eq!(s.slack_of(d, MIN, Some(RISE)), r.arrival - r.required);
            assert_eq!(s.slack_of(d, MIN, None), (r.arrival - r.required).min(f.arrival - f.required));
            assert_eq!(s.slack_of(d, MAX, None), s.vertex_slack(d));
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

    /// `set_max_delay -ignore_clock_latency -from f1/CLK -to f2/D <d>`.
    fn max_delay_f1_f2(d: f32) -> crate::sdc::PathDelay {
        crate::sdc::PathDelay { from_pins: vec!["f1/CLK".into()], from_clock: false, to_pins: vec!["f2/D".into()], to_clock: false, min_max: MAX, ignore_clk_latency: true, break_path: true, delay: d }
    }

    /// Rules (`fromRegClkTag` → `exceptionFromStates`; `PathEndPathDelay::requiredTime`): a path
    /// delay `-from` a register clock pin puts its state on the MAX paths that pin launches (its
    /// min/max only); at the `-to` pin's setup check the path-delay end overrides the check, with
    /// required = the SOURCE clock path's arrival (propagated, `-ignore_clock_latency`) + the
    /// delay − the setup margin, and no CRPR. The input's path to f1/D carries no state.
    #[test]
    fn a_max_delay_from_a_clock_pin_overrides_the_setup_check() {
        let d = 0.3e-9f32;
        timed_with(
            |sdc| sdc.path_delays.push(max_delay_f1_f2(d)),
            |s, v| {
                assert!(s.constraints_modelled().is_ok());
                for p in &s.paths[v("f1/Q")] {
                    assert_eq!(p.tag.states, if p.tag.mm == MAX { States(1) } else { States::NONE });
                }
                assert!(s.paths[v("f1/D")].iter().all(|p| p.tag.states == States::NONE));
                let (dv, ckv) = (v("f2/D"), v("f2/CLK"));
                let e = *s.graph.in_edges[dv].iter().find(|&&e| s.role(e) == Some(Role::Setup) && s.graph.edges[e].from == ckv).unwrap();
                for rf in [RISE, FALL] {
                    let p = path(s, dv, rf, false);
                    let q = s.prev_path(&p).unwrap().1;
                    let src = s.prev_path(q).unwrap().1;
                    assert!(src.tag.is_clock);
                    let k = s.arcs_from(e, RISE).iter().position(|a| a.to_rf == rf).unwrap();
                    assert_eq!(p.required, (src.arrival + d) - s.graph.delay[e][k][MAX]);
                }
            },
        );
    }

    /// Rules (`seedClkArrival` / `fromRegClkTag` → the `-from` clock's states;
    /// `PathEndPathDelay::requiredTime` without `-ignore_clock_latency`, hold role): `set_min_delay
    /// -from clk -to clk` rides the min clock tags and the min paths the registers launch; at a hold
    /// check it overrides the check with required = (target clock latency − CRPR) + the delay −
    /// (−source edge time) + the hold margin — no target edge time.
    #[test]
    fn a_min_delay_between_clocks_overrides_the_hold_check() {
        let d = 0.8e-9f32;
        let pd = crate::sdc::PathDelay { from_pins: Vec::new(), from_clock: true, to_pins: Vec::new(), to_clock: true, min_max: MIN, ignore_clk_latency: false, break_path: true, delay: d };
        let (libs, netlist, mut sdc) = design();
        sdc.output_delays.clear();
        sdc.path_delays.push(pd);
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let mut s = Search::in_graph_order(&g, &sdc);
        assert!(s.constraints_modelled().is_ok());
        s.find_arrivals().unwrap();
        s.find_requireds().unwrap();
        let v = |n: &str| g.vertices.iter().position(|x| x.name == n).unwrap();
        for p in s.paths[v("b1/X")].iter().chain(&s.paths[v("f1/Q")]) {
            assert_eq!(p.tag.states, if p.tag.mm == MIN { States(1) } else { States::NONE });
        }
        let (dv, ckv) = (v("f2/D"), v("f2/CLK"));
        let e = *s.graph.in_edges[dv].iter().find(|&&e| s.role(e) == Some(Role::Hold)).unwrap();
        for rf in [RISE, FALL] {
            let p = *s.paths[dv].iter().find(|p| p.tag.rf == rf && p.tag.mm == MIN && p.tag.clk_edge == Some(RISE) && p.tag.states == States(1)).unwrap();
            let k = s.arcs_from(e, RISE).iter().position(|a| a.to_rf == rf).unwrap();
            let tgt = s.paths[ckv].iter().find(|t| t.tag.mm == MAX && t.tag.rf == RISE && t.tag.is_clock).unwrap();
            let latency = (tgt.arrival - s.edge_time(RISE)) - 0.0;
            let crpr = s.check_crpr(&p, ckv, tgt);
            let expect = (((latency + 0.0) + (0.0 - crpr)) + d) - -s.edge_time(RISE) + s.graph.delay[e][k][MIN];
            assert_eq!(p.required, expect);
        }
    }

    /// The modelled forms only: a `-from` that is not a register clock pin, and `set_min_delay`,
    /// are refused rather than timed wrong.
    #[test]
    fn unmodelled_path_delays_are_refused() {
        let modelled = |pd: crate::sdc::PathDelay| {
            let (libs, netlist, mut sdc) = design();
            sdc.path_delays.push(pd);
            let g = Graph::build(&libs, &netlist).unwrap();
            Search::in_graph_order(&g, &sdc).constraints_modelled()
        };
        assert!(modelled(max_delay_f1_f2(1e-9)).is_ok());
        assert!(modelled(crate::sdc::PathDelay { from_pins: vec!["f1/Q".into()], ..max_delay_f1_f2(1e-9) }).is_err());
        assert!(modelled(crate::sdc::PathDelay { to_pins: vec!["out".into()], ..max_delay_f1_f2(1e-9) }).is_err());
        assert!(modelled(crate::sdc::PathDelay { to_pins: Vec::new(), to_clock: true, ..max_delay_f1_f2(1e-9) }).is_err());
        assert!(modelled(crate::sdc::PathDelay { ignore_clk_latency: false, ..max_delay_f1_f2(1e-9) }).is_err());
    }

    /// in -> b1 -> b2 -> out, no clock, `set_max_delay -from b1/X -to b2/A <d>`.
    fn internal_pins_timed<R>(f: impl FnOnce(&Search, &dyn Fn(&str) -> usize) -> R) -> R {
        let libs = vec![Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()];
        let i = |k: usize, p: &str| Conn::Inst(k, p.into());
        let net = |n: &str, pins: Vec<Conn>| Net { name: n.into(), pins }.into();
        let netlist = Netlist {
            insts: vec![crate::netlist::inst("b1", "buf"), crate::netlist::inst("b2", "buf")],
            ports: vec![("in".into(), PortDir::Input), ("out".into(), PortDir::Output)],
            nets: vec![net("in", vec![i(0, "A"), Conn::Port(0)]), net("m", vec![i(0, "X"), i(1, "A")]), net("out", vec![i(1, "X"), Conn::Port(1)])],
        };
        let pd = crate::sdc::PathDelay { from_pins: vec!["b1/X".into()], from_clock: false, to_pins: vec!["b2/A".into()], to_clock: false, min_max: MAX, ignore_clk_latency: false, break_path: true, delay: 2e-9 };
        let sdc = Sdc { clocks: Vec::new(), input_delays: Vec::new(), output_delays: Vec::new(), path_delays: vec![pd] };
        let mut g = Graph::build(&libs, &netlist).unwrap();
        g.find_delays(&HashMap::new(), None).unwrap();
        let mut s = Search::in_graph_order(&g, &sdc);
        s.constraints_modelled().unwrap();
        s.find_arrivals().unwrap();
        s.find_requireds().unwrap();
        let v = |n: &str| g.vertices.iter().position(|x| x.name == n).unwrap();
        f(&s, &v)
    }

    /// Rules (`seedInputArrival` with no input delay; `isPathDelayInternalFrom` → `makeUnclkedPaths`
    /// with `require_exception`; `…FromBreak` / `…ToBreak`; `isEndpoint`; `pathDelayTo` →
    /// `PathEndPathDelay` with no clock): the input is unclocked at 0; the internal `-from` pin
    /// drops its fanin and starts MAX paths only (the delay's min/max), carrying the delay; the
    /// pin before it has no fanout left and is an end; the internal `-to` pin is an end whose
    /// required is the delay itself, and nothing passes it.
    #[test]
    fn internal_path_delay_pins_start_break_and_end_the_search() {
        internal_pins_timed(|s, v| {
            assert!(s.paths[v("in")].iter().all(|p| p.tag.clk_edge.is_none() && p.arrival == 0.0 && p.tag.states == States::NONE));
            assert!(s.paths[v("b1/X")].iter().all(|p| p.tag.mm == MAX && p.tag.states == States(1) && p.arrival == 0.0 && p.prev.is_none()));
            assert_eq!(s.paths[v("b1/X")].len(), 2);
            assert!(s.paths[v("b2/X")].is_empty() && s.paths[v("out")].is_empty());
            let ends: Vec<usize> = (0..s.graph.vertices.len()).filter(|&x| s.is_endpoint(x)).collect();
            assert_eq!(ends, vec![v("b1/A"), v("b2/A"), v("out")]);
            for p in &s.paths[v("b2/A")] {
                assert_eq!(p.required, 2e-9);
            }
            assert!(s.slack_of(v("b1/A"), MAX, None) >= INF);
        });
    }
}
