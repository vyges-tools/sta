// SPDX-License-Identifier: Apache-2.0
//! Switching activity and instance power, in the reference's order.
//!
//! An activity is a transition density (transitions per second) and a duty (the probability the
//! signal is high), with where it came from.
//!
//! A function's activity, at one instance (each rule pinned by a test below):
//! - a function that is one INTERNAL port (a register's `IQ`) takes that register output's
//!   activity as the register pass left it;
//! - otherwise the function's diagram ([`crate::bdd`]) is walked for the duty: at a node over
//!   variable `x` with duty `p`, `duty = duty(else) * (1 - p) + duty(then) * p`: the first product in
//!   double (`1 - p` is double), the second in FLOAT (float times float), their sum in double,
//!   narrowed; then `1 - duty` when the edge is complemented. A node over an
//!   internal port is that port's duty, with no complement; a node whose port has no pin on the
//!   instance is 0, with no complement;
//! - the density is the sum, over the function's ports IN NAME ORDER that have a pin, of the
//!   pin's density times the duty of the function's Boolean difference on the port, each product
//!   and the running sum in float.

use std::collections::{BTreeSet, HashMap};

use crate::bdd::{self, Bdd, Edge};
use crate::func_expr::FuncExpr;
use crate::graph::{EdgeKind, Graph};
use crate::liberty::{Cell, Direction, Role};
use crate::netlist::Conn;

/// Where an activity came from, in the reference's order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Origin {
    Global,
    Input,
    User,
    Vcd,
    Saif,
    Propagated,
    Clock,
    Constant,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Activity {
    pub density: f32,
    pub duty: f32,
    pub origin: Origin,
}

/// Below this magnitude a density made by [`Activity::new`] is 0 (deep chains of probabilities).
pub const MIN_DENSITY: f32 = 1e-10;

impl Activity {
    /// Not set: 0, 0, unknown.
    pub const UNKNOWN: Activity = Activity { density: 0.0, duty: 0.0, origin: Origin::Unknown };

    /// An activity as constructed or `set`: a density under [`MIN_DENSITY`] in magnitude is 0.
    /// (Assigning the density alone does not clip.)
    pub fn new(density: f32, duty: f32, origin: Origin) -> Activity {
        let density = if density.abs() < MIN_DENSITY { 0.0 } else { density };
        Activity { density, duty, origin }
    }

    pub fn is_set(&self) -> bool {
        self.origin != Origin::Unknown
    }
}

/// What evaluating a function at one instance reads.
pub trait InstPins {
    /// The cell's port is internal (a register's `IQ`, `IQN`): it has no pin on the instance.
    fn is_internal(&self, port: &str) -> bool;
    /// The activity of the instance's pin on `port` (`findActivity`), or `None` when the
    /// instance has no pin there (an internal port included).
    fn pin_activity(&self, port: &str) -> Option<Activity>;
    /// The register output's activity as the register pass set it (`findSeqActivity`):
    /// [`Activity::UNKNOWN`] when none.
    fn seq_activity(&self, port: &str) -> Activity;
}

/// `evalActivity`: the activity a function gives at one instance.
pub fn eval_activity(expr: &FuncExpr, pins: &impl InstPins) -> Activity {
    if let FuncExpr::Port(p) = expr {
        if pins.is_internal(p) {
            return pins.seq_activity(p);
        }
    }
    let (mut b, f) = Bdd::of(expr);
    let duty = eval_bdd_duty(&b, f, pins);
    let density = eval_bdd_activity(&mut b, f, pins);
    Activity::new(density, duty, Origin::Propagated)
}

/// `evalDiffDuty`: the duty of the function's Boolean difference on `port` — the probability the
/// function is sensitive to it.
pub fn eval_diff_duty(expr: &FuncExpr, port: &str, pins: &impl InstPins) -> f32 {
    let (mut b, f) = Bdd::of(expr);
    let Some(v) = b.var_index(port) else {
        // The port is not in the function: the reference asks the diagram for a variable it
        // never made. Not reached by a function that names the port.
        return 0.0;
    };
    let diff = b.boolean_diff(f, v);
    eval_bdd_duty(&b, diff, pins)
}

/// `evalBddDuty`: the probability the diagram's function is 1.
pub fn eval_bdd_duty(b: &Bdd, e: Edge, pins: &impl InstPins) -> f32 {
    if b.is_constant(e) {
        return if e == bdd::ONE { 1.0 } else { 0.0 };
    }
    let duty0 = eval_bdd_duty(b, b.else_child(e), pins);
    let duty1 = eval_bdd_duty(b, b.then_child(e), pins);
    let port = &b.vars()[b.var(e) as usize];
    if pins.is_internal(port) {
        return pins.seq_activity(port).duty;
    }
    match pins.pin_activity(port) {
        Some(a) => {
            let p = f64::from(a.duty);
            let duty = (f64::from(duty0) * (1.0 - p) + f64::from(duty1 * a.duty)) as f32;
            if bdd::is_complement(e) {
                (1.0 - f64::from(duty)) as f32
            } else {
                duty
            }
        }
        None => 0.0,
    }
}

/// `evalBddActivity`: the function's transition density, summed over its ports in name order.
pub fn eval_bdd_activity(b: &mut Bdd, f: Edge, pins: &impl InstPins) -> f32 {
    let mut ports: Vec<(String, u32)> = b.vars().iter().enumerate().map(|(i, p)| (p.clone(), i as u32)).collect();
    ports.sort();
    let mut density = 0.0f32;
    for (port, v) in ports {
        let Some(a) = pins.pin_activity(&port) else { continue };
        let diff = b.boolean_diff(f, v);
        let diff_duty = eval_bdd_duty(b, diff, pins);
        density += a.density * diff_duty;
    }
    density
}

/// A clock's activity (`findActivity` on a clock pin): two transitions per period, at the duty
/// of its waveform's first edges, `(fall - rise) / period`, in float.
pub fn clock_activity(period: f32, rise: f32, fall: f32) -> Activity {
    Activity::new((2.0 / f64::from(period)) as f32, (fall - rise) / period, Origin::Clock)
}

/// The default input activity: a tenth of a transition per fastest clock period (`0.1 /
/// min_period`, in double, narrowed), duty 0.5; with no clock, the period is the time unit.
pub fn input_activity(min_period: f32, time_unit: f32) -> Activity {
    let min_period = if min_period == 0.0 { time_unit } else { min_period };
    Activity::new((0.1 / f64::from(min_period)) as f32, 0.5, Origin::Input)
}

/// `percentChange`: |value - prev| / prev in float; from 0, 0 to 0 is no change and anything
/// else is all change.
pub fn percent_change(value: f32, prev: f32) -> f32 {
    if prev == 0.0 {
        if value == 0.0 {
            0.0
        } else {
            1.0
        }
    } else {
        (value - prev).abs() / prev
    }
}

/// The propagation stops revisiting a pin whose density and duty moved by at most this (and
/// whose origin did not change).
pub const CHANGE_TOLERANCE: f32 = 0.01;
/// The register passes after the first propagation.
pub const MAX_ACTIVITY_PASSES: usize = 50;

/// What the propagation reads besides the graph, per vertex.
pub struct ActivityInputs<'x> {
    /// `Levelize` levels (any order-equivalent numbering).
    pub level: &'x [i32],
    /// A clock is defined at the pin (`isLeafPinClock`): not seeded.
    pub clock_source: &'x [bool],
    /// `ClkNetwork::isClock`.
    pub is_clock: &'x [bool],
    /// The activity of the fastest clock among the pin's arrivals (`findClk`), for a clock pin.
    pub clock: &'x [Option<Activity>],
    /// `getMinRfSlew`: over the analysis points, the least of (rise + fall) / 2.
    pub min_rf_slew: &'x [f32],
    /// The SDC's fastest clock period (0 without clocks) and the time unit.
    pub min_period: f32,
    pub time_unit: f32,
}

/// The activities the propagation settled on, and its trace.
#[derive(Debug, Default, Clone)]
pub struct Activities {
    /// Per vertex, the activity set on it (`None`: never set).
    pub pin: Vec<Option<Activity>>,
    /// Per (instance, register output port), as the register passes set it.
    pub seq: HashMap<(usize, String), Activity>,
    /// The register passes run after the first propagation.
    pub passes: usize,
    pub trace: Vec<String>,
}

/// A vertex name as the reference's path names print it: each escaping backslash dropped
/// (`a\/b\[1\]` prints `a/b[1]`; an escaped backslash prints one backslash). Constraints name
/// pins this way (`get_pins {a.b[0]/IN}` is the pin `a\.b\[0\]/IN`).
pub fn path_name(name: &str) -> String {
    let mut out = String::with_capacity(name.len());
    let mut it = name.chars();
    while let Some(c) = it.next() {
        match c {
            '\\' => out.extend(it.next()),
            c => out.push(c),
        }
    }
    out
}

fn bits(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}

/// The level-ordered queue the propagation visits through (`BfsFwdIterator`): one stack per
/// level, so a level's vertices are visited LAST ENQUEUED FIRST; levels in increasing order.
struct Bfs {
    queue: Vec<Vec<usize>>,
    in_queue: Vec<bool>,
    first: i64,
    last: i64,
}

impl Bfs {
    fn new(n: usize, max_level: i32) -> Bfs {
        Bfs { queue: vec![Vec::new(); max_level as usize + 1], in_queue: vec![false; n], first: i64::from(max_level), last: 0 }
    }

    fn enqueue(&mut self, v: usize, level: i32) {
        if !self.in_queue[v] {
            self.in_queue[v] = true;
            self.queue[level as usize].push(v);
            let level = i64::from(level);
            if self.last < level {
                self.last = level;
            }
            if level < self.first {
                self.first = level;
            }
        }
    }
}

struct Prop<'x, 'g, 'a> {
    g: &'g Graph<'a>,
    inp: &'x ActivityInputs<'x>,
    /// Per instance, its pins' vertices in pin order.
    inst_pins: Vec<Vec<usize>>,
    /// Per vertex, its pin's first vertex: an activity is the PIN's (`activity_map_` is keyed by
    /// pin), so a bidirect pin's load and driver vertices share the first's.
    pin_key: Vec<usize>,
    /// Per vertex, its pin's driver vertex (`pinDrvrVertex`: a bidirect pin's second).
    pin_drvr: Vec<usize>,
    /// Whether the vertex is one of a bidirect pin's two.
    bidir: Vec<bool>,
    funcs: HashMap<String, Option<FuncExpr>>,
    act: Activities,
    bfs: Bfs,
    visited_regs: BTreeSet<usize>,
    max_change: f32,
    max_change_pin: Option<usize>,
}

/// `ensureActivities`: seed the roots, propagate through the combinational logic, then pass
/// through the registers until none of their inputs changes (at most [`MAX_ACTIVITY_PASSES`]).
pub fn ensure_activities(g: &Graph<'_>, inp: &ActivityInputs<'_>) -> Result<Activities, String> {
    let mut p = Prop::new(g, inp)?;
    let input = input_activity(inp.min_period, inp.time_unit);
    p.act.trace.push(format!("VYG-INPUT dens {}", bits(input.density)));
    p.seed_activities(input);
    p.visit_all();
    let mut regs = std::mem::take(&mut p.visited_regs);
    let mut pass = 1;
    while !regs.is_empty() && pass <= MAX_ACTIVITY_PASSES {
        p.max_change = 0.0;
        p.max_change_pin = None;
        for &reg in &regs {
            p.seed_reg_output_activities(reg);
        }
        p.visit_all();
        regs = std::mem::take(&mut p.visited_regs);
        p.act.trace.push(format!("Pass {pass} change {}", bits(p.max_change)));
        pass += 1;
    }
    p.act.passes = pass - 1;
    // a bidirect pin's driver vertex reads the pin's activity
    for v in 0..p.act.pin.len() {
        p.act.pin[v] = p.act.pin[p.pin_key[v]];
    }
    Ok(p.act)
}

impl<'x, 'g, 'a> Prop<'x, 'g, 'a> {
    fn new(g: &'g Graph<'a>, inp: &'x ActivityInputs<'x>) -> Result<Prop<'x, 'g, 'a>, String> {
        let n = g.vertices.len();
        let mut inst_pins = vec![Vec::new(); g.netlist.insts.len()];
        for (v, vx) in g.vertices.iter().enumerate() {
            if let Conn::Inst(i, _) = vx.conn {
                inst_pins[i].push(v);
            }
        }
        let (mut pin_key, mut pin_drvr, mut bidir): (Vec<usize>, Vec<usize>, Vec<bool>) = ((0..n).collect(), (0..n).collect(), vec![false; n]);
        for v in 1..n {
            let (a, b) = (&g.vertices[v - 1], &g.vertices[v]);
            if a.name == b.name && a.conn == b.conn {
                pin_key[v] = v - 1;
                pin_drvr[v - 1] = v;
                bidir[v - 1] = true;
                bidir[v] = true;
            }
        }
        let max_level = inp.level.iter().copied().max().unwrap_or(0);
        Ok(Prop {
            g,
            inp,
            inst_pins,
            pin_key,
            pin_drvr,
            bidir,
            funcs: HashMap::new(),
            act: Activities { pin: vec![None; n], ..Activities::default() },
            bfs: Bfs::new(n, max_level),
            visited_regs: BTreeSet::new(),
            max_change: 0.0,
            max_change_pin: None,
        })
    }

    fn cell_of_inst(&self, i: usize) -> Option<&'a Cell> {
        let cell = &self.g.netlist.insts[i].1;
        self.g.libs.iter().find_map(|l| l.cells.get(cell))
    }

    fn inst_of(&self, v: usize) -> Option<usize> {
        match self.g.vertices[v].conn {
            Conn::Inst(i, _) => Some(i),
            Conn::Port(_) => None,
        }
    }

    /// `Network::isLoad`: an instance input, or a top OUTPUT port — or a bidirect pin or port,
    /// whichever of its vertices (the PIN's direction decides).
    fn is_load(&self, v: usize) -> bool {
        !self.g.vertices[v].is_driver || self.bidir[v]
    }

    /// `Network::isDriver`: an instance output, or a top INPUT port, or a bidirect pin or port.
    fn is_driver(&self, v: usize) -> bool {
        self.g.vertices[v].is_driver || self.bidir[v]
    }

    /// `Vertex::to_string`: the pin's path name, and for a bidirect pin's vertex ` load` or
    /// ` driver`.
    fn label(&self, v: usize) -> String {
        let name = path_name(&self.g.vertices[v].name);
        match (self.bidir[v], self.g.vertices[v].is_driver) {
            (true, true) => format!("{name} driver"),
            (true, false) => format!("{name} load"),
            _ => name,
        }
    }

    /// The liberty function of a port of `cell` (parsed once per cell and port).
    fn func(&mut self, cell: &Cell, port: &str) -> Option<FuncExpr> {
        let key = format!("{}\u{0}{port}", cell.name);
        if let Some(f) = self.funcs.get(&key) {
            return f.clone();
        }
        let f = cell.ports.iter().find(|p| p.name == port).and_then(|p| p.function.as_deref()).and_then(|t| FuncExpr::parse(t).ok());
        self.funcs.insert(key, f.clone());
        f
    }

    /// `ActivitySrchPred::searchThru`: every edge but timing checks, clock-to-Q and latch D->Q.
    fn search_thru(&self, e: usize) -> bool {
        match self.g.edges[e].kind {
            EdgeKind::Wire => true,
            EdgeKind::Gate { set } => !self.g.is_check(e) && !matches!(self.g.arc_set(e, set).role, Role::RegClkToQ | Role::LatchDtoQ) && !self.g.bidirect_inst_paths.contains(&e),
        }
    }

    /// `enqueueAdjacentVertices`: the vertex's fanout through searched edges, NEWEST edge first.
    fn enqueue_adjacent(&mut self, v: usize) {
        for &e in self.g.out_edges[v].iter().rev() {
            if self.search_thru(e) {
                let to = self.g.edges[e].to;
                self.bfs.enqueue(to, self.inp.level[to]);
            }
        }
    }

    /// `seedActivities`: each root in vertex order that is not a clock's pin gets the input
    /// activity, and its fanout is queued.
    fn seed_activities(&mut self, input: Activity) {
        for v in 0..self.g.vertices.len() {
            if !self.is_root(v) || self.inp.clock_source[v] {
                continue;
            }
            self.act.trace.push(format!("seed {}", self.label(v)));
            self.act.pin[self.pin_key[v]] = Some(input);
            // the fanout of the pin's DRIVER vertex
            self.enqueue_adjacent(self.pin_drvr[v]);
        }
    }

    /// `Levelize::isRoot`: no in-edge it levels through (timing checks, latch D->Q, the disabled
    /// set/clear arcs and the bidirect instance and port paths do not count).
    fn is_root(&self, v: usize) -> bool {
        !self.g.in_edges[v].iter().any(|&e| match self.g.edges[e].kind {
            EdgeKind::Wire => !self.g.bidirect_port_paths.contains(&e),
            EdgeKind::Gate { set } => !self.g.is_check(e) && !matches!(self.g.arc_set(e, set).role, Role::RegSetClr | Role::LatchDtoQ) && !self.g.bidirect_inst_paths.contains(&e),
        })
    }

    /// `BfsIterator::visit` to the last level.
    fn visit_all(&mut self) {
        while self.bfs.first <= self.bfs.last && (self.bfs.first as usize) < self.bfs.queue.len() {
            let level = self.bfs.first as usize;
            self.bfs.first += 1;
            while let Some(v) = self.bfs.queue[level].pop() {
                self.bfs.in_queue[v] = false;
                self.visit(v);
            }
        }
    }

    /// `PropActivityVisitor::visit`.
    fn visit(&mut self, v: usize) {
        self.act.trace.push(format!("visit {}", self.label(v)));
        let mut changed = false;
        if self.is_load(v) {
            if let Some(&e) = self.g.in_edges[v].last() {
                if matches!(self.g.edges[e].kind, EdgeKind::Wire) {
                    let from = self.act.pin[self.pin_key[self.g.edges[e].from]].unwrap_or(Activity::UNKNOWN);
                    changed = self.set_activity_check(v, Activity::new(from.density, from.duty, Origin::Propagated));
                }
            }
        }
        let inst = self.inst_of(v);
        if self.is_driver(v) {
            if let (Some(i), Some(port)) = (inst, self.g.vertices[v].port.clone()) {
                if let Some(cell) = self.cell_of_inst(i) {
                    let mut func = self.func(cell, &port);
                    if func.is_none() {
                        if let Some(test) = cell.test_cell.as_deref() {
                            func = self.func(test, &port);
                        }
                    }
                    if let Some(func) = func {
                        let a = eval_activity(&func, &InstView { p: self, inst: i });
                        changed = self.set_activity_check(v, a);
                    }
                    if cell.ports.iter().any(|p| p.name == port && p.is_clock_gate_out) {
                        let [enable, clk, gclk] = self.clock_gate_pins(i);
                        if let (Some(enable), Some(clk), Some(gclk)) = (enable, clk, gclk) {
                            let a1 = self.find_activity(clk);
                            let a2 = self.find_activity(enable);
                            let (p1, p2) = (a1.duty, a2.duty);
                            changed = self.set_activity_check(gclk, Activity::new(a1.density * p2 + a2.density * p1, p1 * p2, Origin::Propagated));
                        }
                    }
                }
            }
        }
        if changed {
            if let Some(i) = inst {
                if let Some(cell) = self.cell_of_inst(i) {
                    if self.is_load(v) {
                        let test_seq = cell.test_cell.as_deref().is_some_and(Cell::is_sequential);
                        if cell.is_sequential() || test_seq {
                            self.visited_regs.insert(i);
                        }
                        if cell.is_clock_gate && self.clock_gate_pins(i)[2].is_some() {
                            self.visited_regs.insert(i);
                        }
                    }
                    self.enqueue_adjacent(v);
                }
            }
        }
    }

    /// `clockGatePins`: the instance's enable, clock and gated clock pins (the last of each).
    fn clock_gate_pins(&self, i: usize) -> [Option<usize>; 3] {
        let mut out = [None; 3];
        let Some(cell) = self.cell_of_inst(i) else { return out };
        for &v in &self.inst_pins[i] {
            let Some(port) = self.g.vertices[v].port.as_deref() else { continue };
            let Some(p) = cell.ports.iter().find(|p| p.name == port) else { continue };
            for (k, on) in [p.is_clock_gate_enable, p.is_clock_gate_clock, p.is_clock_gate_out].into_iter().enumerate() {
                if on {
                    out[k] = Some(v);
                }
            }
        }
        out
    }

    /// `setActivityCheck`: clamp the density to one transition per (least) slew, set it, and
    /// say whether it moved by more than the tolerance or changed origin.
    fn set_activity_check(&mut self, v: usize, mut a: Activity) -> bool {
        // the pin's: `getMinRfSlew` reads its first vertex
        let v = self.pin_key[v];
        let slew = self.inp.min_rf_slew[v];
        let max_density = if slew > 0.0 { (1.0 / f64::from(slew)) as f32 } else { f32::INFINITY };
        if a.density > max_density {
            a.density = max_density;
        }
        let prev = self.act.pin[v].unwrap_or(Activity::UNKNOWN);
        let density_delta = percent_change(a.density, prev.density);
        let duty_delta = percent_change(a.duty, prev.duty);
        if density_delta > self.max_change {
            self.max_change = density_delta;
            self.max_change_pin = Some(v);
        }
        if duty_delta > self.max_change {
            self.max_change = duty_delta;
            self.max_change_pin = Some(v);
        }
        let changed = density_delta > CHANGE_TOLERANCE || duty_delta > CHANGE_TOLERANCE || a.origin != prev.origin;
        self.act.trace.push(format!(
            "VYG-SET {} slew {} dens {} duty {} origin {} changed {}",
            path_name(&self.g.vertices[v].name),
            bits(slew),
            bits(a.density),
            bits(a.duty),
            a.origin as u8,
            u8::from(changed)
        ));
        self.act.pin[v] = Some(a);
        changed
    }

    /// `findActivity`: a clock pin's own activity if one was set, else its clock's; any other
    /// pin's activity if set, else unknown.
    fn find_activity(&self, v: usize) -> Activity {
        let v = self.pin_key[v];
        if self.inp.is_clock[v] {
            if let Some(a) = self.act.pin[v].filter(Activity::is_set) {
                return a;
            }
            return self.inp.clock[v].unwrap_or(Activity::UNKNOWN);
        }
        self.act.pin[v].filter(Activity::is_set).unwrap_or(Activity::UNKNOWN)
    }

    /// `seedRegOutputActivities` (one instance): the cell's registers, else its test cell's,
    /// else a clock gate's output is queued.
    fn seed_reg_output_activities(&mut self, i: usize) {
        let Some(cell) = self.cell_of_inst(i) else { return };
        if !cell.seqs.is_empty() {
            self.seed_reg_seqs(i, cell, None);
        } else if let Some(test) = cell.test_cell.as_deref() {
            self.seed_reg_seqs(i, test, Some(test));
        } else if cell.is_clock_gate {
            for v in self.inst_pins[i].clone() {
                let port = self.g.vertices[v].port.as_deref().unwrap_or("");
                if cell.ports.iter().any(|p| p.name == port && p.is_clock_gate_out) {
                    self.bfs.enqueue(v, self.inp.level[v]);
                }
            }
        }
    }

    /// Each register of `seq_cell`: its outputs' activities, then every pin whose function is
    /// one of them is queued (ports read from the test cell when there is one).
    fn seed_reg_seqs(&mut self, i: usize, seq_cell: &'a Cell, test_cell: Option<&'a Cell>) {
        for seq in &seq_cell.seqs {
            self.seed_reg_output(i, seq, seq.output.as_deref(), false);
            self.seed_reg_output(i, seq, seq.output_inv.as_deref(), true);
            for v in self.inst_pins[i].clone() {
                let Some(port) = self.g.vertices[v].port.clone() else { continue };
                let cell = match test_cell {
                    Some(t) => t,
                    None => match self.cell_of_inst(i) {
                        Some(c) => c,
                        None => continue,
                    },
                };
                if !cell.ports.iter().any(|p| p.name == port) {
                    continue;
                }
                if let Some(FuncExpr::Port(fp)) = self.func(cell, &port) {
                    if Some(&fp) == seq.output.as_ref() || Some(&fp) == seq.output_inv.as_ref() {
                        self.act.trace.push(format!("enqueue reg output {}", path_name(&self.g.vertices[v].name)));
                        self.bfs.enqueue(v, self.inp.level[v]);
                    }
                }
            }
        }
    }

    /// One register output: the data's activity through the register — a register samples at
    /// most once per clock edge pair, so when the data toggles faster than half the clock its
    /// density is 2·duty·(1 − duty)·clock density (float); a latch passes the data while open.
    /// A missing output (an `ff` naming one) is still evaluated and kept under no port.
    fn seed_reg_output(&mut self, i: usize, seq: &crate::liberty::Seq, output: Option<&str>, invert: bool) {
        let view = InstView { p: self, inst: i };
        let parse = |t: &Option<String>| t.as_deref().and_then(|t| FuncExpr::parse(t).ok());
        let in_a = parse(&seq.data).map_or(Activity::UNKNOWN, |f| eval_activity(&f, &view));
        let clk_func = parse(&seq.clock);
        let clk = clk_func.as_ref().map_or(Activity::UNKNOWN, |f| eval_activity(f, &view));
        let (in_density, in_duty) = (in_a.density, in_a.duty);
        let mut out_density = in_density;
        let mut out_duty = in_duty;
        let clk_density = clk.density;
        if in_density > clk_density / 2.0 {
            if seq.is_register {
                out_density = 2.0 * in_duty * (1.0 - in_duty) * clk_density;
            } else {
                let clk_invert = matches!(&clk_func, Some(FuncExpr::Not(x)) if matches!(**x, FuncExpr::Port(_)));
                out_density = if clk_invert { in_density * (1.0 - clk.duty) } else { in_density * clk.duty };
            }
        }
        if invert {
            out_duty = (1.0 - f64::from(out_duty)) as f32;
        }
        let out = Activity::new(out_density, out_duty, Origin::Propagated);
        self.act.trace.push(format!(
            "VYG-SEQ {} {} in {} {} clk {} out {} {}",
            path_name(&self.g.netlist.insts[i].0),
            output.unwrap_or("-"),
            bits(in_density),
            bits(in_duty),
            bits(clk_density),
            bits(out_density),
            bits(out_duty)
        ));
        if let Some(output) = output {
            self.act.seq.insert((i, output.to_string()), out);
        }
    }
}

/// An INTERNAL port of the cell: one declared `direction : internal`, or a register's state
/// variable (`ff (IQ, IQN)`) the cell declares no pin for — the reader makes those internal.
pub fn is_internal_port(cell: &Cell, port: &str) -> bool {
    match cell.ports.iter().find(|p| p.name == port) {
        Some(p) => p.direction == Direction::Internal,
        None => cell.seqs.iter().any(|s| s.output.as_deref() == Some(port) || s.output_inv.as_deref() == Some(port)),
    }
}

/// One instance's power (`PowerResult`): each term summed in float as its pins are visited.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct InstPower {
    pub internal: f32,
    pub switching: f32,
    pub leakage: f32,
}

impl InstPower {
    /// `PowerResult::total`: internal + switching + leakage, in float.
    pub fn total(&self) -> f32 {
        self.internal + self.switching + self.leakage
    }
}

/// What the instance powers read besides the activities, for the analyzed scene (its graph is
/// the one passed: its libraries are the scene's cells, its slews and loads the scene's).
pub struct PowerInputs<'x> {
    /// `Sim` constants by pin name: what a leakage `when` is evaluated under.
    pub constants: &'x HashMap<String, bool>,
    /// Per vertex: in an ideal clock's network, the clock slew per `[rise, fall]`
    /// (`idealClkSlew`, the clocks' `set_clock_transition`, 0 without).
    pub ideal_clock_slew: &'x [Option<[f32; 2]>],
    /// Per vertex: `loadCap(pin, scene, max)` of a driver.
    pub load_cap: &'x [f32],
    /// The libraries in read order: an instance's LINK cell is the first one's of its name. The
    /// network's ports (directions, functions, related power pins) and the cell leakage are the
    /// link cell's; the power groups, the arcs' senses and the supply voltages the scene's.
    pub link_libs: &'x [crate::liberty::Library],
}

/// `Power::findInstPowers`: per instance (`None` without a liberty cell), in instance order,
/// `findInternalPower`, `findSwitchingPower`, then `findLeakagePower`, over the activities
/// `ensure_activities` found on the same graph.
pub fn inst_powers(g: &Graph<'_>, inp: &ActivityInputs<'_>, act: &Activities, pw: &PowerInputs<'_>) -> Result<Vec<Option<InstPower>>, String> {
    let mut p = Prop::new(g, inp)?;
    p.act = act.clone();
    let mut out = Vec::with_capacity(g.netlist.insts.len());
    for i in 0..g.netlist.insts.len() {
        let Some(cell) = p.cell_of_inst(i) else {
            out.push(None);
            continue;
        };
        let link = pw.link_libs.iter().find_map(|l| l.cells.get(&cell.name)).unwrap_or(cell);
        let mut r = InstPower::default();
        p.find_internal_power(i, cell, link, pw, &mut r);
        p.find_switching_power(i, link, pw, &mut r);
        p.find_leakage_power(i, cell, link, pw, &mut r);
        out.push(Some(r));
    }
    Ok(out)
}

fn is_any_output(d: Direction) -> bool {
    matches!(d, Direction::Output | Direction::Tristate | Direction::Bidirect)
}

fn is_any_input(d: Direction) -> bool {
    matches!(d, Direction::Input | Direction::Bidirect)
}

/// `findExprOutPort`: the first port the expression names (left before right) that is an output.
fn find_expr_out_port<'c>(expr: &'c FuncExpr, cell: &Cell) -> Option<&'c str> {
    match expr {
        FuncExpr::Port(p) => cell.port(p).filter(|x| is_any_output(x.direction)).map(|_| p.as_str()),
        FuncExpr::Not(e) => find_expr_out_port(e, cell),
        FuncExpr::And(l, r) | FuncExpr::Or(l, r) | FuncExpr::Xor(l, r) => find_expr_out_port(l, cell).or_else(|| find_expr_out_port(r, cell)),
        FuncExpr::One | FuncExpr::Zero => None,
    }
}

fn has_port(expr: &FuncExpr, port: &str) -> bool {
    match expr {
        FuncExpr::Port(p) => p == port,
        FuncExpr::Not(e) => has_port(e, port),
        FuncExpr::And(l, r) | FuncExpr::Or(l, r) | FuncExpr::Xor(l, r) => has_port(l, port) || has_port(r, port),
        FuncExpr::One | FuncExpr::Zero => false,
    }
}

/// `InternalPowerModel::power`: the table at the axes it names (input transition, output load).
fn internal_energy(model: Option<&crate::table::Table>, slew: f32, load_cap: f32) -> f32 {
    let Some(t) = model else { return 0.0 };
    let pick = |a: usize| match t.axes.get(a).map(|x| x.var) {
        Some(crate::table::AxisVar::InputTransitionTime) => slew,
        Some(crate::table::AxisVar::TotalOutputNetCapacitance) => load_cap,
        _ => 0.0,
    };
    t.find_value(pick(0), pick(1), pick(2))
}

impl<'x, 'g, 'a> Prop<'x, 'g, 'a> {
    /// The instance's pins in network order, one vertex each (a bidirect pin's first).
    fn pins_of(&self, i: usize) -> Vec<usize> {
        self.inst_pins[i].iter().copied().filter(|&v| self.pin_key[v] == v).collect()
    }

    fn port_of(&self, v: usize, cell: &'a Cell) -> Option<&'a crate::liberty::Port> {
        self.g.vertices[v].port.as_deref().and_then(|p| cell.port(p))
    }

    fn pin_on(&self, i: usize, port: &str) -> Option<usize> {
        self.inst_pins[i].iter().copied().find(|&v| self.g.vertices[v].port.as_deref() == Some(port))
    }

    /// `getSlew(vertex, rf, scene)`: an ideal clock pin's clock slew, else the max slew.
    fn get_slew(&self, v: usize, rf: usize, pw: &PowerInputs<'_>) -> f32 {
        match pw.ideal_clock_slew[v] {
            Some(s) => s[rf],
            None => self.g.slew[v][rf][crate::liberty::MAX],
        }
    }

    fn eval_activity_on(&self, expr: &FuncExpr, i: usize) -> Activity {
        eval_activity(expr, &InstView { p: self, inst: i })
    }

    /// `findInternalPower`: per pin with a (link) liberty port, an output's then an input's.
    fn find_internal_power(&mut self, i: usize, cell: &'a Cell, link: &'a Cell, pw: &PowerInputs<'_>, r: &mut InstPower) {
        for v in self.pins_of(i) {
            let Some(port) = self.port_of(v, link) else { continue };
            let out = is_any_output(port.direction);
            let load_cap = if out { pw.load_cap[v] } else { 0.0 };
            let activity = self.find_activity(v);
            if out {
                self.find_output_internal_power(i, cell, port, activity, load_cap, pw, r);
            }
            if is_any_input(port.direction) {
                self.find_input_internal_power(i, v, cell, link, port, activity, load_cap, pw, r);
            }
        }
    }

    /// `findInputInternalPower`: each group on the port, its energy averaged over the
    /// transitions × its `when` duty (the output's Boolean difference when the output's function
    /// names the port) × the pin's density.
    #[allow(clippy::too_many_arguments)]
    fn find_input_internal_power(&mut self, i: usize, v: usize, cell: &'a Cell, link: &Cell, port: &crate::liberty::Port, activity: Activity, load_cap: f32, pw: &PowerInputs<'_>, r: &mut InstPower) {
        let pwrs: Vec<&crate::liberty::InternalPower> = cell.internal_powers.iter().filter(|ip| ip.port == port.name).collect();
        if pwrs.is_empty() {
            return;
        }
        let mut internal = 0.0f32;
        for pwr in pwrs {
            let mut energy = 0.0f32;
            let mut rf_count = 0;
            for rf in [crate::liberty::RISE, crate::liberty::FALL] {
                let slew = self.get_slew(v, rf, pw);
                if slew < 1e30 {
                    energy += internal_energy(pwr.models[rf].as_ref(), slew, load_cap);
                    rf_count += 1;
                }
            }
            if rf_count > 0 {
                energy /= rf_count as f32;
            }
            let mut duty = 1.0f32;
            if let Some(when) = pwr.when.as_deref().and_then(|w| FuncExpr::parse(w).ok()) {
                // the scene port the `when` names, then the link cell's port of that name
                match find_expr_out_port(&when, cell) {
                    Some(out) => {
                        if let Some(out_port) = link.port(out) {
                            match out_port.function.as_deref().and_then(|f| FuncExpr::parse(f).ok()) {
                                Some(func) if has_port(&func, &port.name) => duty = eval_diff_duty(&func, &port.name, &InstView { p: self, inst: i }),
                                _ => duty = self.eval_activity_on(&when, i).duty,
                            }
                        }
                    }
                    None => duty = self.eval_activity_on(&when, i).duty,
                }
            }
            internal += energy * duty * activity.density;
        }
        r.internal += internal;
    }

    /// `findInputDuty`: the related pin's Boolean-difference duty through the output's function
    /// when it names the pin, else the `when` duty, else 0.5; 0 without a related pin.
    fn find_input_duty(&self, i: usize, func: Option<&FuncExpr>, pwr: &crate::liberty::InternalPower) -> f32 {
        let Some(from) = pwr.related.as_deref() else { return 0.0 };
        if self.pin_on(i, from).is_none() {
            return 0.0;
        }
        if let Some(f) = func.filter(|f| has_port(f, from)) {
            return eval_diff_duty(f, from, &InstView { p: self, inst: i });
        }
        match pwr.when.as_deref().and_then(|w| FuncExpr::parse(w).ok()) {
            Some(when) => self.eval_activity_on(&when, i).duty,
            None => 0.5,
        }
    }

    /// `findOutputInternalPower`: each group's energy (the related pin's slew on the transition
    /// its unateness gives) weighted by its share of the related pins' density × duty for its pg
    /// pin, × the output's density.
    #[allow(clippy::too_many_arguments)]
    fn find_output_internal_power(&mut self, i: usize, cell: &'a Cell, to_port: &crate::liberty::Port, to_activity: Activity, load_cap: f32, pw: &PowerInputs<'_>, r: &mut InstPower) {
        let func = to_port.function.as_deref().and_then(|f| FuncExpr::parse(f).ok());
        let pwrs: Vec<&crate::liberty::InternalPower> = cell.internal_powers.iter().filter(|ip| ip.port == to_port.name).collect();
        // per related pg pin (none is a key of its own)
        let mut pg_duty_sum: Vec<(Option<&str>, f32)> = Vec::new();
        for pwr in &pwrs {
            let Some(from) = pwr.related.as_deref() else { continue };
            let from_density = self.pin_on(i, from).map_or(0.0, |fv| self.find_activity(fv).density);
            let duty = self.find_input_duty(i, func.as_ref(), pwr);
            let key = pwr.related_pg_pin.as_deref();
            match pg_duty_sum.iter_mut().find(|(k, _)| *k == key) {
                Some((_, s)) => *s += from_density * duty,
                None => pg_duty_sum.push((key, from_density * duty)),
            }
        }
        let mut internal = 0.0f32;
        for pwr in &pwrs {
            let duty = self.find_input_duty(i, func.as_ref(), pwr);
            let from_pin = pwr.related.as_deref().and_then(|f| self.pin_on(i, f));
            let positive_unate = pwr.related.is_none() || pwr.positive_unate;
            let mut energy = 0.0f32;
            let mut rf_count = 0;
            for to_rf in [crate::liberty::RISE, crate::liberty::FALL] {
                let from_rf = if positive_unate { to_rf } else { 1 - to_rf };
                let slew = from_pin.map_or(0.0, |fv| self.get_slew(self.pin_key[fv], from_rf, pw));
                if slew < 1e30 {
                    energy += internal_energy(pwr.models[to_rf].as_ref(), slew, load_cap);
                    rf_count += 1;
                }
            }
            if rf_count > 0 {
                energy /= rf_count as f32;
            }
            let mut weight = 0.0f32;
            if let Some(&(_, duty_sum)) = pg_duty_sum.iter().find(|(k, _)| *k == pwr.related_pg_pin.as_deref()) {
                if duty_sum != 0.0 {
                    if let Some(fv) = from_pin {
                        let from_density = self.find_activity(fv).density;
                        weight = from_density * duty / duty_sum;
                    }
                }
            }
            internal += weight * energy * to_activity.density;
        }
        r.internal += internal;
    }

    /// `findSwitchingPower`: each output pin's `0.5 × load × volt² × density`, in double.
    fn find_switching_power(&mut self, i: usize, link: &'a Cell, pw: &PowerInputs<'_>, r: &mut InstPower) {
        for v in self.pins_of(i) {
            let Some(port) = self.port_of(v, link) else { continue };
            if !is_any_output(port.direction) {
                continue;
            }
            let load_cap = pw.load_cap[v];
            let activity = self.find_activity(v);
            let volt = self.port_voltage(link, port);
            let switching = 0.5 * f64::from(load_cap) * f64::from(volt) * f64::from(volt) * f64::from(activity.density);
            r.switching += switching as f32;
        }
    }

    /// `portVoltage` / `pgPortVoltage`: the (link) port's related power pin's voltage name, in the
    /// scene cell's library's `voltage_map`; else that library's default operating conditions'
    /// voltage; else 0.
    fn port_voltage(&self, link: &Cell, port: &crate::liberty::Port) -> f32 {
        let lib = self.g.libs.iter().find(|l| l.cells.contains_key(&link.name));
        if let (Some(lib), Some(pg)) = (lib, port.related_power_pin.as_deref()) {
            if link.pg_pins.iter().any(|(n, _)| n == pg) {
                if let Some(v) = link.pg_voltage_names.get(pg).and_then(|name| lib.supply_voltages.get(name)) {
                    return *v;
                }
            }
        }
        lib.and_then(|l| l.default_op_voltage).unwrap_or(0.0)
    }

    /// `Sim::evalExpr(when, inst) == one`: the expression's diagram with the instance's constant
    /// pins substituted is the constant one.
    fn sim_is_one(&self, expr: &FuncExpr, i: usize, pw: &PowerInputs<'_>) -> bool {
        let (mut b, mut f) = Bdd::of(expr);
        for &v in &self.inst_pins[i] {
            let Some(port) = self.g.vertices[v].port.as_deref() else { continue };
            let Some(var) = b.var_index(port) else { continue };
            if let Some(&value) = pw.constants.get(&self.g.vertices[v].name) {
                f = b.restrict(f, var, value);
            }
        }
        f == bdd::ONE
    }

    /// `findLeakagePower`: per related primary power pin (none first, then pg pin order), a
    /// `when` true under the constants gives its leakage; otherwise the `when` groups weighted by
    /// duty plus the cell leakage over the rest of the duty; an unconditional group only without
    /// conditional ones. No group at all: the cell leakage.
    fn find_leakage_power(&mut self, i: usize, cell: &'a Cell, link: &Cell, pw: &PowerInputs<'_>, r: &mut InstPower) {
        #[derive(Default)]
        struct Sum {
            cond_exists: bool,
            cond_leakage: f32,
            cond_duty_sum: f32,
            cond_true_exists: bool,
            cond_true_leakage: f32,
            uncond_exists: bool,
            uncond_leakage: f32,
        }
        // keyed by the pg port: none, then the cell's pg pins in order
        let rank = |pg: Option<&str>| pg.map_or(0, |n| 1 + cell.pg_pins.iter().position(|(p, _)| p == n).unwrap_or(usize::MAX - 1));
        let mut sums: Vec<(usize, Sum)> = Vec::new();
        for lp in &cell.leakage_groups {
            let pg = lp.related_pg_pin.as_deref().filter(|n| cell.pg_pins.iter().any(|(p, _)| p == n));
            if let Some(n) = pg {
                if cell.pg_pins.iter().find(|(p, _)| p == n).is_none_or(|(_, t)| t != "primary_power") {
                    continue;
                }
            }
            let key = rank(pg);
            let k = match sums.iter().position(|(r, _)| *r == key) {
                Some(k) => k,
                None => {
                    sums.push((key, Sum::default()));
                    sums.len() - 1
                }
            };
            let leakage = lp.power;
            match lp.when.as_deref().and_then(|w| FuncExpr::parse(w).ok()) {
                Some(when) => {
                    if self.sim_is_one(&when, i, pw) {
                        sums[k].1.cond_true_leakage = leakage;
                        sums[k].1.cond_true_exists = true;
                    } else {
                        let cond_duty = self.eval_activity_on(&when, i).duty;
                        let sum = &mut sums[k].1;
                        sum.cond_leakage += leakage * cond_duty;
                        if leakage > 0.0 {
                            sum.cond_duty_sum += cond_duty;
                        }
                        sum.cond_exists = true;
                    }
                }
                None => {
                    sums[k].1.uncond_leakage = leakage;
                    sums[k].1.uncond_exists = true;
                }
            }
        }
        // `cell->leakagePower`: the LINK cell's, not the scene's (a corner read from another
        // library keeps the link library's cell leakage)
        let (cell_leakage, cell_leakage_exists) = (link.leakage_power.unwrap_or(0.0), link.leakage_power.is_some());
        if sums.is_empty() {
            r.leakage += cell_leakage;
            return;
        }
        sums.sort_by_key(|(k, _)| *k);
        for (_, sum) in &sums {
            let mut leakage = 0.0f32;
            if sum.cond_true_exists {
                leakage = sum.cond_true_leakage;
            } else if sum.cond_exists {
                leakage = sum.cond_leakage;
                if cell_leakage_exists {
                    let duty = (1.0 - f64::from(sum.cond_duty_sum)) as f32;
                    leakage += cell_leakage * duty;
                }
            } else if sum.uncond_exists {
                leakage = sum.uncond_leakage;
            }
            r.leakage += leakage;
        }
    }
}

/// One instance's pins as a function's evaluation reads them.
struct InstView<'p, 'x, 'g, 'a> {
    p: &'p Prop<'x, 'g, 'a>,
    inst: usize,
}

impl InstView<'_, '_, '_, '_> {
    fn pin(&self, port: &str) -> Option<usize> {
        self.p.inst_pins[self.inst].iter().copied().find(|&v| self.p.g.vertices[v].port.as_deref() == Some(port))
    }
}

impl InstPins for InstView<'_, '_, '_, '_> {
    fn is_internal(&self, port: &str) -> bool {
        let Some(cell) = self.p.cell_of_inst(self.inst) else { return false };
        is_internal_port(cell, port) || cell.test_cell.as_deref().is_some_and(|t| is_internal_port(t, port))
    }
    fn pin_activity(&self, port: &str) -> Option<Activity> {
        self.pin(port).map(|v| self.p.find_activity(v))
    }
    fn seq_activity(&self, port: &str) -> Activity {
        self.p.act.seq.get(&(self.inst, port.to_string())).copied().unwrap_or(Activity::UNKNOWN)
    }
}

#[cfg(test)]
mod tests {
    // The reference's trace prints network path names: the database's escapes are dropped, an
    // escaped backslash keeps one.
    #[test]
    fn trace_names_drop_the_database_escapes() {
        assert_eq!(super::path_name(r"ctrl\/state\/out\[1\]$_DFF_P__2/Y"), "ctrl/state/out[1]$_DFF_P__2/Y");
        assert_eq!(super::path_name(r"a\\b/Y"), r"a\b/Y");
        assert_eq!(super::path_name("_896_/QN"), "_896_/QN");
    }

    use super::*;
    use std::collections::HashMap;

    struct Pins {
        act: HashMap<&'static str, Activity>,
        internal: HashMap<&'static str, Activity>,
    }

    impl InstPins for Pins {
        fn is_internal(&self, port: &str) -> bool {
            self.internal.contains_key(port)
        }
        fn pin_activity(&self, port: &str) -> Option<Activity> {
            self.act.get(port).copied()
        }
        fn seq_activity(&self, port: &str) -> Activity {
            self.internal.get(port).copied().unwrap_or(Activity::UNKNOWN)
        }
    }

    fn pins(act: &[(&'static str, f32, f32)]) -> Pins {
        Pins { act: act.iter().map(|&(p, d, u)| (p, Activity::new(d, u, Origin::Input))).collect(), internal: HashMap::new() }
    }

    fn f(s: &str) -> FuncExpr {
        FuncExpr::parse(s).unwrap()
    }

    // AND: duty pA*pB; density dA*pB + dB*pA (each port weighted by the other being sensitizing).
    #[test]
    fn an_and_gate() {
        let p = pins(&[("A", 1e7, 0.5), ("B", 2e7, 0.25)]);
        let a = eval_activity(&f("A & B"), &p);
        assert_eq!((a.duty, a.density, a.origin), (0.125, 1e7 * 0.25 + 2e7 * 0.5, Origin::Propagated));
    }

    // An inverter's diagram is the complemented variable: duty 1 - p, computed in double.
    #[test]
    fn an_inverter_complements_in_double() {
        let p = pins(&[("A", 1e7, 0.3)]);
        let a = eval_activity(&f("!A"), &p);
        let inner = (0.0f64 * (1.0 - f64::from(0.3f32)) + 1.0 * f64::from(0.3f32)) as f32;
        assert_eq!(a.duty, (1.0 - f64::from(inner)) as f32);
        assert_eq!(a.density, 1e7);
    }

    // The density's terms are summed in NAME order, not variable order: B's term first here, and
    // the float sum is order sensitive.
    #[test]
    fn density_terms_sum_in_name_order() {
        // A, B then C: 1 + 1 + 2^24 = 2^24 + 2; in variable order C first, each 1 is lost.
        let p = pins(&[("C", 16_777_216.0, 0.5), ("A", 1.0, 0.5), ("B", 1.0, 0.5)]);
        let a = eval_activity(&f("C ^ A ^ B"), &p);
        assert_eq!(a.density, 16_777_218.0);
    }

    // The duty step: else-duty × (1 − p) in double, then-duty × p in FLOAT, summed in double.
    // Witnessed (gcd, an XOR2's output, register pass 1): these duties give 0x3f002a51; the step
    // all in double gives 0x3f002a52, and all in float differs again.
    #[test]
    fn the_duty_step_mixes_double_and_float() {
        let (pa, pb) = (f32::from_bits(0x3efc_fa30), f32::from_bits(0x3f0e_0000));
        let p = pins(&[("A", 1e7, pa), ("B", 1e7, pb)]);
        assert_eq!(eval_activity(&f("A ^ B"), &p).duty.to_bits(), 0x3f00_2a51);
    }

    // The mixed step against the all-double one on an OR: A | B is A ? 1 : B.
    #[test]
    fn the_then_product_is_float() {
        let (pa, pb) = (f32::from_bits(0x3efd_a9ab), f32::from_bits(0x3ee6_23b1));
        let p = pins(&[("A", 1e7, pa), ("B", 1e7, pb)]);
        let want = (f64::from(pb) * (1.0 - f64::from(pa)) + f64::from(1.0f32 * pa)) as f32;
        assert_eq!(eval_activity(&f("A | B"), &p).duty, want);
    }

    // A port with no pin on the instance contributes no density, and its node's duty is 0.
    #[test]
    fn a_port_without_a_pin_is_zero() {
        let p = pins(&[("A", 1e7, 0.5)]);
        let a = eval_activity(&f("A | B"), &p);
        assert_eq!(a.density, 0.0 + 1e7 * 0.0);
        // The diagram: A ? 1 : B. B's node is 0 (no pin), so duty = 0 * 0.5 + 1 * 0.5.
        assert_eq!(a.duty, 0.5);
    }

    // A function that is one internal port is the register pass's activity, origin and all.
    #[test]
    fn an_internal_port_is_the_register_activity() {
        let mut p = pins(&[]);
        let iq = Activity::new(3e6, 0.4, Origin::Propagated);
        p.internal.insert("IQ", iq);
        assert_eq!(eval_activity(&f("IQ"), &p), iq);
        // In a diagram, its node is its duty with NO complement: !IQ still reads 0.4.
        assert_eq!(eval_activity(&f("!IQ"), &p).duty, 0.4);
    }

    // A density under 1e-10 is clipped to 0 when the activity is made.
    // A register's state variables (`ff (IQ, IQN)`) are internal ports though no pin declares
    // them; a declared pin is internal only by its direction.
    #[test]
    fn state_variables_are_internal_ports() {
        let lib = crate::liberty::Library::read(
            &crate::liberty_parse::parse(
                "library (l) { cell (dff) { ff (IQ, IQN) { next_state : \"D\"; clocked_on : \"CK\"; }
                 pin (D) { direction : input; } pin (CK) { direction : input; clock : true; }
                 pin (Q) { direction : output; function : \"IQ\"; } pin (QN) { direction : output; function : \"IQN\"; } } }",
            )
            .unwrap(),
        )
        .unwrap();
        let c = &lib.cells["dff"];
        assert!(is_internal_port(c, "IQ") && is_internal_port(c, "IQN"));
        assert!(!is_internal_port(c, "Q") && !is_internal_port(c, "D") && !is_internal_port(c, "X"));
    }

    #[test]
    fn tiny_densities_clip() {
        assert_eq!(Activity::new(9e-11, 0.5, Origin::Input).density, 0.0);
        assert_eq!(Activity::new(-9e-11, 0.5, Origin::Input).density, 0.0);
        assert_eq!(Activity::new(1e-10, 0.5, Origin::Input).density, 1e-10);
    }

    // The sensitizing duty of A in A&B is B's duty.
    #[test]
    fn the_diff_duty_is_the_sensitizing_probability() {
        let p = pins(&[("A", 1e7, 0.5), ("B", 2e7, 0.25)]);
        assert_eq!(eval_diff_duty(&f("A & B"), "A", &p), 0.25);
    }
}
