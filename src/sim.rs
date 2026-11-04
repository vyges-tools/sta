// SPDX-License-Identifier: Apache-2.0
//! The logic simulation the reference runs before it times (`Sim::ensureConstantsPropagated`, not
//! incremental): which pins hold a logic 0 or 1 whatever the rest of the design does. The search
//! and delay calculation read the result as [`crate::graph::SdcEnv::constants`].
//!
//! One function per stage, in the reference's call order:
//! - [`constants`] — `clearSimValues`; `seedConstants`: the network's constant pins (every pin of
//!   every constant net, the zero nets' first), then the `set_logic_*` pins, then the
//!   `set_case_analysis` pins, then the outputs whose function is the constant 0 or 1 (tie cells);
//!   then `propagateConstants(false)`;
//! - [`Sim::set_pin_value`] — a pin a constraint names keeps the constraint's value; a changed
//!   value on an instance INPUT queues the instance (not twice in a row); on a DRIVER it is set on
//!   every load of its net;
//! - [`Sim::eval_instance`] — each output in the cell's port order: a tristate's enable (the
//!   inverse of `three_state`) first and its function only when the enable is 1; otherwise its
//!   function with the known pin values substituted (sequential outputs are NOT looked through);
//!   with no function, a clock-gate output is 0 when its clock or enable pin is.
//!
//! A pin's value only goes from unknown to a constant here unless two seeds disagree, so the fixed
//! point does not depend on the queue's order.
//!
//! Which nets are constant is the reader's (`VerilogReader`): a `supply0` / `supply1` net, and the
//! nets named `zero_` and `one_` — the names it binds every `1'b0` / `1'b1` connection to, so a
//! design's own net of that name is constant too.

use std::collections::{HashMap, VecDeque};

use crate::func_expr::FuncExpr;
use crate::liberty::{Cell, Direction, Library};
use crate::netlist::{Conn, Netlist, PortDir};

/// The net the reader binds every `1'b0` connection to (`zero_net_name_`).
pub const ZERO_NET: &str = "zero_";
/// The net the reader binds every `1'b1` connection to (`one_net_name_`).
pub const ONE_NET: &str = "one_";

/// A known simulation value (`LogicValue` but unknown): a constant, or the one transition a
/// `set_case_analysis rising` / `falling` lets through.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Logic {
    Zero,
    One,
    Rise,
    Fall,
}

impl Logic {
    fn constant(self) -> Option<bool> {
        match self {
            Logic::Zero => Some(false),
            Logic::One => Some(true),
            Logic::Rise | Logic::Fall => None,
        }
    }

    fn of(b: bool) -> Logic {
        if b {
            Logic::One
        } else {
            Logic::Zero
        }
    }
}

/// What the simulation found: the constant pins, and the pins a rising / falling case value
/// holds (`searchThruSimEdge`: only that transition is searched from or to them), by name.
#[derive(Debug, Clone, Default)]
pub struct SimValues {
    pub constants: HashMap<String, bool>,
    /// pin → the one transition searched there (`RISE` / `FALL`)
    pub transitions: HashMap<String, usize>,
    /// Reference warning 1521: a propagated value that differs from a pin's constraint.
    pub warnings: Vec<String>,
}

/// A pin: `(instance, port index in its cell)` or a top-level port.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Pin {
    Inst(usize, usize),
    Port(usize),
}

struct Sim<'a> {
    nl: &'a Netlist,
    cells: Vec<&'a Cell>,
    /// Each pin's net (`None`: unconnected).
    net_of: HashMap<Pin, usize>,
    /// Each net's pins, as the net lists them.
    pins_of: Vec<Vec<Pin>>,
    values: HashMap<Pin, Logic>,
    /// The value each `set_case_analysis` / `set_logic_*` pin is held at; `None` for a
    /// don't-care, which holds nothing but still keeps a propagated value off the pin.
    constraints: HashMap<Pin, Option<Logic>>,
    queue: VecDeque<usize>,
    /// Reference warning 1521: a propagated value that differs from a pin's constraint.
    pub warnings: Vec<String>,
}

fn any_input(d: Direction) -> bool {
    matches!(d, Direction::Input | Direction::Bidirect)
}

fn any_output(d: Direction) -> bool {
    matches!(d, Direction::Output | Direction::Tristate | Direction::Bidirect)
}

impl<'a> Sim<'a> {
    fn new(libs: &'a [Library], nl: &'a Netlist) -> Result<Sim<'a>, String> {
        let mut cells = Vec::with_capacity(nl.insts.len());
        for x in &nl.insts {
            cells.push(libs.iter().find_map(|l| l.cells.get(&x.1)).ok_or_else(|| format!("instance {}: cell {} is in no library read", x.0, x.1))?);
        }
        let mut net_of = HashMap::new();
        let mut pins_of = Vec::with_capacity(nl.nets.len());
        for (k, net) in nl.nets.iter().enumerate() {
            let mut pins = Vec::with_capacity(net.pins.len());
            for c in &net.pins {
                let p = match c {
                    Conn::Inst(i, port) => Pin::Inst(*i, cells[*i].ports.iter().position(|q| &q.name == port).ok_or_else(|| format!("instance {}: cell {} has no port {port}", nl.insts[*i].0, nl.insts[*i].1))?),
                    Conn::Port(i) => Pin::Port(*i),
                };
                net_of.insert(p, k);
                pins.push(p);
            }
            pins_of.push(pins);
        }
        Ok(Sim { nl, cells, net_of, pins_of, values: HashMap::new(), constraints: HashMap::new(), queue: VecDeque::new(), warnings: Vec::new() })
    }

    fn name(&self, p: Pin) -> String {
        match p {
            Pin::Inst(i, k) => format!("{}/{}", self.nl.insts[i].0, self.cells[i].ports[k].name),
            Pin::Port(i) => self.nl.ports[i].0.clone(),
        }
    }

    /// `Network::isDriver`: an instance output, or a top-level input.
    fn is_driver(&self, p: Pin) -> bool {
        match p {
            Pin::Inst(i, k) => any_output(self.cells[i].ports[k].direction),
            Pin::Port(i) => matches!(self.nl.ports[i].1, PortDir::Input | PortDir::Inout),
        }
    }

    /// `Network::isLoad`: an instance input, or a top-level output.
    fn is_load(&self, p: Pin) -> bool {
        match p {
            Pin::Inst(i, k) => any_input(self.cells[i].ports[k].direction),
            Pin::Port(i) => matches!(self.nl.ports[i].1, PortDir::Output | PortDir::Inout),
        }
    }

    /// `Sim::setPinValue`. `None` is unknown.
    fn set_pin_value(&mut self, p: Pin, value: Option<Logic>) {
        if let Some(&held) = self.constraints.get(&p) {
            if value != held {
                if let Some(v) = value {
                    let s = |b: Option<Logic>| match b {
                        None => "X",
                        Some(Logic::Zero) => "0",
                        Some(Logic::One) => "1",
                        Some(Logic::Rise) => "rise",
                        Some(Logic::Fall) => "fall",
                    };
                    self.warnings.push(format!("propagated logic value {} differs from constraint value of {} on pin {}", s(Some(v)), s(held), self.name(p)));
                }
                return;
            }
        }
        if self.values.get(&p).copied() == value {
            return;
        }
        match value {
            Some(v) => self.values.insert(p, v),
            None => self.values.remove(&p),
        };
        match p {
            Pin::Inst(i, k) if any_input(self.cells[i].ports[k].direction) => {
                if self.queue.back() != Some(&i) {
                    self.queue.push_back(i);
                }
            }
            _ if self.is_driver(p) => {
                if let Some(&n) = self.net_of.get(&p) {
                    let loads: Vec<Pin> = self.pins_of[n].iter().copied().filter(|&q| q != p && self.is_load(q)).collect();
                    for q in loads {
                        self.set_pin_value(q, value);
                    }
                }
            }
            _ => {}
        }
    }

    /// `Sim::evalExpr` with the instance's known pin values substituted: 0 or 1 when the function
    /// is that constant under them, else unknown.
    fn eval(&self, i: usize, e: &FuncExpr) -> Result<Option<bool>, String> {
        let cell = self.cells[i];
        // only 0 and 1 are substituted (`funcBddSim`); a transition is unknown to the function
        let fixed = |port: &str| cell.ports.iter().position(|q| q.name == port).and_then(|k| self.values.get(&Pin::Inst(i, k)).and_then(|v| v.constant()));
        crate::search::eval_exact(e, &fixed)
    }

    /// `Sim::evalInstance(inst, false)`.
    fn eval_instance(&mut self, i: usize) -> Result<(), String> {
        let cell = self.cells[i];
        for (k, port) in cell.ports.iter().enumerate() {
            if !any_output(port.direction) {
                continue;
            }
            let parse = |what: &str, f: &str| FuncExpr::parse(f).map_err(|e| format!("cell {} port {}: {what} {f}: {e}", cell.name, port.name));
            let value = match port.function.as_deref().filter(|f| !f.trim().is_empty()) {
                Some(f) => {
                    let func = parse("function", f)?;
                    match port.three_state.as_deref().filter(|t| !t.trim().is_empty()) {
                        Some(t) => {
                            let enable = FuncExpr::Not(Box::new(parse("three_state", t)?));
                            if self.eval(i, &enable)? == Some(true) {
                                self.eval(i, &func)?.map(Logic::of)
                            } else {
                                None
                            }
                        }
                        None => self.eval(i, &func)?.map(Logic::of),
                    }
                }
                None if port.is_clock_gate_out => self.clock_gate_out_value(i),
                None => None,
            };
            let p = Pin::Inst(i, k);
            if value != self.values.get(&p).copied() {
                self.set_pin_value(p, value);
            }
        }
        Ok(())
    }

    /// `Sim::clockGateOutValue`: 0 when the gate's clock or enable pin is 0.
    fn clock_gate_out_value(&self, i: usize) -> Option<Logic> {
        let gated_off = self.cells[i].ports.iter().enumerate().any(|(k, q)| (q.is_clock_gate_clock || q.is_clock_gate_enable) && self.values.get(&Pin::Inst(i, k)) == Some(&Logic::Zero));
        gated_off.then_some(Logic::Zero)
    }
}

/// `recordConstPinFunc`'s test: an output whose function is the constant 0 or 1 and that is not a
/// tristate (a tristate output is not forced constant), and that value.
fn const_func_value(port: &crate::liberty::Port) -> Option<bool> {
    if port.three_state.as_deref().is_some_and(|t| !t.trim().is_empty()) {
        return None;
    }
    match FuncExpr::parse(port.function.as_deref()?).ok()? {
        FuncExpr::Zero => Some(false),
        FuncExpr::One => Some(true),
        _ => None,
    }
}

/// The constant nets the reader makes: `supply` (the netlist's `supply0`/`supply1` nets, by name)
/// plus the nets named [`ZERO_NET`] and [`ONE_NET`], in that order, each once.
pub fn constant_nets(nl: &Netlist, supply: &[(String, bool)]) -> Vec<(String, bool)> {
    let mut out: Vec<(String, bool)> = Vec::new();
    let has = |n: &str| nl.nets.iter().any(|x| x.name == n);
    for (n, v) in supply.iter().cloned().chain([(ZERO_NET.to_string(), false), (ONE_NET.to_string(), true)]) {
        if has(&n) && !out.iter().any(|(m, _)| *m == n) {
            out.push((n, v));
        }
    }
    out
}

/// What the logic simulation holds — every pin by name with its value — and the reference's
/// warnings (1521) on the way. `constant_nets`: `(net, value)` ([`constant_nets`]);
/// `logic_values` and `case_values`: pin or port name → the value it is held at (`None` for a
/// don't-care), `set_logic_*` and `set_case_analysis` (the latter wins on a pin both name, as the
/// reference looks the case value up first).
pub fn simulate(libs: &[Library], nl: &Netlist, constant_nets: &[(String, bool)], logic_values: &HashMap<String, Option<Logic>>, case_values: &HashMap<String, Option<Logic>>) -> Result<SimValues, String> {
    let mut sim = Sim::new(libs, nl)?;
    let mut by_name: HashMap<String, Pin> = HashMap::new();
    for (i, c) in sim.cells.iter().enumerate() {
        for (k, q) in c.ports.iter().enumerate() {
            by_name.insert(format!("{}/{}", nl.insts[i].0, q.name), Pin::Inst(i, k));
        }
    }
    for (i, (p, _)) in nl.ports.iter().enumerate() {
        by_name.insert(p.clone(), Pin::Port(i));
    }
    let pin = |n: &str| by_name.get(n).copied().ok_or_else(|| format!("set_case_analysis / set_logic_*: no pin or port {n}"));
    // `Sdc::caseLogicValue` is looked up first, then `Sdc::logicValue`
    for (n, v) in logic_values {
        sim.constraints.insert(pin(n)?, *v);
    }
    for (n, v) in case_values {
        sim.constraints.insert(pin(n)?, *v);
    }
    // `enqueueConstantPinInputs`: the zero nets' pins, then the one nets'
    for want in [false, true] {
        for (net, _) in constant_nets.iter().filter(|(_, v)| *v == want) {
            let Some(k) = nl.nets.iter().position(|x| x.name == *net) else { continue };
            for p in sim.pins_of[k].clone() {
                sim.set_pin_value(p, Some(Logic::of(want)));
            }
        }
    }
    // `setConstraintConstPins(logicValues)`, then `(caseLogicValues)`, each pin at its own value
    for map in [logic_values, case_values] {
        let mut pins: Vec<(Pin, Option<Logic>)> = map.iter().map(|(n, v)| (by_name[n], *v)).collect();
        pins.sort_by_key(|(p, _)| sim.name(*p));
        for (p, v) in pins {
            sim.set_pin_value(p, v);
        }
    }
    // `setConstFuncPins`: tie cell outputs
    for i in 0..sim.cells.len() {
        for k in 0..sim.cells[i].ports.len() {
            if let Some(v) = const_func_value(&sim.cells[i].ports[k]) {
                sim.set_pin_value(Pin::Inst(i, k), Some(Logic::of(v)));
            }
        }
    }
    // `propagateConstants(false)`
    while let Some(i) = sim.queue.pop_front() {
        sim.eval_instance(i)?;
    }
    let mut out = SimValues::default();
    for (&p, &v) in &sim.values {
        match v {
            Logic::Zero | Logic::One => {
                out.constants.insert(sim.name(p), v == Logic::One);
            }
            Logic::Rise => {
                out.transitions.insert(sim.name(p), crate::liberty::RISE);
            }
            Logic::Fall => {
                out.transitions.insert(sim.name(p), crate::liberty::FALL);
            }
        }
    }
    out.warnings = sim.warnings;
    Ok(out)
}

/// [`simulate`]'s constants and warnings, for a caller with no transition case values.
pub fn constants(libs: &[Library], nl: &Netlist, constant_nets: &[(String, bool)], logic_values: &HashMap<String, Option<Logic>>, case_values: &HashMap<String, Option<Logic>>) -> Result<(HashMap<String, bool>, Vec<String>), String> {
    let v = simulate(libs, nl, constant_nets, logic_values, case_values)?;
    Ok((v.constants, v.warnings))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::netlist::{inst, Net};
    use std::sync::Arc;

    fn port(name: &str, direction: Direction, function: Option<&str>) -> crate::liberty::Port {
        crate::liberty::Port { name: name.into(), direction, function: function.map(String::from), ..Default::default() }
    }

    fn lib() -> Vec<Library> {
        let mut l = Library::default();
        let cell = |name: &str, ports: Vec<crate::liberty::Port>| Cell { name: name.into(), ports, ..Default::default() };
        l.cells.insert("AND2".into(), cell("AND2", vec![port("A", Direction::Input, None), port("B", Direction::Input, None), port("Y", Direction::Output, Some("A&B"))]));
        l.cells.insert("INV".into(), cell("INV", vec![port("A", Direction::Input, None), port("Y", Direction::Output, Some("!A"))]));
        l.cells.insert("TIEHI".into(), cell("TIEHI", vec![port("HI", Direction::Output, Some("1"))]));
        l.cells.insert("DFF".into(), cell("DFF", vec![port("CK", Direction::Input, None), port("D", Direction::Input, None), port("Q", Direction::Output, Some("IQ"))]));
        let mut tri = port("Z", Direction::Output, Some("A"));
        tri.three_state = Some("!EN".into());
        l.cells.insert("TBUF".into(), cell("TBUF", vec![port("A", Direction::Input, None), port("EN", Direction::Input, None), tri]));
        vec![l]
    }

    fn net(name: &str, pins: Vec<Conn>) -> Arc<Net> {
        Arc::new(Net { name: name.into(), pins })
    }

    fn ip(i: usize, p: &str) -> Conn {
        Conn::Inst(i, p.into())
    }

    /// A pin on the `zero_` net (every `1'b0` connection) is 0 and propagates through the gates it
    /// controls: an AND with a 0 input is 0 whatever its other input, and the inverter after it 1.
    /// A gate with an unknown input stays unknown.
    #[test]
    fn a_tied_input_propagates_through_the_gates_it_controls() {
        let nl = Netlist {
            insts: vec![inst("a", "AND2"), inst("n", "INV"), inst("b", "AND2")],
            ports: vec![("x".into(), PortDir::Input)],
            nets: vec![net("w1", vec![ip(1, "A"), ip(0, "Y")]), net("w2", vec![ip(1, "Y")]), net("x", vec![Conn::Port(0), ip(0, "A"), ip(2, "A")]), net(ZERO_NET, vec![ip(0, "B")])],
        };
        let libs = lib();
        let cn = constant_nets(&nl, &[]);
        assert_eq!(cn, vec![(ZERO_NET.to_string(), false)]);
        let (c, w) = constants(&libs, &nl, &cn, &HashMap::new(), &HashMap::new()).unwrap();
        assert!(w.is_empty());
        assert_eq!(c.get("a/B"), Some(&false));
        assert_eq!(c.get("a/Y"), Some(&false));
        assert_eq!(c.get("n/A"), Some(&false));
        assert_eq!(c.get("n/Y"), Some(&true));
        assert_eq!(c.get("b/Y"), None, "b's inputs are unknown");
        assert_eq!(c.get("x"), None);
    }

    /// A tie cell's output (a function that is the constant 1) seeds its loads (`setConstFuncPins`);
    /// a register's output is not looked through (`propagateConstants(false)`).
    #[test]
    fn a_tie_cell_seeds_its_loads_and_a_register_stops_them() {
        let nl = Netlist {
            insts: vec![inst("t", "TIEHI"), inst("r", "DFF"), inst("n", "INV")],
            ports: vec![],
            nets: vec![net("h", vec![ip(1, "D"), ip(0, "HI")]), net("q", vec![ip(2, "A"), ip(1, "Q")])],
        };
        let (c, _) = constants(&lib(), &nl, &[], &HashMap::new(), &HashMap::new()).unwrap();
        assert_eq!(c.get("t/HI"), Some(&true));
        assert_eq!(c.get("r/D"), Some(&true));
        assert_eq!(c.get("r/Q"), None);
        assert_eq!(c.get("n/A"), None);
    }

    /// A tristate output takes its function's value only while its enable (the inverse of
    /// `three_state`) is 1; disabled, it is unknown.
    #[test]
    fn a_tristate_output_is_constant_only_while_enabled() {
        let mk = |en: &str| Netlist {
            insts: vec![inst("t", "TBUF")],
            ports: vec![],
            nets: if en == ONE_NET { vec![net(ONE_NET, vec![ip(0, "A"), ip(0, "EN")])] } else { vec![net(ONE_NET, vec![ip(0, "A")]), net(en, vec![ip(0, "EN")])] },
        };
        for (en, want) in [(ONE_NET, Some(&true)), (ZERO_NET, None)] {
            let nl = mk(en);
            let (c, _) = constants(&lib(), &nl, &constant_nets(&nl, &[]), &HashMap::new(), &HashMap::new()).unwrap();
            assert_eq!(c.get("t/Z"), want, "EN on {en}");
        }
    }

    /// `set_case_analysis` holds its pin: a propagated value that differs is not taken (warning
    /// 1521), and a `rising` case holds the pin at a transition, not a constant, and the function after it sees
    /// an unknown.
    #[test]
    fn a_case_value_holds_its_pin_against_propagation() {
        let nl = Netlist {
            insts: vec![inst("a", "AND2"), inst("n", "INV")],
            ports: vec![],
            nets: vec![net(ZERO_NET, vec![ip(0, "A")]), net("w", vec![ip(1, "A"), ip(0, "Y")])],
        };
        let cn = constant_nets(&nl, &[]);
        let case = HashMap::from([("n/A".to_string(), Some(Logic::One))]);
        let (c, w) = constants(&lib(), &nl, &cn, &HashMap::new(), &case).unwrap();
        assert_eq!(c.get("a/Y"), Some(&false));
        assert_eq!(c.get("n/A"), Some(&true));
        assert_eq!(c.get("n/Y"), Some(&false));
        assert_eq!(w.len(), 1, "{w:?}");
        let rising = HashMap::from([("n/A".to_string(), Some(Logic::Rise))]);
        let v = simulate(&lib(), &nl, &cn, &HashMap::new(), &rising).unwrap();
        assert_eq!(v.constants.get("n/A"), None);
        assert_eq!(v.transitions.get("n/A"), Some(&crate::liberty::RISE));
        assert_eq!(v.constants.get("n/Y"), None, "a transition is unknown to the function");
        assert_eq!(v.transitions.get("n/Y"), None);
    }

    /// A `supply1` net is constant 1 at every pin, and a net only named like the reader's constant
    /// nets is not constant when there is no such net.
    #[test]
    fn supply_nets_are_constant() {
        let nl = Netlist { insts: vec![inst("n", "INV")], ports: vec![], nets: vec![net("vdd", vec![ip(0, "A")])] };
        let cn = constant_nets(&nl, &[("vdd".into(), true), ("absent".into(), false)]);
        assert_eq!(cn, vec![("vdd".to_string(), true)]);
        let (c, _) = constants(&lib(), &nl, &cn, &HashMap::new(), &HashMap::new()).unwrap();
        assert_eq!(c.get("n/Y"), Some(&false));
    }
}
