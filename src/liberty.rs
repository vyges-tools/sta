// SPDX-License-Identifier: Apache-2.0
//! A liberty library as the timer reads it: units, cells, ports and (later) timing arcs.
//!
//! Rules:
//! - numbers are `f32` as the syntax reader keeps them; every value is multiplied by an `f32` unit
//!   scale — `time_unit` (default 1 ns), `capacitive_load_unit` (default 1 pf; `ff` → `x·1e-15F`,
//!   `pf` → `x·1e-12F`);
//! - a unit string is `<1|10|100><k|m|u|n|p|f><suffix>` (or the bare suffix): multiplier times
//!   the prefix scale, both `f32`.

use std::collections::BTreeMap;

use crate::liberty_parse::{Group, Value};
use crate::table::{Axis, AxisVar, GateModel, Table};

/// A pin's direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Input,
    Output,
    Tristate,
    Bidirect,
    Internal,
    Unknown,
}

impl Direction {
    fn parse(s: &str) -> Direction {
        match s {
            "input" => Direction::Input,
            "output" => Direction::Output,
            "inout" => Direction::Bidirect,
            "internal" => Direction::Internal,
            _ => Direction::Unknown,
        }
    }
    pub fn is_input(self) -> bool {
        self == Direction::Input
    }
    /// A driver: output, tristate or bidirect.
    pub fn drives(self) -> bool {
        matches!(self, Direction::Output | Direction::Tristate | Direction::Bidirect)
    }
    /// A load: input or bidirect.
    pub fn loads(self) -> bool {
        matches!(self, Direction::Input | Direction::Bidirect)
    }
}

/// Transition index: rise 0, fall 1. Min/max index: min 0, max 1.
pub const RISE: usize = 0;
pub const FALL: usize = 1;
pub const MIN: usize = 0;
pub const MAX: usize = 1;

#[derive(Debug, Clone, PartialEq)]
pub struct Port {
    pub name: String,
    pub direction: Direction,
    pub is_clock: bool,
    /// `capacitance_` per `[rise/fall][min/max]`, after the defaults.
    pub capacitance: [[f32; 2]; 2],
}

impl Port {
    /// The pin capacitance for a transition and min/max.
    pub fn capacitance(&self, rf: usize, min_max: usize) -> f32 {
        self.capacitance[rf][min_max]
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Cell {
    pub name: String,
    /// Ports in the cell's order.
    pub ports: Vec<Port>,
    /// Timing arc sets in the order the timing groups were read.
    pub arc_sets: Vec<ArcSet>,
    /// `ff`/`latch` groups: `(output names, is_register, clock or enable expression)`.
    pub sequentials: Vec<(Vec<String>, bool, String)>,
}

impl Cell {
    pub fn port(&self, name: &str) -> Option<&Port> {
        self.ports.iter().find(|p| p.name == name)
    }
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Library {
    pub name: String,
    pub time_scale: f32,
    pub cap_scale: f32,
    pub default_input_pin_cap: f32,
    pub default_output_pin_cap: f32,
    pub default_bidirect_pin_cap: f32,
    /// `input_threshold_pct_*`, `output_threshold_pct_*`, `slew_lower/upper_threshold_pct_*`, each
    /// `× 0.01F`, per `[rise, fall]` (defaults .5, .5, .2, .8); `slew_derate_from_library` (1).
    pub input_threshold: [f32; 2],
    pub output_threshold: [f32; 2],
    pub slew_lower_threshold: [f32; 2],
    pub slew_upper_threshold: [f32; 2],
    pub slew_derate: f32,
    /// `lu_table_template`s: each axis's variable and (scaled) values.
    pub templates: BTreeMap<String, Vec<Axis>>,
    pub cells: BTreeMap<String, Cell>,
}

/// A unit's scale. An unknown multiplier only WARNS (then 1 is used), as does an unknown scale
/// letter or suffix (then 1 for the prefix) — the result is never refused.
fn unit_scale(units: &str, suffix: &str) -> f32 {
    let mult_end = units.find(|c: char| !c.is_ascii_digit());
    let (mult, scale_suffix) = match mult_end {
        Some(end) => {
            let m = match &units[..end] {
                "1" => 1.0f32,
                "10" => 10.0,
                "100" => 100.0,
                _ => 1.0,
            };
            (m, &units[end..])
        }
        None => (1.0f32, units),
    };
    let mut scale_mult = 1.0f32;
    if scale_suffix.len() == suffix.len() + 1 && scale_suffix[1..].eq_ignore_ascii_case(suffix) {
        scale_mult = match scale_suffix.as_bytes()[0].to_ascii_lowercase() {
            b'k' => 1e3f32,
            b'm' => 1e-3,
            b'u' => 1e-6,
            b'n' => 1e-9,
            b'p' => 1e-12,
            b'f' => 1e-15,
            _ => 1.0,
        };
    }
    scale_mult * mult
}

impl Library {
    /// Read a parsed `library` group.
    pub fn read(g: &Group) -> Result<Library, String> {
        let mut lib = Library { name: g.name().unwrap_or_default(), time_scale: 1e-9, cap_scale: 1e-12, ..Default::default() };
        if let Some(t) = g.attr_text("time_unit") {
            lib.time_scale = unit_scale(&t, "s");
        }
        if let Some(v) = g.complex_attr("capacitive_load_unit") {
            if let [scale, Value::Str(suffix)] = v.as_slice() {
                let scale = scale.float().ok_or("capacitive_load_unit scale is not a float")?;
                if suffix.eq_ignore_ascii_case("ff") {
                    lib.cap_scale = scale * 1e-15f32;
                } else if suffix.eq_ignore_ascii_case("pf") {
                    lib.cap_scale = scale * 1e-12f32;
                } else {
                    return Err("capacitive_load_units are not ff or pf".into());
                }
            }
        }
        let cap = |name: &str| g.attr_float(name).map(|v| v * lib.cap_scale).unwrap_or(0.0);
        lib.default_input_pin_cap = cap("default_input_pin_cap");
        lib.default_output_pin_cap = cap("default_output_pin_cap");
        lib.default_bidirect_pin_cap = cap("default_inout_pin_cap");
        lib.input_threshold = [0.5; 2];
        lib.output_threshold = [0.5; 2];
        lib.slew_lower_threshold = [0.2; 2];
        lib.slew_upper_threshold = [0.8; 2];
        lib.slew_derate = g.attr_float("slew_derate_from_library").unwrap_or(1.0);
        for (rf, word) in [(RISE, "rise"), (FALL, "fall")] {
            for (field, name) in [(&mut lib.input_threshold, "input_threshold_pct_"), (&mut lib.output_threshold, "output_threshold_pct_"), (&mut lib.slew_lower_threshold, "slew_lower_threshold_pct_"), (&mut lib.slew_upper_threshold, "slew_upper_threshold_pct_")] {
                if let Some(v) = g.attr_float(&format!("{name}{word}")) {
                    field[rf] = v * 0.01f32;
                }
            }
        }
        for tg in g.groups_of("lu_table_template") {
            let Some(name) = tg.name() else { continue };
            let mut axes = Vec::new();
            for k in 1..=3 {
                let Some(var) = tg.attr_text(&format!("variable_{k}")) else { break };
                let var = AxisVar::parse(&var);
                let values = tg.complex_attr(&format!("index_{k}")).map(|v| float_seq(v, 1.0)).unwrap_or_default();
                let scale = if var.is_capacitance() { lib.cap_scale } else { lib.time_scale };
                axes.push(Axis { var, values: values.into_iter().map(|x| x * scale).collect() });
            }
            lib.templates.insert(name, axes);
        }
        for cg in g.groups_of("cell") {
            let cell = lib.read_cell(cg)?;
            lib.cells.insert(cell.name.clone(), cell);
        }
        Ok(lib)
    }

    fn default_cap(&self, dir: Direction) -> f32 {
        match dir {
            Direction::Input => self.default_input_pin_cap,
            Direction::Output | Direction::Tristate => self.default_output_pin_cap,
            Direction::Bidirect => self.default_bidirect_pin_cap,
            _ => 0.0,
        }
    }

    fn read_cell(&self, cg: &Group) -> Result<Cell, String> {
        let mut cell = Cell { name: cg.name().ok_or("cell without a name")?, ..Default::default() };
        if cg.groups.iter().any(|g| g.kind == "bus" || g.kind == "bundle") {
            return Err(format!("cell {}: buses and bundles are not modelled", cell.name));
        }
        for (kind, is_register, clock_attr) in [("ff", true, "clocked_on"), ("latch", false, "enable")] {
            for sg in cg.groups_of(kind) {
                let outputs = sg.params.iter().map(Value::text).collect();
                cell.sequentials.push((outputs, is_register, sg.attr_text(clock_attr).unwrap_or_default()));
            }
        }
        for pg in cg.groups_of("pin") {
            for name in &pg.params {
                cell.ports.push(self.read_port(&name.text(), pg));
            }
        }
        for pg in cg.groups_of("pin") {
            for to in &pg.params {
                let to = to.text();
                let function = pg.attr_text("function");
                for tg in pg.groups_of("timing") {
                    let sets = self.read_timing(&cell, &to, function.as_deref(), tg)?;
                    cell.arc_sets.extend(sets);
                }
            }
        }
        Ok(cell)
    }

    /// Port attributes: `capacitance` sets all four values, then
    /// `rise_/fall_capacitance` a transition's min and max, then `_capacitance_range (min, max)`
    /// each; whatever is still unset takes the library default for the direction.
    fn read_port(&self, name: &str, pg: &Group) -> Port {
        let direction = pg.attr_text("direction").map_or(Direction::Unknown, |d| Direction::parse(&d));
        let mut cap: [[Option<f32>; 2]; 2] = [[None; 2]; 2];
        if let Some(c) = pg.attr_float("capacitance") {
            cap = [[Some(c * self.cap_scale); 2]; 2];
        }
        for (rf, word) in [(RISE, "rise"), (FALL, "fall")] {
            if let Some(c) = pg.attr_float(&format!("{word}_capacitance")) {
                cap[rf] = [Some(c * self.cap_scale); 2];
            }
            if let Some(v) = pg.complex_attr(&format!("{word}_capacitance_range")) {
                if v.len() == 2 {
                    if let Some(lo) = v[0].float() {
                        cap[rf][MIN] = Some(lo * self.cap_scale);
                    }
                    if let Some(hi) = v[1].float() {
                        cap[rf][MAX] = Some(hi * self.cap_scale);
                    }
                }
            }
        }
        let default = self.default_cap(direction);
        let capacitance = cap.map(|row| row.map(|c| c.unwrap_or(default)));
        let is_clock = pg.attr_text("clock").is_some_and(|c| c == "true");
        Port { name: name.to_string(), direction, is_clock, capacitance }
    }
}

/// The timing roles the timer distinguishes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    Combinational,
    RegClkToQ,
    LatchEnToQ,
    LatchDtoQ,
    RegSetClr,
    Setup,
    Hold,
    Recovery,
    Removal,
    /// Pulse width, period, tristate, non-sequential checks, … — read, never timed here.
    Other,
}

/// A timing model: a gate's delay and slew tables, or a check's constraint table.
#[derive(Debug, Clone, PartialEq)]
pub enum Model {
    Gate(GateModel),
    Check(Table),
}

/// One timing arc: from and to transitions (`RISE`/`FALL`) and its model.
#[derive(Debug, Clone, PartialEq)]
pub struct Arc {
    pub from_rf: usize,
    pub to_rf: usize,
    pub model: Model,
}

/// One timing arc set — a timing group from one related pin to one port.
#[derive(Debug, Clone, PartialEq)]
pub struct ArcSet {
    pub from: String,
    pub to: String,
    pub role: Role,
    pub timing_type: String,
    /// `when`, as text.
    pub cond: Option<String>,
    pub arcs: Vec<Arc>,
}

/// A quoted list or plain values as floats, each `× scale`.
fn float_seq(values: &[Value], scale: f32) -> Vec<f32> {
    let mut out = Vec::new();
    for v in values {
        match v {
            Value::Float(f) => out.push(f * scale),
            Value::Str(s) => {
                for tok in s.split([' ', ',', '{', '}']).filter(|t| !t.is_empty()) {
                    if let Ok(f) = tok.parse::<f32>() {
                        out.push(f * scale);
                    }
                }
            }
        }
    }
    out
}

fn identifiers(expr: &str) -> Vec<String> {
    expr.split(|c: char| !(c.is_ascii_alphanumeric() || c == '_' || c == '[' || c == ']')).filter(|t| !t.is_empty()).map(String::from).collect()
}

impl Library {
    /// A table: the template's axes, each overridden by the table's own `index_N`
    /// (scaled by the axis variable's unit); values row by row, `× scale`.
    fn read_table(&self, g: &Group, scale: f32) -> Result<Table, String> {
        let tmpl = g.name().unwrap_or_default();
        let template = if tmpl == "scalar" { Some(Vec::new()) } else { self.templates.get(&tmpl).cloned() };
        let template = template.ok_or_else(|| format!("table template {tmpl} not found"))?;
        let mut axes = Vec::new();
        for (k, ax) in template.iter().enumerate() {
            match g.complex_attr(&format!("index_{}", k + 1)) {
                Some(v) => {
                    let s = if ax.var.is_capacitance() { self.cap_scale } else { self.time_scale };
                    axes.push(Axis { var: ax.var, values: float_seq(v, 1.0).into_iter().map(|x| x * s).collect() });
                }
                None => axes.push(ax.clone()),
            }
        }
        let values = float_seq(g.complex_attr("values").ok_or("table missing values")?, scale);
        let want: usize = axes.iter().map(|a| a.values.len()).product::<usize>().max(1);
        if values.len() != want {
            return Err(format!("table has {} values, its axes {want}", values.len()));
        }
        Ok(Table { axes, values })
    }

    /// The arc sets of one timing group to port `to`: its models per transition, its role and arcs
    /// by `timing_type` and `timing_sense`, one arc set per related pin.
    fn read_timing(&self, cell: &Cell, to: &str, function: Option<&str>, tg: &Group) -> Result<Vec<ArcSet>, String> {
        let timing_type = tg.attr_text("timing_type").unwrap_or_else(|| "combinational".into());
        let sense = tg.attr_text("timing_sense");
        let cond = tg.attr_text("when");
        let mut models: [Option<Model>; 2] = [None, None];
        for (rf, word) in [(RISE, "rise"), (FALL, "fall")] {
            let delay = tg.groups_of(&format!("cell_{word}")).next().map(|g| self.read_table(g, self.time_scale)).transpose()?;
            let slew = tg.groups_of(&format!("{word}_transition")).next().map(|g| self.read_table(g, self.time_scale)).transpose()?;
            if delay.is_some() || slew.is_some() {
                models[rf] = Some(Model::Gate(GateModel { delay, slew }));
            }
            if let Some(g) = tg.groups_of(&format!("{word}_constraint")).next() {
                models[rf] = Some(Model::Check(self.read_table(g, self.time_scale)?));
            }
        }
        // The output's sequential, through the ports its function names.
        let seq = function.and_then(|f| {
            let ids = identifiers(f);
            cell.sequentials.iter().find(|(outs, _, _)| outs.iter().any(|o| ids.contains(o)))
        });
        let related: Vec<String> = tg.attr_text("related_pin").map(|r| r.split_whitespace().map(String::from).collect()).unwrap_or_default();
        if related.is_empty() {
            return Ok(Vec::new());
        }
        let mut out = Vec::new();
        for from in related {
            let in_clock = seq.is_some_and(|(_, _, clk)| identifiers(clk).contains(&from));
            if timing_type == "combinational" && seq.is_some() && in_clock {
                return Err(format!("cell {}: a register timing group without timing_type (from {from}) is not modelled", cell.name));
            }
            let from_transition = |from_rf: usize, role: Role| -> ArcSet {
                let arcs = (0..2).filter_map(|to_rf| models[to_rf].clone().map(|model| Arc { from_rf, to_rf, model })).collect();
                ArcSet { from: from.clone(), to: to.to_string(), role, timing_type: timing_type.clone(), cond: cond.clone(), arcs }
            };
            let set = match timing_type.as_str() {
                "combinational" | "combinational_rise" | "combinational_fall" => {
                    let (to_rise, to_fall) = match timing_type.as_str() {
                        "combinational_rise" => (true, false),
                        "combinational_fall" => (false, true),
                        _ => (true, true),
                    };
                    let mut arcs = Vec::new();
                    let mut push = |from_rf: usize, to_rf: usize| {
                        if let Some(m) = &models[to_rf] {
                            arcs.push(Arc { from_rf, to_rf, model: m.clone() });
                        }
                    };
                    match sense.as_deref() {
                        Some("positive_unate") => {
                            if to_rise {
                                push(RISE, RISE);
                            }
                            if to_fall {
                                push(FALL, FALL);
                            }
                        }
                        Some("negative_unate") => {
                            if to_fall {
                                push(RISE, FALL);
                            }
                            if to_rise {
                                push(FALL, RISE);
                            }
                        }
                        Some("non_unate") => {
                            if to_fall {
                                push(FALL, FALL);
                                push(RISE, FALL);
                            }
                            if to_rise {
                                push(RISE, RISE);
                                push(FALL, RISE);
                            }
                        }
                        _ => return Err(format!("cell {}: a combinational arc {from} -> {to} without timing_sense is not modelled", cell.name)),
                    }
                    ArcSet { from: from.clone(), to: to.to_string(), role: Role::Combinational, timing_type: timing_type.clone(), cond: cond.clone(), arcs }
                }
                "rising_edge" | "falling_edge" => {
                    let from_rf = if timing_type == "rising_edge" { RISE } else { FALL };
                    let role = match seq {
                        Some((_, false, clk)) if identifiers(clk).contains(&from) => Role::LatchEnToQ,
                        _ => Role::RegClkToQ,
                    };
                    from_transition(from_rf, role)
                }
                "setup_rising" => from_transition(RISE, Role::Setup),
                "setup_falling" => from_transition(FALL, Role::Setup),
                "hold_rising" => from_transition(RISE, Role::Hold),
                "hold_falling" => from_transition(FALL, Role::Hold),
                "recovery_rising" => from_transition(RISE, Role::Recovery),
                "recovery_falling" => from_transition(FALL, Role::Recovery),
                "removal_rising" => from_transition(RISE, Role::Removal),
                "removal_falling" => from_transition(FALL, Role::Removal),
                "preset" | "clear" => {
                    let to_rf = if timing_type == "preset" { RISE } else { FALL };
                    let Some(m) = models[to_rf].clone() else { continue };
                    let opp = 1 - to_rf;
                    let arcs = match sense.as_deref() {
                        Some("positive_unate") => vec![Arc { from_rf: to_rf, to_rf, model: m }],
                        Some("negative_unate") => vec![Arc { from_rf: opp, to_rf, model: m }],
                        _ => vec![Arc { from_rf: to_rf, to_rf, model: m.clone() }, Arc { from_rf: opp, to_rf, model: m }],
                    };
                    ArcSet { from: from.clone(), to: to.to_string(), role: Role::RegSetClr, timing_type: timing_type.clone(), cond: cond.clone(), arcs }
                }
                _ => ArcSet { from: from.clone(), to: to.to_string(), role: Role::Other, timing_type: timing_type.clone(), cond: cond.clone(), arcs: Vec::new() },
            };
            out.push(set);
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::liberty_parse::parse;

    // Rule: <1|10|100><prefix><suffix>, both factors f32; an unknown
    // multiplier only warns and counts as 1.
    #[test]
    fn units_are_multiplier_times_prefix() {
        assert_eq!(unit_scale("1ns", "s"), 1e-9f32);
        assert_eq!(unit_scale("10ps", "s"), 1e-12f32 * 10.0);
        assert_eq!(unit_scale("s", "s"), 1.0);
        assert_eq!(unit_scale("2ns", "s"), 1e-9f32, "an unknown multiplier is 1");
    }

    // Rules: capacitance sets all four, rise/fall override a
    // transition, a range sets min and max, the default fills only what is unset.
    #[test]
    fn port_capacitance_layers_its_attributes() {
        let text = r#"library (l) { capacitive_load_unit (1, ff) ; default_input_pin_cap : 2 ;
            cell (C) {
              pin (A) { direction : input ; capacitance : 1.5 ; fall_capacitance : 1.25 ; }
              pin (B) { direction : input ; rise_capacitance_range (0.5, 0.75) ; }
              pin (Y) { direction : output ; }
            } }"#;
        let lib = Library::read(&parse(text).unwrap()).unwrap();
        let c = &lib.cells["C"];
        let s = 1e-15f32;
        assert_eq!(c.port("A").unwrap().capacitance, [[1.5 * s, 1.5 * s], [1.25 * s, 1.25 * s]]);
        assert_eq!(c.port("B").unwrap().capacitance, [[0.5 * s, 0.75 * s], [2.0 * s, 2.0 * s]]);
        assert_eq!(c.port("Y").unwrap().capacitance, [[0.0; 2]; 2]);
    }
}
