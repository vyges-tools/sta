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
    /// `function`, as text (the reader keeps it unparsed; see [`Cell::is_buffer`]).
    pub function: Option<String>,
    /// `max_transition` (`× time unit`) and `max_capacitance` (`× capacitive load unit`), when set.
    /// A `max_transition` of 0 is still set (the reference only warns).
    pub max_transition: Option<f32>,
    pub max_capacitance: Option<f32>,
    /// `fanout_load`, unscaled, when set.
    pub fanout_load: Option<f32>,
    /// `max_fanout`, unscaled, when set.
    pub max_fanout: Option<f32>,
    /// `three_state` (the tristate enable function), as text.
    pub three_state: Option<String>,
}

impl Port {
    /// `PortDirection::isAnyTristate`, as the reader sets it: a tristate or bidirect port, or an
    /// OUTPUT with a non-empty `three_state` (the reader makes such an output tristate; the
    /// `direction` this crate keeps is the attribute as written).
    pub fn is_any_tristate(&self) -> bool {
        matches!(self.direction, Direction::Tristate | Direction::Bidirect)
            || (self.direction == Direction::Output && self.three_state.as_deref().is_some_and(|t| !t.trim().is_empty()))
    }

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
    /// The same groups as the reference's `Sequential`s, in `makeSequentials` order (`ff`, then
    /// `latch`), each as `equivCellSequentials` compares it.
    pub seqs: Vec<Seq>,
    /// The cell has an `ff_bank` or `latch_bank` group (not read: bus ports are not modelled).
    pub has_seq_bank: bool,
    /// `area` (unscaled), `cell_footprint`, `user_function_class` (empty when unset).
    pub area: f32,
    pub footprint: String,
    pub user_function_class: String,
    /// Boolean cell attributes, `true`/`false` compared case-insensitively. `is_pad` is set by
    /// either `is_pad` or `pad_cell`, read in that order, so `pad_cell` wins when both appear.
    pub dont_use: bool,
    pub is_pad: bool,
    pub is_level_shifter: bool,
    pub is_isolation_cell: bool,
    pub always_on: bool,
    pub is_clock_cell: bool,
    /// The cell has a `statetable` group (not read further).
    pub has_statetable: bool,
    /// `pg_pin` groups, in order: (name, `pg_type`). The reference counts them among a cell's ports.
    pub pg_pins: Vec<(String, String)>,
    /// Bus ports: name and member bit ports, in the reader's member order (bit_from to bit_to).
    /// Only the members are in [`Cell::ports`].
    pub buses: Vec<(String, Vec<String>)>,
    /// Each bus as the one top-level port the reader also makes (its name, the bus group's
    /// direction, its function unsliced), in `buses` order.
    pub bus_ports: Vec<Port>,
    /// `cell_leakage_power` × the library's power scale, when set.
    pub leakage_power: Option<f32>,
    /// Each `leakage_power` group's `value` × the power scale, in order (groups without one are
    /// skipped, as the reference only warns).
    pub leakage_powers: Vec<f32>,
}

impl Cell {
    pub fn port(&self, name: &str) -> Option<&Port> {
        self.ports.iter().find(|p| p.name == name)
    }

    /// `bufferPorts`: the single input and single output, in port order. A second input or
    /// output, or a port of any other direction, means none.
    pub fn buffer_ports(&self) -> Option<(&Port, &Port)> {
        // A bus is no single pin: a cell with one has no buffer pins (see `is_buffer`).
        if !self.buses.is_empty() {
            return None;
        }
        let (mut input, mut output) = (None, None);
        for p in &self.ports {
            match p.direction {
                Direction::Input if input.is_none() => input = Some(p),
                Direction::Output if output.is_none() => output = Some(p),
                _ => return None,
            }
        }
        Some((input?, output?))
    }

    /// `isBuffer`: buffer ports, the output's function is the input port itself, and neither a
    /// level shifter nor a pad.
    pub fn is_buffer(&self) -> bool {
        // `bufferPorts` walks the cell's TOP-LEVEL ports: a pin, or a whole bus as one port.
        let top = self.top_level_ports();
        let (mut input, mut output) = (None, None);
        for p in &top {
            match p.direction {
                Direction::Input if input.is_none() => input = Some(*p),
                Direction::Output if output.is_none() => output = Some(*p),
                _ => return false,
            }
        }
        let (Some(i), Some(o)) = (input, output) else { return false };
        o.function.as_deref().is_some_and(|f| function_is_port(f, &i.name)) && !self.is_level_shifter && !self.is_pad
    }

    /// The cell's top-level ports in order: each pin, and each bus once (where its first member is).
    pub fn top_level_ports(&self) -> Vec<&Port> {
        let mut out = Vec::new();
        for p in &self.ports {
            match self.buses.iter().position(|(_, m)| m.contains(&p.name)) {
                Some(b) if self.buses[b].1.first() == Some(&p.name) => out.push(&self.bus_ports[b]),
                Some(_) => {}
                None => out.push(p),
            }
        }
        out
    }

    /// `LibertyPort::driveResistance()`: the largest positive arc drive over the non-check arc
    /// sets into `port`, every transition; 0 when none is positive.
    pub fn drive_resistance(&self, port: &str) -> f32 {
        let mut max_drive = f32::MIN;
        let mut found = false;
        for set in self.arc_sets.iter().filter(|s| s.to == port && !s.role.is_timing_check()) {
            for arc in &set.arcs {
                if let Model::Gate(m) = &arc.model {
                    let drive = m.drive_resistance();
                    if drive > 0.0 {
                        if drive > max_drive {
                            max_drive = drive;
                        }
                        found = true;
                    }
                }
            }
        }
        if found {
            max_drive
        } else {
            0.0
        }
    }
}

/// `isInverter`: buffer ports whose output function is the input inverted (`!A`, `A'`), and
/// neither a level shifter nor a pad.
impl Cell {
    pub fn is_inverter(&self) -> bool {
        self.buffer_ports().is_some_and(|(i, o)| o.function.as_deref().is_some_and(|f| function_is_not_port(f, &i.name))) && !self.is_level_shifter && !self.is_pad
    }
}

fn strip_parens(mut f: &str) -> &str {
    f = f.trim();
    while f.len() >= 2 && f.starts_with('(') && f.ends_with(')') {
        f = f[1..f.len() - 1].trim();
    }
    f
}

/// True when a `function` is one port inverted: `!A`, `!(A)`, `A'`, `(A)'`, in parentheses or not.
pub fn function_is_not_port(function: &str, port: &str) -> bool {
    let f = strip_parens(function);
    if let Some(rest) = f.strip_prefix('!') {
        return function_is_port(rest, port);
    }
    if let Some(rest) = f.strip_suffix('\'') {
        return function_is_port(rest, port);
    }
    false
}

/// True when a `function` is a constant (`0`, `1`, `1'b0`, `1'b1`): a tie cell's output.
pub fn function_is_constant(function: &str) -> bool {
    matches!(strip_parens(function), "0" | "1" | "1'b0" | "1'b1")
}

/// True when a liberty `function` is exactly one port — the expression parser reduces `A`,
/// `(A)` and `((A))` to the port itself.
pub fn function_is_port(function: &str, port: &str) -> bool {
    let mut f = function.trim();
    while f.len() >= 2 && f.starts_with('(') && f.ends_with(')') {
        f = f[1..f.len() - 1].trim();
    }
    f == port
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
    /// `leakage_power_unit` (default 1 W).
    pub power_scale: f32,
    /// `default_max_transition` (`× time unit`) and `default_fanout_load` (unscaled), when set —
    /// a value of 0 is set too (the reference warns and keeps it).
    pub default_max_transition: Option<f32>,
    pub default_fanout_load: Option<f32>,
    /// `default_max_fanout` (unscaled; a 0 is kept). ⚠️ No `default_max_capacitance`: the
    /// reference's reader never sets it, so a library's value has no effect there.
    pub default_max_fanout: Option<f32>,
    /// `lu_table_template`s: each axis's variable and (scaled) values.
    pub templates: BTreeMap<String, Vec<Axis>>,
    /// Library-level bus types (`type` groups): name -> (bit_from, bit_to).
    pub bus_types: BTreeMap<String, (i32, i32)>,
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
        let mut lib = Library { name: g.name().unwrap_or_default(), time_scale: 1e-9, cap_scale: 1e-12, power_scale: 1.0, ..Default::default() };
        if let Some(t) = g.attr_text("leakage_power_unit") {
            lib.power_scale = unit_scale(&t, "W");
        }
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
        lib.default_max_transition = g.attr_float("default_max_transition").map(|v| v * lib.time_scale);
        lib.default_fanout_load = g.attr_float("default_fanout_load");
        lib.default_max_fanout = g.attr_float("default_max_fanout");
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
        if let Some(style) = g.attr_text("bus_naming_style") {
            if style != "%s[%d]" {
                return Err(format!("bus_naming_style {style}: only %s[%d] is modelled"));
            }
        }
        lib.bus_types = read_bus_types(g);
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
        cell.area = cg.attr_float("area").unwrap_or(0.0);
        cell.footprint = cg.attr_text("cell_footprint").unwrap_or_default();
        cell.user_function_class = cg.attr_text("user_function_class").unwrap_or_default();
        let flag = |name: &str, current: bool| match cg.attr_text(name) {
            Some(v) if v.eq_ignore_ascii_case("true") => true,
            Some(v) if v.eq_ignore_ascii_case("false") => false,
            _ => current,
        };
        cell.dont_use = flag("dont_use", false);
        cell.is_pad = flag("pad_cell", flag("is_pad", false));
        cell.is_level_shifter = flag("is_level_shifter", false);
        cell.is_isolation_cell = flag("is_isolation_cell", false);
        cell.always_on = flag("always_on", false);
        cell.is_clock_cell = flag("is_clock_cell", false);
        cell.has_statetable = cg.groups_of("statetable").next().is_some();
        cell.leakage_power = cg.attr_float("cell_leakage_power").map(|v| v * self.power_scale);
        cell.leakage_powers = cg.groups_of("leakage_power").filter_map(|lg| lg.attr_float("value")).map(|v| v * self.power_scale).collect();
        for pg in cg.groups_of("pg_pin") {
            if let Some(name) = pg.name() {
                cell.pg_pins.push((name, pg.attr_text("pg_type").unwrap_or_default()));
            }
        }
        if cg.groups.iter().any(|g| g.kind == "bundle") {
            return Err(format!("cell {}: bundles are not modelled", cell.name));
        }
        for (kind, is_register, clock_attr, data_attr) in [("ff", true, "clocked_on", "next_state"), ("latch", false, "enable", "data_in")] {
            for sg in cg.groups_of(kind) {
                let outputs: Vec<String> = sg.params.iter().map(Value::text).collect();
                cell.sequentials.push((outputs.clone(), is_register, sg.attr_text(clock_attr).unwrap_or_default()));
                let attr = |a: &str| sg.attr_text(a).filter(|t| !t.is_empty());
                cell.seqs.push(Seq {
                    is_register,
                    clock: attr(clock_attr),
                    data: attr(data_attr),
                    clear: attr("clear"),
                    preset: attr("preset"),
                    clear_preset_var1: LogicValue::read(sg.attr_text("clear_preset_var1")),
                    clear_preset_var2: LogicValue::read(sg.attr_text("clear_preset_var2")),
                    output: outputs.first().cloned(),
                    output_inv: outputs.get(1).cloned(),
                });
            }
        }
        cell.has_seq_bank = cg.groups_of("ff_bank").next().is_some() || cg.groups_of("latch_bank").next().is_some();
        // makeCellPorts: pin and bus groups in the file's order (the port group map is ordered by
        // line); a bus makes its member bits, then the pin groups inside it name bits of it.
        let cell_types = read_bus_types(cg);
        // (group, the ports its attributes and timing apply to: a pin's names, or the bus).
        let mut port_groups: Vec<(&Group, Vec<PortRef>)> = Vec::new();
        for pg in &cg.groups {
            match pg.kind.as_str() {
                "pin" => {
                    let names: Vec<String> = pg.params.iter().map(Value::text).collect();
                    for name in &names {
                        cell.ports.push(self.read_port(name, pg));
                    }
                    port_groups.push((pg, names.into_iter().map(PortRef::Pin).collect()));
                }
                "bus" => {
                    let ty = pg.attr_text("bus_type").ok_or_else(|| format!("cell {}: a bus without bus_type is not modelled", cell.name))?;
                    let &(from, to) = cell_types.get(&ty).or_else(|| self.bus_types.get(&ty)).ok_or_else(|| format!("cell {}: bus_type {ty} not found", cell.name))?;
                    for bus in pg.params.iter().map(Value::text) {
                        let bits = bus_bits(&bus, from, to);
                        cell.buses.push((bus.clone(), bits.iter().map(|(b, _)| b.clone()).collect()));
                        cell.bus_ports.push(self.read_port(&bus, pg));
                        port_groups.push((pg, vec![PortRef::Bus(bus.clone())]));
                        for (bit, _) in &bits {
                            let mut port = self.read_port(bit, pg);
                            // Functions are made once every port exists (makePortFuncs), below.
                            port.function = None;
                            port.three_state = None;
                            cell.ports.push(port);
                        }
                        // The pin groups inside the bus name bits of it.
                        for ipg in pg.groups_of("pin") {
                            let mut names = Vec::new();
                            for n in ipg.params.iter().map(Value::text) {
                                names.extend(cell.port_bits(&n));
                            }
                            port_groups.push((ipg, names.into_iter().map(PortRef::Pin).collect()));
                        }
                    }
                }
                _ => {}
            }
        }
        // readPortAttributes / makePortFuncs per port group, in line order: a bus sets every
        // member (its function sliced by member OFFSET, its three_state by member BUS INDEX); a pin
        // group inside a bus then sets what it states on its bits.
        for (pg, refs) in &port_groups {
            for r in refs {
                match r {
                    PortRef::Bus(b) => {
                        let members = cell.bus_members(b).to_vec();
                        for (offset, bit) in members.iter().enumerate() {
                            let index = bit.rsplit_once('[').and_then(|(_, i)| i.strip_suffix(']')).and_then(|i| i.parse::<usize>().ok()).unwrap_or(usize::MAX);
                            let function = pg.attr_text("function").filter(|f| !f.is_empty()).map(|f| bit_sub_expr(&f, offset, &cell.buses)).transpose().map_err(|e| format!("cell {}: {e}", cell.name))?;
                            let three_state = pg.attr_text("three_state").filter(|t| !t.is_empty()).map(|t| bit_sub_expr(&t, index, &cell.buses)).transpose().map_err(|e| format!("cell {}: {e}", cell.name))?;
                            let port = cell.ports.iter_mut().find(|p| &p.name == bit).expect("a member port");
                            port.function = function;
                            port.three_state = three_state;
                        }
                    }
                    PortRef::Pin(n) if pg.kind == "pin" && !cg.groups.iter().any(|g| std::ptr::eq(g, *pg)) => {
                        let i = cell.ports.iter().position(|p| &p.name == n).ok_or_else(|| format!("cell {}: pin {n} not found", cell.name))?;
                        self.override_port(&mut cell.ports[i], pg);
                    }
                    PortRef::Pin(_) => {}
                }
            }
        }
        // Per port group, in line order: its timing groups; per timing group, per port, per
        // related name, the bit pairs that name expands to.
        for (pg, refs) in &port_groups {
            for tg in pg.groups_of("timing") {
                let related = |attr: &str| tg.attr_text(attr).map(|r| r.split_whitespace().map(String::from).collect::<Vec<_>>()).unwrap_or_default();
                let (pins, bus_pins) = (related("related_pin"), related("related_bus_pins"));
                for to in refs {
                    let to_bits = match to {
                        PortRef::Pin(n) => vec![n.clone()],
                        PortRef::Bus(b) => cell.bus_members(b).to_vec(),
                    };
                    let is_bus = matches!(to, PortRef::Bus(_));
                    let mut pairs = Vec::new();
                    for r in &pins {
                        pairs.extend(expand_pairs(&cell.port_bits(r), &to_bits, is_bus, true));
                    }
                    for r in &bus_pins {
                        pairs.extend(expand_pairs(&cell.port_bits(r), &to_bits, is_bus, false));
                    }
                    if pins.is_empty() && bus_pins.is_empty() {
                        for t in &to_bits {
                            let function = cell.ports.iter().find(|p| &p.name == t).and_then(|p| p.function.clone());
                            cell.arc_sets.extend(self.read_timing(&cell, t, function.as_deref(), tg, &[])?);
                        }
                    }
                    for (f, t) in pairs {
                        let function = cell.ports.iter().find(|p| p.name == t).and_then(|p| p.function.clone());
                        let sets = self.read_timing(&cell, &t, function.as_deref(), tg, std::slice::from_ref(&f))?;
                        cell.arc_sets.extend(sets);
                    }
                }
            }
        }
        Ok(cell)
    }

    /// A pin group inside a bus, over the bit the bus made: the attributes it states replace the
    /// bus's (its group comes later in the file, and the reader applies groups in line order).
    fn override_port(&self, port: &mut Port, pg: &Group) {
        let own = self.read_port(&port.name, pg);
        if pg.attr_text("direction").is_some() {
            port.direction = own.direction;
        }
        let caps = ["capacitance", "rise_capacitance", "fall_capacitance", "rise_capacitance_range", "fall_capacitance_range"];
        if caps.iter().any(|a| pg.attr_text(a).is_some() || pg.complex_attr(a).is_some()) {
            port.capacitance = own.capacitance;
        }
        if pg.attr_text("clock").is_some() {
            port.is_clock = own.is_clock;
        }
        for (field, value, present) in [
            (&mut port.function, own.function, pg.attr_text("function").is_some()),
            (&mut port.three_state, own.three_state, pg.attr_text("three_state").is_some()),
        ] {
            if present {
                *field = value;
            }
        }
        if own.max_transition.is_some() {
            port.max_transition = own.max_transition;
        }
        if own.max_capacitance.is_some() {
            port.max_capacitance = own.max_capacitance;
        }
        if own.fanout_load.is_some() {
            port.fanout_load = own.fanout_load;
        }
        if own.max_fanout.is_some() {
            port.max_fanout = own.max_fanout;
        }
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
        let function = pg.attr_text("function");
        let max_transition = pg.attr_float("max_transition").map(|v| v * self.time_scale);
        let max_capacitance = pg.attr_float("max_capacitance").map(|v| v * self.cap_scale);
        let fanout_load = pg.attr_float("fanout_load");
        let max_fanout = pg.attr_float("max_fanout");
        let three_state = pg.attr_text("three_state");
        Port { name: name.to_string(), direction, is_clock, capacitance, function, max_transition, max_capacitance, fanout_load, max_fanout, three_state }
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
    /// `three_state_enable*` / `three_state_disable*`: an edge the levelization searches through.
    /// Their delay models are not read (no arcs): a caller that would time a tristate driver must
    /// refuse.
    TristateEnable,
    TristateDisable,
    Setup,
    Hold,
    Recovery,
    Removal,
    /// Pulse width, period, tristate, non-sequential checks, … — read, never timed here.
    Other,
}

impl Role {
    /// `TimingRole::isTimingCheck` for the roles read with arcs. `Other` carries no arcs, so its
    /// answer never reaches a value; the tristate roles are not checks (they carry no arcs either:
    /// tristate drivers are not timed here).
    pub fn is_timing_check(self) -> bool {
        matches!(self, Role::Setup | Role::Hold | Role::Recovery | Role::Removal | Role::Other)
    }
}

/// A timing model: a gate's delay and slew tables, or a check's constraint table.
#[derive(Debug, Clone, PartialEq)]
pub enum Model {
    Gate(GateModel),
    Check(Table),
}

/// `clear_preset_var1/2` as `getAttrLogicValue` reads them: `L` zero, `H` one, anything else —
/// `X`, an unrecognized value, or no attribute — unknown.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum LogicValue {
    Zero,
    One,
    #[default]
    Unknown,
}

impl LogicValue {
    fn read(text: Option<String>) -> LogicValue {
        match text.as_deref() {
            Some("L") => LogicValue::Zero,
            Some("H") => LogicValue::One,
            _ => LogicValue::Unknown,
        }
    }
}

/// One `ff` (register) or `latch` group as `LibertyReader::makeSequentials` makes it: its
/// clock (`clocked_on` / `enable`), data (`next_state` / `data_in`), `clear` and `preset`
/// expressions as text (an empty attribute is none), the two `clear_preset_var`s, and its output
/// and inverted-output internal port names (the group's first and second parameters).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Seq {
    pub is_register: bool,
    pub clock: Option<String>,
    pub data: Option<String>,
    pub clear: Option<String>,
    pub preset: Option<String>,
    pub clear_preset_var1: LogicValue,
    pub clear_preset_var2: LogicValue,
    pub output: Option<String>,
    pub output_inv: Option<String>,
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

/// The ports a port group's attributes and timing apply to: a pin, or a whole bus.
enum PortRef {
    Pin(String),
    Bus(String),
}

/// `readBusTypes`: the `type` groups under a library or a cell, name -> (bit_from, bit_to).
fn read_bus_types(g: &Group) -> BTreeMap<String, (i32, i32)> {
    let mut out = BTreeMap::new();
    for tg in g.groups_of("type") {
        let int = |a: &str| tg.attr_float(a).map(|v| v as i32);
        if let (Some(name), Some(from), Some(to)) = (tg.name(), int("bit_from"), int("bit_to")) {
            out.insert(name, (from, to));
        }
    }
    out
}

/// `makeBusPortBits`: `name[i]` for i from `from` to `to`, counting up or down, with each index.
fn bus_bits(name: &str, from: i32, to: i32) -> Vec<(String, i32)> {
    let idx: Vec<i32> = if from < to { (from..=to).collect() } else { (to..=from).rev().collect() };
    idx.into_iter().map(|i| (format!("{name}[{i}]"), i)).collect()
}

/// `FuncExpr::bitSubExpr(k)` on an expression's text: each bus named in it becomes its member at
/// position `k`; a bit or a plain pin stays as written.
fn bit_sub_expr(expr: &str, k: usize, buses: &[(String, Vec<String>)]) -> Result<String, String> {
    let chars: Vec<char> = expr.chars().collect();
    let mut out = String::new();
    let mut i = 0;
    while i < chars.len() {
        if chars[i].is_ascii_alphanumeric() || chars[i] == '_' {
            let start = i;
            while i < chars.len() && (chars[i].is_ascii_alphanumeric() || chars[i] == '_') {
                i += 1;
            }
            let word: String = chars[start..i].iter().collect();
            match buses.iter().find(|(b, _)| *b == word) {
                Some((_, members)) if chars.get(i) != Some(&'[') => {
                    out.push_str(members.get(k).ok_or_else(|| format!("{word} has no member {k}"))?);
                }
                _ => out.push_str(&word),
            }
        } else {
            out.push(chars[i]);
            i += 1;
        }
    }
    Ok(out)
}

/// `LibertyReader::makeTimingArcs(cell, from_name, to_port, one_to_one)`: the (from, to) bit pairs,
/// in the order the reader makes them. One to one and a bus to one: every from bit; one to a bus:
/// every to bit; a bus to a bus: one to one aligned at the LAST bits (`related_pin`), or every
/// pair, from bits outer (`related_bus_pins`).
fn expand_pairs(from: &[String], to: &[String], to_is_bus: bool, one_to_one: bool) -> Vec<(String, String)> {
    if from.len() > 1 && to_is_bus {
        if one_to_one {
            let n = from.len().min(to.len());
            let (f, t) = (&from[from.len() - n..], &to[to.len() - n..]);
            return f.iter().cloned().zip(t.iter().cloned()).collect();
        }
        return from.iter().flat_map(|f| to.iter().map(move |t| (f.clone(), t.clone()))).collect();
    }
    if to_is_bus {
        return from.first().map(|f| to.iter().map(|t| (f.clone(), t.clone())).collect()).unwrap_or_default();
    }
    from.iter().map(|f| (f.clone(), to[0].clone())).collect()
}

impl Cell {
    /// A bus's member bits, in member order (empty when there is no such bus).
    pub fn bus_members(&self, bus: &str) -> &[String] {
        self.buses.iter().find(|(b, _)| b == bus).map_or(&[], |(_, m)| m.as_slice())
    }

    /// `PortNameBitIterator`: a bus name -> its bits; `name[a:b]` -> bits a to b; else the name.
    pub fn port_bits(&self, name: &str) -> Vec<String> {
        let members = self.bus_members(name);
        if !members.is_empty() {
            return members.to_vec();
        }
        if let Some((base, rest)) = name.split_once('[') {
            if let Some((a, b)) = rest.strip_suffix(']').and_then(|r| r.split_once(':')) {
                if let (Ok(a), Ok(b)) = (a.trim().parse::<i32>(), b.trim().parse::<i32>()) {
                    return bus_bits(base, a, b).into_iter().map(|(n, _)| n).collect();
                }
            }
        }
        vec![name.to_string()]
    }
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
    fn read_timing(&self, cell: &Cell, to: &str, function: Option<&str>, tg: &Group, related: &[String]) -> Result<Vec<ArcSet>, String> {
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
        if related.is_empty() {
            return Ok(Vec::new());
        }
        let mut out = Vec::new();
        for from in related {
            let in_clock = seq.is_some_and(|(_, _, clk)| identifiers(clk).contains(from));
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
                        Some((_, false, clk)) if identifiers(clk).contains(from) => Role::LatchEnToQ,
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
                t if t.starts_with("three_state_enable") => ArcSet { from: from.clone(), to: to.to_string(), role: Role::TristateEnable, timing_type: timing_type.clone(), cond: cond.clone(), arcs: Vec::new() },
                t if t.starts_with("three_state_disable") => ArcSet { from: from.clone(), to: to.to_string(), role: Role::TristateDisable, timing_type: timing_type.clone(), cond: cond.clone(), arcs: Vec::new() },
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

    const BUS_LIB: &str = r#"library (b) { time_unit : "1ns"; capacitive_load_unit (1, pf);
      type (b4) { base_type : array; data_type : bit; bit_width : 4; bit_from : 3; bit_to : 0; }
      type (b2) { base_type : array; data_type : bit; bit_width : 2; bit_from : 1; bit_to : 0; }
      cell (bus4) {
        bus (in) { bus_type : b4; direction : input; capacitance : 0.0077;
          pin (in[0]) { capacitance : 0.002; } }
        pin (en) { direction : input; capacitance : 0.001; }
        bus (out) { bus_type : b4; direction : output; function : "in";
          timing () { related_pin : "in"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("1"); } cell_fall (scalar) { values ("2"); }
            rise_transition (scalar) { values ("1"); } fall_transition (scalar) { values ("2"); } }
          timing () { related_pin : "en"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("1"); } cell_fall (scalar) { values ("2"); }
            rise_transition (scalar) { values ("1"); } fall_transition (scalar) { values ("2"); } } }
        bus (q) { bus_type : b2; direction : output; function : "in";
          timing () { related_pin : "in"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("1"); } rise_transition (scalar) { values ("1"); } }
          timing () { related_bus_pins : "in"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("1"); } rise_transition (scalar) { values ("1"); } } } } }"#;

    // Rules (makeBusPort, makeBusPortBits, LibertyPort setters): a bus is its bits `name[i]`, bit_from
    // to bit_to; the bus's attributes reach every bit; a pin group inside the bus states its own
    // over them; the function is sliced by member OFFSET (out[3] = in[3], the first of each).
    #[test]
    fn a_bus_is_its_bits_with_the_bus_attributes() {
        let l = Library::read(&parse(BUS_LIB).unwrap()).unwrap();
        let c = &l.cells["bus4"];
        let names: Vec<&str> = c.ports.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(names, ["in[3]", "in[2]", "in[1]", "in[0]", "en", "out[3]", "out[2]", "out[1]", "out[0]", "q[1]", "q[0]"]);
        assert_eq!(c.port("in[2]").unwrap().capacitance[RISE][MAX], 0.0077e-12);
        assert_eq!(c.port("in[0]").unwrap().capacitance[RISE][MAX], 0.002e-12, "the inner pin's own");
        assert_eq!(c.port("in[0]").unwrap().direction, Direction::Input, "kept from the bus");
        assert_eq!(c.port("out[3]").unwrap().function.as_deref(), Some("in[3]"));
        assert_eq!(c.port("out[0]").unwrap().function.as_deref(), Some("in[0]"));
        assert_eq!(c.port("q[1]").unwrap().function.as_deref(), Some("in[3]"), "offset 0 of in");
    }

    // Rules (makeTimingArcs): related_pin bus -> bus is one to one, aligned at the LAST bits when
    // the sizes differ; one -> bus is every bit; related_bus_pins is the cross product, from bits outer.
    #[test]
    fn bus_timing_expands_by_the_readers_rules() {
        let l = Library::read(&parse(BUS_LIB).unwrap()).unwrap();
        let pairs = |to_bus: &str, role_from: &str| -> Vec<(String, String)> {
            l.cells["bus4"].arc_sets.iter().filter(|a| a.to.starts_with(to_bus) && a.from.starts_with(role_from)).map(|a| (a.from.clone(), a.to.clone())).collect()
        };
        let p = |v: &[(&str, &str)]| v.iter().map(|(a, b)| (a.to_string(), b.to_string())).collect::<Vec<_>>();
        assert_eq!(pairs("out", "in"), p(&[("in[3]", "out[3]"), ("in[2]", "out[2]"), ("in[1]", "out[1]"), ("in[0]", "out[0]")]));
        assert_eq!(pairs("out", "en"), p(&[("en", "out[3]"), ("en", "out[2]"), ("en", "out[1]"), ("en", "out[0]")]));
        let q = pairs("q", "in");
        assert_eq!(&q[..2], p(&[("in[1]", "q[1]"), ("in[0]", "q[0]")]).as_slice(), "the last two bits of in");
        assert_eq!(q.len(), 2 + 8, "then the 4 x 2 cross product");
        assert_eq!(&q[2..4], p(&[("in[3]", "q[1]"), ("in[3]", "q[0]")]).as_slice());
    }

    // Rule (LibertyReader: three_state): an OUTPUT with a non-empty three_state is tristate; a
    // plain output is not, nor is an empty three_state.
    #[test]
    fn an_output_with_three_state_is_tristate() {
        let lib = r#"library (t) { time_unit : "1ns"; capacitive_load_unit (1, pf);
          cell (EB) { pin (A) { direction : input; capacitance : 0.001; }
                      pin (TE_B) { direction : input; capacitance : 0.001; }
                      pin (Z) { direction : output; function : "A"; three_state : "TE_B"; } }
          cell (B) { pin (A) { direction : input; capacitance : 0.001; }
                     pin (X) { direction : output; function : "A"; three_state : ""; } } }"#;
        let l = Library::read(&parse(lib).unwrap()).unwrap();
        assert!(l.cells["EB"].port("Z").unwrap().is_any_tristate());
        assert!(!l.cells["EB"].port("A").unwrap().is_any_tristate());
        assert!(!l.cells["B"].port("X").unwrap().is_any_tristate());
    }

    // Rule: <1|10|100><prefix><suffix>, both factors f32; an unknown
    // multiplier only warns and counts as 1.
    #[test]
    fn units_are_multiplier_times_prefix() {
        assert_eq!(unit_scale("1ns", "s"), 1e-9f32);
        assert_eq!(unit_scale("10ps", "s"), 1e-12f32 * 10.0);
        assert_eq!(unit_scale("s", "s"), 1.0);
        assert_eq!(unit_scale("2ns", "s"), 1e-9f32, "an unknown multiplier is 1");
    }

    // Rules (LibertyReader::readCellAttributes, readPortAttributes): area / cell_footprint /
    // user_function_class / dont_use; booleans compare case-insensitively; `pad_cell` is read
    // after `is_pad`; max_transition × time unit, max_capacitance × cap unit; a 0
    // default_max_transition is kept.
    #[test]
    fn cell_and_port_attributes_are_read_with_their_units() {
        let text = r#"library (l) { time_unit : "1ns" ; capacitive_load_unit (1, ff) ;
            default_max_transition : 0 ; default_fanout_load : 1 ;
            cell (B) { area : 2.5 ; cell_footprint : "buf" ; dont_use : TRUE ; is_pad : true ; pad_cell : false ;
              pin (A) { direction : input ; capacitance : 1 ; fanout_load : 2 ; }
              pin (Z) { direction : output ; function : "(A)" ; max_transition : 0.5 ; max_capacitance : 30 ; }
            } }"#;
        let lib = Library::read(&parse(text).unwrap()).unwrap();
        assert_eq!(lib.default_max_transition, Some(0.0));
        assert_eq!(lib.default_fanout_load, Some(1.0));
        let c = &lib.cells["B"];
        assert_eq!((c.area, c.footprint.as_str(), c.dont_use, c.is_pad), (2.5, "buf", true, false));
        let z = c.port("Z").unwrap();
        assert_eq!(z.max_transition, Some(0.5 * 1e-9f32));
        assert_eq!(z.max_capacitance, Some(30.0 * 1e-15f32));
        assert_eq!(c.port("A").unwrap().fanout_load, Some(2.0));
    }

    // Rules (LibertyCell::bufferPorts, isBuffer, hasBufferFunc): one input and one output in
    // port order; the output function is that input port; not a level shifter or pad.
    #[test]
    fn a_buffer_is_one_input_one_output_whose_function_is_the_input() {
        let text = r#"library (l) {
            cell (BUF) { pin (A) { direction : input ; } pin (Z) { direction : output ; function : "A" ; } }
            cell (INV) { pin (A) { direction : input ; } pin (ZN) { direction : output ; function : "!A" ; } }
            cell (AND) { pin (A) { direction : input ; } pin (B) { direction : input ; } pin (Z) { direction : output ; function : "A&B" ; } }
            cell (LS) { is_level_shifter : true ; pin (A) { direction : input ; } pin (Z) { direction : output ; function : "A" ; } }
            }"#;
        let lib = Library::read(&parse(text).unwrap()).unwrap();
        assert!(lib.cells["BUF"].is_buffer());
        assert!(!lib.cells["INV"].is_buffer());
        assert!(lib.cells["AND"].buffer_ports().is_none(), "a second input means no buffer ports");
        assert!(!lib.cells["LS"].is_buffer());
        assert!(function_is_port(" ((A)) ", "A") && !function_is_port("A B", "A"));
    }

    // Rules (LibertyCell::isInverter / hasInverterFunc; Sim's constant functions): the output is
    // the input inverted, written `!A`, `A'` or parenthesized; a constant is 0 / 1 / 1'b0 / 1'b1.
    #[test]
    fn inverter_and_constant_functions() {
        for f in ["!A", " !(A) ", "A'", "(A)'", "(!A)"] {
            assert!(function_is_not_port(f, "A"), "{f}");
        }
        for f in ["A", "!B", "A&B", "!A&B"] {
            assert!(!function_is_not_port(f, "A"), "{f}");
        }
        for f in ["0", "1", "1'b0", "(1'b1)"] {
            assert!(function_is_constant(f), "{f}");
        }
        assert!(!function_is_constant("A"));
        let lib = Library::read(&parse(r#"library (l) {
            cell (INV) { pin (A) { direction : input ; } pin (ZN) { direction : output ; function : "!A" ; } }
            cell (BUF) { pin (A) { direction : input ; } pin (Z) { direction : output ; function : "A" ; } }
            }"#).unwrap()).unwrap();
        assert!(lib.cells["INV"].is_inverter() && !lib.cells["BUF"].is_inverter());
    }

    // Rules (GateTableModel::driveResistance via maxCapSlew, LibertyPort::driveResistance): the
    // slew at input slew 0 and the LAST cap axis value, over that cap; the max positive drive
    // over the arcs into the port.
    #[test]
    fn drive_resistance_is_the_slew_at_the_largest_cap_over_that_cap() {
        let text = r#"library (l) { time_unit : "1ns" ; capacitive_load_unit (1, pf) ;
            lu_table_template (t) { variable_1 : input_net_transition ; variable_2 : total_output_net_capacitance ;
              index_1 ("0, 1") ; index_2 ("0, 2") ; }
            cell (BUF) { pin (A) { direction : input ; }
              pin (Z) { direction : output ; function : "A" ;
                timing () { related_pin : "A" ; timing_sense : positive_unate ;
                  rise_transition (t) { values ("1, 3", "5, 9") ; }
                  fall_transition (t) { values ("1, 5", "5, 9") ; } } } } }"#;
        let lib = Library::read(&parse(text).unwrap()).unwrap();
        // rise: 3 ns / 2 pF; fall: 5 ns / 2 pF — the max.
        assert_eq!(lib.cells["BUF"].drive_resistance("Z"), (5.0 * 1e-9f32) / (2.0 * 1e-12f32));
        assert_eq!(lib.cells["BUF"].drive_resistance("A"), 0.0);
    }

    // Rules (LibertyReader::makeSequentials): ff then latch; clock / data from clocked_on /
    // next_state (enable / data_in); outputs are the group parameters; clear_preset_var L / H /
    // anything else unknown; an empty attribute is none.
    #[test]
    fn sequentials_are_read_as_the_reference_makes_them() {
        let lib = Library::read(
            &crate::liberty_parse::parse(
                r#"library (t) { cell (F) {
                  latch (IL) { enable : "G" ; data_in : "D" ; }
                  ff (IQ, IQN) { clocked_on : "CK" ; next_state : "D" ; clear : "!RN" ; clear_preset_var1 : L ; clear_preset_var2 : X ; preset : "" ; }
                  pin (D) { direction : input ; } pin (CK) { direction : input ; clock : true ; } pin (G) { direction : input ; }
                  pin (RN) { direction : input ; } pin (Q) { direction : output ; function : "IQ" ; } } }"#,
            )
            .unwrap(),
        )
        .unwrap();
        let c = &lib.cells["F"];
        assert_eq!(c.seqs.len(), 2);
        let ff = &c.seqs[0];
        assert!(ff.is_register);
        assert_eq!((ff.clock.as_deref(), ff.data.as_deref(), ff.clear.as_deref(), ff.preset.as_deref()), (Some("CK"), Some("D"), Some("!RN"), None));
        assert_eq!((ff.clear_preset_var1, ff.clear_preset_var2), (LogicValue::Zero, LogicValue::Unknown));
        assert_eq!((ff.output.as_deref(), ff.output_inv.as_deref()), (Some("IQ"), Some("IQN")));
        assert!(!c.seqs[1].is_register);
        assert_eq!((c.seqs[1].clock.as_deref(), c.seqs[1].output_inv.as_deref()), (Some("G"), None));
        assert!(!c.has_seq_bank);
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
