// SPDX-License-Identifier: Apache-2.0
//! A design read by loom, as this timer reads it (feature `loom`): the netlist ([`netlist`]) and
//! the constraints ([`search_sdc_on`], [`sdc_env`]). Everything loom read that the timer cannot
//! represent is REFUSED with a reason, never dropped: a construct left out here would time a
//! different design than the one written. The Liberty is the timer's own reader's.
//!
//! The netlist is laid out in the order the reference network (OpenSTA's `ConcreteNetwork`, read
//! by its `VerilogReader`) gives it, because the graph is numbered from that order and an
//! equal-slack tie is decided by it:
//! - instances in NAME order (the top's children are a `std::map` by name), which is the order
//!   the graph makes their vertices and gate edges in;
//! - ports in HEADER order across directions, a bus's left index first (`linkNetwork`);
//! - a net's port FIRST, then its pins in REVERSE connection order (`ConcreteNet::addPin`
//!   prepends), connections made statement by statement as the module is written. The port is the
//!   net's term, and `visitConnectedPins` visits the terms before the pins — the order wire edges
//!   are made in and a net's pin capacitances are summed in (in float: the order is the last
//!   bit);
//! - nets in the order of their driver — leaf instances' drivers in instance order, each cell's
//!   ports in library order, then the input ports — because the reference makes wire edges per
//!   driver (`makeWireEdges`), and this timer makes them per net.

use std::collections::HashMap;

use vyges_loom::netlist::CompactDesign;

use crate::liberty::{Direction, Library};
use crate::netlist::{inst, Conn, Net, Netlist, PortDir};

/// The top module of `design` as the timer's netlist, its cells linked in `libs` (the first
/// library that has a cell wins, as the reference links). Err names the first construct the
/// timer cannot represent.
pub fn netlist(design: &CompactDesign, libs: &[Library]) -> Result<Netlist, String> {
    let m = design.top();
    let name = design.str(m.name);
    // What loom read and the timer has no place for.
    if m.has_bundle_ports() {
        return Err(format!("module {name}: a port bundle (`.X(expr)` in the header) is not modelled"));
    }
    if let Some((a, b)) = m.aliases.first() {
        return Err(format!("module {name}: assign {} = {} (a net with two names) is not modelled", design.str(*a), design.str(*b)));
    }
    if m.skipped_assigns > 0 {
        return Err(format!("module {name}: {} assigns of unequal widths (reference warning 203) are not modelled", m.skipped_assigns));
    }
    if m.unbound_pins > 0 {
        return Err(format!("module {name}: {} unbound instance pins are not modelled", m.unbound_pins));
    }
    if m.behavioural > 0 {
        return Err(format!("module {name}: {} behavioural constructs: not a structural netlist", m.behavioural));
    }
    if let Some(p) = m.undirected.first() {
        return Err(format!("module {name}: port {} has no direction (reference warning 166); not modelled", design.str(*p)));
    }
    if let Some((n, _)) = m.constants.first() {
        return Err(format!("module {name}: supply net {} (a constant net) is not modelled", design.str(*n)));
    }
    let find_cell = |cell: &str| libs.iter().find_map(|l| l.cells.get(cell));
    // ports, header order
    let dir_of: HashMap<_, PortDir> = m
        .inputs
        .iter()
        .map(|s| (*s, PortDir::Input))
        .chain(m.outputs.iter().map(|s| (*s, PortDir::Output)))
        .chain(m.inouts.iter().map(|s| (*s, PortDir::Inout)))
        .collect();
    let ports: Vec<(String, PortDir)> = m.port_bits().iter().map(|s| (design.str(*s).to_string(), dir_of[s])).collect();
    // instances, name order; `order[k]` is the k-th instance as written
    let mut by_name: Vec<usize> = (0..m.insts.len()).collect();
    by_name.sort_by(|&a, &b| design.str(m.insts[a].name).cmp(design.str(m.insts[b].name)));
    let mut index = vec![0; m.insts.len()];
    for (k, &w) in by_name.iter().enumerate() {
        index[w] = k;
    }
    let mut insts = Vec::with_capacity(m.insts.len());
    for &w in &by_name {
        let x = &m.insts[w];
        let (iname, cname) = (design.str(x.name), design.str(x.cell));
        if design.module(cname).is_some() {
            return Err(format!("instance {iname}: module {cname} is hierarchy, which is not modelled (flatten first)"));
        }
        if find_cell(cname).is_none() {
            return Err(format!("instance {iname}: cell {cname} is in no library read"));
        }
        if let Some((p, _)) = m.ties(x).first() {
            return Err(format!("instance {iname}: pin {} tied to a constant is not modelled", design.str(*p)));
        }
        insts.push(inst(iname, cname));
    }
    // each net's pins in connection order, nets in first-connection order
    let mut nets: Vec<(String, Vec<Conn>)> = Vec::new();
    let mut net_at: HashMap<&str, usize> = HashMap::new();
    for (w, x) in m.insts.iter().enumerate() {
        let (iname, cname) = (design.str(x.name), design.str(x.cell));
        let cell = find_cell(cname).expect("every instance's cell was found above");
        for (p, n) in m.conns(x) {
            let (pin, net) = (design.str(*p), design.str(*n));
            if pin.is_empty() {
                return Err(format!("instance {iname}: a positional connection loom could not name is not modelled"));
            }
            if cell.port(pin).is_none() {
                return Err(format!("instance {iname}: cell {cname} has no port {pin}"));
            }
            let k = *net_at.entry(net).or_insert_with(|| {
                nets.push((net.to_string(), Vec::new()));
                nets.len() - 1
            });
            nets[k].1.push(Conn::Inst(index[w], pin.to_string()));
        }
    }
    // `ConcreteNet::addPin` prepends: the net lists its pins last connected first
    for (_, pins) in &mut nets {
        pins.reverse();
    }
    // a port is the term of the net of its name (`ensureNetBinding(port bit)`), visited before
    // the pins (`ConcreteNetwork::visitConnectedPins`)
    for (i, (pname, _)) in ports.iter().enumerate() {
        let k = *net_at.entry(pname.as_str()).or_insert_with(|| {
            nets.push((pname.clone(), Vec::new()));
            nets.len() - 1
        });
        nets[k].1.insert(0, Conn::Port(i));
    }
    // nets in their driver's order (`makeWireEdges`); a net with no driver after, as connected
    let mut rank: Vec<usize> = vec![usize::MAX; nets.len()];
    let mut next = 0;
    let mut pins_of: Vec<Vec<(&str, usize)>> = vec![Vec::new(); insts.len()];
    for (k, (_, pins)) in nets.iter().enumerate() {
        for c in pins {
            if let Conn::Inst(i, p) = c {
                pins_of[*i].push((p.as_str(), k));
            }
        }
    }
    for (i, x) in insts.iter().enumerate() {
        let cell = find_cell(&x.1).expect("every instance's cell was found above");
        for port in cell.ports.iter().filter(|p| matches!(p.direction, Direction::Output | Direction::Tristate | Direction::Bidirect)) {
            for &(_, k) in pins_of[i].iter().filter(|(p, _)| *p == port.name) {
                if rank[k] == usize::MAX {
                    rank[k] = next;
                    next += 1;
                }
            }
        }
    }
    for (i, (pname, dir)) in ports.iter().enumerate() {
        if matches!(dir, PortDir::Input | PortDir::Inout) {
            let k = net_at[pname.as_str()];
            debug_assert!(nets[k].1.contains(&Conn::Port(i)));
            if rank[k] == usize::MAX {
                rank[k] = next;
                next += 1;
            }
        }
    }
    let mut order: Vec<usize> = (0..nets.len()).collect();
    order.sort_by_key(|&k| rank[k]); // stable: undriven nets keep their first-connection order
    let nets = order.into_iter().map(|k| std::sync::Arc::new(Net { name: nets[k].0.clone(), pins: std::mem::take(&mut nets[k].1) })).collect();
    Ok(Netlist { insts, ports, nets })
}

// ---- the constraints ---------------------------------------------------------------------------
// loom keeps an SDC as written (values in the library's units, objects as named); the timer keeps
// seconds and farads in `f32` through the user unit (`Unit::userToSta`), and only what it models.
// Each function refuses, with the command and flag named, what it does not map.

/// The design as an SDC object query sees it (`Sdc::parse_with`): the netlist's ports (direction
/// as the reference names it), instances, and instance pins with the direction of their liberty
/// port and their net — so `[get_ports {d[*]}]`, `[all_inputs]` and the like name what the
/// reference's queries name rather than staying unexpanded.
pub fn sdc_design(nl: &Netlist, libs: &[Library]) -> vyges_loom::sdc::SdcDesign {
    let dir_name = |d: &Direction| match d {
        Direction::Input => "input",
        Direction::Output => "output",
        Direction::Tristate => "tristate",
        Direction::Bidirect => "bidirect",
        Direction::Internal => "internal",
        Direction::Unknown => "unknown",
    };
    let ports = nl
        .ports
        .iter()
        .map(|(n, d)| (n.clone(), match d { PortDir::Input => "input", PortDir::Output => "output", PortDir::Inout => "bidirect" }.to_string()))
        .collect();
    let mut pins = Vec::new();
    for net in &nl.nets {
        for c in &net.pins {
            if let Conn::Inst(i, p) = c {
                let dir = link_cell(libs, &nl.insts[*i].1).and_then(|cell| cell.port(p)).map_or("unknown", |port| dir_name(&port.direction));
                pins.push((*i, p.clone(), dir.to_string(), Some(net.name.clone())));
            }
        }
    }
    vyges_loom::sdc::SdcDesign { ports, insts: nl.insts.iter().map(|i| (**i).clone()).collect(), pins }
}

/// `[get_ports P …]` against the netlist: each pattern's ports (`object_matches`, the reference's
/// bus-aware match), in port order, each once — loom keeps a query's patterns as written. A
/// pattern no port matches names nothing (the reference warns and applies the command to the
/// ports it found), said in `notes`. The every-input collection `*INPUTS*` is kept.
fn ports_matching(names: &[String], nl: &Netlist, cmd: &str, notes: &mut Vec<String>) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for n in names {
        if n == "*INPUTS*" {
            out.push(n.clone());
            continue;
        }
        let hits: Vec<&String> = nl.ports.iter().map(|(p, _)| p).filter(|p| vyges_loom::sdc::object_matches(n, p)).collect();
        if hits.is_empty() {
            notes.push(format!("{cmd} [get_ports {n}]: no port matches; nothing applied"));
        }
        for h in hits {
            if !out.contains(h) {
                out.push(h.clone());
            }
        }
    }
    out
}

/// Whether a name is a pattern (`*` / `?`) rather than one object.
fn is_pattern(name: &str) -> bool {
    name.contains(['*', '?'])
}

/// The constraints with every port pattern expanded to the ports it names ([`ports_matching`]):
/// I/O delays, loads and input transitions, driving cells and clock sources. A PIN pattern is
/// refused (pins are read by name only).
fn expanded_ports(s: &vyges_loom::sdc::Sdc, nl: &Netlist, notes: &mut Vec<String>) -> Result<vyges_loom::sdc::Sdc, String> {
    let mut s = s.clone();
    for (list, cmd) in [(&mut s.input_delays, "set_input_delay"), (&mut s.output_delays, "set_output_delay")] {
        for d in list.iter_mut().filter(|d| d.accessor == "get_ports" && !d.default) {
            d.ports = ports_matching(&d.ports, nl, cmd, notes);
        }
    }
    for e in s.env.iter_mut().filter(|e| e.accessor == "get_ports") {
        e.objects = ports_matching(&e.objects, nl, &e.cmd, notes);
    }
    for d in &mut s.driving_cells {
        d.objects = ports_matching(&d.objects, nl, "set_driving_cell", notes);
    }
    for c in &mut s.clocks {
        if c.sources.iter().any(|x| is_pattern(x)) {
            c.sources = ports_matching(&c.sources, nl, "create_clock", notes);
        }
    }
    let pins = s.path_delays.iter().flat_map(|d| d.objects.iter().flat_map(|(_, o)| o.names.iter())).chain(s.clock_latencies.iter().flat_map(|l| l.objects.iter()));
    if let Some(p) = pins.into_iter().find(|p| is_pattern(p)) {
        return Err(format!("the pin pattern {p}: not modelled (pins are read by name)"));
    }
    Ok(s)
}

/// The constraints a standalone timer applies, from loom's SDC: the clocks, I/O delays and path
/// delays the search reads ([`search_sdc_on`], clocks ideal unless `set_propagated_clock` names
/// them) and the loads, slews and drives delay calculation reads ([`sdc_env`]) — or Err naming
/// the first command that would move a slack and that the timer does not model. Design-rule
/// limits, and commands that move no slack, are returned as notes.
pub fn constraints(s: &vyges_loom::sdc::Sdc, nl: &Netlist, libs: &[Library]) -> Result<(crate::sdc::Sdc, crate::graph::SdcEnv, Vec<String>), String> {
    // everything loom read and carries in no record (`Sdc::not_applied`, the rejected commands
    // and unevaluated queries among them): refused if it can move a slack, else a note
    let mut notes = Vec::new();
    for n in s.not_applied() {
        if n.affects_timing {
            return Err(format!("{} ({}): not modelled by this timer", n.command, n.why));
        }
        notes.push(format!("{}: {} (no slack moves with it)", n.command, n.why));
    }
    let refused: [(&str, bool); 11] = [
        ("set_clock_latency on the clock source or without pins", s.clock_latencies.iter().any(|l| l.accessor != "get_pins")),
        ("set_clock_transition", !s.clock_transitions.is_empty()),
        ("set_case_analysis", !s.case_values.is_empty()),
        ("set_logic_zero / set_logic_one / set_logic_dc", !s.logic_values.is_empty()),
        ("set_disable_timing", !s.disabled_timing.is_empty()),
        ("set_sense / set_clock_sense", !s.senses.is_empty()),
        ("set_disable_clock_gating_check", !s.disabled_gating_checks.is_empty()),
        ("set_timing_derate", !s.derates.is_empty()),
        ("set_clock_groups", !s.clock_groups.is_empty() || !s.async_groups.is_empty()),
        ("group_path / set_path_margin", !s.other_exceptions.is_empty()),
        ("a command kept as written", !s.kept.is_empty()),
    ];
    if let Some((what, _)) = refused.iter().find(|(_, set)| *set) {
        return Err(format!("{what}: not modelled by this timer"));
    }
    // design-rule limits are carried, and move no slack: read, not checked
    if s.max_fanout.is_some() || !s.max_fanout_objects.is_empty() {
        notes.push("set_max_fanout: a design-rule limit, read and not checked".into());
    }
    if s.max_transition.is_some() || !s.max_transition_objects.is_empty() {
        notes.push("set_max_transition: a design-rule limit, read and not checked".into());
    }
    if s.max_capacitance.is_some() || !s.max_capacitance_objects.is_empty() {
        notes.push("set_max_capacitance: a design-rule limit, read and not checked".into());
    }
    let mut s = expanded_ports(s, nl, &mut notes)?;
    let lib0 = libs.first().ok_or("constraints before any liberty library")?;
    // uncertainties are applied per clock below; the search's design-wide refusal is for rsz
    let uncertainties = std::mem::take(&mut s.uncertainties);
    (s.setup_uncertainty, s.hold_uncertainty) = (0.0, 0.0);
    // false paths go to the search with the path delays (one priority ranks them); the search's
    // design-wide refusal of exceptions is for rsz
    let exceptions = std::mem::take(&mut s.exceptions);
    let mut sdc = search_sdc_on(&s, nl, lib0.time_scale, false, false)?;
    clock_uncertainties(&uncertainties, &mut sdc.clocks, lib0.time_scale)?;
    let falses = timing_exceptions(&exceptions, &s, nl)?;
    if !falses.is_empty() && !sdc.path_delays.is_empty() {
        return Err("set_false_path / set_multicycle_path with set_max_delay / set_min_delay: not modelled (the ids across them are the order they were made in)".into());
    }
    sdc.path_delays.extend(falses);
    let env = sdc_env(&s, libs, nl)?;
    Ok((sdc, env, notes))
}

/// `set_clock_uncertainty` as the reference's command sets it, in file order, the later value
/// replacing the earlier for the same target and min/max (`ClockUncertainties::setValue`):
/// `-setup` alone sets the max (setup) value, `-hold` alone the min, else both. On clocks
/// (`get_clocks`, `all_clocks`, or a bare clock name — `parse_clk_port_pin_arg` takes a clock
/// first): the clock's own. With `-from`/`-to` (`-rise_from`, `-fall_to`, … or `-rise`/`-fall`
/// after `-to`): each pair's inter-clock value. On pins or ports: refused.
fn clock_uncertainties(us: &[vyges_loom::sdc::ClockUncertainty], clocks: &mut [crate::sdc::Clock], time_scale: f32) -> Result<(), String> {
    let names: Vec<String> = clocks.iter().map(|c| c.name.clone()).collect();
    let clocks_of = |o: &vyges_loom::sdc::Objects| -> Result<Vec<usize>, String> {
        let names: Vec<usize> = match o.accessor.as_str() {
            "all_clocks" => (0..names.len()).collect(),
            "get_clocks" | "" => {
                let mut out = Vec::new();
                for n in &o.names {
                    let hits: Vec<usize> = (0..names.len()).filter(|&k| vyges_loom::sdc::object_matches(n, &names[k])).collect();
                    if hits.is_empty() && o.accessor.is_empty() {
                        return Err(format!("set_clock_uncertainty on {n}: a pin or port uncertainty, not modelled (clocks are)"));
                    }
                    for k in hits {
                        if !out.contains(&k) {
                            out.push(k);
                        }
                    }
                }
                out
            }
            other => return Err(format!("set_clock_uncertainty on [{other} …]: not modelled (clocks are)")),
        };
        Ok(names)
    };
    for u in us {
        let mm: &[usize] = match (u.setup, u.hold) {
            (true, false) => &[crate::liberty::MAX],
            (false, true) => &[crate::liberty::MIN],
            _ => &[crate::liberty::MIN, crate::liberty::MAX],
        };
        let v = crate::sdc::user_to_sta(u.value, time_scale);
        if u.from.is_empty() && u.to.is_empty() {
            let mut targets = Vec::new();
            for o in &u.written {
                for k in clocks_of(o)? {
                    if !targets.contains(&k) {
                        targets.push(k);
                    }
                }
            }
            for k in targets {
                for &m in mm {
                    clocks[k].uncertainty[m] = Some(v);
                }
            }
            continue;
        }
        let rfs = |r: bool, f: bool| -> Vec<usize> {
            match (r, f) {
                (true, false) => vec![crate::liberty::RISE],
                (false, true) => vec![crate::liberty::FALL],
                _ => vec![crate::liberty::RISE, crate::liberty::FALL],
            }
        };
        let side = |keys: &[&str]| -> Vec<&vyges_loom::sdc::Objects> { u.written_sides.iter().filter(|(k, _)| keys.contains(&k.as_str())).map(|(_, o)| o).collect() };
        let has = |f: &str| u.flags.iter().any(|x| x == f);
        let from_key = u.written_sides.iter().map(|(k, _)| k.as_str()).find(|k| k.ends_with("from")).unwrap_or("-from");
        let to_key = u.written_sides.iter().map(|(k, _)| k.as_str()).find(|k| k.ends_with("to")).unwrap_or("-to");
        let src_rfs = rfs(from_key == "-rise_from", from_key == "-fall_from");
        let tgt_rfs = if to_key == "-to" { rfs(has("-rise"), has("-fall")) } else { rfs(to_key == "-rise_to", to_key == "-fall_to") };
        let mut froms = Vec::new();
        for o in side(&["-from", "-rise_from", "-fall_from"]) {
            froms.extend(clocks_of(&clock_side(o))?);
        }
        let mut tos = Vec::new();
        for o in side(&["-to", "-rise_to", "-fall_to"]) {
            tos.extend(clocks_of(&clock_side(o))?);
        }
        for &f in &froms {
            for &t in &tos {
                let i = match clocks[t].inter_uncertainties.iter().position(|x| x.from == f) {
                    Some(i) => i,
                    None => {
                        clocks[t].inter_uncertainties.push(crate::sdc::InterClockUncertainty { from: f, value: [[[None; 2]; 2]; 2] });
                        clocks[t].inter_uncertainties.len() - 1
                    }
                };
                for &sr in &src_rfs {
                    for &tr in &tgt_rfs {
                        for &m in mm {
                            clocks[t].inter_uncertainties[i].value[sr][tr][m] = Some(v);
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

/// Every `set_false_path` (`FalsePath`) and `set_multicycle_path` (`MultiCyclePath`) as the
/// reference makes it, in file order: `-from` and `-to` on ports or pins (patterns matched against
/// the netlist, as `get_ports` / `get_pins` do) or on the one clock. A false path of `-setup` alone
/// is the max paths', `-hold` alone the min paths', else one for each; a multicycle path is one
/// exception of `-setup`, `-hold` or neither (`set_multicycle_path` in `Sdc.tcl`: `-end` unless
/// `-hold` alone or `-start`). `-through`, the edge forms (`-rise_from`, `-rise`, …),
/// `-reset_path`, instances and several clocks are refused.
fn timing_exceptions(es: &[vyges_loom::sdc::Exception], s: &vyges_loom::sdc::Sdc, nl: &Netlist) -> Result<Vec<crate::sdc::PathDelay>, String> {
    use crate::liberty::{MAX, MIN};
    let mut pins: Vec<String> = Vec::new();
    for net in &nl.nets {
        for c in &net.pins {
            if let Conn::Inst(i, port) = c {
                pins.push(format!("{}/{port}", nl.insts[*i].0));
            }
        }
    }
    let mut out = Vec::new();
    for e in es {
        let multiplier = match e.kind {
            vyges_loom::sdc::ExcKind::FalsePath => None,
            vyges_loom::sdc::ExcKind::Multicycle(n) => Some(i32::try_from(n).map_err(|_| format!("set_multicycle_path {n}: out of range"))?),
        };
        let cmd = if multiplier.is_some() { "set_multicycle_path" } else { "set_false_path" };
        let allowed = |f: &str| matches!(f, "-setup" | "-hold") || (multiplier.is_some() && matches!(f, "-start" | "-end"));
        if let Some(f) = e.flags.iter().find(|f| !allowed(f)) {
            return Err(format!("{cmd} {f}: not modelled"));
        }
        if let Some((f, _)) = e.args.iter().find(|(f, _)| f != "-from" && f != "-to") {
            return Err(format!("{cmd} {f}: not modelled"));
        }
        let side = |flag: &str| -> Result<(Vec<String>, bool), String> {
            let (mut names, mut clock) = (Vec::new(), false);
            for (_, o) in e.args.iter().filter(|(f, _)| f == flag) {
                match o.accessor.as_str() {
                    "get_ports" => {
                        for n in &o.names {
                            names.extend(nl.ports.iter().map(|(p, _)| p).filter(|p| vyges_loom::sdc::object_matches(n, p)).cloned());
                        }
                    }
                    "get_pins" => {
                        for n in &o.names {
                            names.extend(pins.iter().filter(|p| vyges_loom::sdc::object_matches(n, p)).cloned());
                        }
                    }
                    "get_clocks" if s.clocks.len() == 1 && o.names.iter().all(|n| vyges_loom::sdc::object_matches(n, &s.clocks[0].name)) => clock = true,
                    other => return Err(format!("{cmd} {flag} [{other} {}]: not modelled (ports, pins or the one clock are)", o.names.join(" "))),
                }
            }
            names.dedup();
            Ok((names, clock))
        };
        let (from_pins, from_clock) = side("-from")?;
        let (to_pins, to_clock) = side("-to")?;
        // Warnings 471 / 473: a side given whose objects match nothing makes no exception at all
        // (read as no side, `-from [get_ports -quiet nope]` would cut every path)
        let given = |flag: &str| e.args.iter().any(|(f, _)| f == flag);
        if (given("-from") && from_pins.is_empty() && !from_clock) || (given("-to") && to_pins.is_empty() && !to_clock) {
            continue;
        }
        if !given("-from") && !given("-to") {
            return Err(format!("{cmd} with no -from / -to: not modelled"));
        }
        let has = |f: &str| e.flags.iter().any(|x| x == f);
        if let Some(n) = multiplier {
            let (setup, hold) = (has("-setup"), has("-hold"));
            if has("-start") && has("-end") {
                return Err("set_multicycle_path: cannot use -start with -end (446)".into());
            }
            let use_end_clk = if has("-start") { false } else if has("-end") { true } else { !(hold && !setup) };
            out.push(crate::sdc::PathDelay {
                from_pins,
                from_clock,
                to_pins,
                to_clock,
                min_max: if hold && !setup { MIN } else { MAX },
                ignore_clk_latency: false,
                break_path: false,
                delay: 0.0,
                false_path: false,
                multicycle: Some(crate::sdc::Multicycle { multiplier: n, use_end_clk, all: setup == hold }),
            });
            continue;
        }
        let mms: &[usize] = match (has("-setup"), has("-hold")) {
            (true, false) => &[MAX],
            (false, true) => &[MIN],
            _ => &[MIN, MAX],
        };
        for &mm in mms {
            out.push(crate::sdc::PathDelay {
                from_pins: from_pins.clone(),
                from_clock,
                to_pins: to_pins.clone(),
                to_clock,
                min_max: mm,
                ignore_clk_latency: false,
                break_path: false,
                delay: 0.0,
                false_path: true,
                multicycle: None,
            });
        }
    }
    Ok(out)
}

/// A `-from` / `-to` side as `get_clocks_warn` reads it: names are clocks.
fn clock_side(o: &vyges_loom::sdc::Objects) -> vyges_loom::sdc::Objects {
    let mut o = o.clone();
    if o.accessor.is_empty() {
        o.accessor = "get_clocks".into();
    }
    o
}

/// One design timed for setup: the worst and total negative slack over the endpoints and every
/// pin's slack (`Sta::pinSlack(pin, max)`), seconds.
#[derive(Debug, Clone)]
pub struct Timing {
    pub wns: f32,
    pub tns: f32,
    /// `(pin, slack)` in vertex order; a pin no constrained path reaches has none
    pub pin_slacks: Vec<(String, f32)>,
}

impl Timing {
    /// The report as the reference prints it, one line each: `worst <s>`, `tns <s>` and
    /// `slack <pin> <s>` per pin, in the library's time unit to six digits — `Unit::asString`,
    /// the value divided by the unit's scale in FLOAT, under 1e-6 printed as 0.
    pub fn report_lines(&self, time_scale: f32) -> Vec<String> {
        let user = |x: f32| {
            let u = x / time_scale;
            if u.abs() < 1e-6 {
                0.0
            } else {
                u
            }
        };
        let mut out = vec![format!("worst {:.6}", user(self.wns)), format!("tns {:.6}", user(self.tns))];
        out.extend(self.pin_slacks.iter().map(|(pin, slack)| format!("slack {pin} {:.6}", user(*slack))));
        out
    }
}

/// The design timed as the reference times it with no parasitics read: the graph built from
/// [`netlist`], delays with every net's wire ideal (pin capacitance only), the ideal clocks' network
/// at the ideal slew, arrivals and requireds from [`constraints`]. `wns` / `tns` are over the
/// endpoints' setup slacks (`Search::wnsSlacks`; `tns` the sum of the negative ones).
pub fn time_design(libs: &[Library], nl: &Netlist, sdc: &crate::sdc::Sdc, env: crate::graph::SdcEnv) -> Result<Timing, String> {
    time_design_for(libs, nl, sdc, env, crate::liberty::MAX)
}

/// [`time_design`] for min (`MIN`: hold, `report_worst_slack -min`, `report_tns -min`,
/// `slack_min`) or max (`MAX`).
pub fn time_design_for(libs: &[Library], nl: &Netlist, sdc: &crate::sdc::Sdc, env: crate::graph::SdcEnv, mm: usize) -> Result<Timing, String> {
    use crate::search::Search;
    let mut g = crate::graph::Graph::build(libs, nl)?;
    g.sdc = env;
    g.ideal_clock = crate::search::ideal_clock_pins(&g, sdc);
    g.find_delays(&HashMap::new(), None)?;
    let mut s = Search::in_graph_order(&g, sdc);
    s.constraints_modelled()?;
    s.find_arrivals()?;
    s.find_requireds()?;
    // `Search::findTotalNegativeSlacks`: over the endpoints in vertex order, each slack fuzzily
    // under 0 added in DOUBLE (`DelayDbl`), narrowed to float once (`delayDblAsDelay`)
    let (mut wns, mut tns) = (crate::search::INF, 0.0f64);
    for v in (0..g.vertices.len()).filter(|&v| s.is_endpoint(v)) {
        let slack = s.wns_slack(v, mm == crate::liberty::MAX);
        if crate::fuzzy::less(slack, wns) {
            wns = slack;
        }
        if crate::fuzzy::less(slack, 0.0) {
            tns += f64::from(slack);
        }
    }
    let tns = tns as f32;
    let pin_slacks = (0..g.vertices.len())
        .filter_map(|v| {
            let slack = s.slack_of(v, mm, None);
            (slack < crate::search::INF).then(|| (g.vertices[v].name.clone(), slack))
        })
        .collect();
    Ok(Timing { wns, tns, pin_slacks })
}

/// `network_->findLibertyCell(name)`: the first library that has the name.
fn link_cell<'l>(libs: &'l [Library], name: &str) -> Option<&'l crate::liberty::Cell> {
    libs.iter().find_map(|l| l.cells.get(name))
}

/// `parse_min_max_all_flags`: `-min` alone is min, `-max` alone max, neither or both all — as
/// `[min, max]` membership.
pub fn min_max_all(flags: &[String]) -> [bool; 2] {
    match (flags.iter().any(|f| f == "-min"), flags.iter().any(|f| f == "-max")) {
        (true, false) => [true, false],
        (false, true) => [false, true],
        _ => [true, true],
    }
}

/// `parse_rise_fall_flags`: `-rise` alone rise, `-fall` alone fall, neither or both both.
pub fn rise_fall(flags: &[String]) -> [bool; 2] {
    match (flags.iter().any(|f| f == "-rise"), flags.iter().any(|f| f == "-fall")) {
        (true, false) => [true, false],
        (false, true) => [false, true],
        _ => [true, true],
    }
}

/// The ports `[all_outputs]` / `[all_inputs …]` name. The reference's `allOutputs` keeps every
/// top-level port whose direction is any output (output, bidirect, tristate); `allInputs(no_clks)`
/// every any-input port (input, bidirect), less the clock-source pins under `-no_clocks`. Bus
/// members are already single ports here; netlist order is kept. Each port then takes the setting
/// exactly as `[get_ports …]` would. Err names an accessor flag not modelled.
pub fn port_collection(accessor: &str, flags: &[String], ports: &[(String, crate::netlist::PortDir)], clock_sources: &std::collections::HashSet<&str>) -> Result<Vec<String>, String> {
    use crate::netlist::PortDir;
    let outputs = accessor == "all_outputs";
    if let Some(f) = flags.iter().find(|f| f.as_str() != "-no_clocks" || outputs) {
        return Err(f.clone());
    }
    let no_clocks = flags.iter().any(|f| f == "-no_clocks");
    Ok(ports
        .iter()
        .filter(|(name, dir)| {
            let any = if outputs { matches!(dir, PortDir::Output | PortDir::Inout) } else { matches!(dir, PortDir::Input | PortDir::Inout) };
            any && !(no_clocks && clock_sources.contains(name.as_str()))
        })
        .map(|(name, _)| name.clone())
        .collect())
}


/// The SDC's `set_load` and `set_input_transition`, as the timer keeps them: values through the
/// user unit (`capacitance_ui_sta` / `time_ui_sta`: the value times the default library's FLOAT
/// unit scale, in double, narrowed to the float setter), applied in file order (a later command
/// overwrites); a net's load keyed by the net's DRIVER pins when it is set (`setNetWireCap`); a
/// port's load is its pin load (the default). Refused: `-wire_load`, objects named by anything
pub fn sdc_env(s: &vyges_loom::sdc::Sdc, libs: &[Library], nl: &crate::netlist::Netlist) -> Result<crate::graph::SdcEnv, String> {
    let mut env = crate::graph::SdcEnv::default();
    // `set_driving_cell -lib_cell C -pin P [-from_pin F] [-input_transition_rise/-fall S]`: the
    // port is driven through C's arcs into P, from F (else the default from-port), at slews S
    // (user time unit; 0 when absent).
    for d in &s.driving_cells {
        if let Some(f) = d.flags.first() {
            return Err(format!("set_driving_cell {f}: not modelled"));
        }
        if d.library.is_some() {
            return Err("set_driving_cell -library: not modelled".into());
        }
        if d.multiply_by.is_some() {
            return Err("set_driving_cell -multiply_by: not modelled".into());
        }
        let Some(cell) = &d.lib_cell else {
            return Err("set_driving_cell without -lib_cell: not modelled".into());
        };
        // Without `-pin` the reference takes the cell's ONE output port (output, tristate or
        // bidirect); a cell with several is its error 456, "-pin argument required".
        let inferred;
        let pin = match &d.pin {
            Some(p) => p,
            None => {
                use crate::liberty::Direction;
                let outs: Vec<&str> = link_cell(libs, cell)
                    .map(|c| c.ports.iter().filter(|p| matches!(p.direction, Direction::Output | Direction::Tristate | Direction::Bidirect)).map(|p| p.name.as_str()).collect())
                    .unwrap_or_default();
                match outs.as_slice() {
                    [one] => {
                        inferred = one.to_string();
                        &inferred
                    }
                    [] => return Err(format!("set_driving_cell -lib_cell {cell}: no output port (not modelled)")),
                    _ => return Err(format!("set_driving_cell -lib_cell {cell}: -pin argument required for cells with multiple outputs")),
                }
            }
        };
        let lib0 = libs.first().ok_or("set_driving_cell before any liberty library")?;
        if link_cell(libs, cell).and_then(|c| c.port(pin)).is_none() {
            return Err(format!("set_driving_cell -lib_cell {cell} -pin {pin}: no such cell port (not modelled)"));
        }
        let slews = [crate::sdc::user_to_sta(d.input_transition[0], lib0.time_scale), crate::sdc::user_to_sta(d.input_transition[1], lib0.time_scale)];
        // ⛔ `[all_inputs]` arrives as the collection `*INPUTS*`, not a port: inserted as a name
        // it drove nothing. It is every input and bidirect port — the clocks included, as the
        // reference applies it.
        use crate::netlist::PortDir;
        let ports: Vec<String> = d
            .objects
            .iter()
            .flat_map(|o| -> Vec<String> {
                if o == "*INPUTS*" {
                    nl.ports.iter().filter(|(_, dir)| matches!(dir, PortDir::Input | PortDir::Inout)).map(|(n, _)| n.clone()).collect()
                } else {
                    vec![o.clone()]
                }
            })
            .collect();
        for port in ports {
            env.input_drive.insert(port, crate::graph::InputDrive { cell: cell.clone(), from_port: d.from_pin.clone(), to_port: pin.clone(), from_slews: slews });
        }
    }
    if s.env.is_empty() {
        return Ok(env);
    }
    let lib0 = libs.first().ok_or("set_load / set_input_transition before any liberty library")?;
    // the graph only for a net `set_load` (its drivers), built when one asks
    let mut graph = None;
    for e in &s.env {
        let known = ["-min", "-max", "-rise", "-fall", "-pin_load", "-subtract_pin_load"];
        if let Some(f) = e.flags.iter().find(|f| !known.contains(&f.as_str())) {
            return Err(format!("{} {f}: not modelled", e.cmd));
        }
        let mm = min_max_all(&e.flags);
        let rf = rise_fall(&e.flags);
        let expanded;
        let (accessor, objects) = match e.accessor.as_str() {
            "all_outputs" | "all_inputs" => {
                let clock_sources: std::collections::HashSet<&str> = s.clocks.iter().flat_map(|c| c.sources.iter().map(String::as_str)).collect();
                expanded = port_collection(&e.accessor, &e.accessor_flags, &nl.ports, &clock_sources).map_err(|f| format!("{} [{} {f}]: not modelled", e.cmd, e.accessor))?;
                ("get_ports", &expanded)
            }
            other => (other, &e.objects),
        };
        match (e.cmd.as_str(), accessor) {
            ("set_load", "get_nets") => {
                let cap = (e.value * f64::from(lib0.cap_scale)) as f32;
                let subtract = e.flags.iter().any(|f| f == "-subtract_pin_load");
                for net in objects {
                    let n = nl.nets.iter().position(|x| &x.name == net).ok_or_else(|| format!("set_load: net {net} not found (not modelled)"))?;
                    let graph = match &mut graph {
                        Some(g) => g,
                        none => none.insert(crate::graph::Graph::build(libs, nl).map_err(|e| format!("{e} (not modelled)"))?),
                    };
                    for d in graph.vertices.iter().enumerate().filter(|(i, v)| v.is_driver && graph.vertex_net[*i] == Some(n)) {
                        let slot = env.net_wire_cap.entry(d.1.name.clone()).or_insert([None; 2]);
                        for k in 0..2 {
                            if mm[k] {
                                slot[k] = Some((cap, subtract));
                            }
                        }
                    }
                }
            }
            ("set_load", "get_ports") => {
                let cap = (e.value * f64::from(lib0.cap_scale)) as f32;
                for port in objects {
                    let slot = env.port_pin_cap.entry(port.clone()).or_insert([[None; 2]; 2]);
                    for r in 0..2 {
                        for k in 0..2 {
                            if rf[r] && mm[k] {
                                slot[r][k] = Some(cap);
                            }
                        }
                    }
                }
            }
            ("set_input_transition", "get_ports") if e.clock.is_none() => {
                let slew = crate::sdc::user_to_sta(e.value, lib0.time_scale);
                for port in objects {
                    let slot = env.input_slew.entry(port.clone()).or_insert([[None; 2]; 2]);
                    for r in 0..2 {
                        for k in 0..2 {
                            if rf[r] && mm[k] {
                                slot[r][k] = Some(slew);
                            }
                        }
                    }
                }
            }
            (cmd, acc) => return Err(format!("{cmd} on [{acc} …]{}: not modelled", if e.clock.is_some() { " -clock" } else { "" })),
        }
    }
    Ok(env)
}

/// The input and output delays with `[all_inputs]` / `[all_outputs]` expanded, as
/// [`expand_io_delay_ports`] leaves them, the constraints themselves untouched.
pub fn expanded_io_delays(s: &vyges_loom::sdc::Sdc, nl: &crate::netlist::Netlist) -> [Vec<vyges_loom::sdc::IoDelay>; 2] {
    use crate::netlist::PortDir;
    // the source ports of each clock, by name
    let sources: std::collections::HashMap<String, Vec<String>> = s.clocks.iter().map(|c| (c.name.clone(), c.sources.clone())).collect();
    let mut out = [s.input_delays.clone(), s.output_delays.clone()];
    let [ins, outs] = &mut out;
    for (list, want) in [(ins, PortDir::Input), (outs, PortDir::Output)] {
        let accessor = if want == PortDir::Input { "all_inputs" } else { "all_outputs" };
        let except = format!("{accessor}_except");
        for d in list.iter_mut().filter(|d| d.accessor == accessor || d.accessor == except) {
            // ⛔ A delay relative to a clock is not applied to that clock's own source port — the
            // reference skips it (STA-0441, "set_input_delay relative to a clock defined on the
            // same port/pin not allowed"); expanded with it, the clock port got a data arrival.
            let own: &[String] = d.clock.as_ref().and_then(|c| sources.get(c)).map(Vec::as_slice).unwrap_or(&[]);
            // `[lsearch -not -exact [all_inputs] P]` (`all_inputs_except`): every input but P.
            let excluded: Vec<String> = if d.accessor == except { d.ports.clone() } else { Vec::new() };
            d.ports = nl
                .ports
                .iter()
                .filter(|(n, dir)| (*dir == want || *dir == PortDir::Inout) && !own.contains(n) && !excluded.contains(n))
                .map(|(n, _)| n.clone())
                .collect();
            d.accessor = "get_ports".into();
            d.default = false;
        }
    }
    out
}

/// [`search_sdc`] on the design's ports: `[all_inputs]` / `[all_outputs]` in an I/O delay first
/// expanded to the ports they name ([`expand_io_delay_ports`]), as every caller with a netlist
/// must — read unexpanded, a flow's own `set_input_delay … [all_inputs]` stopped repair_timing.
pub fn search_sdc_on(
    s: &vyges_loom::sdc::Sdc,
    nl: &crate::netlist::Netlist,
    time_scale: f32,
    propagated: bool,
    hold_repair: bool,
) -> Result<crate::sdc::Sdc, String> {
    let [i, o] = expanded_io_delays(s, nl);
    search_sdc_with(s, [&i, &o], time_scale, propagated, hold_repair)
}

/// The constraints the setup search reads (one clock and its port delays), from the SDC as the
/// reference wrote it. Everything else that moves a setup slack is refused here, never dropped.
///
/// An I/O delay's flags place its value: `-max` / `-rise` / `-fall`; a `-min`-only delay bounds
/// hold paths and is not a setup input; `-add_delay` is read; any other flag is refused.
/// `[all_inputs]` / `[all_outputs]` in `set_input_delay` / `set_output_delay` as the ports they
/// name (`Sdc::allInputs`: every input or bidirect port; `allOutputs`: every output or bidirect —
/// the clock ports included, as without `-no_clocks` — less the delay's own clock's source ports,
/// which the reference skips), in the netlist's port order. Other delays
/// are kept as written. [`search_sdc`] reads only named ports; a caller with the netlist expands
/// first.
pub fn search_sdc(s: &vyges_loom::sdc::Sdc, time_scale: f32, propagated: bool, hold_repair: bool) -> Result<crate::sdc::Sdc, String> {
    search_sdc_with(s, [&s.input_delays, &s.output_delays], time_scale, propagated, hold_repair)
}

/// [`search_sdc`] with the input and output delays given (`io`: inputs, outputs).
fn search_sdc_with(
    s: &vyges_loom::sdc::Sdc,
    io: [&[vyges_loom::sdc::IoDelay]; 2],
    time_scale: f32,
    propagated: bool,
    hold_repair: bool,
) -> Result<crate::sdc::Sdc, String> {
    use crate::sdc::{Clock, PortDelay};
    // The clocks in the order they were made (`Clock::index`): each on one source port, or virtual
    // (no source — its edges launch input delays and close output delays). None: every path
    // unclocked. A clock is propagated when `set_propagated_clock` names it (the reference writes it
    // per clock; a virtual clock never is), or — named by none — when the run's command set it.
    let mut named_propagated: Vec<String> = Vec::new();
    for o in &s.propagated {
        if o.accessor != "get_clocks" {
            return Err(format!("set_propagated_clock on [{} {}]: not modelled (clocks are)", o.accessor, o.names.join(" ")));
        }
        named_propagated.extend(o.names.iter().cloned());
    }
    let mut clocks = Vec::new();
    for clock in &s.clocks {
        let source: &str = if clock.is_virtual() {
            ""
        } else {
            let [source] = clock.sources.as_slice() else {
                return Err(format!("clock {} on {} sources: not modelled (one is)", clock.name, clock.sources.len()));
            };
            source
        };
        let prop = !clock.is_virtual() && if named_propagated.is_empty() { propagated } else { named_propagated.contains(&clock.name) };
        let mut c = Clock::new(&clock.name, crate::sdc::user_to_sta(clock.period, time_scale), source, prop);
        // `-waveform {rise fall}`: each edge `time_ui_sta`, like the period. Two edges, the rise
        // first and within a period of the start, the fall after it and less than a period later.
        if let Some(w) = &clock.waveform {
            match w.as_slice() {
                &[r, f] if r >= 0.0 && r < clock.period && f > r && f - r < clock.period => {
                    c.waveform = [crate::sdc::user_to_sta(r, time_scale), crate::sdc::user_to_sta(f, time_scale)];
                }
                _ => return Err(format!("create_clock -waveform {{{}}} on period {}: not modelled (a rise then a fall within one period is)", w.iter().map(f64::to_string).collect::<Vec<_>>().join(" "), clock.period)),
            }
        }
        clocks.push(c);
    }
    let clock_index = |name: &str| s.clocks.iter().position(|c| c.name == name);
    // A hold uncertainty moves only hold checks: read by a hold repair alone.
    if s.setup_uncertainty != 0.0 || (s.hold_uncertainty != 0.0 && hold_repair) {
        return Err("set_clock_uncertainty: not modelled".into());
    }
    if s.late_derate.is_some() || s.early_derate.is_some() || !s.exceptions.is_empty() || !s.async_groups.is_empty() {
        return Err("derates, timing exceptions or clock groups: not modelled".into());
    }
    // `set_clock_latency` on pins (`Sdc::setClockLatency(clk | null, pin)`): an ideal clock's network
    // latency from that pin on. On clocks, `-source`, `-early` / `-late`, or on a propagated clock
    // (where the pin would turn it ideal) — refused.
    for l in &s.clock_latencies {
        if l.accessor != "get_pins" {
            return Err(format!("set_clock_latency on [{} …]: not modelled (pins are)", l.accessor));
        }
        if let Some(f) = l.flags.iter().find(|f| !matches!(f.as_str(), "-rise" | "-fall" | "-min" | "-max")) {
            return Err(format!("set_clock_latency {f}: not modelled"));
        }
        let rfs: Vec<usize> = match (l.flags.iter().any(|f| f == "-rise"), l.flags.iter().any(|f| f == "-fall")) {
            (true, false) => vec![0],
            (false, true) => vec![1],
            _ => vec![0, 1],
        };
        let mms: Vec<usize> = match (l.flags.iter().any(|f| f == "-min"), l.flags.iter().any(|f| f == "-max")) {
            (true, false) => vec![0],
            (false, true) => vec![1],
            _ => vec![0, 1],
        };
        let targets: Vec<usize> = match &l.clock {
            Some(name) => vec![s.clocks.iter().position(|c| &c.name == name).ok_or_else(|| format!("set_clock_latency -clock {name}: no such clock"))?],
            None => (0..clocks.len()).collect(),
        };
        let v = crate::sdc::user_to_sta(l.value, time_scale);
        for k in targets {
            let c: &mut Clock = &mut clocks[k];
            if c.propagated {
                return Err("set_clock_latency on a pin of a propagated clock: not modelled".into());
            }
            for pin in &l.objects {
                let specific = l.clock.is_some();
                let i = match c.pin_latencies.iter().position(|x| &x.pin == pin && x.specific == specific) {
                    Some(i) => i,
                    None => {
                        c.pin_latencies.push(crate::sdc::PinLatency { pin: pin.clone(), specific, delay: [[None; 2]; 2] });
                        c.pin_latencies.len() - 1
                    }
                };
                for &rf in &rfs {
                    for &mm in &mms {
                        c.pin_latencies[i].delay[rf][mm] = Some(v);
                    }
                }
            }
        }
    }
    // Per side (input, output): each delay by `[rf][min/max]` as set so far, keyed by port and
    // clock (`Sdc::findInputDelay` / `findOutputDelay`), in the order made. Without `-add_delay`,
    // `setInputDelay` deletes the port's delays on other clocks and overwrites the values it
    // names; with it, merges them — a max kept if larger, a min if smaller (`mergeValue`).
    type Delays = Vec<(String, usize, [[Option<f32>; 2]; 2])>;
    let mut delays: [Delays; 2] = [Vec::new(), Vec::new()];
    for (side, list) in [(0, io[0]), (1, io[1])] {
        let cmd = ["set_input_delay", "set_output_delay"][side];
        for d in list {
            if !d.value_read {
                return Err(format!("{cmd} on {}: the delay value is not modelled", d.ports.join(" ")));
            }
            if let Some(f) = d.flags.iter().find(|f| !matches!(f.as_str(), "-max" | "-min" | "-rise" | "-fall" | "-add_delay")) {
                return Err(format!("{cmd} {f}: not modelled"));
            }
            if d.reference_pin.is_some() {
                return Err(format!("{cmd} -reference_pin: not modelled"));
            }
            let flag = |f: &str| d.flags.iter().any(|x| x == f);
            let Some(clock_name) = d.clock.as_deref() else {
                return Err(format!("{cmd} on {}: a delay without -clock is not modelled", d.ports.join(" ")));
            };
            let Some(ci) = clock_index(clock_name) else {
                return Err(format!("{cmd} -clock {clock_name}: a delay on an unknown clock is not modelled"));
            };
            if d.default || d.accessor != "get_ports" {
                return Err(format!("{cmd} -clock {clock_name} on [{} …]: ports not read (get_ports are)", d.accessor));
            }
            let mms: Vec<usize> = match (flag("-min"), flag("-max")) {
                (true, false) => vec![0],
                (false, true) => vec![1],
                _ => vec![0, 1],
            };
            let rfs: Vec<usize> = match (flag("-rise"), flag("-fall")) {
                (true, false) => vec![0],
                (false, true) => vec![1],
                _ => vec![0, 1],
            };
            let add = flag("-add_delay");
            let v = crate::sdc::user_to_sta(d.value, time_scale);
            for port in &d.ports {
                let list = &mut delays[side];
                if !add {
                    list.retain(|(p, c, _)| p != port || *c == ci);
                }
                let k = match list.iter().position(|(p, c, _)| p == port && *c == ci) {
                    Some(k) => k,
                    None => {
                        list.push((port.clone(), ci, [[None; 2]; 2]));
                        list.len() - 1
                    }
                };
                for &rf in &rfs {
                    for &mm in &mms {
                        let slot = &mut list[k].2[rf][mm];
                        // `MinMax::compare`: max keeps the larger, min the smaller.
                        if !add || slot.is_none_or(|old| if mm == 1 { v > old } else { v < old }) {
                            *slot = Some(v);
                        }
                    }
                }
            }
        }
    }
    // An input delay: both maxima set (a min value missing takes the max's), every value present.
    // ⚠️ One set for min only is left out, as before hold was modelled — it would move only min
    // arrivals; no capture has one.
    let inputs = |list: &Delays| -> Result<Vec<PortDelay>, String> {
        list.iter()
            .filter(|(_, _, d)| d[0][1].is_some() || d[1][1].is_some())
            .map(|(p, clock, d)| {
                let max = |rf: usize| d[rf][1].ok_or_else(|| format!("port {p}: an input delay without its max value for both transitions is not modelled"));
                let (r, f) = (max(0)?, max(1)?);
                Ok(PortDelay { port: p.clone(), delay: [[d[0][0].unwrap_or(r), r], [d[1][0].unwrap_or(f), f]], exists: [[true; 2]; 2], clock: *clock })
            })
            .collect()
    };
    // An output delay: each `[rf][min/max]` value as set (`RiseFallMinMax`) — a missing one makes no
    // path end. A max set for one transition only is refused (unwitnessed).
    let outputs = |list: &Delays| -> Result<Vec<PortDelay>, String> {
        list.iter()
            .map(|(p, clock, d)| {
                if d[0][1].is_some() != d[1][1].is_some() {
                    return Err(format!("port {p}: an output delay set for one transition only is not modelled"));
                }
                let exists = [[d[0][0].is_some(), d[0][1].is_some()], [d[1][0].is_some(), d[1][1].is_some()]];
                let v = |rf: usize, mm: usize| d[rf][mm].unwrap_or(0.0);
                Ok(PortDelay { port: p.clone(), delay: [[v(0, 0), v(0, 1)], [v(1, 0), v(1, 1)]], exists, clock: *clock })
            })
            .collect()
    };
    Ok(crate::sdc::Sdc {
        clocks,
        input_delays: inputs(&delays[0])?,
        output_delays: outputs(&delays[1])?,
        path_delays: path_delays(s, time_scale)?,
    })
}

/// Every `set_max_delay` / `set_min_delay` as the reference's `write_sdc` writes it:
/// `[-ignore_clock_latency] [-probe] [-from <objs>] [-to <objs>] <delay>`, objects
/// `[get_pins {..}]` or `[get_clocks {..}]`. Any other flag is refused; so is more than one (an
/// exception's id is the order it was MADE in, which the file may not keep). Which forms the
/// timer models is the search's to say (`Search::path_delays_modelled`).
pub fn path_delays(s: &vyges_loom::sdc::Sdc, time_scale: f32) -> Result<Vec<crate::sdc::PathDelay>, String> {
    use crate::liberty::{MAX, MIN};
    let mut out = Vec::new();
    for d in &s.path_delays {
        let cmd = if d.max { "set_max_delay" } else { "set_min_delay" };
        if let Some(f) = d.flags.iter().find(|f| !matches!(f.as_str(), "-ignore_clock_latency" | "-probe")) {
            return Err(format!("{cmd} {f}: not modelled"));
        }
        if let Some((f, _)) = d.objects.iter().find(|(f, _)| f != "-from" && f != "-to") {
            return Err(format!("{cmd} {f}: not modelled"));
        }
        if d.value.is_nan() {
            return Err(format!("{cmd}: the delay value is not modelled"));
        }
        // The objects of one flag: (pins, names the clock).
        let objs = |flag: &str| -> Result<(Vec<String>, bool), String> {
            let Some((_, o)) = d.objects.iter().find(|(f, _)| f == flag) else { return Ok((Vec::new(), false)) };
            if o.accessor == "get_pins" {
                Ok((o.names.clone(), false))
            } else if o.accessor == "get_clocks" && o.names.len() == 1 && s.clocks.iter().any(|c| c.name == o.names[0]) {
                Ok((Vec::new(), true))
            } else {
                Err(format!("{cmd} {flag} [{} {}]: not modelled (pins or the clock are)", o.accessor, o.names.join(" ")))
            }
        };
        let (from_pins, from_clock) = objs("-from")?;
        let (to_pins, to_clock) = objs("-to")?;
        let flag = |f: &str| d.flags.iter().any(|x| x == f);
        out.push(crate::sdc::PathDelay {
            from_pins,
            from_clock,
            to_pins,
            to_clock,
            min_max: if d.max { MAX } else { MIN },
            ignore_clk_latency: flag("-ignore_clock_latency"),
            break_path: !flag("-probe"),
            delay: crate::sdc::user_to_sta(d.value, time_scale),
            false_path: false,
            multicycle: None,
        });
    }
    if out.len() > 1 {
        return Err(format!("{} path delays: not modelled (one is; ids are the order they were made in)", out.len()));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lib() -> Library {
        let text = "library (t) { time_unit : \"1ns\"; capacitive_load_unit (1, pf);\n\
            cell (BUF) { pin (A) { direction : input; capacitance : 0.001; }\n\
              pin (X) { direction : output; function : \"A\"; timing () { related_pin : \"A\"; } } }\n}\n";
        Library::read(&crate::liberty_parse::parse(text).unwrap()).unwrap()
    }

    fn pins(nl: &Netlist, net: &str) -> Vec<String> {
        let n = nl.nets.iter().find(|n| n.name == net).unwrap();
        n.pins.iter().map(|c| nl.pin_name(c)).collect()
    }

    /// Rules, each witnessed by the reference's graph build on the same netlist: instances in name
    /// order; a net's pins last connected first (`ConcreteNet::addPin` prepends), so swapping the
    /// statements reverses the wire edges' loads; its port before them (`visitConnectedPins`
    /// visits terms first: with a port load the float sum of a net's capacitance moved by one ulp
    /// in the last place otherwise, on rv_plic_lite); nets in driver order.
    #[test]
    fn the_netlist_is_laid_out_as_the_reference_network_is() {
        let src = "module ord2 (a, y);\n input a; output y; wire n1;\n\
                   BUF drv (.A(a), .X(n1));\n BUF la (.A(n1), .X());\n BUF lm (.A(n1), .X());\n BUF lz (.A(n1), .X(y));\nendmodule\n";
        let d = vyges_loom::netlist::parse_compact(src).unwrap();
        let nl = netlist(&d, &[lib()]).unwrap();
        let names: Vec<&str> = nl.insts.iter().map(|i| i.0.as_str()).collect();
        assert_eq!(names, ["drv", "la", "lm", "lz"]);
        assert_eq!(pins(&nl, "n1"), ["lz/A", "lm/A", "la/A", "drv/X"]);
        assert_eq!(pins(&nl, "y"), ["y", "lz/X"], "the port (a term) is visited before the pins");
        // drivers: drv/X (n1), lz/X (y) in instance order, then the input a
        let nets: Vec<&str> = nl.nets.iter().map(|n| n.name.as_str()).collect();
        assert_eq!(nets, ["n1", "y", "a"]);

        let swapped = "module ord2 (a, y);\n input a; output y; wire n1;\n\
                   BUF lz (.A(n1), .X(y));\n BUF lm (.A(n1), .X());\n BUF drv (.A(a), .X(n1));\n BUF la (.A(n1), .X());\nendmodule\n";
        let d = vyges_loom::netlist::parse_compact(swapped).unwrap();
        let nl = netlist(&d, &[lib()]).unwrap();
        assert_eq!(pins(&nl, "n1"), ["la/A", "drv/X", "lm/A", "lz/A"]);
    }

    /// What the timer has no place for is refused, naming it: an assign, a tie, hierarchy, a cell
    /// no library holds, a pin the cell lacks.
    #[test]
    fn what_cannot_be_represented_is_refused() {
        let refused = |src: &str, why: &str| {
            let d = vyges_loom::netlist::parse_compact(src).unwrap();
            let e = netlist(&d, &[lib()]).unwrap_err();
            assert!(e.contains(why), "{e}");
        };
        refused("module t (a, y); input a; output y; wire n; assign y = n; BUF b (.A(a), .X(n)); endmodule\n", "assign");
        refused("module t (y); output y; BUF b (.A(1'b0), .X(y)); endmodule\n", "tied to a constant");
        refused("module t (a, y); input a; output y; NAND2 b (.A(a), .B(a), .Y(y)); endmodule\n", "in no library");
        refused("module t (a, y); input a; output y; BUF b (.A(a), .Z(y)); endmodule\n", "no port Z");
        refused("module s (a, y); input a; output y; BUF b (.A(a), .X(y)); endmodule\nmodule t (a, y); input a; output y; s u (.a(a), .y(y)); endmodule\n", "hierarchy");
    }

    /// Rule (`set_clock_uncertainty` in the reference's `sdc/Sdc.tcl`): `-setup` alone sets the
    /// max value, `-hold` alone the min, neither both; a later command replaces an earlier one's
    /// value for the same clock and min/max; `-from/-to` sets the pair (`-rise_from` / `-fall_to`
    /// one edge each); on a pin it is a pin uncertainty, which this timer refuses.
    #[test]
    fn clock_uncertainties_land_on_their_clocks() {
        use crate::liberty::{FALL, MAX, MIN, RISE};
        let ns = |v: f64| crate::sdc::user_to_sta(v, 1e-9);
        let read = |text: &str| -> Result<Vec<crate::sdc::Clock>, String> {
            let s = vyges_loom::sdc::Sdc::parse(&format!("create_clock -name a -period 2 [get_ports ca]\ncreate_clock -name b -period 3\n{text}")).unwrap();
            let mut clocks = vec![crate::sdc::Clock::new("a", 2e-9, "ca", false), crate::sdc::Clock::new("b", 3e-9, "", false)];
            clock_uncertainties(&s.uncertainties, &mut clocks, 1e-9).map(|_| clocks)
        };
        let c = read("set_clock_uncertainty 0.3 [all_clocks]\nset_clock_uncertainty -setup 0.1 [get_clocks a]\n").unwrap();
        assert_eq!(c[0].uncertainty, [Some(ns(0.3)), Some(ns(0.1))], "a: hold kept, setup replaced");
        assert_eq!(c[1].uncertainty, [Some(ns(0.3)), Some(ns(0.3))]);
        let c = read("set_clock_uncertainty -hold 0.02 -rise_from [get_clocks a] -fall_to [get_clocks b]\n").unwrap();
        assert_eq!(c[1].inter_uncertainties.len(), 1);
        let u = &c[1].inter_uncertainties[0];
        assert_eq!((u.from, u.value[RISE][FALL][MIN], u.value[RISE][FALL][MAX], u.value[FALL][FALL][MIN]), (0, Some(ns(0.02)), None, None));
        let c = read("set_clock_uncertainty -setup 0.2 -from a -to b -rise\n").unwrap();
        assert_eq!((c[1].inter_uncertainties[0].value[FALL][RISE][MAX], c[1].inter_uncertainties[0].value[FALL][FALL][MAX]), (Some(ns(0.2)), None), "-rise after -to: the target's rise edge");
        assert!(read("set_clock_uncertainty 0.1 [get_pins r/CK]\n").unwrap_err().contains("not modelled"));
    }


    /// Rules (`Sdc::makeFalsePath` / `makeExceptionFrom`, probed on the reference with
    /// `rv_plic_lite`): `-setup` is the max paths, `-hold` the min ones, neither both; a side
    /// given whose objects match nothing warns (471 `-from`, 473 `-to`) and makes NO exception —
    /// the slacks were the ones without the command, not those of a path cut everywhere.
    #[test]
    fn false_paths_split_by_min_max_and_skip_an_empty_side() {
        use crate::liberty::{MAX, MIN};
        let src = "module ord2 (a, y);\n input a; output y; wire n1;\n BUF drv (.A(a), .X(n1));\n BUF lz (.A(n1), .X(y));\nendmodule\n";
        let nl = netlist(&vyges_loom::netlist::parse_compact(src).unwrap(), &[lib()]).unwrap();
        let read = |text: &str| {
            let s = vyges_loom::sdc::Sdc::parse(text).unwrap();
            timing_exceptions(&s.exceptions, &s, &nl)
        };
        let f = read("set_false_path -setup -from [get_ports a] -to [get_pins lz/X]\n").unwrap();
        assert_eq!(f.len(), 1);
        assert_eq!((f[0].min_max, f[0].from_pins.clone(), f[0].to_pins.clone()), (MAX, vec!["a".to_string()], vec!["lz/X".to_string()]));
        let f = read("set_false_path -to [get_ports y]\n").unwrap();
        assert_eq!(f.iter().map(|p| p.min_max).collect::<Vec<_>>(), vec![MIN, MAX]);
        assert!(read("set_false_path -from [get_ports -quiet nope]\nset_false_path -from [get_ports a] -to [get_ports -quiet nope2]\n").unwrap().is_empty(), "an empty side: no exception");
        assert!(read("set_false_path -from [get_ports a] -through [get_pins lz/A]\n").unwrap_err().contains("not modelled"));
    }

    /// Rule (`set_multicycle_path` in `Sdc.tcl`): one exception per command — `-setup` alone is
    /// max, `-hold` alone min, else neither (`all`); `-end` unless `-hold` alone or `-start`.
    #[test]
    fn a_multicycle_path_reads_as_the_reference_makes_it() {
        use crate::liberty::{MAX, MIN};
        let src = "module ord2 (a, y);\n input a; output y; wire n1;\n BUF drv (.A(a), .X(n1));\n BUF lz (.A(n1), .X(y));\nendmodule\n";
        let nl = netlist(&vyges_loom::netlist::parse_compact(src).unwrap(), &[lib()]).unwrap();
        let read = |text: &str| {
            let s = vyges_loom::sdc::Sdc::parse(text).unwrap();
            timing_exceptions(&s.exceptions, &s, &nl).unwrap()
        };
        let one = |text: &str| {
            let f = read(text);
            assert_eq!(f.len(), 1, "{text}");
            let m = f[0].multicycle.expect("a multicycle path");
            (f[0].min_max, m.multiplier, m.use_end_clk, m.all)
        };
        assert_eq!(one("set_multicycle_path 2 -from [get_ports a]\n"), (MAX, 2, true, true));
        assert_eq!(one("set_multicycle_path -setup 3 -to [get_ports y]\n"), (MAX, 3, true, false));
        assert_eq!(one("set_multicycle_path -hold 1 -to [get_ports y]\n"), (MIN, 1, false, false));
        assert_eq!(one("set_multicycle_path -hold -end 1 -to [get_ports y]\n"), (MIN, 1, true, false));
        assert_eq!(one("set_multicycle_path -setup -start 2 -to [get_ports y]\n"), (MAX, 2, false, false));
    }

    /// The constraints a flow generates — every input and output delayed through `[all_inputs]` /
    /// `[all_outputs]` — read for the timing search: expanded to the design's ports, the clock's
    /// own port skipped (STA-0441). Read unexpanded they stopped repair_timing with an error.
    #[test]
    fn io_delays_on_all_inputs_and_outputs_reach_the_search() {
        use crate::netlist::{Netlist, PortDir::*};
        let sdc = vyges_loom::sdc::Sdc::parse(
            "create_clock -name clk -period 25 [get_ports {clk}]\nset_input_delay 5.0 -clock clk [all_inputs]\nset_output_delay 5.0 -clock clk [all_outputs]\n",
        )
        .unwrap();
        let nl = Netlist { insts: vec![], ports: vec![("clk".into(), Input), ("a".into(), Input), ("y".into(), Output)], nets: vec![] };
        assert!(search_sdc(&sdc, 1e-9, false, false).is_err(), "the plain reader reads only named ports");
        let ss = search_sdc_on(&sdc, &nl, 1e-9, false, false).unwrap();
        assert_eq!(ss.input_delays.iter().map(|d| d.port.as_str()).collect::<Vec<_>>(), ["a"]);
        assert_eq!(ss.output_delays.iter().map(|d| d.port.as_str()).collect::<Vec<_>>(), ["y"]);
    }

    /// `[lsearch -inline -all -not -exact [all_inputs] $clk_port]` (every input but the clock's,
    /// the usual flow constraint) reaches the search as the inputs less that port, netlist order.
    #[test]
    fn every_input_but_the_clock_port_reaches_the_search() {
        use crate::netlist::{Netlist, PortDir::*};
        let sdc = vyges_loom::sdc::Sdc::parse(
            "set clk_port [get_ports clk]\ncreate_clock -name clk -period 25 $clk_port\nset non_clock_inputs [lsearch -inline -all -not -exact [all_inputs] $clk_port]\nset_input_delay 5.0 -clock clk $non_clock_inputs\n",
        )
        .unwrap();
        let nl = Netlist { insts: vec![], ports: vec![("a".into(), Input), ("clk".into(), Input), ("b".into(), Inout), ("y".into(), Output)], nets: vec![] };
        let ss = search_sdc_on(&sdc, &nl, 1e-9, false, false).unwrap();
        assert_eq!(ss.input_delays.iter().map(|d| d.port.as_str()).collect::<Vec<_>>(), ["a", "b"]);
    }

    /// Rules (`Sdc::allOutputs` / `Sdc::allInputs`): any-output = output or bidirect; any-input =
    /// input or bidirect; `-no_clocks` drops clock-source ports only from `all_inputs`, and is not
    /// a flag of `all_outputs`. Netlist order is kept.
    #[test]
    fn port_collections_expand_as_the_reference_does() {
        use crate::netlist::PortDir::*;
        let ports = [("clk".to_string(), Input), ("a".into(), Input), ("io".into(), Inout), ("y".into(), Output)];
        let clk: std::collections::HashSet<&str> = ["clk"].into();
        let none: &[String] = &[];
        assert_eq!(port_collection("all_outputs", none, &ports, &clk).unwrap(), ["io", "y"]);
        assert_eq!(port_collection("all_inputs", none, &ports, &clk).unwrap(), ["clk", "a", "io"]);
        assert_eq!(port_collection("all_inputs", &["-no_clocks".into()], &ports, &clk).unwrap(), ["a", "io"]);
        assert_eq!(port_collection("all_outputs", &["-no_clocks".into()], &ports, &clk), Err("-no_clocks".into()));
        assert_eq!(port_collection("all_inputs", &["-level_sensitive".into()], &ports, &clk), Err("-level_sensitive".into()));
    }
}
