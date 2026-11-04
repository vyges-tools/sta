// SPDX-License-Identifier: Apache-2.0
//! A detailed SPEF read as the reference's `read_spef` reads it with its defaults (no `-reduce`,
//! coupling capacitors not kept, coupling factor 1, the first value of a corner triplet): per net
//! an RC network whose nodes are the timer's pins and the net's internal nodes, for
//! [`crate::graph::Graph::find_delays`] to reduce per driver.
//!
//! Rules (`SpefReader`):
//! - a number is read as a `double` and narrowed to `float`; a value is that `float` times the
//!   unit's `float` scale (`*C_UNIT` `PF` 1e-12, `FF` 1e-15; `*R_UNIT` `OHM` 1, `KOHM` 1e3);
//! - `*NAME_MAP` indices (`*<n>`) are replaced by their names before lookup;
//! - a node `<instance>:<port>` is the instance's pin, `<net>:<digits>` an internal node of that
//!   net, a name with no delimiter a top-level port (`findParasiticNode`);
//! - a `*D_NET` for a net the design does not have is read past; a later `*D_NET` for the same net
//!   replaces the earlier (`deleteReducedParasitics`, `makeParasiticNetwork`);
//! - a one-node `*CAP` adds its value to the node's ground capacitance (`incrCap`, in `float`),
//!   in file order;
//! - a two-node `*CAP` with a positive value, coupling not kept, adds the value times the coupling
//!   factor to each of its nodes that is on the net being read;
//! - a `*RES` whose two nodes resolve is a resistor, in file order;
//! - `*CONN` makes no node.
//!
//! Names are matched as the reference relaxes them: as written, else with the SPEF escapes
//! (`\`) removed (`stripped`), against the design's names with their Verilog escape removed.

use std::collections::HashMap;

use crate::graph::NetParasitics;
use crate::netlist::Netlist;

/// What a read found that the reference warns about and reads past, counted.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SpefReport {
    pub nets: usize,
    /// `*D_NET`s naming a net the design does not have (warning 1650).
    pub unknown_nets: usize,
    /// Node names that resolve to no pin, port or net (warnings 1647–1657).
    pub unknown_nodes: usize,
    /// `*R_NET` blocks (reduced parasitics), which this reader refuses.
    pub reduced_nets: usize,
}

/// A design name as the reference's network keys it: a Verilog escaped identifier's leading `\`
/// and trailing blank removed, and every remaining backslash.
fn key(name: &str) -> String {
    let n = name.trim_end();
    let n = n.strip_prefix('\\').unwrap_or(n);
    n.replace('\\', "")
}

/// `atof` then the cast to `float` (`SpefLex.ll`); of a triplet `a:b:c`, the first.
fn number(tok: &str) -> Result<f32, String> {
    let first = tok.split(':').next().unwrap_or(tok);
    first.parse::<f64>().map(|v| v as f32).map_err(|_| format!("SPEF: {tok} is not a number"))
}

struct Design<'a> {
    nl: &'a Netlist,
    insts: HashMap<String, usize>,
    nets: HashMap<String, usize>,
    ports: HashMap<String, usize>,
    /// Each pin's net, by the pin's timer name.
    net_of_pin: HashMap<String, usize>,
}

/// A node the reader resolved: its name in the network, and the net it belongs to.
struct Node {
    name: String,
    net: Option<usize>,
}

impl<'a> Design<'a> {
    fn new(nl: &'a Netlist) -> Design<'a> {
        let insts = nl.insts.iter().enumerate().map(|(i, x)| (key(&x.0), i)).collect();
        let nets = nl.nets.iter().enumerate().map(|(k, n)| (key(&n.name), k)).collect();
        let ports = nl.ports.iter().enumerate().map(|(i, p)| (key(&p.0), i)).collect();
        let mut net_of_pin = HashMap::new();
        for (k, n) in nl.nets.iter().enumerate() {
            for c in &n.pins {
                net_of_pin.insert(nl.pin_name(c), k);
            }
        }
        Design { nl, insts, nets, ports, net_of_pin }
    }

    fn lookup(map: &HashMap<String, usize>, name: &str) -> Option<usize> {
        map.get(&key(name)).copied()
    }

    /// `findParasiticNode`, against `map` (the name map).
    fn node(&self, name: &str, map: &HashMap<String, String>, delimiter: char) -> Option<Node> {
        let mapped = |n: &str| -> Option<String> {
            match n.strip_prefix('*') {
                Some(_) => map.get(n).cloned(),
                None => Some(n.to_string()),
            }
        };
        match name.rfind(delimiter) {
            Some(at) => {
                let (n1, n2) = (mapped(&name[..at])?, &name[at + 1..]);
                if let Some(i) = Design::lookup(&self.insts, &n1) {
                    let pin = format!("{}/{}", self.nl.insts[i].0, n2);
                    // `findPin(inst, port)`; an unconnected pin has no net
                    let net = self.net_of_pin.get(&pin).copied();
                    return Some(Node { name: pin, net });
                }
                let k = Design::lookup(&self.nets, &n1)?;
                n2.bytes().all(|b| b.is_ascii_digit()).then(|| Node { name: format!("{}:{n2}", self.nl.nets[k].name), net: Some(k) })
            }
            None => {
                let n = mapped(name)?;
                let i = Design::lookup(&self.ports, &n)?;
                let port = self.nl.ports[i].0.clone();
                let net = self.net_of_pin.get(&port).copied();
                Some(Node { name: port, net })
            }
        }
    }
}

/// One net's network under construction.
#[derive(Default)]
struct Building {
    net: usize,
    index: HashMap<String, usize>,
    p: NetParasitics,
}

impl Building {
    /// `ensureParasiticNode`.
    fn node(&mut self, name: &str) -> usize {
        if let Some(&n) = self.index.get(name) {
            return n;
        }
        let n = self.p.node_names.len();
        self.p.node_names.push(name.to_string());
        self.p.network.node_caps.push(0.0);
        self.index.insert(name.to_string(), n);
        n
    }
}

/// Every `*D_NET` of `text` as the RC network of the design's net it names, keyed by the net's
/// timer name, and what was read past.
pub fn read(text: &str, nl: &Netlist) -> Result<(HashMap<String, NetParasitics>, SpefReport), String> {
    const COUPLING_CAP_FACTOR: f32 = 1.0;
    let d = Design::new(nl);
    let mut out: HashMap<String, NetParasitics> = HashMap::new();
    let mut report = SpefReport::default();
    let mut map: HashMap<String, String> = HashMap::new();
    let (mut cap_scale, mut res_scale) = (1.0f32, 1.0f32);
    let mut delimiter = ':';
    // sections: what the current lines are
    #[derive(PartialEq)]
    enum Sec {
        Header,
        NameMap,
        Skip,
        Conn,
        Cap,
        Res,
    }
    let mut sec = Sec::Header;
    let mut cur: Option<Building> = None;
    let mut in_net = false;
    let finish = |cur: &mut Option<Building>, out: &mut HashMap<String, NetParasitics>| {
        if let Some(b) = cur.take() {
            out.insert(nl.nets[b.net].name.clone(), b.p);
        }
    };
    for raw in text.lines() {
        let line = raw.split("//").next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let toks: Vec<&str> = line.split_whitespace().collect();
        match toks[0] {
            "*C_UNIT" | "*R_UNIT" => {
                let scale = toks.get(1).map(|t| number(t)).transpose()?.ok_or("SPEF: a unit without a scale")?;
                let unit = toks.get(2).map(|u| u.to_ascii_uppercase()).unwrap_or_default();
                if toks[0] == "*C_UNIT" {
                    cap_scale = match unit.as_str() {
                        "PF" => scale * 1e-12f32,
                        "FF" => scale * 1e-15f32,
                        u => return Err(format!("SPEF: unknown capacitance unit {u}")),
                    };
                } else {
                    res_scale = match unit.as_str() {
                        "OHM" => scale,
                        "KOHM" => scale * 1e3f32,
                        u => return Err(format!("SPEF: unknown resistance unit {u}")),
                    };
                }
                continue;
            }
            "*DELIMITER" => {
                delimiter = toks.get(1).and_then(|t| t.chars().next()).unwrap_or(':');
                continue;
            }
            "*NAME_MAP" => {
                sec = Sec::NameMap;
                continue;
            }
            "*PORTS" | "*POWER_NETS" | "*GROUND_NETS" | "*DEFINE" | "*PDEFINE" | "*VARIATION_PARAMETERS" => {
                sec = Sec::Skip;
                continue;
            }
            "*D_NET" => {
                finish(&mut cur, &mut out);
                in_net = true;
                sec = Sec::Skip;
                let name = toks.get(1).ok_or("SPEF: *D_NET without a net")?;
                let mapped = if name.starts_with('*') { map.get(*name).cloned() } else { Some(name.to_string()) };
                match mapped.and_then(|n| Design::lookup(&d.nets, &n)) {
                    Some(k) => {
                        report.nets += 1;
                        cur = Some(Building { net: k, ..Default::default() });
                    }
                    None => report.unknown_nets += 1,
                }
                continue;
            }
            "*R_NET" | "*R_PNET" | "*D_PNET" => {
                finish(&mut cur, &mut out);
                if toks[0] == "*R_NET" {
                    report.reduced_nets += 1;
                }
                in_net = true;
                sec = Sec::Skip;
                continue;
            }
            "*CONN" if in_net => {
                sec = Sec::Conn;
                continue;
            }
            "*CAP" if in_net => {
                sec = Sec::Cap;
                continue;
            }
            "*RES" if in_net => {
                sec = Sec::Res;
                continue;
            }
            "*INDUC" if in_net => {
                sec = Sec::Skip;
                continue;
            }
            "*END" => {
                finish(&mut cur, &mut out);
                in_net = false;
                sec = Sec::Skip;
                continue;
            }
            _ => {}
        }
        match sec {
            Sec::NameMap if toks.len() >= 2 && toks[0].starts_with('*') => {
                map.insert(toks[0].to_string(), toks[1].to_string());
            }
            Sec::Cap => {
                let Some(b) = cur.as_mut() else { continue };
                match toks.as_slice() {
                    [_, n, v] => {
                        let value = number(v)? * cap_scale;
                        match d.node(n, &map, delimiter) {
                            Some(node) => {
                                let i = b.node(&node.name);
                                b.p.network.node_caps[i] += value;
                            }
                            None => report.unknown_nodes += 1,
                        }
                    }
                    [_, n1, n2, v] => {
                        let value = number(v)? * cap_scale;
                        let (a, c) = (d.node(n1, &map, delimiter), d.node(n2, &map, delimiter));
                        report.unknown_nodes += usize::from(a.is_none()) + usize::from(c.is_none());
                        // `findParasiticNode` makes both nodes in this network, whichever net
                        let (ia, ic) = (a.as_ref().map(|x| b.node(&x.name)), c.as_ref().map(|x| b.node(&x.name)));
                        if value > 0.0 {
                            let scaled = value * COUPLING_CAP_FACTOR;
                            for (node, i) in [(a, ia), (c, ic)] {
                                if let (Some(node), Some(i)) = (node, i) {
                                    if node.net == Some(b.net) {
                                        b.p.network.node_caps[i] += scaled;
                                    }
                                }
                            }
                        }
                    }
                    _ => return Err(format!("SPEF: a *CAP line of {} fields: {line}", toks.len())),
                }
            }
            Sec::Res => {
                let Some(b) = cur.as_mut() else { continue };
                let [_, n1, n2, v] = toks.as_slice() else { return Err(format!("SPEF: a *RES line of {} fields: {line}", toks.len())) };
                let value = number(v)? * res_scale;
                match (d.node(n1, &map, delimiter), d.node(n2, &map, delimiter)) {
                    (Some(a), Some(c)) => {
                        let (ia, ic) = (b.node(&a.name), b.node(&c.name));
                        b.p.network.resistors.push((ia, ic, value));
                    }
                    (a, c) => report.unknown_nodes += usize::from(a.is_none()) + usize::from(c.is_none()),
                }
            }
            _ => {}
        }
    }
    finish(&mut cur, &mut out);
    if report.reduced_nets > 0 {
        return Err(format!("SPEF: {} reduced (*R_NET) nets: not modelled by this timer", report.reduced_nets));
    }
    Ok((out, report))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::netlist::{inst, Conn, Net, PortDir};
    use std::sync::Arc;

    fn nl() -> Netlist {
        Netlist {
            insts: vec![inst("u1", "BUF"), inst("u2", "BUF")],
            ports: vec![("a".into(), PortDir::Input)],
            nets: vec![
                Arc::new(Net { name: "a".into(), pins: vec![Conn::Port(0), Conn::Inst(0, "A".into())] }),
                Arc::new(Net { name: "n1".into(), pins: vec![Conn::Inst(1, "A".into()), Conn::Inst(0, "X".into())] }),
            ],
        }
    }

    const HEAD: &str = "*SPEF \"IEEE 1481-1998\"\n*DIVIDER /\n*DELIMITER :\n*BUS_DELIMITER [ ]\n*T_UNIT 1 NS\n";

    /// Ground caps add per node in float, in file order; a value is the parsed float times the
    /// unit's float scale; resistors keep file order; the name map is applied.
    #[test]
    fn a_net_reads_as_its_nodes_caps_and_resistors() {
        let text = format!("{HEAD}*C_UNIT 1 PF\n*R_UNIT 1 KOHM\n*NAME_MAP\n*1 n1\n*2 u1\n*3 u2\n*D_NET *1 0.003\n*CONN\n*I *2:X O\n*I *3:A I\n*CAP\n1 *2:X 0.001\n2 *1:1 0.002\n3 *2:X 0.0005\n*RES\n1 *2:X *1:1 0.01\n2 *1:1 *3:A 0.02:0.03:0.04\n*END\n");
        let (p, r) = read(&text, &nl()).unwrap();
        assert_eq!(r, SpefReport { nets: 1, ..Default::default() });
        let n = &p["n1"];
        assert_eq!(n.node_names, vec!["u1/X", "n1:1", "u2/A"]);
        let pf = 1e-12f32;
        assert_eq!(n.network.node_caps, vec![(0.001f64 as f32) * pf + (0.0005f64 as f32) * pf, (0.002f64 as f32) * pf, 0.0]);
        assert_eq!(n.network.resistors, vec![(0, 1, (0.01f64 as f32) * 1e3f32), (1, 2, (0.02f64 as f32) * 1e3f32)]);
    }

    /// A coupling cap adds its whole value to the end on the net being read only; the other
    /// net's node is made in this network but takes nothing; a zero coupling cap adds nothing.
    #[test]
    fn a_coupling_cap_loads_only_its_own_end() {
        let text = format!("{HEAD}*C_UNIT 1 FF\n*R_UNIT 1 OHM\n*D_NET n1 1\n*CAP\n1 n1:1 a:1 0.5\n2 n1:1 a:2 0\n*RES\n1 u1:X n1:1 5\n*END\n");
        let (p, _) = read(&text, &nl()).unwrap();
        let n = &p["n1"];
        assert_eq!(n.node_names, vec!["n1:1", "a:1", "a:2", "u1/X"]);
        assert_eq!(n.network.node_caps, vec![(0.5f64 as f32) * 1e-15f32, 0.0, 0.0, 0.0]);
    }

    /// A port node is the port's pin; a net the design lacks is read past and counted.
    #[test]
    fn ports_and_unknown_nets() {
        let text = format!("{HEAD}*C_UNIT 1 FF\n*R_UNIT 1 OHM\n*D_NET a 1\n*CAP\n1 a 1\n*RES\n1 a u1:A 2\n*END\n*D_NET zz 1\n*CAP\n1 zz:1 1\n*END\n");
        let (p, r) = read(&text, &nl()).unwrap();
        assert_eq!(p["a"].node_names, vec!["a", "u1/A"]);
        assert_eq!(r.unknown_nets, 1);
    }

    /// Reduced (`*R_NET`) parasitics are refused, not dropped.
    #[test]
    fn reduced_nets_are_refused() {
        let text = format!("{HEAD}*C_UNIT 1 FF\n*R_UNIT 1 OHM\n*R_NET n1 1\n*END\n");
        assert!(read(&text, &nl()).is_err());
    }
}
