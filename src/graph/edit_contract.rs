// SPDX-License-Identifier: Apache-2.0
//! The in-place edit's contract as a seeded test (formal pilot item 7, after the 2026-11-15 Kani
//! trial: the generator found every class in seconds where CBMC finished no harness).
//!
//! The property, from the edit API's contract (this module's parent doc and the database's
//! callback rules), not from any known fallback: over small designs and every batch of netlist
//! edits the database can report — instances and nets made, pins connected and disconnected, a
//! master swapped for one with the same terminal names — `Graph::apply_edits` takes the in-place
//! path; and after an in-place edit `IncTimer::import` reads only what was written since.
//!
//! Each refusal is a CLASS, named by the refusal the API itself returns. [`EXPECTED`] lists the
//! classes still open, each with its cause. The test fails when a class appears that is not
//! listed (a fallback reintroduced, or a new one) AND when a listed class no longer appears (a fix
//! landed: take it off the list in the same commit). So the list only shrinks, and a reintroduced
//! fallback fails a seconds-long test rather than a 64-minute run.
//!
//! Bounds: 2 instances to start (a buffer or one of 3 swappable family masters), at most 3 edits
//! per batch (so at most 5 instances), 2 libraries; free liberty port order, LEF terminal order and
//! 0..=2 arc sets per family cell; top port `o` an output or an inout. Seeded xorshift, so a run is
//! reproducible; `VYGES_EDIT_CONTRACT_SAMPLES` raises the sample count for a deeper run.

use super::*;
use crate::liberty::{Arc as LArc, ArcSet, Cell, Library, Model as ArcModel, Port};
use crate::netlist::{Net, PortDir};
use crate::table::GateModel;
use std::cell::Cell as StdCell;
use std::collections::BTreeMap;

/// The classes still open, with their cause. Shrinks as fixes land (plan §6.3, §6.4).
const EXPECTED: &[(&str, &str)] = &[
    ("Holes/swaps only", "a non-equivalent swap to a cell with fewer arc sets frees an edge id; plan §6.3"),
    ("Holes/with pin edits", "a load disconnected or moved to a driverless net frees an edge id; plan §6.4"),
    ("Netlist/loop edge deleted", "loops_kept refuses deleting a loop edge (8b1d8eb, by design); plan §6.4"),
    ("Event/iterm_*", "an edit naming an inout top port is still refused (459fef3); plan §6.4"),
];

/// Samples per run: at the 2026-11-15 rates every open class shows within the first 3 000.
const SAMPLES: usize = 20_000;
const SEED: u64 = 0x9e37_79b9_7f4a_7c15;

const PORTS: [&str; 3] = ["A", "B", "Y"];
const PERMS: [[usize; 3]; 6] = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
/// The arc-set menu a family cell draws from: (from, to, role, timing type).
const MENU: [(&str, &str, Role, &str); 5] = [
    ("A", "Y", Role::Combinational, "combinational"),
    ("B", "Y", Role::Combinational, "combinational"),
    ("A", "Y", Role::RegClkToQ, "rising_edge"),
    ("A", "B", Role::Setup, "setup_rising"),
    ("A", "A", Role::Other, "min_pulse_width"),
];
const FAMILY: [&str; 3] = ["f1", "f2", "f3"];
const INSTS: [&str; 5] = ["u0", "u1", "b0", "b1", "b2"];
const NETS: [&str; 7] = ["ni", "n0", "n1", "nx", "m0", "m1", "m2"];
const MAX_EDITS: usize = 3;

thread_local! {
    static RNG: StdCell<u64> = const { StdCell::new(SEED) };
}

fn reseed(seed: u64) {
    RNG.with(|r| r.set(seed));
}

/// A free choice below `n` (xorshift64).
fn any_below(n: usize) -> usize {
    RNG.with(|r| {
        let mut x = r.get();
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        r.set(x);
        (x % n as u64) as usize
    })
}

fn any_bool() -> bool {
    any_below(2) == 1
}

/// Only the draws where `c` holds count; `None` rejects the draw (not a counterexample).
fn assume(c: bool) -> Option<()> {
    c.then_some(())
}

fn port(name: &str, direction: Direction) -> Port {
    Port { name: name.into(), direction, ..Default::default() }
}

fn arc_set(from: &str, to: &str, role: Role, timing_type: &str) -> ArcSet {
    let arc = |rf: usize| LArc { from_rf: rf, to_rf: rf, model: ArcModel::Gate(GateModel { delay: None, slew: None }) };
    // the liberty reader gives `Other` (width, period, ...) no arcs
    let arcs = if role == Role::Other { Vec::new() } else { vec![arc(0), arc(1)] };
    ArcSet { from: from.into(), to: to.into(), role, timing_type: timing_type.into(), cond: None, arcs }
}

/// A family cell with everything about it free; returns it and its LEF terminal order.
fn any_family_cell(name: &str) -> (Cell, Vec<String>) {
    let order = PERMS[any_below(6)];
    let ports: Vec<Port> = order.iter().map(|&k| port(PORTS[k], if k == 2 { Direction::Output } else { Direction::Input })).collect();
    let n = any_below(3);
    let mut arc_sets = Vec::new();
    for _ in 0..n {
        let (f, t, r, tt) = MENU[any_below(MENU.len())];
        let mut s = arc_set(f, t, r, tt);
        if any_bool() {
            s.cond = Some("B".into());
        }
        arc_sets.push(s);
    }
    let lef: Vec<String> = PERMS[any_below(6)].iter().map(|&k| PORTS[k].to_string()).collect();
    (Cell { name: name.into(), ports, arc_sets, ..Default::default() }, lef)
}

/// Two libraries: the buffer and `f1` in the first, `f2` and `f3` in the second.
fn any_libs() -> ([Library; 2], HashMap<String, Vec<String>>) {
    let mut libs = [Library { name: "l0".into(), ..Default::default() }, Library { name: "l1".into(), ..Default::default() }];
    let mut pins: HashMap<String, Vec<String>> = HashMap::new();
    let buf = Cell { name: "buf".into(), ports: vec![port("A", Direction::Input), port("Y", Direction::Output)], arc_sets: vec![arc_set("A", "Y", Role::Combinational, "combinational")], ..Default::default() };
    libs[0].cells.insert("buf".into(), buf);
    pins.insert("buf".into(), vec!["A".into(), "Y".into()]);
    for (l, name) in [0, 1, 1].into_iter().zip(FAMILY) {
        let (c, lef) = any_family_cell(name);
        libs[l].cells.insert(name.into(), c);
        pins.insert(name.into(), lef);
    }
    (libs, pins)
}

/// A pin in the model: an instance's terminal, or top port `i` (0, input) / `o` (1, output).
#[derive(Clone, Copy, PartialEq)]
enum Pin {
    I(usize, &'static str),
    P(usize),
}

/// The design as the database holds it.
#[derive(Clone)]
struct Model {
    insts: Vec<(usize, &'static str)>,
    nets: Vec<(usize, Vec<Pin>)>,
    o_inout: bool,
}

fn is_family(c: &str) -> bool {
    FAMILY.contains(&c)
}

fn input_pins(c: &str) -> &'static [&'static str] {
    if is_family(c) { &["A", "B"] } else { &["A"] }
}

impl Model {
    fn pin_name(&self, p: Pin) -> String {
        match p {
            Pin::I(i, t) => format!("{}/{t}", INSTS[self.insts[i].0]),
            Pin::P(0) => "i".into(),
            Pin::P(_) => "o".into(),
        }
    }
    fn net_of(&self, p: Pin) -> Option<usize> {
        self.nets.iter().position(|(_, ps)| ps.contains(&p))
    }
    fn on_net(&self, n: usize) -> String {
        let names: Vec<String> = self.nets[n].1.iter().map(|&p| self.pin_name(p)).collect();
        names.join(",")
    }
    fn attach(&mut self, p: Pin, n: usize) {
        let ps = &mut self.nets[n].1;
        let at = if matches!(p, Pin::I(..)) { ps.iter().position(|q| matches!(q, Pin::P(_))).unwrap_or(ps.len()) } else { ps.len() };
        ps.insert(at, p);
    }
    fn detach(&mut self, p: Pin) {
        for (_, ps) in &mut self.nets {
            ps.retain(|&q| q != p);
        }
    }
    fn netlist(&self) -> Netlist {
        let conn = |p: Pin| match p {
            Pin::I(i, t) => Conn::Inst(i, t.into()),
            Pin::P(k) => Conn::Port(k),
        };
        Netlist {
            insts: self.insts.iter().map(|&(n, c)| crate::netlist::inst(INSTS[n], c)).collect(),
            ports: vec![("i".into(), PortDir::Input), ("o".into(), if self.o_inout { PortDir::Inout } else { PortDir::Output })],
            nets: self.nets.iter().map(|(n, ps)| Arc::new(Net { name: NETS[*n].into(), pins: ps.iter().map(|&p| conn(p)).collect() })).collect(),
        }
    }

    // ---- the database's callbacks, each with the event text `dbStaCbk` reports ----
    fn connect(&mut self, p: Pin, n: usize, out: &mut Vec<String>) {
        self.attach(p, n);
        let kind = if matches!(p, Pin::I(..)) { "iterm_connect" } else { "bterm_connect" };
        out.push([kind, &self.pin_name(p), NETS[self.nets[n].0], &self.on_net(n)].join("|"));
    }
    fn disconnect(&mut self, p: Pin, out: &mut Vec<String>) {
        let n = self.net_of(p).expect("callers disconnect a connected pin");
        let kind = if matches!(p, Pin::I(..)) { "iterm_disconnect" } else { "bterm_disconnect" };
        out.push([kind, &self.pin_name(p), NETS[self.nets[n].0], &self.on_net(n)].join("|"));
        self.detach(p);
    }
    fn create_inst(&mut self, cell: &'static str, out: &mut Vec<String>) -> usize {
        let i = self.insts.len();
        self.insts.push((i, cell));
        out.push(["inst_create", INSTS[i], cell].join("|"));
        i
    }
    fn create_net(&mut self, out: &mut Vec<String>) -> usize {
        let name = self.nets.len();
        self.nets.push((name, Vec::new()));
        out.push(["net_create", NETS[name]].join("|"));
        self.nets.len() - 1
    }
    fn swap(&mut self, i: usize, to: &'static str, out: &mut Vec<String>) {
        let from = self.insts[i].1;
        out.push(["swap_before", INSTS[i], from, to, ""].join("|"));
        self.insts[i].1 = to;
        out.push(["swap_after", INSTS[i], ""].join("|"));
    }
    fn any_load(&self) -> Pin {
        let i = any_below(self.insts.len());
        let ins = input_pins(self.insts[i].1);
        Pin::I(i, ins[any_below(ins.len())])
    }
}

/// `u0`, `u1` of any cell, input port `i` on `ni`, each output on its own net, a fourth net `nx`
/// with no driver, each input pin and port `o` on any of those nets or none.
fn any_design() -> Model {
    let cells = ["buf", "f1", "f2", "f3"];
    let mut m = Model { insts: vec![(0, cells[any_below(4)]), (1, cells[any_below(4)])], nets: vec![(0, vec![]), (1, vec![]), (2, vec![]), (3, vec![])], o_inout: any_bool() };
    for i in 0..2 {
        m.attach(Pin::I(i, "Y"), 1 + i);
        for &t in input_pins(m.insts[i].1) {
            let n = any_below(5);
            if n < 4 {
                m.attach(Pin::I(i, t), n);
            }
        }
    }
    m.attach(Pin::P(0), 0);
    let n = any_below(4);
    if n > 0 {
        m.attach(Pin::P(1), n);
    }
    m
}

/// One edit as the resizer and the database may make it: its kind (0 swap, 1 buffer insertion,
/// 2 load moved, 3 load disconnected, 4 load connected), or `None` when not valid here.
fn edit(m: &mut Model, out: &mut Vec<String>, op: usize) -> Option<usize> {
    match op {
        0 => {
            let i = any_below(m.insts.len());
            let to = FAMILY[any_below(3)];
            assume(is_family(m.insts[i].1) && m.insts[i].1 != to)?;
            m.swap(i, to, out);
        }
        1 => {
            let p = m.any_load();
            let n = m.net_of(p)?;
            assume(m.insts.len() < INSTS.len())?;
            let b = m.create_inst("buf", out);
            let k = m.create_net(out);
            m.connect(Pin::I(b, "A"), n, out);
            m.connect(Pin::I(b, "Y"), k, out);
            m.disconnect(p, out);
            m.connect(p, k, out);
        }
        2 => {
            let p = m.any_load();
            let k = any_below(m.nets.len());
            let n = m.net_of(p)?;
            assume(n != k)?;
            m.disconnect(p, out);
            m.connect(p, k, out);
        }
        3 => {
            let p = m.any_load();
            m.net_of(p)?;
            m.disconnect(p, out);
        }
        _ => {
            let p = m.any_load();
            let k = any_below(m.nets.len());
            assume(m.net_of(p).is_none())?;
            m.connect(p, k, out);
        }
    }
    Some(op)
}

/// Which answer the edit gives: the real `apply_edits`, or a vacuity stub.
#[derive(Clone, Copy, PartialEq)]
enum Answer {
    Real,
    StubInPlace,
    StubRebuild,
}

/// A design, a graph built over it, a batch of edits and the graph edited: the answer, and a mask
/// of the edit kinds in the batch. `None`: a rejected draw.
fn edit_any_batch(answer: Answer) -> Option<(Result<(), NotInPlace>, u8)> {
    let (libs, pins) = any_libs();
    let mut m = any_design();
    let mut g = Graph::build_shared(&libs, Arc::new(m.netlist()), Some(&pins)).ok()?;
    let mut events = Vec::new();
    let n = 1 + any_below(MAX_EDITS);
    let mut kinds = 0u8;
    for _ in 0..n {
        kinds |= 1 << edit(&mut m, &mut events, any_below(5))?;
    }
    Some(match answer {
        Answer::Real => (g.apply_edits(&events, Arc::new(m.netlist()), Some(&pins)), kinds),
        Answer::StubInPlace => (Ok(()), kinds),
        Answer::StubRebuild => (Err(NotInPlace::Holes(1)), kinds),
    })
}

/// After an in-place edit, an import reads only what was written since (a sentinel slew on every
/// vertex survives). `None`: a rejected draw or an edit refused (the other property's).
fn import_after_edit() -> Option<Result<(), &'static str>> {
    let (libs, pins) = any_libs();
    let mut m = any_design();
    let mut g = Graph::build_shared(&libs, Arc::new(m.netlist()), Some(&pins)).ok()?;
    let mut t = crate::incr::IncTimer::default();
    t.export(&g);
    t.import(&mut g);
    let mut events = Vec::new();
    for _ in 0..1 + any_below(MAX_EDITS) {
        edit(&mut m, &mut events, any_below(5))?;
    }
    g.apply_edits(&events, Arc::new(m.netlist()), Some(&pins)).ok()?;
    const SENTINEL: f32 = 7.0;
    for s in g.slew.iter_mut() {
        *s = [[SENTINEL; 2]; 2];
    }
    t.import(&mut g);
    Some(if g.slew.iter().all(|s| *s == [[SENTINEL; 2]; 2]) { Ok(()) } else { Err("Import/whole") })
}

/// The class of a refusal, by the API's own answer.
fn class((r, kinds): &(Result<(), NotInPlace>, u8)) -> Option<&'static str> {
    Some(match r {
        Ok(()) => return None,
        Err(NotInPlace::Holes(_)) if *kinds == 1 => "Holes/swaps only",
        Err(NotInPlace::Holes(_)) => "Holes/with pin edits",
        Err(NotInPlace::Event(e)) if e.starts_with("swap_before") => "Event/swap",
        Err(NotInPlace::Event(e)) if e.starts_with("inst_create") => "Event/inst_create",
        Err(NotInPlace::Event(e)) if e.starts_with("iterm_") => "Event/iterm_*",
        Err(NotInPlace::Event(_)) => "Event/other",
        Err(NotInPlace::Netlist(m)) if m.contains("loop") => "Netlist/loop edge deleted",
        Err(NotInPlace::Netlist(_)) => "Netlist/other",
    })
}

/// One seeded run: (valid batches, in place, classes with their count and first sample).
fn run(samples: usize, answer: Answer) -> (usize, usize, BTreeMap<&'static str, (usize, usize)>) {
    reseed(SEED);
    let (mut valid, mut ok) = (0, 0);
    let mut classes: BTreeMap<&'static str, (usize, usize)> = BTreeMap::new();
    for k in 0..samples {
        let Some(r) = edit_any_batch(answer) else { continue };
        valid += 1;
        match class(&r) {
            None => ok += 1,
            Some(c) => classes.entry(c).or_insert((0, k)).0 += 1,
        }
    }
    if answer == Answer::Real {
        for k in 0..samples {
            if let Some(Err(c)) = import_after_edit() {
                classes.entry(c).or_insert((0, k)).0 += 1;
            }
        }
    }
    (valid, ok, classes)
}

fn samples() -> usize {
    std::env::var("VYGES_EDIT_CONTRACT_SAMPLES").ok().and_then(|s| s.parse().ok()).unwrap_or(SAMPLES)
}

/// The contract over every sampled batch: each refusal class met must be on [`EXPECTED`], and each
/// class on it must still be met (else it was fixed: take it off the list).
#[test]
fn the_edit_contract_holds_but_for_the_open_classes() {
    let (valid, ok, classes) = run(samples(), Answer::Real);
    assert!(valid >= samples() / 5, "the generator must reach the edit: {valid} valid of {}", samples());
    assert!(ok > 0, "some batch must be made in place");
    let unexpected: Vec<String> = classes.iter().filter(|(c, _)| !EXPECTED.iter().any(|(e, _)| e == *c)).map(|(c, (n, first))| format!("{c} ({n}x, first at sample {first})")).collect();
    let fixed: Vec<&str> = EXPECTED.iter().filter(|(e, _)| !classes.contains_key(e)).map(|(e, _)| *e).collect();
    assert!(unexpected.is_empty(), "NEW refusal class(es), a fallback reintroduced or a new one: {unexpected:?}");
    assert!(fixed.is_empty(), "class(es) no longer met, fixed? take them off EXPECTED: {fixed:?}");
}

/// Vacuity: the same run over a stub that always answers in place finds no class.
#[test]
fn the_contract_check_passes_an_always_in_place_edit() {
    let (valid, ok, classes) = run(samples(), Answer::StubInPlace);
    assert!(classes.is_empty() && ok == valid && valid > 0, "{valid} valid, {ok} in place, {classes:?}");
}

/// Vacuity: over a stub that always answers rebuild, every valid batch is a refusal.
#[test]
fn the_contract_check_fails_an_always_rebuild_edit() {
    let (valid, ok, classes) = run(samples(), Answer::StubRebuild);
    let refused: usize = classes.values().map(|(n, _)| n).sum();
    assert!(ok == 0 && refused == valid && valid > 0, "{valid} valid, {ok} in place, {classes:?}");
}
