//! Editing a graph in place, from the netlist edits the database's callbacks report, rather than
//! building it again: one function per callback, in the order the callbacks come.
//!
//! Rules:
//! - an instance made: its pin vertices in its liberty cell's port order (the pins its master has),
//!   then its own arcs in the cell's arc-set order;
//! - a pin connected: a driver gets a wire edge to each load on its net, a load one from each
//!   driver, in the net's pin order; a pin disconnected loses its wire edges, newest first;
//! - an edge is made NEWEST in its from vertex's out list and its to vertex's in list, its delays 0;
//!   a deleted edge's id is the next one made (last freed, first reused) — so a load moved from one
//!   driver to another keeps the id of the edge it lost;
//! - an equivalent-arcs swap keeps every edge (the arc sets correspond index for index); any other
//!   swap deletes the arcs out of the instance's inputs, newest first, and makes the new cell's;
//! - a vertex's slews and an edge's delays stay with it: nothing is imported;
//! - an instance or net the database made at a reused id lands mid-order in the netlist: the
//!   instance and net indices the vertices hold move up past it; the vertices themselves do not
//!   (the vertex table appends) — see [`Graph::check_extends`].
//!
//! What is not edited in place — a vertex or net deleted, a port made, an edit that leaves an edge
//! id free, a netlist whose existing instances or nets moved other than by an insertion — is
//! refused, and the caller builds the graph again.

use std::collections::HashMap;
use std::sync::Arc;

use super::{cell, Edge, EdgeKind, Graph, NetlistRef, Vertex};
use crate::liberty::{Direction, Role};
use crate::netlist::{Conn, Netlist};

/// Why a batch of edits was not applied in place; the graph is then unusable and the caller builds
/// it again.
#[derive(Debug, Clone, PartialEq)]
pub enum NotInPlace {
    /// An edit not made in place.
    Event(String),
    /// Edge ids left free at the end of the batch.
    Holes(usize),
    /// The new netlist moved an instance, port or net the graph indexes.
    Netlist(String),
}

impl std::fmt::Display for NotInPlace {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NotInPlace::Event(e) => write!(f, "edit not made in place: {e}"),
            NotInPlace::Holes(n) => write!(f, "{n} edge id(s) left free"),
            NotInPlace::Netlist(m) => write!(f, "netlist: {m}"),
        }
    }
}

/// The edit state of one batch: the vertex index by name, the free edge ids, the pins whose net may
/// have changed and the nets the edits named, the build's master-pin filter, and whether the swap
/// in progress has equivalent arcs (`swap_master_arcs_equiv_`).
struct Batch<'p> {
    index: HashMap<String, usize>,
    free: Vec<usize>,
    touched: Vec<usize>,
    nets: std::collections::HashSet<String>,
    pins: Option<&'p HashMap<String, Vec<String>>>,
    swap_equiv: bool,
    /// The vertices of the instance being swapped.
    swapped: Vec<usize>,
}

/// [`Graph::check_extends`]'s answer: where the new instances went — `at[j]` is the number of old
/// instances before the `j`-th new one, `made_at` its index — and where the old nets went.
struct Moves {
    at: Vec<usize>,
    made_at: HashMap<String, usize>,
    nets: NetMoves,
}

/// Each old net's slot in the new netlist (`map`, empty when every slot stands; `None`: a net the
/// edits named), whether any moved, the old slots the edits named, and the new slots of the names
/// the edits named.
#[derive(Default)]
struct NetMoves {
    map: Vec<Option<usize>>,
    moved: bool,
    named_old: Vec<usize>,
    named_new: Vec<(String, usize)>,
}

/// What the netlists since a graph's last edit changed, by slot, when no slot moves (the caller
/// holds the order: what an edit makes is appended): the instance and net slots replaced or
/// appended, in any order, repeats allowed. With it a graph keeps a netlist of its own and
/// patches those slots ([`Graph::apply_edits_held`]) — the reference's network is edited, never
/// copied — instead of comparing every instance and net of two generations.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SlotDelta {
    pub insts: Vec<usize>,
    pub nets: Vec<usize>,
}

impl SlotDelta {
    /// `other`'s slots added to these.
    pub fn extend(&mut self, other: &SlotDelta) {
        self.insts.extend_from_slice(&other.insts);
        self.nets.extend_from_slice(&other.nets);
    }
}

/// The net names the edits named (made, connected to, disconnected from), in first-named order.
fn named_nets(events: &[String]) -> Vec<&str> {
    let mut named: Vec<&str> = Vec::new();
    for ev in events {
        let f: Vec<&str> = ev.split('|').collect();
        if let ["net_create", net, ..] | ["iterm_connect" | "bterm_connect" | "iterm_disconnect" | "bterm_disconnect", _, net, _] = f.as_slice() {
            if !named.contains(net) {
                named.push(net);
            }
        }
    }
    named
}

/// An old instance at `k` moves up by the number of new ones at or before it.
fn shifted(at: &[usize], k: usize) -> usize {
    k + at.partition_point(|&p| p <= k)
}

impl<'a> Graph<'a> {
    /// The netlist edits `events` (the database's callbacks, in order) made on the graph, which then
    /// takes `netlist` — the generation after them. `pins` is the build's master-pin filter.
    pub fn apply_edits(&mut self, events: &[String], netlist: Arc<Netlist>, pins: Option<&HashMap<String, Vec<String>>>) -> Result<(), NotInPlace> {
        let moves = self.check_extends(&netlist, events)?;
        let mut b = self.batch(pins)?;
        self.shift_instances(&moves.at);
        self.edit_events(&moves, events, &mut b)?;
        let old_nets = self.netlist.nets.len();
        self.shift_nets(&moves.nets, &mut b);
        self.netlist = NetlistRef::Shared(netlist);
        self.update_vertex_nets(&b, old_nets, true)?;
        let _ = self.index_cache.set(Arc::new(b.index));
        Ok(())
    }

    /// [`Graph::apply_edits`] when no slot of the netlist moved since the graph's last edit and
    /// `slots` lists every one replaced or appended: the graph's own netlist is patched at those
    /// slots to `next` (made its own the first time), with no pass over the netlist. Each batch
    /// costs what it edits; comparing two generations and copying one cost the whole design per
    /// repeater. `VYGES_STA_CHECK_EDIT` also checks the patched netlist equals `next`.
    pub fn apply_edits_held(&mut self, events: &[String], next: &Netlist, slots: &SlotDelta, pins: Option<&HashMap<String, Vec<String>>>) -> Result<(), NotInPlace> {
        let moves = self.held_moves(next, events, slots)?;
        let mut b = self.batch(pins)?;
        self.edit_events(&moves, events, &mut b)?;
        let old_nets = self.netlist.nets.len();
        // the vertices on a replaced net the edits named, read again by name (as `shift_nets`)
        for &k in &moves.nets.named_old {
            for c in &self.netlist.nets[k].pins {
                if let Some(&v) = b.index.get(&self.netlist.pin_name(c)) {
                    b.touched.push(v);
                }
            }
        }
        self.patch_netlist(next, slots);
        // the kept slot map takes the slots replaced (a rename moves a name) and appended; a name
        // it then misses is on no net
        if let Some(map) = self.net_slots.as_mut() {
            for &k in &slots.nets {
                if let Some(n) = self.netlist.nets.get(k) {
                    map.insert(n.name.clone(), k);
                }
            }
        }
        self.update_vertex_nets(&b, old_nets, false)?;
        let _ = self.index_cache.set(Arc::new(b.index));
        if std::env::var_os("VYGES_STA_CHECK_EDIT").is_some() && *self.netlist != *next {
            return Err(NotInPlace::Netlist("the patched netlist differs from the next one".into()));
        }
        Ok(())
    }

    /// [`Graph::apply_edits_held`]'s moves: the instances made are the ones appended; no net moved;
    /// a replaced net whose old name the edits named is read again.
    fn held_moves(&self, next: &Netlist, events: &[String], slots: &SlotDelta) -> Result<Moves, NotInPlace> {
        let old = &*self.netlist;
        if next.ports.len() != old.ports.len() {
            return Err(NotInPlace::Netlist("the ports changed".into()));
        }
        if next.insts.len() < old.insts.len() || next.nets.len() < old.nets.len() {
            return Err(NotInPlace::Netlist("a held netlist shrank".into()));
        }
        let made: std::collections::HashSet<&str> = events.iter().filter_map(|e| e.strip_prefix("inst_create|")).filter_map(|r| r.split('|').next()).collect();
        let mut made_at = HashMap::new();
        for (j, i) in next.insts.iter().enumerate().skip(old.insts.len()) {
            if !made.contains(i.0.as_str()) {
                return Err(NotInPlace::Netlist("an instance moved".into()));
            }
            made_at.insert(i.0.clone(), j);
        }
        let named = named_nets(events);
        let mut named_old: Vec<usize> = slots.nets.iter().copied().filter(|&k| k < old.nets.len() && named.contains(&old.nets[k].name.as_str())).collect();
        named_old.sort_unstable();
        named_old.dedup();
        Ok(Moves { at: vec![old.insts.len(); made_at.len()], made_at, nets: NetMoves { named_old, ..NetMoves::default() } })
    }

    /// The graph's netlist made its own (copied once, the first time another holds it) and set to
    /// `next` at the slots listed and the ones appended.
    fn patch_netlist(&mut self, next: &Netlist, slots: &SlotDelta) {
        let unique = matches!(&self.netlist, NetlistRef::Shared(a) if Arc::strong_count(a) == 1 && Arc::weak_count(a) == 0);
        if !unique {
            if std::env::var_os("VYGES_STA_INDEX_TRACE").is_some() {
                eprintln!("VYGINDEX|netlist-copied|{}", self.netlist.nets.len());
            }
            self.netlist = NetlistRef::Shared(Arc::new((*self.netlist).clone()));
        }
        let NetlistRef::Shared(a) = &mut self.netlist else { unreachable!("made shared above") };
        let own = Arc::get_mut(a).expect("held by this graph alone");
        let (ni, nn) = (own.insts.len(), own.nets.len());
        for &k in &slots.insts {
            if k < ni {
                own.insts[k] = next.insts[k].clone();
            }
        }
        own.insts.extend(next.insts[ni..].iter().cloned());
        for &k in &slots.nets {
            if k < nn {
                own.nets[k] = next.nets[k].clone();
            }
        }
        own.nets.extend(next.nets[nn..].iter().cloned());
    }

    /// The batch state: the vertex index taken over from the last build or edit.
    fn batch<'p>(&mut self, pins: Option<&'p HashMap<String, Vec<String>>>) -> Result<Batch<'p>, NotInPlace> {
        let index = match self.index_cache.take() {
            Some(ix) => Arc::try_unwrap(ix).unwrap_or_else(|shared| {
                // ⚠️ O(graph): another holder of the index makes every edit copy it
                if std::env::var_os("VYGES_STA_INDEX_TRACE").is_some() {
                    eprintln!("VYGINDEX|copied|{}", Arc::strong_count(&shared));
                }
                (*shared).clone()
            }),
            None => {
                // ⚠️ O(graph): no index kept from the last build or edit
                if std::env::var_os("VYGES_STA_INDEX_TRACE").is_some() {
                    eprintln!("VYGINDEX|built|{}", self.vertices.len());
                }
                self.vertices.iter().enumerate().map(|(i, v)| (v.name.clone(), i)).collect()
            }
        };
        if index.len() != self.vertices.len() {
            return Err(NotInPlace::Event("a pin name shared by two vertices".into()));
        }
        Ok(Batch { index, free: Vec::new(), touched: Vec::new(), nets: std::collections::HashSet::new(), pins, swap_equiv: false, swapped: Vec::new() })
    }

    /// Each event made on the graph, in order (one function per callback).
    fn edit_events(&mut self, moves: &Moves, events: &[String], b: &mut Batch<'_>) -> Result<(), NotInPlace> {
        for ev in events {
            let f: Vec<&str> = ev.split('|').collect();
            match f.as_slice() {
                ["inst_create", inst, master] => self.make_instance_after(moves, inst, master, b)?,
                ["iterm_connect" | "bterm_connect", pin, net, on_net] => {
                    b.nets.insert(net.to_string());
                    self.connect_pin_after(pin, on_net, b);
                }
                ["iterm_disconnect" | "bterm_disconnect", pin, net, _on_net] => {
                    b.nets.insert(net.to_string());
                    self.disconnect_pin_before(pin, b);
                }
                ["swap_before", inst, from, to, _terms] => self.replace_cell_before(inst, from, to, ev, b)?,
                ["swap_after", inst, _terms] => self.replace_cell_after(inst, b),
                // a net made under one name may hold another by the end (`swapNetNames`): the
                // name it was made with is looked for too
                ["net_create", net, ..] => {
                    b.nets.insert(net.to_string());
                }
                _ => return Err(NotInPlace::Event(ev.clone())),
            }
        }
        if !b.free.is_empty() {
            return Err(NotInPlace::Holes(b.free.len()));
        }
        Ok(())
    }

    /// Where the database put what the edits made, against the graph's netlist (`dbTable::create`
    /// takes the LAST freed id, `dbBlock` iterates by id: after a deletion — `remove_buffers`, or a
    /// database written after one — a new instance or net lands at its id's rank, in the middle).
    ///
    /// - Instances: the old ones, in order, are the new list less the ones `inst_create` named; any
    ///   other difference is a move.
    /// - Nets: an edit replaces the nets it names in their slots, inserts the ones it makes, and may
    ///   trade two of their names (`swapNetNames`: no callback). So the old nets the edits did not
    ///   name, in order, are the new list less the names the edits named; a net named by no edit
    ///   that is not where that says is a move. An entry shared with the old netlist (the same
    ///   allocation) is the same net.
    ///
    /// The graph's own vertices are not moved: `Graph::makeVertex` takes the next id of the
    /// vertex table (appended, as here), whatever the network's order; only the netlist indices a
    /// vertex holds — its instance, its net's slot — follow the insertion.
    fn check_extends(&self, next: &Netlist, events: &[String]) -> Result<Moves, NotInPlace> {
        let old = &*self.netlist;
        if next.ports != old.ports {
            return Err(NotInPlace::Netlist("the ports changed".into()));
        }
        let made: std::collections::HashSet<&str> = events.iter().filter_map(|e| e.strip_prefix("inst_create|")).filter_map(|r| r.split('|').next()).collect();
        let mut at = Vec::new();
        let mut made_at = HashMap::new();
        let mut i = 0;
        for (j, b) in next.insts.iter().enumerate() {
            if old.insts.get(i).is_some_and(|a| Arc::ptr_eq(a, b) || a.0 == b.0) {
                i += 1;
            } else if made.contains(b.0.as_str()) {
                at.push(i);
                made_at.insert(b.0.clone(), j);
            } else {
                return Err(NotInPlace::Netlist("an instance moved".into()));
            }
        }
        if i != old.insts.len() {
            return Err(NotInPlace::Netlist("an instance moved".into()));
        }
        // The names the edits named, as a short list: most batches name a few nets, and this is
        // asked of every net whose entry the edit did not share.
        let named = named_nets(events);
        let big = (named.len() > 16).then(|| named.iter().copied().collect::<std::collections::HashSet<&str>>());
        let is_named = |n: &str| match &big {
            Some(set) => set.contains(n),
            None => named.iter().any(|x| x.len() == n.len() && *x == n),
        };
        match Self::align_nets(old, next, &at, &is_named) {
            Ok(nets) => Ok(Moves { at, made_at, nets }),
            // a rename no edit named (a name swapped onto an appended net): the slots stand
            Err(_) if at.iter().all(|&p| p >= old.insts.len()) && Self::renamed_to_appended(old, next) => Ok(Moves { at, made_at, nets: NetMoves::default() }),
            Err(why) => Err(why),
        }
    }

    /// The old nets the edits did not name, in order, against the new list less the names the edits
    /// named: each found at the same allocation, or by name with the same pins (its instance
    /// indices moved past the new instances, `at`). An old net the edits named is not placed: its
    /// pins are read again by name.
    fn align_nets(old: &Netlist, next: &Netlist, at: &[usize], is_named: &dyn Fn(&str) -> bool) -> Result<NetMoves, NotInPlace> {
        let same_pins = |a: &crate::netlist::Net, b: &crate::netlist::Net| {
            a.pins.len() == b.pins.len()
                && a.pins.iter().zip(&b.pins).all(|(x, y)| match (x, y) {
                    (Conn::Inst(i, p), Conn::Inst(j, q)) => shifted(at, *i) == *j && p == q,
                    (Conn::Port(i), Conn::Port(j)) => i == j,
                    _ => false,
                })
        };
        let mut m = NetMoves { map: vec![None; old.nets.len()], ..NetMoves::default() };
        let mut j = 0;
        for (k, a) in old.nets.iter().enumerate() {
            while let Some(b) = next.nets.get(j).filter(|b| !Arc::ptr_eq(a, b) && is_named(&b.name)) {
                m.named_new.push((b.name.clone(), j));
                j += 1;
            }
            let b = next.nets.get(j);
            if b.is_some_and(|b| Arc::ptr_eq(a, b)) || (!is_named(&a.name) && b.is_some_and(|b| b.name == a.name && same_pins(a, b))) {
                m.map[k] = Some(j);
                m.moved |= j != k;
                j += 1;
            } else if is_named(&a.name) {
                m.named_old.push(k);
            } else {
                let now = b.map_or("the end", |b| b.name.as_str());
                return Err(NotInPlace::Netlist(format!("a net moved: {} at {k} meets {now} at {j} ({} nets, were {})", a.name, next.nets.len(), old.nets.len())));
            }
        }
        for (j, b) in next.nets.iter().enumerate().skip(j) {
            if !is_named(&b.name) {
                return Err(NotInPlace::Netlist(format!("a net no edit named: {}", b.name)));
            }
            m.named_new.push((b.name.clone(), j));
        }
        Ok(m)
    }

    /// The rule before reused ids: every old net keeps its slot, nets are appended, and a slot whose
    /// name differs was renamed to one of the appended nets' names (`swapNetNames` raises no
    /// callback).
    fn renamed_to_appended(old: &Netlist, next: &Netlist) -> bool {
        let Some(appended) = next.nets.get(old.nets.len()..) else { return false };
        old.nets.iter().zip(&next.nets).all(|(a, b)| Arc::ptr_eq(a, b) || a.name == b.name || appended.iter().any(|n| n.name == a.name))
    }

    /// The instance indices the vertices hold, moved up past the instances made before them
    /// ([`Moves::at`]).
    fn shift_instances(&mut self, at: &[usize]) {
        let Some(&first) = at.first().filter(|&&p| p < self.netlist.insts.len()) else { return };
        for v in &mut self.vertices {
            if let Conn::Inst(i, _) = &mut v.conn {
                if *i >= first {
                    *i = shifted(at, *i);
                }
            }
        }
    }

    /// Each vertex's net slot, moved to where its net went ([`NetMoves`]); a vertex on a net the
    /// edits named is read again by name ([`Graph::update_vertex_nets`]) — found from the old
    /// net's own pins, before the netlist moves on. The kept slot map follows.
    fn shift_nets(&mut self, moves: &NetMoves, b: &mut Batch<'_>) {
        for &k in &moves.named_old {
            for c in &self.netlist.nets[k].pins {
                if let Some(&v) = b.index.get(&self.netlist.pin_name(c)) {
                    b.touched.push(v);
                }
            }
        }
        if moves.moved {
            for n in &mut self.vertex_net {
                if let Some(k) = *n {
                    // a net the edits named: its vertices were just listed to be read again
                    *n = moves.map[k];
                }
            }
        }
        if let Some(slots) = self.net_slots.as_mut() {
            if moves.moved {
                slots.retain(|_, k| match moves.map.get(*k).copied().flatten() {
                    Some(j) => {
                        *k = j;
                        true
                    }
                    None => false,
                });
            }
            for (name, j) in &moves.named_new {
                slots.insert(name.clone(), *j);
            }
        }
    }

    /// `Sta::makeInstanceAfter`: `makePinVertices` (the liberty cell's ports, as the master has
    /// them), then `makeInstanceEdges`.
    /// The vertices are appended (the vertex table's next ids); each holds the instance's index in
    /// the new netlist, which after a deletion is mid-order ([`Graph::check_extends`]).
    fn make_instance_after(&mut self, moves: &Moves, inst: &str, master: &str, b: &mut Batch<'_>) -> Result<(), NotInPlace> {
        let Some(lib) = self.libs.iter().position(|l| l.cells.contains_key(master)) else {
            // a physical-only cell: nothing to time
            return Ok(());
        };
        let i = *moves.made_at.get(inst).ok_or_else(|| NotInPlace::Netlist(format!("instance {inst} not in the netlist")))?;
        let has = b.pins.and_then(|m| m.get(master));
        let c = cell(self.libs, lib, master);
        for p in &c.ports {
            if has.is_some_and(|h| !h.contains(&p.name)) {
                continue;
            }
            let is_driver = match p.direction {
                Direction::Input => false,
                Direction::Output | Direction::Tristate => true,
                Direction::Bidirect => return Err(NotInPlace::Event(format!("inst_create|{inst}: a bidirect pin"))),
                _ => continue,
            };
            let name = format!("{inst}/{}", p.name);
            if b.index.contains_key(&name) {
                return Err(NotInPlace::Event(format!("inst_create|{inst}: {name} has a vertex")));
            }
            let v = self.make_vertex(Vertex { name: name.clone(), conn: Conn::Inst(i, p.name.clone()), is_driver, lib: Some(lib), cell: Some(master.to_string()), port: Some(p.name.clone()) });
            b.index.insert(name, v);
            b.touched.push(v);
        }
        self.make_instance_edges(inst, lib, master, b);
        Ok(())
    }

    /// `Graph::makeVertex`: the next vertex, its slews 0, on no net.
    fn make_vertex(&mut self, v: Vertex) -> usize {
        self.vertices.push(v);
        self.in_edges.push(Vec::new());
        self.out_edges.push(Vec::new());
        self.vertex_net.push(None);
        self.slew.push([[0.0; 2]; 2]);
        self.vertices.len() - 1
    }

    /// `Graph::makeInstanceEdges`: an edge per arc set of the cell between two of the instance's
    /// vertices, in the cell's arc-set order (the build's rule: no `Other` role, no empty set but a
    /// tristate one).
    fn make_instance_edges(&mut self, inst: &str, lib: usize, master: &str, b: &mut Batch<'_>) {
        let c = cell(self.libs, lib, master);
        for (k, set) in c.arc_sets.iter().enumerate() {
            let tristate = matches!(set.role, Role::TristateEnable | Role::TristateDisable);
            if set.role == Role::Other || (set.arcs.is_empty() && !tristate) {
                continue;
            }
            let (Some(&f), Some(&t)) = (b.index.get(&format!("{inst}/{}", set.from)), b.index.get(&format!("{inst}/{}", set.to))) else { continue };
            self.make_edge(f, t, EdgeKind::Gate { set: k }, set.arcs.len(), b);
        }
    }

    /// `Graph::makeEdge`: a free id if there is one (the last freed), else the next; newest in both
    /// lists; delays 0 (`initArcDelays`).
    fn make_edge(&mut self, from: usize, to: usize, kind: EdgeKind, rows: usize, b: &mut Batch<'_>) -> usize {
        let edge = Edge { from, to, kind };
        let e = match b.free.pop() {
            Some(e) => {
                self.edges[e] = edge;
                self.delay[e] = vec![[0.0; 2]; rows];
                e
            }
            None => {
                self.edges.push(edge);
                self.delay.push(vec![[0.0; 2]; rows]);
                self.edges.len() - 1
            }
        };
        self.out_edges[from].push(e);
        self.in_edges[to].push(e);
        e
    }

    /// `Graph::deleteEdge`: out of both lists (the rest keep their order), its id free.
    fn delete_edge(&mut self, e: usize, b: &mut Batch<'_>) {
        let Edge { from, to, .. } = self.edges[e];
        self.out_edges[from].retain(|&x| x != e);
        self.in_edges[to].retain(|&x| x != e);
        self.delay[e].clear();
        b.free.push(e);
    }

    /// `Sta::connectPinAfter`: `makeWireEdgesFromPin` for a driver (to each load on the net),
    /// `makeWireEdgesToPin` for a load (from each driver), in the net's pin order.
    fn connect_pin_after(&mut self, pin: &str, on_net: &str, b: &mut Batch<'_>) {
        let Some(&v) = b.index.get(pin) else { return };
        b.touched.push(v);
        let others: Vec<usize> = on_net.split(',').filter(|p| !p.is_empty() && *p != pin).filter_map(|p| b.index.get(p).copied()).collect();
        if self.vertices[v].is_driver {
            let loads: Vec<usize> = others.iter().copied().filter(|&l| !self.vertices[l].is_driver).collect();
            for l in loads {
                self.make_edge(v, l, EdgeKind::Wire, 2, b);
            }
        } else {
            let drivers: Vec<usize> = others.iter().copied().filter(|&d| self.vertices[d].is_driver).collect();
            for d in drivers {
                self.make_edge(d, v, EdgeKind::Wire, 2, b);
            }
        }
    }

    /// `Sta::disconnectPinBefore`: a driver's wire edges out, a load's wire edges in, deleted newest
    /// first (the edge iterator's order).
    fn disconnect_pin_before(&mut self, pin: &str, b: &mut Batch<'_>) {
        let Some(&v) = b.index.get(pin) else { return };
        b.touched.push(v);
        let list = if self.vertices[v].is_driver { &self.out_edges[v] } else { &self.in_edges[v] };
        let wires: Vec<usize> = list.iter().rev().copied().filter(|&e| matches!(self.edges[e].kind, EdgeKind::Wire)).collect();
        for e in wires {
            self.delete_edge(e, b);
        }
    }

    /// The vertices an instance of `master` has, as the build makes them: the cell's ports the
    /// master has, by name — `(port, driver)` in port order.
    fn cell_vertices(&self, lib: usize, master: &str, b: &Batch<'_>) -> Vec<(String, bool)> {
        let has = b.pins.and_then(|m| m.get(master));
        cell(self.libs, lib, master)
            .ports
            .iter()
            .filter(|p| !has.is_some_and(|h| !h.contains(&p.name)))
            .filter_map(|p| match p.direction {
                Direction::Input => Some((p.name.clone(), false)),
                Direction::Output | Direction::Tristate => Some((p.name.clone(), true)),
                _ => None,
            })
            .collect()
    }

    /// `dbStaCbk::inDbInstSwapMasterBefore`: equivalent arcs → `replaceEquivCellBefore` (the arc
    /// sets correspond index for index, so every edge stands), else `replaceCellBefore` (the arcs
    /// out of the instance's inputs deleted, newest first). Either way the instance's vertices take
    /// the new cell. A swap that changes the instance's vertices, or a cell with an arc between two
    /// outputs (which the reference keeps across a swap and makes again after it), is not made here.
    fn replace_cell_before(&mut self, inst: &str, from: &str, to: &str, ev: &str, b: &mut Batch<'_>) -> Result<(), NotInPlace> {
        let refuse = || NotInPlace::Event(ev.to_string());
        let (Some(fl), Some(tl)) = (self.libs.iter().position(|l| l.cells.contains_key(from)), self.libs.iter().position(|l| l.cells.contains_key(to))) else {
            return Err(refuse());
        };
        let had = self.cell_vertices(fl, from, b);
        if self.cell_vertices(tl, to, b) != had {
            return Err(refuse());
        }
        let mut vs = Vec::with_capacity(had.len());
        for (port, driver) in &had {
            match b.index.get(&format!("{inst}/{port}")) {
                Some(&v) if self.vertices[v].is_driver == *driver => vs.push(v),
                _ => return Err(refuse()),
            }
        }
        let (fc, tc) = (cell(self.libs, fl, from), cell(self.libs, tl, to));
        b.swap_equiv = crate::incr::equiv_cells_arcs(fc, tc);
        if !b.swap_equiv {
            let out_to_out = |c: &crate::liberty::Cell| c.arc_sets.iter().any(|s| c.port(&s.from).is_some_and(|p| matches!(p.direction, Direction::Output | Direction::Tristate)));
            if out_to_out(fc) || out_to_out(tc) {
                return Err(refuse());
            }
            let inputs: Vec<usize> = vs.iter().copied().filter(|&v| !self.vertices[v].is_driver).collect();
            for v in inputs {
                let gates: Vec<usize> = self.out_edges[v].iter().rev().copied().filter(|&e| matches!(self.edges[e].kind, EdgeKind::Gate { .. })).collect();
                for e in gates {
                    self.delete_edge(e, b);
                }
            }
        }
        for &v in &vs {
            self.vertices[v].lib = Some(tl);
            self.vertices[v].cell = Some(to.to_string());
        }
        b.swapped = vs;
        Ok(())
    }

    /// `Sta::replaceCellAfter`: `makeInstanceEdges` from the new cell; after an equivalent swap
    /// (`replaceEquivCellAfter`) nothing more for the graph.
    fn replace_cell_after(&mut self, inst: &str, b: &mut Batch<'_>) {
        let swapped = std::mem::take(&mut b.swapped);
        if b.swap_equiv {
            return;
        }
        let Some(&v) = swapped.first() else { return };
        let (Some(lib), Some(master)) = (self.vertices[v].lib, self.vertices[v].cell.clone()) else { return };
        self.make_instance_edges(inst, lib, &master, b);
    }

    /// The net of each pin the batch connected, disconnected or made, from the netlist it moved
    /// to: the nets its edits named, and the ones the netlist added; a pin on none of them is on no
    /// net.
    fn update_vertex_nets(&mut self, b: &Batch<'_>, old_nets: usize, look_for_misses: bool) -> Result<(), NotInPlace> {
        if b.touched.is_empty() {
            return Ok(());
        }
        let slots = self.named_net_slots(&b.nets, old_nets, look_for_misses);
        let mut want: HashMap<&str, usize> = b.touched.iter().map(|&v| (self.vertices[v].name.as_str(), v)).collect();
        let mut found: Vec<(usize, Option<usize>)> = Vec::with_capacity(want.len());
        for n in slots {
            let net = &self.netlist.nets[n];
            for c in &net.pins {
                let name = self.netlist.pin_name(c);
                if let Some(v) = want.remove(name.as_str()) {
                    found.push((v, Some(n)));
                }
            }
        }
        found.extend(want.into_values().map(|v| (v, None)));
        for (v, n) in found {
            self.vertex_net[v] = n;
        }
        Ok(())
    }

    /// The slots of the nets named `names` in the netlist, ascending — the nets a pass over every
    /// net would meet. The kept map takes the nets appended after `old_nets`; a name it holds at a
    /// slot where the netlist has another name (a rename moves a name, not a slot), or a name it
    /// does not hold, is looked for by a pass over the netlist and entered — the names missed, not
    /// the map again (on picorv32 a miss came every twentieth edit, each a whole-map rebuild).
    fn named_net_slots(&mut self, names: &std::collections::HashSet<String>, old_nets: usize, look_for_misses: bool) -> Vec<usize> {
        let nets = &self.netlist.nets;
        let map = match self.net_slots.as_mut() {
            Some(map) => {
                for (k, n) in nets.iter().enumerate().skip(old_nets) {
                    map.insert(n.name.clone(), k);
                }
                map
            }
            None => self.net_slots.insert(nets.iter().enumerate().map(|(k, n)| (n.name.clone(), k)).collect()),
        };
        let held = |map: &HashMap<String, usize>, name: &String| map.get(name).copied().filter(|&k| nets.get(k).is_some_and(|n| &n.name == name));
        let mut missed: Vec<&String> = if look_for_misses { names.iter().filter(|name| held(map, name).is_none()).collect() } else { Vec::new() };
        if !missed.is_empty() {
            if std::env::var_os("VYGES_STA_NET_SLOTS_TRACE").is_some() {
                eprintln!("VYGSLOTS|missed|{missed:?}");
            }
            for (k, n) in nets.iter().enumerate() {
                if let Some(i) = missed.iter().position(|m| **m == n.name) {
                    map.insert(n.name.clone(), k);
                    missed.swap_remove(i);
                    if missed.is_empty() {
                        break;
                    }
                }
            }
        }
        // A name on no net in the netlist has no slot.
        let mut out: Vec<usize> = names.iter().filter_map(|name| held(map, name)).collect();
        out.sort_unstable();
        out
    }

    /// The graph against a fresh build over the same netlist, modulo vertex and edge ids and the
    /// order of each vertex's edge lists: the same vertices by name (pin, instance and port, driver,
    /// cell, net), each with the same edges in and out (from, to, kind, delay rows). `None`: equal.
    pub fn differs_from_build(&self, fresh: &Graph<'_>) -> Option<String> {
        if self.vertices.len() != fresh.vertices.len() {
            return Some(format!("{} vertices, a fresh build {}", self.vertices.len(), fresh.vertices.len()));
        }
        // by name and side: a bidirect pin is two vertices of one name, its driver and its load
        let fix: HashMap<(&str, bool), usize> = fresh.vertices.iter().enumerate().map(|(i, v)| ((v.name.as_str(), v.is_driver), i)).collect();
        let pin = |g: &Graph<'_>, v: usize| -> (String, Option<String>, bool, Option<String>, Option<String>) {
            let x = &g.vertices[v];
            (g.netlist.pin_name(&x.conn), x.port.clone(), x.is_driver, x.cell.clone(), g.vertex_net[v].map(|n| g.netlist.nets[n].name.clone()))
        };
        let edges = |g: &Graph<'_>, list: &[usize]| -> Vec<(String, String, String, usize)> {
            let mut l: Vec<(String, String, String, usize)> = list
                .iter()
                .map(|&e| {
                    let ed = &g.edges[e];
                    (g.vertices[ed.from].name.clone(), g.vertices[ed.to].name.clone(), format!("{:?}", ed.kind), g.delay[e].len())
                })
                .collect();
            l.sort();
            l
        };
        for (v, x) in self.vertices.iter().enumerate() {
            let Some(&w) = fix.get(&(x.name.as_str(), x.is_driver)) else { return Some(format!("{}: no vertex in a fresh build", x.name)) };
            if pin(self, v) != pin(fresh, w) {
                return Some(format!("{}: {:?}, a fresh build {:?}", x.name, pin(self, v), pin(fresh, w)));
            }
            if edges(self, &self.in_edges[v]) != edges(fresh, &fresh.in_edges[w]) {
                return Some(format!("{}: in edges {:?}, a fresh build {:?}", x.name, edges(self, &self.in_edges[v]), edges(fresh, &fresh.in_edges[w])));
            }
            if edges(self, &self.out_edges[v]) != edges(fresh, &fresh.out_edges[w]) {
                return Some(format!("{}: out edges {:?}, a fresh build {:?}", x.name, edges(self, &self.out_edges[v]), edges(fresh, &fresh.out_edges[w])));
            }
        }
        if self.edges.len() != fresh.edges.len() {
            return Some(format!("{} edges, a fresh build {}", self.edges.len(), fresh.edges.len()));
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::liberty::Library;
    use crate::netlist::{Net, PortDir};

    const LIB: &str = r#"library (t) {
      time_unit : "1ns"; capacitive_load_unit (1, pf);
      cell (and2) {
        pin (A) { direction : input; capacitance : 0.001; }
        pin (B) { direction : input; capacitance : 0.001; }
        pin (Y) { direction : output; function : "A&B";
          timing () { related_pin : "A"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("0.1"); } rise_transition (scalar) { values ("1.0"); }
            cell_fall (scalar) { values ("0.1"); } fall_transition (scalar) { values ("1.0"); } }
          timing () { related_pin : "B"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("0.1"); } rise_transition (scalar) { values ("1.0"); }
            cell_fall (scalar) { values ("0.1"); } fall_transition (scalar) { values ("1.0"); } }
        }
      }
      cell (buf) {
        pin (A) { direction : input; capacitance : 0.001; }
        pin (Y) { direction : output; function : "A";
          timing () { related_pin : "A"; timing_sense : positive_unate;
            cell_rise (scalar) { values ("0.1"); } rise_transition (scalar) { values ("1.0"); }
            cell_fall (scalar) { values ("0.1"); } fall_transition (scalar) { values ("1.0"); } }
        }
      }
    }"#;

    fn libs() -> [Library; 1] {
        [Library::read(&crate::liberty_parse::parse(LIB).unwrap()).unwrap()]
    }

    /// Two `and2` in a chain: ports a, b into u1; u1/Y (net n1) and port c into u2; u2/Y out to y.
    /// `buffered`: a `buf1` appended, n1 now u1/Y → buf1/A and a new net n2 buf1/Y → u2/A.
    fn chain(buffered: bool) -> Netlist {
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        let mut nl = Netlist {
            insts: vec![crate::netlist::inst("u1", "and2"), crate::netlist::inst("u2", "and2")],
            ports: vec![("a".into(), PortDir::Input), ("b".into(), PortDir::Input), ("c".into(), PortDir::Input), ("y".into(), PortDir::Output)],
            nets: vec![
                Net { name: "a".into(), pins: vec![pin(0, "A"), Conn::Port(0)] }.into(),
                Net { name: "b".into(), pins: vec![pin(0, "B"), Conn::Port(1)] }.into(),
                Net { name: "n1".into(), pins: vec![pin(0, "Y"), pin(1, "A")] }.into(),
                Net { name: "c".into(), pins: vec![pin(1, "B"), Conn::Port(2)] }.into(),
                Net { name: "y".into(), pins: vec![pin(1, "Y"), Conn::Port(3)] }.into(),
            ],
        };
        if buffered {
            nl.insts.push(crate::netlist::inst("buf1", "buf"));
            Arc::make_mut(&mut nl.nets[2]).pins = vec![pin(0, "Y"), pin(2, "A")];
            nl.nets.push(Net { name: "n2".into(), pins: vec![pin(2, "Y"), pin(1, "A")] }.into());
        }
        nl
    }

    /// The database's callbacks for a buffer inserted before u2/A, as the repair makes them.
    fn insertion() -> Vec<String> {
        [
            "inst_create|buf1|buf",
            "net_create|n2",
            "iterm_connect|buf1/A|n1|u1/Y,u2/A,buf1/A",
            "iterm_connect|buf1/Y|n2|buf1/Y",
            "iterm_disconnect|u2/A|n1|u1/Y,u2/A,buf1/A",
            "iterm_connect|u2/A|n2|buf1/Y,u2/A",
        ]
        .map(String::from)
        .to_vec()
    }

    fn edge(g: &Graph<'_>, from: &str, to: &str) -> Option<usize> {
        (0..g.edges.len()).find(|&e| g.vertices[g.edges[e].from].name == from && g.vertices[g.edges[e].to].name == to && g.out_edges[g.edges[e].from].contains(&e))
    }

    /// Rule (`deleteEdge` frees its id to the object table, `makeEdge` takes the last freed first):
    /// a load moved from one driver to another keeps the id of the wire edge it lost, so a complete
    /// insertion leaves no id free; the reused edge's delays are 0 (`initArcDelays`), and the graph
    /// equals a fresh build of the buffered netlist modulo ids and list order.
    #[test]
    fn an_inserted_buffer_reuses_the_edge_its_load_lost() {
        let libs = libs();
        let mut g = Graph::build_shared(&libs, Arc::new(chain(false)), None).unwrap();
        let lost = edge(&g, "u1/Y", "u2/A").unwrap();
        let (nv, ne) = (g.vertices.len(), g.edges.len());
        g.delay[lost] = vec![[7.0; 2]; g.delay[lost].len()];
        g.apply_edits(&insertion(), Arc::new(chain(true)), None).unwrap();
        assert_eq!(edge(&g, "buf1/Y", "u2/A"), Some(lost), "the reconnected load takes the freed id");
        assert!(g.delay[lost].iter().all(|d| *d == [0.0; 2]), "a reused id's delays are 0");
        assert_eq!((g.vertices.len(), g.edges.len()), (nv + 2, ne + 2), "two vertices, the buffer's arc and one new wire edge");
        let fresh_nl = chain(true);
        let fresh = Graph::build(&libs, &fresh_nl).unwrap();
        assert_eq!(g.differs_from_build(&fresh), None);
    }

    /// Rule (the reference's network is edited, never copied): with the order held, the slots an
    /// edit replaced or appended patch the graph's own netlist; the result is the next netlist,
    /// the graph equals a fresh build of it, and the caller's netlist is left as it was.
    #[test]
    fn a_held_edit_patches_its_slots_and_matches_a_fresh_build() {
        let libs = libs();
        let first = Arc::new(chain(false));
        let mut g = Graph::build_shared(&libs, first.clone(), None).unwrap();
        let next = chain(true);
        let slots = SlotDelta { insts: vec![2], nets: vec![2, 5] };
        g.apply_edits_held(&insertion(), &next, &slots, None).unwrap();
        assert_eq!(*g.netlist, next, "the patched slots give the next netlist");
        assert_eq!(*first, chain(false), "the netlist the graph was built from is not written");
        let fresh = Graph::build(&libs, &next).unwrap();
        assert_eq!(g.differs_from_build(&fresh), None);
        assert!(edge(&g, "buf1/Y", "u2/A").is_some());
    }

    /// Rule (an edit replaces nets in their slots or appends them; a rename moves a NAME, not a
    /// slot): the slot map kept across edits answers as a pass over every net — after an append,
    /// and after two slots trade names (both stale in the map: found by the pass).
    #[test]
    fn the_kept_net_slots_answer_as_a_pass_over_every_net() {
        let libs = libs();
        let mut g = Graph::build_shared(&libs, Arc::new(chain(false)), None).unwrap();
        let names = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<std::collections::HashSet<String>>();
        assert_eq!(g.named_net_slots(&names(&["n1", "y"]), 5, true), vec![2, 4]);
        g.netlist = NetlistRef::Shared(Arc::new(chain(true)));
        assert_eq!(g.named_net_slots(&names(&["n2", "n1"]), 5, true), vec![2, 5], "an appended net enters the map");
        let mut traded = chain(true);
        Arc::make_mut(&mut traded.nets[2]).name = "n2".into();
        Arc::make_mut(&mut traded.nets[5]).name = "n1".into();
        g.netlist = NetlistRef::Shared(Arc::new(traded));
        assert_eq!(g.named_net_slots(&names(&["n1"]), 6, true), vec![5]);
        assert_eq!(g.named_net_slots(&names(&["n2"]), 6, true), vec![2]);
        // a name the map never held, at a slot below the old count; a name on no net has no slot
        let mut moved = chain(true);
        Arc::make_mut(&mut moved.nets[3]).name = "fresh".into();
        g.netlist = NetlistRef::Shared(Arc::new(moved));
        assert_eq!(g.named_net_slots(&names(&["fresh", "nowhere", "n1"]), 6, true), vec![2, 3], "n1 back at 2: its kept slot 5 is stale");
    }

    /// Rule: an entry the next netlist shares with the graph's (the same allocation) is the same
    /// entry; one it replaced is compared by name, so a replaced instance of another name is still
    /// a move.
    #[test]
    fn a_replaced_instance_of_another_name_is_a_move() {
        let libs = libs();
        let old = Arc::new(chain(false));
        let g = Graph::build_shared(&libs, old.clone(), None).unwrap();
        let mut next = (*old).clone();
        assert!(g.check_extends(&next, &[]).is_ok(), "every entry shared");
        next.insts[1] = crate::netlist::inst("u9", "and2");
        assert!(matches!(g.check_extends(&next, &[]), Err(NotInPlace::Netlist(_))));
    }

    /// Rule (odb `dbTable::create` takes the LAST freed id and `dbBlock` iterates by id, so after a
    /// deletion a new instance and its net land mid-order; OpenSTA `Graph::makeVertex` takes the
    /// vertex table's next id, and its `VertexIterator` walks the network, not the table): the
    /// buffer's vertices are appended, the instance and net indices the old vertices hold move up
    /// past the insertion, and the graph equals a fresh build of the new netlist modulo ids.
    #[test]
    fn a_buffer_at_a_reused_id_lands_mid_order_and_the_indices_move_up() {
        let libs = libs();
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        let old = chain(false);
        let mut g = Graph::build_shared(&libs, Arc::new(old.clone()), None).unwrap();
        let nv = g.vertices.len();
        let mut nl = old;
        nl.insts.insert(1, crate::netlist::inst("buf1", "buf"));
        Arc::make_mut(&mut nl.nets[2]).pins = vec![pin(0, "Y"), pin(1, "A")];
        nl.nets.insert(3, Net { name: "n2".into(), pins: vec![pin(1, "Y"), pin(2, "A")] }.into());
        for net in &mut nl.nets[4..6] {
            for c in &mut Arc::make_mut(net).pins {
                if let Conn::Inst(i, _) = c {
                    *i += 1;
                }
            }
        }
        let nl = Arc::new(nl);
        g.apply_edits(&insertion(), nl.clone(), None).unwrap();
        assert_eq!(g.vertices[nv].name, "buf1/A", "the buffer's vertices are appended");
        assert!(matches!(g.vertices[nv].conn, Conn::Inst(1, _)), "and hold its mid-order index");
        let u2y = g.vertices.iter().position(|v| v.name == "u2/Y").unwrap();
        assert!(matches!(g.vertices[u2y].conn, Conn::Inst(2, _)), "u2 moved up one");
        assert_eq!(g.vertex_net[u2y], Some(5), "y moved up one");
        let fresh = Graph::build_shared(&libs, nl, None).unwrap();
        assert_eq!(g.differs_from_build(&fresh), None);
    }

    /// Rule (as above, with `swapNetNames`, which raises no callback): a buffer at an output port,
    /// its instance first in the order and its net inserted BEFORE the port's net, the two names
    /// traded — the port's old slot now carries the new name. Every pin of a net the edits named
    /// is read again by name; the rest move up by slot; a fresh build agrees.
    #[test]
    fn a_port_buffer_mid_order_with_traded_names_matches_a_fresh_build() {
        let libs = libs();
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        let mut g = Graph::build_shared(&libs, Arc::new(chain(false)), None).unwrap();
        let nl = Netlist {
            insts: vec![crate::netlist::inst("obuf", "buf"), crate::netlist::inst("u1", "and2"), crate::netlist::inst("u2", "and2")],
            ports: chain(false).ports,
            nets: vec![
                Net { name: "a".into(), pins: vec![pin(1, "A"), Conn::Port(0)] }.into(),
                Net { name: "b".into(), pins: vec![pin(1, "B"), Conn::Port(1)] }.into(),
                Net { name: "y".into(), pins: vec![pin(2, "Y"), pin(0, "A")] }.into(),
                Net { name: "n1".into(), pins: vec![pin(1, "Y"), pin(2, "A")] }.into(),
                Net { name: "c".into(), pins: vec![pin(2, "B"), Conn::Port(2)] }.into(),
                Net { name: "net9".into(), pins: vec![pin(0, "Y"), Conn::Port(3)] }.into(),
            ],
        };
        let ev = [
            "inst_create|obuf|buf",
            "net_create|net9",
            "iterm_disconnect|u2/Y|y|u2/Y,y",
            "iterm_connect|u2/Y|net9|u2/Y",
            "iterm_connect|obuf/A|net9|u2/Y,obuf/A",
            "iterm_connect|obuf/Y|y|y,obuf/Y",
        ]
        .map(String::from);
        let nl = Arc::new(nl);
        g.apply_edits(&ev, nl.clone(), None).unwrap();
        let fresh = Graph::build_shared(&libs, nl, None).unwrap();
        assert_eq!(g.differs_from_build(&fresh), None);
    }

    /// Rule (`swapNetNames` raises no callback): a name traded from a net no edit named onto a
    /// new one is not the old net — a net found by name is taken only with the same pins; else the
    /// edit is refused (or, kept in place, equals a fresh build).
    #[test]
    fn a_name_traded_from_a_net_no_edit_named_is_not_taken_by_name() {
        let libs = libs();
        let pin = |i: usize, p: &str| Conn::Inst(i, p.into());
        let mut g = Graph::build_shared(&libs, Arc::new(chain(false)), None).unwrap();
        let nl = Netlist {
            insts: vec![crate::netlist::inst("ibuf", "buf"), crate::netlist::inst("u1", "and2"), crate::netlist::inst("u2", "and2")],
            ports: chain(false).ports,
            nets: vec![
                Net { name: "a".into(), pins: vec![pin(1, "A"), Conn::Port(0)] }.into(),
                Net { name: "b".into(), pins: vec![pin(1, "B"), Conn::Port(1)] }.into(),
                Net { name: "n1".into(), pins: vec![pin(1, "Y"), pin(2, "A")] }.into(),
                Net { name: "c".into(), pins: vec![pin(0, "Y")] }.into(),
                Net { name: "net8".into(), pins: vec![pin(2, "B"), Conn::Port(2)] }.into(),
                Net { name: "y".into(), pins: vec![pin(2, "Y"), Conn::Port(3)] }.into(),
            ],
        };
        let ev = ["inst_create|ibuf|buf", "net_create|net8", "iterm_connect|ibuf/Y|net8|ibuf/Y"].map(String::from);
        let nl = Arc::new(nl);
        if g.apply_edits(&ev, nl.clone(), None).is_ok() {
            let fresh = Graph::build_shared(&libs, nl, None).unwrap();
            assert_eq!(g.differs_from_build(&fresh), None);
        }
    }

    /// Rule: a net no edit named that is not where the insertion puts it is a move, not an
    /// insertion — refused, so the caller builds again.
    #[test]
    fn an_unnamed_net_out_of_place_is_refused() {
        let libs = libs();
        let mut g = Graph::build_shared(&libs, Arc::new(chain(false)), None).unwrap();
        let mut nl = chain(false);
        nl.nets.insert(1, Net { name: "stray".into(), pins: vec![] }.into());
        assert!(matches!(g.apply_edits(&[], Arc::new(nl), None), Err(NotInPlace::Netlist(_))));
    }

    /// Rule: an edit that leaves an edge id free (a load disconnected and never reconnected) is not
    /// kept in place — the ids would no longer be the table's — and the caller builds again.
    #[test]
    fn a_free_edge_id_left_at_the_end_is_refused() {
        let libs = libs();
        let mut g = Graph::build_shared(&libs, Arc::new(chain(false)), None).unwrap();
        let ev = ["iterm_disconnect|u2/A|n1|u1/Y,u2/A".to_string()];
        let mut nl = chain(false);
        Arc::make_mut(&mut nl.nets[2]).pins.pop();
        assert_eq!(g.apply_edits(&ev, Arc::new(nl), None), Err(NotInPlace::Holes(1)));
    }

    /// Rule (a bidirect pin is two vertices of one name, its driver and its load): the comparison
    /// with a fresh build matches a vertex by name AND side — two builds of a pad compare equal.
    #[test]
    fn a_bidirect_pin_compares_by_name_and_side() {
        const PAD: &str = r#"library (p) {
          time_unit : "1ns"; capacitive_load_unit (1, pf);
          cell (pad) {
            pin (A) { direction : input; capacitance : 0.001; }
            pin (PAD) { direction : inout; capacitance : 0.001; function : "A";
              timing () { related_pin : "A"; timing_sense : positive_unate;
                cell_rise (scalar) { values ("0.1"); } rise_transition (scalar) { values ("1.0"); }
                cell_fall (scalar) { values ("0.1"); } fall_transition (scalar) { values ("1.0"); } }
            }
          }
        }"#;
        let libs = [Library::read(&crate::liberty_parse::parse(PAD).unwrap()).unwrap()];
        let nl = Netlist {
            insts: vec![crate::netlist::inst("p1", "pad")],
            // the build times a bidirect pin on no net only (the pads repair_design passes over)
            ports: vec![("a".into(), PortDir::Input)],
            nets: vec![Net { name: "a".into(), pins: vec![Conn::Inst(0, "A".into()), Conn::Port(0)] }.into()],
        };
        let (g, f) = (Graph::build(&libs, &nl).unwrap(), Graph::build(&libs, &nl).unwrap());
        assert_eq!(g.vertices.iter().filter(|v| v.name == "p1/PAD").count(), 2, "the pad pin is two vertices");
        assert_eq!(g.differs_from_build(&f), None);
    }

    /// Rule: the database appends what it makes; a netlist whose existing nets moved is refused.
    #[test]
    fn a_moved_net_is_refused() {
        let libs = libs();
        let mut g = Graph::build_shared(&libs, Arc::new(chain(false)), None).unwrap();
        let mut nl = chain(false);
        nl.nets.swap(0, 1);
        assert!(matches!(g.apply_edits(&[], Arc::new(nl), None), Err(NotInPlace::Netlist(_))));
    }

    /// Rule (`swapNetNames` after `createNet`, no callback for either name): a net whose name went
    /// to a net appended since keeps its slot and its vertices — the graph stands, as built.
    #[test]
    fn a_net_renamed_to_an_appended_one_keeps_its_slot() {
        let libs = libs();
        let mut g = Graph::build_shared(&libs, Arc::new(chain(false)), None).unwrap();
        let mut nl = chain(false);
        Arc::make_mut(&mut nl.nets[4]).name = "n2".into();
        nl.nets.push(Net { name: "y".into(), pins: vec![] }.into());
        let nl = Arc::new(nl);
        g.apply_edits(&[], nl.clone(), None).unwrap();
        let fresh = Graph::build_shared(&libs, nl, None).unwrap();
        assert_eq!(g.differs_from_build(&fresh), None);
    }
}
