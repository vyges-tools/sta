//! The scenes of one design over ONE graph structure, as the reference times them.
//!
//! 🔑 The reference has one `Graph` for every scene: its vertices, edges, edge lists, levels, loop
//! breaking and name lookups exist once, and only the slews (`ap_index * 2 + rf` per vertex) and
//! the arc delays (`arc * ap_count + ap` per edge) are kept per analysis point. Its edges are the
//! LINK cell's arc sets (`makeInstanceEdges` on the network's cell); a scene times an edge through
//! `TimingArc::sceneArc`, the set of the same key in the scene's cell (`makeSceneMap`:
//! `findTimingArcSet`, arcs paired in order when `TimingArc::equiv`).
//!
//! Here a scene is a [`Graph`] VIEW: the structure is `Arc`-shared between the views (no copy),
//! each view has its scene's libraries, an arc-set map ([`SceneSets`]) where its cells list their
//! sets in another order than the link cells, and its own per-scene arrays. A view equals a fresh
//! build of its scene wherever the two libraries list every cell's sets in the same order; where
//! they do not, the view's edges follow the LINK order, as the reference's do (the ASAP7 FF
//! corner lists the XOR/XNOR/half-adder sets in another order than the SS link corner).
//!
//! Edits are made ONCE ([`apply_edits`]), through the link view; the other views replay the
//! edit's per-scene array writes ([`ArrayOp`]).
use std::collections::HashMap;
use std::sync::Arc;

use crate::graph::{Graph, NotInPlace};
use crate::liberty::{ArcSet, Cell, Library, MAX, MIN};

/// One per-scene array write of an edit, in the order the edit made it: what each view other
/// than the editing one replays on its own slews and delays.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArrayOp {
    /// `makeVertex`: a vertex appended, its slews 0.
    Vertex,
    /// `makeEdge`: edge `.0` made (at a freed id or appended) with `.1` delay rows, all 0.
    Edge(usize, usize),
    /// `deleteEdge`: edge `.0`'s delays gone, its id free.
    DeleteEdge(usize),
}

/// `LibertyCell::findTimingArcSet(key)`: the scene cell's set with the link set's key — from,
/// to, role and `when` — whose arcs pair with the key's in order (`TimingArc::equiv`: the same
/// transitions; the sdf conditions are not read, `when` compares as text). The first such set.
pub fn find_timing_arc_set(key: &ArcSet, cell: &Cell) -> Option<usize> {
    cell.arc_sets.iter().position(|s| {
        s.from == key.from && s.to == key.to && s.role == key.role && s.cond == key.cond && s.arcs.len() == key.arcs.len() && s.arcs.iter().zip(&key.arcs).all(|(a, b)| a.from_rf == b.from_rf && a.to_rf == b.to_rf)
    })
}

/// A scene's arc-set map (`LibertyLibrary::makeSceneMap`), kept only for the cells whose sets the
/// scene lists in another order than the link: per cell name, per library index, the scene's set
/// index for each link set index.
#[derive(Debug, Default)]
pub struct SceneSets {
    map: HashMap<String, Vec<(usize, Vec<usize>)>>,
}

impl SceneSets {
    /// The scene's set for the link set `set` of cell `name` in library `lib`.
    pub(crate) fn get(&self, lib: usize, name: &str, set: usize) -> usize {
        match self.map.get(name).and_then(|l| l.iter().find(|(x, _)| *x == lib)) {
            Some((_, sets)) => sets[set],
            None => set,
        }
    }

    /// The map from `link` libraries to `scene` libraries, or why the scene cannot time through the
    /// link structure: a cell in no library of the scene, or in another library position (the
    /// graph's vertices name a cell by its position), or a link set the scene cell has none of by
    /// key (the reference then times the link arc itself, Warning 1111 — not seen on any of our
    /// kits: refused until a case shows it).
    pub fn new(link: &[Library], scene: &[Library]) -> Result<SceneSets, String> {
        let mut map: HashMap<String, Vec<(usize, Vec<usize>)>> = HashMap::new();
        for (l, lib) in link.iter().enumerate() {
            for (name, c) in &lib.cells {
                // a cell's link library is the first holding it
                if link.iter().position(|x| x.cells.contains_key(name)) != Some(l) {
                    continue;
                }
                let Some(s) = scene.iter().position(|x| x.cells.contains_key(name)) else { return Err(format!("{name}: in no library of the scene")) };
                if s != l {
                    return Err(format!("{name}: in library {s} of the scene, {l} of the link"));
                }
                let sc = &scene[s].cells[name];
                let mut sets = Vec::with_capacity(c.arc_sets.len());
                for (k, key) in c.arc_sets.iter().enumerate() {
                    match find_timing_arc_set(key, sc) {
                        Some(i) => sets.push(i),
                        None => return Err(format!("{name}: the scene has no arc set {} -> {} ({k}) by key", key.from, key.to)),
                    }
                }
                if sets.iter().enumerate().any(|(k, &i)| k != i) {
                    map.entry(name.clone()).or_default().push((l, sets));
                }
            }
        }
        Ok(SceneSets { map })
    }

    /// No cell lists its sets in another order.
    pub fn is_identity(&self) -> bool {
        self.map.is_empty()
    }
}

/// One graph per scene, every one a view of `link`'s structure (`link` is scene `link_scene`'s
/// graph, built over the link libraries): `scenes[k]` is scene `k`'s libraries. `Err` hands
/// `link` back with the reason the scenes cannot share it ([`SceneSets::new`]); the caller builds
/// each scene on its own then.
#[allow(clippy::result_large_err)]
pub fn scene_views<'a>(link: Graph<'a>, link_scene: usize, scenes: &[&'a [Library]]) -> Result<Vec<Graph<'a>>, (Graph<'a>, String)> {
    if scenes.len() <= 1 {
        return Ok(vec![link]);
    }
    let mut maps = Vec::with_capacity(scenes.len());
    for (k, libs) in scenes.iter().enumerate() {
        if k == link_scene {
            maps.push(None);
            continue;
        }
        match SceneSets::new(link.libs, libs) {
            Ok(m) => maps.push((!m.is_identity()).then(|| Arc::new(m))),
            Err(why) => return Err((link, format!("scene {k}: {why}"))),
        }
    }
    // the name index made once, shared by every view
    link.index();
    let mut views: Vec<Option<Graph<'a>>> = scenes.iter().zip(maps).enumerate().map(|(k, (libs, m))| (k != link_scene).then(|| link.scene_view(libs, m))).collect();
    views[link_scene] = Some(link);
    Ok(views.into_iter().map(|v| v.expect("every scene has a view")).collect())
}

/// The netlist edits made ONCE on scene views `gs` (from [`scene_views`]): the other views let go
/// of the shared structure, `edit` makes the edits through the link view `gs[link_scene]` (which
/// then holds it alone, so nothing is copied), and each other view takes the edited structure
/// back and replays the edit's per-scene array writes. On `Err` the views are unusable (the
/// caller builds again), as one graph is after a failed edit.
///
/// Graphs that do not share the link view's structure (each scene built on its own, when
/// [`scene_views`] refused) are each edited by `edit`: their vertices name their own libraries'
/// positions, so the link's structure is not theirs to take.
pub fn apply_edits<'a>(gs: &mut [Graph<'a>], link_scene: usize, mut edit: impl FnMut(&mut Graph<'a>) -> Result<(), NotInPlace>) -> Result<(), NotInPlace> {
    if gs.len() <= 1 {
        return edit(&mut gs[0]);
    }
    if gs.iter().any(|g| !Arc::ptr_eq(&g.vertices, &gs[link_scene].vertices)) {
        return gs.iter_mut().try_for_each(edit);
    }
    for (k, g) in gs.iter_mut().enumerate() {
        if k != link_scene {
            g.detach();
        }
    }
    let (before, rest) = gs.split_at_mut(link_scene);
    let (link, after) = rest.split_first_mut().expect("the link scene is one of the views");
    link.array_ops = Some(Vec::new());
    let done = edit(link);
    let ops = link.array_ops.take().unwrap_or_default();
    // a failed edit: the views stay let go, unless the link kept its (part-edited) structure for
    // the values the views hold to be read back by name (`Graph::hold_failed`)
    if done.is_err() && !(link.hold_failed && !link.vertices.is_empty()) {
        return done;
    }
    for g in before.iter_mut().chain(after.iter_mut()) {
        g.reattach(link);
        replay(g, &ops);
    }
    done
}

/// `ops` made on a view's own slews and delays.
fn replay(g: &mut Graph<'_>, ops: &[ArrayOp]) {
    for op in ops {
        match *op {
            ArrayOp::Vertex => g.slew.push([[0.0; 2]; 2]),
            ArrayOp::Edge(e, rows) if e < g.delay.len() => g.delay[e] = vec![[0.0; 2]; rows],
            ArrayOp::Edge(_, rows) => g.delay.push(vec![[0.0; 2]; rows]),
            ArrayOp::DeleteEdge(e) => g.delay[e].clear(),
        }
    }
}

/// Where a scene's slew annotation sets its bit (`Vertex::setSlewAnnotated`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AnnotationScope {
    /// The pin's release as shipped: ONE set of bits per vertex for every scene, the bit
    /// `ap_index` with any `ap_index > 1` folded to 0 (`ap_index = scene * 2 + min/max`). A max
    /// annotation in scene 0 sets the max bit; in any later scene, the MIN bit
    /// (parallaxsw/OpenSTA#581).
    #[default]
    PinFold,
    /// Bits per scene (our 0019 patch): each scene's annotation blocks that scene's own slew.
    PerScene,
}

/// [`AnnotationScope`]'s rule: for an annotation of scene `scene` at `mm`, the scene whose bits it
/// sets (`None`: the one set every scene reads) and the min/max bit. `scene` is the scene's
/// `define_corners` index, NOT its place relative to the link scene.
pub fn annotation_bit(scope: AnnotationScope, scene: usize, mm: usize) -> (Option<usize>, usize) {
    match scope {
        AnnotationScope::PinFold => {
            let ap = scene * 2 + mm;
            (None, if ap > 1 { MIN } else { ap })
        }
        AnnotationScope::PerScene => (Some(scene), mm),
    }
}

/// Per scene, the vertices whose slews delay calculation keeps ([`Graph::slew_blocked`]), from
/// each scene's max annotations by pin name (`annotated[k]`), under `scope`. Under
/// [`AnnotationScope::PinFold`] every scene gets the SAME set (one `Arc`). A name shared by two
/// vertices (a bidirect pin's) blocks both.
pub fn slew_blocked(g: &Graph<'_>, annotated: &[HashMap<String, f32>], scope: AnnotationScope) -> Vec<Arc<HashMap<usize, [bool; 2]>>> {
    let index = g.index();
    let unique = index.len() == g.vertices.len();
    let mut shared: HashMap<usize, [bool; 2]> = HashMap::new();
    let mut own: Vec<HashMap<usize, [bool; 2]>> = vec![HashMap::new(); annotated.len()];
    for (k, names) in annotated.iter().enumerate() {
        let (slot, bit) = annotation_bit(scope, k, MAX);
        let set = match slot {
            None => &mut shared,
            Some(s) => &mut own[s],
        };
        if unique {
            for name in names.keys() {
                if let Some(&v) = index.get(name.as_str()) {
                    set.entry(v).or_default()[bit] = true;
                }
            }
        } else {
            for v in 0..g.vertices.len() {
                if names.contains_key(&g.vname(v)) {
                    set.entry(v).or_default()[bit] = true;
                }
            }
        }
    }
    match scope {
        AnnotationScope::PinFold => {
            let one = Arc::new(shared);
            annotated.iter().map(|_| one.clone()).collect()
        }
        AnnotationScope::PerScene => own.into_iter().map(Arc::new).collect(),
    }
}

/// One mode's constraints over a scene set: ONE `Sdc` (the mode's, as the reference keeps it per
/// mode) and one SDC environment per scene (its libraries' drives, loads and units).
#[derive(Debug, Clone, PartialEq)]
pub struct SceneConstraints {
    pub sdc: crate::sdc::Sdc,
    pub envs: Vec<crate::graph::SdcEnv>,
    /// What the reading noted (each note once, in the order first seen).
    pub notes: Vec<String>,
}

/// The design a scene set's constraints are read against: a netlist, or a design with its
/// constant nets (`load::design`).
#[cfg(feature = "loom")]
#[derive(Debug, Clone, Copy)]
pub enum DesignRef<'d> {
    Netlist(&'d crate::netlist::Netlist),
    Design(&'d crate::load::Design),
}

/// A loom-parsed SDC read for every scene (`scenes[k]`: scene k's libraries) with
/// [`crate::load::constraints`] / [`crate::load::design_constraints`] as they stand: the one `Sdc`
/// and each scene's environment, or the first refusal. Refused when two scenes read the SDC into
/// different `Sdc` (one mode is one `Sdc`).
#[cfg(feature = "loom")]
pub fn scene_constraints(s: &vyges_loom::sdc::Sdc, d: DesignRef<'_>, scenes: &[&[Library]]) -> Result<SceneConstraints, String> {
    let mut sdc: Option<crate::sdc::Sdc> = None;
    let (mut envs, mut notes) = (Vec::with_capacity(scenes.len()), Vec::new());
    for (k, libs) in scenes.iter().enumerate() {
        let (sk, env, n) = match d {
            DesignRef::Netlist(nl) => crate::load::constraints(s, nl, libs),
            DesignRef::Design(x) => crate::load::design_constraints(s, x, libs),
        }
        .map_err(|e| format!("scene {k}: {e}"))?;
        match &sdc {
            None => sdc = Some(sk),
            Some(first) if *first != sk => return Err(format!("scene {k}: the SDC reads differently than in scene 0 (one mode is one Sdc)")),
            Some(_) => {}
        }
        envs.push(env);
        for note in n {
            if !notes.contains(&note) {
                notes.push(note);
            }
        }
    }
    let sdc = sdc.ok_or("no scene to read the constraints for")?;
    Ok(SceneConstraints { sdc, envs, notes })
}

/// One incremental timer over a scene set, held across reads as the reference's one `dbSta` is:
/// the scenes' graphs (views of one structure, [`scene_views`]; scenes that cannot share it are
/// built on their own), their timers (the first leads, the others follow:
/// [`crate::incr::IncTimer::follow`]) and the mode's `Sdc`. The netlist is not edited between reads.
pub struct SceneTimers<'a> {
    pub graphs: Vec<Graph<'a>>,
    pub incs: Vec<crate::incr::IncTimer>,
    pub sdc: crate::sdc::Sdc,
}

impl<'a> SceneTimers<'a> {
    /// The timers over `netlist` for every scene (`scenes[k]`: its libraries; the structure is built
    /// over the link scene's), each graph with its scene's SDC environment. `pins`: the build's
    /// master-pin filter ([`Graph::build_with_pins`]).
    pub fn for_design(scenes: &[&'a [Library]], link_scene: usize, netlist: crate::graph::NetlistRef<'a>, pins: Option<&HashMap<String, Vec<String>>>, c: SceneConstraints) -> Result<SceneTimers<'a>, String> {
        if scenes.is_empty() || scenes.len() != c.envs.len() || link_scene >= scenes.len() {
            return Err(format!("{} scenes, {} SDC environments, link scene {link_scene}", scenes.len(), c.envs.len()));
        }
        let build = |libs: &'a [Library]| match &netlist {
            crate::graph::NetlistRef::Borrowed(nl) => Graph::build_with_pins(libs, nl, pins),
            crate::graph::NetlistRef::Shared(nl) => Graph::build_shared(libs, nl.clone(), pins),
        };
        let link = build(scenes[link_scene])?;
        let mut graphs = match scene_views(link, link_scene, scenes) {
            Ok(views) => views,
            Err((link, _)) => {
                let mut link = Some(link);
                scenes.iter().enumerate().map(|(k, libs)| if k == link_scene { Ok(link.take().expect("the link scene once")) } else { build(libs) }).collect::<Result<Vec<_>, _>>()?
            }
        };
        for (g, env) in graphs.iter_mut().zip(c.envs) {
            g.sdc = env;
        }
        let mut incs: Vec<crate::incr::IncTimer> = Vec::with_capacity(graphs.len());
        for _ in 0..graphs.len() {
            let mut t = crate::incr::IncTimer::default();
            match incs.first() {
                Some(first) => t.follow(first),
                None => t.track_netlist(&graphs[0].netlist),
            }
            incs.push(t);
        }
        Ok(SceneTimers { graphs, incs, sdc: c.sdc })
    }

    /// `Sta::delaysInvalidFrom(pin)` in every scene: a pin whose net was estimated again.
    pub fn delays_invalid_from(&mut self, pin: &str) {
        for inc in &mut self.incs {
            inc.delays_invalid_from(pin);
        }
    }
}

/// `Sta::slack(net, max)` for every net of the timers' netlist (grt's `getNetSlack`): the
/// fuzzily least max slack over the net's LOAD pins in its pin order, a load's being the least
/// over its path array — every scene's paths, scene by scene, each scene's in its tag order (the
/// reference's one tag group per vertex, `Tag::matchCmp`: scene, then rise/fall, then min/max).
/// `par[k]` is scene k's parasitics. The first call times in full; a later one updates from what
/// was invalidated since ([`SceneTimers::delays_invalid_from`]), as the resizer's timer does.
pub fn net_slacks(par: &[&HashMap<String, crate::graph::NetParasitics>], t: &mut SceneTimers<'_>) -> Result<HashMap<String, f32>, String> {
    if par.len() != t.graphs.len() {
        return Err(format!("{} scenes, {} parasitic sets", t.graphs.len(), par.len()));
    }
    // `Sta::slack` → `findRequireds` → `findArrivals` → `findDelays`, each from the last read's state
    for (inc, g) in t.incs.iter().zip(t.graphs.iter_mut()) {
        inc.import(g);
    }
    crate::incr::find_delays_scenes(&mut t.incs, &mut t.graphs, par, None)?;
    for (inc, g) in t.incs.iter_mut().zip(t.graphs.iter()) {
        inc.export(g);
    }
    let mut searches: Vec<crate::search::Search<'_, '_>> = t.graphs.iter().map(|g| crate::search::Search::in_graph_order(g, &t.sdc)).collect();
    for (inc, s) in t.incs.iter().zip(searches.iter_mut()) {
        inc.import_paths(s);
    }
    crate::incr::find_arrivals_scenes(&mut t.incs, &mut searches, None)?;
    crate::incr::find_requireds_scenes(&mut t.incs, &mut searches, None)?;
    let g = &t.graphs[0];
    let index = g.pin_index();
    let mut name = String::new();
    Ok(g.netlist
        .nets
        .iter()
        .map(|n| {
            let mut slack = crate::search::INF_SLACK;
            for c in &n.pins {
                name.clear();
                let Some(&v) = index.get(g.netlist.pin_name_into(c, &mut name)) else { continue };
                if g.vertices[v].is_driver {
                    continue;
                }
                let s = scene_vertex_slack(&searches, v);
                if crate::fuzzy::less(s, slack) {
                    slack = s;
                }
            }
            (n.name.clone(), slack)
        })
        .collect())
}

/// `Sta::slack(vertex, max)` over every scene's paths at `v`, scene by scene (the path array's
/// order), fuzzily least: one scene is [`crate::search::Search::vertex_slack`].
fn scene_vertex_slack(searches: &[crate::search::Search<'_, '_>], v: usize) -> f32 {
    let mut slack = crate::search::INF_SLACK;
    for s in searches {
        for p in &s.paths[v] {
            if p.tag.mm == MAX {
                let x = p.required - p.arrival;
                if crate::fuzzy::less(x, slack) {
                    slack = x;
                }
            }
        }
    }
    slack
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::EdgeKind;
    use crate::netlist::{Conn, Net, Netlist, PortDir};

    /// `xor2` with its two sets A->Y then B->Y (`ab`), or B->Y then A->Y; the delays tell the
    /// libraries apart (`d`).
    fn lib(name: &str, ab: bool, d: f32) -> Library {
        let timing = |p: &str, x: f32| format!(r#"timing () {{ related_pin : "{p}"; timing_sense : non_unate;
            cell_rise (scalar) {{ values ("{x}"); }} rise_transition (scalar) {{ values ("0.1"); }}
            cell_fall (scalar) {{ values ("{x}"); }} fall_transition (scalar) {{ values ("0.1"); }} }}"#);
        let (first, second) = if ab { (timing("A", d), timing("B", d * 2.0)) } else { (timing("B", d * 2.0), timing("A", d)) };
        let text = format!(
            r#"library ({name}) {{
  time_unit : "1ns"; capacitive_load_unit (1,pf);
  cell (xor2) {{
    pin (A) {{ direction : input; capacitance : 0.001; }}
    pin (B) {{ direction : input; capacitance : 0.001; }}
    pin (Y) {{ direction : output; function : "A^B"; {first} {second} }}
  }}
  cell (buf) {{
    pin (A) {{ direction : input; capacitance : 0.001; }}
    pin (Y) {{ direction : output; function : "A"; {} }}
  }}
}}"#,
            timing("A", d)
        );
        Library::read(&crate::liberty_parse::parse(&text).unwrap()).unwrap()
    }

    /// Ports a, b into x1 (xor2); x1/Y to port y.
    fn netlist() -> Netlist {
        let pin = |p: &str| Conn::Inst(0, p.into());
        Netlist {
            insts: vec![crate::netlist::inst("x1", "xor2")],
            ports: vec![("a".into(), PortDir::Input), ("b".into(), PortDir::Input), ("y".into(), PortDir::Output)],
            nets: vec![
                Net { name: "a".into(), pins: vec![pin("A"), Conn::Port(0)] }.into(),
                Net { name: "b".into(), pins: vec![pin("B"), Conn::Port(1)] }.into(),
                Net { name: "y".into(), pins: vec![pin("Y"), Conn::Port(2)] }.into(),
            ],
        }
    }

    /// Each gate edge in id order: its ends and its first arc's delay as the scene times it.
    fn gate_edges(g: &Graph<'_>) -> Vec<(String, String, f32)> {
        let delay = |a: &crate::liberty::Arc| match &a.model {
            crate::liberty::Model::Gate(m) => m.gate_delay(0.0, 0.0).0,
            _ => f32::NAN,
        };
        g.edges
            .iter()
            .enumerate()
            .filter_map(|(e, ed)| match ed.kind {
                EdgeKind::Gate { set } => Some((g.vname(ed.from), g.vname(ed.to), delay(&g.arc_set(e, set).arcs[0]))),
                EdgeKind::Wire => None,
            })
            .collect()
    }

    /// A view of a scene whose cells list their sets in the link's order IS a fresh build of that
    /// scene: the same structure and delay rows, the scene's own arcs.
    #[test]
    fn a_view_in_the_link_order_equals_a_fresh_scene_build() {
        let (slow, fast) = ([lib("slow", true, 1.0)], [lib("fast", true, 0.5)]);
        let nl = Arc::new(netlist());
        let link = Graph::build_shared(&slow, nl.clone(), None).unwrap();
        let views = scene_views(link, 0, &[&slow, &fast]).map_err(|(_, w)| w).unwrap();
        let fresh = Graph::build_shared(&fast, nl, None).unwrap();
        assert!(Arc::ptr_eq(&views[0].vertices, &views[1].vertices), "the structure is shared, not copied");
        assert_eq!(views[1].differs_from_build(&fresh), None);
        assert_eq!(gate_edges(&views[1]), gate_edges(&fresh));
    }

    /// Rule (`makeInstanceEdges` on the LINK cell, `TimingArc::sceneArc` by key; witnessed on the
    /// ASAP7 corners, whose FF library lists the XOR/XNOR/half-adder sets in another order than
    /// the SS link): every scene's edges follow the link cell's set order, and a scene times each
    /// through its own cell's set of the same key. A fresh build of the scene orders them its way.
    #[test]
    fn a_scene_listing_its_sets_in_another_order_times_the_link_order_by_key() {
        let (ss, ff) = ([lib("ss", true, 1.0)], [lib("ff", false, 0.5)]);
        let nl = Arc::new(netlist());
        let link = Graph::build_shared(&ss, nl.clone(), None).unwrap();
        let views = scene_views(link, 0, &[&ss, &ff]).map_err(|(_, w)| w).unwrap();
        let order = |g: &Graph<'_>| gate_edges(g).into_iter().map(|(f, _, _)| f).collect::<Vec<_>>();
        assert_eq!(order(&views[1]), order(&views[0]), "the link order in every scene");
        let fresh = Graph::build_shared(&ff, nl, None).unwrap();
        assert_ne!(order(&fresh), order(&views[0]), "a fresh FF build orders them its own way");
        // each edge times through the scene's set of its key: half the link's delay on both
        for ((f, _, d), (_, _, link_d)) in gate_edges(&views[1]).into_iter().zip(gate_edges(&views[0])) {
            assert_eq!(d * 2.0, link_d, "{f}");
        }
    }

    /// A link set the scene cell has none of by key is refused (the link-arc fallback, Warning
    /// 1111, is not transcribed).
    #[test]
    fn a_scene_missing_a_set_by_key_is_refused() {
        let ss = [lib("ss", true, 1.0)];
        let mut other = lib("odd", true, 1.0);
        other.cells.get_mut("xor2").unwrap().arc_sets.pop();
        let other = [other];
        let link = Graph::build_shared(&ss, Arc::new(netlist()), None).unwrap();
        let why = scene_views(link, 0, &[&ss, &other]).map(|_| ()).unwrap_err().1;
        assert!(why.contains("xor2") && why.contains("by key"), "{why}");
    }

    /// An edit made once through the link view reaches every view: each equals a fresh build of
    /// the edited netlist, its new edges' delays 0, the structure still shared.
    #[test]
    fn an_edit_made_once_reaches_every_view() {
        let (slow, fast) = ([lib("slow", true, 1.0)], [lib("fast", true, 0.5)]);
        let link = Graph::build_shared(&slow, Arc::new(netlist()), None).unwrap();
        let mut views = scene_views(link, 1, &[&fast, &slow]).map_err(|(_, w)| w).unwrap();
        for g in views.iter_mut() {
            for row in g.delay.iter_mut() {
                row.iter_mut().for_each(|d| *d = [7.0; 2]);
            }
        }
        // a buffer on y: x1/Y -> b1/A, b1/Y -> port y
        let mut next = netlist();
        next.insts.push(crate::netlist::inst("b1", "buf"));
        Arc::make_mut(&mut next.nets[2]).pins = vec![Conn::Inst(0, "Y".into()), Conn::Inst(1, "A".into())];
        next.nets.push(Net { name: "y2".into(), pins: vec![Conn::Inst(1, "Y".into()), Conn::Port(2)] }.into());
        let events: Vec<String> = ["inst_create|b1|buf", "net_create|y2", "iterm_connect|b1/A|y|x1/Y,y,b1/A", "iterm_connect|b1/Y|y2|b1/Y", "bterm_disconnect|y|y|x1/Y,y,b1/A", "bterm_connect|y|y2|b1/Y,y"].map(String::from).to_vec();
        let next = Arc::new(next);
        apply_edits(&mut views, 1, |g| g.apply_edits(&events, next.clone(), None)).unwrap();
        assert!(Arc::ptr_eq(&views[0].vertices, &views[1].vertices) && Arc::ptr_eq(&views[0].edges, &views[1].edges));
        for (k, libs) in [(0, &fast), (1, &slow)] {
            let fresh = Graph::build_shared(libs, next.clone(), None).unwrap();
            assert_eq!(views[k].differs_from_build(&fresh), None, "scene {k}");
            assert_eq!(views[k].slew.len(), views[k].vertices.len());
            assert_eq!(views[k].delay.iter().map(Vec::len).collect::<Vec<_>>(), views[1].delay.iter().map(Vec::len).collect::<Vec<_>>());
            let made = (0..views[k].edges.len()).filter(|&e| views[k].vname(views[k].edges[e].to).starts_with("b1/") || views[k].vname(views[k].edges[e].from).starts_with("b1/"));
            for e in made {
                assert!(views[k].delay[e].iter().all(|d| *d == [0.0; 2]), "scene {k} edge {e}: made, so 0");
            }
        }
    }

    /// Scenes built on their own (a library at another position: [`scene_views`] refuses) are
    /// each edited on their own; none takes the link's structure, whose vertices name the link's
    /// library positions. Witnessed on CoralNPU's ASAP7 scene 2 (a cell one library later than
    /// in the link): taking the link structure there read a cell from the wrong library.
    #[test]
    fn scenes_built_on_their_own_are_each_edited_on_their_own() {
        let mut spare = lib("spare", true, 3.0);
        spare.cells.remove("xor2");
        let slow = [lib("slow", true, 1.0)];
        let fast = [spare, lib("fast", true, 0.5)];
        let nl = Arc::new(netlist());
        let link = Graph::build_shared(&slow, nl.clone(), None).unwrap();
        let mut gs = match scene_views(link, 0, &[&slow, &fast]) {
            Ok(_) => panic!("a library at another position is refused"),
            Err((link, _)) => vec![link, Graph::build_shared(&fast, nl, None).unwrap()],
        };
        let mut next = netlist();
        next.insts.push(crate::netlist::inst("b1", "buf"));
        Arc::make_mut(&mut next.nets[2]).pins = vec![Conn::Inst(0, "Y".into()), Conn::Inst(1, "A".into())];
        next.nets.push(Net { name: "y2".into(), pins: vec![Conn::Inst(1, "Y".into()), Conn::Port(2)] }.into());
        let events: Vec<String> = ["inst_create|b1|buf", "net_create|y2", "iterm_connect|b1/A|y|x1/Y,y,b1/A", "iterm_connect|b1/Y|y2|b1/Y", "bterm_disconnect|y|y|x1/Y,y,b1/A", "bterm_connect|y|y2|b1/Y,y"].map(String::from).to_vec();
        let next = Arc::new(next);
        apply_edits(&mut gs, 0, |g| g.apply_edits(&events, next.clone(), None)).unwrap();
        assert!(!Arc::ptr_eq(&gs[0].vertices, &gs[1].vertices), "each keeps its own structure");
        for (k, libs) in [(0, &slow[..]), (1, &fast[..])] {
            let fresh = Graph::build_shared(libs, next.clone(), None).unwrap();
            assert_eq!(gs[k].differs_from_build(&fresh), None, "scene {k}");
        }
        assert_eq!(gate_edges(&gs[1]).iter().map(|e| e.2).collect::<Vec<_>>(), gate_edges(&Graph::build_shared(&fast, next, None).unwrap()).iter().map(|e| e.2).collect::<Vec<_>>());
    }

    /// A failed edit through the link view, the views holding a timer's values (`hold_failed`):
    /// every view takes the part-edited structure back, its arrays the length of it, so each
    /// one's values can still be read by name; without `hold_failed` they stay let go.
    #[test]
    fn a_failed_edit_leaves_holding_views_readable() {
        let (slow, fast) = ([lib("slow", true, 1.0)], [lib("fast", true, 0.5)]);
        let mut next = netlist();
        Arc::make_mut(&mut next.nets[2]).pins = vec![Conn::Inst(0, "Y".into())];
        let next = Arc::new(next);
        let ev = ["bterm_disconnect|y|y|x1/Y,y".to_string()];
        for hold in [true, false] {
            let link = Graph::build_shared(&slow, Arc::new(netlist()), None).unwrap();
            let mut views = scene_views(link, 0, &[&slow, &fast]).map_err(|(_, w)| w).unwrap();
            views.iter_mut().for_each(|g| g.hold_failed = hold);
            assert!(apply_edits(&mut views, 0, |g| g.apply_edits(&ev, next.clone(), None)).is_err());
            if hold {
                assert!(!views[1].vertices.is_empty() && Arc::ptr_eq(&views[0].vertices, &views[1].vertices));
                assert_eq!((views[1].slew.len(), views[1].delay.len()), (views[1].vertices.len(), views[1].edges.len()));
                assert_eq!(views[1].vname(0), views[0].vname(0));
            } else {
                assert!(views[1].vertices.is_empty(), "let go");
            }
        }
    }

    /// Rule (#581, the pin's release): ONE set of annotation bits per vertex; a max annotation in
    /// scene 0 sets the max bit, in any later scene the MIN bit — by the scene's `define_corners`
    /// index, not by where the link scene is (here the link is scene 1). Under the 0019 scope each
    /// scene blocks its own max.
    #[test]
    fn annotation_bits_fold_by_scene_index_not_by_the_link_scene() {
        let (slow, fast) = ([lib("slow", true, 1.0)], [lib("fast", true, 0.5)]);
        let link = Graph::build_shared(&slow, Arc::new(netlist()), None).unwrap();
        let views = scene_views(link, 1, &[&fast, &slow]).map_err(|(_, w)| w).unwrap();
        assert_eq!(annotation_bit(AnnotationScope::PinFold, 0, MAX), (None, MAX));
        assert_eq!(annotation_bit(AnnotationScope::PinFold, 1, MAX), (None, MIN));
        assert_eq!(annotation_bit(AnnotationScope::PinFold, 2, MAX), (None, MIN));
        assert_eq!(annotation_bit(AnnotationScope::PerScene, 2, MAX), (Some(2), MAX));
        let mut annotated = vec![HashMap::new(), HashMap::new()];
        annotated[0].insert("x1/A".to_string(), 0.2f32);
        annotated[1].insert("x1/B".to_string(), 0.2f32);
        let (a, b) = (views[0].index()["x1/A"], views[0].index()["x1/B"]);
        let fold = slew_blocked(&views[1], &annotated, AnnotationScope::PinFold);
        assert!(Arc::ptr_eq(&fold[0], &fold[1]), "one set for every scene");
        assert_eq!((fold[0].get(&a), fold[0].get(&b)), (Some(&[false, true]), Some(&[true, false])));
        let own = slew_blocked(&views[1], &annotated, AnnotationScope::PerScene);
        assert_eq!((own[0].get(&a), own[0].get(&b), own[1].get(&b)), (Some(&[false, true]), None, Some(&[false, true])));
    }

    /// Ports a, b into x1; x1/Y to port y; a clock port clk on no net.
    fn clocked() -> Netlist {
        let mut nl = netlist();
        nl.ports.push(("clk".into(), PortDir::Input));
        nl
    }

    /// A clock of 2 ns on clk, 0.1 ns input and output delays on a, b and y.
    fn sdc() -> crate::sdc::Sdc {
        use crate::sdc::{Clock, PortDelay, Sdc};
        Sdc { clocks: vec![Clock::new("clk", 2e-9, "clk", false)], input_delays: vec![PortDelay::uniform("a", 1e-10), PortDelay::uniform("b", 1e-10)], output_delays: vec![PortDelay::uniform("y", 1e-10)], path_delays: Vec::new() }
    }

    /// Net y's wire as an RC network of `cap` at each end.
    fn wire(cap: f32) -> HashMap<String, crate::graph::NetParasitics> {
        let net = crate::graph::NetParasitics { node_names: vec!["x1/Y".into(), "y".into()], network: crate::parasitics::Network { node_caps: vec![cap, cap], resistors: vec![(0, 1, 100.0)] }, ..Default::default() };
        [("y".to_string(), net)].into()
    }

    fn timers<'a>(scenes: &[&'a [Library]], nl: &'a Netlist) -> SceneTimers<'a> {
        let c = SceneConstraints { sdc: sdc(), envs: vec![crate::graph::SdcEnv::default(); scenes.len()], notes: Vec::new() };
        SceneTimers::for_design(scenes, 0, crate::graph::NetlistRef::Borrowed(nl), None, c).unwrap()
    }

    /// The one-scene read as the router's timer makes it today (a graph built per read, one
    /// incremental timer imported, timed, exported): the reference for one scene.
    fn router_read(libs: &[Library], nl: &Netlist, par: &HashMap<String, crate::graph::NetParasitics>, inc: &mut crate::incr::IncTimer) -> HashMap<String, f32> {
        let sdc = sdc();
        let mut g = Graph::build(libs, nl).unwrap();
        inc.import(&mut g);
        inc.find_delays(&mut g, par, None).unwrap();
        inc.export(&g);
        let mut s = crate::search::Search::in_graph_order(&g, &sdc);
        inc.import_paths(&mut s);
        inc.find_arrivals(&mut s, None).unwrap();
        inc.find_requireds(&mut s, None).unwrap();
        let index: HashMap<String, usize> = (0..g.vertices.len()).map(|v| (g.vname(v), v)).collect();
        nl.nets.iter().map(|n| (n.name.clone(), s.net_slack(&n.pins.iter().filter_map(|c| index.get(&nl.pin_name(c)).copied()).filter(|&v| !g.vertices[v].is_driver).collect::<Vec<_>>()))).collect()
    }

    /// One scene: every read, first and incremental, is the router's own timer's bit for bit.
    #[test]
    fn one_scene_net_slacks_are_the_router_timers() {
        let slow = [lib("slow", true, 1.0)];
        let nl = clocked();
        let mut t = timers(&[&slow], &nl);
        let mut inc = crate::incr::IncTimer::default();
        let first = net_slacks(&[&HashMap::new()], &mut t).unwrap();
        assert_eq!(first, router_read(&slow, &nl, &HashMap::new(), &mut inc));
        assert!(first["y"] < 1.0, "net y is timed: {first:?}");
        let par = wire(0.5e-12);
        for pin in ["x1/Y", "y"] {
            t.delays_invalid_from(pin);
            inc.delays_invalid_from(pin);
        }
        let next = net_slacks(&[&par], &mut t).unwrap();
        let bits = |m: &HashMap<String, f32>| m.iter().map(|(k, v)| (k.clone(), v.to_bits())).collect::<std::collections::BTreeMap<_, _>>();
        assert_eq!(bits(&next), bits(&router_read(&slow, &nl, &par, &mut inc)));
        assert_ne!(next["y"], first["y"], "the wire moved net y's slack");
    }

    /// Rule (`Sta::slack(net)`: the least over the loads, each the least over its ONE path array
    /// holding every scene's paths): three scenes read as the least of three one-scene timers.
    #[test]
    fn three_scenes_read_the_least_of_their_one_scene_slacks() {
        let libs = [[lib("ss", true, 1.5)], [lib("tt", true, 1.0)], [lib("ff", false, 0.5)]];
        let nl = clocked();
        let scenes: Vec<&[Library]> = libs.iter().map(|l| &l[..]).collect();
        let mut t = timers(&scenes, &nl);
        assert!(t.incs[1..].iter().all(|i| i.levels.is_empty()), "the scenes' timers follow the first");
        let all = net_slacks(&[&HashMap::new(); 3], &mut t).unwrap();
        let each: Vec<HashMap<String, f32>> = libs.iter().map(|l| net_slacks(&[&HashMap::new()], &mut timers(&[&l[..]], &nl)).unwrap()).collect();
        for (net, s) in &all {
            let least = each.iter().map(|m| m[net]).fold(crate::search::INF_SLACK, f32::min);
            assert_eq!(*s, least, "{net}");
        }
        assert_eq!(all["y"], each[0]["y"], "the slow scene is the worst");
        assert_ne!(each[0]["y"], each[2]["y"]);
    }

    /// An incremental read after the caller invalidated a re-estimated net's pins equals a full
    /// timing of the same parasitics.
    #[test]
    fn an_incremental_read_equals_a_full_one() {
        let libs = [[lib("ss", true, 1.5)], [lib("ff", false, 0.5)]];
        let nl = clocked();
        let scenes: Vec<&[Library]> = libs.iter().map(|l| &l[..]).collect();
        let mut t = timers(&scenes, &nl);
        net_slacks(&[&HashMap::new(); 2], &mut t).unwrap();
        let par = wire(0.5e-12);
        for pin in ["x1/Y", "y"] {
            t.delays_invalid_from(pin);
        }
        let inc = net_slacks(&[&par, &par], &mut t).unwrap();
        let full = net_slacks(&[&par, &par], &mut timers(&scenes, &nl)).unwrap();
        assert_eq!(inc, full);
        let stale = {
            let mut u = timers(&scenes, &nl);
            net_slacks(&[&HashMap::new(); 2], &mut u).unwrap();
            net_slacks(&[&par, &par], &mut u).unwrap()
        };
        assert_ne!(stale["y"], full["y"], "without the invalidation the read is stale (the test can fail)");
    }

    /// Every scene reads the SDC as `load::constraints` reads it for that scene alone; scenes that
    /// read it into different `Sdc` (here another time unit) are refused, not merged.
    #[cfg(feature = "loom")]
    #[test]
    fn scene_constraints_read_the_sdc_per_scene_as_one_mode() {
        let (slow, fast) = ([lib("slow", true, 1.0)], [lib("fast", true, 0.5)]);
        let nl = clocked();
        let text = "create_clock -name clk -period 2 [get_ports clk]\nset_input_delay 0.1 -clock clk [get_ports {a b}]\nset_output_delay 0.1 -clock clk [get_ports y]\n";
        let s = vyges_loom::sdc::Sdc::parse_with(text, &crate::load::sdc_design(&nl, &slow)).expect("the constraints read");
        let c = scene_constraints(&s, DesignRef::Netlist(&nl), &[&slow, &fast]).unwrap();
        let (one, env, _) = crate::load::constraints(&s, &nl, &fast).unwrap();
        assert_eq!((&c.sdc, c.envs.len(), &c.envs[1]), (&one, 2, &env));
        assert_eq!(c.sdc.clocks.len(), 1);
        let mut ps = lib("ps", true, 1.0);
        ps.time_scale = 1e-12;
        let ps = [ps];
        let why = scene_constraints(&s, DesignRef::Netlist(&nl), &[&slow, &ps]).unwrap_err();
        assert!(why.contains("scene 1") && why.contains("one mode"), "{why}");
    }
}
