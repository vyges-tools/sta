// SPDX-License-Identifier: Apache-2.0
//! The constraints the timer reads: one clock on a port, input and output delays relative to it.
//! Values are in seconds, stored as `float`.

/// `create_clock -period P [-waveform {r f}] <port>`, optionally propagated.
#[derive(Debug, Clone, PartialEq)]
pub struct Clock {
    pub name: String,
    pub period: f32,
    /// The rise and fall edge times (`waveform`; default `{0, period / 2}`).
    pub waveform: [f32; 2],
    /// The port the clock is defined on.
    pub source: String,
    pub propagated: bool,
    /// `set_clock_latency` on pins (network latency), each `delay[rf][min/max]` where set:
    /// `specific` for one given with this clock (`-clock`), else one on the pin for every clock.
    pub pin_latencies: Vec<PinLatency>,
    /// `set_clock_uncertainty` on the clock (`Clock::uncertainties`), `[min, max]` where set: the
    /// hold (min) and setup (max) uncertainty of the checks this clock captures.
    pub uncertainty: [Option<f32>; 2],
    /// `set_clock_uncertainty -from/-to` with this clock as the TARGET (`Sdc::clockUncertainty`).
    pub inter_uncertainties: Vec<InterClockUncertainty>,
    /// The clocks (indices in [`Sdc::clocks`]) `set_clock_groups` cuts this one from
    /// (`Sdc::clk_group_exclusions_`, a symmetric pair set): no path between the two is checked.
    pub excluded: Vec<usize>,
}

/// One `InterClockUncertainty`, kept on its target clock: from the clock `from` (its index in
/// [`Sdc::clocks`]), `value[src_rf][tgt_rf][min/max]` where set.
#[derive(Debug, Clone, PartialEq)]
pub struct InterClockUncertainty {
    pub from: usize,
    pub value: [[[Option<f32>; 2]; 2]; 2],
}

/// One `ClockLatency` on a pin.
#[derive(Debug, Clone, PartialEq)]
pub struct PinLatency {
    pub pin: String,
    pub specific: bool,
    pub delay: [[Option<f32>; 2]; 2],
}

impl Clock {
    /// A clock with the default waveform `{0, period / 2}`.
    pub fn new(name: &str, period: f32, source: &str, propagated: bool) -> Clock {
        Clock {
            name: name.into(),
            period,
            waveform: [0.0, period / 2.0],
            source: source.into(),
            propagated,
            pin_latencies: Vec::new(),
            uncertainty: [None; 2],
            inter_uncertainties: Vec::new(),
            excluded: Vec::new(),
        }
    }

    /// `Sdc::clockLatency(clk, pin, rf, min_max)`: the latency given with this clock on the pin,
    /// else (that one unset for this transition and min/max) the pin's for every clock.
    pub fn pin_latency(&self, pin: &str, rf: usize, mm: usize) -> Option<f32> {
        let find = |specific: bool| self.pin_latencies.iter().find(|l| l.specific == specific && l.pin == pin).and_then(|l| l.delay[rf][mm]);
        find(true).or_else(|| find(false))
    }

    /// The time of the clock edge of transition `rf`.
    pub fn edge_time(&self, rf: usize) -> f32 {
        self.waveform[rf]
    }
}

/// `set_input_delay` / `set_output_delay` on a port, relative to the clock's RISE edge:
/// `delay[rf][min/max]`.
#[derive(Debug, Clone, PartialEq)]
pub struct PortDelay {
    pub port: String,
    pub delay: [[f32; 2]; 2],
    /// Which `[rf][min/max]` values the constraint sets (`RiseFallMinMax::exists`): a missing
    /// one makes no path end for that transition and min/max.
    pub exists: [[bool; 2]; 2],
    /// The index of the clock it is relative to (its RISE edge) in [`Sdc::clocks`].
    pub clock: usize,
}

impl PortDelay {
    /// The same value for every transition and min/max (`set_*_delay <value>`), relative to the
    /// first clock.
    pub fn uniform(port: &str, value: f32) -> PortDelay {
        PortDelay { port: port.into(), delay: [[value; 2]; 2], exists: [[true; 2]; 2], clock: 0 }
    }
}

/// `set_max_delay` / `set_min_delay` (`PathDelay`), `-from` pins and `-to` pins or the clock, every
/// transition, no `-through`. Its id (the order it was made in, `ExceptionPath::id`) is its index in
/// [`Sdc::path_delays`].
#[derive(Debug, Clone, PartialEq)]
pub struct PathDelay {
    /// The pins of `-from`; empty with no `-from` pin.
    pub from_pins: Vec<String>,
    /// `-from` names the clock.
    pub from_clock: bool,
    /// The pins of `-to`; empty with no `-to` pin.
    pub to_pins: Vec<String>,
    /// `-to` names the clock.
    pub to_clock: bool,
    /// `MIN` for `set_min_delay`, `MAX` for `set_max_delay`: the only min/max it matches.
    pub min_max: usize,
    pub ignore_clk_latency: bool,
    /// Not `-probe`: an internal `-from` / `-to` pin breaks the search there.
    pub break_path: bool,
    pub delay: f32,
    /// `set_false_path` (a `FalsePath`), not a delay: kept with the path delays because the
    /// reference ranks every exception by one priority (`ExceptionPath::typePriority`). `delay`,
    /// `ignore_clk_latency` and `break_path` mean nothing for it.
    pub false_path: bool,
    /// `set_multicycle_path` (a `MultiCyclePath`), not a delay, ranked with the rest as false
    /// paths are. `min_max` is `MAX` for `-setup`, `MIN` for `-hold`; [`Multicycle::all`] for
    /// neither.
    pub multicycle: Option<Multicycle>,
}

/// A multicycle path's own fields (`MultiCyclePath`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Multicycle {
    pub multiplier: i32,
    /// `-end` (the default but for `-hold` alone): periods of the capture clock, else the launch's.
    pub use_end_clk: bool,
    /// Neither `-setup` nor `-hold` (`MinMaxAll::all()`).
    pub all: bool,
}

impl PathDelay {
    fn has_from(&self) -> bool {
        !self.from_pins.is_empty() || self.from_clock
    }

    /// `ExceptionTo::matches(pin, clk_edge, end_rf)`: a `-to` pin, or `-to` the clock at a clocked
    /// end — or no `-to` at all.
    fn to_matches(&self, pin: &str, clocked: bool) -> bool {
        self.to_pins.iter().any(|p| p == pin) || (clocked && self.to_clock) || (self.to_pins.is_empty() && !self.to_clock)
    }

    /// `typePriority() + fromThruToPriority()`: the type's base (false paths 4000, path delays
    /// 3000) PLUS
    /// `-from` pins 1<<6, `-to` pins 1<<5, `-from` clocks 1<<3, `-to` clocks 1<<2 — added, not OR'd
    /// (3000 has bits 3 and 5 set).
    /// `ExceptionPath::matches(min_max, exactly)`: its own min/max — but a multicycle path matches
    /// every min path too when not asked exactly (`set_multicycle_path -setup` decides the hold
    /// check's accounting, so it rides on min tags).
    fn matches(&self, mm: usize, exactly: bool) -> bool {
        match self.multicycle {
            Some(m) => m.all || self.min_max == mm || (!exactly && mm == crate::liberty::MIN),
            None => self.min_max == mm,
        }
    }

    /// `MultiCyclePath::pathMultiplier(min_max)`: 0 for a min path under a multicycle of neither
    /// `-setup` nor `-hold`.
    fn path_multiplier(&self, mm: usize) -> i32 {
        match self.multicycle {
            Some(m) if m.all && mm == crate::liberty::MIN => 0,
            Some(m) => m.multiplier,
            None => 0,
        }
    }

    /// `ExceptionPath::priority(min_max)`: [`PathDelay::priority`], and for a multicycle path +1
    /// when it is of neither `-setup` nor `-hold`, +2 when of this min/max
    /// (`MultiCyclePath::priority`).
    fn priority_for(&self, mm: usize) -> i32 {
        match self.multicycle {
            Some(m) if m.all => self.priority() + 1,
            Some(_) if self.min_max == mm => self.priority() + 2,
            _ => self.priority(),
        }
    }

    fn priority(&self) -> i32 {
        let mut p = 0;
        if !self.from_pins.is_empty() {
            p |= 1 << 6;
        }
        if !self.to_pins.is_empty() {
            p |= 1 << 5;
        }
        if self.from_clock {
            p |= 1 << 3;
        }
        if self.to_clock {
            p |= 1 << 2;
        }
        let base = if self.false_path {
            4000
        } else if self.multicycle.is_some() {
            2000
        } else {
            3000
        };
        base + p
    }

    /// `ExceptionState::isComplete` at the start: a false path with no `-to` (and no `-through`,
    /// none here) is complete out of the gate — the path it starts is never made.
    fn complete_false(&self) -> bool {
        self.false_path && self.to_pins.is_empty() && !self.to_clock
    }

    /// `PathDelay::tighterThan`: a min delay is tighter when larger, a max delay when smaller.
    /// A false path is never tighter (`FalsePath::tighterThan`): of two at one priority, the
    /// first found stays.
    fn tighter_than(&self, other: &PathDelay) -> bool {
        if self.false_path || other.false_path {
            return false;
        }
        // `MultiCyclePath::tighterThan`: the smaller multiplier (`pathMultiplier()`, the raw one).
        if let Some(m) = self.multicycle {
            return m.multiplier < other.multicycle.map_or(0, |o| o.multiplier);
        }
        if self.min_max == crate::liberty::MIN {
            self.delay > other.delay
        } else {
            self.delay < other.delay
        }
    }
}

/// The exception states a tag carries (`ExceptionStateSet`): one bit per path delay id. With no
/// `-through`, an exception has one state, and it is complete from the start.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct States(pub u64);

impl States {
    pub const NONE: States = States(0);

    pub fn contains(self, id: usize) -> bool {
        (self.0 >> id) & 1 == 1
    }

    pub fn ids(self) -> impl Iterator<Item = usize> {
        (0..64).filter(move |&i| self.contains(i))
    }

    /// `Tag::stateCmp` as a sort key: no states first, then fewer states, then the state lists in
    /// id order compared element by element — the set holding the least id where the two differ is
    /// the lesser, i.e. the larger bit-reversed mask.
    pub fn key(self) -> (u32, u64) {
        (self.0.count_ones(), !self.0.reverse_bits())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Sdc {
    /// The clocks, in the order they were made (`Clock::index`): a clock edge's index is the
    /// clock's times 2 plus its transition (`ClockEdge::index`). None: every path is unclocked.
    pub clocks: Vec<Clock>,
    pub input_delays: Vec<PortDelay>,
    pub output_delays: Vec<PortDelay>,
    /// In the order they were made (their ids). At most 64.
    pub path_delays: Vec<PathDelay>,
}

impl Sdc {
    /// `PathEnd::checkClkUncertainty` for a check of min/max `mm` capturing at the clock edge
    /// `tgt_edge`: the inter-clock uncertainty from the source edge's clock (`checkInterClk-
    /// Uncertainty`; none for an unclocked path), else the target clock's own
    /// (`checkTgtClkUncertainty`), else 0 — negated for a setup (max) check, which it tightens.
    pub fn check_clk_uncertainty(&self, src_edge: Option<usize>, tgt_edge: usize, mm: usize) -> f32 {
        let tgt = &self.clocks[tgt_edge / 2];
        let inter = src_edge.and_then(|s| {
            tgt.inter_uncertainties.iter().find(|u| u.from == s / 2).and_then(|u| u.value[s % 2][tgt_edge % 2][mm])
        });
        let u = inter.or(tgt.uncertainty[mm]).unwrap_or(0.0);
        if mm == crate::liberty::MAX {
            -u
        } else {
            u
        }
    }

    /// `Sdc::sameClockGroup(src, tgt)`: no `set_clock_groups` cuts the two clocks apart. Asked
    /// at every clocked check, output delay and gated clock end (`VisitPathEnds`), path delays
    /// included; an end it fails is not made.
    pub fn same_clock_group(&self, src: usize, tgt: usize) -> bool {
        !self.clocks[src].excluded.contains(&tgt)
    }

    /// `Sdc::exceptionFromStates` returning false: a false path of this min/max complete at the
    /// start (no `-to`) whose `-from` names `pin` — or, for a path the clock launches (`clk`),
    /// the clock. No path starts there (`inputDelayTag` / `fromRegClkTag` make no tag). The
    /// clock's own network paths are seeded regardless (`exceptionFromClkStates`).
    pub fn false_from(&self, pin: &str, clk: bool, mm: usize) -> bool {
        self.path_delays.iter().any(|pd| {
            pd.min_max == mm && pd.complete_false() && (pd.from_pins.iter().any(|p| p == pin) || (clk && pd.from_clock && pd.from_pins.is_empty()))
        })
    }

    /// `VisitPathEnds::falsePathTo` → `Search::exceptionTo(false_path, path, pin, rf, clk_edge =
    /// null, min_max)`: of the FALSE PATHS alone, one of the path's states complete at `pin`, or
    /// one whose first point is a `-to` naming the pin. Read before any end is visited, so a match
    /// makes no end at the pin at all — check, output delay, path delay or gated clock. No clock
    /// edge is passed, so `-to` the clock never matches here.
    pub fn false_path_to(&self, states: States, pin: &str, mm: usize) -> bool {
        let to = |pd: &PathDelay| pd.false_path && pd.min_max == mm && pd.to_matches(pin, false);
        states.ids().any(|id| to(&self.path_delays[id]))
            || self.path_delays.iter().any(|pd| !pd.has_from() && !pd.to_pins.is_empty() && to(pd))
    }

    /// `Sdc::exceptionFromStates`: the states a path starting at `pin` (and, launched by the clock,
    /// at `clk`) carries — every exception of this min/max whose `-from` names the pin, then every
    /// one whose `-from` names the clock. A false path complete at the start is not a state: it
    /// stops the path ([`Sdc::false_from`], which callers ask first).
    pub fn exception_from_states(&self, pin: &str, clk: bool, mm: usize) -> States {
        let mut s = 0u64;
        for (id, pd) in self.path_delays.iter().enumerate() {
            if pd.matches(mm, false) && !pd.complete_false() && (pd.from_pins.iter().any(|p| p == pin) || (clk && pd.from_clock && pd.from_pins.is_empty())) {
                s |= 1 << id;
            }
        }
        States(s)
    }

    /// `Sdc::isCompleteTo(state, pin, rf, min_max)` (the mutation's test): the exception has a
    /// `-to` naming `pin` (clocks do not count) and is of this min/max.
    pub fn is_complete_to_pin(&self, id: usize, pin: &str, mm: usize) -> bool {
        let pd = &self.path_delays[id];
        !pd.false_path && pd.multicycle.is_none() && pd.min_max == mm && pd.to_pins.iter().any(|p| p == pin)
    }

    /// `Search::exceptionTo(any, path, pin, rf, clk_edge, min_max)` over path delays: the highest
    /// priority — ties to the tighter — of the path's states complete at `pin` (a state with no
    /// `-to` completes anywhere), then of the exceptions whose FIRST point is a `-to` matching the
    /// pin or the target clock. `clocked`: the end has a target clock edge.
    ///
    /// False paths are not candidates: a false path at the pin has already stopped the end
    /// ([`Sdc::false_path_to`]), and one that does not match here could not win.
    pub fn path_delay_to(&self, states: States, pin: &str, clocked: bool, mm: usize) -> Option<usize> {
        self.exception_to(states, pin, clocked, mm, false, &|pd| !pd.false_path && pd.multicycle.is_none())
    }

    /// `Search::exceptionTo(multi_cycle, …, match_min_max_exactly)`: the multicycle path ruling a
    /// check of min/max `mm` at `pin` — at a check, below any path delay (2000 against 3000).
    pub fn multicycle_to(&self, states: States, pin: &str, clocked: bool, mm: usize, exactly: bool) -> Option<usize> {
        self.exception_to(states, pin, clocked, mm, exactly, &|pd| pd.multicycle.is_some())
    }

    /// `Search::exceptionTo` over the exceptions `kind` admits.
    fn exception_to(&self, states: States, pin: &str, clocked: bool, mm: usize, exactly: bool, kind: &dyn Fn(&PathDelay) -> bool) -> Option<usize> {
        let mut best: Option<usize> = None;
        let consider = |id: usize, best: &mut Option<usize>| {
            let pd = &self.path_delays[id];
            if !kind(pd) || !pd.matches(mm, exactly) || !pd.to_matches(pin, clocked) {
                return;
            }
            let better = match *best {
                None => true,
                Some(b) => {
                    let bp = &self.path_delays[b];
                    let (p, q) = (pd.priority_for(mm), bp.priority_for(mm));
                    p > q || (p == q && pd.tighter_than(bp))
                }
            };
            if better {
                *best = Some(id);
            }
        };
        for id in states.ids() {
            consider(id, &mut best);
        }
        // `Sdc::exceptionTo`: `first_to_pin_exceptions_`, then `first_to_clk_exceptions_` (clocked
        // ends only).
        for (id, pd) in self.path_delays.iter().enumerate() {
            if !pd.has_from() && !pd.to_pins.is_empty() && pd.to_pins.iter().any(|p| p == pin) {
                consider(id, &mut best);
            }
        }
        for (id, pd) in self.path_delays.iter().enumerate() {
            if clocked && !pd.has_from() && pd.to_pins.is_empty() && pd.to_clock {
                consider(id, &mut best);
            }
        }
        best
    }
}

impl Sdc {
    /// `PathEndClkConstrainedMcp::checkMcpAdjustment` for a check of min/max `mm` at `pin` whose
    /// ruling multicycle is `id` (none: 0), the path launched at `src_edge` (unclocked: the target
    /// edge stands in) and captured at `tgt_edge`. Setup: (multiplier − 1) periods. Hold: the
    /// setup multicycle's (multiplier − 1) periods less the hold one's multiplier periods — the
    /// other of the two found exactly (`findHoldMcps`), each period the capture clock's under
    /// `-end`, the launch's under `-start`. Added to the target clock arrival after the
    /// uncertainty (`targetClkArrivalNoCrpr`).
    pub fn multicycle_adjustment(&self, id: Option<usize>, states: States, pin: &str, src_edge: Option<usize>, tgt_edge: usize, mm: usize) -> f32 {
        use crate::liberty::{MAX, MIN};
        let Some(id) = id else { return 0.0 };
        let src_edge = src_edge.unwrap_or(tgt_edge);
        let period = |pd: &PathDelay| {
            let m = pd.multicycle.expect("a multicycle path");
            self.clocks[(if m.use_end_clk { tgt_edge } else { src_edge }) / 2].period
        };
        let pd = &self.path_delays[id];
        if mm == MAX {
            // `checkSetupMcpAdjustment`: `mcp->minMax()->matches(max)`, default cycles 1
            return if pd.matches(MAX, true) { (pd.path_multiplier(MAX) - 1) as f32 * period(pd) } else { 0.0 };
        }
        let (setup, hold) = if pd.matches(MIN, true) {
            (self.multicycle_to(states, pin, true, MAX, true), Some(id))
        } else {
            (Some(id), self.multicycle_to(states, pin, true, MIN, true))
        };
        match (setup.map(|i| &self.path_delays[i]), hold.map(|i| &self.path_delays[i])) {
            (Some(s), Some(h)) => (s.path_multiplier(MAX) - 1) as f32 * period(s) - h.path_multiplier(MIN) as f32 * period(h),
            (None, Some(h)) => -h.path_multiplier(mm) as f32 * period(h),
            (Some(s), None) => (s.path_multiplier(mm) - 1) as f32 * period(s),
            (None, None) => 0.0,
        }
    }
}

/// A user-unit value (as the command line holds it, a `double`) in seconds: `Unit::userToSta`
/// multiplies the `double` by the unit's `float` scale (`scale_`, e.g. `f32(1e-9)` for ns) in
/// `double`; the command's `float` argument narrows the product once.
pub fn user_to_sta(value: f64, scale: f32) -> f32 {
    (value * f64::from(scale)) as f32
}

/// A value in seconds back in user units, as a `double`: `Unit::staToUser` divides by the unit's
/// `float` scale in `double`. A constraint script reading a clock's period back and scaling it
/// (`period * 0.2`) sees this value.
pub fn sta_to_user(value: f32, scale: f32) -> f64 {
    f64::from(value) / f64::from(scale)
}

/// The check roles `CycleAccting` keeps that this timer reads.
pub const ACCT_SETUP: usize = 0;
pub const ACCT_HOLD: usize = 1;
pub const ACCT_LATCH_SETUP: usize = 2;
/// The gated clock hold check: in the same cycle as the setup check.
pub const ACCT_GCLK_HOLD: usize = 3;

/// `CycleAccting` between a source and a target clock edge: per role ([`ACCT_SETUP`],
/// [`ACCT_HOLD`], [`ACCT_LATCH_SETUP`]) the required time and the source and target cycles.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Accting {
    pub required: [f32; 4],
    pub src_cycle: [i32; 4],
    pub tgt_cycle: [i32; 4],
}

impl Accting {
    /// `CycleAccting::sourceTimeOffset − targetTimeOffset` for a role: the cycles times their
    /// clocks' periods (`float`).
    pub fn shift(&self, role: usize, src: &Clock, tgt: &Clock) -> f32 {
        (self.src_cycle[role] as f32 * src.period) - (self.tgt_cycle[role] as f32 * tgt.period)
    }
}

/// `CycleAccting::firstCycle`.
fn first_cycle(time: f32, period: f32) -> i32 {
    if time < 0.0 {
        1
    } else if time < period {
        0
    } else {
        -1
    }
}

/// `CycleAccting::findDelays` for edge `src_rf` of `src` launching and edge `tgt_rf` of `tgt`
/// capturing. Target cycles walked from the first (up to 100, or 1000 and more for a faster
/// target), source cycles from the first within each; per pairing in `double`: setup — the
/// target strictly (fuzzily) after the source; latch setup — the target's OPPOSITE edge strictly
/// after the source, the enable the target edge before it; hold — the target at or before the
/// source. Each keeps the first pairing with the fuzzily least delay (delays and requireds stored
/// `float`, every fuzzy test in `float`); the walk ends once both setup and hold are found and the
/// cycle starts coincide.
pub fn cycle_accting(src: &Clock, src_rf: usize, tgt: &Clock, tgt_rf: usize) -> Accting {
    use crate::fuzzy::{equal, greater, less, less_equal};
    let mut acct = Accting { required: [0.0; 4], src_cycle: [0; 4], tgt_cycle: [0; 4] };
    let mut delay = [1e30f32; 4];
    let set = |acct: &mut Accting, delay: &mut [f32; 4], role: usize, sc: i32, tc: i32, d: f64, req: f64| {
        acct.src_cycle[role] = sc;
        acct.tgt_cycle[role] = tc;
        delay[role] = d as f32;
        acct.required[role] = req as f32;
    };
    let (src_edge, tgt_edge) = (src.edge_time(src_rf), tgt.edge_time(tgt_rf));
    let tgt_opp_time1 = f64::from(tgt.edge_time(1 - tgt_rf));
    let (tgt_period, src_period) = (f64::from(tgt.period), f64::from(src.period));
    if !(tgt_period > 0.0 && src_period > 0.0) {
        return acct;
    }
    let tgt_max_cycle = if tgt_period > src_period { 100 } else { ((src_period / tgt_period).ceil() as i32).max(1000) };
    let (mut tgt_past_src, mut src_past_tgt) = (false, false);
    let mut tgt_cycle = first_cycle(tgt_edge, tgt.period);
    while tgt_cycle <= tgt_max_cycle {
        let tgt_cycle_start = f64::from(tgt_cycle) * tgt_period;
        let tgt_time = tgt_cycle_start + f64::from(tgt_edge);
        let tgt_opp_time = tgt_cycle_start + tgt_opp_time1;
        let mut src_cycle = first_cycle(src_edge, src.period);
        loop {
            let src_cycle_start = f64::from(src_cycle) * src_period;
            let src_time = src_cycle_start + f64::from(src_edge);
            if tgt_past_src && src_past_tgt && equal(src_cycle_start as f32, tgt_cycle_start as f32) {
                return acct;
            }
            if greater(src_cycle_start as f32, (tgt_cycle_start + tgt_period) as f32) && src_past_tgt {
                break;
            }
            if greater(tgt_time as f32, src_time as f32) {
                tgt_past_src = true;
                let d = tgt_time - src_time;
                if less(d as f32, delay[ACCT_SETUP]) {
                    set(&mut acct, &mut delay, ACCT_SETUP, src_cycle, tgt_cycle, d, tgt_time - src_cycle_start);
                }
            }
            if greater(tgt_opp_time as f32, src_time as f32) {
                let d = tgt_opp_time - src_time;
                if less(d as f32, delay[ACCT_LATCH_SETUP]) {
                    let (mut latch_tgt_time, mut latch_tgt_cycle) = (tgt_time, tgt_cycle);
                    if tgt_time > tgt_opp_time {
                        latch_tgt_time -= tgt_period;
                        latch_tgt_cycle -= 1;
                    }
                    set(&mut acct, &mut delay, ACCT_LATCH_SETUP, src_cycle, latch_tgt_cycle, d, latch_tgt_time - src_cycle_start);
                }
            }
            if less_equal(tgt_time as f32, src_time as f32) {
                let d = src_time - tgt_time;
                src_past_tgt = true;
                if less(d as f32, delay[ACCT_HOLD]) {
                    set(&mut acct, &mut delay, ACCT_HOLD, src_cycle, tgt_cycle, d, tgt_time - src_cycle_start);
                }
            }
            if less_equal(tgt_opp_time as f32, src_time as f32) {
                let d = src_time - tgt_time;
                if less(d as f32, delay[ACCT_GCLK_HOLD]) {
                    set(&mut acct, &mut delay, ACCT_GCLK_HOLD, src_cycle, tgt_cycle, d, tgt_time - src_cycle_start);
                }
            }
            src_cycle += 1;
        }
        tgt_cycle += 1;
    }
    acct
}

/// The setup required time between two edges of the SAME clock ([`cycle_accting`]).
pub fn setup_required_time(clock: &Clock, src_rf: usize, tgt_rf: usize) -> f32 {
    cycle_accting(clock, src_rf, clock, tgt_rf).required[ACCT_SETUP]
}

/// The hold required time between two edges of the SAME clock ([`cycle_accting`]).
pub fn hold_required_time(clock: &Clock, src_rf: usize, tgt_rf: usize) -> f32 {
    cycle_accting(clock, src_rf, clock, tgt_rf).required[ACCT_HOLD]
}

/// The latch setup required time and cycles between two edges of the SAME clock
/// ([`cycle_accting`]): `(required, source cycle, target cycle)`.
pub fn latch_setup_accting(clock: &Clock, src_rf: usize, tgt_rf: usize) -> (f32, i32, i32) {
    let a = cycle_accting(clock, src_rf, clock, tgt_rf);
    (a.required[ACCT_LATCH_SETUP], a.src_cycle[ACCT_LATCH_SETUP], a.tgt_cycle[ACCT_LATCH_SETUP])
}

/// `ClockEdge::pulseWidth`: from this edge to the opposite one.
pub fn pulse_width(clock: &Clock, rf: usize) -> f32 {
    let high = clock.waveform[1] - clock.waveform[0];
    if rf == crate::liberty::RISE {
        high
    } else {
        clock.period - high
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::liberty::{FALL, MAX, MIN, RISE};

    fn false_path(from: &[&str], to: &[&str], mm: usize) -> PathDelay {
        PathDelay { from_pins: from.iter().map(|p| p.to_string()).collect(), from_clock: false, to_pins: to.iter().map(|p| p.to_string()).collect(), to_clock: false, min_max: mm, ignore_clk_latency: false, break_path: false, delay: 0.0, false_path: true, multicycle: None }
    }

    /// Rules (`Sdc::exceptionFromStates`, `VisitPathEnds::falsePathTo`): a false path with no
    /// `-to` is complete at its start — it is no state, and the start makes no tag
    /// (`false_from`); one with a `-to` rides on the tag as a state and stops the end at its `-to`
    /// pin, as does one whose first point is that `-to`. Each only for its own min/max, and never
    /// a path delay candidate (`pathDelayTo` asks for path delays alone).
    #[test]
    fn a_false_path_stops_the_start_or_the_end_it_names() {
        let mut max_delay = false_path(&["a"], &["y"], MAX);
        (max_delay.false_path, max_delay.delay) = (false, 1e-9);
        let sdc = Sdc {
            clocks: vec![],
            input_delays: vec![],
            output_delays: vec![],
            path_delays: vec![false_path(&["rst"], &[], MAX), false_path(&["a"], &["y"], MAX), false_path(&[], &["z"], MIN), max_delay],
        };
        assert!(sdc.false_from("rst", false, MAX) && !sdc.false_from("rst", false, MIN), "complete at the start, max only");
        assert!(!sdc.false_from("a", false, MAX), "a -to: not complete at the start");
        assert_eq!(sdc.exception_from_states("rst", false, MAX), States::NONE, "a complete false path is no state");
        let from_a = sdc.exception_from_states("a", false, MAX);
        assert_eq!(from_a, States(0b1010), "the false path and the max delay from a");
        assert!(sdc.false_path_to(from_a, "y", MAX), "its state completes at y");
        assert!(!sdc.false_path_to(States::NONE, "y", MAX), "a path not from a carries no state");
        assert!(!sdc.false_path_to(from_a, "z", MAX) && sdc.false_path_to(States::NONE, "z", MIN), "first point -to z: any path, min only");
        assert_eq!(sdc.path_delay_to(from_a, "y", false, MAX), Some(3), "the max delay, though the false path ranks higher");
    }

    fn multicycle(from: &[&str], to: &[&str], mm: usize, multiplier: i32, use_end_clk: bool, all: bool) -> PathDelay {
        PathDelay { false_path: false, multicycle: Some(Multicycle { multiplier, use_end_clk, all }), ..false_path(from, to, mm) }
    }

    /// Rules (`MultiCyclePath::matches` / `priority(min_max)` / `pathMultiplier`,
    /// `PathEndClkConstrainedMcp::checkMcpAdjustment` + `findHoldMcps`): setup moves the capture
    /// edge (multiplier − 1) periods; hold takes the setup multicycle's (multiplier − 1) periods
    /// less the hold one's multiplier periods. A `-setup` multicycle rides on min tags, so it sets
    /// the hold accounting alone; of neither `-setup` nor `-hold`, its hold multiplier is 0. At one
    /// set of points the min/max's own outranks one of neither (+2 against +1). `-start` counts
    /// the launch clock's periods.
    #[test]
    fn a_multicycle_moves_setup_and_takes_hold_with_it() {
        let mut fast = Clock::new("f", 2e-9, "cf", false);
        fast.waveform = [0.0, 1e-9];
        let slow = Clock::new("s", 5e-9, "cs", false);
        let at = |path_delays: Vec<PathDelay>| Sdc { clocks: vec![fast.clone(), slow.clone()], input_delays: vec![], output_delays: vec![], path_delays };
        let (src, tgt) = (Some(0), 2);
        let adj = |sdc: &Sdc, mm: usize| {
            let states = sdc.exception_from_states("a", false, mm);
            let id = sdc.multicycle_to(states, "y", true, mm, false);
            sdc.multicycle_adjustment(id, states, "y", src, tgt, mm)
        };
        let setup3 = at(vec![multicycle(&["a"], &[], MAX, 3, true, false)]);
        assert_eq!((adj(&setup3, MAX), adj(&setup3, MIN)), (10e-9, 10e-9), "-setup 3 -end: two capture periods, for hold too");
        let both = at(vec![multicycle(&["a"], &[], MAX, 3, true, false), multicycle(&["a"], &[], MIN, 1, false, false)]);
        assert_eq!(adj(&both, MIN), 10e-9 - 2e-9, "-hold 1 (-start by default): one launch period back");
        let all = at(vec![multicycle(&["a"], &[], MAX, 3, true, true)]);
        assert_eq!((adj(&all, MAX), adj(&all, MIN)), (10e-9, 10e-9), "neither: hold multiplier 0");
        let ranked = at(vec![multicycle(&["a"], &[], MAX, 2, true, true), multicycle(&["a"], &[], MAX, 4, false, false)]);
        assert_eq!(adj(&ranked, MAX), 3.0 * 2e-9, "-setup 4 -start outranks the one of neither; launch periods");
        assert_eq!(at(vec![multicycle(&["a"], &[], MIN, 1, false, false)]).multicycle_to(States(1), "y", true, MAX, false), None, "a -hold multicycle never rules setup");
    }

    /// Rule (`PathEnd::checkClkUncertainty`): an inter-clock uncertainty from the source edge's
    /// clock to the target edge's, for the two edges and the check's min/max, replaces the
    /// target clock's own (it does not add); none set for those edges, or an unclocked path, and
    /// the target clock's own applies; a setup (max) value is negated, a hold (min) one is not.
    #[test]
    fn check_uncertainty_is_the_pair_else_the_target_clock() {
        let mut a = Clock::new("a", 2e-9, "ca", false);
        a.uncertainty = [Some(0.03e-9), Some(0.1e-9)];
        let mut b = Clock::new("b", 3e-9, "", false);
        b.uncertainty = [None, Some(0.3e-9)];
        let mut v = [[[None; 2]; 2]; 2];
        v[RISE][FALL][MAX] = Some(0.2e-9);
        b.inter_uncertainties.push(InterClockUncertainty { from: 0, value: v });
        let sdc = Sdc { clocks: vec![a, b], input_delays: vec![], output_delays: vec![], path_delays: vec![] };
        let edge = |clk: usize, rf: usize| 2 * clk + rf;
        assert_eq!(sdc.check_clk_uncertainty(Some(edge(0, RISE)), edge(1, FALL), MAX), -0.2e-9, "the pair, negated for setup");
        assert_eq!(sdc.check_clk_uncertainty(Some(edge(0, RISE)), edge(1, RISE), MAX), -0.3e-9, "no pair value for that edge: b's own");
        assert_eq!(sdc.check_clk_uncertainty(None, edge(1, FALL), MAX), -0.3e-9, "unclocked: b's own");
        assert_eq!(sdc.check_clk_uncertainty(Some(edge(1, RISE)), edge(0, RISE), MIN), 0.03e-9, "hold: a's own, not negated");
        assert_eq!(sdc.check_clk_uncertainty(Some(edge(0, RISE)), edge(1, FALL), MIN), 0.0, "neither set for hold");
    }

    /// Rise to rise: one period; rise to fall: half a period; fall to rise: a full period from the
    /// source cycle start (the fall edge's arrival already carries its half period).
    /// Rule (`Unit::userToSta` / `staToUser`, both in `double` against the `float` scale; probed
    /// on the reference with `create_clock -period p` and `[$clk period]` / `time_sta_ui`): 0.1 ns
    /// is 9.99999944e-11 (bits 2edbe6fe) and 0.35 ns 3.500000012e-10 (2fc06a1f) — narrowing 0.1 to
    /// `float` first would give 1.000000013e-10, one ulp up, and move every setup required time.
    /// 1.78 ns is bits 30f4a42e either way; read back it is 1.7799999755750928 (the decimal 1e-9
    /// would give 1.7799999252332555), and `period × 0.2` set from it lands on bits 2fc3b68b.
    #[test]
    fn unit_conversions_round_where_the_commands_do() {
        assert_eq!(user_to_sta(0.1, 1e-9).to_bits(), 0x2edb_e6fe);
        assert_ne!(user_to_sta(0.1, 1e-9).to_bits(), (0.1f32 * 1e-9f32).to_bits());
        assert_eq!(user_to_sta(0.35, 1e-9).to_bits(), 0x2fc0_6a1f);
        let period = user_to_sta(1.78, 1e-9);
        assert_eq!(period.to_bits(), 0x30f4_a42e);
        let back = sta_to_user(period, 1e-9);
        assert_eq!(back, 1.7799999755750928);
        assert_eq!(sta_to_user(user_to_sta(0.1, 1e-9), 1e-9), 0.09999999722444236);
        assert_eq!(user_to_sta(back * 0.2, 1e-9).to_bits(), 0x2fc3_b68b);
    }

    /// Rule (`Tag::stateCmp`, `exceptionStateCmp`): no states first, then fewer, then the id lists
    /// element by element — {0, 2} before {1, 2}.
    #[test]
    fn exception_states_order_as_the_tag_compare_does() {
        let k = |ids: &[usize]| States(ids.iter().fold(0, |m, i| m | 1 << i)).key();
        let order = [k(&[]), k(&[0]), k(&[1]), k(&[63]), k(&[0, 1]), k(&[0, 2]), k(&[1, 2]), k(&[0, 1, 2])];
        assert!(order.windows(2).all(|w| w[0] < w[1]), "{order:?}");
    }

    /// Rules (`Search::exceptionTo`, `fromThruToPriority`, `PathDelay::tighterThan`): of the
    /// path delays complete at a pin, the higher priority wins (a `-to` pin over a `-to` clock),
    /// a tie to the tighter (a max delay: the smaller); a `-to` clock needs a clocked end, and
    /// only the delay's own min/max matches.
    #[test]
    fn path_delay_to_takes_the_highest_priority_then_the_tighter() {
        let pd = |to_pins: &[&str], to_clock: bool, delay: f32| PathDelay { from_pins: vec!["r1/CLK".into()], from_clock: false, to_pins: to_pins.iter().map(|p| p.to_string()).collect(), to_clock, min_max: MAX, ignore_clk_latency: true, break_path: true, delay, false_path: false, multicycle: None };
        let sdc = Sdc { clocks: vec![Clock::new("c", 2.0, "clk", true)], input_delays: Vec::new(), output_delays: Vec::new(), path_delays: vec![pd(&[], true, 1.0), pd(&["r3/D"], false, 3.0), pd(&["r3/D"], false, 2.0)] };
        let all = States(0b111);
        assert_eq!(sdc.path_delay_to(all, "r3/D", true, MAX), Some(2));
        assert_eq!(sdc.path_delay_to(States(0b011), "r3/D", true, MAX), Some(1));
        assert_eq!(sdc.path_delay_to(all, "r2/D", true, MAX), Some(0));
        assert_eq!(sdc.path_delay_to(all, "r2/D", false, MAX), None);
        assert_eq!(sdc.path_delay_to(all, "r3/D", true, crate::liberty::MIN), None);
        assert_eq!(sdc.exception_from_states("r1/CLK", true, MAX), all);
        assert_eq!(sdc.exception_from_states("r1/CLK", true, crate::liberty::MIN), States::NONE);
    }

    #[test]
    fn setup_required_times_of_one_clock() {
        let c = Clock::new("c", 2.0, "clk", true);
        assert_eq!(setup_required_time(&c, RISE, RISE), 2.0);
        assert_eq!(setup_required_time(&c, RISE, FALL), 1.0);
        assert_eq!(setup_required_time(&c, FALL, RISE), 2.0);
        assert_eq!(setup_required_time(&c, FALL, FALL), 3.0);
    }

    // Rule (CycleAccting::findDelays, hold): the target edge at or before the source edge with
    // the least separation, less the source cycle's start.
    #[test]
    fn hold_required_times_of_one_clock() {
        let c = Clock::new("c", 2.0, "clk", true);
        assert_eq!(hold_required_time(&c, RISE, RISE), 0.0);
        assert_eq!(hold_required_time(&c, RISE, FALL), -1.0);
        assert_eq!(hold_required_time(&c, FALL, RISE), 0.0);
        assert_eq!(hold_required_time(&c, FALL, FALL), 1.0);
    }
}
