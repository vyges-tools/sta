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
    /// `set_clock_transition` (`Clock::slews_`), `[rf][min/max]` where set: an IDEAL clock's slew
    /// at the register clock pins it reaches (`Clock::slew`: 0 where unset).
    pub slew: [[Option<f32>; 2]; 2],
    /// `set_clock_latency` on the clock (`Sdc::clockLatency(clk, rf, min_max)`), `[rf][min/max]`
    /// where set: an ideal clock's network latency where no pin latency is given.
    pub latency: [[Option<f32>; 2]; 2],
    /// `set_clock_latency -source` on the clock (`ClockInsertion`), `[rf][min/max][early/late]`
    /// where set: the source latency, added to the clock's edge wherever its arrival is taken.
    pub insertion: [[[Option<f32>; 2]; 2]; 2],
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
            slew: [[None; 2]; 2],
            latency: [[None; 2]; 2],
            insertion: [[[None; 2]; 2]; 2],
        }
    }

    /// `Sdc::clockLatency(clk, pin, rf, min_max)`: the latency given with this clock on the pin,
    /// else (that one unset for this transition and min/max) the pin's for every clock.
    pub fn pin_latency(&self, pin: &str, rf: usize, mm: usize) -> Option<f32> {
        let find = |specific: bool| self.pin_latencies.iter().find(|l| l.specific == specific && l.pin == pin).and_then(|l| l.delay[rf][mm]);
        find(true).or_else(|| find(false))
    }

    /// `Search::clockInsertion(clk, pin, rf, min_max, early_late)` with no pin insertion: the
    /// clock's source latency, 0 unset.
    pub fn insertion_of(&self, rf: usize, mm: usize, early_late: usize) -> f32 {
        self.insertion[rf][mm][early_late].unwrap_or(0.0)
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

/// One `-through` point (`ExceptionThru`): pins (and ports), nets and leaf instances, every
/// transition (`-rise_through` / `-fall_through` are not modelled).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Thru {
    pub pins: Vec<String>,
    pub nets: Vec<String>,
    pub insts: Vec<String>,
}

/// What a `-through` point is matched against: the pin a search edge arrives at, its net and its
/// instance (`ExceptionThru::matches(from_pin, to_pin, to_rf)`), and whether the edge is a wire
/// edge on that net (`edges_`: a net's driver-to-load pairs) and the pin an output or a latch
/// data pin (where an instance starts a `-through`, `exceptionThruStates`).
#[derive(Debug, Clone, Copy)]
pub struct ThruPin<'a> {
    pub pin: &'a str,
    pub net: Option<&'a str>,
    pub inst: Option<&'a str>,
    pub wire: bool,
    pub output_or_latch_data: bool,
}

impl Thru {
    /// `ExceptionThru::matches`: the pin is one of the pins, on one of the nets (a net's driver
    /// to load pairs are on the net too), or of one of the instances.
    pub fn matches(&self, to: &ThruPin) -> bool {
        self.pins.iter().any(|p| p == to.pin) || to.net.is_some_and(|n| self.nets.iter().any(|x| x == n)) || to.inst.is_some_and(|i| self.insts.iter().any(|x| x == i))
    }

    /// The `first_thru_*_exceptions_` lookups `exceptionThruStates` makes for the first point:
    /// a pin of it, a wire edge on a net of it, or — at an output or latch data pin — a pin of an
    /// instance of it.
    fn starts_at(&self, to: &ThruPin) -> bool {
        self.pins.iter().any(|p| p == to.pin)
            || (to.wire && to.net.is_some_and(|n| self.nets.iter().any(|x| x == n)))
            || (to.output_or_latch_data && to.inst.is_some_and(|i| self.insts.iter().any(|x| x == i)))
    }
}

/// `set_max_delay` / `set_min_delay` (`PathDelay`), `-from` pins and `-to` pins or the clock, every
/// transition, `-through` points. Its id (the order it was made in, `ExceptionPath::id`) is its
/// index in [`Sdc::path_delays`].
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
    /// The `-through` points, in order.
    pub thrus: Vec<Thru>,
    /// `group_path -name` (a `GroupPath`), not a delay: its name. It moves no slack — it rides
    /// on the tags as any exception does (`exceptionFromStates`), and a path end it reaches is in
    /// its group ([`Sdc::group_paths_to`]). Of every min/max (`MinMaxAll::all()`), priority 0 plus
    /// the points', never tighter. `min_max`, `delay` and the rest mean nothing for it.
    pub group: Option<String>,
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

    fn has_to(&self) -> bool {
        !self.to_pins.is_empty() || self.to_clock
    }

    /// The index of `firstState` (`makeStates`): 0, but 1 when the first point is a `-through`
    /// (no `-from`) — its state is the one AFTER that point, which starts the exception.
    pub fn first_index(&self) -> usize {
        usize::from(!self.thrus.is_empty() && !self.has_from())
    }

    /// The index of the last state, past every `-through` (`next_thru_` null).
    pub fn last_index(&self) -> usize {
        self.thrus.len()
    }

    /// How many states the exception has.
    fn state_count(&self) -> usize {
        self.last_index() + 1 - self.first_index()
    }

    /// `ExceptionState::matchesNextThru` for the state of index `index`: a `-through` still to
    /// come, the exception of this min/max (not exactly), and that point matches.
    pub fn matches_next_thru(&self, index: usize, mm: usize, to: &ThruPin) -> bool {
        self.thrus.get(index).is_some_and(|t| self.matches(mm, false) && t.matches(to))
    }

    /// `ExceptionState::isComplete` for the state of index `index`: past every `-through`, and no
    /// `-to`.
    pub fn is_complete(&self, index: usize) -> bool {
        index == self.last_index() && !self.has_to()
    }

    /// The first point is the `-to` (`ExceptionPath::firstPt`): no `-from` and no `-through`.
    fn first_is_to(&self) -> bool {
        !self.has_from() && self.thrus.is_empty()
    }

    /// `ExceptionPath::isPathDelay`.
    pub fn is_path_delay(&self) -> bool {
        !self.false_path && self.multicycle.is_none() && self.group.is_none()
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
        // `GroupPath`: `MinMaxAll::all()`, which matches either, exactly or not.
        if self.group.is_some() {
            return true;
        }
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
        if !self.thrus.is_empty() {
            p |= 1 << 4;
        }
        if self.to_clock {
            p |= 1 << 2;
        }
        // `groupPathPriority()` is 0: below every other exception.
        let base = if self.false_path {
            4000
        } else if self.multicycle.is_some() {
            2000
        } else if self.group.is_some() {
            0
        } else {
            3000
        };
        base + p
    }

    /// `PathDelay::tighterThan`: a min delay is tighter when larger, a max delay when smaller.
    /// A false path is never tighter (`FalsePath::tighterThan`): of two at one priority, the
    /// first found stays.
    fn tighter_than(&self, other: &PathDelay) -> bool {
        // `GroupPath::tighterThan`: never.
        if self.false_path || other.false_path || self.group.is_some() || other.group.is_some() {
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

/// The exception states a tag carries (`ExceptionStateSet`): one bit per state SLOT. An exception's
/// states (`makeStates`: one per `-through` still to come, and the last, past them all) take
/// consecutive slots, the exceptions in id order ([`Sdc::slot`], [`Sdc::state_of`]) — so bit order
/// is `exceptionStateCmp`'s (exception id, then state index). With no `-through` anywhere, an
/// exception has one state, complete from the start, and its slot is its id.
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

    /// The slot of state `index` of exception `id` ([`States`]): the exceptions before it take
    /// theirs first.
    pub fn slot(&self, id: usize, index: usize) -> usize {
        let base: usize = self.path_delays[..id].iter().map(PathDelay::state_count).sum();
        base + index - self.path_delays[id].first_index()
    }

    /// The exception and state index a slot stands for.
    pub fn state_of(&self, slot: usize) -> (usize, usize) {
        let mut base = 0;
        for (id, pd) in self.path_delays.iter().enumerate() {
            let n = pd.state_count();
            if slot < base + n {
                return (id, pd.first_index() + slot - base);
            }
            base += n;
        }
        panic!("state slot {slot} is past the {base} slots of the exceptions: a set holds only slots made by Sdc::slot");
    }

    /// How many state slots the exceptions take (at most 64 are modelled).
    pub fn state_slots(&self) -> usize {
        self.path_delays.iter().map(PathDelay::state_count).sum()
    }

    /// Whether any exception has a `-through`.
    pub fn has_thrus(&self) -> bool {
        self.path_delays.iter().any(|pd| !pd.thrus.is_empty())
    }

    fn with(&self, states: States, id: usize, index: usize) -> States {
        States(states.0 | 1 << self.slot(id, index))
    }

    /// `Sdc::exceptionFromStates` returning false: no path starts at `pin` ([`Sdc::from_states`]).
    pub fn false_from(&self, pin: &ThruPin, clk: bool, mm: usize) -> bool {
        self.from_states(pin, clk, mm).is_none()
    }

    /// The states [`Sdc::from_states`] gives a path starting at `pin`, none when it stops there.
    pub fn exception_from_states(&self, pin: &ThruPin, clk: bool, mm: usize) -> States {
        self.from_states(pin, clk, mm).unwrap_or(States::NONE)
    }

    /// `Sdc::exceptionFromStates(pin, rf, clk, clk_rf, min_max)`: the states of a path starting at
    /// `pin` (and, launched by the clock, `clk`). Asked of, in order and while none stops the
    /// path: the exceptions whose `-from` names the pin, those whose first point is a `-through`
    /// naming the pin, those whose first `-through` names its instance, and those whose `-from`
    /// names the clock ([`Sdc::exception_list_states`]). None (the reference's false): one of them is a
    /// complete false path, and no path starts.
    pub fn from_states(&self, pin: &ThruPin, clk: bool, mm: usize) -> Option<States> {
        let mut s = States::NONE;
        let lists: [&dyn Fn(&PathDelay) -> bool; 4] = [
            &|pd| self.first_from_pin(pd, pin.pin),
            &|pd| self.first_thru_pin(pd, pin.pin),
            &|pd| self.first_thru_inst(pd, pin.inst),
            &|pd| clk && self.first_from_clk(pd),
        ];
        for list in lists {
            if !self.exception_list_states(&mut s, list, Some(pin), mm) {
                return None;
            }
        }
        Some(s)
    }

    /// `Sdc::exceptionFromClkStates`: the states a clock's seed at its source `pin` carries — the
    /// exceptions whose `-from` names the pin (not advanced: the reference passes no pin), the
    /// first states of those whose first `-through` names the pin, then those whose `-from` names
    /// the clock. A complete false path does not stop the clock: it is kept, alone, as a marker
    /// (and what later lists add with it), so the paths the clock starts as data stop.
    pub fn clock_from_states(&self, pin: &ThruPin, mm: usize) -> States {
        let mut s = States::NONE;
        self.exception_list_states(&mut s, &|pd| self.first_from_pin(pd, pin.pin), None, mm);
        for (id, pd) in self.path_delays.iter().enumerate() {
            if self.first_thru_pin(pd, pin.pin) && pd.matches(mm, false) {
                s = self.with(s, id, pd.first_index());
            }
        }
        self.exception_list_states(&mut s, &|pd| self.first_from_clk(pd), Some(pin), mm);
        s
    }

    fn first_from_pin(&self, pd: &PathDelay, pin: &str) -> bool {
        pd.from_pins.iter().any(|p| p == pin)
    }

    fn first_thru_pin(&self, pd: &PathDelay, pin: &str) -> bool {
        !pd.has_from() && pd.thrus.first().is_some_and(|t| t.pins.iter().any(|p| p == pin))
    }

    fn first_thru_inst(&self, pd: &PathDelay, inst: Option<&str>) -> bool {
        !pd.has_from() && pd.thrus.first().is_some_and(|t| inst.is_some_and(|i| t.insts.iter().any(|x| x == i)))
    }

    fn first_from_clk(&self, pd: &PathDelay) -> bool {
        pd.from_clock && pd.from_pins.is_empty()
    }

    /// `Sdc::exceptionFromStates(exceptions, pin, rf, min_max)` over the exceptions (in id order)
    /// `list` admits, of this min/max (not exactly): each takes its first state, advanced once
    /// when its next `-through` matches `pin`. A complete false path empties the set but for its
    /// own state and ends the list: false.
    fn exception_list_states(&self, s: &mut States, list: &dyn Fn(&PathDelay) -> bool, pin: Option<&ThruPin>, mm: usize) -> bool {
        for (id, pd) in self.path_delays.iter().enumerate() {
            if !list(pd) || !pd.matches(mm, false) {
                continue;
            }
            let mut index = pd.first_index();
            // `matchesNextThru(nullptr, pin)`: `-from clk -through reg/CLK`.
            if pin.is_some_and(|p| pd.matches_next_thru(index, mm, p)) {
                index += 1;
            }
            if pd.is_complete(index) && pd.false_path {
                *s = self.with(States::NONE, id, index);
                return false;
            }
            *s = self.with(*s, id, index);
        }
        true
    }

    /// `Sdc::exceptionThruStates(from_pin, to_pin, to_rf, min_max)`: the first states of the
    /// exceptions whose first point is a `-through` met on the edge into `to` — a pin of it, a
    /// wire edge on a net of it, an output (or latch data) pin of an instance of it — of this
    /// min/max (`matchesFirstPt`, not exactly). Not advanced here: [`Sdc::mutate`]'s second pass
    /// does that.
    pub fn exception_thru_states(&self, to: &ThruPin, mm: usize) -> States {
        let mut s = States::NONE;
        for (id, pd) in self.path_delays.iter().enumerate() {
            if !pd.has_from() && pd.thrus.first().is_some_and(|t| t.starts_at(to)) && pd.matches(mm, false) {
                s = self.with(s, id, pd.first_index());
            }
        }
        s
    }

    /// `Search::mutateTag`'s states over an edge `from` -> `to` of a path of min/max `mm` whose
    /// tag is a clock's (`from_is_clk`). None (the reference's null tag): a false path completes
    /// on a data path, which stops there.
    ///
    /// Two passes. The first looks for a change: a state whose next `-through` matches `to`, or a
    /// path delay complete at `to` (a non-clock path reaching a complete false path stops). The
    /// second, when there is a change or `to` starts a `-through`-first exception, advances every
    /// state through all the `-through`s `to` matches, and keeps it unless it is a path delay
    /// complete at `from` — so a path delay's state is dropped on the edge LEAVING its `-to` pin
    /// only when that edge's own `to` makes a change too (the two passes test different pins).
    pub fn mutate(&self, states: States, from: &ThruPin, to: &ThruPin, from_is_clk: bool, mm: usize) -> Option<States> {
        let mut change = false;
        for slot in states.ids() {
            let (id, index) = self.state_of(slot);
            let pd = &self.path_delays[id];
            if pd.matches_next_thru(index, mm, to) {
                change = true;
                break;
            }
            if pd.is_complete(index) && pd.false_path && !from_is_clk {
                return None;
            }
            if pd.is_path_delay() && self.is_complete_to(id, index, to.pin, mm) {
                change = true;
                break;
            }
        }
        let fresh = if self.has_thrus() { self.exception_thru_states(to, mm) } else { States::NONE };
        if !change && fresh == States::NONE {
            return Some(states);
        }
        let mut out = fresh;
        for slot in states.ids() {
            let (id, mut index) = self.state_of(slot);
            let pd = &self.path_delays[id];
            while pd.matches_next_thru(index, mm, to) {
                index += 1;
            }
            if pd.is_complete(index) && pd.false_path && !from_is_clk {
                return None;
            }
            if !(pd.is_path_delay() && self.is_complete_to(id, index, from.pin, mm)) {
                out = self.with(out, id, index);
            }
        }
        Some(out)
    }

    /// `Sdc::isCompleteTo(state, pin, rf, min_max)` (the mutation's test): the state is past every
    /// `-through`, the exception has a `-to` naming `pin` (clocks do not count), and it is of this
    /// min/max exactly.
    pub fn is_complete_to(&self, id: usize, index: usize, pin: &str, mm: usize) -> bool {
        let pd = &self.path_delays[id];
        index == pd.last_index() && pd.matches(mm, true) && pd.to_pins.iter().any(|p| p == pin)
    }

    /// `VisitPathEnds::falsePathTo` → `Search::exceptionTo(false_path, path, pin, rf, clk_edge =
    /// null, min_max)`: of the FALSE PATHS alone, one of the path's states past every `-through`
    /// and complete at `pin`, or one whose first point is a `-to` naming the pin. Read before any
    /// end is visited, so a match makes no end at the pin at all — check, output delay, path delay
    /// or gated clock. No clock edge is passed, so `-to` the clock never matches here.
    pub fn false_path_to(&self, states: States, pin: &str, mm: usize) -> bool {
        let to = |pd: &PathDelay| pd.false_path && pd.min_max == mm && pd.to_matches(pin, false);
        states.ids().map(|s| self.state_of(s)).any(|(id, index)| index == self.path_delays[id].last_index() && to(&self.path_delays[id]))
            || self.path_delays.iter().any(|pd| pd.first_is_to() && !pd.to_pins.is_empty() && to(pd))
    }

    /// `Search::exceptionTo(any, path, pin, rf, clk_edge, min_max)` over path delays: the highest
    /// priority — ties to the tighter — of the path's states complete at `pin` (a state with no
    /// `-to` completes anywhere), then of the exceptions whose FIRST point is a `-to` matching the
    /// pin or the target clock. `clocked`: the end has a target clock edge.
    ///
    /// False paths are not candidates: a false path at the pin has already stopped the end
    /// ([`Sdc::false_path_to`]), and one that does not match here could not win.
    pub fn path_delay_to(&self, states: States, pin: &str, clocked: bool, mm: usize) -> Option<usize> {
        self.exception_to(states, pin, clocked, mm, false, &|pd| pd.is_path_delay())
    }

    /// `Search::exceptionTo(multi_cycle, …, match_min_max_exactly)`: the multicycle path ruling a
    /// check of min/max `mm` at `pin` — at a check, below any path delay (2000 against 3000).
    pub fn multicycle_to(&self, states: States, pin: &str, clocked: bool, mm: usize, exactly: bool) -> Option<usize> {
        self.exception_to(states, pin, clocked, mm, exactly, &|pd| pd.multicycle.is_some())
    }

    /// `Search::groupPathsTo(path_end)`: the group paths a path end at `pin` is in, as ids, in the
    /// reference's order. First the path's states (id order, `exceptionStateCmp`) whose exception
    /// is a group path matching the end — its `-to` naming the pin (or the target clock at a clocked
    /// end), or no `-to` at all. 🔑 NO completeness test: a state still waiting for a `-through`
    /// matches too (`exceptionMatchesTo`, not `isCompleteTo`). Then the group paths whose FIRST
    /// point is a `-to` naming the pin, then those naming the clock (`Sdc::groupPathsTo`:
    /// `require_to_pin`, so no `-to` never matches there).
    pub fn group_paths_to(&self, states: States, pin: &str, clocked: bool) -> Vec<usize> {
        let mut out = Vec::new();
        for slot in states.ids() {
            let (id, _) = self.state_of(slot);
            if self.path_delays[id].group.is_some() && self.path_delays[id].to_matches(pin, clocked) {
                out.push(id);
            }
        }
        for (id, pd) in self.path_delays.iter().enumerate() {
            if pd.group.is_some() && pd.first_is_to() && pd.to_pins.iter().any(|p| p == pin) {
                out.push(id);
            }
        }
        for (id, pd) in self.path_delays.iter().enumerate() {
            if clocked && pd.group.is_some() && pd.first_is_to() && pd.to_pins.is_empty() && pd.to_clock {
                out.push(id);
            }
        }
        out
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
        // `Sdc::isCompleteTo`: only a state past every `-through` can end here.
        for slot in states.ids() {
            let (id, index) = self.state_of(slot);
            if index == self.path_delays[id].last_index() {
                consider(id, &mut best);
            }
        }
        // `Sdc::exceptionTo`: `first_to_pin_exceptions_`, then `first_to_clk_exceptions_` (clocked
        // ends only).
        for (id, pd) in self.path_delays.iter().enumerate() {
            if pd.first_is_to() && !pd.to_pins.is_empty() && pd.to_pins.iter().any(|p| p == pin) {
                consider(id, &mut best);
            }
        }
        for (id, pd) in self.path_delays.iter().enumerate() {
            if clocked && pd.first_is_to() && pd.to_pins.is_empty() && pd.to_clock {
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

/// What `create_generated_clock` asks of its master (`Clock::divide_by_` … `invert_`), as the
/// command hands it to `Sta::makeGeneratedClock`: 0 for a divide or multiply not given (`-combinational`
/// alone is a divide by 1), `-duty_cycle` with `-multiply_by` only, `-edge_shift` in seconds.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Generated {
    pub divide_by: i32,
    pub multiply_by: i32,
    pub duty_cycle: f32,
    pub edges: Vec<i32>,
    pub edge_shifts: Vec<f32>,
    pub invert: bool,
}

/// `Clock::generate(src_clk)`: a generated clock's period and two edges from its master's, in
/// `float`. A divide by 1 copies the master; a power-of-two divide keeps the master's rise and
/// gives a 50% duty cycle; any other divide, or a multiply, scales the master's edges
/// (`generateScaledClk`; `-duty_cycle` sets the fall instead); `-edges` takes master edges
/// (1-based, counted through the master's two edges cycle after cycle) plus `-edge_shift`
/// (`generateEdgesClk`); `-invert` then moves each edge to the next one's time, the last a period
/// past the first (less a period when the first was a period or more in). None: none of the
/// forms (`create_generated_clock` stops before making such a clock).
pub fn generate(master: &Clock, g: &Generated) -> Option<(f32, [f32; 2])> {
    let src = master.waveform;
    let scaled = |scale: f32| -> (f32, [f32; 2]) {
        let period = master.period * scale;
        if g.duty_cycle != 0.0 {
            let rise = src[0] * scale;
            (period, [rise, rise + period * g.duty_cycle / 100.0])
        } else {
            (period, [src[0] * scale, src[1] * scale])
        }
    };
    let (period, mut wave) = if g.divide_by == 1 {
        (master.period, src)
    } else if g.divide_by > 1 {
        if g.divide_by & (g.divide_by - 1) == 0 {
            let period = master.period * g.divide_by as f32;
            (period, [src[0], src[0] + period / 2.0])
        } else {
            scaled(g.divide_by as f32)
        }
    } else if g.multiply_by >= 1 {
        scaled(1.0 / g.multiply_by as f32)
    } else if g.edges.len() == 3 {
        // `std::div(edge - 1, 2)`: the master edge and how many master periods on
        let at = |k: usize| {
            let e = g.edges[k] - 1;
            let shift = g.edge_shifts.get(k).copied().unwrap_or(0.0);
            (src[(e % 2) as usize] + (e / 2) as f32 * master.period, shift)
        };
        let (r, rs) = at(0);
        let rise = r + rs;
        let (f, fs) = at(1);
        let (p, ps) = at(2);
        (p - rise + ps, [rise, f + fs])
    } else {
        return None;
    };
    if g.invert {
        let first = wave[0];
        let offset = if first >= period { period } else { 0.0 };
        wave[0] = wave[1] - offset;
        wave[1] = first - offset + period;
    }
    Some((period, wave))
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

    /// A pin on no net, of no instance, reached by a gate edge.
    fn pin_at(pin: &str) -> ThruPin<'_> {
        ThruPin { pin, net: None, inst: None, wire: false, output_or_latch_data: false }
    }

    fn false_path(from: &[&str], to: &[&str], mm: usize) -> PathDelay {
        PathDelay { from_pins: from.iter().map(|p| p.to_string()).collect(), from_clock: false, to_pins: to.iter().map(|p| p.to_string()).collect(), to_clock: false, min_max: mm, ignore_clk_latency: false, break_path: false, delay: 0.0, false_path: true, multicycle: None, thrus: Vec::new(), group: None }
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
        assert!(sdc.false_from(&pin_at("rst"), false, MAX) && !sdc.false_from(&pin_at("rst"), false, MIN), "complete at the start, max only");
        assert!(!sdc.false_from(&pin_at("a"), false, MAX), "a -to: not complete at the start");
        assert_eq!(sdc.exception_from_states(&pin_at("rst"), false, MAX), States::NONE, "a complete false path is no state");
        let from_a = sdc.exception_from_states(&pin_at("a"), false, MAX);
        assert_eq!(from_a, States(0b1010), "the false path and the max delay from a");
        assert!(sdc.false_path_to(from_a, "y", MAX), "its state completes at y");
        assert!(!sdc.false_path_to(States::NONE, "y", MAX), "a path not from a carries no state");
        assert!(!sdc.false_path_to(from_a, "z", MAX) && sdc.false_path_to(States::NONE, "z", MIN), "first point -to z: any path, min only");
        assert_eq!(sdc.path_delay_to(from_a, "y", false, MAX), Some(3), "the max delay, though the false path ranks higher");
    }

    fn thru(pins: &[&str], nets: &[&str], insts: &[&str]) -> Thru {
        let v = |x: &[&str]| x.iter().map(|p| p.to_string()).collect();
        Thru { pins: v(pins), nets: v(nets), insts: v(insts) }
    }

    /// Rules (`makeStates`, `ExceptionState::isComplete` / `matchesNextThru`,
    /// `ExceptionThru::matches`, `Search::mutateTag`, `Sdc::exceptionThruStates`,
    /// `Sdc::exceptionFromStates`): an exception's states are one per `-through` still to come
    /// plus the last; with no `-from` the first `-through` is the start, and its state the one
    /// AFTER it. A state advances when its next point matches the pin an edge reaches (a pin of
    /// it, on a net of it, of an instance of it) — through every point the pin matches, in the
    /// second pass. A `-through`-first exception starts on an edge reaching a pin of its first
    /// point, a wire edge on a net of it, or an output of an instance of it; only past every
    /// point can it end, and a false path past every point with no `-to` stops a data path (a
    /// clock's carries it on). The `-through` adds 1<<4 to the priority.
    #[test]
    fn a_through_point_advances_the_state_that_waits_for_it() {
        let mut fp = false_path(&["a"], &["y"], MAX);
        fp.thrus = vec![thru(&["u1/A"], &[], &[]), thru(&[], &["n2"], &[])];
        let mut first = false_path(&[], &[], MAX);
        first.thrus = vec![thru(&[], &[], &["u3"])];
        let sdc = Sdc { clocks: vec![], input_delays: vec![], output_delays: vec![], path_delays: vec![fp, first] };
        assert_eq!(sdc.state_slots(), 4, "3 states for -from + 2 thrus, 1 for the -through-first one");
        assert_eq!((sdc.slot(0, 2), sdc.slot(1, 1), sdc.state_of(3)), (2, 3, (1, 1)));
        let start = sdc.exception_from_states(&pin_at("a"), false, MAX);
        assert_eq!(start, States(1 << sdc.slot(0, 0)), "from a, waiting for u1/A");
        let pin = |pin, net, inst, wire, out| ThruPin { pin, net, inst, wire, output_or_latch_data: out };
        let u1a = pin("u1/A", Some("n1"), Some("u1"), true, false);
        let s1 = sdc.mutate(start, &pin_at("a"), &u1a, false, MAX).unwrap();
        assert_eq!(s1, States(1 << sdc.slot(0, 1)), "past u1/A, waiting for n2");
        assert!(!sdc.false_path_to(s1, "y", MAX), "not past every point: no end at y");
        let u1x = pin("u1/X", Some("n2"), Some("u1"), false, true);
        let s2 = sdc.mutate(s1, &u1a, &u1x, false, MAX).unwrap();
        assert_eq!(s2, States(1 << sdc.slot(0, 2)), "u1/X is on n2: past every point");
        assert!(sdc.false_path_to(s2, "y", MAX), "now it ends the path at y");
        assert_eq!(sdc.mutate(States::NONE, &pin_at("x"), &pin("u3/A", None, Some("u3"), true, false), false, MAX), Some(States::NONE), "an instance starts at its output only");
        let u3x = pin("u3/X", None, Some("u3"), false, true);
        let marked = sdc.mutate(States::NONE, &pin_at("u3/A"), &u3x, false, MAX).unwrap();
        assert_eq!(marked, States(1 << sdc.slot(1, 1)), "the first state, complete, rides into u3/X");
        assert_eq!(sdc.mutate(marked, &u3x, &pin_at("z"), false, MAX), None, "the next data edge stops it");
        assert_eq!(sdc.mutate(marked, &u3x, &pin_at("z"), true, MAX), Some(marked), "a clock's tag carries it on");
        assert!(sdc.false_from(&pin("u3/X", None, Some("u3"), false, true), false, MAX), "a path starting on u3 stops at once");
        assert_eq!(sdc.path_delays[0].priority() - false_path(&["a"], &["y"], MAX).priority(), 1 << 4);
    }

    /// Rule (`Sdc::exceptionFromClkStates`): a clock's seed keeps a false path complete at its
    /// pin as a marker instead of stopping, and adds the `-through`-first exceptions starting at
    /// the pin.
    #[test]
    fn a_clock_seed_carries_a_complete_false_path() {
        let mut first = false_path(&[], &["q"], MAX);
        first.thrus = vec![thru(&["ck"], &[], &[])];
        let sdc = Sdc { clocks: vec![], input_delays: vec![], output_delays: vec![], path_delays: vec![false_path(&["ck"], &[], MAX), first] };
        assert!(sdc.false_from(&pin_at("ck"), true, MAX));
        assert_eq!(sdc.clock_from_states(&pin_at("ck"), MAX), States(0b11), "the marker, and the -through-first state");
    }

    fn group(name: &str, from: &[&str], to: &[&str]) -> PathDelay {
        PathDelay { false_path: false, group: Some(name.into()), ..false_path(from, to, MAX) }
    }

    /// Rules (`GroupPath`, `Search::groupPathsTo`, `Sdc::groupPathsTo`): a group path is of every
    /// min/max (`MinMaxAll::all()`), priority 0 plus its points, never tighter, and never a path
    /// delay (no internal pins, no `pathDelayTo` candidate). It starts on the tag like any
    /// exception. A path end is in it when the path carries one of its states and its `-to` names
    /// the end — with NO completeness test, so a state still waiting for a `-through` counts — or
    /// no `-to` at all; or when its first point is a `-to` naming the end (that list requires a
    /// `-to`). States first, in id order, then the first-`-to` group paths.
    #[test]
    fn a_path_end_is_in_the_groups_its_states_and_its_pin_name() {
        let mut waits = group("w", &["a"], &["y"]);
        waits.thrus = vec![thru(&["u9/A"], &[], &[])];
        let sdc = Sdc {
            clocks: vec![],
            input_delays: vec![],
            output_delays: vec![],
            path_delays: vec![group("g", &["a"], &["y"]), group("t", &[], &["y"]), waits, group("all", &["a"], &[])],
        };
        let g = &sdc.path_delays[0];
        assert!(g.matches(MIN, true) && g.matches(MAX, true), "of every min/max, exactly");
        assert_eq!(g.priority(), 1 << 6 | 1 << 5, "priority 0 plus -from and -to pins");
        assert!(!g.is_path_delay() && !g.tighter_than(&sdc.path_delays[3]));
        let from_a = sdc.exception_from_states(&pin_at("a"), false, MIN);
        assert_eq!(from_a, States(1 << sdc.slot(0, 0) | 1 << sdc.slot(2, 0) | 1 << sdc.slot(3, 0)), "g, w (waiting for u9/A) and all start at a, for min too");
        assert_eq!(sdc.group_paths_to(from_a, "y", false), vec![0, 2, 3, 1], "states in id order (w not through u9/A, yet in), then the first -to");
        assert_eq!(sdc.group_paths_to(from_a, "z", false), vec![3], "no -to: any end");
        assert_eq!(sdc.group_paths_to(States::NONE, "y", false), vec![1], "a path from elsewhere: the first -to only");
        assert_eq!(sdc.path_delay_to(from_a, "y", false, MAX), None, "never a path delay");
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
            let states = sdc.exception_from_states(&pin_at("a"), false, mm);
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

    /// Rules (`Clock::generate`, `generateScaledClk`, `generateEdgesClk`): a power-of-two divide
    /// keeps the rise, 50% duty; a divide by 3 scales both edges; a multiply scales by 1/m in
    /// float, `-duty_cycle` setting the fall; `-edges {1 3 5}` is a divide by 2 on the master's
    /// edges, a shift moving each; `-invert` swaps rise and fall a period on. Neither form: none.
    #[test]
    fn a_generated_clock_derives_its_waveform_from_the_master() {
        let mut m = Clock::new("m", 2e-9, "clk", false);
        m.waveform = [0.5e-9, 1.5e-9];
        let gen = |g: Generated| generate(&m, &g).unwrap();
        assert_eq!(gen(Generated { divide_by: 1, ..Default::default() }), (2e-9, [0.5e-9, 1.5e-9]));
        assert_eq!(gen(Generated { divide_by: 4, ..Default::default() }), (8e-9, [0.5e-9, 0.5e-9 + 4e-9]));
        assert_eq!(gen(Generated { divide_by: 3, ..Default::default() }), (2e-9 * 3.0, [0.5e-9 * 3.0, 1.5e-9 * 3.0]));
        let s = 1.0f32 / 2.0;
        assert_eq!(gen(Generated { multiply_by: 2, duty_cycle: 25.0, ..Default::default() }), (2e-9 * s, [0.5e-9 * s, 0.5e-9 * s + 2e-9 * s * 25.0 / 100.0]));
        assert_eq!(gen(Generated { edges: vec![1, 3, 5], ..Default::default() }), (0.5e-9 + 2.0 * 2e-9 - 0.5e-9, [0.5e-9, 0.5e-9 + 2e-9]));
        let shifted = gen(Generated { edges: vec![2, 3, 4], edge_shifts: vec![0.1e-9, 0.2e-9, 0.1e-9], ..Default::default() });
        assert_eq!(shifted, (1.5e-9 + 2e-9 - (1.5e-9 + 0.1e-9) + 0.1e-9, [1.5e-9 + 0.1e-9, 0.5e-9 + 2e-9 + 0.2e-9]));
        let (p, w) = gen(Generated { divide_by: 2, invert: true, ..Default::default() });
        assert_eq!((p, w), (4e-9, [0.5e-9 + 2e-9, 0.5e-9 + 4e-9]));
        assert_eq!(generate(&m, &Generated::default()), None);
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
        let pd = |to_pins: &[&str], to_clock: bool, delay: f32| PathDelay { from_pins: vec!["r1/CLK".into()], from_clock: false, to_pins: to_pins.iter().map(|p| p.to_string()).collect(), to_clock, min_max: MAX, ignore_clk_latency: true, break_path: true, delay, false_path: false, multicycle: None, thrus: Vec::new(), group: None };
        let sdc = Sdc { clocks: vec![Clock::new("c", 2.0, "clk", true)], input_delays: Vec::new(), output_delays: Vec::new(), path_delays: vec![pd(&[], true, 1.0), pd(&["r3/D"], false, 3.0), pd(&["r3/D"], false, 2.0)] };
        let all = States(0b111);
        assert_eq!(sdc.path_delay_to(all, "r3/D", true, MAX), Some(2));
        assert_eq!(sdc.path_delay_to(States(0b011), "r3/D", true, MAX), Some(1));
        assert_eq!(sdc.path_delay_to(all, "r2/D", true, MAX), Some(0));
        assert_eq!(sdc.path_delay_to(all, "r2/D", false, MAX), None);
        assert_eq!(sdc.path_delay_to(all, "r3/D", true, crate::liberty::MIN), None);
        assert_eq!(sdc.exception_from_states(&pin_at("r1/CLK"), true, MAX), all);
        assert_eq!(sdc.exception_from_states(&pin_at("r1/CLK"), true, crate::liberty::MIN), States::NONE);
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
