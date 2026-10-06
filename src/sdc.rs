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
}

impl Clock {
    /// A clock with the default waveform `{0, period / 2}`.
    pub fn new(name: &str, period: f32, source: &str, propagated: bool) -> Clock {
        Clock { name: name.into(), period, waveform: [0.0, period / 2.0], source: source.into(), propagated }
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

    /// `pathDelayPriority() + fromThruToPriority()`: the type's base (path delays: 3000) PLUS
    /// `-from` pins 1<<6, `-to` pins 1<<5, `-from` clocks 1<<3, `-to` clocks 1<<2 — added, not OR'd
    /// (3000 has bits 3 and 5 set).
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
        3000 + p
    }

    /// `PathDelay::tighterThan`: a min delay is tighter when larger, a max delay when smaller.
    fn tighter_than(&self, other: &PathDelay) -> bool {
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
    /// `Sdc::exceptionFromStates`: the states a path starting at `pin` (and, launched by the clock,
    /// at `clk`) carries — every path delay of this min/max whose `-from` names the pin, then every
    /// one whose `-from` names the clock. No false paths here, so a search always continues from it.
    pub fn exception_from_states(&self, pin: &str, clk: bool, mm: usize) -> States {
        let mut s = 0u64;
        for (id, pd) in self.path_delays.iter().enumerate() {
            if pd.min_max == mm && (pd.from_pins.iter().any(|p| p == pin) || (clk && pd.from_clock && pd.from_pins.is_empty())) {
                s |= 1 << id;
            }
        }
        States(s)
    }

    /// `Sdc::isCompleteTo(state, pin, rf, min_max)` (the mutation's test): the exception has a
    /// `-to` naming `pin` (clocks do not count) and is of this min/max.
    pub fn is_complete_to_pin(&self, id: usize, pin: &str, mm: usize) -> bool {
        let pd = &self.path_delays[id];
        pd.min_max == mm && pd.to_pins.iter().any(|p| p == pin)
    }

    /// `Search::exceptionTo(any, path, pin, rf, clk_edge, min_max)` over path delays: the highest
    /// priority — ties to the tighter — of the path's states complete at `pin` (a state with no
    /// `-to` completes anywhere), then of the exceptions whose FIRST point is a `-to` matching the
    /// pin or the target clock. `clocked`: the end has a target clock edge.
    pub fn path_delay_to(&self, states: States, pin: &str, clocked: bool, mm: usize) -> Option<usize> {
        let mut best: Option<usize> = None;
        let consider = |id: usize, best: &mut Option<usize>| {
            let pd = &self.path_delays[id];
            if pd.min_max != mm || !pd.to_matches(pin, clocked) {
                return;
            }
            let better = match *best {
                None => true,
                Some(b) => {
                    let bp = &self.path_delays[b];
                    pd.priority() > bp.priority() || (pd.priority() == bp.priority() && pd.tighter_than(bp))
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
    use crate::liberty::{FALL, MAX, RISE};

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
        let pd = |to_pins: &[&str], to_clock: bool, delay: f32| PathDelay { from_pins: vec!["r1/CLK".into()], from_clock: false, to_pins: to_pins.iter().map(|p| p.to_string()).collect(), to_clock, min_max: MAX, ignore_clk_latency: true, break_path: true, delay };
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
