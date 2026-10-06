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
}

impl PortDelay {
    /// The same value for every transition and min/max (`set_*_delay <value>`).
    pub fn uniform(port: &str, value: f32) -> PortDelay {
        PortDelay { port: port.into(), delay: [[value; 2]; 2], exists: [[true; 2]; 2] }
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
    pub clock: Clock,
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

/// The setup required time between two edges of the SAME clock: the target edge the
/// soonest strictly (fuzzily) after the source edge, as `target time − source cycle start`,
/// computed in `double` and stored `float`.
///
/// Rule: target cycles are walked from the first, source cycles from the first within
/// each; the first pairing with the smallest delay wins (fuzzily less than the incumbent).
/// For one clock and edges inside one period this is the target edge of cycle 0 when it is after
/// the source edge, else of cycle 1.
pub fn setup_required_time(clock: &Clock, src_rf: usize, tgt_rf: usize) -> f32 {
    let period = f64::from(clock.period);
    let src_time = f64::from(clock.edge_time(src_rf));
    let tgt_edge = f64::from(clock.edge_time(tgt_rf));
    let mut best: Option<(f64, f64)> = None;
    for tgt_cycle in 0..=2 {
        let tgt_time = f64::from(tgt_cycle) * period + tgt_edge;
        for src_cycle in 0..=1 {
            let src_cycle_start = f64::from(src_cycle) * period;
            let src = src_cycle_start + src_time;
            if crate::fuzzy::greater(tgt_time as f32, src as f32) {
                let delay = tgt_time - src;
                if best.is_none_or(|(d, _)| crate::fuzzy::less(delay as f32, d as f32)) {
                    best = Some((delay, tgt_time - src_cycle_start));
                }
            }
        }
    }
    best.map_or(0.0, |(_, r)| r as f32)
}

/// The hold required time between two edges of the SAME clock: the target edge the nearest at
/// or (fuzzily) before the source edge, as `target time − source cycle start` (`CycleAccting`,
/// hold role), computed in `double` and stored `float`.
///
/// Rule: target cycles walked from the first, source cycles from the first within each; the
/// first pairing with the smallest `source − target` wins (fuzzily less than the incumbent).
pub fn hold_required_time(clock: &Clock, src_rf: usize, tgt_rf: usize) -> f32 {
    let period = f64::from(clock.period);
    let src_time = f64::from(clock.edge_time(src_rf));
    let tgt_edge = f64::from(clock.edge_time(tgt_rf));
    let mut best: Option<(f64, f64)> = None;
    for tgt_cycle in 0..=2 {
        let tgt_time = f64::from(tgt_cycle) * period + tgt_edge;
        for src_cycle in 0..=2 {
            let src_cycle_start = f64::from(src_cycle) * period;
            let src = src_cycle_start + src_time;
            if crate::fuzzy::less_equal(tgt_time as f32, src as f32) {
                let delay = src - tgt_time;
                if best.is_none_or(|(d, _)| crate::fuzzy::less(delay as f32, d as f32)) {
                    best = Some((delay, tgt_time - src_cycle_start));
                }
            }
        }
    }
    best.map_or(0.0, |(_, r)| r as f32)
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
        let sdc = Sdc { clock: Clock::new("c", 2.0, "clk", true), input_delays: Vec::new(), output_delays: Vec::new(), path_delays: vec![pd(&[], true, 1.0), pd(&["r3/D"], false, 3.0), pd(&["r3/D"], false, 2.0)] };
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
