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
}

impl PortDelay {
    /// The same value for every transition and min/max (`set_*_delay <value>`).
    pub fn uniform(port: &str, value: f32) -> PortDelay {
        PortDelay { port: port.into(), delay: [[value; 2]; 2] }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Sdc {
    pub clock: Clock,
    pub input_delays: Vec<PortDelay>,
    pub output_delays: Vec<PortDelay>,
}

/// A user-unit value (as the command line holds it, a `double`) in seconds: narrowed to `float`,
/// then scaled in `float` by the unit (`scale`, e.g. `1e-9` for ns).
pub fn user_to_sta(value: f64, scale: f32) -> f32 {
    value as f32 * scale
}

/// A value in seconds back in user units, as a `double`: divided by the unit's DECIMAL scale in
/// `double` (`1e-9`, not the `float` nearest to it). A constraint script reading a clock's period
/// back and scaling it (`period * 0.2`) sees this value.
pub fn sta_to_user(value: f32, scale: f32) -> f64 {
    let decimal: f64 = format!("{scale:e}").parse().expect("a float's decimal form parses");
    f64::from(value) / decimal
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::liberty::{FALL, RISE};

    /// Rise to rise: one period; rise to fall: half a period; fall to rise: a full period from the
    /// source cycle start (the fall edge's arrival already carries its half period).
    /// Rule: a period of 1.78 ns is `f32(1.78) × f32(1e-9)` (bits 30f4a42e, one below the
    /// double-rounded value); read back it is 1.7799999252332555, and a delay of `period × 0.2`
    /// set from it lands on bits 2fc3b68b — the value arrivals are seeded with.
    #[test]
    fn unit_conversions_round_where_the_commands_do() {
        let period = user_to_sta(1.78, 1e-9);
        assert_eq!(period.to_bits(), 0x30f4_a42e);
        assert_ne!(period.to_bits(), (1.78e-9f64 as f32).to_bits());
        let back = sta_to_user(period, 1e-9);
        assert_eq!(back, 1.7799999252332555);
        assert_eq!(user_to_sta(back * 0.2, 1e-9).to_bits(), 0x2fc3_b68b);
    }

    #[test]
    fn setup_required_times_of_one_clock() {
        let c = Clock::new("c", 2.0, "clk", true);
        assert_eq!(setup_required_time(&c, RISE, RISE), 2.0);
        assert_eq!(setup_required_time(&c, RISE, FALL), 1.0);
        assert_eq!(setup_required_time(&c, FALL, RISE), 2.0);
        assert_eq!(setup_required_time(&c, FALL, FALL), 3.0);
    }
}
