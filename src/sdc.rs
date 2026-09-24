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
    #[test]
    fn setup_required_times_of_one_clock() {
        let c = Clock::new("c", 2.0, "clk", true);
        assert_eq!(setup_required_time(&c, RISE, RISE), 2.0);
        assert_eq!(setup_required_time(&c, RISE, FALL), 1.0);
        assert_eq!(setup_required_time(&c, FALL, RISE), 2.0);
        assert_eq!(setup_required_time(&c, FALL, FALL), 3.0);
    }
}
