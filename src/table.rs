// SPDX-License-Identifier: Apache-2.0
//! NLDM table models: axes, values and the lookup.
//!
//! Rules:
//! - an axis index is found by bisection over its `f32` values; a value at or below the first
//!   takes interval 0, at or above the last the LAST interval (`size − 2`), so a lookup outside
//!   the table extrapolates linearly from the end interval;
//! - interpolation is in `double` with weights `(1 − dx)` and `dx`, `dx = (x − xl)/(xu − xl)`;
//!   2D is `(1−dx1)(1−dx2)·y00 + dx1(1−dx2)·y10 + dx1·dx2·y11 + (1−dx1)dx2·y01`, summed in that
//!   order; the result is narrowed to `f32`;
//! - an axis of one value takes no interpolation along it.

/// A table axis variable — the common ones, plus a catch-all.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AxisVar {
    InputNetTransition,
    InputTransitionTime,
    TotalOutputNetCapacitance,
    RelatedPinTransition,
    ConstrainedPinTransition,
    RelatedOutTotalOutputNetCapacitance,
    Other,
}

impl AxisVar {
    pub fn parse(s: &str) -> AxisVar {
        match s {
            "input_net_transition" => AxisVar::InputNetTransition,
            "input_transition_time" => AxisVar::InputTransitionTime,
            "total_output_net_capacitance" => AxisVar::TotalOutputNetCapacitance,
            "related_pin_transition" => AxisVar::RelatedPinTransition,
            "constrained_pin_transition" => AxisVar::ConstrainedPinTransition,
            "related_out_total_output_net_capacitance" => AxisVar::RelatedOutTotalOutputNetCapacitance,
            _ => AxisVar::Other,
        }
    }
    /// True for a capacitance axis, false for a time axis.
    pub fn is_capacitance(self) -> bool {
        matches!(self, AxisVar::TotalOutputNetCapacitance | AxisVar::RelatedOutTotalOutputNetCapacitance)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Axis {
    pub var: AxisVar,
    pub values: Vec<f32>,
}

/// The interval `[i, i+1]` a value falls in, by bisection.
pub fn find_axis_index(value: f32, values: &[f32]) -> usize {
    let size = values.len();
    if size <= 1 || value <= values[0] {
        return 0;
    }
    if value >= values[size - 1] {
        return size - 2;
    }
    let (mut lower, mut upper) = (-1i64, size as i64);
    while upper - lower > 1 {
        let mid = (upper + lower) >> 1;
        if value >= values[mid as usize] {
            lower = mid;
        } else {
            upper = mid;
        }
    }
    lower as usize
}

/// A table of order 0 to 3; `values` row-major over the axes in order.
#[derive(Debug, Clone, PartialEq)]
pub struct Table {
    pub axes: Vec<Axis>,
    pub values: Vec<f32>,
}

impl Table {
    fn v(&self, idx: &[usize]) -> f64 {
        let mut i = 0;
        for (k, &x) in idx.iter().enumerate() {
            i = i * self.axes[k].values.len() + x;
        }
        f64::from(self.values[i])
    }

    /// The table's value at up to three axis values.
    pub fn find_value(&self, a1: f32, a2: f32, a3: f32) -> f32 {
        match self.axes.len() {
            0 => self.values[0],
            1 => self.order1(a1),
            2 => self.order2(a1, a2),
            _ => self.order3(a1, a2, a3),
        }
    }

    fn order1(&self, a1: f32) -> f32 {
        let ax = &self.axes[0].values;
        if ax.len() == 1 {
            return self.values[0];
        }
        let i = find_axis_index(a1, ax);
        let (x1, x1l, x1u) = (f64::from(a1), f64::from(ax[i]), f64::from(ax[i + 1]));
        let (y1, y2) = (self.v(&[i]), self.v(&[i + 1]));
        let dx1 = (x1 - x1l) / (x1u - x1l);
        ((1.0 - dx1) * y1 + dx1 * y2) as f32
    }

    fn order2(&self, a1: f32, a2: f32) -> f32 {
        let (ax1, ax2) = (&self.axes[0].values, &self.axes[1].values);
        let (size1, size2) = (ax1.len(), ax2.len());
        if size1 == 1 {
            if size2 == 1 {
                return self.values[0];
            }
            let i2 = find_axis_index(a2, ax2);
            let x2 = f64::from(a2);
            let y00 = self.v(&[0, i2]);
            let (x2l, x2u) = (f64::from(ax2[i2]), f64::from(ax2[i2 + 1]));
            let dx2 = (x2 - x2l) / (x2u - x2l);
            let y01 = self.v(&[0, i2 + 1]);
            return ((1.0 - dx2) * y00 + dx2 * y01) as f32;
        }
        if size2 == 1 {
            let i1 = find_axis_index(a1, ax1);
            let x1 = f64::from(a1);
            let y00 = self.v(&[i1, 0]);
            let (x1l, x1u) = (f64::from(ax1[i1]), f64::from(ax1[i1 + 1]));
            let dx1 = (x1 - x1l) / (x1u - x1l);
            let y10 = self.v(&[i1 + 1, 0]);
            return ((1.0 - dx1) * y00 + dx1 * y10) as f32;
        }
        let (i1, i2) = (find_axis_index(a1, ax1), find_axis_index(a2, ax2));
        let (x1, x2) = (f64::from(a1), f64::from(a2));
        let y00 = self.v(&[i1, i2]);
        let (x1l, x1u) = (f64::from(ax1[i1]), f64::from(ax1[i1 + 1]));
        let dx1 = (x1 - x1l) / (x1u - x1l);
        let y10 = self.v(&[i1 + 1, i2]);
        let y11 = self.v(&[i1 + 1, i2 + 1]);
        let (x2l, x2u) = (f64::from(ax2[i2]), f64::from(ax2[i2 + 1]));
        let dx2 = (x2 - x2l) / (x2u - x2l);
        let y01 = self.v(&[i1, i2 + 1]);
        ((1.0 - dx1) * (1.0 - dx2) * y00 + dx1 * (1.0 - dx2) * y10 + dx1 * dx2 * y11 + (1.0 - dx1) * dx2 * y01) as f32
    }

    fn order3(&self, a1: f32, a2: f32, a3: f32) -> f32 {
        let (ax1, ax2, ax3) = (&self.axes[0].values, &self.axes[1].values, &self.axes[2].values);
        let (i1, i2, i3) = (find_axis_index(a1, ax1), find_axis_index(a2, ax2), find_axis_index(a3, ax3));
        let (x1, x2, x3) = (f64::from(a1), f64::from(a2), f64::from(a3));
        let (mut dx1, mut dx2, mut dx3) = (0.0f64, 0.0f64, 0.0f64);
        let y000 = self.v(&[i1, i2, i3]);
        let (mut y001, mut y010, mut y011, mut y100, mut y101, mut y110, mut y111) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        if ax1.len() != 1 {
            dx1 = (x1 - f64::from(ax1[i1])) / (f64::from(ax1[i1 + 1]) - f64::from(ax1[i1]));
            y100 = self.v(&[i1 + 1, i2, i3]);
            if ax3.len() != 1 {
                y101 = self.v(&[i1 + 1, i2, i3 + 1]);
            }
            if ax2.len() != 1 {
                y110 = self.v(&[i1 + 1, i2 + 1, i3]);
                if ax3.len() != 1 {
                    y111 = self.v(&[i1 + 1, i2 + 1, i3 + 1]);
                }
            }
        }
        if ax2.len() != 1 {
            dx2 = (x2 - f64::from(ax2[i2])) / (f64::from(ax2[i2 + 1]) - f64::from(ax2[i2]));
            y010 = self.v(&[i1, i2 + 1, i3]);
            if ax3.len() != 1 {
                y011 = self.v(&[i1, i2 + 1, i3 + 1]);
            }
        }
        if ax3.len() != 1 {
            dx3 = (x3 - f64::from(ax3[i3])) / (f64::from(ax3[i3 + 1]) - f64::from(ax3[i3]));
            y001 = self.v(&[i1, i2, i3 + 1]);
        }
        ((1.0 - dx1) * (1.0 - dx2) * (1.0 - dx3) * y000
            + (1.0 - dx1) * (1.0 - dx2) * dx3 * y001
            + (1.0 - dx1) * dx2 * (1.0 - dx3) * y010
            + (1.0 - dx1) * dx2 * dx3 * y011
            + dx1 * (1.0 - dx2) * (1.0 - dx3) * y100
            + dx1 * (1.0 - dx2) * dx3 * y101
            + dx1 * dx2 * (1.0 - dx3) * y110
            + dx1 * dx2 * dx3 * y111) as f32
    }

    /// Each axis takes the input slew or the load cap by its
    /// variable (a related-output cap is 0 here).
    pub fn gate_value(&self, in_slew: f32, load_cap: f32) -> f32 {
        let pick = |k: usize| -> f32 {
            match self.axes.get(k).map(|a| a.var) {
                Some(AxisVar::InputNetTransition | AxisVar::InputTransitionTime) => in_slew,
                Some(AxisVar::TotalOutputNetCapacitance) => load_cap,
                _ => 0.0,
            }
        };
        self.find_value(pick(0), pick(1), pick(2))
    }
}

/// A gate model: a delay table and a slew table.
#[derive(Debug, Clone, PartialEq)]
pub struct GateModel {
    pub delay: Option<Table>,
    pub slew: Option<Table>,
}

impl GateModel {
    /// The delay and slew for an input slew and load cap; a negative slew clips to 0. (Process,
    /// voltage and temperature `k_` factors are not applied.)
    pub fn gate_delay(&self, in_slew: f32, load_cap: f32) -> (f32, f32) {
        let delay = self.delay.as_ref().map_or(0.0, |t| t.gate_value(in_slew, load_cap));
        let slew = self.slew.as_ref().map_or(0.0, |t| t.gate_value(in_slew, load_cap).max(0.0));
        (delay, slew)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rule: clamp to the end intervals; bisection inside.
    #[test]
    fn axis_index_clamps_to_the_end_intervals() {
        let v = [1.0f32, 2.0, 4.0, 8.0];
        assert_eq!(find_axis_index(0.5, &v), 0);
        assert_eq!(find_axis_index(1.0, &v), 0);
        assert_eq!(find_axis_index(2.0, &v), 1);
        assert_eq!(find_axis_index(5.0, &v), 2);
        assert_eq!(find_axis_index(8.0, &v), 2);
        assert_eq!(find_axis_index(20.0, &v), 2);
    }

    // Rule: bilinear in double, the four terms in their order;
    // outside the table it extrapolates from the end interval.
    #[test]
    fn two_d_lookup_is_bilinear_and_extrapolates() {
        let t = Table {
            axes: vec![Axis { var: AxisVar::InputNetTransition, values: vec![0.0, 1.0] }, Axis { var: AxisVar::TotalOutputNetCapacitance, values: vec![0.0, 2.0] }],
            values: vec![1.0, 3.0, 5.0, 9.0],
        };
        assert_eq!(t.find_value(0.5, 1.0, 0.0), ((0.5 * 0.5 * 1.0) + 0.5 * 0.5 * 5.0 + 0.5 * 0.5 * 9.0 + 0.5 * 0.5 * 3.0) as f32);
        assert_eq!(t.find_value(2.0, 0.0, 0.0), 9.0, "extrapolated along axis 1: 1 + 2·(5−1)");
        assert_eq!(t.gate_value(2.0, 0.0), 9.0);
    }
}
