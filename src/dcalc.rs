// SPDX-License-Identifier: Apache-2.0
//! Gate and wire delay: the DMP effective-capacitance calculator on a pi model with Elmore wire
//! delays, with its arithmetic fully specified.
//!
//! Rules:
//! - the table is called with `f32` arguments (input slew, capacitance) and answers `f32`; the
//!   algorithm itself runs in `double`;
//! - `rd`: two table lookups at `c1+c2` and `+1 fF`, all `f32` but the log;
//! - the algorithm: capacitive (`rd < 1e-2`, `rpi < rd·1e-3`, `c1 = 0`, `c1 < c2·1e-3` or `rpi = 0`),
//!   zero-c2 (`c2 < c1·1e-3`), else the full pi; a failed pi falls back to `ceff = c1 + c2`;
//! - `exp2` is `(1 + x/4096)` squared twelve times, 0 below −12 — never libm's `exp`;
//! - the Newton step solves with a closed-form inverse (cofactors × 1/det) and a fixed summation
//!   order for the product; there is NO fused multiply-add anywhere.

use crate::table::GateModel;

/// A computation the algorithm abandons.
#[derive(Debug, Clone, PartialEq)]
pub struct DmpError(pub &'static str);

/// The library's thresholds for the output transition, as the calculator reads them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Thresholds {
    pub vth: f32,
    pub vl: f32,
    pub vh: f32,
    pub slew_derate: f32,
}

/// `exp2`: a fast exponential, NOT libm's.
pub fn exp2(x: f64) -> f64 {
    if x < -12.0 {
        return 0.0;
    }
    let mut y = 1.0 + x / 4096.0;
    for _ in 0..12 {
        y *= y;
    }
    y
}

/// A root by Newton–Raphson safeguarded by bisection.
/// `(root, failed)`.
pub fn find_root(func: &mut dyn FnMut(f64) -> (f64, f64), x1: f64, x2: f64, x_tol: f64, max_iter: i32) -> (f64, bool) {
    let (y1, _) = func(x1);
    let (y2, _) = func(x2);
    let (mut x1, mut x2) = (x1, x2);
    if (y1 > 0.0 && y2 > 0.0) || (y1 < 0.0 && y2 < 0.0) {
        return (0.0, true);
    }
    if y1 == 0.0 {
        return (x1, false);
    }
    if y2 == 0.0 {
        return (x2, false);
    }
    if y1 > 0.0 {
        std::mem::swap(&mut x1, &mut x2);
    }
    let mut root = (x1 + x2) * 0.5;
    let mut dx_prev = (x2 - x1).abs();
    let mut dx = dx_prev;
    let (mut y, mut dy) = func(root);
    for _ in 0..max_iter {
        if (((root - x2) * dy - y) * ((root - x1) * dy - y) > 0.0) || ((2.0 * y).abs() > (dx_prev * dy).abs()) {
            dx_prev = dx;
            dx = (x2 - x1) * 0.5;
            root = x1 + dx;
        } else {
            dx_prev = dx;
            dx = y / dy;
            root -= dx;
        }
        if dx.abs() <= x_tol * root.abs() {
            return (root, false);
        }
        (y, dy) = func(root);
        if y < 0.0 {
            x1 = root;
        } else {
            x2 = root;
        }
    }
    (root, true)
}

type M3 = [[f64; 3]; 3];

fn cofactor(m: &M3, i: usize, j: usize) -> f64 {
    let (i1, i2, j1, j2) = ((i + 1) % 3, (i + 2) % 3, (j + 1) % 3, (j + 2) % 3);
    m[i1][j1] * m[i2][j2] - m[i1][j2] * m[i2][j1]
}

/// The 3×3 determinant, by first-row cofactor expansion.
fn det3(m: &M3) -> f64 {
    let h = |a: usize, b: usize, c: usize| m[0][a] * (m[1][b] * m[2][c] - m[1][c] * m[2][b]);
    h(0, 1, 2) - h(1, 0, 2) + h(2, 0, 1)
}

/// `J⁻¹ · (−f)` for a 3×3 Jacobian: the closed-form inverse (cofactors × 1/det), then the product.
fn solve3(m: &M3, f: [f64; 3]) -> [f64; 3] {
    let c0 = [cofactor(m, 0, 0), cofactor(m, 1, 0), cofactor(m, 2, 0)];
    let det = c0[0] * m[0][0] + c0[1] * m[1][0] + c0[2] * m[2][0];
    let invdet = 1.0 / det;
    let mut inv = [[0.0f64; 3]; 3];
    inv[1][0] = cofactor(m, 0, 1) * invdet;
    inv[1][1] = cofactor(m, 1, 1) * invdet;
    inv[2][0] = cofactor(m, 0, 2) * invdet;
    inv[1][2] = cofactor(m, 2, 1) * invdet;
    inv[2][1] = cofactor(m, 1, 2) * invdet;
    inv[2][2] = cofactor(m, 2, 2) * invdet;
    inv[0] = [c0[0] * invdet, c0[1] * invdet, c0[2] * invdet];
    lazy_product3(&inv, [-f[0], -f[1], -f[2]])
}

/// A 3×3 matrix times a vector, in a fixed summation order: rows 0–1 accumulate column by column
/// — `(a0·b0 + a1·b1) + a2·b2` — and row 2 sums its last two terms first —
/// `a0·b0 + (a1·b1 + a2·b2)`. The two orders differ in the last bit, and a Newton solve can land
/// on either side of it.
fn lazy_product3(m: &M3, b: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * b[0] + m[0][1] * b[1] + m[0][2] * b[2],
        m[1][0] * b[0] + m[1][1] * b[1] + m[1][2] * b[2],
        m[2][0] * b[0] + (m[2][1] * b[1] + m[2][2] * b[2]),
    ]
}

/// `J⁻¹ · (−f)` for the top-left 2×2 of the Jacobian, by its closed-form inverse.
fn solve2(m: &M3, f: [f64; 3]) -> [f64; 3] {
    let det = m[0][0] * m[1][1] - m[1][0] * m[0][1];
    let invdet = 1.0 / det;
    let inv = [[m[1][1] * invdet, -m[0][1] * invdet], [-m[1][0] * invdet, m[0][0] * invdet]];
    let nf = [-f[0], -f[1]];
    [inv[0][0] * nf[0] + inv[0][1] * nf[1], inv[1][0] * nf[0] + inv[1][1] * nf[1], 0.0]
}

const T0: usize = 0;
const DT: usize = 1;
const CEFF: usize = 2;
const Y20: usize = 0;
const Y50: usize = 1;
const IPI: usize = 2;
const DRIVER_PARAM_TOL: f64 = 0.01;
const VTH_TIME_TOL: f64 = 0.01;
const FIND_ROOT_MAX_ITER: i32 = 20;
const NEWTON_RAPHSON_MAX_ITER: i32 = 100;

/// Which algorithm a gate arc uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Alg {
    Cap,
    Pi,
    ZeroC2,
}

/// The DMP state of one gate arc — one of three kinds — which its loads read.
#[derive(Debug, Clone)]
pub struct Dmp<'a> {
    pub alg: Alg,
    model: &'a GateModel,
    rd: f64,
    in_slew: f64,
    c2: f64,
    rpi: f64,
    c1: f64,
    vth: f64,
    vl: f64,
    vh: f64,
    slew_derate: f64,
    t0: f64,
    dt: f64,
    pub ceff: f64,
    drvr_slew: f64,
    vo_delay: f64,
    driver_valid: bool,
    elmore: f64,
    p3: f64,
    // Pi / zero-c2 poles, zeros and coefficients.
    p1: f64,
    p2: f64,
    z1: f64,
    k0: f64,
    k1: f64,
    k2: f64,
    k3: f64,
    k4: f64,
    a: f64,
    b: f64,
    d: f64,
}

/// The driver resistance the table implies around `c1 + c2`.
fn gate_model_rd(model: &GateModel, th: &Thresholds, in_slew: f64, c2: f64, c1: f64) -> f64 {
    let cap1 = (c1 + c2) as f32;
    let cap2 = (f64::from(cap1) + 1e-15) as f32;
    let (d1, _) = model.gate_delay(in_slew as f32, cap1);
    let (d2, _) = model.gate_delay(in_slew as f32, cap2);
    let vth = f64::from(th.vth);
    let rd = (-vth.ln() * f64::from((d1 - d2).abs()) / f64::from(cap2 - cap1)) as f32;
    f64::from(rd)
}

impl<'a> Dmp<'a> {
    /// Chooses the algorithm, then initialises the chosen kind.
    pub fn new(model: &'a GateModel, th: &Thresholds, in_slew: f32, c2: f32, rpi: f32, c1: f32) -> Dmp<'a> {
        let (in_slew, c2, rpi, c1) = (f64::from(in_slew), f64::from(c2), f64::from(rpi), f64::from(c1));
        let rd = gate_model_rd(model, th, in_slew, c2, c1);
        let alg = if rd < 1e-2 || rpi < rd * 1e-3 || (c1 == 0.0 || c1 < c2 * 1e-3 || rpi == 0.0) {
            Alg::Cap
        } else if c2 < c1 * 1e-3 {
            Alg::ZeroC2
        } else {
            Alg::Pi
        };
        let mut d = Dmp {
            alg,
            model,
            rd,
            in_slew,
            c2,
            rpi,
            c1,
            vth: f64::from(th.vth),
            vl: f64::from(th.vl),
            vh: f64::from(th.vh),
            slew_derate: f64::from(th.slew_derate),
            t0: 0.0,
            dt: 0.0,
            ceff: 0.0,
            drvr_slew: 0.0,
            vo_delay: 0.0,
            driver_valid: false,
            elmore: 0.0,
            p3: 0.0,
            p1: 0.0,
            p2: 0.0,
            z1: 0.0,
            k0: 0.0,
            k1: 0.0,
            k2: 0.0,
            k3: 0.0,
            k4: 0.0,
            a: 0.0,
            b: 0.0,
            d: 0.0,
        };
        match alg {
            Alg::Cap => d.ceff = c1 + c2,
            Alg::Pi => {
                d.z1 = 1.0 / (d.rpi * d.c1);
                d.k0 = 1.0 / (d.rd * d.c2);
                let a = d.rpi * d.rd * d.c1 * d.c2;
                let b = d.rd * (d.c1 + d.c2) + d.rpi * d.c1;
                let sqrt = (b * b - 4.0 * a).sqrt();
                d.p1 = (b + sqrt) / (2.0 * a);
                d.p2 = (b - sqrt) / (2.0 * a);
                let p1p2 = d.p1 * d.p2;
                d.k2 = d.z1 / p1p2;
                d.k1 = (1.0 - d.k2 * (d.p1 + d.p2)) / p1p2;
                d.k4 = (d.k1 * d.p1 + d.k2) / (d.p2 - d.p1);
                d.k3 = -d.k1 - d.k4;
                let z = (d.c1 + d.c2) / (d.rpi * d.c1 * d.c2);
                d.a = z / p1p2;
                d.b = (z - d.p1) / (d.p1 * (d.p1 - d.p2));
                d.d = (z - d.p2) / (d.p2 * (d.p2 - d.p1));
            }
            Alg::ZeroC2 => {
                d.ceff = c1;
                d.z1 = 1.0 / (d.rpi * d.c1);
                d.p1 = 1.0 / (d.c1 * (d.rd + d.rpi));
                d.k0 = d.p1 / d.z1;
                d.k2 = 1.0 / d.k0;
                d.k1 = (d.p1 - d.z1) / (d.p1 * d.p1);
                d.k3 = -d.k1;
            }
        }
        d
    }

    pub fn driver_valid(&self) -> bool {
        self.driver_valid
    }
    /// The solver state, for traces: `(rd, t0, dt, ceff)`.
    pub fn state(&self) -> (f64, f64, f64, f64) {
        (self.rd, self.t0, self.dt, self.ceff)
    }

    fn nr_order(&self) -> usize {
        match self.alg {
            Alg::Cap => 1,
            Alg::ZeroC2 => 2,
            Alg::Pi => 3,
        }
    }

    /// The table at `ceff` (both arguments narrowed to `f32`).
    fn gate_cap_delay_slew(&self, ceff: f64) -> (f64, f64) {
        let (d, s) = self.model.gate_delay(self.in_slew as f32, ceff as f32);
        (f64::from(d), f64::from(s))
    }

    /// `(t_vth, t_vl, measured slew)` at `ceff`.
    fn gate_delays(&self, ceff: f64) -> (f64, f64, f64) {
        let (t_vth, table_slew) = self.gate_cap_delay_slew(ceff);
        let slew = table_slew * self.slew_derate;
        let t_vl = t_vth - slew * (self.vth - self.vl) / (self.vh - self.vl);
        (t_vth, t_vl, slew)
    }

    fn y0(&self, t: f64, cl: f64) -> f64 {
        t - self.rd * cl * (1.0 - exp2(-t / (self.rd * cl)))
    }
    fn y0dt(&self, t: f64, cl: f64) -> f64 {
        1.0 - exp2(-t / (self.rd * cl))
    }
    fn y0dcl(&self, t: f64, cl: f64) -> f64 {
        self.rd * ((1.0 + t / (self.rd * cl)) * exp2(-t / (self.rd * cl)) - 1.0)
    }
    fn y(&self, t: f64, t0: f64, dt: f64, cl: f64) -> f64 {
        let t1 = t - t0;
        if t1 <= 0.0 {
            0.0
        } else if t1 <= dt {
            self.y0(t1, cl) / dt
        } else {
            (self.y0(t1, cl) - self.y0(t1 - dt, cl)) / dt
        }
    }
    fn dy(&self, t: f64, t0: f64, dt: f64, cl: f64) -> (f64, f64, f64) {
        let t1 = t - t0;
        if t1 <= 0.0 {
            return (0.0, 0.0, 0.0);
        }
        if t1 <= dt {
            return (-self.y0dt(t1, cl) / dt, -self.y0(t1, cl) / (dt * dt), self.y0dcl(t1, cl) / dt);
        }
        let dydt0 = -(self.y0dt(t1, cl) - self.y0dt(t1 - dt, cl)) / dt;
        let dyddt = -(self.y0(t1, cl) + self.y0(t1 - dt, cl)) / (dt * dt) + self.y0dt(t1 - dt, cl) / dt;
        let dydcl = (self.y0dcl(t1, cl) - self.y0dcl(t1 - dt, cl)) / dt;
        (dydt0, dyddt, dydcl)
    }

    fn ipi_iceff(&self, dt: f64, ceff_time: f64, ceff: f64) -> f64 {
        let exp_p1_dt = exp2(-self.p1 * ceff_time);
        let exp_p2_dt = exp2(-self.p2 * ceff_time);
        let exp_dt_rd_ceff = exp2(-ceff_time / (self.rd * ceff));
        let ipi = (self.a * ceff_time + (self.b / self.p1) * (1.0 - exp_p1_dt) + (self.d / self.p2) * (1.0 - exp_p2_dt)) / (self.rd * ceff_time * dt);
        let iceff = (self.rd * ceff * ceff_time - (self.rd * ceff) * (self.rd * ceff) * (1.0 - exp_dt_rd_ceff)) / (self.rd * ceff_time * dt);
        ipi - iceff
    }

    /// The residuals and the Jacobian (pi and one-pole kinds).
    fn eval(&self, x: &mut [f64; 3], fvec: &mut [f64; 3], fjac: &mut M3) -> Result<(), DmpError> {
        match self.alg {
            Alg::Pi => {
                let (t0, dt, ceff) = (x[T0], x[DT], x[CEFF]);
                if ceff < 0.0 {
                    return Err(DmpError("eqn eval failed: ceff < 0"));
                }
                if ceff > self.c1 + self.c2 {
                    return Err(DmpError("eqn eval failed: ceff > c2 + c1"));
                }
                if dt <= 0.0 {
                    return Err(DmpError("eqn eval failed: dt < 0"));
                }
                let (t_vth, t_vl, slew) = self.gate_delays(ceff);
                if slew == 0.0 {
                    return Err(DmpError("eqn eval failed: slew = 0"));
                }
                let ceff_time = (slew / (self.vh - self.vl)).min(1.4 * dt);
                let exp_p1_dt = exp2(-self.p1 * dt);
                let exp_p2_dt = exp2(-self.p2 * dt);
                let exp_dt_rd_ceff = exp2(-dt / (self.rd * ceff));
                let y50 = self.y(t_vth, t0, dt, ceff);
                let y20 = self.y(t_vl, t0, dt, ceff);
                fvec[IPI] = self.ipi_iceff(dt, ceff_time, ceff);
                fvec[Y50] = y50 - self.vth;
                fvec[Y20] = y20 - self.vl;
                let b_div_p1 = self.b / self.p1;
                let d_div_p2 = self.d / self.p2;
                let rd_ceff = self.rd * ceff;
                fjac[IPI][T0] = 0.0;
                let term_a = -self.a * dt;
                let term_b = self.b * dt * exp_p1_dt - 2.0 * b_div_p1 * (1.0 - exp_p1_dt);
                let term_d = self.d * dt * exp_p2_dt - 2.0 * d_div_p2 * (1.0 - exp_p2_dt);
                let term_rd = rd_ceff * (dt + dt * exp_dt_rd_ceff - 2.0 * rd_ceff * (1.0 - exp_dt_rd_ceff));
                fjac[IPI][DT] = (term_a + term_b + term_d + term_rd) / (self.rd * dt * dt * dt);
                let two_rd_ceff = 2.0 * rd_ceff;
                fjac[IPI][CEFF] = (two_rd_ceff - dt - (two_rd_ceff + dt) * exp_dt_rd_ceff) / (dt * dt);
                (fjac[Y20][T0], fjac[Y20][DT], fjac[Y20][CEFF]) = self.dy(t_vl, t0, dt, ceff);
                (fjac[Y50][T0], fjac[Y50][DT], fjac[Y50][CEFF]) = self.dy(t_vth, t0, dt, ceff);
                Ok(())
            }
            Alg::ZeroC2 => {
                let t0 = x[T0];
                let mut dt = x[DT];
                let (t_vth, t_vl, _) = self.gate_delays(self.ceff);
                if dt <= 0.0 {
                    dt = (t_vl - t_vth) / 100.0;
                    x[DT] = dt;
                }
                fvec[Y50] = self.y(t_vth, t0, dt, self.ceff) - self.vth;
                fvec[Y20] = self.y(t_vl, t0, dt, self.ceff) - self.vl;
                let (a, b, _) = self.dy(t_vl, t0, dt, self.ceff);
                fjac[Y20][T0] = a;
                fjac[Y20][DT] = b;
                let (a, b, _) = self.dy(t_vth, t0, dt, self.ceff);
                fjac[Y50][T0] = a;
                fjac[Y50][DT] = b;
                Ok(())
            }
            Alg::Cap => Ok(()),
        }
    }

    /// Newton–Raphson until every step is within `driver_param_tol` of its parameter.
    fn newton_raphson(&self, x: &mut [f64; 3]) -> Result<(), DmpError> {
        let n = self.nr_order();
        let mut fvec = [0.0f64; 3];
        let mut fjac: M3 = [[0.0; 3]; 3];
        for _ in 0..NEWTON_RAPHSON_MAX_ITER {
            self.eval(x, &mut fvec, &mut fjac)?;
            let p = if n == 2 {
                let det = fjac[0][0] * fjac[1][1] - fjac[1][0] * fjac[0][1];
                if det.abs() < 1e-12 {
                    return Err(DmpError("Jacobian is singular (order 2)"));
                }
                solve2(&fjac, fvec)
            } else {
                if det3(&fjac).abs() < 1e-12 {
                    return Err(DmpError("Jacobian is singular (order 3)"));
                }
                solve3(&fjac, fvec)
            };
            let all_under = (0..n).all(|i| p[i].abs() <= x[i].abs() * DRIVER_PARAM_TOL);
            for i in 0..n {
                x[i] += p[i];
            }
            if all_under {
                return Ok(());
            }
        }
        Err(DmpError("Newton-Raphson max iterations exceeded"))
    }

    /// The driver waveform parameters at `ceff`.
    fn find_driver_params(&mut self, ceff: f64) -> Result<(), DmpError> {
        let mut x = [0.0f64; 3];
        if self.nr_order() == 3 {
            x[CEFF] = ceff;
        }
        let (t_vth, _t_vl, slew) = self.gate_delays(ceff);
        let dt = slew / (self.vh - self.vl);
        let t0 = t_vth + (1.0 - self.vth).ln() * self.rd * ceff - self.vth * dt;
        x[DT] = dt;
        x[T0] = t0;
        self.newton_raphson(&mut x)?;
        self.t0 = x[T0];
        self.dt = x[DT];
        if self.nr_order() == 3 {
            self.ceff = x[CEFF];
        }
        Ok(())
    }

    /// `V0` (unit-ramp driver output) for the pi / zero-c2 kinds.
    fn v0(&self, t: f64) -> (f64, f64) {
        match self.alg {
            Alg::Pi => {
                let (e1, e2) = (exp2(-self.p1 * t), exp2(-self.p2 * t));
                (self.k0 * (self.k1 + self.k2 * t + self.k3 * e1 + self.k4 * e2), self.k0 * (self.k2 - self.k3 * self.p1 * e1 - self.k4 * self.p2 * e2))
            }
            Alg::ZeroC2 => {
                let e1 = exp2(-self.p1 * t);
                (self.k0 * (self.k1 + self.k2 * t + self.k3 * e1), self.k0 * (self.k2 - self.k3 * self.p1 * e1))
            }
            Alg::Cap => (0.0, 0.0),
        }
    }

    fn vo(&self, t: f64) -> (f64, f64) {
        let t1 = t - self.t0;
        if t1 <= 0.0 {
            return (0.0, 0.0);
        }
        if t1 <= self.dt {
            let (v0, dv0) = self.v0(t1);
            return (v0 / self.dt, dv0 / self.dt);
        }
        let (v0, dv0) = self.v0(t1);
        let (v0_dt, dv0_dt) = self.v0(t1 - self.dt);
        ((v0 - v0_dt) / self.dt, (dv0 - dv0_dt) / self.dt)
    }

    fn vo_crossing_upper_bound(&self) -> f64 {
        match self.alg {
            Alg::Pi => self.t0 + self.dt + (self.c1 + self.c2) * (self.rd + self.rpi) * 2.0,
            Alg::ZeroC2 => self.t0 + self.dt + self.c1 * (self.rd + self.rpi) * 2.0,
            Alg::Cap => 0.0,
        }
    }

    fn find_vo_crossing(&self, vth: f64, t_lower: f64, t_upper: f64) -> Result<f64, DmpError> {
        let mut f = |t: f64| {
            let (vo, dvo) = self.vo(t);
            (vo - vth, dvo)
        };
        let (t, failed) = find_root(&mut f, t_lower, t_upper, VTH_TIME_TOL, FIND_ROOT_MAX_ITER);
        if failed {
            Err(DmpError("find Vo crossing failed"))
        } else {
            Ok(t)
        }
    }

    /// The output waveform's threshold crossings.
    fn find_driver_delay_slew(&self) -> Result<(f64, f64), DmpError> {
        let t_upper = self.vo_crossing_upper_bound();
        let delay = self.find_vo_crossing(self.vth, self.t0, t_upper)?;
        let tl = self.find_vo_crossing(self.vl, self.t0, delay)?;
        let th = self.find_vo_crossing(self.vh, delay, t_upper)?;
        Ok((delay, (th - tl) / self.slew_derate))
    }

    /// The gate delay and the driver slew (both `double`; the caller narrows).
    pub fn gate_delay_slew(&mut self) -> (f64, f64) {
        match self.alg {
            Alg::Cap => {
                let (delay, slew) = self.gate_cap_delay_slew(self.ceff);
                self.drvr_slew = slew;
                (delay, slew)
            }
            Alg::Pi => {
                self.driver_valid = false;
                let first = self.find_driver_params(self.c2 + self.c1).or_else(|_| self.find_driver_params(self.c2));
                let (delay, slew) = match first {
                    Ok(()) => {
                        let (table_delay, table_slew) = self.gate_cap_delay_slew(self.ceff);
                        match self.find_driver_delay_slew() {
                            Ok((vo_delay, vo_slew)) => {
                                self.driver_valid = true;
                                self.vo_delay = vo_delay;
                                (table_delay, vo_slew)
                            }
                            Err(_) => (table_delay, table_slew),
                        }
                    }
                    Err(_) => {
                        self.ceff = self.c1 + self.c2;
                        self.gate_cap_delay_slew(self.ceff)
                    }
                };
                self.drvr_slew = slew;
                (delay, slew)
            }
            Alg::ZeroC2 => {
                let r = self.find_driver_params(self.c1).and_then(|_| {
                    self.ceff = self.c1;
                    self.find_driver_delay_slew()
                });
                let (delay, slew) = match r {
                    Ok((d, s)) => {
                        self.driver_valid = true;
                        self.vo_delay = d;
                        (d, s)
                    }
                    Err(_) => {
                        self.driver_valid = false;
                        self.ceff = self.c1;
                        self.gate_cap_delay_slew(self.ceff)
                    }
                };
                self.drvr_slew = slew;
                (delay, slew)
            }
        }
    }

    fn vl0(&self, t: f64) -> (f64, f64) {
        let p3 = self.p3;
        match self.alg {
            Alg::Pi => {
                let d1 = self.k0 * (self.k1 - self.k2 / p3);
                let d3 = -p3 * self.k0 * self.k3 / (self.p1 - p3);
                let d4 = -p3 * self.k0 * self.k4 / (self.p2 - p3);
                let d5 = self.k0 * (self.k2 / p3 - self.k1 + p3 * self.k3 / (self.p1 - p3) + p3 * self.k4 / (self.p2 - p3));
                let (e1, e2, e3) = (exp2(-self.p1 * t), exp2(-self.p2 * t), exp2(-p3 * t));
                (d1 + t + d3 * e1 + d4 * e2 + d5 * e3, 1.0 - d3 * self.p1 * e1 - d4 * self.p2 * e2 - d5 * p3 * e3)
            }
            Alg::ZeroC2 => {
                let d1 = self.k0 * (self.k1 - self.k2 / p3);
                let d3 = -p3 * self.k0 * self.k3 / (self.p1 - p3);
                let d5 = self.k0 * (self.k2 / p3 - self.k1 + p3 * self.k3 / (self.p1 - p3));
                let (e1, e3) = (exp2(-self.p1 * t), exp2(-p3 * t));
                (d1 + t + d3 * e1 + d5 * e3, 1.0 - d3 * self.p1 * e1 - d5 * p3 * e3)
            }
            Alg::Cap => (0.0, 0.0),
        }
    }

    fn vl(&self, t: f64) -> (f64, f64) {
        let t1 = t - self.t0;
        if t1 <= 0.0 {
            return (0.0, 0.0);
        }
        if t1 <= self.dt {
            let (v, dv) = self.vl0(t1);
            return (v / self.dt, dv / self.dt);
        }
        let (v, dv) = self.vl0(t1);
        let (v_dt, dv_dt) = self.vl0(t1 - self.dt);
        ((v - v_dt) / self.dt, (dv - dv_dt) / self.dt)
    }

    fn find_vl_crossing(&self, vth: f64, t_lower: f64, t_upper: f64) -> Result<f64, DmpError> {
        let mut f = |t: f64| {
            let (v, dv) = self.vl(t);
            (v - vth, dv)
        };
        let (t, failed) = find_root(&mut f, t_lower, t_upper, VTH_TIME_TOL, FIND_ROOT_MAX_ITER);
        if failed {
            Err(DmpError("find Vl crossing failed"))
        } else {
            Ok(t)
        }
    }

    /// The wire delay and load slew at a load with this Elmore delay (capacitive kind: the Elmore
    /// delay and the driver slew).
    pub fn load_delay_slew(&mut self, elmore: f64) -> (f64, f64) {
        if self.alg == Alg::Cap {
            return (elmore, self.drvr_slew);
        }
        if !self.driver_valid || elmore == 0.0 || elmore < self.drvr_slew * 1e-3 {
            return (elmore, self.drvr_slew);
        }
        self.elmore = elmore;
        self.p3 = 1.0 / elmore;
        let r = (|| -> Result<(f64, f64), DmpError> {
            let t_lower = self.t0;
            let t_upper = self.vo_crossing_upper_bound() + self.elmore * 2.0;
            let load_delay = self.find_vl_crossing(self.vth, t_lower, t_upper)?;
            let tl = self.find_vl_crossing(self.vl, t_lower, load_delay)?;
            let th = self.find_vl_crossing(self.vh, load_delay, t_upper)?;
            let mut delay = load_delay - self.vo_delay;
            let mut slew = (th - tl) / self.slew_derate;
            if delay < 0.0 {
                delay = elmore;
            }
            if slew < self.drvr_slew {
                slew = self.drvr_slew;
            }
            Ok((delay, slew))
        })();
        r.unwrap_or((self.elmore, self.drvr_slew))
    }
}

/// An input port's wire delay and load slew from the load's Elmore delay,
/// at the LOAD library's thresholds (`vth` its input threshold).
pub fn dspf_wire_delay_slew(drvr_slew: f64, elmore: f32, load: &Thresholds) -> (f64, f64) {
    let (vth, vl, vh, derate) = (load.vth, load.vl, load.vh, load.slew_derate);
    let wire_delay = f64::from(-elmore) * (1.0 - f64::from(vth)).ln();
    let load_slew = drvr_slew + f64::from(elmore) * ((1.0 - f64::from(vl)) / (1.0 - f64::from(vh))).ln() / f64::from(derate);
    (wire_delay, load_slew)
}

/// A library's thresholds for [`threshold_adjust`]: output and input threshold, slew lower and
/// upper threshold, slew derate — for one transition.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LibThresholds {
    pub output: f32,
    pub input: f32,
    pub lower: f32,
    pub upper: f32,
    pub derate: f32,
}

/// When the load's library is not the driver's, move the wire delay and scale
/// the load slew between their thresholds (`rise` adds the delta, `fall` subtracts it), in `f32`
/// factors on the `double` values.
pub fn threshold_adjust(same_library: bool, drvr: &LibThresholds, load: &LibThresholds, rise: bool, wire_delay: &mut f64, load_slew: &mut f64) {
    if same_library {
        return;
    }
    let drvr_slew_delta = drvr.upper - drvr.lower;
    let wire_delay_delta = (*load_slew as f32) * ((load.input - drvr.output) / drvr_slew_delta);
    *wire_delay += if rise { f64::from(wire_delay_delta) } else { f64::from(-wire_delay_delta) };
    let load_slew_delta = load.upper - load.lower;
    *load_slew *= f64::from((load_slew_delta / load.derate) / (drvr_slew_delta / drvr.derate));
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rule: (1 + x/4096)^(2^12), 0 below −12.
    #[test]
    fn exp2_is_the_repeated_square() {
        assert_eq!(exp2(-13.0), 0.0);
        let mut y = 1.0 - 1.0 / 4096.0;
        for _ in 0..12 {
            y *= y;
        }
        assert_eq!(exp2(-1.0), y);
        assert!((exp2(-1.0) - (-1.0f64).exp()).abs() < 1e-3, "an approximation of exp");
    }

    // The closed forms solve the system.
    #[test]
    fn closed_form_solves() {
        let m: M3 = [[2.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 4.0]];
        let f = [1.0, 2.0, 3.0];
        let p = solve3(&m, f);
        for i in 0..3 {
            let r = m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + f[i];
            assert!(r.abs() < 1e-12);
        }
        let p = solve2(&m, f);
        for i in 0..2 {
            assert!((m[i][0] * p[0] + m[i][1] * p[1] + f[i]).abs() < 1e-12);
        }
    }

    // Rule: a bracket that does not straddle a root fails; the root of a
    // line is found.
    /// Rule: rows 0–1 sum left to right, row 2 sums its
    /// last two terms first. With 1 + two sub-half-ulp terms, the orders give 1 and 1 + ulp.
    #[test]
    fn the_scalar_tail_row_sums_its_last_two_terms_first() {
        let t = f64::EPSILON * 0.5 * 0.99;
        let m = [[1.0; 3]; 3];
        let p = lazy_product3(&m, [1.0, t, t]);
        assert_eq!(p[0], 1.0);
        assert_eq!(p[1], 1.0);
        assert_eq!(p[2], 1.0 + f64::EPSILON);
    }

    #[test]
    fn find_root_brackets() {
        let mut f = |x: f64| (x - 0.3, 1.0);
        let (r, failed) = find_root(&mut f, 0.0, 1.0, 1e-9, 20);
        assert!(!failed && (r - 0.3).abs() < 1e-9);
        let mut g = |x: f64| (x + 5.0, 1.0);
        assert!(find_root(&mut g, 0.0, 1.0, 1e-9, 20).1);
    }
}
