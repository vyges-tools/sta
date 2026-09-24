// SPDX-License-Identifier: Apache-2.0
//! Fuzzy float comparison, which every delay, slew, arrival, required and slack comparison goes
//! through.
//!
//! Rule: two values are equal when bitwise equal; when one is zero, when the other's
//! magnitude is under 1e-15; otherwise when they differ by less than 1e-6 of the larger magnitude.
//! Everything is `float` — the tolerance products are `f32`. `greater`/`less` are strict AND not
//! equal. The relation is not transitive, so a merge over several candidates depends on the order
//! it sees them in.

const FLOAT_EQUAL_TOLERANCE: f32 = 1e-15;

/// Fuzzy equality.
pub fn equal(v1: f32, v2: f32) -> bool {
    if v1 == v2 {
        true
    } else if v1 == 0.0 {
        v2.abs() < FLOAT_EQUAL_TOLERANCE
    } else if v2 == 0.0 {
        v1.abs() < FLOAT_EQUAL_TOLERANCE
    } else {
        (v1 - v2).abs() < 1e-6f32 * v1.abs().max(v2.abs())
    }
}

/// Strictly less and not fuzzily equal.
pub fn less(v1: f32, v2: f32) -> bool {
    v1 < v2 && !equal(v1, v2)
}

/// Strictly greater and not fuzzily equal.
pub fn greater(v1: f32, v2: f32) -> bool {
    v1 > v2 && !equal(v1, v2)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rule: a value within 1e-6 (relative) of the incumbent is NOT greater — a witnessed pair of
    /// wire delays (f32 bits 2b303d9d after 2b303d9a): the second does not replace the first for max.
    #[test]
    fn a_value_within_one_ppm_is_not_greater() {
        let (a, b) = (f32::from_bits(0x2b30_3d9a), f32::from_bits(0x2b30_3d9d));
        assert!(b > a);
        assert!(!greater(b, a));
        assert!(!less(a, b));
        assert!(equal(a, b));
    }

    /// Relative 1e-6 of the LARGER magnitude, strictly.
    #[test]
    fn the_tolerance_is_relative_and_strict() {
        assert!(greater(1.0 + 2e-6, 1.0));
        assert!(!greater(1.0 + 5e-7, 1.0));
    }

    /// Against zero the tolerance is absolute 1e-15.
    #[test]
    fn zero_uses_an_absolute_tolerance() {
        assert!(equal(0.0, 1e-16));
        assert!(!equal(0.0, 1e-14));
        assert!(greater(1e-14, 0.0));
        assert!(!greater(1e-16, 0.0));
    }

    /// The initial values of a min/max merge (±1e30) are beaten by any ordinary value.
    #[test]
    fn init_values_are_beaten() {
        assert!(greater(1e-12, -1e30));
        assert!(less(1e-12, 1e30));
    }
}
