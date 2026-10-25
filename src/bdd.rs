// SPDX-License-Identifier: Apache-2.0
//! Reduced ordered binary decision diagrams in the canonical complement-edge form, as the
//! reference's power analysis builds a liberty function's diagram to weigh its inputs.
//!
//! Rules:
//! - an edge is a node and a complement bit; there is ONE constant node, the logic one, and zero
//!   is its complement;
//! - a node's THEN edge is never complemented: a node that would have one is made with both
//!   children complemented and returned complemented. With a fixed variable order this makes every
//!   function's diagram unique, so the diagram (and an evaluation that walks it) does not depend
//!   on how the function was built;
//! - variables are numbered in order of first appearance as the expression is walked (a port,
//!   then a negation's operand, then a binary operator's left then right operand), and the
//!   number is the level: there is no reordering;
//! - an operator with one side missing is the other side (an expression over no ports is the
//!   constant it names).

use std::collections::HashMap;

use crate::func_expr::FuncExpr;

/// A node index shifted left once, with the complement in the low bit.
pub type Edge = u32;

/// The logic one (the constant node, regular).
pub const ONE: Edge = 0;
/// The logic zero (the constant node, complemented).
pub const ZERO: Edge = 1;

pub fn not(e: Edge) -> Edge {
    e ^ 1
}

pub fn is_complement(e: Edge) -> bool {
    e & 1 == 1
}

fn regular(e: Edge) -> Edge {
    e & !1
}

const CONSTANT_VAR: u32 = u32::MAX;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum Op {
    And,
    Xor,
}

/// One diagram manager: its nodes, its unique table and its variables (port names, by index).
#[derive(Debug, Clone)]
pub struct Bdd {
    /// `(variable, then, else)`; node 0 is the constant.
    nodes: Vec<(u32, Edge, Edge)>,
    unique: HashMap<(u32, Edge, Edge), u32>,
    cache: HashMap<(Op, Edge, Edge), Edge>,
    vars: Vec<String>,
}

impl Default for Bdd {
    fn default() -> Self {
        Bdd { nodes: vec![(CONSTANT_VAR, ONE, ONE)], unique: HashMap::new(), cache: HashMap::new(), vars: Vec::new() }
    }
}

impl Bdd {
    /// The diagram of `expr`, its variables numbered as they first appear.
    pub fn of(expr: &FuncExpr) -> (Bdd, Edge) {
        let mut b = Bdd::default();
        let f = b.func(expr).unwrap_or(ONE);
        (b, f)
    }

    fn func(&mut self, expr: &FuncExpr) -> Option<Edge> {
        match expr {
            FuncExpr::Port(p) => Some(self.ensure_var(p)),
            FuncExpr::Not(a) => self.func(a).map(not),
            FuncExpr::Or(a, b) => {
                let (l, r) = (self.func(a), self.func(b));
                match (l, r) {
                    (Some(l), Some(r)) => Some(self.or(l, r)),
                    _ => l.or(r),
                }
            }
            FuncExpr::And(a, b) => {
                let (l, r) = (self.func(a), self.func(b));
                match (l, r) {
                    (Some(l), Some(r)) => Some(self.and(l, r)),
                    _ => l.or(r),
                }
            }
            FuncExpr::Xor(a, b) => {
                let (l, r) = (self.func(a), self.func(b));
                match (l, r) {
                    (Some(l), Some(r)) => Some(self.xor(l, r)),
                    _ => l.or(r),
                }
            }
            FuncExpr::One => Some(ONE),
            FuncExpr::Zero => Some(ZERO),
        }
    }

    /// The variable named `port`, made at the next index the first time it is asked for.
    pub fn ensure_var(&mut self, port: &str) -> Edge {
        let v = match self.vars.iter().position(|p| p == port) {
            Some(v) => v as u32,
            None => {
                self.vars.push(port.to_string());
                (self.vars.len() - 1) as u32
            }
        };
        self.mk(v, ONE, ZERO)
    }

    /// The variables in index order.
    pub fn vars(&self) -> &[String] {
        &self.vars
    }

    pub fn var_index(&self, port: &str) -> Option<u32> {
        self.vars.iter().position(|p| p == port).map(|v| v as u32)
    }

    /// The edge's node's variable (`u32::MAX` for the constant).
    pub fn var(&self, e: Edge) -> u32 {
        self.nodes[(e >> 1) as usize].0
    }

    pub fn is_constant(&self, e: Edge) -> bool {
        self.var(e) == CONSTANT_VAR
    }

    /// The REGULAR node's then child (the edge's complement is not applied).
    pub fn then_child(&self, e: Edge) -> Edge {
        self.nodes[(e >> 1) as usize].1
    }

    /// The REGULAR node's else child (the edge's complement is not applied).
    pub fn else_child(&self, e: Edge) -> Edge {
        self.nodes[(e >> 1) as usize].2
    }

    fn mk(&mut self, v: u32, t: Edge, e: Edge) -> Edge {
        if t == e {
            return t;
        }
        if is_complement(t) {
            return not(self.mk(v, not(t), not(e)));
        }
        if let Some(&n) = self.unique.get(&(v, t, e)) {
            return n << 1;
        }
        let n = self.nodes.len() as u32;
        self.nodes.push((v, t, e));
        self.unique.insert((v, t, e), n);
        n << 1
    }

    /// The function's cofactors on `v`: (`v` = 1, `v` = 0), the edge's complement applied.
    fn cofactors(&self, f: Edge, v: u32) -> (Edge, Edge) {
        if self.var(f) != v {
            return (f, f);
        }
        let c = f & 1;
        (self.then_child(f) ^ c, self.else_child(f) ^ c)
    }

    fn apply(&mut self, op: Op, a: Edge, b: Edge) -> Edge {
        match op {
            Op::And => {
                if a == ZERO || b == ZERO || a == not(b) {
                    return ZERO;
                }
                if a == ONE || a == b {
                    return b;
                }
                if b == ONE {
                    return a;
                }
            }
            Op::Xor => {
                if a == b {
                    return ZERO;
                }
                if a == not(b) {
                    return ONE;
                }
                if a == ZERO {
                    return b;
                }
                if b == ZERO {
                    return a;
                }
                if a == ONE {
                    return not(b);
                }
                if b == ONE {
                    return not(a);
                }
            }
        }
        let key = if a <= b { (op, a, b) } else { (op, b, a) };
        if let Some(&r) = self.cache.get(&key) {
            return r;
        }
        let v = self.var(a).min(self.var(b));
        let (a1, a0) = self.cofactors(a, v);
        let (b1, b0) = self.cofactors(b, v);
        let t = self.apply(op, a1, b1);
        let e = self.apply(op, a0, b0);
        let r = self.mk(v, t, e);
        self.cache.insert(key, r);
        r
    }

    pub fn and(&mut self, a: Edge, b: Edge) -> Edge {
        self.apply(Op::And, a, b)
    }

    pub fn or(&mut self, a: Edge, b: Edge) -> Edge {
        not(self.apply(Op::And, not(a), not(b)))
    }

    pub fn xor(&mut self, a: Edge, b: Edge) -> Edge {
        self.apply(Op::Xor, a, b)
    }

    /// The function with variable `v` set to `value`.
    pub fn restrict(&mut self, f: Edge, v: u32, value: bool) -> Edge {
        if self.is_constant(f) || self.var(f) > v {
            return f;
        }
        let (f1, f0) = self.cofactors(f, v);
        if self.var(f) == v {
            return if value { f1 } else { f0 };
        }
        let c = f & 1;
        let (t, e) = (self.then_child(f), self.else_child(f));
        let t = self.restrict(t, v, value);
        let e = self.restrict(e, v, value);
        let top = self.var(f);
        self.mk(top, t, e) ^ c
    }

    /// The Boolean difference on `v`: where the function is sensitive to it, `f|v=1 xor f|v=0`.
    pub fn boolean_diff(&mut self, f: Edge, v: u32) -> Edge {
        let f1 = self.restrict(f, v, true);
        let f0 = self.restrict(f, v, false);
        self.xor(f1, f0)
    }

    /// Whether the edge is the regular form of its node (for tests and walkers).
    pub fn is_regular(e: Edge) -> bool {
        e == regular(e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bdd(s: &str) -> (Bdd, Edge) {
        Bdd::of(&FuncExpr::parse(s).unwrap())
    }

    // No stored node has a complemented then edge.
    #[test]
    fn then_edges_are_never_complemented() {
        let (b, _) = bdd("!(A & B) ^ (C | !A)");
        assert!(b.nodes.iter().skip(1).all(|&(_, t, _)| Bdd::is_regular(t)));
    }

    // The diagram is canonical: the same function built two ways is the same edge.
    #[test]
    fn equal_functions_are_one_edge() {
        let mut b = Bdd::default();
        let (a, c) = (b.ensure_var("A"), b.ensure_var("B"));
        let x = b.and(a, c);
        let y = not(b.or(not(a), not(c)));
        assert_eq!(x, y);
        let x = b.xor(a, c);
        let n = b.and(a, not(c));
        let m = b.and(not(a), c);
        assert_eq!(x, b.or(n, m));
    }

    // Variables are numbered by first appearance: the left operand before the right.
    #[test]
    fn variables_number_in_order_of_appearance() {
        let (b, _) = bdd("(B & !A) | C");
        assert_eq!(b.vars(), ["B", "A", "C"]);
    }

    // A variable: then one, else zero, regular.
    #[test]
    fn a_variable_is_then_one_else_zero() {
        let (b, f) = bdd("A");
        assert_eq!((b.then_child(f), b.else_child(f), is_complement(f)), (ONE, ZERO, false));
        let (b, f) = bdd("!A");
        assert_eq!((b.then_child(f), b.else_child(f), is_complement(f)), (ONE, ZERO, true));
    }

    // d(A&B)/dA = B; d(A^B)/dA = 1; a variable outside the support has difference 0.
    #[test]
    fn boolean_differences() {
        let (mut b, f) = bdd("A & B");
        let (va, vb) = (b.var_index("A").unwrap(), b.var_index("B").unwrap());
        let bv = b.mk(vb, ONE, ZERO);
        assert_eq!(b.boolean_diff(f, va), bv);
        let (mut b, f) = bdd("A ^ B");
        assert_eq!(b.boolean_diff(f, 0), ONE);
        let (mut b, f) = bdd("A");
        let _ = b.ensure_var("Z");
        assert_eq!(b.boolean_diff(f, 1), ZERO);
    }

    // Constants: `1` is one, `0` is zero, a missing side takes the other.
    #[test]
    fn constants() {
        assert_eq!(bdd("1").1, ONE);
        assert_eq!(bdd("0").1, ZERO);
        let (_, f) = bdd("A | 1");
        assert_eq!(f, ONE);
    }
}
