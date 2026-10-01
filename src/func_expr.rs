// SPDX-License-Identifier: Apache-2.0
//! Liberty `function` expressions as the reference's expression parser builds them, and its
//! STRUCTURAL equivalence (`FuncExpr::equiv`): two cells are interchangeable when their functions
//! parse to the same tree over the same port names — the tree's shape is part of the answer.
//!
//! The grammar (`LibExprParse.yy`):
//! - `terminal`: a port, `0`, `1`, or `( expr )`;
//! - `terminal_expr`: a terminal, `!` terminal, or terminal `'` — negation applies to a TERMINAL;
//! - `implicit_and`: two or more terminal_exprs side by side, folded LEFT into ANDs;
//! - `expr`: a terminal_expr, an implicit_and, or `expr op expr` with every operator left
//!   associative and, from loosest to tightest, `+` `|` (or), `*` `&` (and), `^` (xor).

/// An expression tree.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FuncExpr {
    Port(String),
    Not(Box<FuncExpr>),
    Or(Box<FuncExpr>, Box<FuncExpr>),
    And(Box<FuncExpr>, Box<FuncExpr>),
    Xor(Box<FuncExpr>, Box<FuncExpr>),
    One,
    Zero,
}

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Port(String),
    Op(char),
}

/// The scanner: single-character operators and parentheses; everything else up to one of them or
/// a blank is a port name (`0` and `1` alone are constants).
fn scan(s: &str) -> Vec<Tok> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let flush = |cur: &mut String, out: &mut Vec<Tok>| {
        if !cur.is_empty() {
            out.push(Tok::Port(std::mem::take(cur)));
        }
    };
    for c in s.chars() {
        match c {
            '!' | '\'' | '+' | '|' | '*' | '&' | '^' | '(' | ')' => {
                flush(&mut cur, &mut out);
                out.push(Tok::Op(c));
            }
            c if c.is_whitespace() => flush(&mut cur, &mut out),
            c => cur.push(c),
        }
    }
    flush(&mut cur, &mut out);
    out
}

struct Parser {
    toks: Vec<Tok>,
    at: usize,
}

/// Binding power of a binary operator (higher binds tighter), all left associative.
fn bp(c: char) -> Option<u8> {
    match c {
        '+' | '|' => Some(1),
        '*' | '&' => Some(2),
        '^' => Some(3),
        _ => None,
    }
}

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.at)
    }

    fn terminal(&mut self) -> Result<FuncExpr, String> {
        match self.toks.get(self.at).cloned() {
            Some(Tok::Port(p)) => {
                self.at += 1;
                Ok(match p.as_str() {
                    "0" => FuncExpr::Zero,
                    "1" => FuncExpr::One,
                    _ => FuncExpr::Port(p),
                })
            }
            Some(Tok::Op('(')) => {
                self.at += 1;
                let e = self.expr(0)?;
                if self.peek() != Some(&Tok::Op(')')) {
                    return Err("missing )".into());
                }
                self.at += 1;
                Ok(e)
            }
            t => Err(format!("unexpected {t:?}")),
        }
    }

    fn starts_terminal_expr(&self) -> bool {
        matches!(self.peek(), Some(Tok::Port(_)) | Some(Tok::Op('(')) | Some(Tok::Op('!')))
    }

    fn terminal_expr(&mut self) -> Result<FuncExpr, String> {
        if self.peek() == Some(&Tok::Op('!')) {
            self.at += 1;
            return Ok(FuncExpr::Not(Box::new(self.terminal()?)));
        }
        let t = self.terminal()?;
        if self.peek() == Some(&Tok::Op('\'')) {
            self.at += 1;
            return Ok(FuncExpr::Not(Box::new(t)));
        }
        Ok(t)
    }

    /// A terminal_expr followed by any juxtaposed ones, folded left into ANDs.
    fn primary(&mut self) -> Result<FuncExpr, String> {
        let mut e = self.terminal_expr()?;
        while self.starts_terminal_expr() {
            let r = self.terminal_expr()?;
            e = FuncExpr::And(Box::new(e), Box::new(r));
        }
        Ok(e)
    }

    fn expr(&mut self, min_bp: u8) -> Result<FuncExpr, String> {
        let mut lhs = self.primary()?;
        while let Some(Tok::Op(c)) = self.peek().cloned() {
            let Some(p) = bp(c) else { break };
            if p <= min_bp {
                break;
            }
            self.at += 1;
            let rhs = self.expr(p)?;
            lhs = match c {
                '+' | '|' => FuncExpr::Or(Box::new(lhs), Box::new(rhs)),
                '*' | '&' => FuncExpr::And(Box::new(lhs), Box::new(rhs)),
                _ => FuncExpr::Xor(Box::new(lhs), Box::new(rhs)),
            };
        }
        Ok(lhs)
    }
}

impl FuncExpr {
    /// Parse a liberty function.
    pub fn parse(s: &str) -> Result<FuncExpr, String> {
        let mut p = Parser { toks: scan(s), at: 0 };
        let e = p.expr(0)?;
        if p.at != p.toks.len() {
            return Err(format!("function `{s}`: trailing input"));
        }
        Ok(e)
    }

    /// `FuncExpr::equiv`: the same operator at every node and the same port names (a port's
    /// direction is compared by the caller, with the port).
    pub fn equiv(a: Option<&FuncExpr>, b: Option<&FuncExpr>) -> bool {
        match (a, b) {
            (None, None) => true,
            (Some(x), Some(y)) => x == y,
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(s: &str) -> FuncExpr {
        FuncExpr::parse(s).unwrap()
    }
    fn port(s: &str) -> Box<FuncExpr> {
        Box::new(FuncExpr::Port(s.into()))
    }

    // Rules (LibExprParse.yy): `^` binds tighter than `&`, `&` tighter than `|`; all left
    // associative; juxtaposition is AND, folded left; `!` and `'` negate a terminal.
    #[test]
    fn precedence_and_associativity_follow_the_reference_grammar() {
        assert_eq!(p("A&B^C"), FuncExpr::And(port("A"), Box::new(FuncExpr::Xor(port("B"), port("C")))));
        assert_eq!(p("A|B&C"), FuncExpr::Or(port("A"), Box::new(FuncExpr::And(port("B"), port("C")))));
        assert_eq!(p("A&B&C"), FuncExpr::And(Box::new(FuncExpr::And(port("A"), port("B"))), port("C")));
        assert_eq!(p("A B C"), p("A&B&C"), "juxtaposition folds left");
        assert_eq!(p("!A"), FuncExpr::Not(port("A")));
        assert_eq!(p("A'"), FuncExpr::Not(port("A")));
        assert_eq!(p("(A)"), FuncExpr::Port("A".into()));
        assert_eq!(p("!(A|B)"), FuncExpr::Not(Box::new(FuncExpr::Or(port("A"), port("B")))));
        assert_eq!(p("1"), FuncExpr::One);
    }

    // Rule (FuncExpr::equiv): structural — `A&B` and `B&A` are NOT equivalent.
    #[test]
    fn equivalence_is_structural() {
        assert!(FuncExpr::equiv(Some(&p("A&B")), Some(&p("(A)&(B)"))));
        assert!(!FuncExpr::equiv(Some(&p("A&B")), Some(&p("B&A"))));
        assert!(FuncExpr::equiv(None, None));
        assert!(!FuncExpr::equiv(Some(&p("A")), None));
    }
}
