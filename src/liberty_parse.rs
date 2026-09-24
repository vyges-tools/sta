// SPDX-License-Identifier: Apache-2.0
//! Liberty syntax: groups, simple attributes and complex attributes, every value kept as TEXT.
//!
//! Rules:
//! - an unquoted token is a number when the WHOLE token matches the float pattern
//!   (`[-+]?((D+\.?D*)|(\.D+))([Ee][-+]?D+)?`); anything else — `1ns`, `a1`, `IQ` — is a word;
//! - a number is read with `strtof` and kept as the SHORTEST text that round-trips that float,
//!   which later reads parse back to the same `f32` — so a number here is `text.parse::<f32>()`;
//! - an attribute value is an expression (`A & B'`, `!X`) flattened back to text; a quoted string
//!   is kept verbatim (a `\` line continuation inside it removed, other escapes kept);
//! - `/* … */` comments, blanks and `\` line continuations between tokens are skipped.

/// A value as the reader keeps it.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// A number token, already read as `f32`.
    Float(f32),
    /// Anything else, as text (quoted strings without their quotes).
    Str(String),
}

impl Value {
    /// A number token, or text that parses whole as one.
    pub fn float(&self) -> Option<f32> {
        match self {
            Value::Float(v) => Some(*v),
            Value::Str(s) => s.parse::<f32>().ok(),
        }
    }
    pub fn text(&self) -> String {
        match self {
            Value::Float(v) => format!("{v}"),
            Value::Str(s) => s.clone(),
        }
    }
}

/// A group: `type (params) { … }`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Group {
    pub kind: String,
    pub params: Vec<Value>,
    /// `name : value ;`, in file order.
    pub simple: Vec<(String, Value)>,
    /// `name (values) ;`, in file order.
    pub complex: Vec<(String, Vec<Value>)>,
    pub groups: Vec<Group>,
    pub line: usize,
}

impl Group {
    pub fn attr(&self, name: &str) -> Option<&Value> {
        self.simple.iter().rev().find(|(n, _)| n == name).map(|(_, v)| v)
    }
    pub fn attr_float(&self, name: &str) -> Option<f32> {
        self.attr(name).and_then(Value::float)
    }
    pub fn attr_text(&self, name: &str) -> Option<String> {
        self.attr(name).map(Value::text)
    }
    pub fn complex_attr(&self, name: &str) -> Option<&Vec<Value>> {
        self.complex.iter().find(|(n, _)| n == name).map(|(_, v)| v)
    }
    pub fn groups_of<'a>(&'a self, kind: &'a str) -> impl Iterator<Item = &'a Group> + 'a {
        self.groups.iter().filter(move |g| g.kind == kind)
    }
    pub fn name(&self) -> Option<String> {
        self.params.first().map(Value::text)
    }
}

#[derive(Debug, Clone, PartialEq)]
enum Tok {
    Float(f32),
    Word(String),
    Quoted(String),
    Punct(char),
}

fn is_float_text(t: &str) -> bool {
    let b = t.as_bytes();
    let mut i = 0;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let d0 = i;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    let int_digits = i - d0;
    if int_digits > 0 {
        if i < b.len() && b[i] == b'.' {
            i += 1;
            while i < b.len() && b[i].is_ascii_digit() {
                i += 1;
            }
        }
    } else {
        if !(i < b.len() && b[i] == b'.') {
            return false;
        }
        i += 1;
        let f0 = i;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
        }
        if i == f0 {
            return false;
        }
    }
    if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
        let e = i;
        i += 1;
        if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
            i += 1;
        }
        let x0 = i;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
        }
        if i == x0 {
            i = e;
        }
    }
    i == b.len()
}

const PUNCT: &str = ",:;|(){}+*&!'=";

fn tokenize(text: &str) -> Result<Vec<(Tok, usize)>, String> {
    let c: Vec<char> = text.chars().collect();
    let (mut i, mut line, mut out) = (0usize, 1usize, Vec::new());
    while i < c.len() {
        let ch = c[i];
        if ch == '\n' {
            line += 1;
            i += 1;
        } else if ch == ' ' || ch == '\t' || ch == '\r' || (ch == '\\' && i + 1 < c.len() && (c[i + 1] == '\n' || c[i + 1] == '\r')) {
            // Blank, or a line continuation (its newline is counted next).
            i += 1;
        } else if ch == '/' && i + 1 < c.len() && c[i + 1] == '*' {
            i += 2;
            while i + 1 < c.len() && !(c[i] == '*' && c[i + 1] == '/') {
                if c[i] == '\n' {
                    line += 1;
                }
                i += 1;
            }
            i += 2;
        } else if ch == '/' && i + 1 < c.len() && c[i + 1] == '/' {
            while i < c.len() && c[i] != '\n' {
                i += 1;
            }
        } else if ch == '"' {
            i += 1;
            let mut s = String::new();
            while i < c.len() && c[i] != '"' {
                if c[i] == '\\' && i + 1 < c.len() {
                    if c[i + 1] == '\n' {
                        line += 1;
                        i += 2;
                        continue;
                    }
                    if c[i + 1] == '\r' && i + 2 < c.len() && c[i + 2] == '\n' {
                        line += 1;
                        i += 3;
                        continue;
                    }
                    s.push('\\');
                    s.push(c[i + 1]);
                    i += 2;
                    continue;
                }
                if c[i] == '\n' {
                    return Err(format!("line {line}: unterminated string constant"));
                }
                s.push(c[i]);
                i += 1;
            }
            i += 1;
            out.push((Tok::Quoted(s), line));
        } else if PUNCT.contains(ch) && !(ch == '+' || ch == '-') || (ch == '+' && !(i + 1 < c.len() && (c[i + 1].is_ascii_digit() || c[i + 1] == '.'))) {
            out.push((Tok::Punct(ch), line));
            i += 1;
        } else {
            // A bare token runs to the next punctuation (a leading sign aside), blank or newline.
            let start = i;
            i += 1;
            while i < c.len() {
                let d = c[i];
                if d == ' ' || d == '\t' || d == '\r' || d == '\n' || (PUNCT.contains(d) && d != '+') || d == '"' {
                    break;
                }
                // `+` ends a word but belongs to an exponent (`1e+3`).
                if d == '+' && !matches!(c[i - 1], 'e' | 'E') {
                    break;
                }
                if d == '\\' && i + 1 < c.len() && (c[i + 1] == '\n' || c[i + 1] == '\r') {
                    break;
                }
                i += 1;
            }
            let t: String = c[start..i].iter().collect();
            if is_float_text(&t) {
                let v = t.parse::<f32>().map_err(|e| format!("line {line}: {t}: {e}"))?;
                out.push((Tok::Float(v), line));
            } else {
                out.push((Tok::Word(t), line));
            }
        }
    }
    Ok(out)
}

struct Parser {
    toks: Vec<(Tok, usize)>,
    pos: usize,
}

impl Parser {
    fn peek(&self) -> Option<&Tok> {
        self.toks.get(self.pos).map(|(t, _)| t)
    }
    fn line(&self) -> usize {
        self.toks.get(self.pos).map_or(0, |(_, l)| *l)
    }
    fn next(&mut self) -> Option<Tok> {
        let t = self.toks.get(self.pos).map(|(t, _)| t.clone());
        self.pos += 1;
        t
    }
    fn is_punct(&self, p: char) -> bool {
        matches!(self.peek(), Some(Tok::Punct(c)) if *c == p)
    }
    fn semi_opt(&mut self) {
        if self.is_punct(';') {
            self.pos += 1;
        }
    }

    /// `expr` flattened to one value: a single number stays a number; anything longer is text.
    fn value(&mut self) -> Result<Value, String> {
        let mut parts: Vec<Tok> = Vec::new();
        let mut depth = 0i32;
        loop {
            match self.peek() {
                None => break,
                Some(Tok::Punct(p)) => {
                    let p = *p;
                    if depth == 0 && (p == ',' || p == ';' || p == ')' || p == '{' || p == '}') {
                        break;
                    }
                    if p == '(' {
                        depth += 1;
                    }
                    if p == ')' {
                        depth -= 1;
                    }
                    parts.push(self.next().unwrap());
                }
                Some(Tok::Word(_)) | Some(Tok::Float(_)) | Some(Tok::Quoted(_)) => {
                    // Two adjacent operands are two values (`attr_values attr_value`).
                    if !parts.is_empty() && !matches!(parts.last(), Some(Tok::Punct(p)) if "+|*&^-!(".contains(*p)) {
                        break;
                    }
                    parts.push(self.next().unwrap());
                }
            }
        }
        match parts.as_slice() {
            [Tok::Float(v)] => Ok(Value::Float(*v)),
            [Tok::Quoted(s)] | [Tok::Word(s)] => Ok(Value::Str(s.clone())),
            [] => Err(format!("line {}: expected a value", self.line())),
            _ => Ok(Value::Str(parts.iter().map(|t| match t {
                Tok::Float(v) => format!("{v}"),
                Tok::Word(s) | Tok::Quoted(s) => s.clone(),
                Tok::Punct(p) => p.to_string(),
            }).collect())),
        }
    }

    fn values(&mut self) -> Result<Vec<Value>, String> {
        let mut out = Vec::new();
        while !self.is_punct(')') {
            if self.is_punct(',') {
                self.pos += 1;
                continue;
            }
            out.push(self.value()?);
        }
        self.pos += 1;
        Ok(out)
    }

    fn statements(&mut self, group: &mut Group) -> Result<(), String> {
        while !self.is_punct('}') {
            let line = self.line();
            let name = match self.next() {
                Some(Tok::Word(w)) => w,
                None => return Err("unexpected end of file".into()),
                Some(t) => return Err(format!("line {line}: unexpected {t:?}")),
            };
            match self.next() {
                Some(Tok::Punct(':')) => {
                    let v = self.value()?;
                    self.semi_opt();
                    group.simple.push((name, v));
                }
                Some(Tok::Punct('(')) => {
                    let params = self.values()?;
                    if self.is_punct('{') {
                        self.pos += 1;
                        let mut g = Group { kind: name, params, line, ..Default::default() };
                        self.statements(&mut g)?;
                        self.pos += 1;
                        self.semi_opt();
                        group.groups.push(g);
                    } else {
                        self.semi_opt();
                        group.complex.push((name, params));
                    }
                }
                t => return Err(format!("line {line}: after {name}: unexpected {t:?}")),
            }
        }
        Ok(())
    }
}

/// Parse a liberty file's text into its top-level group (the `library`).
pub fn parse(text: &str) -> Result<Group, String> {
    let toks = tokenize(text)?;
    let mut p = Parser { toks, pos: 0 };
    let mut root = Group::default();
    p.statements_top(&mut root)?;
    root.groups.into_iter().next().ok_or_else(|| "no library group".to_string())
}

impl Parser {
    fn statements_top(&mut self, root: &mut Group) -> Result<(), String> {
        while self.peek().is_some() {
            let line = self.line();
            let name = match self.next() {
                Some(Tok::Word(w)) => w,
                t => return Err(format!("line {line}: unexpected {t:?} at the top level")),
            };
            if !matches!(self.next(), Some(Tok::Punct('('))) {
                return Err(format!("line {line}: {name}: expected a group"));
            }
            let params = self.values()?;
            if !self.is_punct('{') {
                return Err(format!("line {line}: {name}: expected a group body"));
            }
            self.pos += 1;
            let mut g = Group { kind: name, params, line, ..Default::default() };
            self.statements(&mut g)?;
            self.pos += 1;
            self.semi_opt();
            root.groups.push(g);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rule: a number is a WHOLE-token float match, read by strtof; `1ns`,
    // `a1` are words; a signed number is a number; an exponent may carry a sign.
    #[test]
    fn numbers_are_whole_token_floats() {
        for (t, f) in [("0.5", true), ("-1.25e-3", true), ("1e+3", true), (".5", true), ("5.", true), ("1ns", false), ("a1", false), ("e5", false), ("1e", false)] {
            assert_eq!(is_float_text(t), f, "{t}");
        }
        let g = parse("library (x) { time_unit : \"1ns\" ; nom_voltage : 1.8 ; k : -2.5e-1 ; }").unwrap();
        assert_eq!(g.attr("time_unit"), Some(&Value::Str("1ns".into())));
        assert_eq!(g.attr("nom_voltage"), Some(&Value::Float(1.8f32)));
        assert_eq!(g.attr_float("k"), Some(-0.25f32));
    }

    // Groups nest; a complex attribute keeps its values in order; a quoted list stays one text
    // value (its numbers are read later, each with the f32 parse); expressions flatten to text.
    #[test]
    fn groups_complex_attributes_and_expressions() {
        let text = r#"library (lib) {
            capacitive_load_unit (1, pf) ;
            /* a comment */
            cell (INV) {
                pin (Y) { direction : output ; function : "!A" ;
                    timing () { related_pin : "A" ; when : "A&B" ;
                        cell_rise (tmpl) { index_1 ("0.01, 0.02") ; values ("0.1, 0.2", \
                                   "0.3, 0.4") ; }
                    }
                }
            }
        }"#;
        let g = parse(text).unwrap();
        assert_eq!(g.complex_attr("capacitive_load_unit"), Some(&vec![Value::Float(1.0), Value::Str("pf".into())]));
        let cell = g.groups_of("cell").next().unwrap();
        assert_eq!(cell.name().as_deref(), Some("INV"));
        let pin = cell.groups_of("pin").next().unwrap();
        assert_eq!(pin.attr_text("function").as_deref(), Some("!A"));
        let timing = pin.groups_of("timing").next().unwrap();
        assert_eq!(timing.attr_text("when").as_deref(), Some("A&B"));
        let rise = timing.groups_of("cell_rise").next().unwrap();
        assert_eq!(rise.complex_attr("values").unwrap().len(), 2);
    }
}
