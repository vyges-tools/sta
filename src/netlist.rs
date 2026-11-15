// SPDX-License-Identifier: Apache-2.0
//! The flat netlist the timer reads: instances of library cells, top-level ports, and nets with
//! their pins in the network's order.

/// A top-level port's direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortDir {
    Input,
    Output,
    Inout,
}

/// One pin on a net: an instance's terminal, or a top-level port.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum Conn {
    Inst(usize, String),
    Port(usize),
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Net {
    pub name: String,
    /// Instance pins in the net's order, then its ports.
    pub pins: Vec<Conn>,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Netlist {
    /// `(name, cell)`, shared between netlist generations like the nets: an edit appends
    /// instances or replaces one, so the copy a generation makes is one pointer per instance.
    pub insts: Vec<std::sync::Arc<(String, String)>>,
    pub ports: Vec<(String, PortDir)>,
    /// Shared between netlist generations: an edit replaces the nets it touched, so the copy a
    /// generation makes is one pointer per net, not every pin.
    pub nets: Vec<std::sync::Arc<Net>>,
}

/// One [`Netlist::insts`] entry: an instance `name` of library cell `cell`.
pub fn inst(name: impl Into<String>, cell: impl Into<String>) -> std::sync::Arc<(String, String)> {
    std::sync::Arc::new((name.into(), cell.into()))
}

impl Netlist {
    /// A pin's path name: `inst/port` or the port's name.
    pub fn pin_name(&self, c: &Conn) -> String {
        match c {
            Conn::Inst(i, p) => {
                // pushed, not `format!`: the same string without the formatting machinery
                let inst = &self.insts[*i].0;
                let mut name = String::with_capacity(inst.len() + 1 + p.len());
                name.push_str(inst);
                name.push('/');
                name.push_str(p);
                name
            }
            Conn::Port(i) => self.ports[*i].0.clone(),
        }
    }

    /// [`Netlist::pin_name`] written into `buf` (cleared first): a lookup by name per pin of a
    /// net makes no string of its own.
    pub fn pin_name_into<'b>(&self, c: &Conn, buf: &'b mut String) -> &'b str {
        buf.clear();
        match c {
            Conn::Inst(i, p) => {
                buf.push_str(&self.insts[*i].0);
                buf.push('/');
                buf.push_str(p);
            }
            Conn::Port(i) => buf.push_str(&self.ports[*i].0),
        }
        buf
    }

    /// Is [`Netlist::pin_name`] of `c` equal to `s`? Compared piece by piece; no string is made.
    pub fn pin_name_is(&self, c: &Conn, s: &str) -> bool {
        match c {
            Conn::Inst(i, p) => s.strip_prefix(self.insts[*i].0.as_str()).and_then(|r| r.strip_prefix('/')).is_some_and(|r| r == p),
            Conn::Port(i) => self.ports[*i].0 == s,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_pin_name_written_into_a_buffer_is_the_pin_name() {
        let n = Netlist {
            insts: vec![inst("u1", "buf")],
            ports: vec![("clk".into(), PortDir::Input)],
            nets: vec![],
        };
        let mut buf = String::from("left over");
        for c in [Conn::Inst(0, "A".into()), Conn::Port(0)] {
            assert_eq!(n.pin_name_into(&c, &mut buf), n.pin_name(&c));
        }
    }

    /// A comparison by pin answers what comparing the formatted name would, near misses included.
    #[test]
    fn a_pin_name_compared_by_pieces_is_the_pin_name_compared() {
        let n = Netlist {
            insts: vec![inst("u1", "buf"), inst("u1/x", "buf")],
            ports: vec![("clk".into(), PortDir::Input), ("u1/A".into(), PortDir::Input)],
            nets: vec![],
        };
        let pins = [Conn::Inst(0, "A".into()), Conn::Inst(1, "A".into()), Conn::Inst(0, "x/A".into()), Conn::Port(0), Conn::Port(1)];
        let names = ["u1/A", "u1/x/A", "u1", "u1/", "u1A", "clk", "clk/", "", "u1/AB", "u1/x"];
        for c in &pins {
            for s in names {
                assert_eq!(n.pin_name_is(c, s), n.pin_name(c) == s, "{c:?} {s}");
            }
        }
    }
}
