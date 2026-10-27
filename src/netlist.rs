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
#[derive(Debug, Clone, PartialEq, Eq)]
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
            Conn::Inst(i, p) => format!("{}/{p}", self.insts[*i].0),
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
}
