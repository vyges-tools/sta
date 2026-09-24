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
    /// `(name, cell)`.
    pub insts: Vec<(String, String)>,
    pub ports: Vec<(String, PortDir)>,
    pub nets: Vec<Net>,
}

impl Netlist {
    /// A pin's path name: `inst/port` or the port's name.
    pub fn pin_name(&self, c: &Conn) -> String {
        match c {
            Conn::Inst(i, p) => format!("{}/{p}", self.insts[*i].0),
            Conn::Port(i) => self.ports[*i].0.clone(),
        }
    }
}
