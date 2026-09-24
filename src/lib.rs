// SPDX-License-Identifier: Apache-2.0
//! Static timing analysis with fully specified arithmetic: every value is narrowed at a defined
//! point (`float` vs `double`), tables interpolate in a defined order, and every merge visits its
//! candidates in a defined order. The results are reproducible bit for bit, which is what a
//! timing-driven router needs — it orders nets by slack, and slack gaps are far smaller than any
//! approximation's error.
//!
//! The pipeline, in data-flow order:
//! - [`liberty_parse`]: liberty syntax, every value kept as the reader keeps it.
//! - [`liberty`]: the library — units, cells, ports.
//! - [`table`]: NLDM tables and their lookup.
//! - [`dcalc`]: gate and wire delay (DMP effective capacitance, Elmore).
//! - [`netlist`], [`graph`]: the timing graph and delay calculation over it.
//! - [`parasitics`]: a net's RC network reduced to a pi model and per-load Elmore delays.
//! - [`sdc`], [`search`]: the clock and port delays; arrivals, requireds and slacks.

pub mod dcalc;
pub mod fuzzy;
pub mod graph;
pub mod liberty;
pub mod liberty_parse;
pub mod netlist;
pub mod parasitics;
pub mod sdc;
pub mod search;
pub mod table;
