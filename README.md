# vyges-sta

Static timing analysis with fully specified arithmetic. Every value is narrowed at a defined
point (`float` or `double`), every table interpolates in a defined order, and every merge visits
its candidates in a defined order — so delays, arrivals, requireds and slacks are reproducible
bit for bit, run to run and machine to machine.

That matters to a timing-driven router: it orders nets by slack, and the gaps between slacks are
far smaller than any approximation's error. An approximate timer reorders the nets.

The library, in data-flow order:

- `liberty_parse`, `liberty` — Liberty syntax, units, cells, pins, timing arcs (combinational,
  clock-to-Q, setup/hold, recovery/removal, preset/clear), NLDM tables and templates.
- `table` — NLDM lookup: bisection, end-interval extrapolation, interpolation in `double`.
- `parasitics` — a net's RC network reduced to a pi model and per-load Elmore delays.
- `dcalc` — gate delay and slew by DMP effective capacitance on the pi model; wire delay and load
  slew from Elmore; threshold adjustment between libraries.
- `netlist`, `graph` — the timing graph and delay calculation over it (min/max, rise/fall).
- `sdc`, `search` — one propagated clock with input/output delays: arrivals, requireds from setup
  checks and output delays, common-path pessimism removal (CRPR), vertex and net slacks.
- `fuzzy` — the tolerant comparison every merge uses.

**Stability: experimental (v0.1.0).** The scope is what timing-driven global routing reads: NLDM
libraries, one clock, setup requireds. Not yet: hold requireds, multiple or generated clocks,
signal integrity. Bidirect pins, multiply-driven nets and latch data arcs are refused rather than
timed wrong.

## Build

No dependencies; nothing but Rust.

```sh
cargo test --release
```

Licensed under Apache-2.0.
