# vyges-sta

Static timing analysis with fully specified arithmetic. Every value is narrowed at a defined
point (`float` or `double`), every table interpolates in a defined order, and every merge visits
its candidates in a defined order — so delays, arrivals, requireds and slacks are reproducible
bit for bit, run to run and machine to machine.

That matters to the engines that optimize against timing: a router orders nets by slack, a
resizer picks the next path by it, and the gaps between slacks are far smaller than any
approximation's error. An approximate timer reorders the work.

The reference whose arithmetic this crate reproduces is OpenSTA, as the OpenROAD project ships it
(the version the in-repo correlation was regenerated against is in
`tests/data/correlation/reference.txt`).

## What it times

- NLDM libraries: combinational, clock-to-Q, setup/hold, recovery/removal and preset/clear arcs,
  tristate enables, bus and bundle pins.
- Several clocks, each on one port or pin, ideal or propagated, with waveforms and pin network
  latency; virtual clocks for I/O delays.
- Setup and hold requireds, recovery/removal, gated-clock checks, latches with time borrowing,
  common-path pessimism removal (CRPR).
- Input and output delays (min/max, rise/fall, `-add_delay`), input transitions, port and net
  loads, driving cells, a path delay (`set_max_delay` / `set_min_delay`).
- Clock constraints: `set_clock_uncertainty` on clocks and between clocks,
  `set_clock_transition` on ideal clocks, `set_clock_latency` on clocks (network, and `-source`
  insertion with early/late), `set_propagated_clock`, and `set_clock_groups` (`-asynchronous`,
  `-logically_exclusive`, `-physically_exclusive`).
- Timing exceptions: `set_false_path` and `set_multicycle_path` (`-setup` / `-hold`, `-start` /
  `-end`) from and to ports and pins, with the reference's priority between exceptions.
- `set_timing_derate`, design-wide (early/late, cell delay, net delay and cell checks, clock and
  data).
- Logic constants, propagated through the cells' functions as the reference's simulation does:
  pins tied to `1'b0` / `1'b1`, `supply0` / `supply1` nets, tie cells, `set_case_analysis`
  (including `rising` / `falling`, which keep one transition) and `set_logic_*`; conditional
  arcs and arc senses under them.
- Detailed parasitics from SPEF (`spef`, `*D_NET` RC networks with name maps, corner triplets and
  coupling capacitors), read as the reference's `read_spef` reads them by default.
- Gate delay by DMP effective capacitance on a pi model, wire delay by Elmore, or ideal wires
  when no parasitics are given.
- Incremental timing across netlist edits, the k worst paths to an endpoint, and switching
  activity and instance power.

Bidirect pins on a net, multiply-driven nets and anything a caller hands it that it does not
model are refused with a reason, never timed wrong.

## SDC commands

SDC is read by [loom](https://github.com/vyges-tools/loom) and mapped into this timer by `load`.
Each command lands in exactly one of three groups.

**Applied** — the slacks move with it:

| command | notes |
| --- | --- |
| `create_clock` | on ports or pins, with `-waveform`; virtual clocks |
| `create_generated_clock` | of an ideal master: `-divide_by`, `-multiply_by` (`-duty_cycle`), `-edges` (`-edge_shift`), `-invert`, `-combinational`, `-master_clock`, `-add`; the waveform derived from the master's |
| `set_propagated_clock` | |
| `set_clock_latency` | network latency on clocks and pins; `-source` insertion, `-early` / `-late` |
| `set_clock_uncertainty` | on clocks, and between clocks (`-from` / `-to`), setup and hold |
| `set_clock_transition` | ideal clocks |
| `set_clock_groups` | `-asynchronous`, `-logically_exclusive`, `-physically_exclusive` |
| `set_input_delay`, `set_output_delay` | min/max, rise/fall, `-add_delay` |
| `set_input_transition`, `set_load`, `set_driving_cell` | |
| `set_max_delay`, `set_min_delay` | `-from` / `-to` pins or the clock, `-through` |
| `set_false_path` | `-from` / `-to` ports and pins, `-through` pins, ports, nets and cells, `-setup` / `-hold` |
| `set_multicycle_path` | `-setup` / `-hold`, `-start` / `-end`, `-through` as for false paths |
| `set_timing_derate` | design-wide: `-early` / `-late`, cell delay, net delay, cell check, clock and data |
| `set_case_analysis` | `0`, `1`, `zero`, `one`, `rising`, `falling`, on pins and ports (read with `load::design`) |
| `set_logic_zero`, `set_logic_one`, `set_logic_dc` | on pins and ports (read with `load::design`) |

**Read, not checked** — kept and reported as notes; no slack moves with them: `set_max_fanout`,
`set_max_transition`, `set_max_capacitance`, `set_min_capacitance`, and directives a timer never
consults (`set_dont_use`, `set_dont_touch`, `set_max_area`, `set_max_dynamic_power`,
`set_max_leakage_power`, voltage areas, clock-gating style, …). A command loom keeps as written
follows the same rule: one of these is a note with its count (`set_max_leakage_power (2
commands): kept and written back, not applied (no slack moves with it)`), any other is refused.

**Refused** — the load stops and names the command, rather than time the design without it:

- `create_generated_clock` of a propagated master, or itself propagated (its source latency is
  the master's arrival through the circuit), or whose master is not defined on its `-source` pin
- `set_disable_timing`, `set_sense` / `set_clock_sense`, `set_disable_clock_gating_check`
- `group_path`, `set_path_margin`
- exceptions given `-rise_through` / `-fall_through`, from or to a clock with several clocks
  defined, on an internal pin, or on a design with latches; multicycle paths on a design with
  gated-clock checks
- any other command that could move a slack (the rule is in `load::constraints`: a command loom
  carries in no record of its own is refused unless it is one of the directives above)

These commands are the next to move from refused to applied: generated clocks of propagated masters, disabled timing
(already modelled in [sta-si](https://github.com/vyges-tools/sta-si)), and latches.

A design with pins tied to a constant or supply nets is read with `load::design` and its
constraints with `load::design_constraints`, which carry the constants to the search; `load::netlist`
refuses such a design rather than time it without them.

## Modules, in data-flow order

- `liberty_parse`, `liberty` — Liberty syntax, units, cells, pins, arcs, NLDM tables.
- `func_expr`, `bdd` — Liberty functions as the reference parses them, their equivalence, and
  their decision diagrams.
- `table` — NLDM lookup: bisection, end-interval extrapolation, interpolation in `double`.
- `parasitics` — a net's RC network reduced to a pi model and per-load Elmore delays.
- `spef` — a detailed SPEF read into those networks.
- `dcalc` — gate delay and slew (DMP effective capacitance); wire delay and load slew; threshold
  adjustment between libraries.
- `netlist`, `graph` (with `graph::edit`) — the timing graph, delay calculation over it, and
  in-place edits from netlist changes.
- `sim` — the logic constants and their propagation through the cells' functions.
- `sdc`, `search` — clocks and constraints; arrivals, requireds, slacks.
- `incr` — the timer's state across edits, each update doing the reference's incremental work.
- `path_enum` — the k worst paths to one endpoint.
- `power` — switching activity and instance power.
- `fuzzy` — the tolerant comparison every merge uses.
- `load` (feature `loom`) — a design read by [loom](https://github.com/vyges-tools/loom)
  (structural Verilog, SDC) mapped into this timer, in the order the reference's network lays it
  out; every construct the timer cannot represent is refused with a reason.

## Correlation, in the repository

`tests/correlation.rs` times a 1,699-cell sky130 netlist of our own under 13 constraint
variants, one per family of constraints above, and compares the worst slack, the total negative
slack and every pin's slack with the reference's, line for line, for setup and for hold
(`tests/data/correlation`). Two more data sets reuse that netlist: `tests/data/constants` ties
pins to constants and supply nets and applies case analysis, and `tests/data/spef` gives every
net an RC network from a SPEF. The reference's outputs there are
regenerated by running it on those files, never edited.

## sta and sta-si

Two timers, two jobs:

- **sta** (this crate) is the timer the optimizing engines link: a library, exact to the
  reference's bits so that an engine ranks paths as the reference does.
- **[sta-si](https://github.com/vyges-tools/sta-si)** is the timer you run on a design: it reads
  Verilog or Yosys JSON, Liberty, SPEF and SDC, reports setup, hold and recovery/removal, applies
  case analysis, disabled timing and data checks, and writes SDF.

Variation-aware timing (LVF/POCV), current-source models (CCS) and signal integrity belong to
sta-si's side of that line.

## Build

No dependencies by default; nothing but Rust.

```sh
cargo test --release                  # the library
cargo test --release --features loom  # with the loom loader and the correlation test
```

Licensed under Apache-2.0.
