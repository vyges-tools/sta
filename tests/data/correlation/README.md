# Correlation data

The timer's results on a real design and library, against the reference timer (OpenSTA; the
exact version is in `reference.txt`). `tests/correlation.rs` times each constraint variant and
compares worst slack, total negative slack and every pin's setup slack with the `.golden` file,
and the same for hold with the `.hold.golden` file,
line for line.

| file | what |
|---|---|
| `rv_plic_lite.v` | Vyges `rv_plic_lite` (Apache-2.0, github.com/vyges/vyges-rv-plic-lite), synthesized by Yosys to `sky130_fd_sc_hd`: 1,699 cells |
| `sky130_fd_sc_hd__tt_025C_1v80.lib` | the SkyWater `sky130_fd_sc_hd` typical-corner library (Apache-2.0, github.com/google/skywater-pdk), cut to the cells the netlist and the constraints name; nothing else changed |
| `*.sdc` | constraint variants: `base` (relaxed, all met), `tight` (failing paths), `env` (input transitions, output loads), `drive` (min/max input delays, driving cells, a clock waveform, bus patterns), `max_delay` (a path delay) |
| `*.golden` | the reference's `report_worst_slack`, `report_tns` and `get_property slack_max` of every pin, six digits, sorted |
| `*.hold.golden` | the same for min: `report_worst_slack -min`, `report_tns -min`, `slack_min` |

The goldens and the library cut are regenerated, never edited: the regeneration script runs
the reference on these inputs and rewrites them.
