// SPDX-License-Identifier: Apache-2.0
//! Every SDC command loom's command registry says this timer applies, applied: each test times a
//! small design with and without the one command and requires the result to move, so a command
//! the timer stops applying fails here, not silently in a report. The design (sky130, the
//! correlation library) has an input-to-register path, a register-to-register path through a mux,
//! a register-to-output path, a buffered clock branch and a path into a second clock's register.
#![cfg(feature = "loom")]

use vyges_sta::liberty::Library;

const LIB: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/correlation/sky130_fd_sc_hd__tt_025C_1v80.lib");

const NETLIST: &str = "module t (clk, clk2, rst_n, a, b, sel, y);
  input clk, clk2, rst_n, a, b, sel;
  output y;
  wire n1, q1, m, q2, n3, q3, ck2;
  sky130_fd_sc_hd__buf_2 g1 (.A(a), .X(n1));
  sky130_fd_sc_hd__dfrtp_2 r1 (.CLK(clk), .D(n1), .RESET_B(rst_n), .Q(q1));
  sky130_fd_sc_hd__mux2_1 mx (.A0(q1), .A1(b), .S(sel), .X(m));
  sky130_fd_sc_hd__buf_2 cb (.A(clk), .X(ck2));
  sky130_fd_sc_hd__dfrtp_2 r2 (.CLK(ck2), .D(m), .RESET_B(rst_n), .Q(q2));
  sky130_fd_sc_hd__buf_2 g2 (.A(q2), .X(y));
  sky130_fd_sc_hd__buf_2 g3 (.A(q1), .X(n3));
  sky130_fd_sc_hd__dfrtp_2 r3 (.CLK(clk2), .D(n3), .RESET_B(rst_n), .Q(q3));
endmodule
";

/// Clock `c` on `clk`, I/O delays on the data ports; `clk2` has no clock unless a test adds one.
const BASE: &str = "create_clock -name c -period 10 [get_ports clk]
set_input_delay 1 -clock c [get_ports {b sel}]
set_output_delay 1 -clock c [get_ports y]
";

fn libs() -> Vec<Library> {
    let text = std::fs::read_to_string(LIB).expect("the library is in the repository");
    vec![Library::read(&vyges_sta::liberty_parse::parse(&text).expect("parses")).expect("reads")]
}

fn design(libs: &[Library]) -> vyges_sta::load::Design {
    // written once: the tests run in parallel
    static PATH: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    let path = PATH.get_or_init(|| {
        let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("sdc_applied");
        std::fs::create_dir_all(&dir).expect("scratch dir");
        let path = dir.join("t.v");
        std::fs::write(&path, NETLIST).expect("netlist written");
        path.to_string_lossy().into_owned()
    });
    let d = vyges_loom::netlist::load_compact(path).expect("the netlist reads");
    vyges_sta::load::design(&d, libs).expect("every construct maps")
}

/// Every pin's setup slack (and the worst), as the report prints them. `simulated`: through
/// [`vyges_sta::load::design_constraints`], where the logic simulation takes the constants.
fn report(sdc_text: &str, simulated: bool) -> Vec<String> {
    let libs = libs();
    let d = design(&libs);
    let sdc = vyges_loom::sdc::Sdc::parse_with(sdc_text, &vyges_sta::load::sdc_design(&d.netlist, &libs)).expect("the constraints read");
    let (search, env, _notes) = match simulated {
        true => vyges_sta::load::design_constraints(&sdc, &d, &libs),
        false => vyges_sta::load::constraints(&sdc, &d.netlist, &libs),
    }
    .unwrap_or_else(|e| panic!("the constraints map: {e}\n{sdc_text}"));
    let t = vyges_sta::load::time_design(&libs, &d.netlist, &search, env).expect("the design times");
    t.report_lines(libs[0].time_scale)
}

/// The command changes the timing, and the same file without it times as before.
fn moves(extra: &str) -> (Vec<String>, Vec<String>) {
    moves_on(BASE, extra, false)
}

fn moves_on(base: &str, extra: &str, simulated: bool) -> (Vec<String>, Vec<String>) {
    let without = report(base, simulated);
    let with = report(&format!("{base}{extra}\n"), simulated);
    assert_ne!(with, without, "`{extra}` changed nothing:\n{}", with.join("\n"));
    assert_eq!(report(base, simulated), without, "the base times the same twice");
    (without, with)
}

/// The slack of `pin`, if a constrained path reaches it.
fn slack(lines: &[String], pin: &str) -> Option<f64> {
    lines.iter().find_map(|l| l.strip_prefix(&format!("slack {pin} ")).map(|v| v.parse().expect("a number")))
}

/// `create_clock`: a register on the clock's source becomes an endpoint.
#[test]
fn create_clock_times_the_registers_it_clocks() {
    let (without, with) = moves("create_clock -name c2 -period 10 [get_ports clk2]");
    assert_eq!(slack(&without, "r3/D"), None);
    assert!(slack(&with, "r3/D").is_some(), "{with:?}");
}

/// `create_generated_clock`: a clock derived from a master clocks what its source pin drives.
#[test]
fn create_generated_clock_times_the_registers_it_clocks() {
    let (without, with) = moves("create_generated_clock -name g -source [get_ports clk] -divide_by 2 [get_ports clk2]");
    assert_eq!(slack(&without, "r3/D"), None);
    assert!(slack(&with, "r3/D").is_some(), "{with:?}");
}

/// `set_false_path`: the path is not timed; its endpoint has no slack.
#[test]
fn set_false_path_removes_the_path() {
    let (without, with) = moves("set_false_path -to [get_pins r2/D]");
    assert!(slack(&without, "r2/D").is_some());
    assert_eq!(slack(&with, "r2/D"), None, "{with:?}");
}

/// `set_input_delay`: a path from the port starts at the delay after the clock edge.
#[test]
fn set_input_delay_times_the_port() {
    let (without, with) = moves("set_input_delay 3 -clock c [get_ports a]");
    assert_eq!(slack(&without, "r1/D"), None);
    assert!(slack(&with, "r1/D").is_some(), "{with:?}");
}

/// `set_multicycle_path -setup 2`: the capture edge moves one period later, so the slack grows by
/// the period.
#[test]
fn set_multicycle_path_moves_the_capture_edge() {
    let (without, with) = moves("set_multicycle_path 2 -setup -to [get_pins r2/D]");
    let gain = slack(&with, "r2/D").expect("timed") - slack(&without, "r2/D").expect("timed");
    assert!((gain - 10.0).abs() < 1e-3, "gain {gain}");
}

/// `set_output_delay`: a path to the port ends at the delay before the clock edge.
#[test]
fn set_output_delay_times_the_port() {
    let base = BASE.replace("set_output_delay 1 -clock c [get_ports y]\n", "");
    let (without, with) = moves_on(&base, "set_output_delay 1 -clock c [get_ports y]", false);
    assert_eq!(slack(&without, "y"), None);
    assert!(slack(&with, "y").is_some(), "{with:?}");
}

/// `set_case_analysis`: a constant on the mux select disables the arc from the deselected input.
/// Applied by the logic simulation (`design_constraints`); without it the timer refuses the file.
#[test]
fn set_case_analysis_disables_the_deselected_input() {
    let base = format!("{BASE}set_input_delay 6 -clock c [get_ports b]\n");
    let (without, with) = moves_on(&base, "set_case_analysis 0 [get_ports sel]", true);
    assert!(slack(&with, "r2/D").expect("timed") > slack(&without, "r2/D").expect("timed"), "{with:?}");
}

/// `set_logic_zero`: as a case value of 0, a constant the simulation propagates.
#[test]
fn set_logic_zero_disables_the_deselected_input() {
    let base = format!("{BASE}set_input_delay 6 -clock c [get_ports b]\n");
    let (without, with) = moves_on(&base, "set_logic_zero [get_ports sel]", true);
    assert!(slack(&with, "r2/D").expect("timed") > slack(&without, "r2/D").expect("timed"), "{with:?}");
}

/// `set_logic_one`: selects `A1`, so the register path through `A0` is not timed.
#[test]
fn set_logic_one_disables_the_deselected_input() {
    let (without, with) = moves_on(BASE, "set_logic_one [get_ports sel]", true);
    assert_ne!(slack(&with, "r2/D"), slack(&without, "r2/D"), "{with:?}");
}

/// `set_logic_dc`: the pin is a don't-care (X), not a constant. A pin holds one logic value, the
/// last set, so a don't-care after `set_logic_one` takes the constant back off: the timing is
/// that of no constant at all.
#[test]
fn set_logic_dc_replaces_a_constant_with_a_dont_care() {
    let base = format!("{BASE}set_logic_one [get_ports sel]\n");
    let (_, with) = moves_on(&base, "set_logic_dc [get_ports sel]", true);
    assert_eq!(with, report(BASE, true), "a don't-care times as no constant");
}

/// `set_clock_groups -asynchronous`: paths between the groups are not timed.
#[test]
fn set_clock_groups_removes_paths_between_groups() {
    let base = format!("{BASE}create_clock -name c2 -period 10 [get_ports clk2]\n");
    let (without, with) = moves_on(&base, "set_clock_groups -asynchronous -group c -group c2", false);
    assert!(slack(&without, "r3/D").is_some());
    assert_eq!(slack(&with, "r3/D"), None, "{with:?}");
}

/// `set_clock_latency`: an ideal clock's edges arrive that much later at its registers. On a
/// path from clock `c` to clock `c2`, latency on `c` alone delays the launch, so the slack
/// shrinks by it.
#[test]
fn set_clock_latency_delays_the_clock() {
    let base = format!("{BASE}create_clock -name c2 -period 10 [get_ports clk2]\n");
    let (without, with) = moves_on(&base, "set_clock_latency 0.5 [get_clocks c]", false);
    let loss = slack(&without, "r3/D").expect("timed") - slack(&with, "r3/D").expect("timed");
    assert!((loss - 0.5).abs() < 1e-3, "loss {loss}");
}

/// `set_clock_transition`: the ideal clock's slew at the register clock pins, which sets the
/// clock-to-output delay and the setup time.
#[test]
fn set_clock_transition_sets_the_register_slew() {
    moves("set_clock_transition 0.5 [get_clocks c]");
}

/// `set_clock_uncertainty`: the setup check is that much tighter.
#[test]
fn set_clock_uncertainty_tightens_setup() {
    let (without, with) = moves("set_clock_uncertainty 0.2 [get_clocks c]");
    let loss = slack(&without, "r2/D").expect("timed") - slack(&with, "r2/D").expect("timed");
    assert!((loss - 0.2).abs() < 1e-3, "loss {loss}");
}

/// `set_driving_cell`: the port is driven through the cell's output, which delays the input path.
#[test]
fn set_driving_cell_drives_the_port() {
    let base = format!("{BASE}set_input_delay 1 -clock c [get_ports a]\n");
    let (without, with) = moves_on(&base, "set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 [get_ports a]", false);
    assert!(slack(&with, "r1/D") < slack(&without, "r1/D"), "{with:?}");
}

/// `set_input_transition`: the slew at the port, which the first cell's delay depends on.
#[test]
fn set_input_transition_sets_the_port_slew() {
    let base = format!("{BASE}set_input_delay 1 -clock c [get_ports a]\n");
    let (without, with) = moves_on(&base, "set_input_transition 1.5 [get_ports a]", false);
    assert!(slack(&with, "r1/D") < slack(&without, "r1/D"), "{with:?}");
}

/// `set_load`: capacitance on the port, which the driving cell's delay depends on.
#[test]
fn set_load_loads_the_port() {
    let (without, with) = moves("set_load 0.2 [get_ports y]");
    assert!(slack(&with, "y") < slack(&without, "y"), "{with:?}");
}

/// `set_propagated_clock`: the clock's arrival is computed through the clock network (here a
/// buffer on the capturing register's branch) instead of being ideal.
#[test]
fn set_propagated_clock_times_the_clock_network() {
    let (without, with) = moves("set_propagated_clock [get_clocks c]");
    assert!(slack(&with, "r2/D") > slack(&without, "r2/D"), "the capture is later: {with:?}");
}

/// `set_timing_derate -late`: late path delays are scaled up, so setup slack shrinks.
#[test]
fn set_timing_derate_scales_delays() {
    let (without, with) = moves("set_timing_derate -late 1.2");
    assert!(slack(&with, "r2/D") < slack(&without, "r2/D"), "{with:?}");
}
