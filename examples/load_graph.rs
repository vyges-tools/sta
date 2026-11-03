// SPDX-License-Identifier: Apache-2.0
//! `load_graph LIB.lib NETLIST.v [CONSTRAINTS.sdc]`: the netlist read by loom and laid out by
//! `load`, its timing graph built once — run with `VYGES_STA_BUILD_TRACE=1` to print the build
//! (each vertex, then each edge, in id order) for comparison with the reference's. With
//! constraints, the design is timed for setup and `wns`, `tns` and every pin's slack are printed
//! in the library's time unit, six digits, as the reference's `report_worst_slack` / `report_tns`
//! / `get_property slack_max` print them.
fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (lib, verilog, sdc) = match args.as_slice() {
        [l, v] => (l, v, None),
        [l, v, s] => (l, v, Some(s)),
        _ => return Err("usage: load_graph LIB.lib NETLIST.v [CONSTRAINTS.sdc]".into()),
    };
    let text = std::fs::read_to_string(lib).map_err(|e| format!("{lib}: {e}"))?;
    let libs = vec![vyges_sta::liberty::Library::read(&vyges_sta::liberty_parse::parse(&text)?)?];
    let design = vyges_loom::netlist::load_compact(verilog).map_err(|e| e.0)?;
    let nl = vyges_sta::load::netlist(&design, &libs)?;
    let Some(sdc) = sdc else {
        let g = vyges_sta::graph::Graph::build(&libs, &nl)?;
        println!("{} vertices, {} edges", g.vertices.len(), g.edges.len());
        return Ok(());
    };
    let text = std::fs::read_to_string(sdc).map_err(|e| format!("{sdc}: {e}"))?;
    let s = vyges_loom::sdc::Sdc::parse_with(&text, &vyges_sta::load::sdc_design(&nl, &libs)).map_err(|e| format!("{sdc}: {e:?}"))?;
    let (search_sdc, env, notes) = vyges_sta::load::constraints(&s, &nl, &libs)?;
    for n in notes {
        eprintln!("note: {n}");
    }
    // what reached the timer, for a reader comparing with the reference's own reports
    eprintln!("constraints: {} clocks, input delays on {} ports, output delays on {} ports, {} drives, {} port loads, {} input slews", search_sdc.clocks.len(), search_sdc.input_delays.len(), search_sdc.output_delays.len(), env.input_drive.len(), env.port_pin_cap.len(), env.input_slew.len());
    // `VYGES_LOAD_PINS=a/Y,b/A`: each pin's slews, arrivals and requireds (ns, nine digits)
    if let Ok(pins) = std::env::var("VYGES_LOAD_PINS") {
        let mut g = vyges_sta::graph::Graph::build(&libs, &nl)?;
        g.sdc = env.clone();
        g.ideal_clock = vyges_sta::search::ideal_clock_pins(&g, &search_sdc);
        g.find_delays(&std::collections::HashMap::new(), None)?;
        let mut s = vyges_sta::search::Search::in_graph_order(&g, &search_sdc);
        s.find_arrivals()?;
        s.find_requireds()?;
        for pin in pins.split(',') {
            let Some(v) = g.vertices.iter().position(|x| x.name == pin) else { eprintln!("{pin}: no such pin"); continue };
            for (rf, edge) in [(0, "^"), (1, "v")] {
                let max = vyges_sta::liberty::MAX;
                eprintln!("{pin} {edge} slew {:.9} arrival {:.9} required {:.9}", g.slew[v][rf][max] / libs[0].time_scale, s.arrival_rf(v, rf, max) / libs[0].time_scale, s.required_rf(v, rf, max) / libs[0].time_scale);
            }
        }
    }
    let t = vyges_sta::load::time_design(&libs, &nl, &search_sdc, env)?;
    for line in t.report_lines(libs[0].time_scale) {
        println!("{line}");
    }
    Ok(())
}
