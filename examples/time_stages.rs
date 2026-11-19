// SPDX-License-Identifier: Apache-2.0
//! `time_stages THREADS NETLIST.v CONSTRAINTS.sdc LIB.lib...`: the design timed stage by stage on
//! THREADS threads (`set_thread_count` semantics: 0 = every hardware thread), each stage's wall
//! time and the process's peak memory printed (`stage NAME SECONDS VmHWM_KB`), then a digest of
//! every slew, delay, arrival and required, as bits — equal digests at two thread counts mean the
//! runs stored the same bits.
use std::time::Instant;

fn vm_hwm_kb() -> u64 {
    let s = std::fs::read_to_string("/proc/self/status").unwrap_or_default();
    s.lines().find_map(|l| l.strip_prefix("VmHWM:")).and_then(|v| v.split_whitespace().next()).and_then(|v| v.parse().ok()).unwrap_or(0)
}

/// FNV-1a over the bytes, one running hash.
fn fnv(h: &mut u64, bytes: &[u8]) {
    for b in bytes {
        *h ^= u64::from(*b);
        *h = h.wrapping_mul(0x100000001b3);
    }
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [threads, verilog, sdc, libs @ ..] = args.as_slice() else {
        return Err("usage: time_stages THREADS NETLIST.v CONSTRAINTS.sdc LIB.lib...".into());
    };
    let threads: i32 = threads.parse().map_err(|e| format!("THREADS: {e}"))?;
    let t = Instant::now();
    let mut lib_list = Vec::new();
    for lib in libs {
        let text = std::fs::read_to_string(lib).map_err(|e| format!("{lib}: {e}"))?;
        lib_list.push(vyges_sta::liberty::Library::read(&vyges_sta::liberty_parse::parse(&text)?)?);
    }
    let design = vyges_loom::netlist::load_compact(verilog).map_err(|e| e.0)?;
    let d = vyges_sta::load::design(&design, &lib_list)?;
    let nl = &d.netlist;
    let text = std::fs::read_to_string(sdc).map_err(|e| format!("{sdc}: {e}"))?;
    let s = vyges_loom::sdc::Sdc::parse_with(&text, &vyges_sta::load::sdc_design(nl, &lib_list)).map_err(|e| format!("{sdc}: {e:?}"))?;
    // `design_constraints`: the logic constants (case analysis, constant nets) too.
    let (search_sdc, env, _notes) = vyges_sta::load::design_constraints(&s, &d, &lib_list)?;
    println!("stage read {:.3} {}", t.elapsed().as_secs_f64(), vm_hwm_kb());
    let t = Instant::now();
    let mut g = vyges_sta::graph::Graph::build(&lib_list, nl)?;
    let n = g.set_thread_count(threads);
    g.sdc = env;
    g.ideal_clock = vyges_sta::search::ideal_clock_pins(&g, &search_sdc);
    g.ideal_clock_slew = vyges_sta::search::ideal_clock_slews(&g, &search_sdc);
    println!("stage graph {:.3} {} threads={n} vertices={} edges={}", t.elapsed().as_secs_f64(), vm_hwm_kb(), g.vertices.len(), g.edges.len());
    let t = Instant::now();
    g.find_delays(&std::collections::HashMap::new(), None)?;
    println!("stage delays {:.3} {}", t.elapsed().as_secs_f64(), vm_hwm_kb());
    let mut search = vyges_sta::search::Search::in_graph_order(&g, &search_sdc);
    search.constraints_modelled()?;
    let t = Instant::now();
    search.find_arrivals()?;
    println!("stage arrivals {:.3} {}", t.elapsed().as_secs_f64(), vm_hwm_kb());
    let t = Instant::now();
    search.find_requireds()?;
    println!("stage requireds {:.3} {}", t.elapsed().as_secs_f64(), vm_hwm_kb());
    let mut h = 0xcbf29ce484222325u64;
    for s in &g.slew {
        for x in s.iter().flatten() {
            fnv(&mut h, &x.to_bits().to_le_bytes());
        }
    }
    for row in g.delay.iter() {
        for x in row.iter().flatten() {
            fnv(&mut h, &x.to_bits().to_le_bytes());
        }
    }
    let mut paths = 0usize;
    for ps in &search.paths {
        fnv(&mut h, &ps.len().to_le_bytes());
        for p in ps {
            paths += 1;
            fnv(&mut h, format!("{:?}{:?}", p.tag, p.prev).as_bytes());
            fnv(&mut h, &p.arrival.to_bits().to_le_bytes());
            fnv(&mut h, &p.required.to_bits().to_le_bytes());
        }
    }
    let wns = (0..g.vertices.len()).filter(|&v| search.is_endpoint(v)).map(|v| search.wns_slack(v, true)).fold(f32::INFINITY, f32::min);
    println!("digest {h:016x} paths={paths} wns_ns={:.6}", wns / lib_list[0].time_scale);
    Ok(())
}
