//! Timing runner for the Stream Python ↔ pipe-stream benchmark
//! (see `bench/README.md`).
//!
//! Loads a compiled scenario whose vehicles are already assigned (paths and
//! entry times exported from Stream Python), simulates it `repeats` times and
//! writes a JSON report: wall-clock timings, event count and every vehicle's
//! node passage times, so the Python driver can compare trajectories.
//!
//! ```bash
//! cargo run --release --no-default-features --example bench_runner -- \
//!     scenario.json report.json [repeats]
//! ```

use std::time::Instant;
use stream_core_rust::{io::load_scenario, simulation::Simulation, validation::Validate};

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        anyhow::bail!("usage: bench_runner <scenario.json> <report.json> [repeats]");
    }
    let repeats: usize = args.get(3).map(|s| s.parse()).transpose()?.unwrap_or(5).max(1);

    let t = Instant::now();
    let scenario = load_scenario(&args[1])?;
    let load_s = t.elapsed().as_secs_f64();
    scenario.validate()?;

    // `init_s` covers Simulation::new (pipe materialisation, event seeding),
    // `run_s` the event loop alone. Each repeat starts from a fresh clone.
    let mut init_s = Vec::with_capacity(repeats);
    let mut run_s = Vec::with_capacity(repeats);
    let mut events = 0usize;
    let mut last = None;
    for _ in 0..repeats {
        let s = scenario.clone();
        let t = Instant::now();
        let mut sim = Simulation::new(s);
        init_s.push(t.elapsed().as_secs_f64());
        let t = Instant::now();
        events = sim.run()?;
        run_s.push(t.elapsed().as_secs_f64());
        last = Some(sim);
    }
    let sim = last.expect("repeats >= 1");

    let node_times: Vec<&Vec<f64>> = sim.scenario.vehicles.iter().map(|v| &v.node_times).collect();
    let report = serde_json::json!({
        "load_s": load_s,
        "init_s": init_s,
        "run_s": run_s,
        "events": events,
        "vehicles": sim.scenario.vehicles.len(),
        "node_times": node_times,
    });
    std::fs::write(&args[2], serde_json::to_string(&report)?)?;
    Ok(())
}
