//! Integration regression tests.
//!
//! Each test runs the full pipeline (compile → assign → simulate → analyse) on
//! one of the committed scenario folders and asserts against the
//! `results/last_run.json` fixture that was captured when the simulation was
//! known to be correct.
//!
//! Determinism note: the event-loop contains no randomness, so `events_processed`
//! must be bit-for-bit identical across runs for the same input.

use std::path::PathBuf;
use stream_core_rust::{
    analysis::compute_link_stats,
    assignment::assign_demand,
    io::compile_scenario_from_values,
    model::{Scenario, VehicleState},
    simulation::Simulation,
};

// ── Helpers ──────────────────────────────────────────────────────────────────

fn scenario_dir(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("scenarios")
        .join(name)
}

/// Full pipeline: compile → assign → simulate.
fn pipeline(name: &str) -> (Simulation, usize) {
    let dir = scenario_dir(name);

    let demand_val: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(dir.join("demand.json")).unwrap()).unwrap();
    let config_val: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(dir.join("config.json")).unwrap()).unwrap();

    let mut scenario =
        compile_scenario_from_values(dir.join("network.geojson"), demand_val, config_val)
            .expect("compile_scenario_from_values should succeed");

    assign_demand(&mut scenario).expect("assignment should succeed");

    let mut sim = Simulation::new(scenario);
    let events = sim.run().expect("simulation should succeed");
    (sim, events)
}

#[derive(serde::Deserialize)]
struct Fixture {
    events_processed: usize,
    vehicles_count: usize,
}

/// Order-sensitive FNV-1a hash over every vehicle's node_times (bit patterns).
/// `events_processed` alone can mask reorderings that conserve the event count;
/// this pins the exact trajectory of every vehicle on the single-stream engine.
fn node_times_checksum(scenario: &Scenario) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    let mut mix = |bits: u64| {
        for byte in bits.to_le_bytes() {
            h ^= byte as u64;
            h = h.wrapping_mul(0x100000001b3);
        }
    };
    for veh in &scenario.vehicles {
        mix(veh.node_times.len() as u64);
        for &t in &veh.node_times {
            mix(t.to_bits());
        }
    }
    h
}

fn load_fixture(name: &str) -> Fixture {
    let path = scenario_dir(name).join("results").join("last_run.json");
    serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap()
}

/// Asserts that `compute_link_stats` output is physically sane:
/// correct series lengths, all values non-negative.
fn assert_stats_sane(scenario: &Scenario, bin_size: f64) {
    let stats = compute_link_stats(scenario, bin_size);
    let n_bins = (scenario.duration / bin_size).ceil() as usize;

    assert_eq!(
        stats.len(),
        scenario.links.len(),
        "stats entry count should match link count"
    );

    for (link_id, s) in &stats {
        assert_eq!(s.flow.values.len(), n_bins, "link {}: wrong flow bin count", link_id);
        assert_eq!(s.density.values.len(), n_bins, "link {}: wrong density bin count", link_id);
        assert_eq!(s.speed.values.len(), n_bins, "link {}: wrong speed bin count", link_id);

        for (i, &v) in s.flow.values.iter().enumerate() {
            assert!(v >= 0.0, "link {}: negative flow {:.2} at bin {}", link_id, v, i);
        }
        for (i, &v) in s.density.values.iter().enumerate() {
            assert!(v >= 0.0, "link {}: negative density {:.2} at bin {}", link_id, v, i);
        }
        for (i, &v) in s.speed.values.iter().enumerate() {
            assert!(v >= 0.0, "link {}: negative speed {:.2} at bin {}", link_id, v, i);
        }
    }
}

// ── Bottleneck ───────────────────────────────────────────────────────────────

#[test]
fn bottleneck_regression() {
    let (sim, events) = pipeline("bottleneck");
    let fix = load_fixture("bottleneck");

    assert_eq!(sim.scenario.vehicles.len(), fix.vehicles_count, "bottleneck: vehicles_count");
    assert_eq!(events, fix.events_processed, "bottleneck: events_processed");
    assert_stats_sane(&sim.scenario, 60.0);
}

/// Upstream link (id=0) feeds a capacity-constrained downstream link.
/// A queue must form: peak density on the upstream link should be well above
/// free-flow conditions (~0.2 veh/km).
#[test]
fn bottleneck_upstream_queue_forms() {
    let (sim, _) = pipeline("bottleneck");
    let stats = compute_link_stats(&sim.scenario, 60.0);

    let upstream = stats.get(&0).expect("link 0 should have stats");
    let peak_density = upstream
        .density
        .values
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);

    assert!(
        peak_density > 5.0,
        "expected queue on upstream link, peak density was {:.2} veh/km",
        peak_density
    );
}

// ── Diverge ──────────────────────────────────────────────────────────────────

#[test]
fn diverge_regression() {
    let (sim, events) = pipeline("diverge");
    let fix = load_fixture("diverge");

    assert_eq!(sim.scenario.vehicles.len(), fix.vehicles_count, "diverge: vehicles_count");
    assert_eq!(events, fix.events_processed, "diverge: events_processed");
    assert_stats_sane(&sim.scenario, 60.0);
}

/// Vehicles released near the end of the simulation window may not complete
/// their trip before time runs out — that is expected.  What we're guarding
/// against is a deadlock that leaves a large fraction of vehicles stuck.
/// A ≥ 97% exit rate is the threshold (for 2000 vehicles over 3600s, only ~44
/// late-starters can't finish their ~90s trip before simulation cutoff).
#[test]
fn diverge_no_deadlock() {
    let (sim, _) = pipeline("diverge");
    let total = sim.scenario.vehicles.len();
    let exited = sim
        .scenario
        .vehicles
        .iter()
        .filter(|v| v.state == VehicleState::Exited)
        .count();

    let exit_rate = exited as f64 / total as f64;
    assert!(
        exit_rate >= 0.97,
        "only {:.1}% of vehicles exited ({}/{}) — possible deadlock",
        exit_rate * 100.0,
        exited,
        total
    );
}

// ── Real world ───────────────────────────────────────────────────────────────

#[test]
fn real_world_regression() {
    let (sim, events) = pipeline("real_world");
    let fix = load_fixture("real_world");

    assert_eq!(sim.scenario.vehicles.len(), fix.vehicles_count, "real_world: vehicles_count");
    assert_eq!(events, fix.events_processed, "real_world: events_processed");
    assert_stats_sane(&sim.scenario, 60.0);
}

// ── Fixture regeneration (run manually after physics changes) ────────────────

/// Regenerate last_run.json for scenarios whose event counts changed due to
/// a model correction (e.g., the Lax-Hopf backward wave fix).
/// Run with:  cargo test --no-default-features update_fixtures -- --ignored
#[test]
#[ignore]
fn update_fixtures() {
    use stream_core_rust::analysis::compute_link_stats;
    use serde_json::json;

    for name in ["bottleneck", "diverge", "real_world"] {
        let (sim, events) = pipeline(name);
        let link_stats = compute_link_stats(&sim.scenario, 60.0);
        let payload = json!({
            "events_processed": events,
            "duration_ms": 0,
            "vehicles_count": sim.scenario.vehicles.len(),
            "status": "success",
            "link_stats": link_stats,
        });
        let path = scenario_dir(name).join("results").join("last_run.json");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        let f = std::fs::File::create(&path).unwrap();
        serde_json::to_writer_pretty(f, &payload).unwrap();
        println!("Updated {}: events={} vehicles={}", name, events, sim.scenario.vehicles.len());
    }
}

// ── Grid (no fixture — sanity only) ──────────────────────────────────────────

/// Grid has no committed fixture yet. We just verify the simulation completes
/// without panicking and produces physically sane output.
#[test]
fn grid_sanity() {
    let (sim, events) = pipeline("grid");

    assert!(sim.scenario.vehicles.len() > 0, "grid: no vehicles generated");
    assert!(events > 0, "grid: no events processed");
    assert_stats_sane(&sim.scenario, 60.0);
}

// -- Determinism goldens (pipe-stream back-compat gate) -----------------------
//
// Recorded on the single-stream engine before the pipe refactor. Any change
// that alters even one passage time on these scenarios fails here.
// Print current values with:
//   cargo test --no-default-features print_checksums -- --ignored --nocapture

#[test]
#[ignore]
fn print_checksums() {
    for name in ["bottleneck", "diverge", "real_world"] {
        let (sim, _) = pipeline(name);
        println!("{}: {:#018x}", name, node_times_checksum(&sim.scenario));
    }
}

#[test]
fn bottleneck_node_times_golden() {
    let (sim, _) = pipeline("bottleneck");
    assert_eq!(node_times_checksum(&sim.scenario), 0xfec5498ca5c480ef, "bottleneck trajectories changed");
}

#[test]
fn diverge_node_times_golden() {
    let (sim, _) = pipeline("diverge");
    assert_eq!(node_times_checksum(&sim.scenario), 0x75ec76840c5feb40, "diverge trajectories changed");
}

#[test]
fn real_world_node_times_golden() {
    let (sim, _) = pipeline("real_world");
    assert_eq!(node_times_checksum(&sim.scenario), 0x8277a09c75c87f9c, "real_world trajectories changed");
}
