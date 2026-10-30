//! The bundled urban scenario (`scenarios/urban`, built by
//! `examples/build_urban.rs`): every node mechanism at once must verify,
//! and each one must actually be exercised.

use stream_core_rust::api::{compile, ScenarioInputs};
use stream_core_rust::assignment::assign_demand;
use stream_core_rust::simulation::Simulation;
use stream_core_rust::verification::{verify_scenario, CheckStatus, ElementType};

fn run() -> Simulation {
    let inputs = ScenarioInputs::from_dir("scenarios/urban", &[]).expect("scenario folder");
    let (mut s, _) = compile(&inputs).expect("compiles and validates");
    assign_demand(&mut s).expect("assignment");
    let mut sim = Simulation::new(s);
    sim.run().expect("run");
    sim
}

#[test]
fn urban_scenario_verifies_and_exercises_every_mechanism() {
    let sim = run();
    let s = &sim.scenario;
    let report = verify_scenario(s, 300.0);
    for el in &report.elements {
        for c in &el.checks {
            assert_ne!(c.status, CheckStatus::Fail, "{} / {}: {}", el.label, c.name, c.detail);
        }
    }
    let check = |label: &str, id: &str| {
        let el = report.elements.iter().find(|e| e.label == label).unwrap_or_else(|| panic!("no element {label}"));
        el.checks.iter().find(|c| c.id == id).unwrap_or_else(|| panic!("{label}: no check {id}")).status
    };
    // Roundabout G3.
    assert!(s.nodes[3].roundabout.is_some());
    for id in ["ring_traversal", "ring_entry_headway", "ring_exit_headway", "ring_fifo"] {
        assert_eq!(check("Node 3", id), CheckStatus::Pass, "roundabout {id}");
    }
    // Priority junctions: G0, G1 (stop), G2, G6, G7, and G4, G5 under signals.
    for n in [0, 1, 2, 4, 5, 6, 7] {
        assert_eq!(check(&format!("Node {n}"), "junction_priority"), CheckStatus::Pass, "junction at node {n}");
    }
    // G8 is an ordinary node.
    assert!(s.nodes[8].junction.is_none() && s.nodes[8].roundabout.is_none());
    // Every signal group was exercised and passed.
    let groups: Vec<_> = report.elements.iter().filter(|e| e.element_type == ElementType::Signal).collect();
    assert_eq!(groups.len(), 6);
    for g in &groups {
        for c in &g.checks {
            assert_eq!(c.status, CheckStatus::Pass, "{}: {} — {}", g.label, c.name, c.detail);
        }
    }
    // Demand: everybody entered the network.
    assert!(s.vehicles.iter().all(|v| !v.node_times.is_empty()), "every vehicle entered");

    // Toll: departures from the toll lane at least 6 s apart (cars).
    let toll = s.signals.tolls[0].link;
    let mut deps: Vec<f64> = s
        .vehicles
        .iter()
        .filter(|v| v.path[0] == toll && v.class_id == 0)
        .filter_map(|v| v.link_exit_time(0))
        .collect();
    deps.sort_by(f64::total_cmp);
    assert!(deps.len() > 100);
    assert!(deps.windows(2).all(|w| w[1] - w[0] >= 2.0 - 1e-9), "toll service respected");

    // Buses use the bus lane on the boulevard approaches of the centre.
    let centre_approaches: Vec<usize> = s.nodes[4].incoming_links.iter().copied().filter(|&l| s.links[l].pipes.len() == 3).collect();
    let mut bus_lane = 0;
    for v in s.vehicles.iter().filter(|v| v.class_id == 1) {
        for (i, l) in v.path.iter().enumerate() {
            if centre_approaches.contains(l) && v.pipes_taken.get(i) == Some(&0) {
                bus_lane += 1;
            }
        }
    }
    assert!(bus_lane > 10, "buses took the bus lane ({bus_lane})");
    for v in s.vehicles.iter().filter(|v| v.class_id == 0) {
        for (i, l) in v.path.iter().enumerate() {
            if centre_approaches.contains(l) {
                assert_ne!(v.pipes_taken.get(i), Some(&0), "car {} in the bus lane", v.id);
            }
        }
    }

    // Ramp metering: one ramp vehicle at most every 4 s while the plan runs.
    let ramp = *s.nodes[9].incoming_links.iter().max().unwrap();
    let mut metered: Vec<f64> = s
        .vehicles
        .iter()
        .filter(|v| v.path[0] == ramp)
        .filter_map(|v| v.link_exit_time(0))
        .filter(|&t| (900.0..2700.0).contains(&t))
        .collect();
    metered.sort_by(f64::total_cmp);
    assert!(metered.len() > 50);
    assert!(metered.windows(2).all(|w| w[1] - w[0] >= 4.0 - 1e-9), "metered headway");

    // Plans change at 30 min: the centre's cycle is 80 s, then 100 s.
    let g = s.signals.groups.iter().position(|g| g.id == "centre_boulevard").unwrap();
    let greens: Vec<f64> = s.signal_log.iter().filter(|r| r.group == g && r.green).map(|r| r.time).collect();
    assert!(greens.contains(&80.0) && greens.contains(&1900.0), "both plans applied");
}

/// The four scenario files are present and valid JSON (regenerate them with
/// `cargo run --example build_urban`).
#[test]
fn urban_scenario_files_are_valid_json() {
    for f in ["network.geojson", "demand.json", "config.json", "signals.json"] {
        let text = std::fs::read_to_string(format!("scenarios/urban/{f}")).expect("file exists");
        let _: serde_json::Value = serde_json::from_str(&text).unwrap_or_else(|e| panic!("{f}: {e}"));
    }
}
