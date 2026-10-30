//! Priority junctions: closed-form checks.
//!
//! A four-leg crossing at node 0, major road east–west. Arm k (E = 0,
//! N = 1, W = 2, S = 3) enters by link 2k (100 m at 10 m/s: 10 s to the
//! node) from entry node 1 + 2k and leaves by link 2k + 1 to exit node
//! 2 + 2k. Links carry their geometry, so the junction classifies its
//! movements itself.

use stream_core_rust::junction::{resolve_spec, Turn};
use stream_core_rust::model::{FundamentalDiagram, Link, Node, NodeType, Scenario, Vehicle, VehicleState};
use stream_core_rust::signals::{FixedTimePlan, Movement, OffState, SignalGroup, SignalNetwork};
use stream_core_rust::simulation::Simulation;
use stream_core_rust::validation::Validate;

const E: usize = 0;
const N: usize = 1;
const W: usize = 2;
const S: usize = 3;

fn approach(k: usize) -> usize {
    2 * k
}
fn exit(k: usize) -> usize {
    2 * k + 1
}

/// `junction`: the node's `junction` object; `trips`: (arm in, arm out, start).
fn crossing(junction: serde_json::Value, trips: &[(usize, usize, f64)], exit_c: [f64; 4], exit_kx: [f64; 4]) -> Scenario {
    let centre = (1000.0, 1000.0);
    let mut nodes = vec![Node {
        id: 0,
        node_type: NodeType::Internal,
        incoming_links: vec![],
        outgoing_links: vec![],
        points: centre,
        signals: vec![],
        roundabout: None,
        junction: None,
    }];
    let mut links = Vec::new();
    for k in 0..4 {
        let a = k as f64 * std::f64::consts::FRAC_PI_2;
        let far = (centre.0 + 100.0 * a.cos(), centre.1 + 100.0 * a.sin());
        for (id, t) in [(1 + 2 * k, NodeType::Entry), (2 + 2 * k, NodeType::Exit)] {
            nodes.push(Node { id, node_type: t, incoming_links: vec![], outgoing_links: vec![], points: far, signals: vec![], roundabout: None, junction: None });
        }
        let fd_in = FundamentalDiagram { u: 10.0, w: 5.0, kx: 0.15, c: 0.5 };
        links.push(Link::new(approach(k), 1 + 2 * k, 0, 100.0, 10.0, 1, 0.5, fd_in, vec![far, centre]));
        let fd_out = FundamentalDiagram { u: 10.0, w: 5.0, kx: exit_kx[k], c: exit_c[k] };
        links.push(Link::new(exit(k), 0, 2 + 2 * k, 100.0, 10.0, 1, exit_c[k], fd_out, vec![centre, far]));
    }
    for l in &links {
        nodes[l.node_up].outgoing_links.push(l.id);
        nodes[l.node_down].incoming_links.push(l.id);
    }
    nodes[0].junction = Some(resolve_spec(&junction, &nodes[0], &links).expect("junction resolves"));
    let vehicles = trips
        .iter()
        .enumerate()
        .map(|(i, &(a, b, t))| Vehicle::new(i, 0, vec![approach(a), exit(b)], t, 1 + 2 * a, 2 + 2 * b))
        .collect();
    let s = Scenario {
        nodes,
        links,
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 100_000.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
        signals: Default::default(),
        signal_log: Vec::new(),
    };
    s.validate().expect("valid");
    s
}

fn free_exits() -> ([f64; 4], [f64; 4]) {
    ([10.0; 4], [10.0; 4])
}

fn major() -> serde_json::Value {
    serde_json::json!({ "major": [approach(E), exit(E), approach(W), exit(W)] })
}

fn run(s: Scenario) -> Simulation {
    let mut sim = Simulation::new(s);
    sim.run().expect("run");
    sim
}

fn close(a: f64, b: f64, msg: &str) {
    assert!((a - b).abs() < 1e-9, "{msg}: expected {b}, got {a}");
}

fn all_exited(sim: &Simulation) {
    for v in &sim.scenario.vehicles {
        assert_eq!(v.state, VehicleState::Exited, "vehicle {} did not exit", v.id);
    }
}

#[test]
fn movements_are_classified_and_ranked() {
    let (c, k) = free_exits();
    let s = crossing(major(), &[], c, k);
    let spec = s.nodes[0].junction.as_ref().unwrap();
    let m = |a: usize, b: usize| spec.movement(approach(a), exit(b)).unwrap();
    assert_eq!((m(W, E).turn, m(W, E).rank), (Turn::Through, 1));
    assert_eq!((m(W, S).turn, m(W, S).rank), (Turn::Right, 1));
    assert_eq!((m(W, N).turn, m(W, N).rank), (Turn::Left, 2));
    assert_eq!((m(S, E).turn, m(S, E).rank), (Turn::Right, 2));
    assert_eq!((m(S, N).turn, m(S, N).rank), (Turn::Through, 3));
    assert_eq!((m(S, W).turn, m(S, W).rank), (Turn::Left, 4));
    assert_eq!(m(S, S).turn, Turn::UTurn);
    // HCM defaults.
    assert_eq!((m(W, N).critical_gap, m(W, N).follow_up), (4.1, 2.2));
    assert_eq!((m(S, N).critical_gap, m(S, N).follow_up), (6.5, 4.0));
}

/// Siegloch at a priority junction: a regular major stream of period H in
/// front of a saturated minor right turn lets floor((H − t_0)/t_f) + 1
/// vehicles in per gap (t_c = 6, t_f = 4 ⇒ t_0 = 4); the major stream is
/// never delayed.
#[test]
fn minor_right_follows_siegloch_in_a_regular_major_stream() {
    for (h, n_per_gap) in [(7.9, 1usize), (9.0, 2), (12.5, 3), (16.5, 4)] {
        let j = serde_json::json!({
            "major": [approach(E), exit(E), approach(W), exit(W)],
            "movements": [ { "from": approach(S), "to": exit(E), "critical_gap": 6.0, "follow_up": 4.0 } ]
        });
        let mut trips: Vec<(usize, usize, f64)> = (0..40).map(|k| (W, E, k as f64 * h)).collect();
        trips.extend((0..250).map(|_| (S, E, 0.0)));
        let (c, k) = free_exits();
        let sim = run(crossing(j, &trips, c, k));
        all_exited(&sim);
        let v = &sim.scenario.vehicles;
        let major: Vec<f64> = v[..40].iter().map(|v| v.node_times[1]).collect();
        for (k, &t) in major.iter().enumerate() {
            close(t, 10.0 + k as f64 * h, &format!("H {h}: major vehicle {k} not delayed"));
        }
        for k in 2..30 {
            let n = v[40..].iter().filter(|v| v.node_times[1] > major[k] && v.node_times[1] < major[k + 1]).count();
            assert_eq!(n, n_per_gap, "H {h}: minor entries in gap {k}");
        }
    }
}

/// Yield vs stop: an isolated minor vehicle goes at once at a yield, after
/// the stop time at a stop.
#[test]
fn stop_sign_adds_the_stop_time() {
    let (c, k) = free_exits();
    let sim = run(crossing(major(), &[(S, E, 0.0)], c, k));
    close(sim.scenario.vehicles[0].node_times[1], 10.0, "yield");
    let j = serde_json::json!({ "major": [approach(E), exit(E), approach(W), exit(W)], "control": "stop" });
    let sim = run(crossing(j, &[(S, E, 0.0)], c, k));
    close(sim.scenario.vehicles[0].node_times[1], 11.0, "stop: 1 s at the line");
}

/// A major vehicle due within the lag keeps its priority: the minor
/// vehicle waits for it, then `lead`.
#[test]
fn approaching_major_vehicle_keeps_priority() {
    // Minor right S → E at 10 s; major W → E arrives at 11 s (lag 6.2 − 3.3).
    let (c, k) = free_exits();
    let sim = run(crossing(major(), &[(S, E, 0.0), (W, E, 1.0)], c, k));
    let v = &sim.scenario.vehicles;
    close(v[1].node_times[1], 11.0, "major passes on arrival");
    close(v[0].node_times[1], 11.0 + 3.3 / 2.0, "minor follows after lead = t_f/2");
}

/// A major vehicle stuck behind a full exit does not hold the minor
/// movements that cross its path.
#[test]
fn stuck_major_vehicle_does_not_block_minor_traffic() {
    // Exit E stores one vehicle (kx·L = 1) and drains in 100 s: the second
    // eastbound vehicle waits at the line. A minor through S → N arriving
    // at 15 s crosses its path and must not wait for it.
    let (mut c, mut k) = free_exits();
    c[E] = 0.01;
    k[E] = 0.01;
    let sim = run(crossing(major(), &[(W, E, 0.0), (W, E, 1.0), (S, N, 5.0)], c, k));
    all_exited(&sim);
    let v = &sim.scenario.vehicles;
    assert!(v[1].node_times[1] > 20.0, "second major vehicle held by its exit");
    close(v[2].node_times[1], 15.0, "minor through goes past the stuck vehicle");
}

/// Permitted vs protected left turn: the eastbound left turn (W → N) gives
/// way to the westbound through. Without signals it waits for the end of a
/// dense opposing stream; when a signal holds the opposing through on red
/// it goes at once.
#[test]
fn left_turn_permitted_then_protected() {
    // Westbound through every 2.5 s (gaps below t_0 = 4.1 − 1.1 = 3.0 s)
    // arriving from 10 s to 107.5 s; the left-turner arrives at 20 s.
    let mut trips: Vec<(usize, usize, f64)> = (0..40).map(|k| (E, W, k as f64 * 2.5)).collect();
    trips.push((W, N, 10.0));
    let (c, k) = free_exits();
    let sim = run(crossing(major(), &trips, c, k));
    all_exited(&sim);
    close(sim.scenario.vehicles[40].node_times[1], 107.5 + 2.2 / 2.0, "permitted: after the stream");

    // Protected: the westbound through is red over [40, 60).
    let mut s = crossing(major(), &trips, c, k);
    s.signals = SignalNetwork {
        groups: vec![SignalGroup {
            id: "westbound".into(),
            node: 0,
            movements: vec![Movement { from: approach(E), to: Some(exit(W)) }],
            vehicles_per_green: None,
            off: OffState::Green,
            sneakers: 2,
            clearance: 3.0,
        }],
        plans: vec![FixedTimePlan {
            name: None,
            from: 40.0,
            until: 60.0,
            cycle: 20.0,
            offset: 0.0,
            groups: vec!["westbound".into()],
            greens: Default::default(),
        }],
        tolls: vec![],
    };
    s.validate().unwrap();
    let sim = run(s);
    all_exited(&sim);
    close(sim.scenario.vehicles[40].node_times[1], 40.0, "protected: as soon as the opposing through turns red");
}

/// Every movement at once, heavy demand: everybody gets through (no
/// priority deadlock) and the order is deterministic.
#[test]
fn heavy_demand_on_every_movement_clears() {
    let mut trips = Vec::new();
    for a in 0..4 {
        for b in 0..4 {
            if a != b {
                trips.extend((0..30).map(|k| (a, b, k as f64 * 6.0 + (a * 4 + b) as f64 * 0.3)));
            }
        }
    }
    let (c, k) = free_exits();
    let s = crossing(major(), &trips, c, k);
    let x = run(s.clone());
    all_exited(&x);
    let y = run(s);
    for (a, b) in x.scenario.vehicles.iter().zip(&y.scenario.vehicles) {
        assert_eq!(a.node_times, b.node_times);
    }
}

/// GeoJSON: the `junction` object of a node.
#[test]
fn geojson_junction_compiles() {
    let mut features = vec![serde_json::json!({
        "type": "Feature", "geometry": { "type": "Point", "coordinates": [1000.0, 1000.0] },
        "properties": { "id": 0, "junction": { "major": [0, 1, 4, 5], "control": "stop" } }
    })];
    for k in 0..4usize {
        let a = k as f64 * std::f64::consts::FRAC_PI_2;
        let far = [1000.0 + 100.0 * a.cos(), 1000.0 + 100.0 * a.sin()];
        features.push(serde_json::json!({ "type": "Feature", "geometry": { "type": "Point", "coordinates": far }, "properties": { "id": 1 + 2 * k, "type": "Entry" } }));
        features.push(serde_json::json!({ "type": "Feature", "geometry": { "type": "Point", "coordinates": far }, "properties": { "id": 2 + 2 * k, "type": "Exit" } }));
        for (id, up, down, coords) in [(2 * k, 1 + 2 * k, 0, [far, [1000.0, 1000.0]]), (2 * k + 1, 0, 2 + 2 * k, [[1000.0, 1000.0], far])] {
            features.push(serde_json::json!({
                "type": "Feature", "geometry": { "type": "LineString", "coordinates": coords },
                "properties": { "id": id, "node_up": up, "node_down": down, "length": 100.0, "speed": 10.0, "lanes": 1, "capacity": 0.5 }
            }));
        }
    }
    let network = serde_json::json!({ "type": "FeatureCollection", "features": features });
    let s = stream_core_rust::io::compile_scenario_from_network_value(network, serde_json::json!([]), serde_json::json!({})).unwrap();
    s.validate().unwrap();
    let spec = s.nodes[0].junction.as_ref().unwrap();
    assert_eq!(spec.movements.len(), 16);
    assert_eq!(spec.movement(6, 3).unwrap().rank, 3, "S → N is the minor through");
    assert_eq!(spec.movement(6, 3).unwrap().stop_time, 1.0);
    assert_eq!(spec.movement(4, 1).unwrap().stop_time, 0.0, "major movements never stop");
}

/// `verify` checks the priority rules: it passes on engine output and
/// fails when a minor departure is moved into a forbidden gap.
#[test]
fn verification_checks_gap_acceptance() {
    use stream_core_rust::verification::{verify_scenario, CheckStatus};
    let mut trips: Vec<(usize, usize, f64)> = (0..40).map(|k| (W, E, k as f64 * 9.0)).collect();
    trips.extend((0..60).map(|_| (S, E, 0.0)));
    trips.extend((0..20).map(|k| (E, W, k as f64 * 7.0 + 3.0)));
    trips.extend((0..10).map(|k| (W, N, k as f64 * 20.0 + 1.0)));
    let (c, k) = free_exits();
    let sim = run(crossing(major(), &trips, c, k));
    all_exited(&sim);
    let status = |s: &Scenario| {
        let report = verify_scenario(s, 300.0);
        let el = report.elements.iter().find(|e| e.label == "Node 0").unwrap();
        el.checks.iter().find(|c| c.id == "junction_priority").unwrap().status
    };
    assert_eq!(status(&sim.scenario), CheckStatus::Pass);
    // Move a minor right turn 0.5 s after an eastbound passage.
    let mut bad = sim.scenario.clone();
    let t_major = bad.vehicles[10].node_times[1];
    let v = bad.vehicles.iter_mut().find(|v| v.path[0] == approach(S) && v.node_times[1] > t_major).unwrap();
    v.node_times[1] = t_major + 0.5;
    assert_eq!(status(&bad), CheckStatus::Fail);
}
