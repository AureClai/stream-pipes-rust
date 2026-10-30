//! Roundabout node: closed-form benchmarks and end-to-end checks.
//!
//! Layout used throughout: one roundabout node 0, and per arm k an approach
//! (link 2k, 100 m at 10 m/s) from entry node 1 + 2k and an exit (link
//! 2k + 1, 100 m at 10 m/s) to exit node 2 + 2k. The ring runs at 10 m/s.

use stream_core_rust::io::compile_scenario_from_network_value;
use stream_core_rust::model::{
    FundamentalDiagram, Link, Node, NodeType, RoundaboutArm, RoundaboutSpec, Scenario, Vehicle,
    VehicleState,
};
use stream_core_rust::roundabout::{Passage, RingGeometry};
use stream_core_rust::simulation::Simulation;
use stream_core_rust::validation::Validate;
use stream_core_rust::verification::{verify_scenario, CheckStatus};

#[derive(Clone)]
struct Cfg {
    circumference: f64,
    /// Arm positions; `false` = a dummy arm without links.
    arms: Vec<(f64, bool)>,
    island: f64,
    ring_kx: f64,
    ring_c: f64,
    critical_gap: f64,
    follow_up: f64,
    approach_c: f64,
    /// Per real arm: exit link capacity (veh/s) and jam density.
    exit_c: Vec<f64>,
    exit_kx: f64,
}

/// 100 m ring, four arms at 0/25/50/75 m, 4 m islands: points
/// X0 = 98, M0 = 2, X1 = 23, M1 = 27, X2 = 48, M2 = 52, X3 = 73, M3 = 77.
/// Ring FD u = 10, w = 5, kx = 0.15 → C = 0.5 veh/s (1/C = 2 s).
fn base() -> Cfg {
    Cfg {
        circumference: 100.0,
        arms: vec![(0.0, true), (25.0, true), (50.0, true), (75.0, true)],
        island: 4.0,
        ring_kx: 0.15,
        ring_c: 0.5,
        critical_gap: 4.0,
        follow_up: 2.0,
        approach_c: 1.0,
        exit_c: vec![10.0; 4],
        exit_kx: 10.0,
    }
}

fn fd(c: f64, kx: f64) -> FundamentalDiagram {
    FundamentalDiagram { u: 10.0, w: 5.0, kx, c }
}

fn node(id: usize, node_type: NodeType) -> Node {
    Node {
        id,
        node_type,
        incoming_links: vec![],
        outgoing_links: vec![],
        points: (id as f64, 0.0),
        signals: vec![],
        roundabout: None,
    }
}

/// `trips`: (arm in, arm out, start time), arms counted among real arms.
fn build(cfg: &Cfg, trips: &[(usize, usize, f64)]) -> Scenario {
    let n_real = cfg.arms.iter().filter(|a| a.1).count();
    let mut nodes = vec![node(0, NodeType::Internal)];
    let mut links = Vec::new();
    for k in 0..n_real {
        nodes.push(node(1 + 2 * k, NodeType::Entry));
        nodes.push(node(2 + 2 * k, NodeType::Exit));
        links.push(Link::new(2 * k, 1 + 2 * k, 0, 100.0, 10.0, 1, cfg.approach_c, fd(cfg.approach_c, 0.15), vec![]));
        let c = cfg.exit_c[k];
        links.push(Link::new(2 * k + 1, 0, 2 + 2 * k, 100.0, 10.0, 1, c, fd(c, cfg.exit_kx), vec![]));
    }
    for l in &links {
        nodes[l.node_up].outgoing_links.push(l.id);
        nodes[l.node_down].incoming_links.push(l.id);
    }
    let mut k = 0;
    let arms = cfg
        .arms
        .iter()
        .map(|&(position, real)| {
            let arm = if real {
                let a = RoundaboutArm { in_link: Some(2 * k), out_link: Some(2 * k + 1), position };
                k += 1;
                a
            } else {
                RoundaboutArm { in_link: None, out_link: None, position }
            };
            arm
        })
        .collect();
    nodes[0].roundabout = Some(RoundaboutSpec {
        circumference: cfg.circumference,
        lanes: 1,
        speed: 10.0,
        fd: FundamentalDiagram { u: 10.0, w: 5.0, kx: cfg.ring_kx, c: cfg.ring_c },
        critical_gap: cfg.critical_gap,
        follow_up: cfg.follow_up,
        island: cfg.island,
        arms,
    });
    let vehicles = trips
        .iter()
        .enumerate()
        .map(|(i, &(a, b, t))| Vehicle::new(i, 0, vec![2 * a, 2 * b + 1], t, 1 + 2 * a, 2 + 2 * b))
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
    };
    s.validate().expect("valid roundabout scenario");
    s
}

fn run(s: Scenario) -> Simulation {
    let mut sim = Simulation::new(s);
    sim.run().expect("run");
    sim
}

fn close(a: f64, b: f64, msg: &str) {
    assert!((a - b).abs() < 1e-9, "{msg}: expected {b}, got {a}");
}

fn trace(sim: &Simulation, point: usize) -> &[Passage] {
    &sim.roundabouts[0].as_ref().unwrap().trace[point]
}

fn geom(sim: &Simulation) -> &RingGeometry {
    &sim.roundabouts[0].as_ref().unwrap().geom
}

fn all_exited(sim: &Simulation) {
    for v in &sim.scenario.vehicles {
        assert_eq!(v.state, VehicleState::Exited, "vehicle {} did not exit", v.id);
        assert_eq!(v.node_times.len(), 3, "vehicle {} incomplete", v.id);
    }
}

// ── Benchmarks ────────────────────────────────────────────────────────────────

/// Free flow: approach 10 s, ring arc M_a → X_b at 10 m/s, exit 10 s.
#[test]
fn free_flow_traversal_time() {
    // Right turn 0 → 1 (M0 = 2 → X1 = 23: 21 m), through 0 → 2 (46 m),
    // U-turn 0 → 0 (M0 → X0 = 98: 96 m), started far apart.
    let sim = run(build(&base(), &[(0, 1, 0.0), (0, 2, 100.0), (0, 0, 200.0)]));
    let v = &sim.scenario.vehicles;
    for (veh, arc) in v.iter().zip([21.0, 46.0, 96.0]) {
        let t0 = veh.start_time;
        close(veh.node_times[0], t0, "entry");
        close(veh.node_times[1], t0 + 10.0, "ring entry");
        assert_eq!(veh.ring_exits.len(), 1);
        assert_eq!(veh.ring_exits[0].0, 1);
        close(veh.ring_exits[0].1, t0 + 10.0 + arc / 10.0, "ring exit");
        close(veh.node_times[2], t0 + 20.0 + arc / 10.0, "trip end");
        close(veh.link_entry_time(1).unwrap(), t0 + 10.0 + arc / 10.0, "exit link entry");
    }
}

/// No circulating traffic: a saturated entry discharges at
/// min(1/t_f, C_ring, C_approach).
#[test]
fn entry_capacity_without_conflict() {
    for (follow_up, approach_c, headway) in [(2.5, 1.0, 2.5), (1.0, 1.0, 2.0), (1.0, 0.25, 4.0)] {
        let mut cfg = base();
        cfg.follow_up = follow_up;
        cfg.critical_gap = 4.0;
        cfg.approach_c = approach_c;
        let trips: Vec<_> = (0..8).map(|_| (0, 1, 0.0)).collect();
        let sim = run(build(&cfg, &trips));
        all_exited(&sim);
        for (k, v) in sim.scenario.vehicles.iter().enumerate() {
            close(
                v.node_times[1],
                10.0 + headway * k as f64,
                &format!("t_f {follow_up}, C_app {approach_c}: entry {k}"),
            );
        }
    }
}

/// Siegloch: a regular circulating stream of period H in front of a
/// saturated entry lets exactly floor((H − t_0)/t_f) + 1 vehicles in per
/// gap (t_c = 6, t_f = 4 ⇒ lead = lag = 2 s = 1/C, t_0 = 4), and the
/// circulating vehicles are never delayed.
#[test]
fn entry_capacity_follows_siegloch_in_a_regular_stream() {
    for (h, n_per_gap) in [(7.9, 1usize), (9.0, 2), (12.5, 3), (16.5, 4)] {
        let mut cfg = base();
        cfg.critical_gap = 6.0;
        cfg.follow_up = 4.0;
        // Circulating: arm 3 → arm 1 (M3 = 77 → X1 = 23, passes M0 at +25 m).
        let mut trips: Vec<(usize, usize, f64)> = (0..40).map(|k| (3, 1, k as f64 * h)).collect();
        // Entering: a queue on arm 0, bound for arm 2, that outlasts the
        // circulating stream (≤ 4 per gap × 40 gaps).
        trips.extend((0..250).map(|_| (0, 2, 0.0)));
        let sim = run(build(&cfg, &trips));
        all_exited(&sim);

        let m0 = geom(&sim).merge_point[0];
        let passages = trace(&sim, m0);
        let circ: Vec<f64> = passages.iter().filter(|p| p.circulating).map(|p| p.time).collect();
        assert_eq!(circ.len(), 40);
        for (k, &t) in circ.iter().enumerate() {
            close(t, 12.5 + k as f64 * h, &format!("H {h}: circulating {k} at M0 (priority)"));
        }
        for k in 2..30 {
            let entered = passages
                .iter()
                .filter(|p| !p.circulating && p.time > circ[k] && p.time < circ[k + 1])
                .count();
            assert_eq!(entered, n_per_gap, "H {h}: entries in gap {k}");
        }
    }
}

/// A saturated exit spills back around the ring: vehicles queue on the ring
/// behind it, the storage between the entry and that exit is never exceeded,
/// vehicles that only pass the exit are held behind the queue (FIFO), and
/// the exit link discharges at exactly its capacity.
#[test]
fn saturated_exit_spills_back_around_the_ring() {
    let mut cfg = base();
    // Exit of arm 2: 0.1 veh/s (10 s headway), small storage.
    cfg.exit_c[2] = 0.1;
    cfg.exit_kx = 0.15;
    // Heavy 0 → 2, light 1 → 3 (passes X2 = 48 without leaving there).
    let mut trips: Vec<(usize, usize, f64)> = (0..60).map(|k| (0, 2, k as f64 * 2.0)).collect();
    trips.extend((0..10).map(|k| (1, 3, 100.0 + k as f64 * 20.0)));
    let sim = run(build(&cfg, &trips));
    all_exited(&sim);
    let v = &sim.scenario.vehicles;

    // Exit-link discharge: link 5 (arm 2's exit) admits at 1/C = 10 s once
    // the queue forms.
    let mut admits: Vec<f64> = v[..60].iter().map(|v| v.ring_exits[0].1).collect();
    admits.sort_by(f64::total_cmp);
    for w in admits[20..].windows(2) {
        assert!((w[1] - w[0] - 10.0).abs() < 1e-9, "exit headway {}", w[1] - w[0]);
    }

    // Storage M0 → X2: 46 m · 0.15 = 6.9 ⇒ at most 7 vehicles on that arc.
    let ring_in: Vec<(f64, f64)> = v[..60].iter().map(|v| (v.node_times[1], v.ring_exits[0].1)).collect();
    let max_on_arc = ring_in
        .iter()
        .map(|&(t, _)| ring_in.iter().filter(|&&(a, b)| a <= t && b > t).count())
        .max()
        .unwrap();
    assert!(max_on_arc <= 7, "{max_on_arc} vehicles stored on a 6.9-vehicle arc");
    assert!(max_on_arc >= 6, "the arc should fill up ({max_on_arc})");

    // The 1 → 3 vehicles are held behind the queue at X2: ring time far
    // above the free-flow 4.6 s (M1 = 27 → X3 = 73), for some of them.
    let worst = v[60..]
        .iter()
        .map(|v| v.ring_exits[0].1 - v.node_times[1])
        .fold(0.0, f64::max);
    assert!(worst > 4.6 + 5.0, "through traffic not held by the spillback (worst {worst})");

    // FIFO at X2: passages there follow the order of arrival at the point.
    let x2 = geom(&sim).exit_point[2];
    let times: Vec<f64> = trace(&sim, x2).iter().map(|p| p.time).collect();
    assert!(times.windows(2).all(|w| w[1] >= w[0]));
}

/// A dummy point pair (an arm without links) must not change any result:
/// the storage bound is taken against every downstream point. Exact here
/// because kx·d is an integer on every arc.
#[test]
fn adding_a_point_without_links_changes_nothing() {
    // 120 m ring, arms at 0/30/60/90, 10 m islands: points at 115, 5, 25, 35,
    // … arcs of 10 and 20 m; kx = 0.2 ⇒ 2 and 4 vehicles. Dummy at 15
    // (points 10 and 20: arcs of 5 m ⇒ 1 vehicle).
    let mut cfg = base();
    cfg.circumference = 120.0;
    cfg.arms = vec![(0.0, true), (30.0, true), (60.0, true), (90.0, true)];
    cfg.island = 10.0;
    cfg.ring_kx = 0.2;
    cfg.ring_c = 0.2 * 50.0 / 15.0;
    cfg.exit_c[2] = 0.15;
    cfg.exit_kx = 0.15;
    let mut trips: Vec<(usize, usize, f64)> = (0..50).map(|k| (0, 2, k as f64 * 1.5)).collect();
    trips.extend((0..30).map(|k| (3, 1, k as f64 * 3.0)));
    trips.extend((0..20).map(|k| (1, 3, k as f64 * 4.0)));
    let plain = run(build(&cfg, &trips));

    let mut with_dummy = cfg.clone();
    with_dummy.arms.insert(1, (15.0, false));
    let dummy = run(build(&with_dummy, &trips));

    all_exited(&plain);
    for (a, b) in plain.scenario.vehicles.iter().zip(&dummy.scenario.vehicles) {
        for (x, y) in a.node_times.iter().zip(&b.node_times) {
            assert!((x - y).abs() < 1e-9, "vehicle {}: node times {x} vs {y}", a.id);
        }
        assert!((a.ring_exits[0].1 - b.ring_exits[0].1).abs() < 1e-9, "vehicle {}: ring exit", a.id);
    }
}

/// Two runs are bit-identical.
#[test]
fn deterministic() {
    let mut trips = Vec::new();
    for a in 0..4 {
        for b in 0..4 {
            trips.extend((0..15).map(|k| (a, b, k as f64 * (3.0 + a as f64) + b as f64 * 0.7)));
        }
    }
    let s = build(&base(), &trips);
    let x = run(s.clone());
    let y = run(s);
    for (a, b) in x.scenario.vehicles.iter().zip(&y.scenario.vehicles) {
        let bits = |v: &[f64]| v.iter().map(|t| t.to_bits()).collect::<Vec<_>>();
        assert_eq!(bits(&a.node_times), bits(&b.node_times));
        assert_eq!(a.ring_exits.len(), b.ring_exits.len());
    }
}

/// Every movement for one hour, `headway` seconds apart (staggered so the
/// streams do not arrive in lockstep).
fn all_movements(headway: f64, horizon: f64) -> Vec<(usize, usize, f64)> {
    let mut trips = Vec::new();
    for a in 0..4 {
        for b in 0..4 {
            if a != b {
                let offset = (a * 4 + b) as f64 * headway / 16.0;
                let n = (horizon / headway) as usize;
                trips.extend((0..n).map(|k| (a, b, offset + k as f64 * headway)));
            }
        }
    }
    trips
}

/// Oversaturated demand on every movement with free exits: everybody is
/// eventually served and the ring ends empty (no gridlock, no loss).
#[test]
fn oversaturated_demand_is_served() {
    let sim = run(build(&base(), &all_movements(9.0, 540.0)));
    all_exited(&sim);
    assert!(sim.roundabouts[0].as_ref().unwrap().on_ring.is_empty());
}

/// Stationary demand for an hour: the verification apparatus finds no
/// failure on any link, pipe or node.
#[test]
fn stationary_demand_verifies() {
    // Physical FDs on the approaches (C = kx·u·w/(u + w) = 0.5 veh/s), so
    // the FD adherence check is meaningful.
    let mut cfg = base();
    cfg.approach_c = 0.5;
    // Demand ends early enough that no trip straddles the last 300 s bin
    // boundary (the conservation check compares bins).
    let sim = run(build(&cfg, &all_movements(40.0, 3480.0)));
    all_exited(&sim);
    let report = verify_scenario(&sim.scenario, 300.0);
    for el in &report.elements {
        for c in &el.checks {
            assert_ne!(c.status, CheckStatus::Fail, "{} / {}: {}", el.label, c.name, c.detail);
        }
    }
}

/// GeoJSON: `"type": "Roundabout"` with arms inferred from the bearings.
#[test]
fn geojson_roundabout_compiles_and_runs() {
    let mut features = vec![serde_json::json!({
        "type": "Feature", "geometry": { "type": "Point", "coordinates": [0.0, 0.0] },
        "properties": { "id": 0, "type": "Roundabout", "roundabout": { "diameter": 30.0 } }
    })];
    for k in 0..4usize {
        let a = k as f64 * std::f64::consts::FRAC_PI_2;
        let far = [300.0 * a.cos(), 300.0 * a.sin()];
        features.push(serde_json::json!({
            "type": "Feature", "geometry": { "type": "Point", "coordinates": far },
            "properties": { "id": 1 + 2 * k, "type": "Entry" }
        }));
        features.push(serde_json::json!({
            "type": "Feature", "geometry": { "type": "Point", "coordinates": far },
            "properties": { "id": 2 + 2 * k, "type": "Exit" }
        }));
        for (id, up, down, coords) in [
            (2 * k, 1 + 2 * k, 0, [far, [0.0, 0.0]]),
            (2 * k + 1, 0, 2 + 2 * k, [[0.0, 0.0], far]),
        ] {
            features.push(serde_json::json!({
                "type": "Feature", "geometry": { "type": "LineString", "coordinates": coords },
                "properties": { "id": id, "node_up": up, "node_down": down, "length": 300.0,
                                "speed": 13.9, "lanes": 1, "capacity": 0.5, "fd_kx": 0.15 }
            }));
        }
    }
    let network = serde_json::json!({ "type": "FeatureCollection", "features": features });
    let demand = serde_json::json!([
        { "period_start": 0.0, "period_end": 600.0, "origin": 1, "destination": 6, "flow": 600.0, "count": 100.0 },
        { "period_start": 0.0, "period_end": 600.0, "origin": 3, "destination": 2, "flow": 600.0, "count": 100.0 }
    ]);
    let config = serde_json::json!({ "duration": 3600.0 });
    let mut s = compile_scenario_from_network_value(network, demand, config).expect("compile");
    s.validate().expect("validate");
    let spec = s.nodes[0].roundabout.as_ref().expect("roundabout parsed");
    assert_eq!(spec.arms.len(), 4);
    for (k, arm) in spec.arms.iter().enumerate() {
        assert_eq!(arm.in_link, Some(2 * k));
        assert_eq!(arm.out_link, Some(2 * k + 1));
    }
    stream_core_rust::assignment::assign_demand(&mut s).expect("assignment");
    let sim = run(s);
    all_exited(&sim);
}
