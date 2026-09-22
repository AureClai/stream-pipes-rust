//! Friction (φ) under sustained spillback — regression for the wake-up
//! re-arming defect.
//!
//! `Pipe::last_scheduled_ready_time` is a single dedup slot written by two
//! *kinds* of wake-up with unrelated clocks: admission wakes (entry-queue /
//! transfer WakeAt, both derived from storage releases and the entry-headway
//! clock) and the discharge-gate wake armed by friction (φ < 1) or the
//! upstream-capacity flag. With φ < 1 under spillback both kinds are pending
//! on the same pipe simultaneously; each write stomps the other's marker, so
//! every rescan re-pushes the stomped event, and every duplicate pop rescans
//! again — the event population grows without bound (observed in the field
//! as a 34 GB allocation failure on the A47 scenario).
//!
//! With φ = 1 the gate is never armed and the marker has a single writer
//! family — which is why the golden fixtures never caught this.

use stream_core_rust::model::{
    FundamentalDiagram, Link, Node, NodeType, PipeSpec, Scenario, Vehicle, ALL_CLASSES,
};
use stream_core_rust::simulation::Simulation;

/// Entry(0) →[L0: 2 pipes, φ]→ Internal(1) →[L1: 1-lane bottleneck]→ Exit(2)
///
/// L0: 200 m, u = 10, w = 5, kx = 0.05, c = 1.0/lane → per pipe dn = 10,
///     wave delay 40 s, discharge headway 1 s (2 s when gated at φ = 0.5).
/// L1: 100 m, capacity 0.05 veh/s (20 s headway) → permanent spillback on
///     both L0 pipes; long entry queues keep admission wakes pending while
///     the friction gate keeps discharge wakes pending on the same pipes.
fn spillback_scenario(phi: f64, n_veh: usize) -> Scenario {
    let fd0 = FundamentalDiagram {
        u: 10.0,
        w: 5.0,
        kx: 0.05,
        c: 1.0,
    };
    let fd1 = FundamentalDiagram {
        u: 10.0,
        w: 5.0,
        kx: 0.1,
        c: 0.05,
    };
    let mut l0 = Link::new(0, 0, 1, 200.0, 10.0, 2, 2.0, fd0, vec![]);
    l0.pipe_specs = vec![
        PipeSpec {
            lanes: 1,
            class_mask: ALL_CLASSES,
        },
        PipeSpec {
            lanes: 1,
            class_mask: ALL_CLASSES,
        },
    ];
    l0.friction = phi;
    let l1 = Link::new(1, 1, 2, 100.0, 10.0, 1, 0.05, fd1, vec![]);

    let vehicles = (0..n_veh)
        .map(|i| Vehicle::new(i, 0, vec![0, 1], i as f64 * 0.25, 0, 2))
        .collect();

    Scenario {
        nodes: vec![
            Node {
                id: 0,
                node_type: NodeType::Entry,
                incoming_links: vec![],
                outgoing_links: vec![0],
                points: (0.0, 0.0),
                signals: vec![],
            },
            Node {
                id: 1,
                node_type: NodeType::Internal,
                incoming_links: vec![0],
                outgoing_links: vec![1],
                points: (200.0, 0.0),
                signals: vec![],
            },
            Node {
                id: 2,
                node_type: NodeType::Exit,
                incoming_links: vec![1],
                outgoing_links: vec![],
                points: (300.0, 0.0),
                signals: vec![],
            },
        ],
        links: vec![l0, l1],
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 7200.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
    }
}

/// The φ < 1 run must terminate, and with an event count of the same order
/// as the φ = 1 run — not orders of magnitude above it. Pre-fix this test
/// does not return (unbounded event generation).
#[test]
fn friction_under_spillback_terminates() {
    let mut base = Simulation::new(spillback_scenario(1.0, 300));
    let base_events = base.run().expect("phi = 1 run");

    let mut gated = Simulation::new(spillback_scenario(0.5, 300));
    let gated_events = gated.run().expect("phi = 0.5 run");

    // The gate adds wake-ups (one per blocked discharge attempt), so some
    // overhead is expected; a healthy engine stays within a small factor.
    assert!(
        gated_events < base_events * 10,
        "runaway event generation: {} events with phi = 0.5 vs {} with phi = 1.0",
        gated_events,
        base_events
    );
}

/// Physics must be unchanged by the dedup fix: with the downstream bottleneck
/// binding (20 s headway ≫ gated headways), φ must not alter WHAT flows —
/// same served vehicles, same final exit time — only bookkeeping may differ.
#[test]
fn friction_dedup_fix_preserves_throughput() {
    let mut base = Simulation::new(spillback_scenario(1.0, 300));
    base.run().expect("phi = 1 run");
    let mut gated = Simulation::new(spillback_scenario(0.5, 300));
    gated.run().expect("phi = 0.5 run");

    let exits = |s: &Simulation| {
        s.scenario
            .vehicles
            .iter()
            .filter(|v| v.node_times.len() == 3)
            .count()
    };
    assert_eq!(exits(&base), exits(&gated), "served-vehicle count changed");

    let last_exit = |s: &Simulation| {
        s.scenario
            .vehicles
            .iter()
            .filter_map(|v| v.node_times.last().copied())
            .fold(0.0f64, f64::max)
    };
    let (a, b) = (last_exit(&base), last_exit(&gated));
    assert!(
        (a - b).abs() < 1e-6,
        "final exit time changed: {} vs {}",
        a,
        b
    );
}
