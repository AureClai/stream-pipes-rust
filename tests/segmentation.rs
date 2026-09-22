//! Link-splitting invariance gate (P4 material).
//!
//! Newell's theory makes an intermediate link boundary a pure measurement
//! point: cutting a homogeneous link in two with the same FD must not change
//! any passage time, provided the cut leaves the storage rounding untouched
//! (kx * L * lanes integer on both halves, so ceil() is exact and the
//! fractional wave-delay correction vanishes). The discrete scheme is expected
//! to preserve this *bit-exactly* at phi = 1. Friction (phi < 1) deliberately
//! breaks the invariance — the spill trigger is resolved at link granularity —
//! which is documented in the paper (§4.3); this gate pins the phi = 1
//! baseline so that friction's segmentation-dependence is measured against an
//! exact-invariant base model rather than suspected of hiding in the solver.
//!
//! Scenario: physical triangular FD (k_c = c/u < kx), a 200 m two-lane
//! approach feeding a one-lane bottleneck, demand above bottleneck capacity so
//! the queue spills back through the cut point and the storage + backward-wave
//! constraints are exercised on both sides of it.

use stream_core_rust::model::{FundamentalDiagram, Link, Node, NodeType, Scenario, Vehicle};
use stream_core_rust::simulation::Simulation;

fn node(id: usize, node_type: NodeType, incoming: Vec<usize>, outgoing: Vec<usize>) -> Node {
    Node {
        id,
        node_type,
        incoming_links: incoming,
        outgoing_links: outgoing,
        points: (id as f64, 0.0),
        signals: vec![],
    }
}

/// u = 10, w = 5, c = 0.5 veh/s/lane, kx = 0.15 veh/m/lane (k_c = 0.05 < kx).
fn fd_main() -> FundamentalDiagram {
    FundamentalDiagram {
        u: 10.0,
        w: 5.0,
        kx: 0.15,
        c: 0.5,
    }
}

/// Whole approach: Entry ->A(200 m, 2 lanes)-> ->B(bottleneck)-> Exit.
fn whole(n_veh: usize) -> Scenario {
    let fd_b = FundamentalDiagram {
        u: 10.0,
        w: 5.0,
        kx: 0.15,
        c: 0.25,
    };
    let a = Link::new(0, 0, 1, 200.0, 10.0, 2, 1.0, fd_main(), vec![]);
    let b = Link::new(1, 1, 2, 100.0, 10.0, 1, 0.25, fd_b, vec![]);
    let vehicles = (0..n_veh)
        .map(|i| Vehicle::new(i, 0, vec![0, 1], i as f64, 0, 2))
        .collect();
    Scenario {
        nodes: vec![
            node(0, NodeType::Entry, vec![], vec![0]),
            node(1, NodeType::Internal, vec![0], vec![1]),
            node(2, NodeType::Exit, vec![1], vec![]),
        ],
        links: vec![a, b],
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 1_000_000.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
    }
}

/// Same corridor with A cut at its midpoint: A1(100 m) + A2(100 m).
/// Storage: kx*L*lanes = 0.15*100*2 = 30 exactly on each half (no rounding);
/// tau and the wave delay L/w split additively and exactly in floating point.
fn split(n_veh: usize) -> Scenario {
    let fd_b = FundamentalDiagram {
        u: 10.0,
        w: 5.0,
        kx: 0.15,
        c: 0.25,
    };
    let a1 = Link::new(0, 0, 1, 100.0, 10.0, 2, 1.0, fd_main(), vec![]);
    let a2 = Link::new(1, 1, 2, 100.0, 10.0, 2, 1.0, fd_main(), vec![]);
    let b = Link::new(2, 2, 3, 100.0, 10.0, 1, 0.25, fd_b, vec![]);
    let vehicles = (0..n_veh)
        .map(|i| Vehicle::new(i, 0, vec![0, 1, 2], i as f64, 0, 3))
        .collect();
    Scenario {
        nodes: vec![
            node(0, NodeType::Entry, vec![], vec![0]),
            node(1, NodeType::Internal, vec![0], vec![1]),
            node(2, NodeType::Internal, vec![1], vec![2]),
            node(3, NodeType::Exit, vec![2], vec![]),
        ],
        links: vec![a1, a2, b],
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 1_000_000.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
    }
}

#[test]
fn link_splitting_is_bit_exact_at_phi_1() {
    let n = 100;
    let mut sim_w = Simulation::new(whole(n));
    sim_w.run().expect("whole-link run failed");
    let mut sim_s = Simulation::new(split(n));
    sim_s.run().expect("split-link run failed");

    for (vw, vs) in sim_w
        .scenario
        .vehicles
        .iter()
        .zip(sim_s.scenario.vehicles.iter())
    {
        assert_eq!(vw.id, vs.id);
        let entry_w = vw.node_times.first().copied();
        let entry_s = vs.node_times.first().copied();
        assert_eq!(
            entry_w, entry_s,
            "corridor entry differs for vehicle {} (whole {:?} vs split {:?})",
            vw.id, entry_w, entry_s
        );
        let out_w = vw.node_times.last().copied();
        let out_s = vs.node_times.last().copied();
        assert_eq!(
            out_w, out_s,
            "destination arrival differs for vehicle {} (whole {:?} vs split {:?})",
            vw.id, out_w, out_s
        );
        // The bottleneck passage (shared boundary) must also coincide:
        // whole: node_times[1] (A->B); split: node_times[2] (A2->B).
        assert_eq!(
            vw.node_times.get(1),
            vs.node_times.get(2),
            "bottleneck passage differs for vehicle {}",
            vw.id
        );
    }
}
