//! Dynamic-patch integration tests: closed-form LWR expectations for mid-run
//! link mutations, plus the inertness gate (empty schedule ⇒ bit-identical to
//! the unpatched engine).

use stream_core_rust::model::{
    FundamentalDiagram, Link, LinkAttrs, LinkStateChange, Node, NodeType, Scenario, Vehicle,
};
use stream_core_rust::simulation::Simulation;

fn assert_time(actual: f64, expected: f64, msg: &str) {
    assert!(
        (actual - expected).abs() < 1e-9,
        "{msg}: expected {expected}, got {actual}"
    );
}

/// Entry(0) →[L0]→ Internal(1) →[L1]→ Exit(2).
/// L0: 100 m, u = 10 m/s, capacity 10 veh/s (0.1 s inflow headway),
///     kx per lane 0.02 → storage 2·lanes.
/// L1: 100 m, huge storage, capacity `c1` = the bottleneck discharge rate.
fn corridor(lanes_l0: u8, kx0: f64, c1: f64, n_veh: usize) -> Scenario {
    let fd0 = FundamentalDiagram {
        u: 10.0,
        w: 5.0,
        kx: kx0,
        c: 10.0,
    };
    let fd1 = FundamentalDiagram {
        u: 10.0,
        w: 5.0,
        kx: 10.0,
        c: c1,
    };
    let vehicles = (0..n_veh)
        .map(|i| Vehicle::new(i, 0, vec![0, 1], 0.0, 0, 2))
        .collect();
    let node = |id, node_type, incoming: Vec<usize>, outgoing: Vec<usize>, x: f64| Node {
        id,
        node_type,
        incoming_links: incoming,
        outgoing_links: outgoing,
        points: (x, 0.0),
        signals: vec![],
    };
    Scenario {
        nodes: vec![
            node(0, NodeType::Entry, vec![], vec![0], 0.0),
            node(1, NodeType::Internal, vec![0], vec![1], 1.0),
            node(2, NodeType::Exit, vec![1], vec![], 2.0),
        ],
        links: vec![
            Link::new(0, 0, 1, 100.0, 10.0, lanes_l0, 10.0, fd0, vec![]),
            Link::new(1, 1, 2, 100.0, 10.0, 1, c1, fd1, vec![]),
        ],
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 100_000.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
    }
}

fn run(scenario: Scenario) -> Simulation {
    let mut sim = Simulation::new(scenario);
    sim.run().expect("simulation should complete");
    sim
}

fn change(time: f64, link_id: usize, attrs: LinkAttrs) -> LinkStateChange {
    LinkStateChange {
        time,
        link_id,
        attrs,
    }
}

/// (a) Capacity drop at T on a saturated bottleneck: admissions into L1 are
/// spaced 1/C before T and exactly 1/C' after. The headway armed before T
/// (veh 2's slot at 210) is honored; the new rate applies from the next
/// admission on. Same wake-up structure as the single-stream
/// `bottleneck_discharge_at_capacity` test.
#[test]
fn capacity_drop_switches_discharge_headway() {
    // Unpatched baseline: c1 = 0.01 → passages at 10, 110, 210, 310, 410, 510.
    let base = run(corridor(2, 10.0, 0.01, 6));
    for (k, veh) in base.scenario.vehicles.iter().enumerate() {
        assert_time(
            veh.node_times[1],
            10.0 + 100.0 * k as f64,
            &format!("baseline veh {k} discharge"),
        );
    }

    // Patch at t = 150: capacity 0.01 → 0.005 (headway 100 s → 200 s).
    let mut s = corridor(2, 10.0, 0.01, 6);
    s.link_schedule = vec![change(
        150.0,
        1,
        LinkAttrs {
            capacity: Some(0.005),
            ..Default::default()
        },
    )];
    let sim = run(s);
    let v = &sim.scenario.vehicles;
    // veh 0 and 1 pass under the old headway; veh 2's slot was armed at 110
    // (pre-patch, old rate) and is honored at 210; from then on 200 s.
    let expected = [10.0, 110.0, 210.0, 410.0, 610.0, 810.0];
    for (k, &t) in expected.iter().enumerate() {
        assert_time(v[k].node_times[1], t, &format!("veh {k} discharge"));
    }
}

/// (b) Full closure window [4.5, 40) via class masks (drain semantics):
/// vehicles already on the link exit normally; queued/unentered vehicles
/// resume entry exactly at the window end.
#[test]
fn closure_drains_and_reopens_exactly_at_until() {
    let mut s = corridor(1, 10.0, 10.0, 10); // storage huge; entry headway 0.1 s
                                             // Slow the entry headway to 1 s so exactly 5 vehicles are in before 4.5.
    s.links[0].capacity = 1.0;
    s.link_schedule = vec![
        change(
            4.5,
            0,
            LinkAttrs {
                class_masks: Some(vec![0]),
                ..Default::default()
            },
        ),
        change(
            40.0,
            0,
            LinkAttrs {
                class_masks: Some(vec![u64::MAX]),
                ..Default::default()
            },
        ),
    ];
    let sim = run(s);
    let v = &sim.scenario.vehicles;

    // Vehicles 0–4 entered at 0..4 (1 s headway) — before the closure.
    for k in 0..5usize {
        assert_time(v[k].node_times[0], k as f64, &format!("veh {k} entry"));
        // Drain: traversal unaffected (free flow 10 s, L1 unconstrained).
        assert_time(
            v[k].node_times[1],
            k as f64 + 10.0,
            &format!("veh {k} drains through node 1"),
        );
    }
    // Vehicles 5–9: blocked by the closure, resume at exactly t = 40 with the
    // capacity headway.
    for k in 5..10usize {
        assert_time(
            v[k].node_times[0],
            40.0 + (k - 5) as f64,
            &format!("veh {k} entry after reopen"),
        );
    }
}

/// (d) Storage growth mid-run (added lane): a wave-blocked vehicle is
/// admitted immediately at the patch instant — the new lane starts empty.
#[test]
fn lane_add_admits_blocked_vehicle_immediately() {
    // Baseline (no patch): storage 2, veh 2 waits for veh 0's exit wave
    // (exit at 10 + L/w 20 = 30).
    let base = run(corridor(1, 0.02, 0.01, 3));
    assert_time(
        base.scenario.vehicles[2].node_times[0],
        30.0,
        "baseline veh 2 wave-gated entry",
    );

    let mut s = corridor(1, 0.02, 0.01, 3);
    s.link_schedule = vec![change(
        15.0,
        0,
        LinkAttrs {
            num_lanes: Some(2),
            ..Default::default()
        },
    )];
    let sim = run(s);
    assert_time(
        sim.scenario.vehicles[2].node_times[0],
        15.0,
        "veh 2 admitted at the lane-add instant",
    );
}

/// (e) Storage shrink below current occupancy: the over-fill must drain
/// before ANY new admission — the first (occupancy − dn) exits swallow their
/// releases.
#[test]
fn storage_shrink_blocks_until_overfill_drains() {
    // L0 storage 4 (2 lanes · kx 0.02 · 100 m); L1 discharges 1 veh / 100 s.
    // Vehicles 0–3 fill L0 by t = 0.3.
    let mut s = corridor(2, 0.02, 0.01, 6);
    // At t = 5: lanes 2 → 1, storage 2. Occupancy 4 ⇒ over-fill 2.
    s.link_schedule = vec![change(
        5.0,
        0,
        LinkAttrs {
            num_lanes: Some(1),
            ..Default::default()
        },
    )];
    let sim = run(s);
    let v = &sim.scenario.vehicles;

    // Discharges into L1: veh 0 at 10, then 1/C = 100 s headway.
    assert_time(v[0].node_times[1], 10.0, "veh 0 discharge");
    assert_time(v[1].node_times[1], 110.0, "veh 1 discharge");
    assert_time(v[2].node_times[1], 210.0, "veh 2 discharge");

    // veh 0 and veh 1's exits only drain the over-fill (4 → 3 → 2): no slot
    // opens. veh 2's exit at 210 launches the first real release, usable at
    // 210 + L/w = 230 — veh 4 enters then, NOT at 30 (the unpatched timing).
    assert_time(
        v[4].node_times[0],
        230.0,
        "veh 4 entry waits for the over-fill to drain",
    );
}

/// (c) Inertness: an empty schedule and a schedule whose only entry lies
/// beyond the horizon end produce byte-identical node_times to the unpatched
/// run.
#[test]
fn empty_or_out_of_horizon_schedule_is_inert() {
    let baseline = run(corridor(2, 0.02, 0.01, 6));

    let empty = run({
        let mut s = corridor(2, 0.02, 0.01, 6);
        s.link_schedule = Vec::new();
        s
    });
    let beyond = run({
        let mut s = corridor(2, 0.02, 0.01, 6);
        s.duration = 1000.0;
        s.link_schedule = vec![change(
            2000.0,
            0,
            LinkAttrs {
                capacity: Some(0.1),
                ..Default::default()
            },
        )];
        s
    });
    let baseline_short = run({
        let mut s = corridor(2, 0.02, 0.01, 6);
        s.duration = 1000.0;
        s
    });

    for (a, b) in baseline
        .scenario
        .vehicles
        .iter()
        .zip(empty.scenario.vehicles.iter())
    {
        assert_eq!(
            a.node_times.len(),
            b.node_times.len(),
            "veh {} node_times length",
            a.id
        );
        for (ta, tb) in a.node_times.iter().zip(b.node_times.iter()) {
            assert!(ta.to_bits() == tb.to_bits(), "veh {} bit-identical", a.id);
        }
    }
    for (a, b) in baseline_short
        .scenario
        .vehicles
        .iter()
        .zip(beyond.scenario.vehicles.iter())
    {
        for (ta, tb) in a.node_times.iter().zip(b.node_times.iter()) {
            assert!(
                ta.to_bits() == tb.to_bits(),
                "veh {} bit-identical with out-of-horizon schedule",
                a.id
            );
        }
    }
}

/// Speed change mid-run only affects vehicles that enter after the boundary
/// (traversal time is fixed at entry).
#[test]
fn speed_change_applies_to_subsequent_entries() {
    let mut s = corridor(1, 10.0, 10.0, 4);
    s.links[0].capacity = 1.0; // entries at 0, 1, 2, 3
    s.link_schedule = vec![change(
        1.5,
        0,
        LinkAttrs {
            speed: Some(5.0),
            ..Default::default()
        },
    )];
    let sim = run(s);
    let v = &sim.scenario.vehicles;
    // Vehicles 0–1 entered before 1.5 → 10 s traversal.
    assert_time(v[0].node_times[1], 10.0, "veh 0 old speed");
    assert_time(v[1].node_times[1], 11.0, "veh 1 old speed");
    // Vehicles 2–3 enter at 2, 3 → 20 s traversal.
    assert_time(v[2].node_times[1], 22.0, "veh 2 new speed");
    assert_time(v[3].node_times[1], 23.0, "veh 3 new speed");
}
