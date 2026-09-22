//! Solver trace behind Fig. 4 of the working paper: escalation at a blocked
//! exit under the friction coupling, on the worked-example link of §4.1
//! (L = 500 m, u = 25 m/s, w = 5 m/s, c = 0.55 veh/s/lane, kx = 0.127 veh/m,
//! 3 lanes = shoulder exit pipe + 2-lane through pipe), phi = 0.7 for
//! legibility.
//!
//! Demand is staged to expose the three regimes:
//!   - exit-bound 0.5 veh/s from t = 0 against a near-blocked off-ramp
//!     (100 s headway) -> the shoulder pipe (dn0 = 64) spills at t = 128 s;
//!   - through 0.6 veh/s < phi*C1 = 0.77 until t = 180 (throttle armed at
//!     each discharge but never binding), then 1.0 veh/s > phi*C1 (staircase
//!     at exactly 1/(phi*C1) s, queue grows at 0.23 veh/s).
//!
//! Emits one CSV row per vehicle on stdout:
//!   id,kind,pipe,t_entry,t_discharge   (times on/off link L0; NaN = never)
//!
//!   cargo run --example fig4_friction_trace > fig4_trace.csv

use stream_core_rust::model::{
    FundamentalDiagram, Link, Node, NodeType, PipeSpec, Scenario, Vehicle,
};
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

fn scenario() -> Scenario {
    // Worked-example FD of the paper's §4.1.
    let fd0 = FundamentalDiagram {
        u: 25.0,
        w: 5.0,
        kx: 0.127,
        c: 0.55,
    };
    let fd_out = FundamentalDiagram {
        u: 25.0,
        w: 5.0,
        kx: 10.0,
        c: 2.0,
    };
    let mut fd_ramp = fd_out.clone();
    fd_ramp.c = 0.01; // blocked off-ramp: 100 s discharge headway

    let mut l0 = Link::new(0, 0, 1, 500.0, 25.0, 3, 1.65, fd0, vec![]);
    l0.pipe_specs = vec![
        PipeSpec {
            lanes: 1,
            class_mask: u64::MAX,
        }, // shoulder / exit pipe: C0 = 0.55, dn0 = 64
        PipeSpec {
            lanes: 2,
            class_mask: u64::MAX,
        }, // through pipe: C1 = 1.10, dn1 = 127
    ];
    l0.moves.insert(2, vec![0]); // off-ramp reachable only from the shoulder pipe
    l0.moves.insert(1, vec![1]); // through traffic keeps the left pipes
    l0.friction = 0.7;

    let l1 = Link::new(1, 1, 2, 500.0, 25.0, 2, 4.0, fd_out, vec![]);
    let l2 = Link::new(2, 1, 3, 500.0, 25.0, 1, 0.01, fd_ramp, vec![]);

    let mut vehicles = Vec::new();
    let mut id = 0usize;
    // Exit-bound: 0.5 veh/s over [0, 300).
    for k in 0..150 {
        vehicles.push(Vehicle::new(id, 0, vec![0, 2], 2.0 * k as f64, 0, 3));
        id += 1;
    }
    // Through, stage A: 0.6 veh/s over [0, 180).
    for k in 0..108 {
        vehicles.push(Vehicle::new(id, 0, vec![0, 1], k as f64 * (5.0 / 3.0), 0, 2));
        id += 1;
    }
    // Through, stage B: 1.0 veh/s over [180, 360).
    for k in 0..180 {
        vehicles.push(Vehicle::new(id, 0, vec![0, 1], 180.0 + k as f64, 0, 2));
        id += 1;
    }

    Scenario {
        nodes: vec![
            node(0, NodeType::Entry, vec![], vec![0]),
            node(1, NodeType::Internal, vec![0], vec![1, 2]),
            node(2, NodeType::Exit, vec![1], vec![]),
            node(3, NodeType::Exit, vec![2], vec![]),
        ],
        links: vec![l0, l1, l2],
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 500.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
    }
}

fn main() {
    let mut sim = Simulation::new(scenario());
    sim.run().expect("fig4 trace simulation failed");

    println!("id,kind,pipe,t_entry,t_discharge");
    for v in &sim.scenario.vehicles {
        let kind = if v.path.get(1) == Some(&2) { "exit" } else { "through" };
        let pipe = v
            .pipes_taken
            .first()
            .map(|p| p.to_string())
            .unwrap_or_else(|| "-".to_string());
        let t_in = v.node_times.first().copied().unwrap_or(f64::NAN);
        let t_out = v.node_times.get(1).copied().unwrap_or(f64::NAN);
        println!("{},{},{},{:.9},{:.9}", v.id, kind, pipe, t_in, t_out);
    }
}
