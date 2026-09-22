//! Paired contrast scenario: the FIFO-entrapment inconsistency of the
//! single-stream model at a blocked off-ramp, and its resolution by the pipe
//! partition — identical geometry, identical demand, both solved event-exactly.
//!
//! Worked-example link of the paper's §4.1 (L = 500 m, u = 25 m/s, w = 5 m/s,
//! c = 0.55 veh/s/lane, kx = 0.127 veh/m/lane, 3 lanes), off-ramp blocked to a
//! 100 s discharge headway. Demand over [0, 1800) s: through 1.0 veh/s,
//! exit-bound 0.1 veh/s (9% exit share).
//!
//! Run A (single stream): no pipe partition. Strict FIFO holds across the
//! whole carriageway: each blocked exit-bound vehicle at the head of the link
//! entraps everything behind it, and mainline throughput collapses toward the
//! exit-interleaving rate ~ 1 / (share x ramp headway).
//! Run B (pipes): one-lane exit pipe + two-lane through pipe, phi = 0.95 (the
//! calibrated value); through traffic is untouched up to phi*C1 = 1.045 veh/s.
//!
//! Muñoz & Daganzo (2002) and Newell (1999) document what really happens at
//! such a diverge: through traffic keeps moving at reduced capacity. Run A is
//! therefore not imprecise but qualitatively inconsistent; run B reproduces
//! the documented regime.
//!
//! Emits one CSV row per vehicle and run on stdout:
//!   run,id,kind,t_depart,t_entry,t_discharge
//!
//!   cargo run --example diverge_entrapment > entrapment_trace.csv

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

fn scenario(pipes: bool) -> Scenario {
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
        c: 4.0,
    };
    let mut fd_ramp = fd_out.clone();
    fd_ramp.c = 0.01; // blocked off-ramp: 100 s discharge headway

    let mut l0 = Link::new(0, 0, 1, 500.0, 25.0, 3, 1.65, fd0, vec![]);
    if pipes {
        l0.pipe_specs = vec![
            PipeSpec {
                lanes: 1,
                class_mask: u64::MAX,
            }, // shoulder / exit pipe
            PipeSpec {
                lanes: 2,
                class_mask: u64::MAX,
            }, // through pipe
        ];
        l0.moves.insert(2, vec![0]);
        l0.moves.insert(1, vec![1]);
        l0.friction = 0.95; // calibrated value (§6.4)
    }
    let l1 = Link::new(1, 1, 2, 500.0, 25.0, 3, 6.0, fd_out, vec![]);
    let l2 = Link::new(2, 1, 3, 500.0, 25.0, 1, 0.01, fd_ramp, vec![]);

    // Identical demand for both runs: through 1.0 veh/s, exit 0.1 veh/s,
    // interleaved in departure order over [0, 1800).
    let mut vehicles = Vec::new();
    let mut id = 0usize;
    for k in 0..1800 {
        vehicles.push(Vehicle::new(id, 0, vec![0, 1], k as f64, 0, 2));
        id += 1;
    }
    for k in 0..180 {
        vehicles.push(Vehicle::new(id, 0, vec![0, 2], 0.5 + 10.0 * k as f64, 0, 3));
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
        duration: 2000.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
    }
}

fn main() {
    println!("run,id,kind,t_depart,t_entry,t_discharge");
    for (label, pipes) in [("single", false), ("pipes", true)] {
        let mut sim = Simulation::new(scenario(pipes));
        sim.run().expect("entrapment contrast simulation failed");
        for v in &sim.scenario.vehicles {
            let kind = if v.path.get(1) == Some(&2) { "exit" } else { "through" };
            let t_in = v.node_times.first().copied().unwrap_or(f64::NAN);
            let t_out = v.node_times.get(1).copied().unwrap_or(f64::NAN);
            println!(
                "{},{},{},{:.3},{:.9},{:.9}",
                label, v.id, kind, v.start_time, t_in, t_out
            );
        }
    }
}
