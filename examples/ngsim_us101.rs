//! NGSIM US-101 replication: observed demand through the pipe partition.
//!
//! Network rebuilt from the site's measured geometry (merge and gore
//! positions estimated from the trajectories themselves), per-lane pipes on
//! the weaving section, off-ramp reachable from the auxiliary pipe only.
//! Demand is the observed vehicle population — every entry time, origin
//! (mainline / on-ramp) and destination (through / exit) taken from the
//! trajectory data, no demand model. The downstream boundary condition is
//! the observed minute discharge at 630 m imposed as a time-windowed
//! capacity on a short gate link (the boundary-gate recipe of the paper's
//! field protocol). FD: u and per-lane capacity from the data's own
//! envelope; w = 5.5 m/s, kx = 0.13 veh/m/lane from the literature.
//!
//! Input:  scenarios/ngsim/data/out/us101_sim_input.json
//!         (produced by scenarios/ngsim/build_us101_demand.py)
//! Output: one CSV row per vehicle on stdout:
//!         id,origin,kind,t_depart,t_300m,pipe_300m,t_final
//!
//!   cargo run --release --example ngsim_us101 > us101_sim_trace.csv

use std::collections::HashMap;
use stream_core_rust::model::{
    FundamentalDiagram, Link, LinkAttrs, LinkStateChange, Node, NodeType, PipeSpec, Scenario,
    Vehicle,
};
use stream_core_rust::simulation::Simulation;

const W: f64 = 5.5; // congestion wave speed (m/s), literature value
const KX: f64 = 0.13; // per-lane jam density (veh/m), literature value
const RAMP_LEN: f64 = 80.0;

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

fn lane_pipes(n: u8) -> Vec<PipeSpec> {
    (0..n)
        .map(|_| PipeSpec {
            lanes: 1,
            class_mask: u64::MAX,
        })
        .collect()
}

fn main() {
    let input_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "scenarios/ngsim/data/out/us101_sim_input.json".to_string());
    let raw = std::fs::read_to_string(&input_path)
        .unwrap_or_else(|e| panic!("cannot read {input_path}: {e}"));
    let input: serde_json::Value = serde_json::from_str(&raw).expect("invalid input JSON");

    let u = input["u_est_ms"].as_f64().expect("u_est_ms");
    let c = input["c_lane_est_veh_s"].as_f64().expect("c_lane_est_veh_s");
    let merge = input["merge_y_m"].as_f64().expect("merge_y_m");
    let gore = input["gore_y_m"].as_f64().expect("gore_y_m");
    let duration = input["duration_s"].as_f64().expect("duration_s");
    let fd = FundamentalDiagram { u, w: W, kx: KX, c };

    // Nodes: 0 main entry, 1 ramp entry, 2 merge, 3 station 300 m,
    //        4 gore, 5 gate head (611 m), 6 main exit (640 m), 7 ramp exit.
    let nodes = vec![
        node(0, NodeType::Entry, vec![], vec![0]),
        node(1, NodeType::Entry, vec![], vec![1]),
        node(2, NodeType::Internal, vec![0, 1], vec![2]),
        node(3, NodeType::Internal, vec![2], vec![3]),
        node(4, NodeType::Internal, vec![3], vec![4, 6]),
        node(5, NodeType::Internal, vec![4], vec![5]),
        node(6, NodeType::Exit, vec![5], vec![]),
        node(7, NodeType::Exit, vec![6], vec![]),
    ];

    // Links (id, up, down, length, lanes):
    //   0: 0->2 mainline approach (5 lanes, per-lane pipes)
    //   1: 1->2 on-ramp (1 lane)
    //   2: 2->3 weaving upstream part (6 lanes incl. aux, per-lane pipes)
    //   3: 3->4 weaving downstream part (6 lanes, per-lane pipes;
    //           off-ramp from aux pipe 0 only, through from pipes 1-5)
    //   4: 4->5 downstream carriageway (5 lanes)
    //   5: 5->6 gate link (5 lanes, scheduled capacity)
    //   6: 4->7 off-ramp (1 lane)
    let mut l0 = Link::new(0, 0, 2, merge, u, 5, 5.0 * c, fd.clone(), vec![]);
    l0.pipe_specs = lane_pipes(5);
    let l1 = Link::new(1, 1, 2, RAMP_LEN, u, 1, c, fd.clone(), vec![]);
    let mut l2 = Link::new(2, 2, 3, 300.0 - merge, u, 6, 6.0 * c, fd.clone(), vec![]);
    l2.pipe_specs = lane_pipes(6);
    let mut l3 = Link::new(3, 3, 4, gore - 300.0, u, 6, 6.0 * c, fd.clone(), vec![]);
    l3.pipe_specs = lane_pipes(6);
    l3.moves = HashMap::from([(6usize, vec![0u8]), (4usize, vec![1, 2, 3, 4, 5])]);
    let l4 = Link::new(4, 4, 5, 611.0 - gore, u, 5, 5.0 * c, fd.clone(), vec![]);
    let l5 = Link::new(5, 5, 6, 29.0, u, 5, 5.0 * c, fd.clone(), vec![]);
    let l6 = Link::new(6, 4, 7, 150.0, u, 1, c, fd.clone(), vec![]);

    // Observed vehicle population.
    let mut vehicles = Vec::new();
    for (id, v) in input["vehicles"].as_array().expect("vehicles").iter().enumerate() {
        let start = v["start_time"].as_f64().expect("start_time");
        let is_exit = v["exit"].as_bool().expect("exit");
        let origin_main = v["origin"].as_str().expect("origin") == "main";
        let (path, origin, dest) = match (origin_main, is_exit) {
            (true, false) => (vec![0, 2, 3, 4, 5], 0, 6),
            (true, true) => (vec![0, 2, 3, 6], 0, 7),
            (false, false) => (vec![1, 2, 3, 4, 5], 1, 6),
            (false, true) => (vec![1, 2, 3, 6], 1, 7),
        };
        // ramp entrants were timestamped at the ramp lane: place the entry
        // so the merge arrival matches the observation
        let start = if origin_main { start } else { (start - RAMP_LEN / u).max(0.0) };
        vehicles.push(Vehicle::new(id, 0, path, start, origin, dest));
    }

    // Boundary gate: observed minute discharge at 630 m -> capacity of link 5.
    let mut link_schedule = Vec::new();
    for entry in input["gate_schedule"].as_array().expect("gate_schedule") {
        let minute = entry["minute"].as_f64().expect("minute");
        let cap = entry["capacity_veh_s"].as_f64().expect("capacity_veh_s");
        link_schedule.push(LinkStateChange {
            time: minute * 60.0,
            link_id: 5,
            attrs: LinkAttrs {
                capacity: Some(cap.max(0.05)),
                ..Default::default()
            },
        });
    }
    link_schedule.sort_by(|a, b| a.time.total_cmp(&b.time));

    let scenario = Scenario {
        nodes,
        links: vec![l0, l1, l2, l3, l4, l5, l6],
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration,
        classes: vec!["car".to_string()],
        link_schedule,
        assignment_excluded: Vec::new(),
    };

    let mut sim = Simulation::new(scenario);
    let events = sim.run().expect("us-101 replication failed");
    eprintln!("simulation done: {events} events");

    // path index of link 3 (300 m -> gore) is 2 for every path, so
    // node_times[2] is the 300 m crossing and pipes_taken[2] its pipe.
    println!("id,origin,kind,t_depart,t_300m,pipe_300m,t_final");
    for v in &sim.scenario.vehicles {
        let kind = if v.destination == 7 { "exit" } else { "through" };
        let origin = if v.origin == 0 { "main" } else { "ramp" };
        let t300 = v.node_times.get(2).copied().unwrap_or(f64::NAN);
        let pipe = v
            .pipes_taken
            .get(2)
            .map(|p| p.to_string())
            .unwrap_or_default();
        let t_final = v.node_times.last().copied().unwrap_or(f64::NAN);
        println!(
            "{},{},{},{:.2},{:.6},{},{:.6}",
            v.id, origin, kind, v.start_time, t300, pipe, t_final
        );
    }
}
