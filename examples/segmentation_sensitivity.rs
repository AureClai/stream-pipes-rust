//! Segmentation sensitivity of the friction trigger (paper §4.3).
//!
//! The spill indicator is resolved at link granularity, so where the modeler
//! cuts the diverge approach changes when — and over what extent — phi acts.
//! This demo runs the same physical corridor (500 m worked-example approach,
//! blocked off-ramp, phi = 0.7, through 1.0 veh/s > phi*C1, exit 0.5 veh/s)
//! under two segmentations: one 500 m link, and two 250 m links cut at the
//! midpoint. It prints the friction onset (first throttled through headway at
//! the diverge), through service, and delays for both.
//!
//!   cargo run --example segmentation_sensitivity

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

fn pipes3() -> Vec<PipeSpec> {
    vec![
        PipeSpec {
            lanes: 1,
            class_mask: u64::MAX,
        },
        PipeSpec {
            lanes: 2,
            class_mask: u64::MAX,
        },
    ]
}

/// `segments`: number of equal approach links (1 -> 500 m, 2 -> 250 m each).
fn scenario(segments: usize) -> Scenario {
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
    fd_ramp.c = 0.01;

    let seg_len = 500.0 / segments as f64;
    let mut links = Vec::new();
    let mut nodes = vec![node(0, NodeType::Entry, vec![], vec![0])];
    for s in 0..segments {
        let mut l = Link::new(s, s, s + 1, seg_len, 25.0, 3, 1.65, fd0.clone(), vec![]);
        l.pipe_specs = pipes3();
        l.friction = 0.7;
        if s == segments - 1 {
            // last approach segment: movements onto the out-links
            l.moves.insert(segments, vec![1]); // through link
            l.moves.insert(segments + 1, vec![0]); // ramp
        }
        links.push(l);
        if s > 0 {
            nodes.push(node(s, NodeType::Internal, vec![s - 1], vec![s]));
        }
    }
    let nd = segments; // diverge node id
    nodes.push(node(nd, NodeType::Internal, vec![segments - 1], vec![segments, segments + 1]));
    nodes.push(node(nd + 1, NodeType::Exit, vec![segments], vec![]));
    nodes.push(node(nd + 2, NodeType::Exit, vec![segments + 1], vec![]));
    links.push(Link::new(segments, nd, nd + 1, 500.0, 25.0, 3, 6.0, fd_out, vec![]));
    links.push(Link::new(segments + 1, nd, nd + 2, 500.0, 25.0, 1, 0.01, fd_ramp, vec![]));

    let thru_path: Vec<usize> = (0..segments).chain([segments]).collect();
    let exit_path: Vec<usize> = (0..segments).chain([segments + 1]).collect();
    let mut vehicles = Vec::new();
    let mut id = 0usize;
    for k in 0..600 {
        vehicles.push(Vehicle::new(id, 0, thru_path.clone(), k as f64, 0, nd + 1));
        id += 1;
    }
    for k in 0..300 {
        vehicles.push(Vehicle::new(id, 0, exit_path.clone(), 0.5 + 2.0 * k as f64, 0, nd + 2));
        id += 1;
    }

    Scenario {
        nodes,
        links,
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 700.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
    }
}

fn main() {
    let phi_c1 = 0.7 * 1.10;
    let h_throttled = 1.0 / phi_c1;
    for segments in [1usize, 2] {
        let mut sim = Simulation::new(scenario(segments));
        sim.run().expect("segmentation run failed");
        let diverge_idx = segments; // node_times index of the diverge passage
        let tau = 20.0;
        let mut disc: Vec<(f64, f64)> = Vec::new(); // (t_pass, delay)
        for v in &sim.scenario.vehicles {
            if v.path.last() == Some(&segments) {
                if let Some(&t) = v.node_times.get(diverge_idx) {
                    if t <= 600.0 {
                        disc.push((t, t - (v.start_time + tau)));
                    }
                }
            }
        }
        disc.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        // friction onset: first through discharge whose *following* headway is
        // the throttled one (tolerance 1e-6)
        let onset = disc
            .windows(2)
            .find(|w| (w[1].0 - w[0].0 - h_throttled).abs() < 1e-6)
            .map(|w| w[0].0);
        let delays: Vec<f64> = disc.iter().map(|d| d.1).collect();
        let mean = delays.iter().sum::<f64>() / delays.len() as f64;
        let max = delays.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        println!(
            "segments={} | through passed by t=600: {} | friction onset (first throttled headway): {:?} s | through delay mean {:.1} s max {:.1} s",
            segments,
            disc.len(),
            onset,
            mean,
            max
        );
    }
}
