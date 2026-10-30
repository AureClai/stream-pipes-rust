//! A four-arm roundabout coded two ways: as a ring of micro-links (the usual
//! workaround) and as one `Roundabout` node.
//!
//! Every arm sends the same flow `q` (veh/h) to the three other arms in equal
//! parts, for one hour, followed by a half-hour drain. For each coding the
//! demo reports the events processed, the run time, the vehicles served and
//! the mean delay over free flow.
//!
//! The micro-link ring is run with 4, 8 and 16 ring segments, with the ring
//! links numbered before or after the approaches (which decides the priority
//! of the default in-order merge) and with the capacity-proportional merge.
//!
//!   cargo run --release --no-default-features --example roundabout_vs_microlinks

use std::time::Instant;

use stream_core_rust::assignment::assign_demand;
use stream_core_rust::model::{
    Demand, FundamentalDiagram, Link, Node, NodeType, RoundaboutArm, RoundaboutSpec, Scenario,
};
use stream_core_rust::roundabout::{self, RingGeometry};
use stream_core_rust::simulation::{MergeRule, Simulation};
use stream_core_rust::validation::Validate;

const ARMS: usize = 4;
const DIAMETER: f64 = 30.0;
const RING_SPEED: f64 = 25.0 / 3.0; // 30 km/h
const ARM_SPEED: f64 = 125.0 / 9.0; // 50 km/h
const ARM_LENGTH: f64 = 300.0;
const HORIZON: f64 = 3600.0;
const DRAIN: f64 = 1800.0;

fn ring_fd() -> FundamentalDiagram {
    let (u, w, kx) = (RING_SPEED, 5.0, 0.15);
    FundamentalDiagram { u, w, kx, c: kx * u * w / (u + w) }
}

fn arm_fd() -> FundamentalDiagram {
    FundamentalDiagram { u: ARM_SPEED, w: 5.0, kx: 0.15, c: 0.5 }
}

fn node(id: usize, node_type: NodeType, x: f64, y: f64) -> Node {
    Node {
        id,
        node_type,
        incoming_links: vec![],
        outgoing_links: vec![],
        points: (x, y),
        signals: vec![],
        roundabout: None,
    }
}

/// Fill incoming/outgoing lists from the links (ids are already contiguous).
fn wire(nodes: &mut [Node], links: &[Link]) {
    for l in links {
        nodes[l.node_up].outgoing_links.push(l.id);
        nodes[l.node_down].incoming_links.push(l.id);
    }
    for n in nodes.iter_mut() {
        n.incoming_links.sort_unstable();
        n.outgoing_links.sort_unstable();
    }
}

/// Arm k sits at angle k·90° (counter-clockwise from east).
fn arm_angle(k: usize) -> f64 {
    k as f64 * std::f64::consts::TAU / ARMS as f64
}

fn arm_link(id: usize, up: usize, down: usize, from: (f64, f64), to: (f64, f64)) -> Link {
    Link::new(id, up, down, ARM_LENGTH, ARM_SPEED, 1, 0.5, arm_fd(), vec![from, to])
}

/// Demand: every arm sends `q` veh/h to each of the three other arms in
/// equal parts. Entry node of arm k = `entry(k)`, exit node = `exit(k)`.
/// The OD streams are regular; their starts are staggered so they do not
/// arrive in lockstep (which would leave artificially wide gaps).
fn demand(q: f64, entry: impl Fn(usize) -> usize, exit: impl Fn(usize) -> usize) -> Vec<Demand> {
    let mut d = Vec::new();
    let n_od = ARMS * (ARMS - 1);
    for o in 0..ARMS {
        for turn in 1..ARMS {
            let dest = (o + turn) % ARMS;
            let count = q / (ARMS - 1) as f64 * HORIZON / 3600.0;
            let headway = HORIZON / count;
            let j = o * (ARMS - 1) + turn - 1;
            let offset = headway * j as f64 / n_od as f64;
            d.push(Demand {
                period_start: offset,
                period_end: HORIZON + offset,
                origin: entry(o),
                destination: exit(dest),
                flow: q / (ARMS - 1) as f64,
                count,
                class: None,
            });
        }
    }
    d
}

/// Ring of `segments` micro-links (a multiple of ARMS). `ring_first` numbers
/// the ring links before the approaches, so the in-order merge serves the
/// ring first at every arm node.
fn micro_links(segments: usize, ring_first: bool, q: f64) -> Scenario {
    assert_eq!(segments % ARMS, 0);
    let r = DIAMETER / 2.0;
    let seg_len = std::f64::consts::PI * DIAMETER / segments as f64;
    let per_arm = segments / ARMS;

    // Nodes: ring nodes 0..segments, then per arm an entry and an exit node.
    let mut nodes: Vec<Node> = (0..segments)
        .map(|j| {
            let a = j as f64 * std::f64::consts::TAU / segments as f64;
            node(j, NodeType::Internal, r * a.cos(), r * a.sin())
        })
        .collect();
    let entry = |k: usize| segments + 2 * k;
    let exit = |k: usize| segments + 2 * k + 1;
    for k in 0..ARMS {
        let a = arm_angle(k);
        let far = r + ARM_LENGTH;
        nodes.push(node(entry(k), NodeType::Entry, far * a.cos(), far * a.sin()));
        nodes.push(node(exit(k), NodeType::Exit, far * a.cos(), far * a.sin()));
    }

    let ring_ids = |j: usize| if ring_first { j } else { 2 * ARMS + j };
    let arm_ids = |k: usize, out: bool| {
        let base = if ring_first { segments } else { 0 };
        base + 2 * k + usize::from(out)
    };
    let mut links = Vec::new();
    for j in 0..segments {
        let up = j;
        let down = (j + 1) % segments;
        let mut l = Link::new(
            ring_ids(j), up, down, seg_len, RING_SPEED, 1, ring_fd().c, ring_fd(),
            vec![nodes[up].points, nodes[down].points],
        );
        l.priority = 1.0;
        links.push(l);
    }
    for k in 0..ARMS {
        let ring_node = k * per_arm;
        let p_ring = nodes[ring_node].points;
        links.push(arm_link(arm_ids(k, false), entry(k), ring_node, nodes[entry(k)].points, p_ring));
        links.push(arm_link(arm_ids(k, true), ring_node, exit(k), p_ring, nodes[exit(k)].points));
    }
    links.sort_by_key(|l| l.id);
    wire(&mut nodes, &links);

    Scenario {
        nodes,
        links,
        vehicles: vec![],
        demand: demand(q, entry, exit),
        start_time: 0.0,
        duration: HORIZON + DRAIN,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
    }
}

/// The same roundabout as one `Roundabout` node (HCM 6 gap parameters).
fn roundabout_node(q: f64) -> Scenario {
    let r = DIAMETER / 2.0;
    let circumference = std::f64::consts::PI * DIAMETER;
    let mut nodes = vec![node(0, NodeType::Internal, 0.0, 0.0)];
    let entry = |k: usize| 1 + 2 * k;
    let exit = |k: usize| 2 + 2 * k;
    for k in 0..ARMS {
        let a = arm_angle(k);
        let far = r + ARM_LENGTH;
        nodes.push(node(entry(k), NodeType::Entry, far * a.cos(), far * a.sin()));
        nodes.push(node(exit(k), NodeType::Exit, far * a.cos(), far * a.sin()));
    }
    let mut links = Vec::new();
    let mut arms = Vec::new();
    for k in 0..ARMS {
        links.push(arm_link(2 * k, entry(k), 0, nodes[entry(k)].points, (0.0, 0.0)));
        links.push(arm_link(2 * k + 1, 0, exit(k), (0.0, 0.0), nodes[exit(k)].points));
        arms.push(RoundaboutArm {
            in_link: Some(2 * k),
            out_link: Some(2 * k + 1),
            position: k as f64 * circumference / ARMS as f64,
        });
    }
    wire(&mut nodes, &links);
    nodes[0].roundabout = Some(RoundaboutSpec {
        circumference,
        lanes: 1,
        speed: RING_SPEED,
        fd: ring_fd(),
        critical_gap: roundabout::DEFAULT_CRITICAL_GAP,
        follow_up: roundabout::DEFAULT_FOLLOW_UP,
        island: roundabout::DEFAULT_ISLAND,
        arms,
    });
    let scenario = Scenario {
        nodes,
        links,
        vehicles: vec![],
        demand: demand(q, entry, exit),
        start_time: 0.0,
        duration: HORIZON + DRAIN,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
    };
    scenario.validate().expect("valid roundabout scenario");
    scenario
}

struct Outcome {
    events: usize,
    millis: f64,
    total: usize,
    exited: usize,
    mean_delay: f64,
}

fn run(mut scenario: Scenario, rule: MergeRule) -> Outcome {
    assign_demand(&mut scenario).expect("assignment");
    let mut sim = Simulation::new(scenario);
    sim.merge_rule = rule;
    let t0 = Instant::now();
    let events = sim.run().expect("run");
    let millis = t0.elapsed().as_secs_f64() * 1e3;

    let s = &sim.scenario;
    let free: Vec<f64> = s.links.iter().map(|l| l.length / l.speed).collect();
    let mut delays = Vec::new();
    let mut exited = 0;
    for v in &s.vehicles {
        if v.node_times.len() == v.path.len() + 1 {
            exited += 1;
            let tt = v.node_times.last().unwrap() - v.start_time;
            let ff: f64 = v.path.iter().map(|&l| free[l]).sum::<f64>() + ring_ff(s, v);
            delays.push(tt - ff);
        }
    }
    Outcome {
        events,
        millis,
        total: s.vehicles.len(),
        exited,
        mean_delay: delays.iter().sum::<f64>() / delays.len().max(1) as f64,
    }
}

/// Free-flow time spent inside roundabout nodes (zero for micro-links).
fn ring_ff(s: &Scenario, v: &stream_core_rust::model::Vehicle) -> f64 {
    let mut t = 0.0;
    for w in v.path.windows(2) {
        let node = &s.nodes[s.links[w[0]].node_down];
        let Some(spec) = &node.roundabout else { continue };
        let g = RingGeometry::new(spec);
        let arm_in = spec.arms.iter().position(|a| a.in_link == Some(w[0])).unwrap();
        let arm_out = spec.arms.iter().position(|a| a.out_link == Some(w[1])).unwrap();
        t += g.arc(g.merge_point[arm_in], g.exit_point[arm_out]) / spec.speed;
    }
    t
}

fn print(label: &str, q: f64, o: &Outcome) {
    println!(
        "{label:<34} q={q:>5.0} | events/veh {:>6.1} | {:>7.2} ms | served {:>5}/{:<5} | mean delay {:>7.1} s",
        o.events as f64 / o.total.max(1) as f64,
        o.millis,
        o.exited,
        o.total,
        o.mean_delay
    );
}

fn main() {
    for q in [300.0, 600.0, 900.0] {
        let o = run(roundabout_node(q), MergeRule::InOrder);
        print("roundabout node", q, &o);
        for segments in [4usize, 8, 16] {
            for (ring_first, tag) in [(true, "ring ids first"), (false, "approach ids first")] {
                let o = run(micro_links(segments, ring_first, q), MergeRule::InOrder);
                print(&format!("micro {segments:>2} seg, in-order, {tag}"), q, &o);
            }
            let o = run(micro_links(segments, true, q), MergeRule::CapacityProportional);
            print(&format!("micro {segments:>2} seg, capacity"), q, &o);
        }
        println!();
    }
}
