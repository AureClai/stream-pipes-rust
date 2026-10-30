//! Junction configurations: what every kind of node must handle.
//!
//! Ordinary nodes (merge, diverge, crossing, U-turn, parallel links, trip
//! ends) and roundabout nodes (one-way arms, chained roundabouts, a closed
//! exit, the same roundabout twice, two-lane rings), plus the scenario
//! checks that keep a vehicle on a connected path.

use stream_core_rust::assignment::assign_demand;
use stream_core_rust::model::{
    Demand, FundamentalDiagram, Link, LinkAttrs, LinkStateChange, Node, NodeType, PipeSpec,
    RoundaboutArm, RoundaboutSpec, Scenario, Vehicle, VehicleState, ALL_CLASSES,
};
use stream_core_rust::simulation::Simulation;
use stream_core_rust::validation::Validate;

fn fd(c: f64) -> FundamentalDiagram {
    FundamentalDiagram { u: 10.0, w: 5.0, kx: 0.15, c }
}

fn link(id: usize, up: usize, down: usize, length: f64, c: f64) -> Link {
    Link::new(id, up, down, length, 10.0, 1, c, fd(c), vec![])
}

/// Nodes 0..n with types; incoming/outgoing lists filled from `links`.
fn network(types: &[NodeType], links: Vec<Link>) -> (Vec<Node>, Vec<Link>) {
    let mut nodes: Vec<Node> = types
        .iter()
        .enumerate()
        .map(|(id, &t)| Node {
            id,
            node_type: t,
            incoming_links: vec![],
            outgoing_links: vec![],
            points: (id as f64 * 100.0, 0.0),
            signals: vec![],
            roundabout: None,
        })
        .collect();
    for l in &links {
        nodes[l.node_up].outgoing_links.push(l.id);
        nodes[l.node_down].incoming_links.push(l.id);
    }
    (nodes, links)
}

fn scenario(nodes: Vec<Node>, links: Vec<Link>, vehicles: Vec<Vehicle>) -> Scenario {
    Scenario {
        nodes,
        links,
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 100_000.0,
        classes: vec!["car".to_string(), "bus".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
    }
}

fn run(s: Scenario) -> Simulation {
    let mut sim = Simulation::new(s);
    sim.run().expect("run");
    sim
}

fn all_exited(sim: &Simulation) {
    for v in &sim.scenario.vehicles {
        assert_eq!(v.state, VehicleState::Exited, "vehicle {} did not exit (path {:?})", v.id, v.path);
        assert_eq!(v.node_times.len(), v.path.len() + 1, "vehicle {} incomplete", v.id);
    }
}

use NodeType::{Entry, Exit, Internal};

// ── Ordinary nodes ────────────────────────────────────────────────────────────

/// Three approaches merging into one link: everybody passes, the merge
/// discharges at the downstream capacity.
#[test]
fn three_way_merge() {
    let (nodes, links) = network(
        &[Entry, Entry, Entry, Internal, Exit],
        vec![link(0, 0, 3, 100.0, 1.0), link(1, 1, 3, 100.0, 1.0), link(2, 2, 3, 100.0, 1.0), link(3, 3, 4, 100.0, 0.5)],
    );
    let mut veh = Vec::new();
    for k in 0..30 {
        let a = k % 3;
        veh.push(Vehicle::new(k, 0, vec![a, 3], (k / 3) as f64, a, 4));
    }
    let sim = run(scenario(nodes, links, veh));
    all_exited(&sim);
    let mut t: Vec<f64> = sim.scenario.vehicles.iter().map(|v| v.node_times[1]).collect();
    t.sort_by(f64::total_cmp);
    for w in t.windows(2) {
        assert!(w[1] - w[0] >= 2.0 - 1e-9, "merge discharges faster than C");
    }
}

/// A four-leg crossing: every movement goes through. Movements only
/// interact through the links they share (no conflict area at the node).
#[test]
fn four_leg_crossing_all_movements() {
    // Arms k = 0..4: entry node k, exit node 4 + k, centre 8.
    let mut types = vec![Entry; 4];
    types.extend([Exit; 4]);
    types.push(Internal);
    let mut ls = Vec::new();
    for k in 0..4 {
        ls.push(link(2 * k, k, 8, 100.0, 1.0));
        ls.push(link(2 * k + 1, 8, 4 + k, 100.0, 1.0));
    }
    let (nodes, links) = network(&types, ls);
    let mut veh = Vec::new();
    for a in 0..4 {
        for b in 0..4 {
            let id = veh.len();
            veh.push(Vehicle::new(id, 0, vec![2 * a, 2 * b + 1], id as f64, a, 4 + b));
        }
    }
    let sim = run(scenario(nodes, links, veh));
    all_exited(&sim);
}

/// U-turn at an ordinary node: out and back on the same pair of nodes.
#[test]
fn u_turn_at_an_ordinary_node() {
    let (nodes, links) = network(
        &[Entry, Internal, Internal, Exit],
        vec![link(0, 0, 1, 100.0, 1.0), link(1, 1, 2, 100.0, 1.0), link(2, 2, 1, 100.0, 1.0), link(3, 1, 3, 100.0, 1.0)],
    );
    let veh = vec![Vehicle::new(0, 0, vec![0, 1, 2, 3], 0.0, 0, 3)];
    let sim = run(scenario(nodes, links, veh));
    all_exited(&sim);
    assert!((sim.scenario.vehicles[0].node_times[4] - 40.0).abs() < 1e-9);
}

/// A trip may end at an internal node and start at one.
#[test]
fn trips_starting_and_ending_at_internal_nodes() {
    let (nodes, links) = network(
        &[Entry, Internal, Internal, Exit],
        vec![link(0, 0, 1, 100.0, 1.0), link(1, 1, 2, 100.0, 1.0), link(2, 2, 3, 100.0, 1.0)],
    );
    let veh = vec![
        Vehicle::new(0, 0, vec![0, 1], 0.0, 0, 2),
        Vehicle::new(1, 0, vec![1, 2], 0.0, 1, 3),
    ];
    let s = scenario(nodes, links, veh);
    s.validate().expect("valid");
    let sim = run(s);
    all_exited(&sim);
}

/// Two parallel links between the same nodes: the assignment must take the
/// faster one, whatever their order.
#[test]
fn assignment_picks_the_faster_of_two_parallel_links() {
    for (slow_first, fast_id) in [(true, 1usize), (false, 0)] {
        let (l_slow, l_fast) = if slow_first {
            (link(0, 0, 1, 1000.0, 1.0), link(1, 0, 1, 100.0, 1.0))
        } else {
            (link(1, 0, 1, 1000.0, 1.0), link(0, 0, 1, 100.0, 1.0))
        };
        let mut ls = vec![l_slow, l_fast];
        ls.sort_by_key(|l| l.id);
        ls.push(link(2, 1, 2, 100.0, 1.0));
        let (nodes, links) = network(&[Entry, Internal, Exit], ls);
        let mut s = scenario(nodes, links, vec![]);
        s.demand = vec![Demand {
            period_start: 0.0,
            period_end: 10.0,
            origin: 0,
            destination: 2,
            flow: 360.0,
            count: 1.0,
            class: None,
        }];
        assign_demand(&mut s).unwrap();
        assert_eq!(s.vehicles[0].path, vec![fast_id, 2], "slow link listed first: {slow_first}");
    }
}

/// A path whose links do not connect must be rejected, not teleported.
#[test]
fn disconnected_path_is_rejected() {
    let (nodes, links) = network(
        &[Entry, Internal, Internal, Exit],
        vec![link(0, 0, 1, 100.0, 1.0), link(1, 2, 3, 100.0, 1.0)],
    );
    let s = scenario(nodes, links, vec![Vehicle::new(0, 0, vec![0, 1], 0.0, 0, 3)]);
    assert!(s.validate().is_err(), "path 0 → 1 skips from node 1 to node 2");
}

/// A path must start at the vehicle's origin and end at its destination.
#[test]
fn path_not_matching_origin_or_destination_is_rejected() {
    let (nodes, links) = network(
        &[Entry, Internal, Exit],
        vec![link(0, 0, 1, 100.0, 1.0), link(1, 1, 2, 100.0, 1.0)],
    );
    let s = scenario(nodes.clone(), links.clone(), vec![Vehicle::new(0, 0, vec![1], 0.0, 0, 2)]);
    assert!(s.validate().is_err(), "origin 0 but path starts at node 1");
    let s = scenario(nodes, links, vec![Vehicle::new(0, 0, vec![0], 0.0, 0, 2)]);
    assert!(s.validate().is_err(), "destination 2 but path ends at node 1");
}

/// The engine indexes links and nodes by id: ids must be 0..n.
#[test]
fn non_contiguous_link_ids_are_rejected() {
    let (nodes, mut links) = network(
        &[Entry, Internal, Exit],
        vec![link(0, 0, 1, 100.0, 1.0), link(1, 1, 2, 100.0, 1.0)],
    );
    links[1].id = 5;
    let mut nodes = nodes;
    nodes[1].outgoing_links = vec![5];
    nodes[2].incoming_links = vec![5];
    let s = scenario(nodes, links, vec![Vehicle::new(0, 0, vec![0, 5], 0.0, 0, 2)]);
    assert!(s.validate().is_err(), "link ids 0 and 5");
}

/// A diverge where the movement restriction sends each movement to its own
/// pipe: a blocked exit pipe does not stop the through pipe.
#[test]
fn diverge_with_movement_pipes() {
    let mut l0 = Link::new(0, 0, 1, 100.0, 10.0, 2, 2.0, fd(1.0), vec![]);
    l0.pipe_specs = vec![PipeSpec { lanes: 1, class_mask: ALL_CLASSES }, PipeSpec { lanes: 1, class_mask: ALL_CLASSES }];
    l0.moves.insert(1, vec![1]);
    l0.moves.insert(2, vec![0]);
    let (nodes, links) = network(
        &[Entry, Internal, Exit, Exit],
        vec![l0, link(1, 1, 2, 100.0, 1.0), link(2, 1, 3, 100.0, 0.05)],
    );
    let mut veh = Vec::new();
    for k in 0..20 {
        let exit = k % 2 == 0;
        veh.push(Vehicle::new(k, 0, vec![0, if exit { 2 } else { 1 }], k as f64, 0, if exit { 3 } else { 2 }));
    }
    let sim = run(scenario(nodes, links, veh));
    all_exited(&sim);
    for v in sim.scenario.vehicles.iter().filter(|v| v.path[1] == 1) {
        assert!((v.node_times[1] - v.node_times[0] - 10.0).abs() < 1e-9, "through vehicle {} delayed", v.id);
    }
}

// ── Roundabout nodes ──────────────────────────────────────────────────────────

/// Roundabout at node `centre` with arms (in, out, position).
fn roundabout(arms: &[(Option<usize>, Option<usize>, f64)]) -> RoundaboutSpec {
    RoundaboutSpec {
        circumference: 100.0,
        lanes: 1,
        speed: 10.0,
        fd: FundamentalDiagram { u: 10.0, w: 5.0, kx: 0.15, c: 0.5 },
        critical_gap: 4.0,
        follow_up: 2.0,
        island: 4.0,
        arms: arms
            .iter()
            .map(|&(i, o, p)| RoundaboutArm { in_link: i, out_link: o, position: p })
            .collect(),
    }
}

/// Four-arm roundabout node 0; arm k: entry node 1 + 2k →(link 2k)→ 0
/// →(link 2k + 1)→ exit node 2 + 2k.
fn four_arm(exit_c: [f64; 4]) -> (Vec<Node>, Vec<Link>) {
    let mut types = vec![Internal];
    let mut ls = Vec::new();
    for k in 0..4 {
        types.push(Entry);
        types.push(Exit);
        ls.push(link(2 * k, 1 + 2 * k, 0, 100.0, 1.0));
        ls.push(link(2 * k + 1, 0, 2 + 2 * k, 100.0, exit_c[k]));
    }
    let (mut nodes, links) = network(&types, ls);
    nodes[0].roundabout = Some(roundabout(&(0..4).map(|k| (Some(2 * k), Some(2 * k + 1), 25.0 * k as f64)).collect::<Vec<_>>()));
    (nodes, links)
}

/// One-way arms: an entry-only arm and an exit-only arm.
#[test]
fn roundabout_with_one_way_arms() {
    // Node 0 roundabout; entry-only arm (link 0 from node 1), exit-only arm
    // (link 1 to node 2), a two-way arm (links 2 in from 3, 3 out to 4).
    let (mut nodes, links) = network(
        &[Internal, Entry, Exit, Entry, Exit],
        vec![link(0, 1, 0, 100.0, 1.0), link(1, 0, 2, 100.0, 1.0), link(2, 3, 0, 100.0, 1.0), link(3, 0, 4, 100.0, 1.0)],
    );
    nodes[0].roundabout = Some(roundabout(&[(Some(0), None, 0.0), (None, Some(1), 30.0), (Some(2), Some(3), 60.0)]));
    let veh = vec![
        Vehicle::new(0, 0, vec![0, 1], 0.0, 1, 2),
        Vehicle::new(1, 0, vec![0, 3], 1.0, 1, 4),
        Vehicle::new(2, 0, vec![2, 1], 2.0, 3, 2),
        Vehicle::new(3, 0, vec![2, 3], 3.0, 3, 4),
    ];
    let s = scenario(nodes, links, veh);
    s.validate().expect("valid");
    let sim = run(s);
    all_exited(&sim);
}

/// Two roundabouts chained by a short link, traffic through both.
#[test]
fn two_chained_roundabouts() {
    // R0 = node 0, R1 = node 1. Link 0: entry 2 → R0; link 1: R0 → R1 (30 m);
    // link 2: R1 → exit 3; link 3: entry 4 → R1; link 4: R1 → R0 (30 m);
    // link 5: R0 → exit 5.
    let (mut nodes, links) = network(
        &[Internal, Internal, Entry, Exit, Entry, Exit],
        vec![
            link(0, 2, 0, 100.0, 1.0),
            link(1, 0, 1, 30.0, 1.0),
            link(2, 1, 3, 100.0, 1.0),
            link(3, 4, 1, 100.0, 1.0),
            link(4, 1, 0, 30.0, 1.0),
            link(5, 0, 5, 100.0, 1.0),
        ],
    );
    nodes[0].roundabout = Some(roundabout(&[(Some(0), Some(5), 0.0), (Some(4), Some(1), 50.0)]));
    nodes[1].roundabout = Some(roundabout(&[(Some(1), Some(4), 0.0), (Some(3), Some(2), 50.0)]));
    let mut veh = Vec::new();
    for k in 0..40 {
        let id = veh.len();
        veh.push(Vehicle::new(id, 0, vec![0, 1, 2], k as f64 * 1.5, 2, 3));
        let id = veh.len();
        veh.push(Vehicle::new(id, 0, vec![3, 4, 5], k as f64 * 1.5 + 0.5, 4, 5));
    }
    let s = scenario(nodes, links, veh);
    s.validate().expect("valid");
    let sim = run(s);
    all_exited(&sim);
}

/// A trip that crosses the same roundabout twice.
#[test]
fn same_roundabout_twice() {
    // Four-arm roundabout; extra loop link 8: exit node of arm 1 is replaced
    // by an internal node feeding back into arm 3's approach.
    let (mut nodes, mut links) = four_arm([10.0; 4]);
    // Arm 1 exit (link 3) goes to node 4; make node 4 internal and add a
    // link 8 from node 4 to node 7 (entry of arm 3) made internal too.
    nodes[4].node_type = Internal;
    nodes[7].node_type = Internal;
    links.push(link(8, 4, 7, 50.0, 1.0));
    nodes[4].outgoing_links.push(8);
    nodes[7].incoming_links.push(8);
    let veh = vec![Vehicle::new(0, 0, vec![0, 3, 8, 6, 5], 0.0, 1, 6)];
    let s = scenario(nodes, links, veh);
    s.validate().expect("valid");
    let sim = run(s);
    all_exited(&sim);
    let v = &sim.scenario.vehicles[0];
    assert_eq!(v.ring_exits.len(), 2);
    assert_eq!(v.ring_exits[0].0, 1);
    assert_eq!(v.ring_exits[1].0, 4);
}

/// A trip that ends on an approach (destination = the roundabout node).
#[test]
fn trip_ending_at_the_roundabout_node() {
    let (nodes, links) = four_arm([10.0; 4]);
    let veh = vec![Vehicle::new(0, 0, vec![0], 0.0, 1, 0), Vehicle::new(1, 0, vec![0, 3], 1.0, 1, 4)];
    let s = scenario(nodes, links, veh);
    let sim = run(s);
    all_exited(&sim);
}

/// An exit closed mid-run (patch): vehicles bound for it must wait on their
/// approach, as at an ordinary node, instead of stopping on the ring and
/// blocking every other movement.
#[test]
fn closed_exit_does_not_block_the_ring() {
    let (nodes, links) = four_arm([10.0; 4]);
    // Arm 0 → arm 2 (exit link 5) is closed from t = 0 to t = 1000;
    // arm 1 → arm 3 passes X2 and must flow freely.
    let mut veh = Vec::new();
    for k in 0..10 {
        let id = veh.len();
        veh.push(Vehicle::new(id, 0, vec![0, 5], k as f64 * 5.0, 1, 6));
        let id = veh.len();
        veh.push(Vehicle::new(id, 0, vec![2, 7], k as f64 * 5.0 + 1.0, 3, 8));
    }
    let mut s = scenario(nodes, links, veh);
    let closed = LinkAttrs { class_masks: Some(vec![0]), ..Default::default() };
    let open = LinkAttrs { class_masks: Some(vec![ALL_CLASSES]), ..Default::default() };
    s.link_schedule = vec![
        LinkStateChange { time: 0.0, link_id: 5, attrs: closed },
        LinkStateChange { time: 1000.0, link_id: 5, attrs: open },
    ];
    let sim = run(s);
    all_exited(&sim);
    for v in sim.scenario.vehicles.iter().filter(|v| v.path[0] == 2) {
        let ring = v.ring_exits[0].1 - v.node_times[1];
        assert!(ring < 4.6 + 1.0, "vehicle {} held {ring:.1} s on the ring behind the closed exit", v.id);
    }
    for v in sim.scenario.vehicles.iter().filter(|v| v.path[0] == 0) {
        assert!(v.node_times[1] >= 1000.0 - 1e-9, "vehicle {} entered the ring toward a closed exit", v.id);
    }
}

/// A two-lane ring with a two-pipe approach: both lanes of the approach
/// feed the ring, every vehicle is served.
#[test]
fn two_lane_ring_and_two_pipe_approach() {
    let (mut nodes, mut links) = four_arm([10.0; 4]);
    let spec = nodes[0].roundabout.as_mut().unwrap();
    spec.lanes = 2;
    let mut a = Link::new(0, 1, 0, 100.0, 10.0, 2, 2.0, fd(1.0), vec![]);
    a.pipe_specs = vec![PipeSpec { lanes: 1, class_mask: ALL_CLASSES }, PipeSpec { lanes: 1, class_mask: ALL_CLASSES }];
    // Right lane for the first exit, left lane for the others.
    a.moves.insert(3, vec![0]);
    a.moves.insert(5, vec![1]);
    a.moves.insert(7, vec![1]);
    links[0] = a;
    let mut veh = Vec::new();
    for k in 0..30 {
        let b = 1 + k % 3;
        veh.push(Vehicle::new(k, 0, vec![0, 2 * b + 1], 0.0, 1, 2 + 2 * b));
    }
    let s = scenario(nodes, links, veh);
    s.validate().expect("valid");
    let sim = run(s);
    all_exited(&sim);
    for v in &sim.scenario.vehicles {
        let expected = if v.path[1] == 3 { 0 } else { 1 };
        assert_eq!(v.pipes_taken[0], expected, "vehicle {} lane", v.id);
    }
}

/// Arms cannot be inferred from links without geometry: the error must say
/// so, not report arms "closer than the island".
#[test]
fn arm_inference_without_geometry_says_why() {
    let raw = serde_json::json!({ "diameter": 30.0 });
    let (nodes, links) = four_arm([10.0; 4]);
    let err = stream_core_rust::roundabout::resolve_spec(&raw, &nodes[0], &links)
        .and_then(|spec| stream_core_rust::roundabout::validate_spec(&spec, &nodes[0]))
        .expect_err("links without geometry");
    let msg = err.to_string();
    assert!(msg.contains("geometry") || msg.contains("arms"), "unhelpful message: {msg}");
    assert!(!msg.contains("closer than the island"), "misleading message: {msg}");
}
