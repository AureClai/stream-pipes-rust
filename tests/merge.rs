//! Merge rule at a saturated node.
//!
//! Two entries merge into a bottleneck:
//!
//! Entry(0) →[L0: mainline, 2 lanes, C = 1.0 veh/s]→ ┐
//!                                                    Internal(2) →[L2: C = 0.3 veh/s]→ Exit(3)
//! Entry(1) →[L1: ramp, 1 lane, C = 0.5 veh/s]──────→ ┘
//!
//! Both arrive faster than L2 discharges, so the merge is saturated and its
//! rule alone decides who passes. In-order service gives L0 every slot;
//! the capacity-proportional rule must give L0 2/3 and L1 1/3 of L2's flow.

use stream_core_rust::model::{FundamentalDiagram, Link, Node, NodeType, Scenario, Vehicle};
use stream_core_rust::simulation::{MergeRule, Simulation};

fn fd(c: f64, kx: f64) -> FundamentalDiagram {
    FundamentalDiagram { u: 20.0, w: 5.0, kx, c }
}

fn node(id: usize, node_type: NodeType, incoming: Vec<usize>, outgoing: Vec<usize>) -> Node {
    Node { id, node_type, incoming_links: incoming, outgoing_links: outgoing, points: (0.0, 0.0), signals: vec![], roundabout: None }
}

/// `q0`, `q1`: arrival rates (veh/s) on L0 and L1, for `t_end` seconds.
fn merge_scenario(q0: f64, q1: f64, c2: f64, t_end: f64) -> Scenario {
    let l0 = Link::new(0, 0, 2, 500.0, 20.0, 2, 1.0, fd(0.5, 0.15), vec![]);
    let l1 = Link::new(1, 1, 2, 300.0, 20.0, 1, 0.5, fd(0.5, 0.15), vec![]);
    let l2 = Link::new(2, 2, 3, 500.0, 20.0, 1, c2, fd(c2, 0.15), vec![]);
    let mut vehicles = Vec::new();
    for (link, origin, q) in [(0usize, 0usize, q0), (1, 1, q1)] {
        if q <= 0.0 {
            continue;
        }
        let n = (q * t_end) as usize;
        for k in 0..n {
            let id = vehicles.len();
            vehicles.push(Vehicle::new(id, 0, vec![link, 2], k as f64 / q, origin, 3));
        }
    }
    Scenario {
        nodes: vec![
            node(0, NodeType::Entry, vec![], vec![0]),
            node(1, NodeType::Entry, vec![], vec![1]),
            node(2, NodeType::Internal, vec![0, 1], vec![2]),
            node(3, NodeType::Exit, vec![2], vec![]),
        ],
        links: vec![l0, l1, l2],
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 20_000.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(), signals: Default::default(), signal_log: Vec::new(),
    }
}

fn run(scenario: Scenario, rule: MergeRule) -> Simulation {
    let mut sim = Simulation::new(scenario);
    sim.merge_rule = rule;
    sim.run().expect("run");
    sim
}

/// Vehicles from each origin link passing the merge in [t0, t1).
fn passed(sim: &Simulation, t0: f64, t1: f64) -> (usize, usize) {
    let mut n = (0, 0);
    for v in &sim.scenario.vehicles {
        if let Some(&t) = v.node_times.get(1) {
            if t >= t0 && t < t1 {
                if v.path[0] == 0 {
                    n.0 += 1;
                } else {
                    n.1 += 1;
                }
            }
        }
    }
    n
}

#[test]
fn in_order_merge_gives_the_first_link_every_slot() {
    let sim = run(merge_scenario(0.6, 0.3, 0.3, 1500.0), MergeRule::InOrder);
    let (a, b) = passed(&sim, 600.0, 1200.0);
    assert!(a > 0 && (b as f64) < 0.05 * (a + b) as f64, "in-order shares {a}/{b}");
}

#[test]
fn capacity_merge_shares_the_bottleneck_by_incoming_capacity() {
    let sim = run(merge_scenario(0.6, 0.3, 0.3, 1500.0), MergeRule::CapacityProportional);
    let (a, b) = passed(&sim, 600.0, 1200.0);
    let share = a as f64 / (a + b) as f64;
    // C0 / (C0 + C1) = 1.0 / 1.5.
    assert!((share - 2.0 / 3.0).abs() < 0.02, "mainline share {share:.3} ({a}/{b}), expected 0.667");
    // The bottleneck still discharges at capacity: 0.3 veh/s × 600 s.
    assert!(((a + b) as f64 - 180.0).abs() <= 2.0, "throughput {}", a + b);
}

#[test]
fn capacity_merge_lets_a_lone_link_take_all_the_supply() {
    // Ramp demand below its fair share: the mainline takes the rest.
    let sim = run(merge_scenario(0.6, 0.05, 0.3, 1500.0), MergeRule::CapacityProportional);
    let (a, b) = passed(&sim, 600.0, 1200.0);
    assert!(((a + b) as f64 - 180.0).abs() <= 2.0, "throughput {}", a + b);
    assert!((b as f64 - 30.0).abs() <= 2.0, "the ramp passes all its 0.05 veh/s: {b}");
}

#[test]
fn capacity_merge_is_bit_identical_when_the_merge_never_binds() {
    // L2 is wide enough: no head ever waits on another link's account.
    let a = run(merge_scenario(0.2, 0.1, 2.0, 1500.0), MergeRule::InOrder);
    let b = run(merge_scenario(0.2, 0.1, 2.0, 1500.0), MergeRule::CapacityProportional);
    for (va, vb) in a.scenario.vehicles.iter().zip(&b.scenario.vehicles) {
        let bits = |t: &[f64]| t.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
        assert_eq!(bits(&va.node_times), bits(&vb.node_times), "vehicle {}", va.id);
    }
}

#[test]
fn capacity_merge_is_deterministic() {
    let a = run(merge_scenario(0.6, 0.3, 0.3, 1500.0), MergeRule::CapacityProportional);
    let b = run(merge_scenario(0.6, 0.3, 0.3, 1500.0), MergeRule::CapacityProportional);
    for (va, vb) in a.scenario.vehicles.iter().zip(&b.scenario.vehicles) {
        assert_eq!(va.node_times, vb.node_times);
    }
}

#[test]
fn merge_rule_reads_from_config() {
    assert_eq!(MergeRule::from_config(&serde_json::json!({})).unwrap(), MergeRule::InOrder);
    assert_eq!(MergeRule::from_config(&serde_json::json!({"merge": "capacity"})).unwrap(), MergeRule::CapacityProportional);
    assert!(MergeRule::from_config(&serde_json::json!({"merge": "zipper"})).is_err());
}
