//! Signals, ramp metering, tolls and external controllers: closed-form checks.
//!
//! Layout: Entry 0 →[L0: approach, 100 m at 10 m/s, saturation flow
//! C = 0.5 veh/s (2 s headway), storage 15]→ node 1 →[L1: 100 m, free]→ Exit 2,
//! and →[L2: 100 m, free]→ Exit 3. Vehicles reach node 1 10 s after entering.

use stream_core_rust::model::{
    FundamentalDiagram, Link, Node, NodeType, PipeSpec, Scenario, Vehicle, VehicleState, ALL_CLASSES,
};
use stream_core_rust::signals::{
    Command, ControlView, Controller, FixedTimePlan, Movement, OffState, SignalGroup, SignalNetwork, Toll,
};
use stream_core_rust::simulation::Simulation;
use stream_core_rust::validation::Validate;

fn node(id: usize, t: NodeType, inc: Vec<usize>, out: Vec<usize>) -> Node {
    Node { id, node_type: t, incoming_links: inc, outgoing_links: out, points: (0.0, 0.0), signals: vec![], roundabout: None, junction: None }
}

fn base(l0: Link, trips: &[(usize, f64)]) -> Scenario {
    let free = FundamentalDiagram { u: 10.0, w: 5.0, kx: 10.0, c: 10.0 };
    Scenario {
        nodes: vec![
            node(0, NodeType::Entry, vec![], vec![0]),
            node(1, NodeType::Internal, vec![0], vec![1, 2]),
            node(2, NodeType::Exit, vec![1], vec![]),
            node(3, NodeType::Exit, vec![2], vec![]),
        ],
        links: vec![
            l0,
            Link::new(1, 1, 2, 100.0, 10.0, 1, 10.0, free.clone(), vec![]),
            Link::new(2, 1, 3, 100.0, 10.0, 1, 10.0, free, vec![]),
        ],
        vehicles: trips
            .iter()
            .enumerate()
            .map(|(i, &(out, t))| Vehicle::new(i, 0, vec![0, out], t, 0, out + 1))
            .collect(),
        demand: vec![],
        start_time: 0.0,
        duration: 100_000.0,
        classes: vec!["car".to_string(), "bus".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
        signals: Default::default(),
        signal_log: Vec::new(),
    }
}

fn approach() -> Link {
    Link::new(0, 0, 1, 100.0, 10.0, 1, 0.5, FundamentalDiagram { u: 10.0, w: 5.0, kx: 0.15, c: 0.5 }, vec![])
}

fn group(id: &str, movements: &[(usize, Option<usize>)]) -> SignalGroup {
    SignalGroup {
        id: id.into(),
        node: 1,
        movements: movements.iter().map(|&(from, to)| Movement { from, to }).collect(),
        vehicles_per_green: None,
        off: OffState::Green,
    }
}

fn plan(cycle: f64, greens: &[(&str, &[[f64; 2]])]) -> FixedTimePlan {
    FixedTimePlan {
        name: None,
        from: 0.0,
        until: 100_000.0,
        cycle,
        offset: 0.0,
        groups: vec![],
        greens: greens.iter().map(|(g, w)| (g.to_string(), w.to_vec())).collect(),
    }
}

fn run(s: Scenario) -> Simulation {
    s.validate().expect("valid");
    let mut sim = Simulation::new(s);
    sim.run().expect("run");
    sim
}

fn passages(sim: &Simulation) -> Vec<f64> {
    let mut t: Vec<f64> = sim.scenario.vehicles.iter().map(|v| v.node_times[1]).collect();
    t.sort_by(f64::total_cmp);
    t
}

fn all_exited(sim: &Simulation) {
    for v in &sim.scenario.vehicles {
        assert_eq!(v.state, VehicleState::Exited, "vehicle {} did not exit", v.id);
    }
}

fn close(a: f64, b: f64, msg: &str) {
    assert!((a - b).abs() < 1e-9, "{msg}: expected {b}, got {a}");
}

/// Cycle 60 s, green [0, 20): a vehicle arriving on green passes at once;
/// the queue formed on red leaves at green start at the saturation flow
/// (2 s headway): ceil(g·s) = 10 vehicles per green.
#[test]
fn red_holds_and_green_discharges_at_saturation_flow() {
    let mut s = base(approach(), &(0..40).map(|_| (1, 0.0)).collect::<Vec<_>>());
    s.signals = SignalNetwork {
        groups: vec![group("main", &[(0, Some(1))])],
        plans: vec![plan(60.0, &[("main", &[[0.0, 20.0]])])],
        tolls: vec![],
    };
    let sim = run(s);
    all_exited(&sim);
    let t = passages(&sim);
    // Arrivals at node 1 every 2 s from 10 s: 10, 12, …, 18 pass on green.
    for (k, &x) in t[..5].iter().enumerate() {
        close(x, 10.0 + 2.0 * k as f64, "first green");
    }
    // Then 10 per green at 60 + 2k, 120 + 2k, …
    for (k, &x) in t[5..35].iter().enumerate() {
        let cycle = 60.0 * (1 + k / 10) as f64;
        close(x, cycle + 2.0 * (k % 10) as f64, &format!("vehicle {} of the queue", k));
    }
}

/// Ramp metering: one vehicle per green, cycle 6 s ⇒ 600 veh/h exactly.
#[test]
fn ramp_meter_lets_one_vehicle_per_green() {
    let mut s = base(approach(), &(0..20).map(|_| (1, 0.0)).collect::<Vec<_>>());
    let mut g = group("ramp", &[(0, None)]);
    g.vehicles_per_green = Some(1);
    s.signals = SignalNetwork { groups: vec![g], plans: vec![plan(6.0, &[("ramp", &[[0.0, 2.0]])])], tolls: vec![] };
    let sim = run(s);
    all_exited(&sim);
    let t = passages(&sim);
    // First arrival at 10 s, during the green [6, 8)? No: 10 is red ⇒ 12.
    close(t[0], 12.0, "first metered vehicle");
    for w in t.windows(2) {
        close(w[1] - w[0], 6.0, "metered headway");
    }
}

/// A red right turn in a shared lane holds the through vehicles behind it;
/// with a dedicated pocket (its own pipe) the through traffic flows.
#[test]
fn red_turn_blocks_a_shared_lane_not_a_pocket() {
    // Right turn = L0 → L2, red over [0, 100); through = L0 → L1 uncontrolled.
    let signals = |_: ()| SignalNetwork {
        groups: vec![group("right", &[(0, Some(2))])],
        plans: vec![FixedTimePlan { from: 0.0, until: 100.0, ..plan(100.0, &[]) }],
        tolls: vec![],
    };
    let mut plan_red = signals(());
    plan_red.plans[0].groups = vec!["right".into()];
    let trips = [(2, 0.0), (1, 0.5), (1, 1.0)];

    // Shared lane: the through vehicles wait behind the right-turner.
    let mut shared = base(approach(), &trips);
    shared.signals = plan_red.clone();
    let sim = run(shared);
    all_exited(&sim);
    close(sim.scenario.vehicles[0].node_times[1], 100.0, "right turn released at green");
    assert!(sim.scenario.vehicles[1].node_times[1] >= 100.0, "through held behind it (FIFO)");

    // Pocket: two pipes, the right turn has its own.
    let mut l0 = Link::new(0, 0, 1, 100.0, 10.0, 2, 1.0, FundamentalDiagram { u: 10.0, w: 5.0, kx: 0.15, c: 0.5 }, vec![]);
    l0.pipe_specs = vec![PipeSpec { lanes: 1, class_mask: ALL_CLASSES }, PipeSpec { lanes: 1, class_mask: ALL_CLASSES }];
    l0.moves.insert(2, vec![0]);
    l0.moves.insert(1, vec![1]);
    let mut pocket = base(l0, &trips);
    pocket.signals = plan_red;
    let sim = run(pocket);
    all_exited(&sim);
    close(sim.scenario.vehicles[0].node_times[1], 100.0, "right turn released at green");
    for v in &sim.scenario.vehicles[1..] {
        close(v.node_times[1], v.node_times[0] + 10.0, "through vehicle in its own pipe");
    }
}

/// Outside its plan period a group is in its `off` state.
#[test]
fn off_state_outside_plan_periods() {
    // Red over [100, 200) only; green (dark signal) otherwise.
    let mut s = base(approach(), &[(1, 0.0), (1, 95.0), (1, 300.0)]);
    s.signals = SignalNetwork {
        groups: vec![group("g", &[(0, None)])],
        plans: vec![FixedTimePlan { from: 100.0, until: 200.0, groups: vec!["g".into()], ..plan(100.0, &[]) }],
        tolls: vec![],
    };
    let sim = run(s);
    let v = &sim.scenario.vehicles;
    close(v[0].node_times[1], 10.0, "before the plan");
    close(v[1].node_times[1], 200.0, "arrives at 105, red until 200");
    close(v[2].node_times[1], 310.0, "after the plan");
    // The off state Red keeps it red outside the plan.
    let mut s = base(approach(), &[(1, 0.0)]);
    let mut g = group("g", &[(0, None)]);
    g.off = OffState::Red;
    s.signals = SignalNetwork {
        groups: vec![g],
        plans: vec![FixedTimePlan { from: 100.0, until: 200.0, ..plan(100.0, &[("g", &[[0.0, 50.0]])]) }],
        tolls: vec![],
    };
    let sim = run(s);
    close(sim.scenario.vehicles[0].node_times[1], 100.0, "dark-red until the plan's first green");
}

/// Toll: departure = max(arrival, previous departure) + service, per class.
#[test]
fn toll_lane_serves_one_vehicle_at_a_time() {
    let mut s = base(approach(), &[(1, 0.0), (1, 0.0), (1, 0.0), (1, 100.0)]);
    s.vehicles[1].class_id = 1; // a bus: 4 s
    s.signals = SignalNetwork {
        groups: vec![],
        plans: vec![],
        tolls: vec![Toll { link: 0, pipes: None, service_time: 12.0, by_class: [("bus".to_string(), 4.0)].into_iter().collect() }],
    };
    let sim = run(s);
    all_exited(&sim);
    let v = &sim.scenario.vehicles;
    // Arrivals at the booth: 10, 12, 14, 110.
    close(v[0].node_times[1], 22.0, "car: 10 + 12");
    close(v[1].node_times[1], 26.0, "bus: max(12, 22) + 4");
    close(v[2].node_times[1], 38.0, "car: max(14, 26) + 12");
    close(v[3].node_times[1], 122.0, "idle booth: 110 + 12");
}

/// An external controller (outside the engine) drives a group: green for
/// 10 s every 30 s, through commands and wake-ups only.
struct Alternating {
    period: f64,
    green: f64,
}

impl Controller for Alternating {
    fn start(&mut self, view: &ControlView) -> Vec<Command> {
        self.wake(view)
    }
    fn wake(&mut self, view: &ControlView) -> Vec<Command> {
        let t = view.time();
        vec![
            Command::Switch { group: "ext".into(), at: t, green: true },
            Command::Switch { group: "ext".into(), at: t + self.green, green: false },
            Command::WakeAt(t + self.period),
        ]
    }
}

#[test]
fn external_controller_drives_a_group() {
    let mut s = base(approach(), &(0..12).map(|_| (1, 0.0)).collect::<Vec<_>>());
    let mut g = group("ext", &[(0, None)]);
    g.off = OffState::Red;
    s.signals = SignalNetwork { groups: vec![g], plans: vec![], tolls: vec![] };
    s.validate().unwrap();
    let mut sim = Simulation::new(s);
    sim.add_controller(Box::new(Alternating { period: 30.0, green: 10.0 }));
    sim.run().unwrap();
    all_exited(&sim);
    let t = passages(&sim);
    // Greens [0,10), [30,40), [60,70)…; arrivals from 10 s on.
    let expected = [30.0, 32.0, 34.0, 36.0, 38.0, 60.0, 62.0, 64.0, 66.0, 68.0, 90.0, 92.0];
    for (x, e) in t.iter().zip(expected) {
        close(*x, e, "controller-driven discharge");
    }
    // The applied switches are logged.
    let log = &sim.scenario.signal_log;
    assert!(log.iter().any(|r| r.time == 30.0 && r.green));
    assert!(log.iter().any(|r| r.time == 40.0 && !r.green));
}

/// Controllers read the sensors: vehicles, queue, cumulative counts.
#[test]
fn controller_sensors_count_vehicles() {
    struct Probe(std::sync::Arc<std::sync::Mutex<Vec<(f64, usize, usize, u64, u64)>>>);
    impl Controller for Probe {
        fn start(&mut self, _: &ControlView) -> Vec<Command> {
            vec![Command::WakeAt(15.0)]
        }
        fn wake(&mut self, v: &ControlView) -> Vec<Command> {
            self.0.lock().unwrap().push((v.time(), v.vehicles(0), v.queue(0), v.entered(0), v.exited(0)));
            vec![Command::WakeAt(v.time() + 50.0)]
        }
    }
    let mut s = base(approach(), &(0..5).map(|_| (1, 0.0)).collect::<Vec<_>>());
    let mut g = group("g", &[(0, None)]);
    g.off = OffState::Red;
    s.signals = SignalNetwork { groups: vec![g], plans: vec![], tolls: vec![] };
    let log = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let mut sim = Simulation::new(s);
    sim.add_controller(Box::new(Probe(log.clone())));
    sim.scenario.duration = 100.0;
    sim.run().unwrap();
    let log = log.lock().unwrap();
    // At 15 s: 5 entered (0, 2, 4, 6, 8), all on L0, 3 queued (arrived at
    // 10, 12, 14), none left (red forever).
    assert_eq!(log[0], (15.0, 5, 3, 5, 0));
}

/// Scenario checks: unknown group in a plan, movement through a link that
/// does not reach the node, two groups on one movement.
#[test]
fn invalid_signal_networks_are_rejected() {
    let mut s = base(approach(), &[]);
    s.signals = SignalNetwork { groups: vec![group("g", &[(0, None)])], plans: vec![plan(60.0, &[("h", &[[0.0, 10.0]])])], tolls: vec![] };
    assert!(s.validate().is_err(), "unknown group in a plan");
    s.signals = SignalNetwork { groups: vec![group("g", &[(1, None)])], plans: vec![], tolls: vec![] };
    assert!(s.validate().is_err(), "link 1 does not end at node 1");
    s.signals = SignalNetwork { groups: vec![group("g", &[(0, None)]), group("h", &[(0, Some(1))])], plans: vec![], tolls: vec![] };
    assert!(s.validate().is_err(), "two groups on 0 → 1");
    s.signals = SignalNetwork { groups: vec![group("g", &[(0, Some(1))])], plans: vec![plan(60.0, &[("g", &[[50.0, 120.0]])])], tolls: vec![] };
    assert!(s.validate().is_err(), "green longer than the cycle");
}

/// Without signals nothing changes: the empty network is skipped.
#[test]
fn no_signals_no_change() {
    let trips: Vec<_> = (0..30).map(|k| (1 + k % 2, k as f64 * 0.5)).collect();
    let a = run(base(approach(), &trips));
    let mut s = base(approach(), &trips);
    s.signals = SignalNetwork::default();
    let b = run(s);
    for (x, y) in a.scenario.vehicles.iter().zip(&b.scenario.vehicles) {
        assert_eq!(x.node_times, y.node_times);
    }
}

/// `verify` reports every signal group: no passage on red, cap respected.
#[test]
fn verification_reports_signal_groups() {
    use stream_core_rust::verification::{verify_scenario, CheckStatus, ElementType};
    let mut s = base(approach(), &(0..40).map(|_| (1, 0.0)).collect::<Vec<_>>());
    let mut g = group("main", &[(0, Some(1))]);
    g.vehicles_per_green = Some(8);
    s.signals = SignalNetwork { groups: vec![g], plans: vec![plan(60.0, &[("main", &[[0.0, 20.0]])])], tolls: vec![] };
    let sim = run(s);
    let report = verify_scenario(&sim.scenario, 300.0);
    let el = report.elements.iter().find(|e| e.element_type == ElementType::Signal).expect("signal element");
    assert_eq!(el.checks.len(), 2);
    for c in &el.checks {
        assert_eq!(c.status, CheckStatus::Pass, "{}: {}", c.name, c.detail);
    }
    // Corrupt one passage onto the red: the check must fail.
    let mut bad = sim.scenario.clone();
    let v = bad.vehicles.iter_mut().find(|v| v.node_times[1] > 60.0).unwrap();
    v.node_times[1] = 45.0;
    let report = verify_scenario(&bad, 300.0);
    let el = report.elements.iter().find(|e| e.element_type == ElementType::Signal).unwrap();
    assert_eq!(el.checks[0].status, CheckStatus::Fail);
}
