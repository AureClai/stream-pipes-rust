//! Ramp metering with ALINEA, written outside the engine.
//!
//! The engine only knows signal groups, sensors and commands. The control
//! law lives here, in a type that implements `signals::Controller`, exactly
//! as a host crate (a "mod") would write it. Nothing in `src/` knows about
//! ALINEA.
//!
//! Network: a two-lane motorway (L0, 1 km) and a one-lane on-ramp (L1)
//! merge at node 2 into L2 (500 m), which ends in a lane drop (L3, one lane
//! of capacity). Mainline 1 500 veh/h + ramp 500 veh/h exceed
//! the bottleneck (1 800 veh/h): without control the queue fills L2 and
//! spills back onto both the mainline and the ramp. The merge shares its
//! outflow in proportion to the incoming capacities.
//!
//! ALINEA (Papageorgiou et al., 1991): every T seconds,
//!   r(k) = r(k−1) + K_R · (ô − o(k)),
//! with o the occupancy measured downstream of the merge (here L2, from
//! the engine's sensors) and ô a set point just below the critical
//! occupancy. The rate r is applied as one vehicle per green, a green every
//! 3600 / r seconds.
//!
//!   cargo run --release --no-default-features --example ramp_metering_alinea

use stream_core_rust::model::{FundamentalDiagram, Link, Node, NodeType, Scenario, Vehicle};
use stream_core_rust::signals::{
    Command, ControlView, Controller, Movement, OffState, SignalGroup, SignalNetwork,
};
use stream_core_rust::simulation::{MergeRule, Simulation};
use stream_core_rust::validation::Validate;

// ── The controller: outside the engine ────────────────────────────────────────

struct Alinea {
    group: String,
    /// Detector link downstream of the merge.
    detector: usize,
    /// Control period (s).
    period: f64,
    /// Set point and gain: occupancy in [0, 1], K_R in veh/h per unit.
    target: f64,
    gain: f64,
    rate: f64,
    min_rate: f64,
    max_rate: f64,
    /// Green duration of one metering cycle (s).
    green: f64,
    /// (time, occupancy, rate) per control step, for the report.
    history: std::sync::Arc<std::sync::Mutex<Vec<(f64, f64, f64)>>>,
}

impl Alinea {
    fn commands(&mut self, view: &ControlView) -> Vec<Command> {
        let t = view.time();
        let occupancy = view.occupancy(self.detector);
        self.rate = (self.rate + self.gain * (self.target - occupancy)).clamp(self.min_rate, self.max_rate);
        self.history.lock().unwrap().push((t, occupancy, self.rate));
        // One green every 3600 / r seconds over the next control period.
        let cycle = 3600.0 / self.rate;
        let mut out = Vec::new();
        let mut g = t;
        while g < t + self.period {
            out.push(Command::Switch { group: self.group.clone(), at: g, green: true });
            out.push(Command::Switch { group: self.group.clone(), at: g + self.green, green: false });
            g += cycle;
        }
        out.push(Command::WakeAt(t + self.period));
        out
    }
}

impl Controller for Alinea {
    fn start(&mut self, view: &ControlView) -> Vec<Command> {
        self.commands(view)
    }
    fn wake(&mut self, view: &ControlView) -> Vec<Command> {
        self.commands(view)
    }
}

// ── Scenario ──────────────────────────────────────────────────────────────────

const HORIZON: f64 = 3600.0;

fn node(id: usize, t: NodeType, inc: Vec<usize>, out: Vec<usize>) -> Node {
    Node { id, node_type: t, incoming_links: inc, outgoing_links: out, points: (id as f64, 0.0), signals: vec![], roundabout: None }
}

fn fd(u: f64, c: f64) -> FundamentalDiagram {
    FundamentalDiagram { u, w: 5.0, kx: 0.125, c }
}

fn scenario(metered: bool) -> Scenario {
    let links = vec![
        Link::new(0, 0, 2, 1000.0, 25.0, 2, 1.0, fd(25.0, 0.5), vec![]),
        Link::new(1, 1, 2, 300.0, 15.0, 1, 0.5, fd(15.0, 0.5), vec![]),
        Link::new(2, 2, 3, 500.0, 25.0, 2, 1.0, fd(25.0, 0.5), vec![]),
        Link::new(3, 3, 4, 1000.0, 25.0, 2, 0.5, fd(25.0, 0.25), vec![]),
    ];
    let nodes = vec![
        node(0, NodeType::Entry, vec![], vec![0]),
        node(1, NodeType::Entry, vec![], vec![1]),
        node(2, NodeType::Internal, vec![0, 1], vec![2]),
        node(3, NodeType::Internal, vec![2], vec![3]),
        node(4, NodeType::Exit, vec![3], vec![]),
    ];
    let mut vehicles = Vec::new();
    let (q_main, q_ramp) = (1500.0, 500.0);
    for (link, origin, q) in [(0usize, 0usize, q_main), (1, 1, q_ramp)] {
        let n = (q * HORIZON / 3600.0) as usize;
        for k in 0..n {
            let id = vehicles.len();
            vehicles.push(Vehicle::new(id, 0, vec![link, 2, 3], k as f64 * 3600.0 / q, origin, 4));
        }
    }
    let signals = if metered {
        SignalNetwork {
            groups: vec![SignalGroup {
                id: "ramp".into(),
                node: 2,
                movements: vec![Movement { from: 1, to: None }],
                vehicles_per_green: Some(1),
                off: OffState::Red,
            }],
            ..Default::default()
        }
    } else {
        SignalNetwork::default()
    };
    let s = Scenario {
        nodes,
        links,
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: HORIZON * 2.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
        signals,
        signal_log: Vec::new(),
    };
    s.validate().expect("valid");
    s
}

/// Mean trip time (s) of the vehicles entering by `link`, and how many.
fn mean_trip(s: &Scenario, link: usize) -> (f64, usize) {
    let t: Vec<f64> = s
        .vehicles
        .iter()
        .filter(|v| v.path[0] == link && v.node_times.len() == v.path.len() + 1)
        .map(|v| v.node_times.last().unwrap() - v.start_time)
        .collect();
    (t.iter().sum::<f64>() / t.len().max(1) as f64, t.len())
}

fn main() {
    let mut plain = Simulation::new(scenario(false));
    plain.merge_rule = MergeRule::CapacityProportional;
    plain.run().expect("run");

    let history = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let mut metered = Simulation::new(scenario(true));
    metered.merge_rule = MergeRule::CapacityProportional;
    // Critical occupancy of L2: k_c / kx = (C/u) / kx = (0.5/25) / 0.125 = 16 %.
    metered.add_controller(Box::new(Alinea {
        group: "ramp".into(),
        detector: 2,
        period: 60.0,
        target: 0.14,
        gain: 70.0 * 100.0, // 70 veh/h per % of occupancy
        rate: 900.0,
        min_rate: 200.0,
        max_rate: 1800.0,
        green: 1.0,
        history: history.clone(),
    }));
    metered.run().expect("run");

    for (name, sim) in [("no metering", &plain), ("ALINEA", &metered)] {
        let (main, n_main) = mean_trip(&sim.scenario, 0);
        let (ramp, n_ramp) = mean_trip(&sim.scenario, 1);
        let total = main * n_main as f64 + ramp * n_ramp as f64;
        println!(
            "{name:<12} | mainline {n_main:>5} veh, mean trip {main:>6.1} s | ramp {n_ramp:>4} veh, mean trip {ramp:>6.1} s | total {:>6.1} veh·h",
            total / 3600.0
        );
    }

    // The control loop: occupancy downstream of the merge against the set
    // point, and the metering rate it chose, every 10 minutes.
    println!("\n  t (s) | occupancy L2 | set point | ramp rate (veh/h)");
    for &(t, o, r) in history.lock().unwrap().iter().filter(|h| (h.0 as usize) % 600 == 0) {
        println!("{t:>7.0} | {:>11.1} % | {:>7.1} % | {r:>6.0}", o * 100.0, 14.0);
    }
}
