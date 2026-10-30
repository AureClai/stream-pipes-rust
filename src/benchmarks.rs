//! Analytic benchmark suite — canonical LWR situations with closed-form solutions.
//!
//! Each case builds a minimal scenario whose exact outcome is derivable by hand
//! from kinematic wave theory, runs the engine on it, and reports measured vs
//! analytic. This validates the *methodology* itself: if the engine drifts from
//! LWR on these primitives, every larger simulation inherits the error.
//!
//! Cases:
//! 1. **free_flow_transit** — one vehicle on an empty two-link corridor.
//!    Travel time = Σ Lᵢ/uᵢ exactly.
//! 2. **bottleneck_discharge** — N vehicles released at once upstream of a
//!    bottleneck of capacity C. The n-th vehicle passes the bottleneck at
//!    t = L/u + n/C (server never idles), so the last passes at L/u + (N−1)/C.
//! 3. **queue_delay** — arrivals at rate λ > C. Vehicle n arrives at the
//!    bottleneck at n/λ + L/u but passes at L/u + n/C; its queueing delay is
//!    n·(1/C − 1/λ). Total delay = Σ = (1/C − 1/λ)·N(N−1)/2.
//! 4. **spillback_wave** — a storage-limited link at jam: the (dn+1)-th entry
//!    is enabled by the first exit and must lag it by exactly L/w.
//! 5. **merge_throughput** — two entry branches feeding one bottleneck: the
//!    combined discharge rate equals C (capacity is conserved through the merge).
//!
//! The later cases cover pipes, friction and the roundabout node
//! (`roundabout_*`: ring free flow, entry capacity without conflict, Siegloch
//! gap acceptance in a regular circulating stream, spillback storage on the
//! ring).

use crate::model::{FundamentalDiagram, Link, Node, NodeType, PipeSpec, Scenario, Vehicle};
use crate::simulation::Simulation;
use crate::verification::CheckStatus;
use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

/// Relative error bounds. The engine is deterministic and event-exact: on
/// these cases its answer and the closed-form value are the *same real
/// number*, so the only admissible difference is floating-point
/// representation. The pass bound is therefore set at the scale of accumulated
/// rounding rather than at a modelling tolerance — a deviation the engine
/// could not have produced by rounding is a defect, however small it looks.
/// Measured worst case across the suite is 1 ulp (1.7e-16 relative), on
/// `friction_throttle`; the other eight agree bit-for-bit.
const PASS_PCT: f64 = 1e-10; // 1e-12 relative
/// Above the exactness bound and below this, a benchmark is not "accurate
/// enough": it is an undiagnosed defect. The separate status exists to aid
/// bisection, not to license the deviation — both Warn and Fail break the
/// build (see `all_benchmarks_are_exact` and the CLI exit status).
const WARN_PCT: f64 = 5.0;

#[derive(Debug, Serialize, Deserialize)]
pub struct BenchmarkResult {
    pub id: String,
    pub name: String,
    /// What the case sets up and what theory predicts.
    pub description: String,
    pub analytic: f64,
    pub measured: f64,
    pub unit: String,
    pub error_pct: f64,
    pub status: CheckStatus,
}

fn result(
    id: &str,
    name: &str,
    description: &str,
    analytic: f64,
    measured: f64,
    unit: &str,
) -> BenchmarkResult {
    let error_pct = if analytic.abs() > 1e-12 {
        ((measured - analytic) / analytic).abs() * 100.0
    } else {
        (measured - analytic).abs() * 100.0
    };
    let status = if error_pct < PASS_PCT {
        CheckStatus::Pass
    } else if error_pct < WARN_PCT {
        CheckStatus::Warn
    } else {
        CheckStatus::Fail
    };
    BenchmarkResult {
        id: id.to_string(),
        name: name.to_string(),
        description: description.to_string(),
        analytic,
        measured,
        unit: unit.to_string(),
        error_pct,
        status,
    }
}

// ── Scenario builders ─────────────────────────────────────────────────────────

fn node(id: usize, node_type: NodeType, incoming: Vec<usize>, outgoing: Vec<usize>) -> Node {
    Node {
        id,
        node_type,
        incoming_links: incoming,
        outgoing_links: outgoing,
        points: (id as f64, 0.0),
        signals: vec![], roundabout: None, junction: None,
    }
}

/// Entry(0) →L0→ Internal(1) →L1→ Exit(2).
/// `c1` is the bottleneck capacity (veh/s); `kx0` the per-lane jam density of L0.
fn corridor(kx0: f64, c1: f64, starts: &[f64]) -> Scenario {
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
    let vehicles = starts
        .iter()
        .enumerate()
        .map(|(i, &t)| Vehicle::new(i, 0, vec![0, 1], t, 0, 2))
        .collect();
    Scenario {
        nodes: vec![
            node(0, NodeType::Entry, vec![], vec![0]),
            node(1, NodeType::Internal, vec![0], vec![1]),
            node(2, NodeType::Exit, vec![1], vec![]),
        ],
        links: vec![
            Link::new(0, 0, 1, 100.0, 10.0, 2, 10.0, fd0, vec![]),
            Link::new(1, 1, 2, 100.0, 10.0, 1, c1, fd1, vec![]),
        ],
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 1_000_000.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(), signals: Default::default(), signal_log: Vec::new(),
    }
}

/// Two entries (0, 1) →L0/L1→ merge node (2) →L2 (bottleneck)→ Exit(3).
fn merge(c_bottleneck: f64, per_branch: usize) -> Scenario {
    let fd_feed = FundamentalDiagram {
        u: 10.0,
        w: 5.0,
        kx: 10.0,
        c: 10.0,
    };
    let fd_out = FundamentalDiagram {
        u: 10.0,
        w: 5.0,
        kx: 10.0,
        c: c_bottleneck,
    };
    let mut vehicles = Vec::new();
    for _ in 0..per_branch {
        let id = vehicles.len();
        vehicles.push(Vehicle::new(id, 0, vec![0, 2], 0.0, 0, 3));
        let id = vehicles.len();
        vehicles.push(Vehicle::new(id, 0, vec![1, 2], 0.0, 1, 3));
    }
    Scenario {
        nodes: vec![
            node(0, NodeType::Entry, vec![], vec![0]),
            node(1, NodeType::Entry, vec![], vec![1]),
            node(2, NodeType::Internal, vec![0, 1], vec![2]),
            node(3, NodeType::Exit, vec![2], vec![]),
        ],
        links: vec![
            Link::new(0, 0, 2, 100.0, 10.0, 2, 10.0, fd_feed.clone(), vec![]),
            Link::new(1, 1, 2, 100.0, 10.0, 2, 10.0, fd_feed, vec![]),
            Link::new(2, 2, 3, 100.0, 10.0, 1, c_bottleneck, fd_out, vec![]),
        ],
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 1_000_000.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(), signals: Default::default(), signal_log: Vec::new(),
    }
}

fn run(scenario: Scenario) -> Result<Scenario> {
    let mut sim = Simulation::new(scenario);
    sim.run().context("benchmark simulation failed")?;
    Ok(sim.scenario)
}

// ── Cases ─────────────────────────────────────────────────────────────────────

fn case_free_flow_transit() -> Result<BenchmarkResult> {
    let s = run(corridor(10.0, 10.0, &[0.0]))?;
    let v = &s.vehicles[0];
    let measured = v.node_times.last().copied().unwrap_or(f64::NAN) - v.node_times[0];
    // L0/u + L1/u = 100/10 + 100/10
    Ok(result(
        "free_flow_transit",
        "Free-flow transit time",
        "One vehicle on an empty 2×100 m corridor at u = 10 m/s. Theory: Σ L/u = 20 s.",
        20.0,
        measured,
        "s",
    ))
}

fn case_bottleneck_discharge() -> Result<BenchmarkResult> {
    let n = 30usize;
    let c = 0.1;
    let starts = vec![0.0; n];
    let s = run(corridor(10.0, c, &starts))?;
    // Last vehicle's passage at the bottleneck node (entry into L1).
    let measured = s
        .vehicles
        .iter()
        .filter_map(|v| v.node_times.get(1).copied())
        .fold(f64::NEG_INFINITY, f64::max);
    let analytic = 100.0 / 10.0 + (n as f64 - 1.0) / c;
    Ok(result(
        "bottleneck_discharge",
        "Bottleneck discharge timing",
        "30 vehicles released at once upstream of a C = 0.1 veh/s bottleneck. \
         Theory: n-th vehicle passes at L/u + n/C; last at 10 + 290 = 300 s.",
        analytic,
        measured,
        "s",
    ))
}

fn case_queue_delay() -> Result<BenchmarkResult> {
    let n = 20usize;
    let lambda = 0.2; // veh/s arrivals
    let c = 0.1; // bottleneck
    let starts: Vec<f64> = (0..n).map(|i| i as f64 / lambda).collect();
    let s = run(corridor(10.0, c, &starts))?;
    // Total queueing delay at the bottleneck vs free-flow arrival.
    let measured: f64 = s
        .vehicles
        .iter()
        .filter_map(|v| {
            let t_pass = v.node_times.get(1)?;
            let t_free = v.node_times.first()? + 100.0 / 10.0;
            Some((t_pass - t_free).max(0.0))
        })
        .sum();
    let analytic = (1.0 / c - 1.0 / lambda) * (n as f64) * (n as f64 - 1.0) / 2.0;
    Ok(result(
        "queue_delay",
        "Deterministic queueing delay",
        "Arrivals at λ = 0.2 veh/s into a C = 0.1 veh/s bottleneck. \
         Theory: vehicle n waits n(1/C − 1/λ); total = 5·N(N−1)/2 = 950 s.",
        analytic,
        measured,
        "s",
    ))
}

fn case_spillback_wave() -> Result<BenchmarkResult> {
    // L0: kx 0.02 × 100 m × 2 lanes → dn = 4 vehicles, wave delay = L/w = 20 s.
    let n = 6usize;
    let s = run(corridor(0.02, 0.01, &vec![0.0; n]))?;
    let dn = 4usize;
    // Entry of vehicle dn is enabled by the first bottleneck passage (first
    // exit of L0), which occurs at L/u = 10 s. Measured lag = t_entry − t_exit.
    let first_exit = s
        .vehicles
        .iter()
        .filter_map(|v| v.node_times.get(1).copied())
        .fold(f64::INFINITY, f64::min);
    let mut entries: Vec<f64> = s
        .vehicles
        .iter()
        .filter_map(|v| v.node_times.first().copied())
        .collect();
    entries.sort_by(|a, b| a.total_cmp(b));
    let measured = entries.get(dn).map_or(f64::NAN, |t| t - first_exit);
    Ok(result(
        "spillback_wave",
        "Spillback backward wave",
        "A 4-slot link at jam: the 5th entry is enabled by the 1st exit and must \
         lag it by the backward wave travel time L/w = 100/5 = 20 s.",
        20.0,
        measured,
        "s",
    ))
}

fn case_fractional_storage_wave() -> Result<BenchmarkResult> {
    // The only case with NON-INTEGER storage, and the only one that can tell
    // frac/C from frac·C.
    //
    // L0: kx 0.0125 × 100 m × 2 lanes → 2.5 vehicles of storage, which a
    // discrete solver must round up to 3 slots. Rounding up hands the pipe
    // half a vehicle of room the continuum never granted it, so each slot
    // release has to be held back by the time that half-vehicle needs to
    // clear at capacity.
    //
    // Derived from Newell's continuous supply bound, independently of the
    // engine's own expression:
    //     t_in(n) ≥ t_out(n − kx·L·Λ) + L/w,        kx·L·Λ = 2.5 veh
    // whose discrete form can only index whole vehicles, n − ⌈kx·L·Λ⌉, and so
    // reads the downstream count frac = ⌈2.5⌉ − 2.5 = 0.5 vehicles too early.
    // At the discharge rate C those 0.5 vehicles occupy frac/C = 0.5/10 =
    // 0.05 s. Expected lag = L/w + frac/C = 100/5 + 0.05 = 20.05 s.
    //
    // The sign and the dimension are what this pins. The correction is a time,
    // so it divides by C; multiplying instead would give 100/5 + 0.5·10 = 25 s,
    // a 24.7% error here but exactly 0 in every integer-storage case — which is
    // how the defect survived the rest of this suite.
    let n = 6usize;
    let s = run(corridor(0.0125, 0.01, &vec![0.0; n]))?;
    let dn = 3usize; // ⌈2.5⌉
    let first_exit = s
        .vehicles
        .iter()
        .filter_map(|v| v.node_times.get(1).copied())
        .fold(f64::INFINITY, f64::min);
    let mut entries: Vec<f64> = s
        .vehicles
        .iter()
        .filter_map(|v| v.node_times.first().copied())
        .collect();
    entries.sort_by(|a, b| a.total_cmp(b));
    let measured = entries.get(dn).map_or(f64::NAN, |t| t - first_exit);
    Ok(result(
        "fractional_storage_wave",
        "Fractional-slot backward wave",
        "A link holding 2.5 vehicles, discretised to 3 slots. Theory: the 4th \
         entry lags the 1st exit by L/w plus the fractional slot's discharge \
         time, 100/5 + 0.5/10 = 20.05 s.",
        20.05,
        measured,
        "s",
    ))
}

fn case_merge_throughput() -> Result<BenchmarkResult> {
    let c = 0.1;
    let per_branch = 20usize;
    let s = run(merge(c, per_branch))?;
    // Passage times into the bottleneck link (index 1 of each path).
    let mut passages: Vec<f64> = s
        .vehicles
        .iter()
        .filter_map(|v| v.node_times.get(1).copied())
        .collect();
    passages.sort_by(|a, b| a.total_cmp(b));
    let measured = if passages.len() >= 2 {
        (passages.len() as f64 - 1.0) / (passages.last().unwrap() - passages.first().unwrap())
    } else {
        f64::NAN
    };
    Ok(result(
        "merge_throughput",
        "Merge throughput conservation",
        "Two saturated branches merging into a C = 0.1 veh/s bottleneck. \
         Theory: combined discharge rate equals the bottleneck capacity.",
        c,
        measured,
        "veh/s",
    ))
}

// ── Pipe-stream cases ─────────────────────────────────────────────────────────

/// Two-link corridor where both links have a car-only GP pipe (2 lanes) and a
/// bus-only pipe (1 lane). fd.c is tiny, so consecutive cars queue massively;
/// buses are spaced far apart, so no constraint ever binds for them.
fn bus_lane_scenario() -> Scenario {
    let fd = FundamentalDiagram {
        u: 10.0,
        w: 5.0,
        kx: 0.1,
        c: 0.01,
    };
    let pipe_specs = vec![
        PipeSpec {
            lanes: 2,
            class_mask: 0b01,
        }, // GP: cars only
        PipeSpec {
            lanes: 1,
            class_mask: 0b10,
        }, // bus lane
    ];
    let mut l0 = Link::new(0, 0, 1, 100.0, 10.0, 3, 0.03, fd.clone(), vec![]);
    let mut l1 = Link::new(1, 1, 2, 100.0, 10.0, 3, 0.03, fd, vec![]);
    l0.pipe_specs = pipe_specs.clone();
    l1.pipe_specs = pipe_specs;

    // 40 cars at 2 s headways (saturating), 3 buses spaced 200 s.
    let mut vehicles: Vec<Vehicle> = (0..40)
        .map(|i| Vehicle::new(i, 0, vec![0, 1], i as f64 * 2.0, 0, 2))
        .collect();
    for (k, t) in [100.0, 300.0, 500.0].iter().enumerate() {
        let mut bus = Vehicle::new(40 + k, 1, vec![0, 1], *t, 0, 2);
        bus.class_id = 1;
        vehicles.push(bus);
    }

    Scenario {
        nodes: vec![
            node(0, NodeType::Entry, vec![], vec![0]),
            node(1, NodeType::Internal, vec![0], vec![1]),
            node(2, NodeType::Exit, vec![1], vec![]),
        ],
        links: vec![l0, l1],
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 1_000_000.0,
        classes: vec!["car".to_string(), "bus".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(), signals: Default::default(), signal_log: Vec::new(),
    }
}

fn case_bus_lane_immunity() -> Result<BenchmarkResult> {
    let s = run(bus_lane_scenario())?;

    // Guard: the GP pipes must actually be congested, otherwise immunity
    // is vacuous. Worst car delay (including the entry queue: arrival at the
    // destination minus scheduled start minus free-flow time) must be large.
    let worst_car_trip = s
        .vehicles
        .iter()
        .filter(|v| v.class_id == 0 && v.node_times.len() == 3)
        .map(|v| v.node_times[2] - v.start_time - 20.0)
        .fold(0.0f64, f64::max);
    if worst_car_trip < 200.0 {
        return Ok(result(
            "bus_lane_immunity",
            "Bus lane immunity",
            "INVALID CASE: the car queue failed to form, immunity is vacuous.",
            20.0,
            f64::NAN,
            "s",
        ));
    }

    // Every bus trip must be exactly free flow: 2 x L/u = 20 s.
    let worst_bus_trip = s
        .vehicles
        .iter()
        .filter(|v| v.class_id == 1)
        .map(|v| v.node_times.last().copied().unwrap_or(f64::NAN) - v.node_times[0])
        .fold(f64::NEG_INFINITY, f64::max);
    Ok(result(
        "bus_lane_immunity",
        "Bus lane immunity",
        "Cars saturate the 2-lane GP pipes (worst car trip > 200 s) while the \
         bus pipe stays empty. Theory: every bus traverses at exactly 2xL/u = 20 s.",
        20.0,
        worst_bus_trip,
        "s",
    ))
}

/// Shoulder-exit diverge: L0 = [exit pipe (1 lane), through pipes (2 lanes)]
/// with movement restrictions; the exit link is a near-blocked bottleneck.
/// `phi` is the friction coefficient of L0 (1.0 = coupling disabled).
fn shoulder_exit_scenario(phi: f64) -> Scenario {
    let fd0 = FundamentalDiagram {
        u: 10.0,
        w: 5.0,
        kx: 0.02,
        c: 5.0,
    };
    let fd_out = FundamentalDiagram {
        u: 10.0,
        w: 5.0,
        kx: 10.0,
        c: 10.0,
    };
    let mut l0 = Link::new(0, 0, 1, 100.0, 10.0, 3, 15.0, fd0, vec![]);
    l0.pipe_specs = vec![
        PipeSpec {
            lanes: 1,
            class_mask: u64::MAX,
        }, // shoulder / exit pipe
        PipeSpec {
            lanes: 2,
            class_mask: u64::MAX,
        }, // through pipes
    ];
    l0.moves.insert(2, vec![0]); // exit reachable only from the shoulder pipe
    l0.moves.insert(1, vec![1]); // through traffic keeps left pipes
    l0.friction = phi;
    let mut l2 = Link::new(2, 1, 3, 100.0, 10.0, 1, 0.01, fd_out.clone(), vec![]);
    l2.fd.c = 0.01; // blocked off-ramp: 100 s headway

    // 4 exit-bound vehicles at t = 0 (shoulder pipe storage = 2), then
    // 4 through vehicles.
    let mut vehicles: Vec<Vehicle> = (0..4)
        .map(|i| Vehicle::new(i, 0, vec![0, 2], 0.0, 0, 3))
        .collect();
    vehicles.extend((4..8).map(|i| Vehicle::new(i, 0, vec![0, 1], 0.0, 0, 2)));

    Scenario {
        nodes: vec![
            node(0, NodeType::Entry, vec![], vec![0]),
            node(1, NodeType::Internal, vec![0], vec![1, 2]),
            node(2, NodeType::Exit, vec![1], vec![]),
            node(3, NodeType::Exit, vec![2], vec![]),
        ],
        links: vec![
            l0,
            Link::new(1, 1, 2, 100.0, 10.0, 2, 20.0, fd_out, vec![]),
            l2,
        ],
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 1_000_000.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(), signals: Default::default(), signal_log: Vec::new(),
    }
}

fn case_shoulder_exit_wave() -> Result<BenchmarkResult> {
    let s = run(shoulder_exit_scenario(1.0))?;
    // Shoulder pipe stores 2 vehicles; vehicle 2's entry is enabled by the
    // first off-ramp discharge (t = L/u = 10) plus the backward wave L/w = 20.
    let measured = s.vehicles[2]
        .node_times
        .first()
        .copied()
        .unwrap_or(f64::NAN);
    Ok(result(
        "shoulder_exit_wave",
        "Shoulder-exit spillback wave",
        "Blocked 1-lane off-ramp: the exit queue occupies only the shoulder \
         pipe (storage 2). Theory: the 3rd exit-bound vehicle enters at \
         first discharge (10 s) + L/w (20 s) = 30 s.",
        30.0,
        measured,
        "s",
    ))
}

fn case_shoulder_exit_through() -> Result<BenchmarkResult> {
    let s = run(shoulder_exit_scenario(1.0))?;
    // Through vehicles are unaffected by the shoulder jam: worst traversal of
    // L0 must be exactly L/u = 10 s (partial FIFO via pipes, phi = 1 here).
    let measured = s
        .vehicles
        .iter()
        .filter(|v| v.path.get(1) == Some(&1) && v.node_times.len() >= 2)
        .map(|v| v.node_times[1] - v.node_times[0])
        .fold(f64::NEG_INFINITY, f64::max);
    Ok(result(
        "shoulder_exit_through",
        "Through traffic past a blocked exit",
        "Same diverge: through vehicles use the 2-lane pipes and must traverse \
         L0 at exactly L/u = 10 s while the shoulder pipe spills back.",
        10.0,
        measured,
        "s",
    ))
}

fn case_friction_throttle() -> Result<BenchmarkResult> {
    // Same diverge with the friction coupling armed: phi = 0.7 on L0.
    // The shoulder pipe is spilled over [0.2, 30) — its 2 storage slots are
    // consumed at t = 0.2 and the first freed slot matures at 10 + L/w = 30 —
    // which covers every through discharge. Through arrivals reach the node at
    // 10.0, 10.1, 10.2, 10.3 (entry headway 1/C1 = 0.1 s); the first discharge
    // is ungated (no throttle armed yet) and each discharge arms the next
    // headway 1/(phi*C1) = 1/7 s from the spill state at that instant, so the
    // 4th through vehicle passes at exactly 10 + 3/(phi*C1) s.
    let phi = 0.7;
    let c1 = 10.0; // through-pipe capacity: c * lanes = 5 * 2 (veh/s)
    let s = run(shoulder_exit_scenario(phi))?;
    let measured = s
        .vehicles
        .iter()
        .filter(|v| v.path.get(1) == Some(&1) && v.node_times.len() >= 2)
        .map(|v| v.node_times[1])
        .fold(f64::NEG_INFINITY, f64::max);
    Ok(result(
        "friction_throttle",
        "Friction-throttled sibling discharge",
        "Same diverge with phi = 0.7: while the shoulder pipe is spilled the \
         through pipe re-arms its discharge headway to 1/(phi*C1) = 1/7 s at \
         each discharge. Theory: 4th through passage at 10 + 3/(phi*C1) s.",
        10.0 + 3.0 / (phi * c1),
        measured,
        "s",
    ))
}
// ── Roundabout node ───────────────────────────────────────────────────────────

/// One roundabout node 0 on a 100 m ring at 10 m/s (C = 0.5 veh/s), four arms
/// at 0/25/50/75 m with 4 m islands — points X0 = 98, M0 = 2, X1 = 23,
/// M1 = 27, X2 = 48, M2 = 52, X3 = 73, M3 = 77. Arm k: approach link 2k
/// (100 m at 10 m/s, C = 1 veh/s) and exit link 2k + 1 (100 m at 10 m/s,
/// capacity `exit_c[k]`). `trips`: (arm in, arm out, start).
fn roundabout_scenario(
    critical_gap: f64,
    follow_up: f64,
    exit_c: [f64; 4],
    trips: &[(usize, usize, f64)],
) -> Scenario {
    use crate::model::{RoundaboutArm, RoundaboutSpec};
    let fd = |c: f64, kx: f64| FundamentalDiagram { u: 10.0, w: 5.0, kx, c };
    let mut nodes = vec![node(0, NodeType::Internal, vec![], vec![])];
    let mut links = Vec::new();
    let mut arms = Vec::new();
    for k in 0..4 {
        nodes.push(node(1 + 2 * k, NodeType::Entry, vec![], vec![2 * k]));
        nodes.push(node(2 + 2 * k, NodeType::Exit, vec![2 * k + 1], vec![]));
        nodes[0].incoming_links.push(2 * k);
        nodes[0].outgoing_links.push(2 * k + 1);
        links.push(Link::new(2 * k, 1 + 2 * k, 0, 100.0, 10.0, 1, 1.0, fd(1.0, 0.15), vec![]));
        links.push(Link::new(2 * k + 1, 0, 2 + 2 * k, 100.0, 10.0, 1, exit_c[k], fd(exit_c[k], 0.15), vec![]));
        arms.push(RoundaboutArm { in_link: Some(2 * k), out_link: Some(2 * k + 1), position: 25.0 * k as f64 });
    }
    nodes[0].roundabout = Some(RoundaboutSpec {
        circumference: 100.0,
        lanes: 1,
        speed: 10.0,
        fd: fd(0.5, 0.15),
        critical_gap,
        follow_up,
        island: 4.0,
        arms,
    });
    let vehicles = trips
        .iter()
        .enumerate()
        .map(|(i, &(a, b, t))| Vehicle::new(i, 0, vec![2 * a, 2 * b + 1], t, 1 + 2 * a, 2 + 2 * b))
        .collect();
    Scenario {
        nodes,
        links,
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 100_000.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(), signals: Default::default(), signal_log: Vec::new(),
    }
}

fn case_roundabout_free_flow() -> Result<BenchmarkResult> {
    let s = run(roundabout_scenario(4.0, 2.0, [10.0; 4], &[(0, 2, 0.0)]))?;
    let v = &s.vehicles[0];
    let measured = v.node_times.last().copied().unwrap_or(f64::NAN) - v.node_times[0];
    Ok(result(
        "roundabout_free_flow",
        "Roundabout free-flow traversal",
        "One vehicle through a roundabout node, approach and exit 100 m at 10 m/s, \
         ring arc M0 → X2 = 46 m at 10 m/s. Theory: 10 + 4.6 + 10 = 24.6 s.",
        10.0 + 46.0 / 10.0 + 10.0,
        measured,
        "s",
    ))
}

fn case_roundabout_entry_capacity() -> Result<BenchmarkResult> {
    let trips: Vec<_> = (0..8).map(|_| (0, 1, 0.0)).collect();
    let s = run(roundabout_scenario(4.0, 2.5, [10.0; 4], &trips))?;
    let measured = s
        .vehicles
        .iter()
        .filter_map(|v| v.node_times.get(1).copied())
        .fold(f64::NEG_INFINITY, f64::max);
    Ok(result(
        "roundabout_entry_capacity",
        "Roundabout entry capacity, no conflict",
        "8 vehicles queued on an approach, empty ring, t_f = 2.5 s > 1/C_ring = 2 s > \
         1/C_approach. Theory: entries every t_f from the first arrival at 10 s; \
         last at 10 + 7·2.5 = 27.5 s.",
        10.0 + 7.0 * 2.5,
        measured,
        "s",
    ))
}

fn case_roundabout_siegloch() -> Result<BenchmarkResult> {
    // Circulating 3 → 1 every H = 12.5 s passes M0 at 12.5 + kH; a saturated
    // queue on arm 0 (bound for arm 2) faces it.
    let h = 12.5;
    let mut trips: Vec<(usize, usize, f64)> = (0..40).map(|k| (3, 1, k as f64 * h)).collect();
    trips.extend((0..250).map(|_| (0, 2, 0.0)));
    let mut sim = Simulation::new(roundabout_scenario(6.0, 4.0, [10.0; 4], &trips));
    sim.run().context("benchmark simulation failed")?;
    let st = sim.roundabouts[0].as_ref().context("roundabout state")?;
    let passages = &st.trace[st.geom.merge_point[0]];
    let circ: Vec<f64> = passages.iter().filter(|p| p.circulating).map(|p| p.time).collect();
    let gaps = 2..30;
    let n_gaps = gaps.len();
    let entered: usize = gaps
        .map(|k| {
            passages
                .iter()
                .filter(|p| !p.circulating && p.time > circ[k] && p.time < circ[k + 1])
                .count()
        })
        .sum();
    Ok(result(
        "roundabout_siegloch",
        "Roundabout gap acceptance (Siegloch)",
        "Regular circulating stream every H = 12.5 s in front of a saturated entry, \
         t_c = 6 s, t_f = 4 s (t_0 = 4 s). Theory: floor((H − t_0)/t_f) + 1 = 3 \
         entries per gap.",
        ((h - 4.0) / 4.0f64).floor() + 1.0,
        entered as f64 / n_gaps as f64,
        "veh/gap",
    ))
}

fn case_roundabout_spillback() -> Result<BenchmarkResult> {
    // Arm 2's exit discharges 0.1 veh/s; 60 vehicles 0 → 2 fill the ring
    // between M0 and X2 (46 m · 0.15 = 6.9 → 7 slots) and wait there.
    let trips: Vec<_> = (0..60).map(|k| (0, 2, k as f64 * 2.0)).collect();
    let s = run(roundabout_scenario(4.0, 2.0, [10.0, 10.0, 0.1, 10.0], &trips))?;
    let on_ring: Vec<(f64, f64)> = s
        .vehicles
        .iter()
        .filter_map(|v| Some((*v.node_times.get(1)?, v.ring_exits.first()?.1)))
        .collect();
    let max_on_arc = on_ring
        .iter()
        .map(|&(t, _)| on_ring.iter().filter(|&&(a, b)| a <= t && b > t).count())
        .max()
        .unwrap_or(0);
    Ok(result(
        "roundabout_spillback",
        "Roundabout spillback storage",
        "A saturated exit (0.1 veh/s) makes its traffic queue on the ring back to \
         the entry point 46 m upstream. Theory: the arc holds ceil(kx·d) = \
         ceil(0.15·46) = 7 vehicles at most, and fills up.",
        7.0,
        max_on_arc as f64,
        "veh",
    ))
}

// ── Signals and tolls ─────────────────────────────────────────────────────────

/// Entry(0) →[L0: 100 m at 10 m/s, saturation flow 0.5 veh/s]→ node 1
/// →[L1: free]→ Exit(2); `n` vehicles released at t = 0 (they reach node 1
/// every 2 s from 10 s).
fn signal_scenario(n: usize, signals: crate::signals::SignalNetwork) -> Scenario {
    let fd0 = FundamentalDiagram { u: 10.0, w: 5.0, kx: 0.15, c: 0.5 };
    let free = FundamentalDiagram { u: 10.0, w: 5.0, kx: 10.0, c: 10.0 };
    Scenario {
        nodes: vec![
            node(0, NodeType::Entry, vec![], vec![0]),
            node(1, NodeType::Internal, vec![0], vec![1]),
            node(2, NodeType::Exit, vec![1], vec![]),
        ],
        links: vec![
            Link::new(0, 0, 1, 100.0, 10.0, 1, 0.5, fd0, vec![]),
            Link::new(1, 1, 2, 100.0, 10.0, 1, 10.0, free, vec![]),
        ],
        vehicles: (0..n).map(|i| Vehicle::new(i, 0, vec![0, 1], 0.0, 0, 2)).collect(),
        demand: vec![],
        start_time: 0.0,
        duration: 100_000.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
        signals,
        signal_log: Vec::new(),
    }
}

fn one_group(cap: Option<u32>, cycle: f64, green: [f64; 2]) -> crate::signals::SignalNetwork {
    use crate::signals::{FixedTimePlan, Movement, OffState, SignalGroup, SignalNetwork};
    SignalNetwork {
        groups: vec![SignalGroup {
            id: "g".into(),
            node: 1,
            movements: vec![Movement { from: 0, to: None }],
            vehicles_per_green: cap,
            off: OffState::Green,
        }],
        plans: vec![FixedTimePlan {
            name: None,
            from: 0.0,
            until: 100_000.0,
            cycle,
            offset: 0.0,
            groups: vec![],
            greens: [("g".to_string(), vec![green])].into_iter().collect(),
        }],
        tolls: vec![],
    }
}

fn node_passages(s: &Scenario) -> Vec<f64> {
    let mut t: Vec<f64> = s.vehicles.iter().filter_map(|v| v.node_times.get(1).copied()).collect();
    t.sort_by(f64::total_cmp);
    t
}

fn case_signal_saturation() -> Result<BenchmarkResult> {
    let s = run(signal_scenario(60, one_group(None, 60.0, [0.0, 20.0])))?;
    let t = node_passages(&s);
    // Greens [60, 80), [120, 140), [180, 200) are fully saturated.
    let per_green = t.iter().filter(|&&x| (60.0..200.0).contains(&x)).count() as f64 / 3.0;
    Ok(result(
        "signal_saturation",
        "Signal: discharge per green",
        "Fixed-time signal, cycle 60 s, effective green 20 s, saturated approach with \
         saturation flow s = 0.5 veh/s. Theory: ceil(g·s) = 10 vehicles per green.",
        10.0,
        per_green,
        "veh/green",
    ))
}

fn case_ramp_meter() -> Result<BenchmarkResult> {
    let s = run(signal_scenario(20, one_group(Some(1), 6.0, [0.0, 2.0])))?;
    let t = node_passages(&s);
    let measured = (t[t.len() - 1] - t[0]) / (t.len() - 1) as f64;
    Ok(result(
        "ramp_meter",
        "Ramp metering: one vehicle per green",
        "Ramp signal, cycle 6 s, one vehicle per green, saturated ramp. Theory: \
         one vehicle every 6 s (600 veh/h).",
        6.0,
        measured,
        "s",
    ))
}

fn case_toll_service() -> Result<BenchmarkResult> {
    use crate::signals::{SignalNetwork, Toll};
    let tolls = SignalNetwork {
        tolls: vec![Toll { link: 0, pipes: None, service_time: 12.0, by_class: Default::default() }],
        ..Default::default()
    };
    let s = run(signal_scenario(10, tolls))?;
    let t = node_passages(&s);
    Ok(result(
        "toll_service",
        "Toll lane: deterministic service",
        "10 vehicles reach a toll lane (service 12 s) every 2 s from 10 s. Theory: \
         departures at max(arrival, previous) + 12 s; the last at 10 + 10·12 = 130 s.",
        130.0,
        t[t.len() - 1],
        "s",
    ))
}

// ── Priority junction ─────────────────────────────────────────────────────────

fn case_junction_siegloch() -> Result<BenchmarkResult> {
    // T junction at node 0 (1000, 1000): major road west → east, minor
    // approach from the south turning right (east). Approaches 100 m at
    // 10 m/s; the minor right turn has t_c = 6 s, t_f = 4 s (t_0 = 4 s).
    let centre = (1000.0, 1000.0);
    let (west, east, south) = ((900.0, 1000.0), (1100.0, 1000.0), (1000.0, 900.0));
    let fd_in = FundamentalDiagram { u: 10.0, w: 5.0, kx: 0.15, c: 0.5 };
    let free = FundamentalDiagram { u: 10.0, w: 5.0, kx: 10.0, c: 10.0 };
    let pt = |id, t, p| Node { points: p, ..node(id, t, vec![], vec![]) };
    let mut nodes = vec![
        pt(0, NodeType::Internal, centre),
        pt(1, NodeType::Entry, west),
        pt(2, NodeType::Exit, east),
        pt(3, NodeType::Entry, south),
    ];
    let links = vec![
        Link::new(0, 1, 0, 100.0, 10.0, 1, 0.5, fd_in.clone(), vec![west, centre]),
        Link::new(1, 0, 2, 100.0, 10.0, 1, 10.0, free, vec![centre, east]),
        Link::new(2, 3, 0, 100.0, 10.0, 1, 0.5, fd_in, vec![south, centre]),
    ];
    for l in &links {
        nodes[l.node_up].outgoing_links.push(l.id);
        nodes[l.node_down].incoming_links.push(l.id);
    }
    let raw = serde_json::json!({
        "major": [0, 1],
        "movements": [ { "from": 2, "to": 1, "critical_gap": 6.0, "follow_up": 4.0 } ]
    });
    nodes[0].junction = Some(crate::junction::resolve_spec(&raw, &nodes[0], &links)?);
    let h = 12.5;
    let mut vehicles: Vec<Vehicle> = (0..40).map(|k| Vehicle::new(k, 0, vec![0, 1], k as f64 * h, 1, 2)).collect();
    vehicles.extend((0..250).map(|k| Vehicle::new(40 + k, 0, vec![2, 1], 0.0, 3, 2)));
    let s = run(Scenario {
        nodes,
        links,
        vehicles,
        demand: vec![],
        start_time: 0.0,
        duration: 100_000.0,
        classes: vec!["car".to_string()],
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
        signals: Default::default(),
        signal_log: Vec::new(),
    })?;
    let major: Vec<f64> = s.vehicles[..40].iter().map(|v| v.node_times[1]).collect();
    let gaps = 2..30;
    let n_gaps = gaps.len();
    let entered: usize = gaps
        .map(|k| s.vehicles[40..].iter().filter(|v| v.node_times[1] > major[k] && v.node_times[1] < major[k + 1]).count())
        .sum();
    Ok(result(
        "junction_siegloch",
        "Priority junction: gap acceptance (Siegloch)",
        "Regular major stream every H = 12.5 s in front of a saturated minor right \
         turn, t_c = 6 s, t_f = 4 s (t_0 = 4 s). Theory: floor((H − t_0)/t_f) + 1 = 3 \
         departures per gap.",
        ((h - 4.0) / 4.0f64).floor() + 1.0,
        entered as f64 / n_gaps as f64,
        "veh/gap",
    ))
}

// ── Entry point ───────────────────────────────────────────────────────────────

pub fn run_benchmarks() -> Result<Vec<BenchmarkResult>> {
    Ok(vec![
        case_free_flow_transit()?,
        case_bottleneck_discharge()?,
        case_queue_delay()?,
        case_spillback_wave()?,
        case_fractional_storage_wave()?,
        case_merge_throughput()?,
        case_bus_lane_immunity()?,
        case_shoulder_exit_wave()?,
        case_shoulder_exit_through()?,
        case_friction_throttle()?,
        case_roundabout_free_flow()?,
        case_roundabout_entry_capacity()?,
        case_roundabout_siegloch()?,
        case_roundabout_spillback()?,
        case_signal_saturation()?,
        case_ramp_meter()?,
        case_toll_service()?,
        case_junction_siegloch()?,
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_benchmarks_pass_against_theory() {
        for b in run_benchmarks().expect("benchmark suite should run") {
            assert_eq!(
                b.status,
                CheckStatus::Pass,
                "benchmark '{}' deviates from analytic solution: analytic {} {}, measured {}, err {:.3}%",
                b.id,
                b.analytic,
                b.unit,
                b.measured,
                b.error_pct
            );
        }
    }

    /// The acceptance bar of the papers, enforced literally: agreement with
    /// the closed-form value up to floating-point representation. Stated in
    /// ulps because percentages hide the scale involved — 0.5% would pass a
    /// benchmark wrong by 13 orders of magnitude more than rounding can
    /// explain. A regression here means the solver stopped being exact, not
    /// that it got slightly less accurate.
    #[test]
    fn all_benchmarks_are_exact() {
        const MAX_ULPS: i64 = 4;
        for b in run_benchmarks().expect("benchmark suite should run") {
            let ulps = (b.measured.to_bits() as i64 - b.analytic.to_bits() as i64).abs();
            assert!(
                ulps <= MAX_ULPS,
                "benchmark '{}' is not event-exact: analytic {:.17} {}, measured {:.17}, \
                 {} ulps apart (limit {})",
                b.id,
                b.analytic,
                b.unit,
                b.measured,
                ulps,
                MAX_ULPS
            );
        }
    }
}
