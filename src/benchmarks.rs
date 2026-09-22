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

use crate::model::{FundamentalDiagram, Link, Node, NodeType, PipeSpec, Scenario, Vehicle};
use crate::simulation::Simulation;
use crate::verification::CheckStatus;
use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

/// Relative error bounds: the engine is deterministic and event-exact, so
/// benchmarks must match theory almost perfectly.
const PASS_PCT: f64 = 0.5;
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
        signals: vec![],
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
        assignment_excluded: Vec::new(),
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
        assignment_excluded: Vec::new(),
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
        assignment_excluded: Vec::new(),
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
        assignment_excluded: Vec::new(),
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
// ── Entry point ───────────────────────────────────────────────────────────────

pub fn run_benchmarks() -> Result<Vec<BenchmarkResult>> {
    Ok(vec![
        case_free_flow_transit()?,
        case_bottleneck_discharge()?,
        case_queue_delay()?,
        case_spillback_wave()?,
        case_merge_throughput()?,
        case_bus_lane_immunity()?,
        case_shoulder_exit_wave()?,
        case_shoulder_exit_through()?,
        case_friction_throttle()?,
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
}
