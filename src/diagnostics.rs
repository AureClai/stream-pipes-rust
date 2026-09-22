//! Physics-consistency diagnostics for a completed simulation.
//!
//! Three independent checks:
//!
//! 1. **Fundamental Diagram analysis** — for every (link, time-bin), compare
//!    the simulated (density, flow) point against the theoretical triangular FD.
//!    Deviation reveals where the model drifts from LWR kinematics.
//!
//! 2. **Flow conservation** — at every internal node, inflow should equal
//!    outflow in each time bin.  Persistent imbalance indicates a mass-
//!    conservation bug.
//!
//! 3. **Space-time trajectories** — per-vehicle (time, cumulative-distance)
//!    traces.  Wave slopes, queue formation, and merge behaviour are visible
//!    directly from this data.

use crate::analysis::compute_link_stats;
use crate::model::{LinkID, NodeID, Scenario, VehID, VehicleState};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ── Fundamental Diagram ───────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FDRegime {
    FreeFlow,
    Congested,
    /// Edie density exceeds jam density (kx). Vehicles have accumulated large
    /// waiting times — the FD comparison is no longer meaningful for this bin.
    QueueOverflow,
}

/// One simulated (density, flow) observation on a link, annotated with the
/// theoretical value from the link's FD parameters.
#[derive(Debug, Serialize, Deserialize)]
pub struct FDPoint {
    pub t: f64,
    pub link_id: LinkID,
    pub density: f64,          // veh/km  (simulated)
    pub flow: f64,             // veh/h   (simulated)
    pub theoretical_flow: f64, // veh/h   (from triangular FD)
    /// Signed % deviation: (simulated − theoretical) / max(theoretical, 1) × 100
    pub error_pct: f64,
    pub regime: FDRegime,
}

/// Fundamental-diagram parameters for one link (sent to the frontend so it can
/// draw the theoretical curve as an overlay on the scatter plot).
#[derive(Debug, Serialize, Deserialize)]
pub struct LinkFD {
    pub link_id: LinkID,
    pub u: f64,      // free-flow speed (m/s)
    pub w: f64,      // backwave speed (m/s)
    pub kx: f64,     // jam density (veh/m) — stored as veh/km in model
    pub c: f64,      // capacity (veh/h)
    pub k_crit: f64, // critical density = c / u (veh/km)
    pub length_km: f64,
}

// ── Flow conservation ─────────────────────────────────────────────────────────

/// Mass-conservation check at one internal node for one time bin.
#[derive(Debug, Serialize, Deserialize)]
pub struct NodeBalance {
    pub t: f64,
    pub node_id: NodeID,
    pub inflow: f64,    // veh/h — sum of incoming link flows
    pub outflow: f64,   // veh/h — sum of outgoing link flows
    /// (outflow − inflow) / max(inflow, 1) × 100.  Ideal value: 0.
    pub error_pct: f64,
}

// ── Space-time trajectories ───────────────────────────────────────────────────

/// One node-passage record: the time and cumulative distance from the vehicle's
/// origin at the moment it crossed this node.
#[derive(Debug, Serialize, Deserialize)]
pub struct TrajPoint {
    pub t: f64, // seconds from simulation start
    pub x: f64, // cumulative distance from vehicle origin (km)
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Trajectory {
    pub veh_id: VehID,
    pub origin: NodeID,
    pub destination: NodeID,
    pub path: Vec<LinkID>,
    /// One point per node passage (path.len() + 1 entries when complete).
    pub points: Vec<TrajPoint>,
    pub exited: bool,
}

// ── Summary ───────────────────────────────────────────────────────────────────

#[derive(Debug, Serialize, Deserialize)]
pub struct DiagnosticsSummary {
    pub total_vehicles: usize,
    pub exited_vehicles: usize,
    pub pct_exited: f64,
    pub pct_bins_congested: f64,
    pub fd_mean_abs_error_pct: f64,
    pub fd_max_abs_error_pct: f64,
    pub conservation_mean_abs_error_pct: f64,
    pub conservation_max_abs_error_pct: f64,
}

// ── Report ────────────────────────────────────────────────────────────────────

#[derive(Debug, Serialize, Deserialize)]
pub struct DiagnosticsReport {
    pub fd_points: Vec<FDPoint>,
    pub links_fd: Vec<LinkFD>,
    pub node_balance: Vec<NodeBalance>,
    pub trajectories: Vec<Trajectory>,
    pub summary: DiagnosticsSummary,
}

// ── Entry point ───────────────────────────────────────────────────────────────

/// Compute all three diagnostic analyses.
///
/// `max_trajectories`: cap on how many vehicle trajectories to include in the
/// report (they are sampled uniformly).  Use 0 for no limit.
pub fn run_diagnostics(
    scenario: &Scenario,
    bin_size: f64,
    max_trajectories: usize,
) -> DiagnosticsReport {
    let stats = compute_link_stats(scenario, bin_size);
    let n_bins = (scenario.duration / bin_size).ceil() as usize;

    let fd_points = compute_fd_points(scenario, &stats, bin_size, n_bins);
    let links_fd = collect_links_fd(scenario);
    let node_balance = compute_node_balance(scenario, &stats, bin_size, n_bins);
    let trajectories = compute_trajectories(scenario, max_trajectories);
    let summary = compute_summary(scenario, &fd_points, &node_balance);

    DiagnosticsReport { fd_points, links_fd, node_balance, trajectories, summary }
}

// ── Internal computations ─────────────────────────────────────────────────────

// FD parameters are stored in SI units in the model (m/s, veh/m, veh/s).
// All analysis functions must convert to transport units before comparisons:
//   speed : m/s  × 3.6    = km/h
//   density: veh/m × 1000  = veh/km
//   flow  : veh/s × 3600   = veh/h
// k_crit (veh/km) = c (veh/h) / u (km/h) = (c_si × 3600) / (u_si × 3.6) = c_si/u_si × 1000

fn fd_transport(u_si: f64, w_si: f64, kx_si: f64, c_si: f64) -> (f64, f64, f64, f64, f64) {
    let u  = u_si  * 3.6;
    let w  = w_si  * 3.6;
    let kx = kx_si * 1000.0;
    let c  = c_si  * 3600.0;
    let k_crit = if u > 0.0 { c / u } else { 0.0 };
    (u, w, kx, c, k_crit)
}

fn collect_links_fd(scenario: &Scenario) -> Vec<LinkFD> {
    scenario
        .links
        .iter()
        .map(|l| {
            // Whole-link FD: fd.kx is per lane; observed Edie densities/flows
            // aggregate all lanes, so scale jam density by lanes and use the
            // link's total capacity.
            let (u, w, kx, c, k_crit) =
                fd_transport(l.fd.u, l.fd.w, l.fd.kx * f64::from(l.num_lanes), l.capacity);
            LinkFD {
                link_id: l.id,
                u,
                w,
                kx,
                c,
                k_crit,
                length_km: l.length / 1000.0,
            }
        })
        .collect()
}

pub(crate) fn compute_fd_points(
    scenario: &Scenario,
    stats: &HashMap<LinkID, crate::analysis::LinkStats>,
    bin_size: f64,
    n_bins: usize,
) -> Vec<FDPoint> {
    let mut points = Vec::new();

    for link in &scenario.links {
        let Some(s) = stats.get(&link.id) else { continue };

        // Convert FD parameters to transport units: km/h, veh/km, veh/h.
        // Whole-link values: per-lane kx × lanes, total link capacity.
        let (u, w, kx, c, k_crit) =
            fd_transport(link.fd.u, link.fd.w, link.fd.kx * f64::from(link.num_lanes), link.capacity);

        for i in 0..n_bins {
            let density = s.density.values[i]; // veh/km
            let flow = s.flow.values[i];       // veh/h

            if density < 0.001 && flow < 0.001 {
                continue;
            }

            // When Edie density exceeds kx, vehicles are in a persistent queue.
            // The FD branch formula breaks down — flag as overflow, skip error calc.
            if density > kx {
                points.push(FDPoint {
                    t: i as f64 * bin_size + scenario.start_time,
                    link_id: link.id,
                    density,
                    flow,
                    theoretical_flow: 0.0,
                    error_pct: 0.0,
                    regime: FDRegime::QueueOverflow,
                });
                continue;
            }

            let (theoretical_flow, regime) = if density <= k_crit {
                // Free-flow branch: q (veh/h) = u (km/h) × k (veh/km)
                (u * density, FDRegime::FreeFlow)
            } else {
                // Congested branch: q (veh/h) = w (km/h) × (kx − k) (veh/km)
                let q = w * (kx - density);
                (q.max(0.0), FDRegime::Congested)
            };

            let theoretical_capped = theoretical_flow.min(c);

            let error_pct = if theoretical_capped > c * 0.01 {
                (flow - theoretical_capped) / theoretical_capped * 100.0
            } else if flow > c * 0.01 {
                (flow / c) * 100.0
            } else {
                0.0
            };

            points.push(FDPoint {
                t: i as f64 * bin_size + scenario.start_time,
                link_id: link.id,
                density,
                flow,
                theoretical_flow: theoretical_capped,
                error_pct,
                regime,
            });
        }
    }

    points
}

pub(crate) fn compute_node_balance(
    scenario: &Scenario,
    stats: &HashMap<LinkID, crate::analysis::LinkStats>,
    bin_size: f64,
    n_bins: usize,
) -> Vec<NodeBalance> {
    let mut balances = Vec::new();

    for node in &scenario.nodes {
        // Entry/exit nodes are network boundaries — conservation doesn't apply
        if node.incoming_links.is_empty() || node.outgoing_links.is_empty() {
            continue;
        }

        // Minimum free-flow travel time to reach this node from any incoming link.
        // Bins before this point will show inflow without outflow (startup artifact).
        let min_reach_s: f64 = node.incoming_links.iter().filter_map(|&lid| {
            scenario.links.get(lid).map(|l| {
                if l.speed > 0.0 { l.length / l.speed } else { f64::MAX }
            })
        }).fold(f64::MAX, f64::min);
        let startup_bins = if min_reach_s.is_finite() {
            (min_reach_s / bin_size).ceil() as usize
        } else {
            1
        };

        for i in 0..n_bins {
            let inflow: f64 = node
                .incoming_links
                .iter()
                .filter_map(|lid| stats.get(lid))
                .map(|s| s.flow.values[i])
                .sum();

            let outflow: f64 = node
                .outgoing_links
                .iter()
                .filter_map(|lid| stats.get(lid))
                .map(|s| s.flow.values[i])
                .sum();

            // Skip idle bins and the startup window (vehicles still in transit)
            if inflow < 0.001 && outflow < 0.001 {
                continue;
            }
            if i < startup_bins {
                continue;
            }

            let error_pct = if inflow > 1.0 {
                (outflow - inflow) / inflow * 100.0
            } else {
                0.0
            };

            balances.push(NodeBalance {
                t: i as f64 * bin_size,
                node_id: node.id,
                inflow,
                outflow,
                error_pct,
            });
        }
    }

    balances
}

fn compute_trajectories(scenario: &Scenario, max_trajectories: usize) -> Vec<Trajectory> {
    let all: Vec<&crate::model::Vehicle> = scenario
        .vehicles
        .iter()
        .filter(|v| !v.node_times.is_empty() && !v.path.is_empty())
        .collect();

    // Uniform sampling when there are more vehicles than the cap
    let step = if max_trajectories == 0 || all.len() <= max_trajectories {
        1
    } else {
        all.len() / max_trajectories
    };

    let mut result = Vec::new();

    for veh in all.iter().step_by(step.max(1)) {
        let mut cumulative_km = 0.0;
        let mut points = Vec::with_capacity(veh.node_times.len());

        // First point: vehicle at its origin (t=entry, x=0)
        points.push(TrajPoint { t: veh.node_times[0], x: 0.0 });

        for (i, &link_id) in veh.path.iter().enumerate() {
            if i + 1 >= veh.node_times.len() {
                break;
            }
            if link_id >= scenario.links.len() {
                break;
            }
            cumulative_km += scenario.links[link_id].length / 1000.0;
            points.push(TrajPoint {
                t: veh.node_times[i + 1],
                x: cumulative_km,
            });
        }

        if points.len() >= 2 {
            result.push(Trajectory {
                veh_id: veh.id,
                origin: veh.origin,
                destination: veh.destination,
                path: veh.path.clone(),
                points,
                exited: veh.state == VehicleState::Exited,
            });
        }
    }

    result
}

fn compute_summary(
    scenario: &Scenario,
    fd: &[FDPoint],
    balance: &[NodeBalance],
) -> DiagnosticsSummary {
    let total = scenario.vehicles.len();
    let exited = scenario
        .vehicles
        .iter()
        .filter(|v| v.state == VehicleState::Exited)
        .count();

    // Exclude QueueOverflow bins from FD error stats — they indicate queue saturation,
    // not a model physics failure that can be measured by FD deviation.
    let fd_errors: Vec<f64> = fd.iter()
        .filter(|p| p.regime != FDRegime::QueueOverflow)
        .map(|p| p.error_pct.abs())
        .collect();
    let fd_mean = mean(&fd_errors);
    let fd_max = fd_errors.iter().cloned().fold(0.0_f64, f64::max);

    let balance_errors: Vec<f64> = balance.iter().map(|b| b.error_pct.abs()).collect();
    let cons_mean = mean(&balance_errors);
    let cons_max = balance_errors.iter().cloned().fold(0.0_f64, f64::max);

    let congested = fd.iter().filter(|p| p.regime == FDRegime::Congested).count();
    let overflow  = fd.iter().filter(|p| p.regime == FDRegime::QueueOverflow).count();
    let pct_congested = if fd.is_empty() {
        0.0
    } else {
        (congested + overflow) as f64 / fd.len() as f64 * 100.0
    };

    DiagnosticsSummary {
        total_vehicles: total,
        exited_vehicles: exited,
        pct_exited: if total > 0 { exited as f64 / total as f64 * 100.0 } else { 0.0 },
        pct_bins_congested: pct_congested,
        fd_mean_abs_error_pct: fd_mean,
        fd_max_abs_error_pct: fd_max,
        conservation_mean_abs_error_pct: cons_mean,
        conservation_max_abs_error_pct: cons_max,
    }
}

fn mean(values: &[f64]) -> f64 {
    if values.is_empty() {
        0.0
    } else {
        values.iter().sum::<f64>() / values.len() as f64
    }
}

// ── XML export ────────────────────────────────────────────────────────────────

/// Escape the five XML special characters in a string value.
fn xe(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
}

fn f2(v: f64) -> String { format!("{:.2}", v) }
fn f1(v: f64) -> String { format!("{:.1}", v) }

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() { return 0.0; }
    let idx = (p / 100.0 * (sorted.len() - 1) as f64) as usize;
    sorted[idx.min(sorted.len() - 1)]
}

fn fd_quality(mean_err: f64) -> &'static str {
    if mean_err < 5.0 { "GOOD" } else if mean_err < 20.0 { "ACCEPTABLE" } else { "POOR" }
}

fn conservation_quality(mean_err: f64) -> &'static str {
    if mean_err < 1.0 { "GOOD" } else if mean_err < 5.0 { "ACCEPTABLE" } else { "POOR" }
}

/// Serialise a completed diagnostics run to a structured XML document designed
/// for ingestion by an LLM.  Every numeric attribute carries a unit suffix,
/// interpretation notes are embedded inline, and physics anomalies are
/// surfaced as explicit `<Finding>` elements.
pub fn export_to_xml(
    report: &DiagnosticsReport,
    scenario: &Scenario,
    project_name: &str,
    scenario_name: &str,
) -> String {
    use crate::analysis::compute_link_stats;

    let stats = compute_link_stats(scenario, 60.0);
    let s = &report.summary;

    // ── per-vehicle travel-time data (all vehicles, not just sampled) ─────────
    struct VehStats { id: VehID, origin: usize, destination: usize, start_s: f64, travel_s: f64, dist_km: f64, speed_kmh: f64, exited: bool }
    let mut veh_stats: Vec<VehStats> = scenario.vehicles.iter().filter_map(|v| {
        if v.node_times.is_empty() || v.path.is_empty() { return None; }
        let travel_s = v.node_times.last().unwrap() - v.node_times[0];
        // Only count links the vehicle actually exited (one node_time per node passage)
        let links_completed = v.node_times.len().saturating_sub(1);
        let dist_km: f64 = v.path.iter()
            .take(links_completed)
            .filter(|&&lid| lid < scenario.links.len())
            .map(|&lid| scenario.links[lid].length / 1000.0)
            .sum();
        let speed_kmh = if travel_s > 0.0 { dist_km / travel_s * 3600.0 } else { 0.0 };
        Some(VehStats {
            id: v.id, origin: v.origin, destination: v.destination,
            start_s: v.node_times[0], travel_s, dist_km, speed_kmh,
            exited: v.state == VehicleState::Exited,
        })
    }).collect();

    let mut sorted_travel: Vec<f64> = veh_stats.iter().filter(|v| v.exited).map(|v| v.travel_s).collect();
    sorted_travel.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mean_travel = mean(&sorted_travel);
    let mean_speed = if mean_travel > 0.0 {
        veh_stats.iter().filter(|v| v.exited).map(|v| v.dist_km).sum::<f64>() / sorted_travel.len() as f64 / mean_travel * 3600.0
    } else { 0.0 };

    // top-20 slowest
    veh_stats.sort_by(|a, b| b.travel_s.partial_cmp(&a.travel_s).unwrap());
    let slowest: Vec<&VehStats> = veh_stats.iter().take(20).collect();
    let unexited: Vec<&VehStats> = veh_stats.iter().filter(|v| !v.exited).collect();

    // ── per-link aggregate FD stats ────────────────────────────────────────────
    struct LinkAgg { free_bins: usize, cong_bins: usize, overflow_bins: usize, mean_err: f64, max_err: f64, worst: Option<FDPoint2> }
    struct FDPoint2 { t: f64, density: f64, flow: f64, theoretical: f64, error_pct: f64 }
    let mut link_agg: HashMap<LinkID, LinkAgg> = HashMap::new();
    for p in &report.fd_points {
        let e = link_agg.entry(p.link_id).or_insert(LinkAgg { free_bins: 0, cong_bins: 0, overflow_bins: 0, mean_err: 0.0, max_err: 0.0, worst: None });
        match p.regime {
            FDRegime::FreeFlow      => e.free_bins += 1,
            FDRegime::Congested     => e.cong_bins += 1,
            FDRegime::QueueOverflow => { e.overflow_bins += 1; continue; }
        }
        let ae = p.error_pct.abs();
        e.mean_err += ae;
        if ae > e.max_err {
            e.max_err = ae;
            e.worst = Some(FDPoint2 { t: p.t, density: p.density, flow: p.flow, theoretical: p.theoretical_flow, error_pct: p.error_pct });
        }
    }
    for (lid, agg) in link_agg.iter_mut() {
        let total = (agg.free_bins + agg.cong_bins) as f64;
        if total > 0.0 { agg.mean_err /= total; }
        let _ = lid;
    }

    // ── per-node aggregate conservation stats ─────────────────────────────────
    struct NodeAgg { bins: usize, mean_err: f64, max_err: f64, worst_t: f64, worst_in: f64, worst_out: f64, worst_epct: f64 }
    let mut node_agg: HashMap<NodeID, NodeAgg> = HashMap::new();
    for b in &report.node_balance {
        let e = node_agg.entry(b.node_id).or_insert(NodeAgg { bins: 0, mean_err: 0.0, max_err: 0.0, worst_t: 0.0, worst_in: 0.0, worst_out: 0.0, worst_epct: 0.0 });
        let ae = b.error_pct.abs();
        e.bins += 1;
        e.mean_err += ae;
        if ae > e.max_err {
            e.max_err = ae;
            e.worst_t = b.t;
            e.worst_in = b.inflow;
            e.worst_out = b.outflow;
            e.worst_epct = b.error_pct;
        }
    }
    for (nid, agg) in node_agg.iter_mut() {
        if agg.bins > 0 { agg.mean_err /= agg.bins as f64; }
        let _ = nid;
    }

    // ── automatic physics findings ─────────────────────────────────────────────
    let mut findings: Vec<(String, String, String)> = Vec::new(); // (severity, component, text)

    if s.pct_exited < 80.0 {
        findings.push(("CRITICAL".into(), "global".into(), format!(
            "{:.0}% of vehicles did not exit the network. This may indicate a deadlock, \
             insufficient simulation duration, or routing errors. Investigate vehicles \
             with empty node_times.", 100.0 - s.pct_exited)));
    } else if s.pct_exited < 95.0 {
        findings.push(("WARNING".into(), "global".into(), format!(
            "{} vehicles ({:.1}%) remain in the network at simulation end. \
             These are likely vehicles released near the end of the demand period whose \
             trip is longer than the remaining simulation time.", s.total_vehicles - s.exited_vehicles, 100.0 - s.pct_exited)));
    }

    if s.fd_mean_abs_error_pct > 20.0 {
        findings.push(("WARNING".into(), "fundamental_diagram".into(),
            "Mean FD deviation exceeds 20%. The constant-speed assumption causes significant \
             departure from LWR kinematics in congested bins. Density-dependent speed \
             (e.g. Greenshields or BPR) would improve physical fidelity.".into()));
    } else if s.fd_mean_abs_error_pct > 5.0 {
        findings.push(("INFO".into(), "fundamental_diagram".into(),
            "Moderate FD deviation (5-20%). Simulated flow is slightly higher than the \
             theoretical congested-branch capacity. This is a known consequence of the \
             mesoscopic model's constant-speed travel time.".into()));
    } else {
        findings.push(("INFO".into(), "fundamental_diagram".into(),
            "FD error is below 5%. The model is operating predominantly in free-flow \
             conditions where constant-speed is physically accurate.".into()));
    }

    if s.conservation_mean_abs_error_pct > 5.0 {
        findings.push(("WARNING".into(), "flow_conservation".into(),
            "Mean mass-conservation error exceeds 5%. Investigate node balance data \
             for systematic source/sink behaviour, or check for vehicle indexing bugs.".into()));
    } else if s.conservation_mean_abs_error_pct > 1.0 {
        findings.push(("INFO".into(), "flow_conservation".into(),
            "Residual conservation error (1-5%) is within expected discretisation noise: \
             vehicles in transit between links at bin boundaries create temporary imbalances.".into()));
    } else {
        findings.push(("INFO".into(), "flow_conservation".into(),
            "Mass is well conserved (<1% error). No source/sink anomalies detected.".into()));
    }

    if s.pct_bins_congested > 70.0 {
        findings.push(("WARNING".into(), "congestion".into(), format!(
            "{:.0}% of link-bin observations are in the congested FD regime. \
             Demand likely exceeds network capacity. Consider increasing capacity, \
             reducing demand, or extending the simulation to observe dissipation.", s.pct_bins_congested)));
    } else if s.pct_bins_congested > 30.0 {
        findings.push(("INFO".into(), "congestion".into(), format!(
            "{:.0}% congested bins. The network experiences notable congestion — \
             queue formation and shockwave propagation are expected.", s.pct_bins_congested)));
    }

    // flag worst FD link
    if let Some((&lid, agg)) = link_agg.iter().max_by(|a, b| a.1.max_err.partial_cmp(&b.1.max_err).unwrap()) {
        if agg.max_err > 30.0 {
            findings.push(("INFO".into(), format!("link_{}", lid), format!(
                "Link {} has the highest single-bin FD error ({:.1}%). \
                 This is the primary contributor to overall FD deviation.", lid, agg.max_err)));
        }
    }

    // ── assemble XML ───────────────────────────────────────────────────────────
    let mut x = String::with_capacity(65_536);

    x.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    x.push_str("<!--\n");
    x.push_str("  Stream Traffic Simulation - Physics Diagnostics Report\n");
    x.push_str("  Structured for LLM analysis.\n");
    x.push_str("  - All numeric attributes include a unit suffix in the attribute name.\n");
    x.push_str("  - <Interpretation> elements contain human-readable physics notes.\n");
    x.push_str("  - <Finding> elements flag anomalies with severity INFO / WARNING / CRITICAL.\n");
    x.push_str("  - Time series are space-separated values at 60-second bin intervals.\n");
    x.push_str("-->\n");
    x.push_str(&format!(
        "<DiagnosticsReport project=\"{}\" scenario=\"{}\" schema_version=\"1.0\">\n\n",
        xe(project_name), xe(scenario_name)
    ));

    // ── Context ───────────────────────────────────────────────────────────────
    x.push_str("  <!-- === SIMULATION CONTEXT ========================================== -->\n");
    x.push_str("  <Context>\n");
    x.push_str(&format!("    <Network links=\"{}\" nodes=\"{}\" internal_nodes=\"{}\"/>\n",
        scenario.links.len(), scenario.nodes.len(),
        scenario.nodes.iter().filter(|n| !n.incoming_links.is_empty() && !n.outgoing_links.is_empty()).count()
    ));
    x.push_str(&format!(
        "    <Simulation start_time_s=\"{}\" duration_s=\"{}\" bin_size_s=\"60\"\n              \
         total_vehicles=\"{}\" exited_vehicles=\"{}\" pct_exited_pct=\"{}\"/>\n",
        f1(scenario.start_time), f1(scenario.duration),
        s.total_vehicles, s.exited_vehicles, f1(s.pct_exited)
    ));
    x.push_str("    <Model type=\"Mesoscopic event-driven\" fd_shape=\"Triangular LWR\"\n");
    x.push_str("           assignment=\"Static shortest-path\"\n");
    x.push_str("           speed_model=\"Constant free-flow speed (density-independent)\"\n");
    x.push_str("           note=\"Vehicle travel times use free-flow speed unconditionally.\n");
    x.push_str("                 FD deviations measure the gap to full LWR density-dependent flow.\"/>\n");
    x.push_str("  </Context>\n\n");

    // ── Summary ───────────────────────────────────────────────────────────────
    x.push_str("  <!-- === SUMMARY ====================================================== -->\n");
    x.push_str("  <Summary>\n");
    x.push_str(&format!("    <Completeness exited=\"{}\" total=\"{}\" pct_exited_pct=\"{}\" status=\"{}\"/>\n",
        s.exited_vehicles, s.total_vehicles, f1(s.pct_exited),
        if s.pct_exited >= 99.0 { "OK" } else if s.pct_exited >= 90.0 { "WARN" } else { "FAIL" }
    ));
    x.push_str(&format!("    <CongestionLevel pct_link_bins_pct=\"{}\"\n", f1(s.pct_bins_congested)));
    x.push_str("                   note=\"% of (link × time-bin) observations in the congested FD regime.\"/>\n");
    x.push_str(&format!(
        "    <FundamentalDiagram mean_abs_error_pct=\"{}\" max_abs_error_pct=\"{}\" quality=\"{}\">\n",
        f2(s.fd_mean_abs_error_pct), f2(s.fd_max_abs_error_pct), fd_quality(s.fd_mean_abs_error_pct)
    ));
    x.push_str("      <Interpretation>Measures how closely simulated (density, flow) pairs lie on\n");
    x.push_str("        the theoretical triangular FD. The constant-speed model produces exact\n");
    x.push_str("        agreement in free-flow; deviations appear in the congested regime where\n");
    x.push_str("        real speed should decrease with density.</Interpretation>\n");
    x.push_str("    </FundamentalDiagram>\n");
    x.push_str(&format!(
        "    <FlowConservation mean_abs_error_pct=\"{}\" max_abs_error_pct=\"{}\" quality=\"{}\">\n",
        f2(s.conservation_mean_abs_error_pct), f2(s.conservation_max_abs_error_pct),
        conservation_quality(s.conservation_mean_abs_error_pct)
    ));
    x.push_str("      <Interpretation>At every internal node, inflow should equal outflow in each\n");
    x.push_str("        time bin. Residual error below 1% is normal discretisation noise;\n");
    x.push_str("        persistent error above 5% indicates a model defect.</Interpretation>\n");
    x.push_str("    </FlowConservation>\n");
    x.push_str("  </Summary>\n\n");

    // ── Link analysis ─────────────────────────────────────────────────────────
    x.push_str("  <!-- === PER-LINK FUNDAMENTAL DIAGRAM ANALYSIS ======================== -->\n");
    x.push_str("  <LinkAnalysis>\n");
    for link in &scenario.links {
        let fd = &link.fd;
        let k_crit = if fd.u > 0.0 { fd.c / fd.u } else { 0.0 };
        let agg = link_agg.get(&link.id);
        x.push_str(&format!(
            "    <Link id=\"{}\" length_km=\"{}\">\n",
            link.id, f2(link.length / 1000.0)
        ));
        x.push_str(&format!(
            "      <FD free_flow_speed_ms=\"{}\" backwave_speed_ms=\"{}\" \
             jam_density_veh_km=\"{}\" capacity_veh_h=\"{}\" critical_density_veh_km=\"{}\"/>\n",
            f2(fd.u), f2(fd.w), f2(fd.kx), f2(fd.c), f2(k_crit)
        ));
        if let Some(agg) = agg {
            let total_bins = agg.free_bins + agg.cong_bins + agg.overflow_bins;
            x.push_str(&format!(
                "      <Observations total_bins=\"{}\" free_flow_bins=\"{}\" congested_bins=\"{}\" \
                 queue_overflow_bins=\"{}\" \
                 mean_abs_error_pct=\"{}\" max_abs_error_pct=\"{}\"/>\n",
                total_bins, agg.free_bins, agg.cong_bins, agg.overflow_bins,
                f2(agg.mean_err), f2(agg.max_err)
            ));
            if let Some(w) = &agg.worst {
                x.push_str(&format!(
                    "      <WorstBin t_s=\"{}\" density_veh_km=\"{}\" \
                     simulated_flow_veh_h=\"{}\" theoretical_flow_veh_h=\"{}\" error_pct=\"{}\"/>\n",
                    f1(w.t), f2(w.density), f2(w.flow), f2(w.theoretical), f2(w.error_pct)
                ));
            }
        } else {
            x.push_str("      <Observations total_bins=\"0\" note=\"No traffic on this link.\"/>\n");
        }
        // compact time-series (flow, density, speed at 60s bins)
        if let Some(ls) = stats.get(&link.id) {
            let n = ls.flow.values.len();
            if n > 0 {
                let flow_str: Vec<String> = ls.flow.values.iter().map(|v| format!("{:.0}", v)).collect();
                let dens_str: Vec<String> = ls.density.values.iter().map(|v| format!("{:.2}", v)).collect();
                let spd_str:  Vec<String> = ls.speed.values.iter().map(|v| format!("{:.1}", v)).collect();
                x.push_str(&format!(
                    "      <TimeSeries bins=\"{}\" bin_size_s=\"60\">\n", n));
                x.push_str(&format!("        <Flow unit=\"veh_h\">{}</Flow>\n", flow_str.join(" ")));
                x.push_str(&format!("        <Density unit=\"veh_km\">{}</Density>\n", dens_str.join(" ")));
                x.push_str(&format!("        <Speed unit=\"km_h\">{}</Speed>\n", spd_str.join(" ")));
                x.push_str("      </TimeSeries>\n");
            }
        }
        x.push_str("    </Link>\n");
    }
    x.push_str("  </LinkAnalysis>\n\n");

    // ── Node conservation ──────────────────────────────────────────────────────
    x.push_str("  <!-- === PER-NODE FLOW CONSERVATION =================================== -->\n");
    x.push_str("  <NodeAnalysis>\n");
    for node in &scenario.nodes {
        let is_internal = !node.incoming_links.is_empty() && !node.outgoing_links.is_empty();
        let node_type = if node.incoming_links.is_empty() { "Entry" }
                        else if node.outgoing_links.is_empty() { "Exit" }
                        else { "Internal" };
        x.push_str(&format!(
            "    <Node id=\"{}\" type=\"{}\" incoming_links=\"{}\" outgoing_links=\"{}\">\n",
            node.id, node_type, node.incoming_links.len(), node.outgoing_links.len()
        ));
        if is_internal {
            if let Some(agg) = node_agg.get(&node.id) {
                x.push_str(&format!(
                    "      <Conservation bins_observed=\"{}\" mean_abs_error_pct=\"{}\" max_abs_error_pct=\"{}\"/>\n",
                    agg.bins, f2(agg.mean_err), f2(agg.max_err)
                ));
                x.push_str(&format!(
                    "      <WorstBin t_s=\"{}\" inflow_veh_h=\"{}\" outflow_veh_h=\"{}\" error_pct=\"{}\"/>\n",
                    f1(agg.worst_t), f2(agg.worst_in), f2(agg.worst_out), f2(agg.worst_epct)
                ));
            } else {
                x.push_str("      <Conservation bins_observed=\"0\" note=\"No flow observed at this node.\"/>\n");
            }
        } else {
            x.push_str("      <!-- Boundary node: conservation does not apply. -->\n");
        }
        x.push_str("    </Node>\n");
    }
    x.push_str("  </NodeAnalysis>\n\n");

    // ── Travel-time distribution ───────────────────────────────────────────────
    x.push_str("  <!-- === VEHICLE TRAVEL-TIME DISTRIBUTION ============================= -->\n");
    x.push_str("  <TravelTimeDistribution>\n");
    if !sorted_travel.is_empty() {
        x.push_str(&format!(
            "    <Statistics exited=\"{}\" total=\"{}\" mean_travel_s=\"{}\" mean_speed_km_h=\"{}\">\n",
            sorted_travel.len(), s.total_vehicles, f2(mean_travel), f2(mean_speed)
        ));
        for p in [10u8, 25, 50, 75, 90, 95, 99] {
            x.push_str(&format!(
                "      <Percentile p=\"{}\" travel_time_s=\"{}\"/>\n",
                p, f2(percentile(&sorted_travel, p as f64))
            ));
        }
        x.push_str("    </Statistics>\n");
    }
    x.push_str(&format!("    <SlowestVehicles count=\"{}\" note=\"Top 20 by travel time\">\n", slowest.len().min(20)));
    for v in slowest.iter().take(20) {
        x.push_str(&format!(
            "      <Vehicle id=\"{}\" origin=\"{}\" destination=\"{}\" start_s=\"{}\" \
             travel_s=\"{}\" dist_km=\"{}\" avg_speed_km_h=\"{}\" exited=\"{}\"/>\n",
            v.id, v.origin, v.destination, f2(v.start_s), f2(v.travel_s),
            f2(v.dist_km), f2(v.speed_kmh), v.exited
        ));
    }
    x.push_str("    </SlowestVehicles>\n");
    if !unexited.is_empty() {
        x.push_str(&format!("    <UnexitedVehicles count=\"{}\">\n", unexited.len()));
        for v in &unexited {
            x.push_str(&format!(
                "      <Vehicle id=\"{}\" origin=\"{}\" destination=\"{}\" start_s=\"{}\" \
                 partial_travel_s=\"{}\" partial_dist_km=\"{}\"/>\n",
                v.id, v.origin, v.destination, f2(v.start_s), f2(v.travel_s), f2(v.dist_km)
            ));
        }
        x.push_str("    </UnexitedVehicles>\n");
    } else {
        x.push_str("    <UnexitedVehicles count=\"0\"/>\n");
    }
    x.push_str("  </TravelTimeDistribution>\n\n");

    // ── Physics findings ───────────────────────────────────────────────────────
    x.push_str("  <!-- === AUTOMATIC PHYSICS FINDINGS =================================== -->\n");
    x.push_str("  <PhysicsFindings>\n");
    for (severity, component, text) in &findings {
        x.push_str(&format!(
            "    <Finding severity=\"{}\" component=\"{}\">{}</Finding>\n",
            severity, xe(component), xe(text)
        ));
    }
    x.push_str("  </PhysicsFindings>\n\n");

    x.push_str("</DiagnosticsReport>\n");
    x
}
