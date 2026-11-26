//! Realism verification — per-element physics checks against LWR theory.
//!
//! ## Methodology
//!
//! The simulation engine *should* reproduce kinematic wave theory exactly:
//! it is an event-based solver of the LWR model with a triangular fundamental
//! diagram. Verification therefore measures each mechanism from the simulation
//! *output alone* (vehicle passage times) and compares it against the value the
//! link/node parameters predict. A deviation means either a scenario coding
//! problem (unrealistic parameters) or an engine defect — both are exactly
//! what a methodology under development needs to surface.
//!
//! Checks per element:
//!
//! **Links**
//! - `free_flow_traversal` — fastest observed traversal vs L/u. A faster-than-
//!   free-flow passage is a physics violation (Fail); if every vehicle was
//!   delayed the check is NotExercised (pure congestion is legitimate).
//! - `inflow_capacity` — smallest observed entry headway vs 1/C. Entering
//!   faster than capacity violates the demand/supply model.
//! - `storage_bound` — peak simultaneous occupancy vs kx·L·lanes. More
//!   vehicles than the jam accumulation is unphysical.
//! - `backward_wave` — when spillback occurred, the measured delay between a
//!   slot being freed downstream (exit of vehicle n−dn) and the entry it
//!   enables (vehicle n) vs the theoretical wave travel time L/w.
//! - `fd_adherence` — mean deviation of binned (density, flow) observations
//!   from the theoretical triangular FD (Edie-style link averages).
//!
//! **Nodes**
//! - `flow_conservation` — vehicles leaving the incoming links vs vehicles
//!   entering the outgoing links, per time bin (vehicles must not
//!   appear or vanish at internal nodes).
//! - `fifo_discipline` — passage order at the node must follow arrival order
//!   per incoming link (LWR first-order traffic has no overtaking).
//!
//! **Entries**
//! - `demand_served` — fraction of scheduled vehicles that physically entered
//!   the network, plus entry-queue waiting statistics.

use crate::analysis::compute_link_stats;
use crate::diagnostics::{compute_fd_points, FDRegime};
use crate::model::{LinkID, NodeID, NodeType, PipeIdx, RoundaboutSpec, Scenario};
use crate::roundabout::RingGeometry;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ── Thresholds (the quantitative side of the methodology) ─────────────────────

/// Relative tolerance for mechanism checks that should be near-exact
/// (free-flow time, wave delay, capacity headway): 1%.
const TOL_MECHANISM: f64 = 0.01;
/// FD adherence: mean |error| below → Pass, below warn bound → Warn.
const FD_PASS_PCT: f64 = 10.0;
const FD_WARN_PCT: f64 = 25.0;
/// FD adherence: bins with fewer vehicles than this are not compared.
const FD_MIN_VEHICLES: f64 = 30.0;
/// Flow conservation mean |error| bounds (%).
const CONS_PASS_PCT: f64 = 2.0;
const CONS_WARN_PCT: f64 = 10.0;
/// Demand served bounds (%).
const SERVED_PASS_PCT: f64 = 99.0;
const SERVED_WARN_PCT: f64 = 90.0;

// ── Report types ──────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CheckStatus {
    Pass,
    Warn,
    Fail,
    /// The mechanism was never triggered by this scenario (e.g. no spillback
    /// on this link) — not a defect, but the check carries no evidence.
    NotExercised,
}

impl CheckStatus {
    fn severity(self) -> u8 {
        match self {
            CheckStatus::Fail => 3,
            CheckStatus::Warn => 2,
            CheckStatus::Pass => 1,
            CheckStatus::NotExercised => 0,
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Check {
    pub id: String,
    pub name: String,
    pub status: CheckStatus,
    /// Value measured from the simulation output (unit below).
    pub measured: Option<f64>,
    /// Value predicted by the element's parameters.
    pub expected: Option<f64>,
    pub unit: String,
    /// |measured − expected| / expected × 100 when both are defined.
    pub error_pct: Option<f64>,
    /// Human-readable methodology note: what was compared and on how much data.
    pub detail: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ElementType {
    Link,
    Node,
    Entry,
    Signal,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ElementReport {
    pub element_type: ElementType,
    pub element_id: usize,
    /// For multi-pipe links: which pipe this element reports on.
    /// `None` = the whole link (or a non-link element).
    #[serde(default)]
    pub pipe: Option<PipeIdx>,
    pub label: String,
    /// Worst status among the element's checks.
    pub status: CheckStatus,
    pub checks: Vec<Check>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct VerificationSummary {
    pub elements_checked: usize,
    pub elements_pass: usize,
    pub elements_warn: usize,
    pub elements_fail: usize,
    pub elements_not_exercised: usize,
    pub checks_total: usize,
    pub checks_fail: usize,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct VerificationReport {
    pub summary: VerificationSummary,
    pub elements: Vec<ElementReport>,
}

// ── Passage extraction ────────────────────────────────────────────────────────

/// Per-link passage record reconstructed from vehicle node_times.
#[derive(Default)]
struct LinkPassages {
    /// Entry times, sorted ascending (admission order).
    entries: Vec<f64>,
    /// Exit times (node passage at the downstream end), sorted ascending.
    exits: Vec<f64>,
    /// (entry, exit) pairs of completed traversals, unsorted.
    completed: Vec<(f64, f64)>,
    /// exit − entry for completed traversals.
    travel_times: Vec<f64>,
}

fn collect_passages(scenario: &Scenario) -> HashMap<(LinkID, PipeIdx), LinkPassages> {
    let mut map: HashMap<(LinkID, PipeIdx), LinkPassages> = HashMap::new();
    for veh in &scenario.vehicles {
        debug_assert!(
            veh.pipes_taken.len() >= veh.node_times.len().saturating_sub(1),
            "vehicle {}: pipes_taken shorter than traversed prefix",
            veh.id
        );
        for (i, &lid) in veh.path.iter().enumerate() {
            let Some(t_in) = veh.link_entry_time(i) else {
                break;
            };
            let pipe = veh.pipes_taken.get(i).copied().unwrap_or(0);
            let p = map.entry((lid, pipe)).or_default();
            p.entries.push(t_in);
            if let Some(t_out) = veh.link_exit_time(i) {
                p.exits.push(t_out);
                p.completed.push((t_in, t_out));
                p.travel_times.push(t_out - t_in);
            }
        }
    }
    for p in map.values_mut() {
        p.entries.sort_by(|a, b| a.total_cmp(b));
        p.exits.sort_by(|a, b| a.total_cmp(b));
    }
    map
}

/// Kinematic-wave parameters of one stream (a whole single-pipe link, or one
/// pipe of a multi-pipe link) — what the mechanism checks compare against.
struct StreamParams {
    length: f64,
    speed: f64,
    capacity: f64,
    storage_exact: f64,
    storage_veh: Option<usize>,
    wave_delay: f64,
    w_positive: bool,
}

impl StreamParams {
    fn from_link(link: &crate::model::Link) -> Self {
        Self {
            length: link.length,
            speed: link.speed,
            capacity: link.capacity,
            storage_exact: link.storage_exact(),
            storage_veh: link.storage_veh(),
            wave_delay: link.wave_delay(),
            w_positive: link.fd.w > 0.0,
        }
    }

    fn from_pipe(link: &crate::model::Link, pipe: &crate::model::Pipe) -> Self {
        Self {
            length: link.length,
            speed: link.speed,
            capacity: pipe.capacity,
            storage_exact: pipe.storage_exact,
            storage_veh: pipe.storage_veh,
            wave_delay: pipe.wave_delay,
            w_positive: link.fd.w > 0.0,
        }
    }
}

// ── Check builders ────────────────────────────────────────────────────────────

/// Ceiling for relative errors: keeps `error_pct` finite so it serializes as a
/// number (serde_json turns NaN/Infinity into `null`, which would be
/// indistinguishable from "not computed").
const ERR_CEILING_PCT: f64 = 1e9;

fn rel_err(measured: f64, expected: f64) -> f64 {
    if expected.abs() < 1e-12 {
        return if measured.abs() < 1e-12 {
            0.0
        } else {
            ERR_CEILING_PCT
        };
    }
    (((measured - expected) / expected).abs() * 100.0).min(ERR_CEILING_PCT)
}

fn check(
    id: &str,
    name: &str,
    status: CheckStatus,
    measured: Option<f64>,
    expected: Option<f64>,
    unit: &str,
    detail: String,
) -> Check {
    let error_pct = match (measured, expected) {
        (Some(m), Some(e)) => Some(rel_err(m, e)),
        _ => None,
    };
    Check {
        id: id.to_string(),
        name: name.to_string(),
        status,
        measured,
        expected,
        unit: unit.to_string(),
        error_pct,
        detail,
    }
}

fn not_exercised(id: &str, name: &str, detail: &str) -> Check {
    check(
        id,
        name,
        CheckStatus::NotExercised,
        None,
        None,
        "",
        detail.to_string(),
    )
}

/// Fastest traversal must equal L/u; faster is unphysical.
fn check_free_flow(sp: &StreamParams, p: &LinkPassages) -> Check {
    let id = "free_flow_traversal";
    let name = "Free-flow traversal time";
    if p.travel_times.is_empty() {
        return not_exercised(id, name, "No completed traversal on this link.");
    }
    let expected = sp.length / sp.speed;
    let min_tt = p.travel_times.iter().cloned().fold(f64::INFINITY, f64::min);
    let detail = format!(
        "Fastest of {} traversals vs L/u = {:.2}/{:.2}. Faster than free flow would violate the FD.",
        p.travel_times.len(),
        sp.length,
        sp.speed
    );
    let status = if min_tt < expected * (1.0 - TOL_MECHANISM) {
        CheckStatus::Fail // traversed faster than free flow
    } else if rel_err(min_tt, expected) <= TOL_MECHANISM * 100.0 {
        CheckStatus::Pass
    } else {
        // Every vehicle was delayed — congestion, not a defect.
        CheckStatus::NotExercised
    };
    check(id, name, status, Some(min_tt), Some(expected), "s", detail)
}

/// No two entries may be closer than the capacity headway 1/C.
fn check_inflow_capacity(sp: &StreamParams, p: &LinkPassages) -> Check {
    let id = "inflow_capacity";
    let name = "Inflow ≤ capacity";
    if p.entries.len() < 2 {
        return not_exercised(id, name, "Fewer than two entries — headway not measurable.");
    }
    if sp.capacity <= 0.0 {
        return not_exercised(id, name, "Stream has no positive capacity parameter.");
    }
    let expected = 1.0 / sp.capacity;
    let min_headway = p
        .entries
        .windows(2)
        .map(|w| w[1] - w[0])
        .fold(f64::INFINITY, f64::min);
    let exercised = rel_err(min_headway, expected) <= TOL_MECHANISM * 100.0;
    let status = if min_headway < expected * (1.0 - TOL_MECHANISM) {
        CheckStatus::Fail
    } else {
        CheckStatus::Pass
    };
    let detail = format!(
        "Smallest of {} entry headways vs 1/C. {}",
        p.entries.len() - 1,
        if exercised {
            "The capacity constraint was active (inflow reached C)."
        } else {
            "Inflow never reached capacity — bound respected with margin."
        }
    );
    check(
        id,
        name,
        status,
        Some(min_headway),
        Some(expected),
        "s",
        detail,
    )
}

/// Peak simultaneous occupancy must not exceed the jam accumulation kx·L·lanes.
fn check_storage_bound(sp: &StreamParams, p: &LinkPassages) -> Check {
    let id = "storage_bound";
    let name = "Storage ≤ jam accumulation";
    let Some(dn) = sp.storage_veh else {
        return not_exercised(id, name, "Storage constraint disabled (kx ≤ 0).");
    };
    if p.entries.is_empty() {
        return not_exercised(id, name, "No entry on this link.");
    }
    // Sweep: +1 at entry, −1 at exit; exits first at equal timestamps.
    let mut events: Vec<(f64, i32)> = p.entries.iter().map(|&t| (t, 1)).collect();
    events.extend(p.exits.iter().map(|&t| (t, -1)));
    events.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    let mut occ = 0i64;
    let mut max_occ = 0i64;
    for (_, delta) in events {
        occ += delta as i64;
        max_occ = max_occ.max(occ);
    }
    let status = if max_occ as usize > dn {
        CheckStatus::Fail
    } else {
        CheckStatus::Pass
    };
    let detail = format!(
        "Peak simultaneous vehicles vs ⌈kx·L·lanes⌉ = ⌈{:.2}⌉. Utilisation {:.0}%.",
        sp.storage_exact,
        100.0 * max_occ as f64 / dn as f64
    );
    check(
        id,
        name,
        status,
        Some(max_occ as f64),
        Some(dn as f64),
        "veh",
        detail,
    )
}

/// When spillback occurred, entries gated by the storage constraint must lag
/// the enabling exit by exactly the backward wave delay.
fn check_backward_wave(sp: &StreamParams, p: &LinkPassages) -> Check {
    let id = "backward_wave";
    let name = "Backward wave speed";
    let Some(dn) = sp.storage_veh else {
        return not_exercised(id, name, "Storage constraint disabled (kx ≤ 0).");
    };
    if !sp.w_positive {
        return not_exercised(id, name, "Non-positive wave speed parameter.");
    }
    let expected = sp.wave_delay;
    let mut binding_delays = Vec::new();
    let mut violations = 0usize;
    for n in dn..p.entries.len() {
        let Some(&t_free) = p.exits.get(n - dn) else {
            break;
        };
        let delay = p.entries[n] - t_free;
        if delay < expected * (1.0 - TOL_MECHANISM) {
            violations += 1;
        } else if rel_err(delay, expected) <= TOL_MECHANISM * 100.0 {
            binding_delays.push(delay);
        }
    }
    if violations > 0 {
        return check(
            id,
            name,
            CheckStatus::Fail,
            None,
            Some(expected),
            "s",
            format!(
                "{} entries occurred before the freed slot's backward wave reached the entry (< L/w after the enabling exit).",
                violations
            ),
        );
    }
    if binding_delays.is_empty() {
        return not_exercised(
            id,
            name,
            "No spillback observed — the storage constraint never gated an entry.",
        );
    }
    let mean = binding_delays.iter().sum::<f64>() / binding_delays.len() as f64;
    let detail = format!(
        "{} spillback-gated entries; mean measured slot delay vs L/w (+ fractional-slot correction). \
         The entry↔exit pairing assumes FIFO discipline — cross-check the node FIFO check.",
        binding_delays.len()
    );
    check(
        id,
        name,
        CheckStatus::Pass,
        Some(mean),
        Some(expected),
        "s",
        detail,
    )
}

/// Mean |deviation| of binned (k, q) observations from the triangular FD.
/// A link that ends at a stop line — signal-controlled movements, a
/// priority junction, a roundabout, a toll — carries interrupted flow: its
/// averages mix a standing queue with free flow, which the fundamental
/// diagram does not describe.
fn interrupted_flow(scenario: &Scenario, link: &crate::model::Link) -> bool {
    let node = scenario.nodes.get(link.node_down);
    node.is_some_and(|n| n.roundabout.is_some() || n.junction.is_some())
        || scenario.signals.groups.iter().any(|g| g.movements.iter().any(|m| m.from == link.id))
        || scenario.signals.tolls.iter().any(|t| t.link == link.id)
}

fn fd_check(scenario: &Scenario, link: &crate::model::Link, fd_errors: &HashMap<LinkID, Vec<f64>>) -> Check {
    if interrupted_flow(scenario, link) {
        return not_exercised(
            "fd_adherence",
            "Fundamental diagram adherence",
            "Interrupted flow: the link ends at a stop line (signal, priority junction, roundabout \
             or toll), whose queue the fundamental diagram does not describe.",
        );
    }
    check_fd_adherence(link.id, fd_errors)
}

/// Per internal node, the relative imbalance per time bin between the
/// vehicles leaving its incoming links and those entering its outgoing
/// links — node passages, not the outgoing links' exits, which would also
/// count the storage building up on those links.
fn node_passage_errors(scenario: &Scenario, bin_size: f64) -> HashMap<NodeID, Vec<f64>> {
    let bin = |t: f64| ((t - scenario.start_time) / bin_size).floor().max(0.0) as usize;
    let mut inflow: HashMap<(NodeID, usize), f64> = HashMap::new();
    let mut outflow: HashMap<(NodeID, usize), f64> = HashMap::new();
    let internal = |n: NodeID| {
        scenario.nodes.get(n).is_some_and(|n| !n.incoming_links.is_empty() && !n.outgoing_links.is_empty())
    };
    for veh in &scenario.vehicles {
        for (i, &lid) in veh.path.iter().enumerate() {
            let Some(link) = scenario.links.get(lid) else { continue };
            if let Some(t) = veh.link_exit_time(i) {
                if internal(link.node_down) {
                    *inflow.entry((link.node_down, bin(t))).or_default() += 1.0;
                }
            }
            if let Some(t) = veh.link_entry_time(i) {
                if internal(link.node_up) {
                    *outflow.entry((link.node_up, bin(t))).or_default() += 1.0;
                }
            }
        }
    }
    let mut keys: Vec<(NodeID, usize)> = inflow.keys().chain(outflow.keys()).copied().collect();
    keys.sort_unstable();
    keys.dedup();
    let mut errors: HashMap<NodeID, Vec<f64>> = HashMap::new();
    for key in keys {
        let i = inflow.get(&key).copied().unwrap_or(0.0);
        let o = outflow.get(&key).copied().unwrap_or(0.0);
        let e = if i > 0.0 { (o - i) / i * 100.0 } else { 0.0 };
        errors.entry(key.0).or_default().push(e.abs());
    }
    errors
}

fn check_fd_adherence(link_id: LinkID, fd_errors: &HashMap<LinkID, Vec<f64>>) -> Check {
    let id = "fd_adherence";
    let name = "Fundamental diagram adherence";
    let Some(errors) = fd_errors.get(&link_id).filter(|e| !e.is_empty()) else {
        return not_exercised(id, name, "No (density, flow) observations for this link.");
    };
    let mean = errors.iter().sum::<f64>() / errors.len() as f64;
    let status = if mean < FD_PASS_PCT {
        CheckStatus::Pass
    } else if mean < FD_WARN_PCT {
        CheckStatus::Warn
    } else {
        CheckStatus::Fail
    };
    let detail = format!(
        "Mean |simulated − theoretical flow| over {} time bins, both FD branches. \
         Bins above 90% of jam density are excluded (non-uniform queue/discharge states \
         where the FD does not describe the spatial average), and bins with fewer than 30 \
         vehicles (edge effects dominate).",
        errors.len()
    );
    Check {
        id: id.to_string(),
        name: name.to_string(),
        status,
        measured: Some(mean),
        expected: Some(0.0),
        unit: "%".to_string(),
        error_pct: Some(mean),
        detail,
    }
}

/// FIFO at the node: per incoming (link, pipe), passage order must follow
/// arrival order. FIFO holds only *within* a pipe — cross-pipe overtaking is
/// legal multi-pipe physics, so passages are grouped by the pipe each vehicle
/// actually took (`pipes_taken`).
fn check_node_fifo(scenario: &Scenario, node_id: NodeID) -> Check {
    let id = "fifo_discipline";
    let name = "FIFO discipline";
    let Some(node) = scenario.nodes.get(node_id) else {
        return not_exercised(id, name, "Node not found in scenario.");
    };
    let mut pairs_checked = 0usize;
    let mut violations = 0usize;

    for &in_link in &node.incoming_links {
        let Some(link) = scenario.links.get(in_link) else {
            continue;
        };
        let ff = if link.speed > 0.0 {
            link.length / link.speed
        } else {
            0.0
        };
        let n_pipes = link.pipes.len().max(1);

        // (arrival at node, passage at node), grouped by pipe taken.
        let mut per_pipe: Vec<Vec<(f64, f64)>> = vec![Vec::new(); n_pipes];
        for veh in &scenario.vehicles {
            for (i, &lid) in veh.path.iter().enumerate() {
                if lid != in_link {
                    continue;
                }
                if let (Some(t_in), Some(t_out)) = (veh.link_entry_time(i), veh.link_exit_time(i)) {
                    let pipe = veh.pipes_taken.get(i).copied().unwrap_or(0);
                    if let Some(bucket) = per_pipe.get_mut(usize::from(pipe)) {
                        bucket.push((t_in + ff, t_out));
                    }
                }
            }
        }
        for passages in &mut per_pipe {
            passages.sort_by(|a, b| a.0.total_cmp(&b.0));
            for w in passages.windows(2) {
                pairs_checked += 1;
                if w[1].1 < w[0].1 - 1e-9 {
                    violations += 1;
                }
            }
        }
    }

    if pairs_checked == 0 {
        return not_exercised(id, name, "No consecutive passages to compare.");
    }
    let status = if violations == 0 {
        CheckStatus::Pass
    } else {
        CheckStatus::Fail
    };
    check(
        id,
        name,
        status,
        Some(violations as f64),
        Some(0.0),
        "violations",
        format!(
            "{} consecutive arrival pairs per incoming (link, pipe) checked for in-pipe overtaking at the node.",
            pairs_checked
        ),
    )
}

/// One crossing of a roundabout node, reconstructed from a vehicle record.
struct RingCrossing {
    arm_in: usize,
    arm_out: usize,
    approach: (LinkID, PipeIdx),
    t_in: f64,
    t_out: f64,
}

fn ring_crossings(scenario: &Scenario, node_id: NodeID, spec: &RoundaboutSpec) -> Vec<RingCrossing> {
    let mut out = Vec::new();
    for veh in &scenario.vehicles {
        for (i, w) in veh.path.windows(2).enumerate() {
            let Some(l) = scenario.links.get(w[0]) else { continue };
            if l.node_down != node_id {
                continue;
            }
            let arm_in = spec.arms.iter().position(|a| a.in_link == Some(w[0]));
            let arm_out = spec.arms.iter().position(|a| a.out_link == Some(w[1]));
            let (Some(arm_in), Some(arm_out)) = (arm_in, arm_out) else { continue };
            let (Some(t_in), Some(t_out)) = (veh.link_exit_time(i), veh.link_entry_time(i + 1)) else {
                continue;
            };
            let pipe = veh.pipes_taken.get(i).copied().unwrap_or(0);
            out.push(RingCrossing { arm_in, arm_out, approach: (w[0], pipe), t_in, t_out });
        }
    }
    out
}

/// Smallest gap between consecutive times of each group (sorted in place).
fn min_headway<K: Ord>(groups: &mut std::collections::BTreeMap<K, Vec<f64>>) -> Option<f64> {
    groups
        .values_mut()
        .flat_map(|ts| {
            ts.sort_by(f64::total_cmp);
            ts.windows(2).map(|w| w[1] - w[0]).collect::<Vec<_>>()
        })
        .min_by(f64::total_cmp)
}

/// Roundabout mechanism checks, from vehicle records alone: ring traversals
/// never beat free flow, entries respect the follow-up time per approach
/// lane and the ring headway per entry point, exits respect the ring
/// headway, and vehicles of one movement leave in the order they entered
/// (FIFO ring). The gap-acceptance rule itself is exercised by the
/// `roundabout_siegloch` benchmark.
fn check_roundabout(scenario: &Scenario, node_id: NodeID, spec: &RoundaboutSpec) -> Vec<Check> {
    use std::collections::BTreeMap;
    const EPS: f64 = 1e-9;
    let crossings = ring_crossings(scenario, node_id, spec);
    if crossings.is_empty() {
        return vec![not_exercised("ring_traversal", "Ring free-flow traversal", "No vehicle crossed the roundabout.")];
    }
    let geom = RingGeometry::new(spec);
    let headway = 1.0 / spec.capacity();

    // 1. Free flow on the ring.
    let slack = crossings
        .iter()
        .map(|c| {
            let ff = geom.arc(geom.merge_point[c.arm_in], geom.exit_point[c.arm_out]) / spec.speed;
            c.t_out - c.t_in - ff
        })
        .fold(f64::INFINITY, f64::min);
    let traversal = check(
        "ring_traversal",
        "Ring free-flow traversal",
        if slack >= -EPS { CheckStatus::Pass } else { CheckStatus::Fail },
        Some(slack),
        Some(0.0),
        "s",
        format!(
            "Smallest (ring time − free-flow arc time) over {} crossings; negative means faster than free flow.",
            crossings.len()
        ),
    );

    // 2. Entries: follow-up per approach lane, ring headway per entry point.
    let mut per_lane: BTreeMap<(LinkID, PipeIdx), Vec<f64>> = BTreeMap::new();
    let mut per_merge: BTreeMap<usize, Vec<f64>> = BTreeMap::new();
    let mut per_exit: BTreeMap<usize, Vec<f64>> = BTreeMap::new();
    for c in &crossings {
        per_lane.entry(c.approach).or_default().push(c.t_in);
        per_merge.entry(c.arm_in).or_default().push(c.t_in);
        per_exit.entry(c.arm_out).or_default().push(c.t_out);
    }
    let lane_min = min_headway(&mut per_lane);
    let merge_min = min_headway(&mut per_merge);
    let entry_ok = lane_min.is_none_or(|h| h >= spec.follow_up - EPS) && merge_min.is_none_or(|h| h >= headway - EPS);
    let entries = match (lane_min, merge_min) {
        (None, None) => not_exercised("ring_entry_headway", "Entry headways", "No two entries to compare."),
        _ => check(
            "ring_entry_headway",
            "Entry headways",
            if entry_ok { CheckStatus::Pass } else { CheckStatus::Fail },
            lane_min,
            Some(spec.follow_up),
            "s",
            format!(
                "Smallest entry headway per approach lane (≥ t_f = {:.3} s) and per entry point \
                 ({} s, ≥ 1/C = {:.3} s).",
                spec.follow_up,
                merge_min.map_or("n/a".to_string(), |h| format!("{h:.3}")),
                headway
            ),
        ),
    };

    // 3. Exits: ring headway per exit point.
    let exit_min = min_headway(&mut per_exit);
    let exits = match exit_min {
        None => not_exercised("ring_exit_headway", "Exit headways", "No two exits at one arm."),
        Some(h) => check(
            "ring_exit_headway",
            "Exit headways",
            if h >= headway - EPS { CheckStatus::Pass } else { CheckStatus::Fail },
            Some(h),
            Some(headway),
            "s",
            format!("Smallest exit headway at one exit point (≥ 1/C = {headway:.3} s)."),
        ),
    };

    // 4. FIFO ring: within one movement, exit order follows entry order.
    let mut per_move: BTreeMap<(usize, usize), Vec<(f64, f64)>> = BTreeMap::new();
    for c in &crossings {
        per_move.entry((c.arm_in, c.arm_out)).or_default().push((c.t_in, c.t_out));
    }
    let mut pairs = 0usize;
    let mut violations = 0usize;
    for v in per_move.values_mut() {
        v.sort_by(|a, b| a.0.total_cmp(&b.0));
        for w in v.windows(2) {
            pairs += 1;
            if w[1].1 < w[0].1 - EPS {
                violations += 1;
            }
        }
    }
    let fifo = if pairs == 0 {
        not_exercised("ring_fifo", "Ring FIFO", "No two crossings of one movement.")
    } else {
        check(
            "ring_fifo",
            "Ring FIFO",
            if violations == 0 { CheckStatus::Pass } else { CheckStatus::Fail },
            Some(violations as f64),
            Some(0.0),
            "violations",
            format!("{pairs} consecutive crossings per movement checked for overtaking on the ring."),
        )
    };

    vec![traversal, entries, exits, fifo]
}

/// Priority-junction check, from the vehicle records and the applied signal
/// switches: every departure of a minor movement respects, against each
/// movement it gives way to (unless that one was on red), `lead` after its
/// last passage and `lag` before its next arrival at the node, plus the
/// follow-up time and the stop time of its lane.
fn check_junction(scenario: &Scenario, node_id: NodeID, spec: &crate::junction::JunctionSpec) -> Check {
    const EPS: f64 = 1e-9;
    let id = "junction_priority";
    let name = "Priority (gap acceptance)";
    // Per movement: passages and arrivals at the node; per lane: departures.
    let mut passes: HashMap<(LinkID, LinkID), Vec<f64>> = HashMap::new();
    // Conflicting vehicles that reached the node and went through without
    // stopping (arrival = passage): those a minor vehicle must not cut in
    // front of. One stuck at the line (behind a turning vehicle, a full
    // exit) is no threat.
    type LaneDeparture = (f64, f64, Option<LinkID>, f64);
    let mut arrivals: HashMap<(LinkID, LinkID), Vec<f64>> = HashMap::new();
    let mut lanes: HashMap<(LinkID, PipeIdx), Vec<LaneDeparture>> = HashMap::new();
    for veh in &scenario.vehicles {
        for (i, &lid) in veh.path.iter().enumerate() {
            let Some(link) = scenario.links.get(lid) else { continue };
            if link.node_down != node_id {
                continue;
            }
            let (Some(t_in), Some(t_out)) = (veh.link_entry_time(i), veh.link_exit_time(i)) else { continue };
            let next = veh.path.get(i + 1).copied();
            let arrival = t_in + link.length / link.speed;
            if let Some(n) = next {
                passes.entry((lid, n)).or_default().push(t_out);
                if (t_out - arrival).abs() < EPS {
                    arrivals.entry((lid, n)).or_default().push(arrival);
                }
            }
            let pipe = veh.pipes_taken.get(i).copied().unwrap_or(0);
            lanes.entry((lid, pipe)).or_default().push((t_out, arrival, next, t_in));
        }
    }
    for v in passes.values_mut().chain(arrivals.values_mut()) {
        v.sort_by(f64::total_cmp);
    }
    // Red at t for a movement controlled by a signal group.
    let red_at = |from: LinkID, to: LinkID, t: f64| -> bool {
        let Some(g) = scenario.signals.groups.iter().position(|g| {
            g.node == node_id && g.movements.iter().any(|m| m.from == from && (m.to.is_none() || m.to == Some(to)))
        }) else {
            return false;
        };
        let state = scenario
            .signal_log
            .iter()
            .rfind(|r| r.group == g && r.time <= t)
            .map(|r| r.green)
            .unwrap_or(true);
        !state
    };
    let mut checked = 0usize;
    let mut violations = 0usize;
    for (&(link, _pipe), deps) in lanes.iter_mut() {
        // FIFO within a lane: simultaneous departures in entry order.
        deps.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.3.total_cmp(&b.3)));
        let mut prev: Option<f64> = None;
        for &(t, arrival, next, _) in deps.iter() {
            let m = next.and_then(|n| spec.movement(link, n));
            if let Some(m) = m.filter(|m| !m.yields_to.is_empty()) {
                checked += 1;
                let mut ok = true;
                if let Some(p) = prev {
                    ok &= t >= p + m.follow_up - EPS;
                }
                ok &= t >= arrival.max(prev.unwrap_or(f64::NEG_INFINITY)) + m.stop_time - EPS;
                for &[a, b] in &m.yields_to {
                    if let Some(ps) = passes.get(&(a, b)) {
                        // Last conflicting passage at or before t.
                        let k = ps.partition_point(|&x| x <= t + EPS);
                        if k > 0 && ps[k - 1] > t - m.lead() + EPS {
                            ok = false;
                        }
                    }
                    if let Some(arr) = arrivals.get(&(a, b)) {
                        let k = arr.partition_point(|&x| x < t - EPS);
                        if k < arr.len() && arr[k] < t + m.lag() - EPS && !red_at(a, b, t) {
                            ok = false;
                        }
                    }
                }
                if !ok {
                    violations += 1;
                }
            }
            prev = Some(t);
        }
    }
    if checked == 0 {
        return not_exercised(id, name, "No minor movement was used.");
    }
    check(
        id,
        name,
        if violations == 0 { CheckStatus::Pass } else { CheckStatus::Fail },
        Some(violations as f64),
        Some(0.0),
        "departures",
        format!("{checked} minor departures checked for lead, lag, follow-up and stop time against the movements they give way to."),
    )
}

/// Signal group mechanism checks, from the applied switches
/// (`signal_log`) and the vehicle records: no vehicle takes a controlled
/// movement on red, and no green lets more vehicles through than its cap.
fn check_signal_group(scenario: &Scenario, g: usize) -> Vec<Check> {
    let group = &scenario.signals.groups[g];
    let mut switches: Vec<(f64, bool, bool)> = scenario
        .signal_log
        .iter()
        .filter(|r| r.group == g)
        .map(|r| (r.time, r.green, r.capped))
        .collect();
    switches.sort_by(|a, b| a.0.total_cmp(&b.0));
    // State at t: the last switch at or before t (a switch at t applies to
    // passages at t: switches are processed first at an instant).
    let green_index = |t: f64| -> Option<usize> {
        let i = switches.partition_point(|&(s, _, _)| s <= t);
        (i > 0 && switches[i - 1].1).then(|| i - 1)
    };
    let controls = |from: LinkID, to: Option<LinkID>| {
        group.movements.iter().any(|m| m.from == from && (m.to.is_none() || m.to == to))
    };
    // A permitted movement (one that gives way at a priority junction) may
    // clear up to `sneakers` vehicles within `clearance` of a red.
    let permitted = |from: LinkID, to: Option<LinkID>| {
        scenario.nodes.get(group.node).and_then(|n| n.junction.as_ref()).is_some_and(|j| {
            to.and_then(|t| j.movement(from, t)).is_some_and(|m| !m.yields_to.is_empty())
        })
    };
    let mut passages = 0usize;
    let mut on_red = 0usize;
    let mut sneakers = 0usize;
    let mut per_green: HashMap<usize, u32> = HashMap::new();
    let mut per_red: HashMap<usize, u32> = HashMap::new();
    for veh in &scenario.vehicles {
        for (i, &lid) in veh.path.iter().enumerate() {
            let Some(link) = scenario.links.get(lid) else { continue };
            let next = veh.path.get(i + 1).copied();
            if link.node_down != group.node || !controls(lid, next) {
                continue;
            }
            let Some(t) = veh.link_exit_time(i) else { continue };
            passages += 1;
            match green_index(t) {
                // The cap applies to plan or controller greens, not to the
                // dark signal.
                Some(k) if switches[k].2 => *per_green.entry(k).or_default() += 1,
                Some(_) => {}
                None => {
                    let r = switches.partition_point(|&(s, _, _)| s <= t);
                    let red_start = if r > 0 { switches[r - 1].0 } else { f64::NEG_INFINITY };
                    let count = per_red.entry(r).or_default();
                    if permitted(lid, next) && t < red_start + group.clearance && *count < group.sneakers {
                        *count += 1;
                        sneakers += 1;
                    } else {
                        on_red += 1;
                    }
                }
            }
        }
    }
    if passages == 0 {
        return vec![not_exercised("signal_red", "No passage on red", "No vehicle took a movement of this group.")];
    }
    let red = check(
        "signal_red",
        "No passage on red",
        if on_red == 0 { CheckStatus::Pass } else { CheckStatus::Fail },
        Some(on_red as f64),
        Some(0.0),
        "vehicles",
        format!(
            "{passages} passages through the group's movements checked against {} applied switches ({sneakers} permitted vehicles cleared just after a red).",
            switches.len()
        ),
    );
    let mut checks = vec![red];
    if let Some(cap) = group.vehicles_per_green {
        let worst = per_green.values().copied().max().unwrap_or(0);
        checks.push(check(
            "signal_cap",
            "Vehicles per green",
            if worst <= cap { CheckStatus::Pass } else { CheckStatus::Fail },
            Some(f64::from(worst)),
            Some(f64::from(cap)),
            "vehicles",
            format!("Largest number of vehicles passed in one green, over {} greens (cap {cap}).", per_green.len()),
        ));
    }
    checks
}

/// Flow conservation at the node, aggregated over time bins.
fn check_node_conservation(node_id: NodeID, errors: &HashMap<NodeID, Vec<f64>>) -> Check {
    let id = "flow_conservation";
    let name = "Flow conservation";
    let Some(errs) = errors.get(&node_id).filter(|e| !e.is_empty()) else {
        return not_exercised(id, name, "No flow through this node.");
    };
    let mean = errs.iter().sum::<f64>() / errs.len() as f64;
    let status = if mean < CONS_PASS_PCT {
        CheckStatus::Pass
    } else if mean < CONS_WARN_PCT {
        CheckStatus::Warn
    } else {
        CheckStatus::Fail
    };
    Check {
        id: id.to_string(),
        name: name.to_string(),
        status,
        measured: Some(mean),
        expected: Some(0.0),
        unit: "%".to_string(),
        error_pct: Some(mean),
        detail: format!(
            "Mean |entries into outgoing links − exits from incoming links| / exits over {} time bins (node passages).",
            errs.len()
        ),
    }
}

/// Demand served at an entry node + waiting statistics.
fn check_entry_demand(scenario: &Scenario, node_id: NodeID) -> Vec<Check> {
    let vehicles: Vec<_> = scenario
        .vehicles
        .iter()
        .filter(|v| v.origin == node_id)
        .collect();
    if vehicles.is_empty() {
        return vec![not_exercised(
            "demand_served",
            "Demand served",
            "No vehicle originates at this entry.",
        )];
    }
    let served: Vec<_> = vehicles
        .iter()
        .filter(|v| !v.node_times.is_empty())
        .collect();
    let pct = 100.0 * served.len() as f64 / vehicles.len() as f64;
    let status = if pct >= SERVED_PASS_PCT {
        CheckStatus::Pass
    } else if pct >= SERVED_WARN_PCT {
        CheckStatus::Warn
    } else {
        CheckStatus::Fail
    };
    let waits: Vec<f64> = served
        .iter()
        .map(|v| v.node_times[0] - v.start_time)
        .collect();
    let mean_wait = if waits.is_empty() {
        0.0
    } else {
        waits.iter().sum::<f64>() / waits.len() as f64
    };
    let max_wait = waits.iter().cloned().fold(0.0, f64::max);
    vec![check(
        "demand_served",
        "Demand served",
        status,
        Some(pct),
        Some(100.0),
        "%",
        format!(
            "{}/{} scheduled vehicles entered. Entry wait: mean {:.1}s, max {:.1}s. \
             Unserved demand means the entry queue never drained before simulation end.",
            served.len(),
            vehicles.len(),
            mean_wait,
            max_wait
        ),
    )]
}

// ── Entry point ───────────────────────────────────────────────────────────────

/// Run all realism checks on a completed simulation.
///
/// `bin_size` is the time-bin width (seconds) used for the FD-adherence and
/// flow-conservation checks (60 s is the conventional choice).
pub fn verify_scenario(scenario: &Scenario, bin_size: f64) -> VerificationReport {
    let passages = collect_passages(scenario);

    // Shared binned analyses (one pass for the whole network).
    let stats = compute_link_stats(scenario, bin_size);
    let n_bins = (scenario.duration / bin_size).ceil() as usize;
    let fd_points = compute_fd_points(scenario, &stats, bin_size, n_bins);
    let mut fd_errors: HashMap<LinkID, Vec<f64>> = HashMap::new();
    for pt in &fd_points {
        // Too few vehicles in the bin: exit counts and occupancy times
        // disagree by edge effects alone (a vehicle more or less), whatever
        // the dynamics. Require a minimum sample.
        if pt.flow * bin_size / 3600.0 < FD_MIN_VEHICLES {
            continue;
        }
        if pt.regime == FDRegime::QueueOverflow {
            continue;
        }
        // Near-jam bins are non-uniform states (standing queue + discharge
        // front): the FD predicts the *local* flow, not the spatial average,
        // so the comparison is meaningless there. Exclude ≥ 90% of kx.
        let near_jam = scenario
            .links
            .get(pt.link_id)
            .map(|l| {
                let kx_link_veh_km = l.fd.kx * f64::from(l.num_lanes) * 1000.0;
                kx_link_veh_km > 0.0 && pt.density >= 0.9 * kx_link_veh_km
            })
            .unwrap_or(false);
        if near_jam {
            continue;
        }
        fd_errors
            .entry(pt.link_id)
            .or_default()
            .push(pt.error_pct.abs());
    }
    let cons_errors = node_passage_errors(scenario, bin_size);

    let mut elements = Vec::new();
    let empty = LinkPassages::default();

    // Links — single-pipe: one element with all five checks; multi-pipe: one
    // element per pipe (mechanism checks against the pipe's own parameters)
    // plus a link-level element carrying the whole-link FD adherence.
    for link in &scenario.links {
        let n_pipes = link.pipes.len().max(1);
        if n_pipes == 1 {
            let p = passages.get(&(link.id, 0)).unwrap_or(&empty);
            let sp = StreamParams::from_link(link);
            let checks = vec![
                check_free_flow(&sp, p),
                check_inflow_capacity(&sp, p),
                check_storage_bound(&sp, p),
                check_backward_wave(&sp, p),
                fd_check(scenario, link, &fd_errors),
            ];
            elements.push(element_report(
                ElementType::Link,
                link.id,
                None,
                format!("Link {}", link.id),
                checks,
            ));
        } else {
            for (pi, pipe) in link.pipes.iter().enumerate() {
                let p = passages.get(&(link.id, pi as PipeIdx)).unwrap_or(&empty);
                let sp = StreamParams::from_pipe(link, pipe);
                let checks = vec![
                    check_free_flow(&sp, p),
                    check_inflow_capacity(&sp, p),
                    check_storage_bound(&sp, p),
                    check_backward_wave(&sp, p),
                ];
                elements.push(element_report(
                    ElementType::Link,
                    link.id,
                    Some(pi as PipeIdx),
                    format!("Link {} · pipe {}", link.id, pi),
                    checks,
                ));
            }
            let checks = vec![fd_check(scenario, link, &fd_errors)];
            elements.push(element_report(
                ElementType::Link,
                link.id,
                None,
                format!("Link {}", link.id),
                checks,
            ));
        }
    }

    // Internal nodes
    for node in &scenario.nodes {
        match node.node_type {
            NodeType::Internal => {
                let mut checks = vec![
                    check_node_conservation(node.id, &cons_errors),
                    check_node_fifo(scenario, node.id),
                ];
                if let Some(spec) = &node.roundabout {
                    checks.extend(check_roundabout(scenario, node.id, spec));
                }
                if let Some(spec) = &node.junction {
                    checks.push(check_junction(scenario, node.id, spec));
                }
                elements.push(element_report(
                    ElementType::Node,
                    node.id,
                    None,
                    format!("Node {}", node.id),
                    checks,
                ));
            }
            NodeType::Entry => {
                let checks = check_entry_demand(scenario, node.id);
                elements.push(element_report(
                    ElementType::Entry,
                    node.id,
                    None,
                    format!("Entry {}", node.id),
                    checks,
                ));
            }
            NodeType::Exit => {}
        }
    }

    // Signal groups
    for (g, group) in scenario.signals.groups.iter().enumerate() {
        elements.push(element_report(
            ElementType::Signal,
            g,
            None,
            format!("Signal group {}", group.id),
            check_signal_group(scenario, g),
        ));
    }

    // Worst elements first, then by id for stability.
    elements.sort_by(|a, b| {
        b.status
            .severity()
            .cmp(&a.status.severity())
            .then(a.element_id.cmp(&b.element_id))
    });

    let summary = summarize(&elements);
    VerificationReport { summary, elements }
}

fn element_report(
    element_type: ElementType,
    element_id: usize,
    pipe: Option<PipeIdx>,
    label: String,
    checks: Vec<Check>,
) -> ElementReport {
    let status = checks
        .iter()
        .map(|c| c.status)
        .max_by_key(|s| s.severity())
        .unwrap_or(CheckStatus::NotExercised);
    ElementReport {
        element_type,
        element_id,
        pipe,
        label,
        status,
        checks,
    }
}

fn summarize(elements: &[ElementReport]) -> VerificationSummary {
    let count = |s: CheckStatus| elements.iter().filter(|e| e.status == s).count();
    VerificationSummary {
        elements_checked: elements.len(),
        elements_pass: count(CheckStatus::Pass),
        elements_warn: count(CheckStatus::Warn),
        elements_fail: count(CheckStatus::Fail),
        elements_not_exercised: count(CheckStatus::NotExercised),
        checks_total: elements.iter().map(|e| e.checks.len()).sum(),
        checks_fail: elements
            .iter()
            .flat_map(|e| &e.checks)
            .filter(|c| c.status == CheckStatus::Fail)
            .count(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{FundamentalDiagram, Link, Node, Vehicle};
    use crate::simulation::Simulation;

    /// Entry(0) →L0→ Internal(1) →L1(bottleneck)→ Exit(2), demand exceeding
    /// the bottleneck: exercises capacity, storage, spillback and FIFO.
    fn congested_scenario() -> Scenario {
        let fd0 = FundamentalDiagram {
            u: 10.0,
            w: 5.0,
            kx: 0.02,
            c: 10.0,
        };
        let fd1 = FundamentalDiagram {
            u: 10.0,
            w: 5.0,
            kx: 10.0,
            c: 0.05,
        };
        let vehicles = (0..12)
            .map(|i| Vehicle::new(i, 0, vec![0, 1], 0.0, 0, 2))
            .collect();
        Scenario {
            nodes: vec![
                Node {
                    id: 0,
                    node_type: NodeType::Entry,
                    incoming_links: vec![],
                    outgoing_links: vec![0],
                    points: (0.0, 0.0),
                    signals: vec![], roundabout: None, junction: None,
                },
                Node {
                    id: 1,
                    node_type: NodeType::Internal,
                    incoming_links: vec![0],
                    outgoing_links: vec![1],
                    points: (1.0, 0.0),
                    signals: vec![], roundabout: None, junction: None,
                },
                Node {
                    id: 2,
                    node_type: NodeType::Exit,
                    incoming_links: vec![1],
                    outgoing_links: vec![],
                    points: (2.0, 0.0),
                    signals: vec![], roundabout: None, junction: None,
                },
            ],
            links: vec![
                Link::new(0, 0, 1, 100.0, 10.0, 2, 10.0, fd0, vec![]),
                Link::new(1, 1, 2, 100.0, 10.0, 1, 0.05, fd1, vec![]),
            ],
            vehicles,
            demand: vec![],
            start_time: 0.0,
            duration: 100_000.0,
            classes: vec!["car".to_string()],
            link_schedule: Vec::new(),
            assignment_excluded: Vec::new(), signals: Default::default(), signal_log: Vec::new(),
        }
    }

    fn run_and_verify() -> VerificationReport {
        let mut sim = Simulation::new(congested_scenario());
        sim.run().expect("simulation should complete");
        verify_scenario(&sim.scenario, 60.0)
    }

    #[test]
    fn engine_output_passes_all_mechanism_checks() {
        // Mechanism checks are event-exact reproductions of the engine's own
        // constraints — they must never fail on engine output. `fd_adherence`
        // is deliberately excluded: it compares whole-link binned averages to
        // the FD, which legitimately deviates during transient mixed states
        // (queue build-up/drain) even for a perfect LWR solver.
        let report = run_and_verify();
        let mechanism_failures: Vec<_> = report
            .elements
            .iter()
            .flat_map(|e| &e.checks)
            .filter(|c| c.status == CheckStatus::Fail && c.id != "fd_adherence")
            .collect();
        assert!(
            mechanism_failures.is_empty(),
            "engine output should never fail its own physics: {:#?}",
            mechanism_failures
        );
    }

    #[test]
    fn spillback_exercises_backward_wave_check() {
        let report = run_and_verify();
        let l0 = report
            .elements
            .iter()
            .find(|e| e.element_type == ElementType::Link && e.element_id == 0)
            .expect("link 0 report");
        let wave = l0
            .checks
            .iter()
            .find(|c| c.id == "backward_wave")
            .expect("wave check");
        assert_eq!(
            wave.status,
            CheckStatus::Pass,
            "spillback must be observed and match L/w: {:?}",
            wave
        );
        let measured = wave.measured.expect("measured wave delay");
        let expected = wave.expected.expect("expected wave delay");
        assert!((measured - expected).abs() / expected < 0.01);
    }

    #[test]
    fn capacity_check_reflects_bottleneck() {
        let report = run_and_verify();
        let l1 = report
            .elements
            .iter()
            .find(|e| e.element_type == ElementType::Link && e.element_id == 1)
            .expect("link 1 report");
        let cap = l1
            .checks
            .iter()
            .find(|c| c.id == "inflow_capacity")
            .expect("capacity check");
        assert_eq!(cap.status, CheckStatus::Pass);
        // Bottleneck inflow is capacity-bound: min headway == 1/C = 20 s.
        assert!((cap.measured.unwrap() - 20.0).abs() < 1e-6);
    }

    #[test]
    fn entry_reports_demand_served() {
        let report = run_and_verify();
        let entry = report
            .elements
            .iter()
            .find(|e| e.element_type == ElementType::Entry)
            .expect("entry report");
        let served = entry
            .checks
            .iter()
            .find(|c| c.id == "demand_served")
            .expect("served check");
        assert_eq!(served.status, CheckStatus::Pass);
        assert!((served.measured.unwrap() - 100.0).abs() < 1e-9);
    }

    #[test]
    fn corrupted_output_fails_free_flow_check() {
        let mut sim = Simulation::new(congested_scenario());
        sim.run().expect("simulation should complete");
        // Corrupt one vehicle: teleport across link 0 (faster than free flow).
        let v = &mut sim.scenario.vehicles[0];
        if v.node_times.len() >= 2 {
            v.node_times[1] = v.node_times[0] + 1.0; // L/u is 10 s
        }
        let report = verify_scenario(&sim.scenario, 60.0);
        let l0 = report
            .elements
            .iter()
            .find(|e| e.element_type == ElementType::Link && e.element_id == 0)
            .expect("link 0 report");
        let ff = l0
            .checks
            .iter()
            .find(|c| c.id == "free_flow_traversal")
            .unwrap();
        assert_eq!(
            ff.status,
            CheckStatus::Fail,
            "teleporting vehicle must be caught"
        );
    }
}

#[cfg(test)]
mod pipe_verification_tests {
    use super::*;
    use crate::model::{FundamentalDiagram, Link, Node, PipeSpec, Vehicle, ALL_CLASSES};
    use crate::simulation::Simulation;

    /// Multi-pipe corridor with a jammed exit pipe: the engine's own output
    /// must pass every per-pipe mechanism check.
    fn multi_pipe_report() -> VerificationReport {
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
        let mut l0 = Link::new(0, 0, 1, 100.0, 10.0, 2, 10.0, fd0, vec![]);
        l0.pipe_specs = vec![
            PipeSpec {
                lanes: 1,
                class_mask: ALL_CLASSES,
            },
            PipeSpec {
                lanes: 1,
                class_mask: ALL_CLASSES,
            },
        ];
        l0.moves.insert(2, vec![0]);
        l0.moves.insert(1, vec![1]);
        let mut l2 = Link::new(2, 1, 3, 100.0, 10.0, 1, 0.01, fd_out.clone(), vec![]);
        l2.fd.c = 0.01;
        let mut vehicles: Vec<Vehicle> = (0..4)
            .map(|i| Vehicle::new(i, 0, vec![0, 2], 0.0, 0, 3))
            .collect();
        vehicles.extend((4..6).map(|i| Vehicle::new(i, 0, vec![0, 1], 0.0, 0, 2)));
        let scenario = Scenario {
            nodes: vec![
                Node {
                    id: 0,
                    node_type: NodeType::Entry,
                    incoming_links: vec![],
                    outgoing_links: vec![0],
                    points: (0.0, 0.0),
                    signals: vec![], roundabout: None, junction: None,
                },
                Node {
                    id: 1,
                    node_type: NodeType::Internal,
                    incoming_links: vec![0],
                    outgoing_links: vec![1, 2],
                    points: (1.0, 0.0),
                    signals: vec![], roundabout: None, junction: None,
                },
                Node {
                    id: 2,
                    node_type: NodeType::Exit,
                    incoming_links: vec![1],
                    outgoing_links: vec![],
                    points: (2.0, 0.0),
                    signals: vec![], roundabout: None, junction: None,
                },
                Node {
                    id: 3,
                    node_type: NodeType::Exit,
                    incoming_links: vec![2],
                    outgoing_links: vec![],
                    points: (2.0, 1.0),
                    signals: vec![], roundabout: None, junction: None,
                },
            ],
            links: vec![
                l0,
                Link::new(1, 1, 2, 100.0, 10.0, 1, 10.0, fd_out, vec![]),
                l2,
            ],
            vehicles,
            demand: vec![],
            start_time: 0.0,
            duration: 100_000.0,
            classes: vec!["car".to_string()],
            link_schedule: Vec::new(),
            assignment_excluded: Vec::new(), signals: Default::default(), signal_log: Vec::new(),
        };
        let mut sim = Simulation::new(scenario);
        sim.run().unwrap();
        verify_scenario(&sim.scenario, 60.0)
    }

    #[test]
    fn multi_pipe_engine_output_passes_mechanism_checks() {
        let report = multi_pipe_report();
        let failures: Vec<_> = report
            .elements
            .iter()
            .flat_map(|e| &e.checks)
            .filter(|c| {
                // Binned indicators (fd_adherence, flow_conservation) are
                // sampling-noise dominated in this sparse synthetic scenario;
                // the event-exact mechanism checks must never fail.
                c.status == CheckStatus::Fail
                    && c.id != "fd_adherence"
                    && c.id != "flow_conservation"
            })
            .collect();
        assert!(failures.is_empty(), "{failures:#?}");
    }

    #[test]
    fn multi_pipe_link_gets_per_pipe_elements() {
        let report = multi_pipe_report();
        let pipes: Vec<_> = report
            .elements
            .iter()
            .filter(|e| {
                e.element_type == ElementType::Link && e.element_id == 0 && e.pipe.is_some()
            })
            .collect();
        assert_eq!(pipes.len(), 2, "one element per pipe of link 0");
        // The jammed exit pipe must have an exercised (Pass) backward-wave check.
        let wave = pipes[0]
            .checks
            .iter()
            .chain(pipes[1].checks.iter())
            .filter(|c| c.id == "backward_wave")
            .find(|c| c.status == CheckStatus::Pass);
        assert!(
            wave.is_some(),
            "spillback on the exit pipe must exercise the wave check"
        );
    }
}
