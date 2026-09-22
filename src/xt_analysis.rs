//! Space-time (XT) diagram analysis — Newell's CVC method + Edie's formulas.
//!
//! ## Theory
//!
//! The algorithm is based on the Moskowitz surface N(x, t): the cumulative
//! number of vehicles that have passed position x by time t.  For each link:
//!
//! 1. **Boundary CVCs** — collect entry times (CVCin) and exit times (CVCout)
//!    from the simulation's node_times.  Sorting them gives the n-th vehicle's
//!    passage time at each boundary.
//!
//! 2. **Interior CVC** (Newell's variational formula) — for any interior
//!    position x the passage time of vehicle n is:
//!      t_n(x) = max( CVCin[n] + x/u,              ← demand (free-flow wave)
//!                    CVCout[n−kx·(L−x)] + (L−x)/w ) ← supply (backward wave)
//!
//! 3. **Edie's formulas** per (dx, dt) cell:
//!      q = ΣΔx / (dx · dt)   flow    [veh/s → veh/h]
//!      k = ΣΔt / (dx · dt)   density [veh/m → veh/km]
//!      v = q / k              space-mean speed [m/s → km/h]
//!
//! ## Performance
//!
//! The key optimisation over the Python reference is to exploit the fact that
//! CVC arrays are sorted (vehicles numbered in order of passage).  For each
//! cell, two binary searches (`partition_point`) locate the contiguous slice of
//! vehicles present — O(log N) instead of O(N) per cell.  Links are processed
//! in parallel with Rayon.

use crate::model::{LinkID, PipeIdx, Scenario};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ── Public types ──────────────────────────────────────────────────────────────

/// Resolution parameters for the XT grid.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct XTParams {
    /// Spatial cell width in metres (default 50 m).
    pub dx_m: f64,
    /// Temporal cell width in seconds (default 60 s).
    pub dt_s: f64,
}

impl Default for XTParams {
    fn default() -> Self {
        Self {
            dx_m: 50.0,
            dt_s: 60.0,
        }
    }
}

/// Space-time analysis results for one link.
///
/// `flow[xi][ti]`, `density[xi][ti]` and `speed[xi][ti]` are indexed by
/// spatial bin `xi ∈ [0, nx)` and temporal bin `ti ∈ [0, nt)`.
///
/// `x_bins[xi]` is the left (upstream) edge of spatial cell xi in metres.
/// `t_bins[ti]` is the left (earlier) edge of temporal cell ti in absolute
/// seconds — consistent with the time axis used throughout the codebase.
#[derive(Debug, Serialize, Deserialize)]
pub struct XTDiagram {
    pub link_id: LinkID,
    /// Pipe index when this diagram covers a single pipe; None = whole link.
    #[serde(default)]
    pub pipe: Option<PipeIdx>,
    pub x_bins: Vec<f64>,       // metres from link start,  length nx
    pub t_bins: Vec<f64>,       // absolute seconds,        length nt
    pub flow: Vec<Vec<f64>>,    // [nx][nt]  veh/h
    pub density: Vec<Vec<f64>>, // [nx][nt]  veh/km
    pub speed: Vec<Vec<f64>>,   // [nx][nt]  km/h
    /// Free-flow speed in km/h — used to normalise V/u for the traficolor map.
    pub u_kmh: f64,
    /// Jam density in veh/km — upper bound for the density colour scale.
    pub kx_veh_km: f64,
    /// Capacity in veh/h — peak of the theoretical FD.
    pub capacity_veh_h: f64,
}

impl XTDiagram {
    /// Return the spatial speed profile `[nx]` at the temporal bin that
    /// contains absolute time `t_abs`.  Returns `None` if `t_abs` is outside
    /// the simulation window.
    pub fn speed_at_time(&self, t_abs: f64) -> Option<Vec<f64>> {
        let ti = time_bin_index(&self.t_bins, t_abs)?;
        Some(self.speed.iter().map(|row| row[ti]).collect())
    }

    /// Same as `speed_at_time` but returns normalised V/u ∈ [0, 1].
    /// Values > 1 are clamped to 1.
    pub fn normalised_speed_at_time(&self, t_abs: f64) -> Option<Vec<f64>> {
        let ti = time_bin_index(&self.t_bins, t_abs)?;
        Some(
            self.speed
                .iter()
                .map(|row| (row[ti] / self.u_kmh).min(1.0).max(0.0))
                .collect(),
        )
    }

    /// Return the spatial density profile `[nx]` in veh/km at time `t_abs`.
    pub fn density_at_time(&self, t_abs: f64) -> Option<Vec<f64>> {
        let ti = time_bin_index(&self.t_bins, t_abs)?;
        Some(self.density.iter().map(|row| row[ti]).collect())
    }
}

// ── Public entry point ────────────────────────────────────────────────────────

/// Compute XT diagrams for **all links in parallel** (Rayon).
///
/// Input: a completed `Scenario` whose `vehicle.node_times` are fully populated
/// (i.e. the simulation has been run).
pub fn compute_xt_diagrams(scenario: &Scenario, params: &XTParams) -> HashMap<LinkID, XTDiagram> {
    scenario
        .links
        .par_iter()
        .map(|link| {
            let diag = compute_xt_link(scenario, link.id, params);
            (link.id, diag)
        })
        .collect()
}

// ── Per-link computation ──────────────────────────────────────────────────────

/// Compute the XT diagram for a single link.  Exposed as `pub` so callers can
/// analyse individual links without computing the whole network.
pub fn compute_xt_link(scenario: &Scenario, link_id: LinkID, params: &XTParams) -> XTDiagram {
    let link = &scenario.links[link_id];
    // Whole-link jam density: fd.kx is per lane, but CVCs count vehicles over
    // the entire link, so Newell's shift kx·(L−x) needs kx × lanes.
    let kx = link.fd.kx * f64::from(link.num_lanes); // veh/m (all lanes)
    let (cvc_in, cvc_out) = collect_link_cvcs(scenario, link_id, None);
    compute_xt_core(
        scenario,
        link_id,
        None,
        cvc_in,
        cvc_out,
        kx,
        link.capacity,
        params,
    )
}

/// Compute the XT diagram for one pipe of a link: CVCs are restricted to
/// vehicles that took that pipe, and the Newell shift / density cap use the
/// pipe's lane count.
pub fn compute_xt_link_pipe(
    scenario: &Scenario,
    link_id: LinkID,
    pipe: PipeIdx,
    params: &XTParams,
) -> XTDiagram {
    let link = &scenario.links[link_id];
    let (lanes, capacity) = link
        .pipes
        .get(usize::from(pipe))
        .map(|p| (p.lanes, p.capacity))
        .unwrap_or((link.num_lanes, link.capacity));
    let kx = link.fd.kx * f64::from(lanes);
    let (cvc_in, cvc_out) = collect_link_cvcs(scenario, link_id, Some(pipe));
    compute_xt_core(
        scenario,
        link_id,
        Some(pipe),
        cvc_in,
        cvc_out,
        kx,
        capacity,
        params,
    )
}

/// Per-pipe XT diagrams for all links (Rayon-parallel over links).
pub fn compute_xt_pipe_diagrams(
    scenario: &Scenario,
    params: &XTParams,
) -> HashMap<LinkID, Vec<XTDiagram>> {
    scenario
        .links
        .par_iter()
        .map(|link| {
            let n_pipes = link.pipes.len().max(1);
            let diags = (0..n_pipes)
                .map(|p| compute_xt_link_pipe(scenario, link.id, p as PipeIdx, params))
                .collect();
            (link.id, diags)
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn compute_xt_core(
    scenario: &Scenario,
    link_id: LinkID,
    pipe: Option<PipeIdx>,
    cvc_in: Vec<f64>,
    cvc_out: Vec<f64>,
    kx: f64,
    capacity: f64,
    params: &XTParams,
) -> XTDiagram {
    let link = &scenario.links[link_id];
    let len = link.length; // m
    let u = link.fd.u; // m/s
    let w = link.fd.w; // m/s

    let x_edges = build_bins(0.0, len, params.dx_m);
    let t_edges = build_bins(0.0, scenario.duration, params.dt_s);

    let nx = x_edges.len().saturating_sub(1);
    let nt = t_edges.len().saturating_sub(1);

    let x_bins: Vec<f64> = x_edges[..nx].to_vec();
    let t_bins: Vec<f64> = t_edges[..nt]
        .iter()
        .map(|&t| t + scenario.start_time)
        .collect();

    let default_speed = u * 3.6;

    if cvc_in.is_empty() || nx == 0 || nt == 0 {
        return XTDiagram {
            link_id,
            pipe,
            x_bins,
            t_bins,
            flow: vec![vec![0.0; nt]; nx],
            density: vec![vec![0.0; nt]; nx],
            speed: vec![vec![default_speed; nt]; nx],
            u_kmh: default_speed,
            kx_veh_km: kx * 1000.0,
            capacity_veh_h: capacity * 3600.0,
        };
    }

    // ── Step 1: build the CVC surface at every x-grid edge ────────────────
    // cvcs[xi] = sorted passage times of all N vehicles at position x_edges[xi]
    let cvcs: Vec<Vec<f64>> = x_edges
        .iter()
        .map(|&x| cvc_at_x(&cvc_in, &cvc_out, x, len, u, w, kx))
        .collect();

    // ── Step 2: Edie cells ────────────────────────────────────────────────
    let cell_area = params.dx_m * params.dt_s; // m·s

    let mut flow = vec![vec![0.0f64; nt]; nx];
    let mut density = vec![vec![0.0f64; nt]; nx];
    let mut speed = vec![vec![default_speed; nt]; nx];

    for xi in 0..nx {
        let cvc_up = &cvcs[xi];
        let cvc_down = &cvcs[xi + 1];

        for ti in 0..nt {
            let (dt_sum, pt_sum) =
                edie_cell(cvc_up, cvc_down, t_edges[ti], t_edges[ti + 1], params.dx_m);

            if pt_sum < 1e-12 {
                continue; // no vehicles → leave default free-flow speed
            }

            let q = dt_sum / cell_area; // veh/s
            let k = pt_sum / cell_area; // veh/m

            flow[xi][ti] = q * 3600.0;
            density[xi][ti] = k * 1000.0;
            speed[xi][ti] = if k > 1e-9 {
                (q / k) * 3.6
            } else {
                default_speed
            };
        }
    }

    XTDiagram {
        link_id,
        pipe,
        x_bins,
        t_bins,
        flow,
        density,
        speed,
        u_kmh: default_speed,
        kx_veh_km: kx * 1000.0,
        capacity_veh_h: capacity * 3600.0,
    }
}

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Build bin-edge values `[start, start+step, …, end]`.
/// The last value is always exactly `end`.
fn build_bins(start: f64, end: f64, step: f64) -> Vec<f64> {
    let capacity = ((end - start) / step).ceil() as usize + 2;
    let mut bins = Vec::with_capacity(capacity);
    let mut x = start;
    while x < end - 1e-9 {
        bins.push(x);
        x += step;
    }
    bins.push(end);
    bins
}

/// Find the time-bin index for absolute time `t_abs` in a sorted `t_bins` array
/// (left edges).  Returns `None` if `t_abs` is before the first bin or at/after
/// the last bin edge (i.e. outside the simulation window).
fn time_bin_index(t_bins: &[f64], t_abs: f64) -> Option<usize> {
    if t_bins.is_empty() || t_abs < t_bins[0] {
        return None;
    }
    let ti = t_bins.partition_point(|&t| t <= t_abs).saturating_sub(1);
    if ti >= t_bins.len() {
        None
    } else {
        Some(ti)
    }
}

/// Collect sorted entry and exit times (relative to `scenario.start_time`)
/// for all vehicles that traverse `link_id`.
///
/// Both vectors are sorted independently — this is the standard FIFO assumption
/// used in Newell's CVC theory: the n-th vehicle to enter is the n-th to exit.
/// Vehicles whose traversal is incomplete (no recorded exit time) are excluded.
fn collect_link_cvcs(
    scenario: &Scenario,
    link_id: LinkID,
    pipe: Option<PipeIdx>,
) -> (Vec<f64>, Vec<f64>) {
    let t0 = scenario.start_time;
    let mut entries = Vec::new();
    let mut exits = Vec::new();

    for veh in &scenario.vehicles {
        for (idx, &lid) in veh.path.iter().enumerate() {
            let pipe_ok = pipe.is_none_or(|p| veh.pipes_taken.get(idx).copied().unwrap_or(0) == p);
            if lid == link_id && pipe_ok && idx + 1 < veh.node_times.len() {
                entries.push(veh.node_times[idx] - t0);
                exits.push(veh.node_times[idx + 1] - t0);
            }
        }
    }

    // Sort each array independently (FIFO: ordering of entry = ordering of exit)
    entries.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    exits.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    // Keep only the vehicles whose traversal is represented in both arrays
    let n = entries.len().min(exits.len());
    entries.truncate(n);
    exits.truncate(n);

    (entries, exits)
}

/// Reconstruct the CVC at interior position `x` using Newell's demand-supply
/// max principle.
///
/// For each vehicle n (0-indexed, sorted by passage time at the link entry):
///
///   t_demand[n] = cvc_in[n]  + x / u           (free-flow wave, forward)
///   t_supply[n] = cvc_out[k] + (L−x) / w       (congested wave, backward)
///     where k = interpolated index max(n+1 − kx·(L−x), 0)
///     in the extended CVCout = [−∞, cvc_out[0..N−1], +∞]
///
///   cvc[x, n] = max(t_demand[n], t_supply[n])
///
/// Special cases at the boundaries avoid floating-point drift:
///   x ≈ 0   → return cvc_in  (entry boundary)
///   x ≈ L   → return cvc_out (exit boundary)
fn cvc_at_x(
    cvc_in: &[f64],
    cvc_out: &[f64],
    x: f64,
    link_len: f64,
    u_ms: f64,
    w_ms: f64,
    kx_vehm: f64,
) -> Vec<f64> {
    debug_assert_eq!(cvc_in.len(), cvc_out.len());

    // Boundary short-circuits
    if x < 1e-9 {
        return cvc_in.to_vec();
    }
    if (x - link_len).abs() < 1e-9 {
        return cvc_out.to_vec();
    }

    let n = cvc_in.len();
    let shift = kx_vehm * (link_len - x); // vehicles fitting between x and L at jam density
    let t_back = (link_len - x) / w_ms; // backward travel time from L to x

    (0..n)
        .map(|i| {
            let t_demand = cvc_in[i] + x / u_ms;

            // Extended-array lookup index (clamped to [0, n]):
            //   index 0     → −∞ sentinel  (no supply constraint)
            //   index 1..n  → cvc_out[0..n−1]
            let ext_idx = ((i as f64 + 1.0 - shift).max(0.0)).min(n as f64);

            let t_supply = if ext_idx < 1.0 {
                // Below the first real cvc_out entry → free-flow dominates
                f64::NEG_INFINITY
            } else {
                let lo = ext_idx.floor() as usize; // in [1, n]
                let frac = ext_idx - lo as f64;
                let t_lo = cvc_out[lo - 1];
                // Upper neighbour: clamp to last element (frac is 0 when lo==n)
                let t_hi = if lo < n { cvc_out[lo] } else { t_lo };
                t_lo + frac * (t_hi - t_lo)
            };

            t_demand.max(t_supply + t_back)
        })
        .collect()
}

/// Compute Edie's (distance_travelled_m, time_spent_s) for one cell
/// `[x_i, x_i+dx] × [t_j, t_j+dt]`.
///
/// Both CVC arrays are monotonically increasing (sorted), so the set of
/// vehicles present in the cell is the **contiguous slice** `[n2, n1)`:
///   n1 = last vehicle whose upstream passage time < t_end   (binary search)
///   n2 = first vehicle whose downstream passage time > t_start (binary search)
///
/// Each vehicle's contribution is clipped to the cell's time window to handle
/// trajectories that partially overlap a cell boundary.
fn edie_cell(
    cvc_up: &[f64],   // sorted passage times at the upstream edge   x_i
    cvc_down: &[f64], // sorted passage times at the downstream edge x_i+dx
    t_start: f64,
    t_end: f64,
    dx_m: f64,
) -> (f64, f64) {
    debug_assert_eq!(cvc_up.len(), cvc_down.len());

    // Vehicles present in cell: those that entered upstream before t_end
    // AND exited downstream after t_start.
    let n1 = cvc_up.partition_point(|&t| t < t_end); // (0, n1)  entered upstream
    let n2 = cvc_down.partition_point(|&t| t <= t_start); // [n2, N)  exited downstream

    if n2 >= n1 {
        return (0.0, 0.0);
    }

    let mut dt = 0.0f64; // total distance travelled (m)
    let mut pt = 0.0f64; // total time spent         (s)

    for idx in n2..n1 {
        let t_in = cvc_up[idx];
        let t_out = cvc_down[idx];

        let travel = t_out - t_in;
        if travel <= 0.0 {
            continue;
        }

        let speed_ms = dx_m / travel; // m/s within this spatial cell

        // Clip to the cell's time window
        let clip_start = t_in.max(t_start);
        let clip_end = t_out.min(t_end);

        if clip_end > clip_start {
            let presence = clip_end - clip_start;
            pt += presence;
            dt += speed_ms * presence;
        }
    }

    (dt, pt)
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::*;

    fn fd() -> FundamentalDiagram {
        FundamentalDiagram {
            u: 25.0,
            w: 5.0,
            kx: 0.12,
            c: 0.5,
        }
    }

    // ── build_bins ────────────────────────────────────────────────────────

    #[test]
    fn build_bins_ends_at_length() {
        let bins = build_bins(0.0, 840.0, 50.0);
        assert_eq!(*bins.last().unwrap(), 840.0);
        assert!(bins.len() >= 2);
    }

    #[test]
    fn build_bins_exact_multiple() {
        let bins = build_bins(0.0, 300.0, 100.0);
        assert_eq!(bins, vec![0.0, 100.0, 200.0, 300.0]);
    }

    // ── cvc_at_x ─────────────────────────────────────────────────────────

    #[test]
    fn cvc_at_zero_returns_cvc_in() {
        let cvc_in = vec![0.0, 2.0, 4.0];
        let cvc_out = vec![4.0, 6.0, 8.0]; // 4 s free-flow on a 100 m link at 25 m/s
        let result = cvc_at_x(&cvc_in, &cvc_out, 0.0, 100.0, 25.0, 5.0, 0.12);
        assert_eq!(result, cvc_in);
    }

    #[test]
    fn cvc_at_link_length_returns_cvc_out() {
        let cvc_in = vec![0.0, 2.0, 4.0];
        let cvc_out = vec![4.0, 6.0, 8.0];
        let result = cvc_at_x(&cvc_in, &cvc_out, 100.0, 100.0, 25.0, 5.0, 0.12);
        assert_eq!(result, cvc_out);
    }

    #[test]
    fn cvc_midpoint_free_flow_no_congestion() {
        // 3 vehicles on a 100 m link at u=25 m/s (transit = 4 s), kx=0.12 veh/m.
        // Jam capacity = 0.12 × 100 m = 12 vehicles; only 3 present → free-flow.
        // At x=50 m the shift = kx × (L-x) = 0.12 × 50 = 6 vehicles, which is
        // larger than all vehicle indices (0,1,2) → ext_idx = 0 → supply = -∞
        // → demand dominates for all three vehicles.
        let cvc_in = vec![0.0, 10.0, 20.0];
        let cvc_out = vec![4.0, 14.0, 24.0];
        let result = cvc_at_x(&cvc_in, &cvc_out, 50.0, 100.0, 25.0, 5.0, 0.12);
        for (i, &t) in result.iter().enumerate() {
            let expected = cvc_in[i] + 50.0 / 25.0; // t + 2 s
            assert!(
                (t - expected).abs() < 1e-9,
                "vehicle {i}: got {t:.4}, expected {expected:.4}"
            );
        }
    }

    #[test]
    fn cvc_is_monotone_increasing() {
        // The CVC must be sorted (FIFO property preserved by Newell's formula).
        let cvc_in = vec![0.0, 1.44, 2.88, 4.32, 5.76];
        let cvc_out = vec![2.0, 3.44, 4.88, 6.32, 7.76];
        let result = cvc_at_x(&cvc_in, &cvc_out, 500.0, 1000.0, 25.0, 5.0, 0.12);
        for w in result.windows(2) {
            assert!(w[1] >= w[0], "CVC not monotone: {:?}", result);
        }
    }

    // ── edie_cell ─────────────────────────────────────────────────────────

    #[test]
    fn edie_single_vehicle_full_cell() {
        // One vehicle, 100 m cell in 40 s → speed = 2.5 m/s.
        // Cell time window [0, 60); vehicle is fully inside.
        let cvc_up = vec![10.0]; // enters upstream at t=10
        let cvc_down = vec![50.0]; // exits downstream at t=50  (40 s, 2.5 m/s)
        let (dt, pt) = edie_cell(&cvc_up, &cvc_down, 0.0, 60.0, 100.0);
        assert!((dt - 100.0).abs() < 1e-9); // 2.5 × 40 = 100 m
        assert!((pt - 40.0).abs() < 1e-9); // 40 s
    }

    #[test]
    fn edie_single_vehicle_partial_entry() {
        // Vehicle enters upstream at t=30, exits downstream at t=70.
        // Cell time [0, 60): clipped presence = [30, 60) = 30 s.
        let cvc_up = vec![30.0];
        let cvc_down = vec![70.0];
        let (dt, pt) = edie_cell(&cvc_up, &cvc_down, 0.0, 60.0, 100.0);
        let speed = 100.0 / 40.0; // 2.5 m/s
        assert!((pt - 30.0).abs() < 1e-9);
        assert!((dt - speed * 30.0).abs() < 1e-6);
    }

    #[test]
    fn edie_no_vehicles_in_cell() {
        let cvc_up = vec![100.0, 200.0];
        let cvc_down = vec![104.0, 204.0];
        // Cell is [0, 60) — all vehicles are after the window.
        let (dt, pt) = edie_cell(&cvc_up, &cvc_down, 0.0, 60.0, 100.0);
        assert_eq!(dt, 0.0);
        assert_eq!(pt, 0.0);
    }

    #[test]
    fn edie_flow_density_speed_consistent() {
        // 5 vehicles crossing 100 m in 4 s each, at 1-s headway.
        // Steady-state in cell [0, 60): q = 3600 veh/h (one/s), k ≈ q/u.
        let cvc_up: Vec<f64> = (0..5).map(|i| i as f64 * 1.0).collect(); // 0,1,2,3,4
        let cvc_down: Vec<f64> = (0..5).map(|i| i as f64 * 1.0 + 4.0).collect(); // 4,5,6,7,8
        let dx = 100.0;
        let cell_area = dx * 60.0; // m·s
        let (dt, pt) = edie_cell(&cvc_up, &cvc_down, 0.0, 60.0, dx);
        let q = dt / cell_area;
        let k = pt / cell_area;
        let v = q / k;
        assert!(
            (v - 25.0).abs() < 0.1,
            "speed should be ~25 m/s, got {v:.2}"
        );
    }

    // ── compute_xt_link ───────────────────────────────────────────────────

    fn minimal_scenario_for_xt() -> Scenario {
        // Entry(0) → [Link 0, 1000 m] → Internal(1) → [Link 1, 1000 m] → Exit(2)
        let nodes = vec![
            Node {
                id: 0,
                node_type: NodeType::Entry,
                incoming_links: vec![],
                outgoing_links: vec![0],
                points: (0.0, 0.0),
                signals: vec![],
            },
            Node {
                id: 1,
                node_type: NodeType::Internal,
                incoming_links: vec![0],
                outgoing_links: vec![1],
                points: (1.0, 0.0),
                signals: vec![],
            },
            Node {
                id: 2,
                node_type: NodeType::Exit,
                incoming_links: vec![1],
                outgoing_links: vec![],
                points: (2.0, 0.0),
                signals: vec![],
            },
        ];
        let links = vec![
            Link::new(0, 0, 1, 1000.0, 25.0, 1, 0.5, fd(), vec![]),
            Link::new(1, 1, 2, 1000.0, 25.0, 1, 0.5, fd(), vec![]),
        ];
        // One vehicle: enters link 0 at t=0, exits link 0 / enters link 1 at t=40,
        // exits link 1 at t=80.
        let vehicles = vec![Vehicle {
            id: 0,
            class_id: 0,
            path: vec![0, 1],
            start_time: 0.0,
            origin: 0,
            destination: 2,
            state: VehicleState::Exited,
            current_link_idx: 1,
            node_times: vec![0.0, 40.0, 80.0],
            pipes_taken: vec![0, 0],
        }];
        Scenario {
            nodes,
            links,
            vehicles,
            demand: vec![],
            start_time: 0.0,
            duration: 120.0,
            classes: vec!["car".to_string()],
            link_schedule: Vec::new(),
            assignment_excluded: Vec::new(),
        }
    }

    #[test]
    fn compute_xt_link_single_vehicle_free_flow() {
        let scenario = minimal_scenario_for_xt();
        let params = XTParams {
            dx_m: 500.0,
            dt_s: 60.0,
        };
        let diag = compute_xt_link(&scenario, 0, &params);

        assert_eq!(diag.x_bins.len(), 2); // 0 m, 500 m
        assert_eq!(diag.t_bins.len(), 2); // 0 s, 60 s

        // Speed in bin (0, 0): vehicle traverses at 25 m/s
        let spd = diag.speed[0][0];
        assert!(
            (spd - 25.0 * 3.6).abs() < 1.0,
            "expected ~90 km/h, got {spd:.1}"
        );
    }

    #[test]
    fn compute_xt_link_empty_link_returns_free_flow_speed() {
        let scenario = minimal_scenario_for_xt();
        let params = XTParams {
            dx_m: 500.0,
            dt_s: 60.0,
        };
        // Link 1 has one vehicle, but analyse a hypothetical link 99 (empty).
        // Use link 0 but with a fresh scenario with no vehicles.
        let empty = Scenario {
            vehicles: vec![],
            ..scenario
        };
        let diag = compute_xt_link(&empty, 0, &params);
        for row in &diag.speed {
            for &s in row {
                assert!((s - 25.0 * 3.6).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn speed_at_time_returns_correct_bin() {
        let scenario = minimal_scenario_for_xt();
        let params = XTParams {
            dx_m: 500.0,
            dt_s: 60.0,
        };
        let diag = compute_xt_link(&scenario, 0, &params);
        // t_abs = 30 s → falls in bin 0 (absolute: [0, 60))
        let profile = diag.speed_at_time(30.0).unwrap();
        assert_eq!(profile.len(), diag.x_bins.len());
    }
}
