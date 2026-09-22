//! Reference-data comparison — simulated vs observed measurements.
//!
//! Observations are point measurements aggregated over a time window on one
//! link (what a loop detector or a floating-car dataset provides):
//!
//! ```json
//! [
//!   { "link_id": 3, "t_start": 25200, "t_end": 28800,
//!     "flow_veh_h": 1450, "speed_kmh": 78, "label": "D3 morning peak" }
//! ]
//! ```
//!
//! ## Methodology
//!
//! - **Flow**: simulated flow over the window = number of vehicles exiting the
//!   link in [t_start, t_end) × 3600 / window. Compared with the **GEH
//!   statistic** — the standard empirical measure for traffic model
//!   calibration: GEH = √(2(m−o)² / (m+o)) with m, o in veh/h.
//!   Convention (UK DMRB / FHWA): GEH < 5 acceptable, 5–10 warrants caution,
//!   > 10 unacceptable.
//! - **Speed**: simulated space-mean speed over the window = L / mean travel
//!   time of vehicles entering the link within the window, in km/h. Compared
//!   by relative error (< 10% pass, < 25% warn).

use crate::model::{LinkID, Scenario};
use crate::verification::CheckStatus;
use serde::{Deserialize, Serialize};

const GEH_PASS: f64 = 5.0;
const GEH_WARN: f64 = 10.0;
const SPEED_PASS_PCT: f64 = 10.0;
const SPEED_WARN_PCT: f64 = 25.0;

/// One observed measurement on a link over a time window.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObservedRecord {
    pub link_id: LinkID,
    /// Window start / end in absolute simulation seconds.
    pub t_start: f64,
    pub t_end: f64,
    pub flow_veh_h: Option<f64>,
    pub speed_kmh: Option<f64>,
    pub label: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReferenceKind {
    Flow,
    Speed,
}

/// One simulated-vs-observed comparison.
#[derive(Debug, Serialize, Deserialize)]
pub struct ReferenceCheck {
    pub link_id: LinkID,
    pub label: Option<String>,
    pub t_start: f64,
    pub t_end: f64,
    pub kind: ReferenceKind,
    pub observed: f64,
    pub simulated: f64,
    /// GEH statistic (flow comparisons only).
    pub geh: Option<f64>,
    pub error_pct: f64,
    pub status: CheckStatus,
    pub detail: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ReferenceReport {
    pub checks: Vec<ReferenceCheck>,
    pub n_pass: usize,
    pub n_warn: usize,
    pub n_fail: usize,
}

fn geh(m: f64, o: f64) -> f64 {
    if m + o <= 0.0 {
        return 0.0;
    }
    (2.0 * (m - o).powi(2) / (m + o)).sqrt()
}

/// Simulated flow (veh/h) and space-mean speed (km/h) on `link_id` over the window.
fn simulate_window(
    scenario: &Scenario,
    link_id: LinkID,
    t_start: f64,
    t_end: f64,
) -> (Option<f64>, Option<f64>) {
    let window = t_end - t_start;
    if window <= 0.0 {
        return (None, None);
    }
    let Some(link) = scenario.links.get(link_id) else {
        return (None, None);
    };

    let mut exits = 0usize;
    let mut travel_times = Vec::new();
    for veh in &scenario.vehicles {
        for (i, &lid) in veh.path.iter().enumerate() {
            if lid != link_id {
                continue;
            }
            if let (Some(&t_in), Some(&t_out)) = (veh.node_times.get(i), veh.node_times.get(i + 1))
            {
                if t_out >= t_start && t_out < t_end {
                    exits += 1;
                }
                if t_in >= t_start && t_in < t_end {
                    travel_times.push(t_out - t_in);
                }
            }
        }
    }

    let flow = Some(exits as f64 * 3600.0 / window);
    let speed = if travel_times.is_empty() {
        None
    } else {
        let mean_tt = travel_times.iter().sum::<f64>() / travel_times.len() as f64;
        if mean_tt > 0.0 {
            Some(link.length / mean_tt * 3.6)
        } else {
            None
        }
    };
    (flow, speed)
}

/// Compare a completed simulation with observed records.
pub fn compare_with_observations(
    scenario: &Scenario,
    records: &[ObservedRecord],
) -> ReferenceReport {
    let mut checks = Vec::new();

    for rec in records {
        let (sim_flow, sim_speed) = simulate_window(scenario, rec.link_id, rec.t_start, rec.t_end);

        if let (Some(obs), Some(sim)) = (rec.flow_veh_h, sim_flow) {
            let g = geh(sim, obs);
            let status = if g < GEH_PASS {
                CheckStatus::Pass
            } else if g < GEH_WARN {
                CheckStatus::Warn
            } else {
                CheckStatus::Fail
            };
            checks.push(ReferenceCheck {
                link_id: rec.link_id,
                label: rec.label.clone(),
                t_start: rec.t_start,
                t_end: rec.t_end,
                kind: ReferenceKind::Flow,
                observed: obs,
                simulated: sim,
                geh: Some(g),
                error_pct: if obs > 0.0 {
                    ((sim - obs) / obs).abs() * 100.0
                } else {
                    0.0
                },
                status,
                detail: format!(
                    "GEH = {:.2} (DMRB: <5 acceptable, 5–10 caution, >10 unacceptable). \
                     Simulated = link exits in window × 3600 / window.",
                    g
                ),
            });
        }

        if let (Some(obs), Some(sim)) = (rec.speed_kmh, sim_speed) {
            let err = if obs > 0.0 {
                ((sim - obs) / obs).abs() * 100.0
            } else {
                0.0
            };
            let status = if err < SPEED_PASS_PCT {
                CheckStatus::Pass
            } else if err < SPEED_WARN_PCT {
                CheckStatus::Warn
            } else {
                CheckStatus::Fail
            };
            checks.push(ReferenceCheck {
                link_id: rec.link_id,
                label: rec.label.clone(),
                t_start: rec.t_start,
                t_end: rec.t_end,
                kind: ReferenceKind::Speed,
                observed: obs,
                simulated: sim,
                geh: None,
                error_pct: err,
                status,
                detail: "Space-mean speed = L / mean travel time of vehicles entering the link in the window.".to_string(),
            });
        }
    }

    let count = |s: CheckStatus| checks.iter().filter(|c| c.status == s).count();
    ReferenceReport {
        n_pass: count(CheckStatus::Pass),
        n_warn: count(CheckStatus::Warn),
        n_fail: count(CheckStatus::Fail),
        checks,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{FundamentalDiagram, Link, Node, NodeType, Vehicle};
    use crate::simulation::Simulation;

    fn free_flow_scenario(n: usize, spacing: f64) -> Scenario {
        let fd = FundamentalDiagram {
            u: 10.0,
            w: 5.0,
            kx: 10.0,
            c: 10.0,
        };
        let vehicles = (0..n)
            .map(|i| Vehicle::new(i, 0, vec![0], i as f64 * spacing, 0, 1))
            .collect();
        Scenario {
            nodes: vec![
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
                    node_type: NodeType::Exit,
                    incoming_links: vec![0],
                    outgoing_links: vec![],
                    points: (1.0, 0.0),
                    signals: vec![],
                },
            ],
            links: vec![Link::new(0, 0, 1, 100.0, 10.0, 1, 10.0, fd, vec![])],
            vehicles,
            demand: vec![],
            start_time: 0.0,
            duration: 100_000.0,
            classes: vec!["car".to_string()],
            link_schedule: Vec::new(),
            assignment_excluded: Vec::new(),
        }
    }

    #[test]
    fn matching_observation_passes_geh_and_speed() {
        // 36 vehicles spaced 10 s → flow 360 veh/h; free flow → 36 km/h.
        let mut sim = Simulation::new(free_flow_scenario(36, 10.0));
        sim.run().unwrap();
        let records = vec![ObservedRecord {
            link_id: 0,
            t_start: 0.0,
            t_end: 360.0,
            flow_veh_h: Some(360.0),
            speed_kmh: Some(36.0),
            label: Some("synthetic detector".into()),
        }];
        let report = compare_with_observations(&sim.scenario, &records);
        assert_eq!(report.checks.len(), 2);
        assert_eq!(report.n_fail, 0, "{:#?}", report.checks);
        assert_eq!(report.n_pass, 2, "{:#?}", report.checks);
    }

    #[test]
    fn wrong_observation_fails() {
        let mut sim = Simulation::new(free_flow_scenario(36, 10.0));
        sim.run().unwrap();
        let records = vec![ObservedRecord {
            link_id: 0,
            t_start: 0.0,
            t_end: 360.0,
            flow_veh_h: Some(1800.0), // observed 5× the simulated flow
            speed_kmh: None,
            label: None,
        }];
        let report = compare_with_observations(&sim.scenario, &records);
        assert_eq!(report.n_fail, 1, "{:#?}", report.checks);
    }
}
