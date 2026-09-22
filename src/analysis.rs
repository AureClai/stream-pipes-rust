use crate::model::{LinkID, PipeIdx, Scenario};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Serialize, Deserialize, Debug)]
pub struct TimeSeries {
    pub t: Vec<f64>,
    pub values: Vec<f64>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct LinkStats {
    pub link_id: LinkID,
    pub flow: TimeSeries,    // veh/h  — vehicles exiting the link per hour
    pub density: TimeSeries, // veh/km — vehicles present on the link / link_length_km; bounded by kx
    pub speed: TimeSeries,   // km/h   — space-mean speed = flow / density
    /// Jam density in veh/km (= fd.kx × 1000). Use as display cap on the density axis.
    #[serde(default)]
    pub kx_veh_km: f64,
    /// Link capacity in veh/h (= fd.c × 3600).
    #[serde(default)]
    pub capacity_veh_h: f64,
}

/// Per-pipe statistics: same series as `LinkStats`, one entry per pipe of a
/// multi-pipe link. Uses `Vehicle::pipes_taken` to attribute each traversal
/// to the pipe it actually used.
#[derive(Serialize, Deserialize, Debug)]
pub struct PipeStats {
    pub link_id: LinkID,
    pub pipe: PipeIdx,
    pub lanes: u8,
    pub flow: TimeSeries,    // veh/h
    pub density: TimeSeries, // veh/km (per pipe: vehicles / link length)
    pub speed: TimeSeries,   // km/h
    pub kx_veh_km: f64,      // fd.kx × pipe lanes × 1000
    pub capacity_veh_h: f64, // pipe capacity × 3600
}

/// Compute per-pipe time series for every link (single-pipe links produce one
/// entry). Requires a completed simulation (pipes materialized, `pipes_taken`
/// populated).
pub fn compute_pipe_stats(scenario: &Scenario, bin_size: f64) -> HashMap<LinkID, Vec<PipeStats>> {
    let mut stats: HashMap<LinkID, Vec<PipeStats>> = HashMap::new();
    let n_bins = (scenario.duration / bin_size).ceil() as usize;

    for link in &scenario.links {
        let n_pipes = link.pipes.len().max(1);
        let entries = (0..n_pipes)
            .map(|p| {
                let (lanes, capacity) = link
                    .pipes
                    .get(p)
                    .map(|pipe| (pipe.lanes, pipe.capacity))
                    .unwrap_or((link.num_lanes, link.capacity));
                let mut t_axis = Vec::with_capacity(n_bins);
                for i in 0..n_bins {
                    t_axis.push(scenario.start_time + i as f64 * bin_size);
                }
                PipeStats {
                    link_id: link.id,
                    pipe: p as PipeIdx,
                    lanes,
                    flow: TimeSeries {
                        t: t_axis.clone(),
                        values: vec![0.0; n_bins],
                    },
                    density: TimeSeries {
                        t: t_axis.clone(),
                        values: vec![0.0; n_bins],
                    },
                    speed: TimeSeries {
                        t: t_axis,
                        values: vec![0.0; n_bins],
                    },
                    kx_veh_km: link.fd.kx * f64::from(lanes) * 1000.0,
                    capacity_veh_h: capacity * 3600.0,
                }
            })
            .collect();
        stats.insert(link.id, entries);
    }

    // Accumulate per (link, pipe) using the same Edie logic as compute_link_stats.
    for veh in &scenario.vehicles {
        debug_assert!(
            veh.pipes_taken.len() >= veh.node_times.len().saturating_sub(1),
            "vehicle {}: pipes_taken shorter than traversed prefix",
            veh.id
        );
        for (i, &link_id) in veh.path.iter().enumerate() {
            if i + 1 >= veh.node_times.len() {
                break;
            }
            let pipe = usize::from(veh.pipes_taken.get(i).copied().unwrap_or(0));

            let t_entry = veh.node_times[i] - scenario.start_time;
            let t_exit = veh.node_times[i + 1] - scenario.start_time;
            if t_exit <= t_entry {
                continue;
            }

            let Some(stat) = stats.get_mut(&link_id).and_then(|v| v.get_mut(pipe)) else {
                continue;
            };
            let exit_bin = (t_exit / bin_size).floor() as usize;
            if exit_bin < n_bins {
                stat.flow.values[exit_bin] += 1.0;
            }
            let start_bin = (t_entry / bin_size).floor() as usize;
            let end_bin = (t_exit / bin_size).floor() as usize;
            for bin in start_bin..=end_bin {
                if bin >= n_bins {
                    break;
                }
                let bin_start = bin as f64 * bin_size;
                let bin_end = (bin + 1) as f64 * bin_size;
                let t0 = f64::max(t_entry, bin_start);
                let t1 = f64::min(t_exit, bin_end);
                if t1 > t0 {
                    stat.density.values[bin] += (t1 - t0) / 3600.0;
                }
            }
        }
    }

    // Normalize
    for (lid, pipes) in stats.iter_mut() {
        let (link_len_km, u_kmh) = scenario
            .links
            .iter()
            .find(|l| l.id == *lid)
            .map(|l| (l.length / 1000.0, l.fd.u * 3.6))
            .unwrap_or((1.0, 0.0));
        for stat in pipes {
            for i in 0..n_bins {
                stat.flow.values[i] /= bin_size / 3600.0;
                stat.density.values[i] /= (bin_size / 3600.0) * link_len_km;
                stat.speed.values[i] = if stat.density.values[i] > 0.001 {
                    stat.flow.values[i] / stat.density.values[i]
                } else {
                    u_kmh
                };
            }
        }
    }

    stats
}

pub fn compute_link_stats(scenario: &Scenario, bin_size: f64) -> HashMap<LinkID, LinkStats> {
    let mut stats = HashMap::new();
    let duration = scenario.duration;
    let n_bins = (duration / bin_size).ceil() as usize;

    // Initialize stats for all links
    for link in &scenario.links {
        stats.insert(
            link.id,
            LinkStats {
                link_id: link.id,
                flow: TimeSeries {
                    t: Vec::with_capacity(n_bins),
                    values: vec![0.0; n_bins],
                },
                density: TimeSeries {
                    t: Vec::with_capacity(n_bins),
                    values: vec![0.0; n_bins],
                },
                speed: TimeSeries {
                    t: Vec::with_capacity(n_bins),
                    values: vec![0.0; n_bins],
                },
                // Whole-link values: fd.kx is per lane, observed density spans all lanes.
                kx_veh_km: link.fd.kx * f64::from(link.num_lanes) * 1000.0, // veh/m → veh/km
                capacity_veh_h: link.capacity * 3600.0,                     // veh/s → veh/h
            },
        );
    }

    // Time axis: relative to simulation start so bin 0 = t_0, bin 1 = t_0 + bin_size, etc.
    for i in 0..n_bins {
        let t = scenario.start_time + i as f64 * bin_size;
        for s in stats.values_mut() {
            s.flow.t.push(t);
            s.density.t.push(t);
            s.speed.t.push(t);
        }
    }

    // Process vehicles
    for veh in &scenario.vehicles {
        if veh.node_times.is_empty() {
            continue;
        }

        // veh.node_times corresponds to nodes in veh.path (origins of links + dest of last link)
        // Link i in path connects path[i] -> path[i+1] (nodes, but path stores LinkIDs??)
        // Wait, Vehicle struct says `path: Vec<LinkID>`.
        // And `node_times` stores time at nodes?
        // Let's check `simulation.rs` how `node_times` are recorded.
        // Usually node_times[i] is entry time to link path[i].
        // And node_times[len] is exit time of last link.

        // Correct logic:
        // path has L links.
        // node_times has L+1 times (Entry to L0, Entry to L1... Exit of L_last).

        for (i, &link_id) in veh.path.iter().enumerate() {
            if i + 1 >= veh.node_times.len() {
                break;
            }

            // Use relative time so bin 0 always corresponds to simulation start,
            // regardless of the absolute start_time value.
            let t_entry = veh.node_times[i] - scenario.start_time;
            let t_exit = veh.node_times[i + 1] - scenario.start_time;

            if t_exit <= t_entry {
                continue;
            }

            if let Some(stat) = stats.get_mut(&link_id) {
                // Flow: count the vehicle in the bin where it exits the link
                let exit_bin = (t_exit / bin_size).floor() as usize;
                if exit_bin < n_bins {
                    stat.flow.values[exit_bin] += 1.0;
                }

                // Density: Edie's definition — weight each vehicle by the fraction
                // of the bin it actually occupies (time overlap / bin_size).
                // This correctly handles partial presence:
                //   - A vehicle transiting in 9.8s on a 60s bin contributes 9.8/60,
                //     not 1.0 (which count-based would give, overcounting by ×6).
                //   - A vehicle present for the full bin contributes exactly 1.0.
                // Accumulate vehicle-hours; normalize to veh/km below.
                let start_bin = (t_entry / bin_size).floor() as usize;
                let end_bin = (t_exit / bin_size).floor() as usize;

                for bin in start_bin..=end_bin {
                    if bin >= n_bins {
                        break;
                    }
                    let bin_start = bin as f64 * bin_size;
                    let bin_end = (bin + 1) as f64 * bin_size;
                    let t0 = f64::max(t_entry, bin_start);
                    let t1 = f64::min(t_exit, bin_end);
                    if t1 > t0 {
                        stat.density.values[bin] += (t1 - t0) / 3600.0; // vehicle-hours
                    }
                }
            }
        }
    }

    // Normalize
    for (lid, stat) in stats.iter_mut() {
        let (link_len_km, u_kmh) = scenario
            .links
            .iter()
            .find(|l| l.id == *lid)
            .map(|l| (l.length / 1000.0, l.fd.u * 3.6))
            .unwrap_or((1.0, 0.0));

        for i in 0..n_bins {
            // Flow: count / bin_hours → veh/h
            stat.flow.values[i] /= bin_size / 3600.0;

            // Density: total_vehicle_hours / (bin_hours × link_km) → veh/km
            // Edie's definition: time-averaged spatial density.
            // Bounded by kx when the storage constraint is respected.
            // Values > kx in the raw data indicate queue-overflow (vehicles waiting
            // at the downstream node inflate the measured travel time on the link).
            // The UI uses kx_veh_km to cap the display axis.
            stat.density.values[i] /= (bin_size / 3600.0) * link_len_km;

            // Speed: space-mean speed q/k; free-flow when link is empty
            stat.speed.values[i] = if stat.density.values[i] > 0.001 {
                stat.flow.values[i] / stat.density.values[i]
            } else {
                u_kmh
            };
        }
    }

    stats
}

#[cfg(test)]
mod pipe_stats_tests {
    use super::*;
    use crate::model::{
        FundamentalDiagram, Link, Node, NodeType, PipeSpec, Scenario, Vehicle, ALL_CLASSES,
    };
    use crate::simulation::Simulation;

    /// Entry(0) -> [L0: 2 pipes] -> Exit(1), light traffic.
    fn scenario() -> Scenario {
        let fd = FundamentalDiagram {
            u: 10.0,
            w: 5.0,
            kx: 0.2,
            c: 1.0,
        };
        let mut l0 = Link::new(0, 0, 1, 100.0, 10.0, 2, 2.0, fd, vec![]);
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
            links: vec![l0],
            vehicles: (0..8)
                .map(|i| Vehicle::new(i, 0, vec![0], i as f64 * 5.0, 0, 1))
                .collect(),
            demand: vec![],
            start_time: 0.0,
            duration: 600.0,
            classes: vec!["car".to_string()],
            link_schedule: Vec::new(),
            assignment_excluded: Vec::new(),
        }
    }

    #[test]
    fn pipe_stats_sum_to_link_stats() {
        let mut sim = Simulation::new(scenario());
        sim.run().unwrap();
        let link_stats = compute_link_stats(&sim.scenario, 60.0);
        let pipe_stats = compute_pipe_stats(&sim.scenario, 60.0);

        let ls = &link_stats[&0];
        let ps = &pipe_stats[&0];
        assert_eq!(ps.len(), 2);
        for i in 0..ls.flow.values.len() {
            let flow_sum: f64 = ps.iter().map(|p| p.flow.values[i]).sum();
            let dens_sum: f64 = ps.iter().map(|p| p.density.values[i]).sum();
            assert!((flow_sum - ls.flow.values[i]).abs() < 1e-9, "flow bin {i}");
            assert!(
                (dens_sum - ls.density.values[i]).abs() < 1e-9,
                "density bin {i}"
            );
        }
        // Both pipes used (least-occupied alternation).
        assert!(ps.iter().all(|p| p.flow.values.iter().sum::<f64>() > 0.0));
    }
}
