use crate::model::{Link, LinkID, PipeIdx, Scenario, VehicleState};
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
            let (Some(t_in), Some(t_out)) = (veh.link_entry_time(i), veh.link_exit_time(i)) else {
                break;
            };
            let pipe = usize::from(veh.pipes_taken.get(i).copied().unwrap_or(0));

            let t_entry = t_in - scenario.start_time;
            let t_exit = t_out - scenario.start_time;
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

        for (i, &link_id) in veh.path.iter().enumerate() {
            let (Some(t_in), Some(t_out)) = (veh.link_entry_time(i), veh.link_exit_time(i)) else {
                break;
            };

            // Use relative time so bin 0 always corresponds to simulation start,
            // regardless of the absolute start_time value.
            let t_entry = t_in - scenario.start_time;
            let t_exit = t_out - scenario.start_time;

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

/// Pre-computed entry and exit cumulative volume curves (CVC) for one link,
/// sampled on the `bin_size` grid `[start_time, start_time + bin_size, …, end_time]`.
///
/// At timestamp `t[k]`:
/// - `entry[k]` is the cumulative number of vehicles that have entered the link;
/// - `exit[k]` is the cumulative number of vehicles that have exited the link;
/// - `entry[k] - exit[k]` is the vehicle accumulation on the link.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct LinkCvc {
    pub link_id: LinkID,
    /// Sample timestamps in absolute simulation seconds (`n_bins + 1` points).
    pub t: Vec<f64>,
    /// Cumulative vehicle entries at each timestamp in `t`.
    pub entry: Vec<usize>,
    /// Cumulative vehicle exits at each timestamp in `t`.
    pub exit: Vec<usize>,
}

/// Compute entry/exit cumulative volume curves (CVC) for every link, sampled at
/// bin boundaries `[start_time, start_time + bin_size, …, start_time + duration]`.
///
/// Entries are counted even for vehicles still on the link at the end of the
/// simulation, so `entry.last() - exit.last()` equals the remaining link
/// accumulation.
pub fn compute_link_cvcs(scenario: &Scenario, bin_size: f64) -> HashMap<LinkID, LinkCvc> {
    let mut cvcs = HashMap::with_capacity(scenario.links.len());
    let n_bins = if scenario.duration > 0.0 && bin_size > 0.0 {
        (scenario.duration / bin_size).ceil() as usize
    } else {
        0
    };
    let end_time = scenario.start_time + scenario.duration.max(0.0);

    let mut t_axis = Vec::with_capacity(n_bins + 1);
    t_axis.push(scenario.start_time);
    for k in 1..=n_bins {
        t_axis.push((scenario.start_time + k as f64 * bin_size).min(end_time));
    }

    for link in &scenario.links {
        cvcs.insert(
            link.id,
            LinkCvc {
                link_id: link.id,
                t: t_axis.clone(),
                entry: vec![0; n_bins + 1],
                exit: vec![0; n_bins + 1],
            },
        );
    }

    if n_bins == 0 {
        return cvcs;
    }

    for veh in &scenario.vehicles {
        for (i, &link_id) in veh.path.iter().enumerate() {
            let Some(t_in) = veh.link_entry_time(i) else {
                break;
            };
            let Some(cvc) = cvcs.get_mut(&link_id) else {
                continue;
            };
            let bin_in = (((t_in - scenario.start_time).max(0.0) / bin_size).floor() as usize)
                .min(n_bins - 1);
            cvc.entry[bin_in + 1] += 1;

            if let Some(t_out) = veh.link_exit_time(i) {
                let bin_out = (((t_out - scenario.start_time).max(0.0) / bin_size).floor()
                    as usize)
                    .min(n_bins - 1);
                cvc.exit[bin_out + 1] += 1;
            }
        }
    }

    for cvc in cvcs.values_mut() {
        for k in 1..=n_bins {
            cvc.entry[k] += cvc.entry[k - 1];
            cvc.exit[k] += cvc.exit[k - 1];
        }
    }

    cvcs
}

/// Summary vehicle and network-wide performance KPIs computed across all
/// vehicles in a simulated scenario.
#[derive(Serialize, Deserialize, Debug, Clone, Default, PartialEq)]
pub struct VehicleKpis {
    /// Total scheduled vehicles (`scenario.vehicles.len()`).
    pub total: usize,
    /// Vehicles that physically entered the network (`!node_times.is_empty()`).
    pub entered: usize,
    /// Vehicles that reached their destination (`state == VehicleState::Exited`).
    pub arrived: usize,
    /// Vehicles still on the network at the end of the horizon (`entered - arrived`).
    pub in_network: usize,
    /// Vehicles still waiting at an entry at the end of the horizon (`total - entered`).
    pub queued_at_entry: usize,
    /// Fraction of scheduled vehicles that arrived (`arrived / total`, 0.0 when empty).
    pub completion_rate: f64,
    /// Total vehicle-kilometres travelled over completed link traversals (veh·km).
    pub total_vkt_veh_km: f64,
    /// Total vehicle-hours spent in the network during the horizon (veh·h):
    /// `t_exit - t_entry` for arrived vehicles, `horizon_end - t_entry` for those still in network.
    pub total_vht_veh_h: f64,
    /// Total vehicle-hours spent waiting in entry queues during the horizon (veh·h):
    /// `t_entry - start_time` for entered vehicles, `max(horizon_end - start_time, 0)` for still-queued ones.
    pub total_entry_wait_veh_h: f64,
    /// Total in-network delay across arrived vehicles (veh·h):
    /// `sum(max(in_network_travel_time - free_flow_time, 0)) / 3600`.
    pub total_delay_veh_h: f64,
    /// Mean in-network travel time of arrived vehicles (s).
    pub mean_travel_time_s: f64,
    /// Median (50th percentile) in-network travel time of arrived vehicles (s).
    pub p50_travel_time_s: f64,
    /// 95th percentile in-network travel time of arrived vehicles (s).
    pub p95_travel_time_s: f64,
    /// Mean free-flow travel time of arrived vehicles (s).
    pub mean_free_flow_time_s: f64,
    /// Mean in-network delay of arrived vehicles (s).
    pub mean_delay_s: f64,
    /// Mean entry-queue waiting time across entered vehicles (s).
    pub mean_entry_wait_s: f64,
    /// Maximum entry-queue waiting time across entered vehicles (s).
    pub max_entry_wait_s: f64,
    /// Space-mean speed across arrived vehicles: `(sum dist_km) / (sum travel_time_h)` (km/h).
    pub mean_speed_kmh: f64,
}

fn percentile_sorted(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let pos = (p / 100.0) * (sorted.len() - 1) as f64;
    let lo = pos.floor() as usize;
    let hi = pos.ceil() as usize;
    let frac = pos - lo as f64;
    sorted[lo] + frac * (sorted[hi] - sorted[lo])
}

/// Compute aggregate vehicle KPIs from a simulated scenario.
pub fn compute_vehicle_kpis(scenario: &Scenario) -> VehicleKpis {
    let total = scenario.vehicles.len();
    if total == 0 {
        return VehicleKpis::default();
    }

    let link_map: HashMap<LinkID, &Link> = scenario.links.iter().map(|l| (l.id, l)).collect();
    let horizon_end = scenario.start_time + scenario.duration.max(0.0);

    let mut entered = 0usize;
    let mut arrived = 0usize;
    let mut total_vkt_m = 0.0f64;
    let mut total_vht_s = 0.0f64;
    let mut entered_wait_s = 0.0f64;
    let mut unentered_wait_s = 0.0f64;
    let mut max_entry_wait_s = 0.0f64;

    let mut arrived_travel_times = Vec::new();
    let mut arrived_travel_s_sum = 0.0f64;
    let mut arrived_ff_s_sum = 0.0f64;
    let mut arrived_delay_s_sum = 0.0f64;
    let mut arrived_dist_m_sum = 0.0f64;

    for veh in &scenario.vehicles {
        if veh.node_times.is_empty() {
            if horizon_end > veh.start_time {
                unentered_wait_s += horizon_end - veh.start_time;
            }
            continue;
        }

        entered += 1;
        let t_enter = veh.node_times[0];
        let wait = (t_enter - veh.start_time).max(0.0);
        entered_wait_s += wait;
        max_entry_wait_s = max_entry_wait_s.max(wait);

        let links_completed = veh.node_times.len().saturating_sub(1);
        for &lid in veh.path.iter().take(links_completed) {
            if let Some(link) = link_map.get(&lid) {
                total_vkt_m += link.length;
            }
        }

        if veh.state == VehicleState::Exited {
            arrived += 1;
            let t_exit = *veh.node_times.last().unwrap_or(&t_enter);
            let travel_s = (t_exit - t_enter).max(0.0);
            let mut dist_m = 0.0f64;
            let mut ff_s = 0.0f64;
            for &lid in &veh.path {
                if let Some(link) = link_map.get(&lid) {
                    dist_m += link.length;
                    if link.speed > 0.0 {
                        ff_s += link.length / link.speed;
                    }
                }
            }
            let delay_s = (travel_s - ff_s).max(0.0);
            arrived_travel_times.push(travel_s);
            arrived_travel_s_sum += travel_s;
            arrived_ff_s_sum += ff_s;
            arrived_delay_s_sum += delay_s;
            arrived_dist_m_sum += dist_m;
            total_vht_s += travel_s;
        } else {
            total_vht_s += (horizon_end - t_enter).max(0.0);
        }
    }

    arrived_travel_times.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    VehicleKpis {
        total,
        entered,
        arrived,
        in_network: entered.saturating_sub(arrived),
        queued_at_entry: total.saturating_sub(entered),
        completion_rate: arrived as f64 / total as f64,
        total_vkt_veh_km: total_vkt_m / 1000.0,
        total_vht_veh_h: total_vht_s / 3600.0,
        total_entry_wait_veh_h: (entered_wait_s + unentered_wait_s) / 3600.0,
        total_delay_veh_h: arrived_delay_s_sum / 3600.0,
        mean_travel_time_s: if arrived > 0 {
            arrived_travel_s_sum / arrived as f64
        } else {
            0.0
        },
        p50_travel_time_s: percentile_sorted(&arrived_travel_times, 50.0),
        p95_travel_time_s: percentile_sorted(&arrived_travel_times, 95.0),
        mean_free_flow_time_s: if arrived > 0 {
            arrived_ff_s_sum / arrived as f64
        } else {
            0.0
        },
        mean_delay_s: if arrived > 0 {
            arrived_delay_s_sum / arrived as f64
        } else {
            0.0
        },
        mean_entry_wait_s: if entered > 0 {
            entered_wait_s / entered as f64
        } else {
            0.0
        },
        max_entry_wait_s,
        mean_speed_kmh: if arrived_travel_s_sum > 0.0 {
            (arrived_dist_m_sum / 1000.0) / (arrived_travel_s_sum / 3600.0)
        } else {
            0.0
        },
    }
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
                    roundabout: None,
                    junction: None,
                },
                Node {
                    id: 1,
                    node_type: NodeType::Exit,
                    incoming_links: vec![0],
                    outgoing_links: vec![],
                    points: (1.0, 0.0),
                    signals: vec![],
                    roundabout: None,
                    junction: None,
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
            signals: Default::default(),
            signal_log: Vec::new(),
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

    #[test]
    fn link_cvcs_track_entries_exits_and_accumulation() {
        let mut sim = Simulation::new(scenario());
        sim.run().unwrap();
        let bin_size = 60.0;
        let cvcs = compute_link_cvcs(&sim.scenario, bin_size);
        let stats = compute_link_stats(&sim.scenario, bin_size);

        let cvc = &cvcs[&0];
        let ls = &stats[&0];
        assert_eq!(cvc.t.len(), ls.flow.values.len() + 1);
        assert_eq!(cvc.entry.len(), cvc.t.len());
        assert_eq!(cvc.exit.len(), cvc.t.len());
        assert_eq!(cvc.entry[0], 0);
        assert_eq!(cvc.exit[0], 0);
        assert_eq!(*cvc.entry.last().unwrap(), 8);
        assert_eq!(*cvc.exit.last().unwrap(), 8);

        for k in 0..cvc.t.len() {
            assert!(cvc.entry[k] >= cvc.exit[k], "causality violated at {k}");
            if k > 0 {
                assert!(cvc.entry[k] >= cvc.entry[k - 1]);
                assert!(cvc.exit[k] >= cvc.exit[k - 1]);
                let bin_exits = (cvc.exit[k] - cvc.exit[k - 1]) as f64;
                let flow_exits = ls.flow.values[k - 1] * (bin_size / 3600.0);
                assert!((bin_exits - flow_exits).abs() < 1e-9, "bin {k}");
            }
        }
    }

    #[test]
    fn link_cvcs_count_unexited_vehicles_remaining_on_link() {
        // Short horizon: 2 vehicles enter link 0 (length 100 m, u = 10 m/s -> 10 s transit),
        // vehicle 0 at t=0 exits at t=10; vehicle 1 at t=12 enters before end=15 and stays on link.
        let mut sc = scenario();
        sc.duration = 15.0;
        sc.vehicles = vec![
            Vehicle::new(0, 0, vec![0], 0.0, 0, 1),
            Vehicle::new(1, 0, vec![0], 12.0, 0, 1),
        ];
        let mut sim = Simulation::new(sc);
        sim.run().unwrap();

        let cvcs = compute_link_cvcs(&sim.scenario, 10.0);
        let cvc = &cvcs[&0];
        assert_eq!(cvc.t, vec![0.0, 10.0, 15.0]);
        assert_eq!(cvc.entry, vec![0, 1, 2]);
        assert_eq!(cvc.exit, vec![0, 0, 1]);
        assert_eq!(cvc.entry.last().unwrap() - cvc.exit.last().unwrap(), 1);
    }

    #[test]
    fn vehicle_kpis_exact_in_free_flow_and_with_incomplete_trips() {
        // Free flow: 8 vehicles, 100 m at 10 m/s -> 10 s travel time, 36 km/h, 0 delay.
        let mut sim = Simulation::new(scenario());
        sim.run().unwrap();
        let kpis = compute_vehicle_kpis(&sim.scenario);
        assert_eq!(kpis.total, 8);
        assert_eq!(kpis.entered, 8);
        assert_eq!(kpis.arrived, 8);
        assert_eq!(kpis.in_network, 0);
        assert_eq!(kpis.queued_at_entry, 0);
        assert!((kpis.completion_rate - 1.0).abs() < 1e-12);
        assert!((kpis.total_vkt_veh_km - 0.8).abs() < 1e-12);
        assert!((kpis.total_vht_veh_h - 80.0 / 3600.0).abs() < 1e-12);
        assert!((kpis.total_delay_veh_h).abs() < 1e-12);
        assert!((kpis.mean_travel_time_s - 10.0).abs() < 1e-12);
        assert!((kpis.p50_travel_time_s - 10.0).abs() < 1e-12);
        assert!((kpis.p95_travel_time_s - 10.0).abs() < 1e-12);
        assert!((kpis.mean_speed_kmh - 36.0).abs() < 1e-12);
        assert!((kpis.mean_entry_wait_s).abs() < 1e-12);

        // Partial horizon: 1 arrived, 1 in_network, 1 queued_at_entry.
        let mut sc = scenario();
        sc.duration = 15.0;
        sc.vehicles = vec![
            Vehicle::new(0, 0, vec![0], 0.0, 0, 1), // enters at 0, exits at 10
            Vehicle::new(1, 0, vec![0], 12.0, 0, 1), // enters at 12, still on link at 15 (3 s)
            Vehicle::new(2, 0, vec![0], 20.0, 0, 1), // scheduled after horizon -> queued_at_entry
        ];
        let mut sim2 = Simulation::new(sc);
        sim2.run().unwrap();
        let k2 = compute_vehicle_kpis(&sim2.scenario);
        assert_eq!(k2.total, 3);
        assert_eq!(k2.entered, 2);
        assert_eq!(k2.arrived, 1);
        assert_eq!(k2.in_network, 1);
        assert_eq!(k2.queued_at_entry, 1);
        assert!((k2.total_vkt_veh_km - 0.1).abs() < 1e-12);
        assert!((k2.total_vht_veh_h - 13.0 / 3600.0).abs() < 1e-12);
        assert!((k2.mean_travel_time_s - 10.0).abs() < 1e-12);
    }
}
