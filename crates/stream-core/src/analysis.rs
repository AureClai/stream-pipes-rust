use crate::model::{Link, LinkID, NodeID, PipeIdx, Scenario, Vehicle, VehicleState};
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

/// Turning-movement volume across one node (`from_link -> to_link`), aggregated
/// over the simulation horizon.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct TurningMovementCount {
    pub node_id: NodeID,
    pub from_link: LinkID,
    pub to_link: LinkID,
    /// Vehicles that completed `from_link` and entered `to_link` during the run.
    pub count: usize,
    /// Equivalent hourly flow over `scenario.duration` (`count * 3600 / duration`, veh/h).
    pub flow_veh_h: f64,
    /// Split share of `from_link`'s total outgoing crossings at `node_id` in `[0, 1]`.
    pub share: f64,
}

/// Compute turning-movement counts across all traversed node pairs `(from_link -> to_link)`,
/// sorted by `(node_id, from_link, to_link)`.
pub fn compute_turning_counts(scenario: &Scenario) -> Vec<TurningMovementCount> {
    let link_down_map: HashMap<LinkID, NodeID> =
        scenario.links.iter().map(|l| (l.id, l.node_down)).collect();

    let mut counts: HashMap<(NodeID, LinkID, LinkID), usize> = HashMap::new();
    let mut from_totals: HashMap<(NodeID, LinkID), usize> = HashMap::new();

    for veh in &scenario.vehicles {
        for i in 0..veh.path.len().saturating_sub(1) {
            if veh.link_exit_time(i).is_none() {
                break;
            }
            let from_link = veh.path[i];
            let to_link = veh.path[i + 1];
            let Some(&node_id) = link_down_map.get(&from_link) else {
                continue;
            };
            *counts.entry((node_id, from_link, to_link)).or_insert(0) += 1;
            *from_totals.entry((node_id, from_link)).or_insert(0) += 1;
        }
    }

    let duration_h = if scenario.duration > 0.0 {
        scenario.duration / 3600.0
    } else {
        0.0
    };

    let mut out: Vec<TurningMovementCount> = counts
        .into_iter()
        .map(|((node_id, from_link, to_link), count)| {
            let total = from_totals.get(&(node_id, from_link)).copied().unwrap_or(0);
            TurningMovementCount {
                node_id,
                from_link,
                to_link,
                count,
                flow_veh_h: if duration_h > 0.0 {
                    count as f64 / duration_h
                } else {
                    0.0
                },
                share: if total > 0 {
                    count as f64 / total as f64
                } else {
                    0.0
                },
            }
        })
        .collect();
    out.sort_by_key(|c| (c.node_id, c.from_link, c.to_link));
    out
}

/// Queueing, occupancy, and spillback summary for one link over `[start_time, end_time]`.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct LinkQueueSummary {
    pub link_id: LinkID,
    /// Peak simultaneous vehicles present on the link during the horizon.
    pub max_occupancy: usize,
    /// Vehicles remaining on the link at the end of the horizon.
    pub end_occupancy: usize,
    /// Whole-link theoretical storage capacity (`kx * num_lanes * length`, veh; 0 if `kx <= 0`).
    pub storage_veh: f64,
    /// `max_occupancy / storage_veh` (0.0 when storage is unlimited).
    pub max_occupancy_ratio: f64,
    /// Total duration (s) during which at least one pipe on the link was at jam storage.
    pub spillback_duration_s: f64,
    /// Mean traversal delay over free-flow (`L / fd.u`) across completed link traversals (s).
    pub mean_delay_s: f64,
    /// Sum of traversal delays across completed link traversals (veh·h).
    pub total_delay_veh_h: f64,
}

/// Sweep `(t, delta)` events (exits `-1` ordered before entries `+1` at equal times)
/// to measure peak occupancy and total duration where `occ >= threshold` (when `Some`).
fn sweep_occupancy(events: &mut [(f64, i32)], horizon_end: f64, threshold: Option<usize>) -> (usize, Vec<(f64, f64)>) {
    events.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.1.cmp(&b.1)) // -1 (exit) before +1 (entry) at identical t
    });
    let mut occ: isize = 0;
    let mut max_occ: usize = 0;
    let mut full_intervals: Vec<(f64, f64)> = Vec::new();
    let mut full_start: Option<f64> = None;

    for &(t, delta) in events.iter() {
        occ = (occ + delta as isize).max(0);
        let uocc = occ as usize;
        max_occ = max_occ.max(uocc);
        if let Some(cap) = threshold {
            if uocc >= cap && full_start.is_none() {
                full_start = Some(t);
            } else if uocc < cap {
                if let Some(s) = full_start.take() {
                    if t > s {
                        full_intervals.push((s, t));
                    }
                }
            }
        }
    }
    if let Some(s) = full_start {
        if horizon_end > s {
            full_intervals.push((s, horizon_end));
        }
    }
    (max_occ, full_intervals)
}

/// Compute union duration of `[start, end)` intervals.
fn union_duration(mut intervals: Vec<(f64, f64)>) -> f64 {
    if intervals.is_empty() {
        return 0.0;
    }
    intervals.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let mut total = 0.0;
    let (mut cur_s, mut cur_e) = intervals[0];
    for (s, e) in intervals.into_iter().skip(1) {
        if s <= cur_e {
            cur_e = cur_e.max(e);
        } else {
            total += (cur_e - cur_s).max(0.0);
            cur_s = s;
            cur_e = e;
        }
    }
    total + (cur_e - cur_s).max(0.0)
}

/// Compute per-link occupancy, spillback duration, and traversal delay summaries,
/// sorted by `link_id`.
pub fn compute_queue_summaries(scenario: &Scenario) -> Vec<LinkQueueSummary> {
    let start_time = scenario.start_time;
    let horizon_end = scenario.start_time + scenario.duration.max(0.0);

    // Per link: link events, per-pipe events, end occupancy, delay sum, completed count.
    struct Acc {
        link_events: Vec<(f64, i32)>,
        pipe_events: Vec<Vec<(f64, i32)>>,
        end_occupancy: usize,
        delay_s_sum: f64,
        completed: usize,
    }

    let link_idx: HashMap<LinkID, usize> = scenario
        .links
        .iter()
        .enumerate()
        .map(|(i, l)| (l.id, i))
        .collect();
    let mut accs: Vec<Acc> = scenario
        .links
        .iter()
        .map(|l| Acc {
            link_events: Vec::new(),
            pipe_events: vec![Vec::new(); l.pipes.len().max(1)],
            end_occupancy: 0,
            delay_s_sum: 0.0,
            completed: 0,
        })
        .collect();

    for veh in &scenario.vehicles {
        for (i, &lid) in veh.path.iter().enumerate() {
            let Some(t_in) = veh.link_entry_time(i) else {
                break;
            };
            let Some(&li) = link_idx.get(&lid) else {
                continue;
            };
            let link = &scenario.links[li];
            let acc = &mut accs[li];
            let pipe = usize::from(veh.pipes_taken.get(i).copied().unwrap_or(0))
                .min(acc.pipe_events.len() - 1);

            let t_in_c = t_in.clamp(start_time, horizon_end);
            acc.link_events.push((t_in_c, 1));
            acc.pipe_events[pipe].push((t_in_c, 1));

            match veh.link_exit_time(i) {
                Some(t_out) => {
                    let t_out_c = t_out.clamp(start_time, horizon_end);
                    acc.link_events.push((t_out_c, -1));
                    acc.pipe_events[pipe].push((t_out_c, -1));
                    if t_out > horizon_end {
                        acc.end_occupancy += 1;
                    }
                    let ff = if link.fd.u > 0.0 {
                        link.length / link.fd.u
                    } else {
                        0.0
                    };
                    acc.delay_s_sum += (t_out - t_in - ff).max(0.0);
                    acc.completed += 1;
                }
                None => {
                    acc.end_occupancy += 1;
                }
            }
        }
    }

    let mut out: Vec<LinkQueueSummary> = scenario
        .links
        .iter()
        .zip(accs.iter_mut())
        .map(|(link, acc)| {
            let (max_occupancy, _) = sweep_occupancy(&mut acc.link_events, horizon_end, None);
            let mut full_intervals = Vec::new();
            for (p, pev) in acc.pipe_events.iter_mut().enumerate() {
                let cap = link
                    .pipes
                    .get(p)
                    .map(|pipe| pipe.storage_veh)
                    .unwrap_or_else(|| link.storage_veh());
                if cap.is_some() {
                    let (_, intervals) = sweep_occupancy(pev, horizon_end, cap);
                    full_intervals.extend(intervals);
                }
            }
            let storage_veh = if link.fd.kx > 0.0 {
                link.storage_exact()
            } else {
                0.0
            };
            LinkQueueSummary {
                link_id: link.id,
                max_occupancy,
                end_occupancy: acc.end_occupancy,
                storage_veh,
                max_occupancy_ratio: if storage_veh > 0.0 {
                    max_occupancy as f64 / storage_veh
                } else {
                    0.0
                },
                spillback_duration_s: union_duration(full_intervals),
                mean_delay_s: if acc.completed > 0 {
                    acc.delay_s_sum / acc.completed as f64
                } else {
                    0.0
                },
                total_delay_veh_h: acc.delay_s_sum / 3600.0,
            }
        })
        .collect();
    out.sort_by_key(|q| q.link_id);
    out
}

/// Origin-destination travel-time skim summary for one `(origin, destination)` pair.
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct OdSkim {
    pub origin: NodeID,
    pub destination: NodeID,
    /// Scheduled vehicles for this OD pair.
    pub vehicles_total: usize,
    /// Vehicles that reached `destination` within the simulation horizon.
    pub vehicles_arrived: usize,
    /// Mean in-network travel time (`t_exit - t_enter`, s) of arrived vehicles.
    pub mean_travel_time_s: f64,
    /// Median (50th percentile) in-network travel time (s) of arrived vehicles.
    pub p50_travel_time_s: f64,
    /// 95th percentile in-network travel time (s) of arrived vehicles.
    pub p95_travel_time_s: f64,
    /// Mean total travel time including entry-queue wait (`t_exit - start_time`, s).
    pub mean_total_time_s: f64,
    /// Mean free-flow travel time (s) of arrived vehicles.
    pub mean_free_flow_time_s: f64,
    /// Mean in-network delay over free-flow (s) of arrived vehicles.
    pub mean_delay_s: f64,
}

/// Returns `(travel_s, total_s, ff_s, delay_s, dist_m)` for a vehicle that exited.
fn exited_vehicle_metrics(
    veh: &Vehicle,
    link_map: &HashMap<LinkID, &Link>,
) -> Option<(f64, f64, f64, f64, f64)> {
    if veh.state != VehicleState::Exited || veh.node_times.is_empty() {
        return None;
    }
    let t_enter = veh.node_times[0];
    let t_exit = *veh.node_times.last()?;
    let travel_s = (t_exit - t_enter).max(0.0);
    let total_s = (t_exit - veh.start_time).max(0.0);
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
    Some((travel_s, total_s, ff_s, delay_s, dist_m))
}

/// Compute origin-destination travel-time skims across all vehicles, sorted by `(origin, destination)`.
pub fn compute_od_skims(scenario: &Scenario) -> Vec<OdSkim> {
    let link_map: HashMap<LinkID, &Link> = scenario.links.iter().map(|l| (l.id, l)).collect();

    #[derive(Default)]
    struct OdAcc {
        total: usize,
        arrived: usize,
        travel_times: Vec<f64>,
        travel_sum: f64,
        total_sum: f64,
        ff_sum: f64,
        delay_sum: f64,
    }

    let mut groups: HashMap<(NodeID, NodeID), OdAcc> = HashMap::new();
    for veh in &scenario.vehicles {
        let acc = groups.entry((veh.origin, veh.destination)).or_default();
        acc.total += 1;
        if let Some((travel_s, total_s, ff_s, delay_s, _)) = exited_vehicle_metrics(veh, &link_map) {
            acc.arrived += 1;
            acc.travel_times.push(travel_s);
            acc.travel_sum += travel_s;
            acc.total_sum += total_s;
            acc.ff_sum += ff_s;
            acc.delay_sum += delay_s;
        }
    }

    let mut out: Vec<OdSkim> = groups
        .into_iter()
        .map(|((origin, destination), mut acc)| {
            acc.travel_times
                .sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let n = acc.arrived as f64;
            OdSkim {
                origin,
                destination,
                vehicles_total: acc.total,
                vehicles_arrived: acc.arrived,
                mean_travel_time_s: if acc.arrived > 0 { acc.travel_sum / n } else { 0.0 },
                p50_travel_time_s: percentile_sorted(&acc.travel_times, 50.0),
                p95_travel_time_s: percentile_sorted(&acc.travel_times, 95.0),
                mean_total_time_s: if acc.arrived > 0 { acc.total_sum / n } else { 0.0 },
                mean_free_flow_time_s: if acc.arrived > 0 { acc.ff_sum / n } else { 0.0 },
                mean_delay_s: if acc.arrived > 0 { acc.delay_sum / n } else { 0.0 },
            }
        })
        .collect();
    out.sort_by_key(|s| (s.origin, s.destination));
    out
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

        if let Some((travel_s, _, ff_s, delay_s, dist_m)) = exited_vehicle_metrics(veh, &link_map) {
            arrived += 1;
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

    #[test]
    fn turning_counts_queue_summaries_and_od_skims() {
        // Diverge: Entry(0) -> L0 (100 m, u=10, c=1) -> Internal(1) -> L1 (to Exit 2) / L2 (to Exit 3).
        let fd = FundamentalDiagram {
            u: 10.0,
            w: 5.0,
            kx: 0.2,
            c: 1.0,
        };
        let node = |id, node_type, inc, out| crate::model::Node {
            id,
            node_type,
            incoming_links: inc,
            outgoing_links: out,
            points: (id as f64, 0.0),
            signals: vec![],
            roundabout: None,
            junction: None,
        };
        let mut l1 = Link::new(1, 1, 2, 100.0, 10.0, 1, 1.0, fd.clone(), vec![]);
        l1.pipe_specs = vec![];
        let mut l2 = Link::new(2, 1, 3, 200.0, 10.0, 1, 0.5, FundamentalDiagram { c: 0.5, ..fd.clone() }, vec![]);
        l2.pipe_specs = vec![];
        // L0 (100 m, u=10 -> 10 s transit, kx=0.05 -> storage 5 veh, c=2.0) holds multiple vehicles simultaneously.
        let mut l0 = Link::new(
            0,
            0,
            1,
            100.0,
            10.0,
            1,
            2.0,
            FundamentalDiagram {
                kx: 0.05,
                c: 2.0,
                ..fd
            },
            vec![],
        );
        l0.pipe_specs = vec![];

        let sc = Scenario {
            nodes: vec![
                node(0, crate::model::NodeType::Entry, vec![], vec![0]),
                node(1, crate::model::NodeType::Internal, vec![0], vec![1, 2]),
                node(2, crate::model::NodeType::Exit, vec![1], vec![]),
                node(3, crate::model::NodeType::Exit, vec![2], vec![]),
            ],
            links: vec![l0, l1, l2],
            vehicles: vec![
                Vehicle::new(0, 0, vec![0, 1], 0.0, 0, 2),
                Vehicle::new(1, 0, vec![0, 1], 0.5, 0, 2),
                Vehicle::new(2, 0, vec![0, 2], 1.0, 0, 3),
                Vehicle::new(3, 0, vec![0, 2], 1.5, 0, 3),
                Vehicle::new(4, 0, vec![0, 2], 2.0, 0, 3),
            ],
            demand: vec![],
            start_time: 0.0,
            duration: 600.0,
            classes: vec!["car".to_string()],
            link_schedule: Vec::new(),
            assignment_excluded: Vec::new(),
            signals: Default::default(),
            signal_log: Vec::new(),
        };

        let mut sim = Simulation::new(sc);
        sim.run().unwrap();

        let turns = compute_turning_counts(&sim.scenario);
        assert_eq!(turns.len(), 2);
        assert_eq!(turns[0].node_id, 1);
        assert_eq!(turns[0].from_link, 0);
        assert_eq!(turns[0].to_link, 1);
        assert_eq!(turns[0].count, 2);
        assert!((turns[0].share - 0.4).abs() < 1e-12);
        assert_eq!(turns[1].to_link, 2);
        assert_eq!(turns[1].count, 3);
        assert!((turns[1].share - 0.6).abs() < 1e-12);
        assert!((turns[1].flow_veh_h - 18.0).abs() < 1e-12);

        let queues = compute_queue_summaries(&sim.scenario);
        assert_eq!(queues.len(), 3);
        // L2 (c = 0.5 veh/s = 2 s headway) throttles entry at node 1, queueing vehicles on L0.
        assert!(queues[0].mean_delay_s > 0.0);
        assert_eq!(queues[0].max_occupancy, 5);
        assert!(queues[0].spillback_duration_s > 0.0);
        assert_eq!(queues[0].end_occupancy, 0);

        let skims = compute_od_skims(&sim.scenario);
        assert_eq!(skims.len(), 2);
        assert_eq!(skims[0].origin, 0);
        assert_eq!(skims[0].destination, 2);
        assert_eq!(skims[0].vehicles_total, 2);
        assert_eq!(skims[0].vehicles_arrived, 2);
        assert!((skims[0].mean_free_flow_time_s - 20.0).abs() < 1e-12);
        assert_eq!(skims[1].destination, 3);
        assert_eq!(skims[1].vehicles_arrived, 3);
        assert!(skims[1].mean_delay_s > 0.0);
    }
}
