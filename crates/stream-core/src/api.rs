//! Stable entry point for hosts that embed the engine (the RustGIS plugin,
//! the CLI, scripts).
//!
//! Everything goes in and out as plain serializable values, so a host never
//! touches solver internals:
//!
//! - [`check`] compiles and validates a scenario without simulating it;
//! - [`run`] compiles, assigns, simulates and aggregates, with progress
//!   reports and cancellation;
//! - [`RunOutput`] carries per-vehicle results and per-link / per-pipe series.
//!
//! Inputs use the same schema as the files of a scenario folder
//! (`network.geojson`, `demand.json`, `config.json`, `patches/*.json`).

use crate::analysis::{
    compute_link_cvcs, compute_link_stats, compute_od_skims, compute_pipe_stats,
    compute_queue_summaries, compute_turning_counts, compute_vehicle_kpis, LinkCvc,
    LinkQueueSummary, LinkStats, OdSkim, PipeStats, TurningMovementCount, VehicleKpis,
};
use crate::assignment::{assign_demand, NetworkGraph};
use crate::io::compile_with_patches;
use crate::model::{LinkID, NodeID, NodeType, PipeIdx, Scenario};
use crate::patch::Patch;
use crate::simulation::{MergeRule, Simulation};
use crate::validation::Validate;
use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Version of the [`RunOutput`] / [`CheckReport`] format. Bumped on any
/// breaking change so hosts can refuse results they cannot read.
pub const API_VERSION: u32 = 1;

/// The source inputs of one scenario variant.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioInputs {
    /// GeoJSON FeatureCollection of nodes (Points) and links (LineStrings).
    pub network: serde_json::Value,
    /// Array of demand entries.
    pub demand: serde_json::Value,
    /// `{start_time, duration, classes}`.
    pub config: serde_json::Value,
    /// `signals.json`: signal groups, fixed-time plans, tolls (optional).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub signals: Option<serde_json::Value>,
    /// Patches applied in order on top of the base inputs.
    #[serde(default)]
    pub patches: Vec<Patch>,
}

impl ScenarioInputs {
    /// Read a scenario folder (`network.geojson`, `demand.json`,
    /// `config.json`, and `signals.json` when present) plus the named
    /// patches from its `patches/` folder.
    pub fn from_dir(dir: impl AsRef<Path>, patch_names: &[String]) -> Result<Self> {
        let dir = dir.as_ref();
        let read = |name: &str| -> Result<serde_json::Value> {
            let path = dir.join(name);
            let text = std::fs::read_to_string(&path)
                .with_context(|| format!("cannot read {}", path.display()))?;
            serde_json::from_str(&text).with_context(|| format!("invalid JSON in {}", path.display()))
        };
        let patches = patch_names
            .iter()
            .map(|n| {
                let v = read(&format!("patches/{n}.json"))?;
                serde_json::from_value(v).with_context(|| format!("invalid patch '{n}'"))
            })
            .collect::<Result<Vec<Patch>>>()?;
        Ok(Self {
            network: read("network.geojson")?,
            demand: read("demand.json")?,
            config: read("config.json")?,
            signals: if dir.join("signals.json").exists() { Some(read("signals.json")?) } else { None },
            patches,
        })
    }
}

/// Compile and validate the inputs into a ready-to-run scenario.
/// Returns the scenario and the patch warnings.
pub fn compile(inputs: &ScenarioInputs) -> Result<(Scenario, Vec<String>)> {
    let (mut scenario, warnings) = compile_with_patches(
        inputs.network.clone(),
        inputs.demand.clone(),
        inputs.config.clone(),
        &inputs.patches,
    )?;
    if let Some(signals) = &inputs.signals {
        scenario.signals = crate::signals::compile(signals.clone(), &scenario)?;
    }
    scenario.validate()?;
    Ok((scenario, warnings))
}

// ── check ────────────────────────────────────────────────────────────────────

/// What `check` found: the scenario's size, and its problems.
/// `errors` empty ⇔ the scenario can be simulated.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CheckReport {
    pub api_version: u32,
    pub nodes: usize,
    pub entries: usize,
    pub exits: usize,
    pub links: usize,
    /// Links with more than one pipe.
    pub multi_pipe_links: usize,
    pub demand_entries: usize,
    /// Σ demand `count` (vehicles to generate).
    pub demand_vehicles: f64,
    pub classes: Vec<String>,
    pub start_time: f64,
    pub duration: f64,
    pub patches: usize,
    /// Mid-run link changes produced by the patches.
    pub schedule_changes: usize,
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
}

impl CheckReport {
    pub fn ok(&self) -> bool {
        self.errors.is_empty()
    }
}

/// Compile and validate the inputs without simulating. Never fails: problems
/// are reported in `errors` (compilation stops at the first one) and
/// `warnings` (demand that assignment would silently drop, patch notes).
pub fn check(inputs: &ScenarioInputs) -> CheckReport {
    let mut report = CheckReport {
        api_version: API_VERSION,
        patches: inputs.patches.len(),
        ..Default::default()
    };
    let (scenario, warnings) = match compile(inputs) {
        Ok(v) => v,
        Err(e) => {
            report.errors.push(format!("{e:#}"));
            return report;
        }
    };
    if let Err(e) = MergeRule::from_config(&inputs.config) {
        report.errors.push(format!("{e:#}"));
    }
    report.warnings = warnings;
    report.nodes = scenario.nodes.len();
    report.entries = count_nodes(&scenario, NodeType::Entry);
    report.exits = count_nodes(&scenario, NodeType::Exit);
    report.links = scenario.links.len();
    report.multi_pipe_links = scenario.links.iter().filter(|l| l.pipe_specs.len() > 1).count();
    report.demand_entries = scenario.demand.len();
    report.demand_vehicles = scenario.demand.iter().map(|d| d.count.max(0.0)).sum();
    report.classes = scenario.classes.clone();
    report.start_time = scenario.start_time;
    report.duration = scenario.duration;
    report.schedule_changes = scenario.link_schedule.len();

    // Assignment skips demand it cannot route without saying so: report it.
    let graph = NetworkGraph::from_scenario(&scenario);
    let horizon_end = scenario.start_time + scenario.duration;
    for d in &scenario.demand {
        if d.count <= 0.0 {
            continue;
        }
        if let Some(path) = &d.path {
            if let Some(&closed) = path.iter().find(|lid| scenario.assignment_excluded.contains(lid)) {
                report.warnings.push(format!(
                    "demand {}→{}: explicit path uses closed link {}, its {} vehicle(s) will not be generated",
                    d.origin, d.destination, closed, d.count
                ));
            }
        } else if graph.shortest_path(d.origin, d.destination).is_none() {
            report.warnings.push(format!(
                "demand {}→{}: no route, its {} vehicle(s) will not be generated",
                d.origin, d.destination, d.count
            ));
        }
        if d.period_end <= d.period_start {
            report.warnings.push(format!(
                "demand {}→{}: empty period [{}, {}), skipped",
                d.origin, d.destination, d.period_start, d.period_end
            ));
        } else if d.period_start >= horizon_end || d.period_end <= scenario.start_time {
            report.warnings.push(format!(
                "demand {}→{}: period [{}, {}) lies outside the simulated horizon",
                d.origin, d.destination, d.period_start, d.period_end
            ));
        }
    }
    // Assignment-time errors (class reachability) without keeping the vehicles.
    let mut probe = scenario;
    match assign_demand(&mut probe) {
        Err(e) => report.errors.push(format!("{e:#}")),
        Ok(0) if probe.vehicles.is_empty() => report.warnings.push(
            "the demand generates no vehicle: the simulation would exercise nothing".to_string(),
        ),
        Ok(_) => {}
    }
    report
}

fn count_nodes(s: &Scenario, t: NodeType) -> usize {
    s.nodes.iter().filter(|n| n.node_type == t).count()
}

// ── run ──────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RunOptions {
    /// Generate and route vehicles from the demand before simulating.
    pub assignment: bool,
    /// Width of the aggregation bins, in seconds.
    pub bin_size: f64,
    /// Upstream supply term at node transfers (see `Simulation`).
    pub upstream_capacity: bool,
    /// Keep per-vehicle results in the output (can be large).
    pub keep_vehicles: bool,
}

impl Default for RunOptions {
    fn default() -> Self {
        Self { assignment: true, bin_size: 300.0, upstream_capacity: false, keep_vehicles: false }
    }
}

/// Progress of a running simulation, handed to the observer.
#[derive(Debug, Clone, Copy)]
pub enum Progress {
    Compiled { links: usize, vehicles: usize },
    Simulating { sim_time: f64, start_time: f64, end_time: f64, events: usize },
    Aggregating,
}

/// One vehicle's trip: the links it took, the pipe on each, and the time it
/// passed each node (`node_times[i]` = entry of `path[i]`, the last one = exit).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleResult {
    pub id: usize,
    pub class_id: usize,
    pub origin: NodeID,
    pub destination: NodeID,
    pub start_time: f64,
    pub path: Vec<LinkID>,
    pub node_times: Vec<f64>,
    pub pipes_taken: Vec<PipeIdx>,
    /// Roundabout exits: (index in `path` of the link entered, time). At a
    /// roundabout, `node_times` holds the ring entry; the next link starts
    /// here. Absent when the trip crosses no roundabout.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub ring_exits: Vec<(usize, f64)>,
}

/// Everything a host needs to present a run.
#[derive(Debug, Serialize, Deserialize)]
pub struct RunOutput {
    pub api_version: u32,
    pub engine_version: String,
    pub start_time: f64,
    pub duration: f64,
    pub bin_size: f64,
    pub classes: Vec<String>,
    pub events_processed: usize,
    pub vehicles_total: usize,
    /// Vehicles that reached their destination within the horizon.
    pub vehicles_arrived: usize,
    /// Pre-computed aggregate vehicle KPIs across all vehicles.
    #[serde(default)]
    pub vehicle_kpis: VehicleKpis,
    pub warnings: Vec<String>,
    /// Sorted by link id.
    pub link_stats: Vec<LinkStats>,
    /// Pre-computed entry/exit cumulative volume curves per link, sorted by link id.
    #[serde(default)]
    pub link_cvcs: Vec<LinkCvc>,
    /// Per-pipe series of multi-pipe links only, sorted by (link, pipe).
    pub pipe_stats: Vec<PipeStats>,
    /// Turning-movement counts across traversed node pairs, sorted by `(node_id, from_link, to_link)`.
    #[serde(default)]
    pub turning_counts: Vec<TurningMovementCount>,
    /// Per-link occupancy, spillback duration, and delay summaries, sorted by `link_id`.
    #[serde(default)]
    pub queue_summaries: Vec<LinkQueueSummary>,
    /// Origin-destination travel-time skims, sorted by `(origin, destination)`.
    #[serde(default)]
    pub od_skims: Vec<OdSkim>,
    /// Empty unless `RunOptions::keep_vehicles`.
    #[serde(default)]
    pub vehicles: Vec<VehicleResult>,
}

/// Number of events between two progress reports.
const PROGRESS_EVERY: usize = 50_000;

/// Compile, assign, simulate and aggregate one scenario variant.
/// `observer` is told how the run goes; returning `false` from it cancels the
/// run (the call then returns an error).
pub fn run<F>(inputs: &ScenarioInputs, options: &RunOptions, mut observer: F) -> Result<RunOutput>
where
    F: FnMut(Progress) -> bool,
{
    if !(options.bin_size.is_finite() && options.bin_size >= 1.0) {
        anyhow::bail!("bin_size must be a finite number of seconds ≥ 1");
    }
    let (mut scenario, warnings) = compile(inputs)?;
    if options.assignment {
        assign_demand(&mut scenario)?;
    }
    if !observer(Progress::Compiled { links: scenario.links.len(), vehicles: scenario.vehicles.len() }) {
        anyhow::bail!("simulation cancelled");
    }
    let start_time = scenario.start_time;
    let end_time = start_time + scenario.duration;

    let mut sim = Simulation::new(scenario);
    sim.upstream_capacity = options.upstream_capacity;
    sim.merge_rule = MergeRule::from_config(&inputs.config)?;
    let events_processed = sim.run_with_progress(PROGRESS_EVERY, |sim_time, events| {
        observer(Progress::Simulating { sim_time, start_time, end_time, events })
    })?;
    observer(Progress::Aggregating);
    Ok(collect(sim.scenario, events_processed, warnings, options))
}

/// Turn a simulated scenario into a `RunOutput`.
pub fn collect(
    scenario: Scenario,
    events_processed: usize,
    warnings: Vec<String>,
    options: &RunOptions,
) -> RunOutput {
    let mut link_stats: Vec<LinkStats> =
        compute_link_stats(&scenario, options.bin_size).into_values().collect();
    link_stats.sort_by_key(|s| s.link_id);

    let mut link_cvcs: Vec<LinkCvc> =
        compute_link_cvcs(&scenario, options.bin_size).into_values().collect();
    link_cvcs.sort_by_key(|c| c.link_id);

    let multi: std::collections::HashSet<LinkID> =
        scenario.links.iter().filter(|l| l.pipes.len() > 1).map(|l| l.id).collect();
    let mut pipe_stats: Vec<PipeStats> = if multi.is_empty() {
        Vec::new()
    } else {
        compute_pipe_stats(&scenario, options.bin_size)
            .into_iter()
            .filter(|(id, _)| multi.contains(id))
            .flat_map(|(_, v)| v)
            .collect()
    };
    pipe_stats.sort_by_key(|s| (s.link_id, s.pipe));

    let vehicle_kpis = compute_vehicle_kpis(&scenario);
    let vehicles_arrived = vehicle_kpis.arrived;
    let turning_counts = compute_turning_counts(&scenario);
    let queue_summaries = compute_queue_summaries(&scenario);
    let od_skims = compute_od_skims(&scenario);
    let vehicles = if options.keep_vehicles {
        scenario
            .vehicles
            .iter()
            .map(|v| VehicleResult {
                id: v.id,
                class_id: v.class_id,
                origin: v.origin,
                destination: v.destination,
                start_time: v.start_time,
                path: v.path.clone(),
                node_times: v.node_times.clone(),
                pipes_taken: v.pipes_taken.clone(),
                ring_exits: v.ring_exits.clone(),
            })
            .collect()
    } else {
        Vec::new()
    };

    RunOutput {
        api_version: API_VERSION,
        engine_version: env!("CARGO_PKG_VERSION").to_string(),
        start_time: scenario.start_time,
        duration: scenario.duration,
        bin_size: options.bin_size,
        classes: scenario.classes.clone(),
        events_processed,
        vehicles_total: scenario.vehicles.len(),
        vehicles_arrived,
        vehicle_kpis,
        warnings,
        link_stats,
        link_cvcs,
        pipe_stats,
        turning_counts,
        queue_summaries,
        od_skims,
        vehicles,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dir(name: &str) -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("scenarios")
            .join(name)
    }

    #[test]
    fn check_reports_a_valid_scenario() {
        let inputs = ScenarioInputs::from_dir(dir("diverge"), &[]).unwrap();
        let r = check(&inputs);
        assert!(r.ok(), "{:?}", r.errors);
        assert!(r.links > 0 && r.entries > 0 && r.exits > 0);
    }

    #[test]
    fn check_turns_compile_errors_into_a_report() {
        let mut inputs = ScenarioInputs::from_dir(dir("diverge"), &[]).unwrap();
        inputs.network = serde_json::json!({"type": "Point", "coordinates": [0, 0]});
        let r = check(&inputs);
        assert!(!r.ok());
    }

    #[test]
    fn run_matches_the_plain_engine_bit_for_bit() {
        let inputs = ScenarioInputs::from_dir(dir("diverge"), &[]).unwrap();
        let out = run(&inputs, &RunOptions { keep_vehicles: true, ..Default::default() }, |_| true).unwrap();

        let (mut scenario, _) = compile(&inputs).unwrap();
        assign_demand(&mut scenario).unwrap();
        let mut sim = Simulation::new(scenario);
        let events = sim.run().unwrap();

        assert_eq!(out.events_processed, events);
        assert_eq!(out.vehicles.len(), sim.scenario.vehicles.len());
        for (a, b) in out.vehicles.iter().zip(&sim.scenario.vehicles) {
            let bits = |t: &[f64]| t.iter().map(|x| x.to_bits()).collect::<Vec<_>>();
            assert_eq!(bits(&a.node_times), bits(&b.node_times), "vehicle {}", a.id);
            assert_eq!(a.pipes_taken, b.pipes_taken);
        }
        assert!(out.vehicles_arrived > 0);
        assert_eq!(out.link_stats.len(), sim.scenario.links.len());
        assert_eq!(out.link_cvcs.len(), sim.scenario.links.len());
        assert_eq!(out.vehicle_kpis.total, sim.scenario.vehicles.len());
        assert_eq!(out.vehicle_kpis.arrived, out.vehicles_arrived);
        assert!(out.vehicle_kpis.total_vkt_veh_km > 0.0);
        assert!(out.vehicle_kpis.mean_speed_kmh > 0.0);
    }

    #[test]
    fn run_can_be_cancelled() {
        let inputs = ScenarioInputs::from_dir(dir("diverge"), &[]).unwrap();
        let err = run(&inputs, &RunOptions::default(), |p| !matches!(p, Progress::Compiled { .. }));
        assert!(err.is_err());
    }

    #[test]
    fn output_round_trips_through_json() {
        let inputs = ScenarioInputs::from_dir(dir("diverge"), &[]).unwrap();
        let out = run(&inputs, &RunOptions::default(), |_| true).unwrap();
        assert!(out.vehicles.is_empty(), "keep_vehicles defaults to false");
        assert_eq!(out.link_cvcs.len(), out.link_stats.len());
        assert_eq!(out.vehicle_kpis.total, out.vehicles_total);
        let text = serde_json::to_string(&out).unwrap();
        let back: RunOutput = serde_json::from_str(&text).unwrap();
        assert_eq!(back.events_processed, out.events_processed);
        assert_eq!(back.link_stats.len(), out.link_stats.len());
        assert_eq!(back.link_cvcs, out.link_cvcs);
        assert_eq!(back.vehicle_kpis, out.vehicle_kpis);
        assert_eq!(back.turning_counts, out.turning_counts);
        assert_eq!(back.queue_summaries.len(), out.queue_summaries.len());
        for (a, b) in back.queue_summaries.iter().zip(out.queue_summaries.iter()) {
            assert_eq!(a.link_id, b.link_id);
            assert_eq!(a.max_occupancy, b.max_occupancy);
            assert_eq!(a.end_occupancy, b.end_occupancy);
            assert!((a.total_delay_veh_h - b.total_delay_veh_h).abs() < 1e-12);
        }
        assert_eq!(back.od_skims.len(), out.od_skims.len());
        assert!(!out.turning_counts.is_empty());
        assert_eq!(out.queue_summaries.len(), out.link_stats.len());
        assert!(!out.od_skims.is_empty());
    }

    #[test]
    fn route_based_demand_runs_through_api_and_warns_on_closed_link() {
        let mut inputs = ScenarioInputs::from_dir(dir("diverge"), &[]).unwrap();
        inputs.demand = serde_json::json!([
            {"period_start": 0.0, "period_end": 1800.0, "flow": 360.0, "path": [0, 1]},
            {"period_start": 0.0, "period_end": 1800.0, "flow": 180.0, "path": [0, 2]}
        ]);
        let r = check(&inputs);
        assert!(r.ok(), "{:?}", r.errors);
        assert!((r.demand_vehicles - 270.0).abs() < 1e-9);

        let out = run(&inputs, &RunOptions { keep_vehicles: true, ..Default::default() }, |_| true).unwrap();
        assert_eq!(out.vehicles_total, 270);
        assert_eq!(out.vehicles.iter().filter(|v| v.path == vec![0, 1]).count(), 180);
        assert_eq!(out.vehicles.iter().filter(|v| v.path == vec![0, 2]).count(), 90);

        inputs.patches.push(crate::patch::Patch {
            name: "close-branch-1".into(),
            description: String::new(),
            ops: vec![crate::patch::PatchOp::CloseLink {
                link: 1,
                from: None,
                until: None,
            }],
        });
        let r_closed = check(&inputs);
        assert!(r_closed.ok(), "{:?}", r_closed.errors);
        assert!(
            r_closed.warnings.iter().any(|w| w.contains("closed link 1")),
            "expected warning for closed link 1, got {:?}",
            r_closed.warnings
        );
    }
}
