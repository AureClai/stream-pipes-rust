use axum::{
    extract::{Json, Path},
    http::{Method, StatusCode},
    response::{IntoResponse, Response},
    routing::{get, post},
    Router,
};
use serde::{Deserialize, Serialize};
use std::net::SocketAddr;
use std::path::PathBuf;
use std::fs;
use std::io::{BufWriter, Write};
use std::collections::HashMap;
use stream_core_rust::simulation::Simulation;
use stream_core_rust::assignment::assign_demand;
use stream_core_rust::analysis::{compute_link_stats, compute_pipe_stats, LinkStats, PipeStats};
use stream_core_rust::benchmarks::{run_benchmarks, BenchmarkResult};
use stream_core_rust::diagnostics::{run_diagnostics, export_to_xml, DiagnosticsReport};
use stream_core_rust::reference::{compare_with_observations, ObservedRecord, ReferenceReport};
use stream_core_rust::verification::{verify_scenario, VerificationReport};
use stream_core_rust::xt_analysis::{compute_xt_diagrams, XTDiagram, XTParams};
use stream_core_rust::model::LinkID;
use stream_core_rust::io::compile_with_patches;
use stream_core_rust::patch::Patch;
use stream_core_rust::validation::Validate;
use tower_http::cors::{Any, CorsLayer};

// ── Error type ────────────────────────────────────────────────────────────────

enum AppError {
    NotFound(String),
    BadRequest(String),
    Internal(String),
}

impl IntoResponse for AppError {
    fn into_response(self) -> Response {
        match self {
            AppError::NotFound(msg) => (StatusCode::NOT_FOUND, msg).into_response(),
            AppError::BadRequest(msg) => (StatusCode::BAD_REQUEST, msg).into_response(),
            AppError::Internal(msg) => (StatusCode::INTERNAL_SERVER_ERROR, msg).into_response(),
        }
    }
}

impl From<std::io::Error> for AppError {
    fn from(e: std::io::Error) -> Self {
        AppError::Internal(e.to_string())
    }
}

impl From<serde_json::Error> for AppError {
    fn from(e: serde_json::Error) -> Self {
        AppError::Internal(e.to_string())
    }
}

impl From<anyhow::Error> for AppError {
    fn from(e: anyhow::Error) -> Self {
        AppError::Internal(e.to_string())
    }
}

// ── Request / Response types ──────────────────────────────────────────────────

#[derive(Deserialize)]
struct RunRequest {
    project_name: String,
    scenario_name: Option<String>,
    assignment: bool,
    /// Patch-name override: when present, these patches are applied instead
    /// of the variant's saved list (previews / calibration working set).
    #[serde(default)]
    patches: Option<Vec<String>>,
}

#[derive(Deserialize)]
struct DiagnosticsRequest {
    project_name: String,
    scenario_name: Option<String>,
    assignment: bool,
    #[serde(default)]
    patches: Option<Vec<String>>,
    /// Time-bin width in seconds (default: 60).
    bin_size: Option<f64>,
    /// Maximum vehicle trajectories to return (default: 500, 0 = unlimited).
    max_trajectories: Option<usize>,
}

#[derive(Serialize, Deserialize)]
struct RunResponse {
    events_processed: usize,
    duration_ms: u128,
    vehicles_count: usize,
    status: String,
    link_stats: HashMap<LinkID, LinkStats>,
    /// Per-pipe series — present only when the network has multi-pipe links.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pipe_stats: Option<HashMap<LinkID, Vec<PipeStats>>>,
    /// Variant metadata — `#[serde(default)]` so pre-patch stored results load.
    #[serde(default)]
    scenario_name: String,
    #[serde(default)]
    patches_applied: Vec<String>,
    #[serde(default)]
    warnings: Vec<String>,
    #[serde(default)]
    run_at_unix: u64,
}

/// Header-level view of one stored result — the `/runs` listing.
#[derive(Serialize)]
struct RunSummary {
    scenario: String,
    events_processed: usize,
    vehicles_count: usize,
    patches_applied: Vec<String>,
    run_at_unix: u64,
}

/// Patch-library listing entry.
#[derive(Serialize)]
struct PatchSummary {
    name: String,
    description: String,
    n_ops: usize,
}

#[derive(Serialize)]
struct ProjectSummary {
    name: String,
    path: String,
}

#[derive(Serialize)]
struct ProjectDetails {
    name: String,
    scenarios: Vec<String>,
}

#[derive(Serialize, Deserialize)]
struct ProjectSource {
    network: serde_json::Value,
    scenarios: HashMap<String, ScenarioDefinition>,
}

#[derive(Serialize, Deserialize)]
struct ScenarioDefinition {
    demand: serde_json::Value,
    config: serde_json::Value,
    /// Ordered patch names composing this variant (applied on top of the
    /// project's base network/demand). Default empty — old files load.
    #[serde(default)]
    patches: Vec<String>,
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Reject project/scenario names that could escape the scenarios directory
/// (path traversal): only simple file-name characters are allowed.
fn validate_name(name: &str) -> Result<(), AppError> {
    let ok = !name.is_empty()
        && name.len() <= 128
        && !name.contains("..")
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | ' ' | '.'));
    if ok {
        Ok(())
    } else {
        Err(AppError::BadRequest(format!("Invalid name '{}'", name)))
    }
}

/// Validate an optional time-bin width: must be a finite value ≥ 1 second
/// (tiny values would drive unbounded per-link allocations).
fn checked_bin_size(bin_size: Option<f64>) -> Result<f64, AppError> {
    let b = bin_size.unwrap_or(60.0);
    if b.is_finite() && b >= 1.0 {
        Ok(b)
    } else {
        Err(AppError::BadRequest("bin_size must be a finite value >= 1 second".to_string()))
    }
}

/// Run CPU-bound work on the blocking thread pool so simulation/analysis
/// requests do not starve the async runtime.
async fn blocking<T, F>(f: F) -> Result<T, AppError>
where
    T: Send + 'static,
    F: FnOnce() -> Result<T, AppError> + Send + 'static,
{
    tokio::task::spawn_blocking(f)
        .await
        .map_err(|e| AppError::Internal(format!("Background task failed: {}", e)))?
}

fn read_json_file(path: &PathBuf) -> Result<serde_json::Value, AppError> {
    let file = fs::File::open(path)?;
    let val = serde_json::from_reader(file)?;
    Ok(val)
}

fn read_scenario_def(path: &PathBuf) -> Result<ScenarioDefinition, AppError> {
    let file = fs::File::open(path)?;
    let def = serde_json::from_reader(file)?;
    Ok(def)
}

/// Resolve a project + scenario name to (demand, config, patch names).
fn load_scenario_inputs(
    project: &str,
    scenario_name: &str,
) -> Result<(serde_json::Value, serde_json::Value, Vec<String>), AppError> {
    validate_name(project)?;
    validate_name(scenario_name)?;
    let base_path = PathBuf::from("scenarios").join(project);
    if scenario_name == "Default" {
        Ok((
            read_json_file(&base_path.join("demand.json"))?,
            read_json_file(&base_path.join("config.json"))?,
            Vec::new(),
        ))
    } else {
        let path = base_path.join("scenarios").join(format!("{}.json", scenario_name));
        if !path.exists() {
            return Err(AppError::NotFound(format!("Scenario '{}' not found", scenario_name)));
        }
        let def = read_scenario_def(&path)?;
        Ok((def.demand, def.config, def.patches))
    }
}

fn patch_path(project: &str, name: &str) -> PathBuf {
    PathBuf::from("scenarios")
        .join(project)
        .join("patches")
        .join(format!("{}.json", name))
}

/// Load the named patches of a project, in order.
fn load_patches(project: &str, names: &[String]) -> Result<Vec<Patch>, AppError> {
    names
        .iter()
        .map(|n| {
            validate_name(n)?;
            let path = patch_path(project, n);
            if !path.exists() {
                return Err(AppError::NotFound(format!("Patch '{}' not found", n)));
            }
            let file = fs::File::open(&path)?;
            serde_json::from_reader(file)
                .map_err(|e| AppError::BadRequest(format!("Invalid patch '{}': {}", n, e)))
        })
        .collect()
}

fn unix_now() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0)
}

/// Variant metadata produced alongside a run.
struct RunMeta {
    events: usize,
    patches_applied: Vec<String>,
    warnings: Vec<String>,
}

/// Compile the scenario (base + patches), optionally assign demand, and run
/// the simulation to completion. Shared by every analysis endpoint.
/// `patch_override`, when present, replaces the variant's saved patch list
/// (previews / calibration working set).
fn compile_and_run(
    project: &str,
    scenario_name: Option<&str>,
    assignment: bool,
    patch_override: Option<&[String]>,
) -> Result<(Simulation, RunMeta), AppError> {
    let scenario_name = scenario_name
        .ok_or_else(|| AppError::BadRequest("scenario_name is required".to_string()))?;
    let network_path = PathBuf::from("scenarios").join(project).join("network.geojson");
    let (demand_val, config_val, saved_patches) = load_scenario_inputs(project, scenario_name)?;

    let patch_names: Vec<String> = match patch_override {
        Some(names) => names.to_vec(),
        None => saved_patches,
    };
    let patches = load_patches(project, &patch_names)?;

    let network_val = read_json_file(&network_path)?;
    let (mut scenario, warnings) =
        compile_with_patches(network_val, demand_val, config_val, &patches)
            .map_err(|e| AppError::BadRequest(format!("Failed to compile scenario: {}", e)))?;

    scenario
        .validate()
        .map_err(|e| AppError::BadRequest(format!("Invalid scenario: {}", e)))?;

    if assignment {
        assign_demand(&mut scenario)
            .map_err(|e| AppError::BadRequest(format!("Assignment failed: {}", e)))?;
    }

    let mut sim = Simulation::new(scenario);
    let events = sim
        .run()
        .map_err(|e| AppError::Internal(format!("Simulation failed: {}", e)))?;
    Ok((
        sim,
        RunMeta {
            events,
            patches_applied: patch_names,
            warnings,
        },
    ))
}

// ── Handlers ──────────────────────────────────────────────────────────────────

async fn list_projects() -> Json<Vec<ProjectSummary>> {
    let mut projects = Vec::new();
    let base_path = PathBuf::from("scenarios");

    if let Ok(entries) = fs::read_dir(base_path) {
        for entry in entries.flatten() {
            if let Ok(ft) = entry.file_type() {
                if ft.is_dir() {
                    if let Ok(name) = entry.file_name().into_string() {
                        projects.push(ProjectSummary {
                            name: name.clone(),
                            path: format!("scenarios/{}", name),
                        });
                    }
                }
            }
        }
    }
    Json(projects)
}

async fn get_project_details(Path(name): Path<String>) -> Result<Json<ProjectDetails>, AppError> {
    validate_name(&name)?;
    let base_path = PathBuf::from("scenarios").join(&name);
    let mut scenarios = Vec::new();

    if base_path.join("demand.json").exists() {
        scenarios.push("Default".to_string());
    }
    if let Ok(entries) = fs::read_dir(base_path.join("scenarios")) {
        for entry in entries.flatten() {
            if let Ok(fname) = entry.file_name().into_string() {
                if fname.ends_with(".json") {
                    scenarios.push(fname.replace(".json", ""));
                }
            }
        }
    }
    Ok(Json(ProjectDetails { name, scenarios }))
}

async fn get_project_source(Path(name): Path<String>) -> Result<Json<ProjectSource>, AppError> {
    validate_name(&name)?;
    let base_path = PathBuf::from("scenarios").join(&name);

    let network = read_json_file(&base_path.join("network.geojson"))?;

    let mut scenarios_map = HashMap::new();

    if base_path.join("demand.json").exists() {
        scenarios_map.insert(
            "Default".to_string(),
            ScenarioDefinition {
                demand: read_json_file(&base_path.join("demand.json"))?,
                config: read_json_file(&base_path.join("config.json"))?,
                patches: Vec::new(),
            },
        );
    }

    if let Ok(entries) = fs::read_dir(base_path.join("scenarios")) {
        for entry in entries.flatten() {
            if let Ok(fname) = entry.file_name().into_string() {
                if fname.ends_with(".json") {
                    let scenario_name = fname.replace(".json", "");
                    let path = base_path.join("scenarios").join(&fname);
                    if let Ok(def) = read_scenario_def(&path) {
                        scenarios_map.insert(scenario_name, def);
                    }
                }
            }
        }
    }

    Ok(Json(ProjectSource { network, scenarios: scenarios_map }))
}

async fn get_scenario_content(
    Path((project, scenario)): Path<(String, String)>,
) -> Result<Json<ScenarioDefinition>, AppError> {
    validate_name(&project)?;
    validate_name(&scenario)?;
    let base_path = PathBuf::from("scenarios").join(&project);

    let def = if scenario == "Default" {
        ScenarioDefinition {
            demand: read_json_file(&base_path.join("demand.json"))?,
            config: read_json_file(&base_path.join("config.json"))?,
            patches: Vec::new(),
        }
    } else {
        let path = base_path.join("scenarios").join(format!("{}.json", scenario));
        if !path.exists() {
            return Err(AppError::NotFound(format!("Scenario '{}' not found", scenario)));
        }
        read_scenario_def(&path)?
    };

    Ok(Json(def))
}

async fn save_scenario_content(
    Path((project, scenario)): Path<(String, String)>,
    Json(payload): Json<ScenarioDefinition>,
) -> Result<StatusCode, AppError> {
    validate_name(&project)?;
    validate_name(&scenario)?;
    let base_path = PathBuf::from("scenarios").join(&project);

    if scenario == "Default" {
        let mut file = BufWriter::new(fs::File::create(base_path.join("demand.json"))?);
        serde_json::to_writer_pretty(&mut file, &payload.demand)?;
        file.flush()?;
        let mut file = BufWriter::new(fs::File::create(base_path.join("config.json"))?);
        serde_json::to_writer_pretty(&mut file, &payload.config)?;
        file.flush()?;
    } else {
        let scenarios_dir = base_path.join("scenarios");
        fs::create_dir_all(&scenarios_dir)?;
        let mut file =
            BufWriter::new(fs::File::create(scenarios_dir.join(format!("{}.json", scenario)))?);
        serde_json::to_writer_pretty(&mut file, &payload)?;
        file.flush()?;
    }
    Ok(StatusCode::OK)
}

async fn create_scenario(
    Path(project): Path<String>,
    Json(name): Json<String>,
) -> Result<StatusCode, AppError> {
    validate_name(&project)?;
    validate_name(&name)?;
    let scenarios_dir = PathBuf::from("scenarios").join(&project).join("scenarios");
    fs::create_dir_all(&scenarios_dir)?;

    let path = scenarios_dir.join(format!("{}.json", name));
    if path.exists() {
        return Err(AppError::BadRequest(format!("Scenario '{}' already exists", name)));
    }

    let empty_def = ScenarioDefinition {
        demand: serde_json::json!([]),
        config: serde_json::json!({ "duration": 3600 }),
        patches: Vec::new(),
    };
    let mut file = BufWriter::new(fs::File::create(path)?);
    serde_json::to_writer_pretty(&mut file, &empty_def)?;
    file.flush()?;

    Ok(StatusCode::CREATED)
}

async fn get_project_results(Path(name): Path<String>) -> Result<Json<RunResponse>, AppError> {
    validate_name(&name)?;
    let path = PathBuf::from("scenarios")
        .join(&name)
        .join("results")
        .join("last_run.json");

    if !path.exists() {
        return Err(AppError::NotFound("No results found".to_string()));
    }

    let file = fs::File::open(&path)?;
    let results: RunResponse = serde_json::from_reader(file)
        .map_err(|e| AppError::Internal(format!("Failed to parse results: {}", e)))?;

    Ok(Json(results))
}

async fn run_simulation(Json(payload): Json<RunRequest>) -> Result<Json<RunResponse>, AppError> {
    let response = blocking(move || {
        let start = std::time::Instant::now();

        let scenario_name = payload
            .scenario_name
            .clone()
            .unwrap_or_else(|| "Default".to_string());
        let (sim, meta) = compile_and_run(
            &payload.project_name,
            Some(&scenario_name),
            payload.assignment,
            payload.patches.as_deref(),
        )?;

        let duration_ms = start.elapsed().as_millis();
        let link_stats = compute_link_stats(&sim.scenario, 60.0);
        let has_pipes = sim.scenario.links.iter().any(|l| l.pipes.len() > 1);
        let pipe_stats = has_pipes.then(|| compute_pipe_stats(&sim.scenario, 60.0));

        let response = RunResponse {
            events_processed: meta.events,
            duration_ms,
            vehicles_count: sim.scenario.vehicles.len(),
            status: "success".to_string(),
            link_stats,
            pipe_stats,
            scenario_name: scenario_name.clone(),
            patches_applied: meta.patches_applied,
            warnings: meta.warnings,
            run_at_unix: unix_now(),
        };

        // Persist results: per-scenario file + last_run.json (back-compat).
        // Preview runs with a patch override are NOT persisted — they don't
        // represent the saved variant.
        if payload.patches.is_none() {
            let results_dir = PathBuf::from("scenarios")
                .join(&payload.project_name)
                .join("results");
            fs::create_dir_all(&results_dir)?;
            // BufWriter is load-bearing: to_writer_pretty emits many small
            // writes, and unbuffered syscalls made /run take seconds. The
            // explicit flush is equally load-bearing: BufWriter's drop
            // swallows flush errors, which would report success on a
            // truncated file.
            let mut file = BufWriter::new(fs::File::create(results_dir.join("last_run.json"))?);
            serde_json::to_writer_pretty(&mut file, &response)?;
            file.flush()?;
            let mut file = BufWriter::new(fs::File::create(
                results_dir.join(format!("{}.json", scenario_name)),
            )?);
            serde_json::to_writer_pretty(&mut file, &response)?;
            file.flush()?;
        }

        Ok(response)
    })
    .await?;

    Ok(Json(response))
}

/// List stored per-scenario results (header fields only).
async fn list_runs(Path(name): Path<String>) -> Result<Json<Vec<RunSummary>>, AppError> {
    validate_name(&name)?;
    let results_dir = PathBuf::from("scenarios").join(&name).join("results");
    let mut runs = Vec::new();
    if let Ok(entries) = fs::read_dir(&results_dir) {
        for entry in entries.flatten() {
            let Ok(fname) = entry.file_name().into_string() else {
                continue;
            };
            let Some(scenario) = fname.strip_suffix(".json") else {
                continue;
            };
            if scenario == "last_run" {
                continue; // legacy alias of the most recent run
            }
            let Ok(file) = fs::File::open(entry.path()) else {
                continue;
            };
            let Ok(r) = serde_json::from_reader::<_, RunResponse>(file) else {
                continue; // unreadable/foreign file — skip, don't fail the listing
            };
            runs.push(RunSummary {
                scenario: scenario.to_string(),
                events_processed: r.events_processed,
                vehicles_count: r.vehicles_count,
                patches_applied: r.patches_applied,
                run_at_unix: r.run_at_unix,
            });
        }
    }
    runs.sort_by(|a, b| b.run_at_unix.cmp(&a.run_at_unix));
    Ok(Json(runs))
}

/// Full stored result of one scenario/variant.
async fn get_scenario_results(
    Path((project, scenario)): Path<(String, String)>,
) -> Result<Json<RunResponse>, AppError> {
    validate_name(&project)?;
    validate_name(&scenario)?;
    let path = PathBuf::from("scenarios")
        .join(&project)
        .join("results")
        .join(format!("{}.json", scenario));
    if !path.exists() {
        return Err(AppError::NotFound(format!(
            "No stored results for scenario '{}'",
            scenario
        )));
    }
    let file = fs::File::open(&path)?;
    let results: RunResponse = serde_json::from_reader(file)
        .map_err(|e| AppError::Internal(format!("Failed to parse results: {}", e)))?;
    Ok(Json(results))
}

// ── Patch CRUD ───────────────────────────────────────────────────────────────

async fn list_patches(Path(name): Path<String>) -> Result<Json<Vec<PatchSummary>>, AppError> {
    validate_name(&name)?;
    let dir = PathBuf::from("scenarios").join(&name).join("patches");
    let mut patches = Vec::new();
    if let Ok(entries) = fs::read_dir(&dir) {
        for entry in entries.flatten() {
            let Ok(fname) = entry.file_name().into_string() else {
                continue;
            };
            if !fname.ends_with(".json") {
                continue;
            }
            let Ok(file) = fs::File::open(entry.path()) else {
                continue;
            };
            if let Ok(p) = serde_json::from_reader::<_, Patch>(file) {
                patches.push(PatchSummary {
                    name: fname.trim_end_matches(".json").to_string(),
                    description: p.description,
                    n_ops: p.ops.len(),
                });
            }
        }
    }
    patches.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(Json(patches))
}

async fn get_patch(
    Path((project, patch)): Path<(String, String)>,
) -> Result<Json<Patch>, AppError> {
    validate_name(&project)?;
    validate_name(&patch)?;
    let path = patch_path(&project, &patch);
    if !path.exists() {
        return Err(AppError::NotFound(format!("Patch '{}' not found", patch)));
    }
    let file = fs::File::open(&path)?;
    let p: Patch = serde_json::from_reader(file)
        .map_err(|e| AppError::Internal(format!("Failed to parse patch: {}", e)))?;
    Ok(Json(p))
}

/// Upsert a patch. The filename is canonical: the payload's `name` is
/// overwritten with the path segment.
async fn save_patch(
    Path((project, patch)): Path<(String, String)>,
    Json(mut payload): Json<Patch>,
) -> Result<StatusCode, AppError> {
    validate_name(&project)?;
    validate_name(&patch)?;
    if !PathBuf::from("scenarios").join(&project).is_dir() {
        return Err(AppError::NotFound(format!("Project '{}' not found", project)));
    }
    payload.name = patch.clone();
    let dir = PathBuf::from("scenarios").join(&project).join("patches");
    fs::create_dir_all(&dir)?;
    let mut file = BufWriter::new(fs::File::create(patch_path(&project, &patch))?);
    serde_json::to_writer_pretty(&mut file, &payload)?;
    file.flush()?;
    Ok(StatusCode::OK)
}

async fn delete_patch(
    Path((project, patch)): Path<(String, String)>,
) -> Result<StatusCode, AppError> {
    validate_name(&project)?;
    validate_name(&patch)?;
    let path = patch_path(&project, &patch);
    if !path.exists() {
        return Err(AppError::NotFound(format!("Patch '{}' not found", patch)));
    }
    fs::remove_file(&path)?;
    Ok(StatusCode::OK)
}

// ── Observed data ────────────────────────────────────────────────────────────

async fn get_observed(Path(name): Path<String>) -> Result<Json<Vec<ObservedRecord>>, AppError> {
    validate_name(&name)?;
    let path = PathBuf::from("scenarios").join(&name).join("observed.json");
    if !path.exists() {
        return Err(AppError::NotFound(format!(
            "No observed data for project '{}' — POST an array of \
             {{link_id, t_start, t_end, flow_veh_h?, speed_kmh?, label?}}",
            name
        )));
    }
    let file = fs::File::open(&path)?;
    let records: Vec<ObservedRecord> = serde_json::from_reader(file)
        .map_err(|e| AppError::Internal(format!("Invalid observed.json: {}", e)))?;
    Ok(Json(records))
}

async fn save_observed(
    Path(name): Path<String>,
    Json(records): Json<Vec<ObservedRecord>>,
) -> Result<StatusCode, AppError> {
    validate_name(&name)?;
    let base = PathBuf::from("scenarios").join(&name);
    if !base.is_dir() {
        return Err(AppError::NotFound(format!("Project '{}' not found", name)));
    }
    for (i, r) in records.iter().enumerate() {
        if !(r.t_end > r.t_start) {
            return Err(AppError::BadRequest(format!(
                "Observed record {}: t_end must be > t_start",
                i
            )));
        }
        if r.flow_veh_h.is_none() && r.speed_kmh.is_none() {
            return Err(AppError::BadRequest(format!(
                "Observed record {}: at least one of flow_veh_h / speed_kmh is required",
                i
            )));
        }
    }
    let mut file = BufWriter::new(fs::File::create(base.join("observed.json"))?);
    serde_json::to_writer_pretty(&mut file, &records)?;
    file.flush()?;
    Ok(StatusCode::OK)
}

async fn export_diagnostics_xml_handler(
    Json(payload): Json<DiagnosticsRequest>,
) -> Result<Response, AppError> {
    let scenario_name = payload
        .scenario_name
        .clone()
        .ok_or_else(|| AppError::BadRequest("scenario_name is required".to_string()))?;
    let bin_size = checked_bin_size(payload.bin_size)?;
    let filename = format!("{}_{}_diagnostics.xml", payload.project_name, scenario_name);

    let xml = blocking(move || {
        let (sim, _) = compile_and_run(
            &payload.project_name,
            Some(&scenario_name),
            payload.assignment,
            payload.patches.as_deref(),
        )?;
        let max_traj = payload.max_trajectories.unwrap_or(500);
        let report = run_diagnostics(&sim.scenario, bin_size, max_traj);
        Ok(export_to_xml(&report, &sim.scenario, &payload.project_name, &scenario_name))
    })
    .await?;
    Ok(axum::response::Response::builder()
        .header(axum::http::header::CONTENT_TYPE, "application/xml; charset=utf-8")
        .header(
            axum::http::header::CONTENT_DISPOSITION,
            format!("attachment; filename=\"{}\"", filename),
        )
        .body(axum::body::Body::from(xml))
        .unwrap())
}

async fn run_diagnostics_handler(
    Json(payload): Json<DiagnosticsRequest>,
) -> Result<Json<DiagnosticsReport>, AppError> {
    let bin_size = checked_bin_size(payload.bin_size)?;
    let report = blocking(move || {
        let (sim, _) = compile_and_run(
            &payload.project_name,
            payload.scenario_name.as_deref(),
            payload.assignment,
            payload.patches.as_deref(),
        )?;
        let max_traj = payload.max_trajectories.unwrap_or(500);
        Ok(run_diagnostics(&sim.scenario, bin_size, max_traj))
    })
    .await?;

    Ok(Json(report))
}

// ── Verification ─────────────────────────────────────────────────────────────

#[derive(Deserialize)]
struct VerificationRequest {
    project_name: String,
    scenario_name: Option<String>,
    assignment: bool,
    #[serde(default)]
    patches: Option<Vec<String>>,
    /// Time-bin width in seconds for FD/conservation checks (default: 60).
    bin_size: Option<f64>,
}

/// Per-element realism checks against LWR theory (links, nodes, entries).
async fn run_verification_handler(
    Json(payload): Json<VerificationRequest>,
) -> Result<Json<VerificationReport>, AppError> {
    let bin_size = checked_bin_size(payload.bin_size)?;
    let report = blocking(move || {
        let (sim, _) = compile_and_run(
            &payload.project_name,
            payload.scenario_name.as_deref(),
            payload.assignment,
            payload.patches.as_deref(),
        )?;
        Ok(verify_scenario(&sim.scenario, bin_size))
    })
    .await?;
    Ok(Json(report))
}

/// Analytic benchmark suite — canonical LWR cases with closed-form solutions.
/// Independent of any project: validates the engine/methodology itself.
async fn run_benchmarks_handler() -> Result<Json<Vec<BenchmarkResult>>, AppError> {
    let results = blocking(|| {
        run_benchmarks().map_err(|e| AppError::Internal(format!("Benchmark run failed: {}", e)))
    })
    .await?;
    Ok(Json(results))
}

#[derive(Deserialize)]
struct ReferenceRequest {
    project_name: String,
    scenario_name: Option<String>,
    assignment: bool,
    #[serde(default)]
    patches: Option<Vec<String>>,
    /// Observed records; when omitted, `scenarios/<project>/observed.json` is used.
    observed: Option<Vec<ObservedRecord>>,
}

/// Compare simulation output with observed measurements (GEH / speed error).
async fn run_reference_handler(
    Json(payload): Json<ReferenceRequest>,
) -> Result<Json<ReferenceReport>, AppError> {
    validate_name(&payload.project_name)?;

    let records = match payload.observed {
        Some(recs) if !recs.is_empty() => recs,
        _ => {
            let path = PathBuf::from("scenarios")
                .join(&payload.project_name)
                .join("observed.json");
            if !path.exists() {
                return Err(AppError::NotFound(format!(
                    "No observed data: provide 'observed' in the request or create scenarios/{}/observed.json \
                     (array of {{link_id, t_start, t_end, flow_veh_h?, speed_kmh?, label?}})",
                    payload.project_name
                )));
            }
            let file = fs::File::open(&path)?;
            serde_json::from_reader(file)
                .map_err(|e| AppError::BadRequest(format!("Invalid observed.json: {}", e)))?
        }
    };

    let report = blocking(move || {
        let (sim, _) = compile_and_run(
            &payload.project_name,
            payload.scenario_name.as_deref(),
            payload.assignment,
            payload.patches.as_deref(),
        )?;
        Ok(compare_with_observations(&sim.scenario, &records))
    })
    .await?;
    Ok(Json(report))
}

// ── XT Analysis ──────────────────────────────────────────────────────────────

#[derive(Deserialize)]
struct XTAnalysisRequest {
    project_name:  String,
    scenario_name: Option<String>,
    assignment:    bool,
    #[serde(default)]
    patches: Option<Vec<String>>,
    /// Spatial resolution in metres (default 50).
    dx_m: Option<f64>,
    /// Temporal resolution in seconds (default 60).
    dt_s: Option<f64>,
}

async fn run_xt_analysis_handler(
    Json(payload): Json<XTAnalysisRequest>,
) -> Result<Json<HashMap<LinkID, XTDiagram>>, AppError> {
    let params = XTParams {
        dx_m: payload.dx_m.unwrap_or(50.0),
        dt_s: payload.dt_s.unwrap_or(60.0),
    };
    if !(params.dx_m.is_finite() && params.dx_m >= 1.0 && params.dt_s.is_finite() && params.dt_s >= 1.0) {
        return Err(AppError::BadRequest("dx_m and dt_s must be finite values >= 1".to_string()));
    }

    let diagrams = blocking(move || {
        let (sim, _) = compile_and_run(
            &payload.project_name,
            payload.scenario_name.as_deref(),
            payload.assignment,
            payload.patches.as_deref(),
        )?;
        Ok(compute_xt_diagrams(&sim.scenario, &params))
    })
    .await?;
    Ok(Json(diagrams))
}

// ── Entry point ───────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() {
    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods([Method::GET, Method::POST, Method::PUT, Method::DELETE])
        .allow_headers(Any);

    let app = Router::new()
        .route("/", get(|| async { "Stream Core Server Running" }))
        .route("/projects", get(list_projects))
        .route("/projects/:name", get(get_project_details))
        .route("/projects/:name/source", get(get_project_source))
        .route("/projects/:name/results", get(get_project_results))
        .route("/projects/:name/results/:s_name", get(get_scenario_results))
        .route("/projects/:name/runs", get(list_runs))
        .route("/projects/:name/scenarios", post(create_scenario))
        .route(
            "/projects/:name/scenarios/:s_name",
            get(get_scenario_content).post(save_scenario_content),
        )
        .route("/projects/:name/patches", get(list_patches))
        .route(
            "/projects/:name/patches/:p_name",
            get(get_patch).post(save_patch).delete(delete_patch),
        )
        .route(
            "/projects/:name/observed",
            get(get_observed).post(save_observed),
        )
        .route("/run", post(run_simulation))
        .route("/diagnostics", post(run_diagnostics_handler))
        .route("/diagnostics/export", post(export_diagnostics_xml_handler))
        .route("/xt-analysis", post(run_xt_analysis_handler))
        .route("/verification", post(run_verification_handler))
        .route("/verification/benchmarks", post(run_benchmarks_handler))
        .route("/verification/reference", post(run_reference_handler))
        .layer(cors);

    let addr = SocketAddr::from(([127, 0, 0, 1], 8080));
    println!("Listening on {}", addr);
    let listener = tokio::net::TcpListener::bind(addr)
        .await
        .expect("Failed to bind to port 8080");
    axum::serve(listener, app).await.expect("Server error");
}
