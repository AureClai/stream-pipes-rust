mod control;
mod convert;

use control::{PyCommand, PyControlView, PyControllerAdapter};
use convert::{parse_from_py, py_to_json, serialize_to_py};
use pyo3::exceptions::PyRuntimeError;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use serde::Serialize;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use stream_core::api::{
    check as core_check, collect, compile as compile_inputs, Progress, RunOptions, ScenarioInputs,
};
use stream_core::assignment::assign_demand;
use stream_core::benchmarks::run_benchmarks as core_run_benchmarks;
use stream_core::patch::Patch;
use stream_core::reference::{compare_with_observations, ObservedRecord};
use stream_core::simulation::{MergeRule, Simulation as CoreSimulation};
use stream_core::validation::Validate;
use stream_core::verification::verify_scenario;

fn anyhow_err(e: anyhow::Error) -> PyErr {
    PyRuntimeError::new_err(format!("{e:#}"))
}

fn load_patches_py(
    patches: Option<&Bound<'_, PyAny>>,
    dir: Option<&str>,
) -> PyResult<Vec<Patch>> {
    let Some(p_any) = patches else {
        return Ok(Vec::new());
    };
    if p_any.is_none() {
        return Ok(Vec::new());
    }
    let items: Vec<Bound<'_, PyAny>> = p_any.extract()?;
    let mut out = Vec::with_capacity(items.len());
    for item in items {
        if let Ok(name_or_path) = item.extract::<String>() {
            let p = PathBuf::from(&name_or_path);
            let resolved = if p.exists() {
                p
            } else if let Some(d) = dir {
                let d_path = PathBuf::from(d);
                let c1 = d_path.join("patches").join(&name_or_path);
                let c2 = d_path.join("patches").join(format!("{name_or_path}.json"));
                if c1.exists() {
                    c1
                } else if c2.exists() {
                    c2
                } else {
                    return Err(PyValueError::new_err(format!(
                        "patch '{name_or_path}' not found"
                    )));
                }
            } else {
                return Err(PyValueError::new_err(format!(
                    "patch file '{name_or_path}' not found"
                )));
            };
            let text = std::fs::read_to_string(&resolved)
                .map_err(|e| PyRuntimeError::new_err(format!("cannot read {}: {e}", resolved.display())))?;
            let patch: Patch = serde_json::from_str(&text)
                .map_err(|e| PyValueError::new_err(format!("invalid patch {}: {e}", resolved.display())))?;
            out.push(patch);
        } else {
            out.push(parse_from_py::<Patch>(&item)?);
        }
    }
    Ok(out)
}

fn build_scenario_inputs(
    inputs: Option<&Bound<'_, PyAny>>,
    dir: Option<&str>,
    network: Option<&Bound<'_, PyAny>>,
    demand: Option<&Bound<'_, PyAny>>,
    config: Option<&Bound<'_, PyAny>>,
    signals: Option<&Bound<'_, PyAny>>,
    patches: Option<&Bound<'_, PyAny>>,
) -> PyResult<ScenarioInputs> {
    if let Some(d) = dir {
        let mut si = ScenarioInputs::from_dir(d, &[]).map_err(anyhow_err)?;
        let extra = load_patches_py(patches, Some(d))?;
        si.patches = extra;
        if let Some(n) = network {
            si.network = py_to_json(n)?;
        }
        if let Some(dem) = demand {
            si.demand = py_to_json(dem)?;
        }
        if let Some(cfg) = config {
            si.config = py_to_json(cfg)?;
        }
        if let Some(sig) = signals {
            if !sig.is_none() {
                si.signals = Some(py_to_json(sig)?);
            }
        }
        return Ok(si);
    }

    if let Some(inp) = inputs {
        if let Ok(path_str) = inp.extract::<String>() {
            return build_scenario_inputs(None, Some(&path_str), network, demand, config, signals, patches);
        }
        let mut si = parse_from_py::<ScenarioInputs>(inp)?;
        let extra = load_patches_py(patches, None)?;
        if !extra.is_empty() {
            si.patches.extend(extra);
        }
        return Ok(si);
    }

    let net_val = py_to_json(
        network.ok_or_else(|| PyValueError::new_err("provide `dir`, `inputs`, or `network`/`demand`/`config`"))?,
    )?;
    let dem_val = py_to_json(
        demand.ok_or_else(|| PyValueError::new_err("`demand` is required when `network` is provided"))?,
    )?;
    let cfg_val = py_to_json(
        config.ok_or_else(|| PyValueError::new_err("`config` is required when `network` is provided"))?,
    )?;
    let sig_val = match signals {
        Some(s) if !s.is_none() => Some(py_to_json(s)?),
        _ => None,
    };
    let loaded_patches = load_patches_py(patches, None)?;
    Ok(ScenarioInputs {
        network: net_val,
        demand: dem_val,
        config: cfg_val,
        signals: sig_val,
        patches: loaded_patches,
    })
}

fn attach_controllers(
    sim: &mut CoreSimulation,
    controllers: Option<&Bound<'_, PyAny>>,
    py: Python<'_>,
) -> PyResult<Arc<Mutex<Option<PyErr>>>> {
    let shared_err = Arc::new(Mutex::new(None));
    if let Some(ctrl_seq) = controllers {
        if !ctrl_seq.is_none() {
            let objs: Vec<PyObject> = ctrl_seq.extract()?;
            for obj in objs {
                let bound = obj.bind(py);
                if !bound.hasattr("start")? || !bound.hasattr("wake")? {
                    return Err(PyValueError::new_err(
                        "each controller must implement `start(view)` and `wake(view)`",
                    ));
                }
                sim.add_controller(Box::new(PyControllerAdapter::new(
                    obj,
                    Arc::clone(&shared_err),
                )));
            }
        }
    }
    Ok(shared_err)
}

fn progress_to_py(py: Python<'_>, p: Progress) -> PyResult<PyObject> {
    let dict = PyDict::new_bound(py);
    match p {
        Progress::Compiled { links, vehicles } => {
            dict.set_item("stage", "compiled")?;
            dict.set_item("links", links)?;
            dict.set_item("vehicles", vehicles)?;
        }
        Progress::Simulating {
            sim_time,
            start_time,
            end_time,
            events,
        } => {
            dict.set_item("stage", "simulating")?;
            dict.set_item("sim_time", sim_time)?;
            dict.set_item("start_time", start_time)?;
            dict.set_item("end_time", end_time)?;
            dict.set_item("events", events)?;
        }
        Progress::Aggregating => {
            dict.set_item("stage", "aggregating")?;
        }
    }
    Ok(dict.into_py(py))
}

/// Interactive handle to a compiled [`Simulation`] in Python.
#[pyclass(name = "Simulation")]
pub struct PySimulation {
    inner: CoreSimulation,
    warnings: Vec<String>,
    controller_error: Arc<Mutex<Option<PyErr>>>,
}

#[pymethods]
impl PySimulation {
    /// Build a `Simulation` from a scenario folder (`network.geojson`, `demand.json`, `config.json`, ...).
    #[staticmethod]
    #[pyo3(signature = (path, patches=None, assignment=true, merge=None, upstream_capacity=false))]
    fn from_dir(
        path: &str,
        patches: Option<&Bound<'_, PyAny>>,
        assignment: bool,
        merge: Option<&str>,
        upstream_capacity: bool,
    ) -> PyResult<Self> {
        let mut inputs = ScenarioInputs::from_dir(path, &[]).map_err(anyhow_err)?;
        inputs.patches = load_patches_py(patches, Some(path))?;
        Self::build_from_inputs(&inputs, assignment, merge, upstream_capacity)
    }

    /// Build a `Simulation` directly from in-memory Python dicts/lists (`network`, `demand`, `config`, ...).
    #[staticmethod]
    #[pyo3(signature = (network, demand, config, signals=None, patches=None, assignment=true, merge=None, upstream_capacity=false))]
    #[allow(clippy::too_many_arguments)]
    fn from_inputs(
        network: &Bound<'_, PyAny>,
        demand: &Bound<'_, PyAny>,
        config: &Bound<'_, PyAny>,
        signals: Option<&Bound<'_, PyAny>>,
        patches: Option<&Bound<'_, PyAny>>,
        assignment: bool,
        merge: Option<&str>,
        upstream_capacity: bool,
    ) -> PyResult<Self> {
        let inputs = build_scenario_inputs(
            None,
            None,
            Some(network),
            Some(demand),
            Some(config),
            signals,
            patches,
        )?;
        Self::build_from_inputs(&inputs, assignment, merge, upstream_capacity)
    }

    /// Attach a Python controller object implementing `start(view)` and `wake(view)`.
    fn add_controller(&mut self, py: Python<'_>, controller: PyObject) -> PyResult<()> {
        let bound = controller.bind(py);
        if !bound.hasattr("start")? || !bound.hasattr("wake")? {
            return Err(PyValueError::new_err(
                "controller must implement `start(view)` and `wake(view)`",
            ));
        }
        self.inner.add_controller(Box::new(PyControllerAdapter::new(
            controller,
            Arc::clone(&self.controller_error),
        )));
        Ok(())
    }

    /// Execute the simulation event loop and return the number of processed events.
    fn run(&mut self, py: Python<'_>) -> PyResult<usize> {
        let events = py
            .allow_threads(|| self.inner.run())
            .map_err(anyhow_err)?;
        if let Some(err) = self.controller_error.lock().unwrap().take() {
            return Err(err);
        }
        Ok(events)
    }

    /// Collect full [`RunOutput`] study KPIs, binned series, and optional vehicle trajectories.
    #[pyo3(signature = (events_processed=0, bin_size=60.0, keep_vehicles=false))]
    fn collect(
        &self,
        py: Python<'_>,
        events_processed: usize,
        bin_size: f64,
        keep_vehicles: bool,
    ) -> PyResult<PyObject> {
        if !(bin_size.is_finite() && bin_size >= 1.0) {
            return Err(PyValueError::new_err("bin_size must be a finite number >= 1"));
        }
        let opts = RunOptions {
            bin_size,
            keep_vehicles,
            ..Default::default()
        };
        let out = collect(
            self.inner.scenario.clone(),
            events_processed,
            self.warnings.clone(),
            &opts,
        );
        serialize_to_py(py, &out)
    }

    /// Run the built-in self-auditing verification apparatus and return `VerificationReport`.
    #[pyo3(signature = (bin_size=300.0))]
    fn verify(&self, py: Python<'_>, bin_size: f64) -> PyResult<PyObject> {
        let report = verify_scenario(&self.inner.scenario, bin_size);
        serialize_to_py(py, &report)
    }

    /// Compare simulated flows and speeds against observed detector records (`list[dict]`).
    fn compare_observations(
        &self,
        py: Python<'_>,
        observed: &Bound<'_, PyAny>,
    ) -> PyResult<PyObject> {
        let records: Vec<ObservedRecord> = parse_from_py(observed)?;
        let report = compare_with_observations(&self.inner.scenario, &records);
        serialize_to_py(py, &report)
    }

    /// Return the applied signal switch log (`list[dict]`).
    fn signal_log(&self, py: Python<'_>) -> PyResult<PyObject> {
        serialize_to_py(py, &self.inner.scenario.signal_log)
    }

    #[getter]
    fn current_time(&self) -> f64 {
        self.inner.current_time
    }

    #[getter]
    fn vehicles_count(&self) -> usize {
        self.inner.scenario.vehicles.len()
    }

    #[getter]
    fn links_count(&self) -> usize {
        self.inner.scenario.links.len()
    }

    #[getter]
    fn nodes_count(&self) -> usize {
        self.inner.scenario.nodes.len()
    }

    #[getter]
    fn warnings(&self) -> Vec<String> {
        self.warnings.clone()
    }
}

impl PySimulation {
    fn build_from_inputs(
        inputs: &ScenarioInputs,
        assignment: bool,
        merge: Option<&str>,
        upstream_capacity: bool,
    ) -> PyResult<Self> {
        let (mut scenario, warnings) = compile_inputs(inputs).map_err(anyhow_err)?;
        if assignment {
            assign_demand(&mut scenario).map_err(anyhow_err)?;
        }
        let merge_rule = match merge {
            Some(m) => MergeRule::from_config(&serde_json::json!({ "merge": m })).map_err(anyhow_err)?,
            None => MergeRule::from_config(&inputs.config).map_err(anyhow_err)?,
        };
        let mut inner = CoreSimulation::new(scenario);
        inner.merge_rule = merge_rule;
        inner.upstream_capacity = upstream_capacity;
        Ok(Self {
            inner,
            warnings,
            controller_error: Arc::new(Mutex::new(None)),
        })
    }
}

/// Compile and validate scenario inputs without simulating (`CheckReport` dict).
#[pyfunction]
#[pyo3(signature = (inputs=None, *, dir=None, network=None, demand=None, config=None, signals=None, patches=None))]
#[allow(clippy::too_many_arguments)]
fn check(
    py: Python<'_>,
    inputs: Option<&Bound<'_, PyAny>>,
    dir: Option<&str>,
    network: Option<&Bound<'_, PyAny>>,
    demand: Option<&Bound<'_, PyAny>>,
    config: Option<&Bound<'_, PyAny>>,
    signals: Option<&Bound<'_, PyAny>>,
    patches: Option<&Bound<'_, PyAny>>,
) -> PyResult<PyObject> {
    let si = build_scenario_inputs(inputs, dir, network, demand, config, signals, patches)?;
    let report = py.allow_threads(|| core_check(&si));
    serialize_to_py(py, &report)
}

fn prepare_simulation(
    py: Python<'_>,
    inputs: &ScenarioInputs,
    assignment: bool,
    merge: Option<&str>,
    upstream_capacity: bool,
    controllers: Option<&Bound<'_, PyAny>>,
) -> PyResult<(CoreSimulation, Vec<String>, Arc<Mutex<Option<PyErr>>>)> {
    let (mut scenario, warnings) = compile_inputs(inputs).map_err(anyhow_err)?;
    if assignment {
        assign_demand(&mut scenario).map_err(anyhow_err)?;
    }
    scenario.validate().map_err(anyhow_err)?;
    let merge_rule = match merge {
        Some(m) => MergeRule::from_config(&serde_json::json!({ "merge": m })).map_err(anyhow_err)?,
        None => MergeRule::from_config(&inputs.config).map_err(anyhow_err)?,
    };
    let mut sim = CoreSimulation::new(scenario);
    sim.merge_rule = merge_rule;
    sim.upstream_capacity = upstream_capacity;
    let ctrl_err = attach_controllers(&mut sim, controllers, py)?;
    Ok((sim, warnings, ctrl_err))
}

fn execute_sim(
    py: Python<'_>,
    sim: &mut CoreSimulation,
    ctrl_err: &Arc<Mutex<Option<PyErr>>>,
    progress: Option<PyObject>,
) -> PyResult<usize> {
    let start_time = sim.scenario.start_time;
    let end_time = start_time + sim.scenario.duration;
    let cb_err: Arc<Mutex<Option<PyErr>>> = Arc::new(Mutex::new(None));
    let events = match progress {
        Some(cb) => {
            let compiled_evt = Progress::Compiled {
                links: sim.scenario.links.len(),
                vehicles: sim.scenario.vehicles.len(),
            };
            let py_evt = progress_to_py(py, compiled_evt)?;
            let keep_going = cb.call1(py, (py_evt,))?;
            if matches!(keep_going.extract::<bool>(py), Ok(false)) {
                return Err(PyRuntimeError::new_err("simulation cancelled"));
            }
            let cb_err_ref = Arc::clone(&cb_err);
            let ev = py
                .allow_threads(|| {
                    sim.run_with_progress(50_000, |sim_time, events| {
                        Python::with_gil(|py| {
                            let evt = Progress::Simulating {
                                sim_time,
                                start_time,
                                end_time,
                                events,
                            };
                            match progress_to_py(py, evt).and_then(|d| cb.call1(py, (d,))) {
                                Ok(ret) => !matches!(ret.extract::<bool>(py), Ok(false)),
                                Err(e) => {
                                    *cb_err_ref.lock().unwrap() = Some(e);
                                    false
                                }
                            }
                        })
                    })
                })
                .map_err(anyhow_err)?;
            ev
        }
        None => py.allow_threads(|| sim.run()).map_err(anyhow_err)?,
    };

    if let Some(err) = cb_err.lock().unwrap().take() {
        return Err(err);
    }
    if let Some(err) = ctrl_err.lock().unwrap().take() {
        return Err(err);
    }
    Ok(events)
}

/// Compile, assign, simulate, and return full [`RunOutput`] as a Python `dict`.
#[pyfunction]
#[pyo3(signature = (
    inputs=None,
    *,
    dir=None,
    network=None,
    demand=None,
    config=None,
    signals=None,
    patches=None,
    assignment=true,
    merge=None,
    upstream_capacity=false,
    bin_size=60.0,
    keep_vehicles=false,
    controllers=None,
    progress=None
))]
#[allow(clippy::too_many_arguments)]
fn run(
    py: Python<'_>,
    inputs: Option<&Bound<'_, PyAny>>,
    dir: Option<&str>,
    network: Option<&Bound<'_, PyAny>>,
    demand: Option<&Bound<'_, PyAny>>,
    config: Option<&Bound<'_, PyAny>>,
    signals: Option<&Bound<'_, PyAny>>,
    patches: Option<&Bound<'_, PyAny>>,
    assignment: bool,
    merge: Option<&str>,
    upstream_capacity: bool,
    bin_size: f64,
    keep_vehicles: bool,
    controllers: Option<&Bound<'_, PyAny>>,
    progress: Option<PyObject>,
) -> PyResult<PyObject> {
    if !(bin_size.is_finite() && bin_size >= 1.0) {
        return Err(PyValueError::new_err("bin_size must be a finite number >= 1"));
    }
    let si = build_scenario_inputs(inputs, dir, network, demand, config, signals, patches)?;
    let (mut sim, warnings, ctrl_err) =
        prepare_simulation(py, &si, assignment, merge, upstream_capacity, controllers)?;
    let events = execute_sim(py, &mut sim, &ctrl_err, progress)?;

    let opts = RunOptions {
        assignment,
        upstream_capacity,
        bin_size,
        keep_vehicles,
    };
    let out = py.allow_threads(|| collect(sim.scenario, events, warnings, &opts));
    serialize_to_py(py, &out)
}

#[derive(Serialize)]
struct PyVerifyReport {
    #[serde(flatten)]
    verification: stream_core::verification::VerificationReport,
    #[serde(skip_serializing_if = "Option::is_none")]
    benchmarks: Option<Vec<stream_core::benchmarks::BenchmarkResult>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    reference: Option<stream_core::reference::ReferenceReport>,
    warnings: Vec<String>,
}

/// Simulate a scenario and run the built-in self-auditing verification apparatus.
#[pyfunction]
#[pyo3(signature = (
    inputs=None,
    *,
    dir=None,
    network=None,
    demand=None,
    config=None,
    signals=None,
    patches=None,
    assignment=true,
    merge=None,
    upstream_capacity=false,
    bin_size=300.0,
    benchmarks=false,
    observed=None,
    controllers=None
))]
#[allow(clippy::too_many_arguments)]
fn verify(
    py: Python<'_>,
    inputs: Option<&Bound<'_, PyAny>>,
    dir: Option<&str>,
    network: Option<&Bound<'_, PyAny>>,
    demand: Option<&Bound<'_, PyAny>>,
    config: Option<&Bound<'_, PyAny>>,
    signals: Option<&Bound<'_, PyAny>>,
    patches: Option<&Bound<'_, PyAny>>,
    assignment: bool,
    merge: Option<&str>,
    upstream_capacity: bool,
    bin_size: f64,
    benchmarks: bool,
    observed: Option<&Bound<'_, PyAny>>,
    controllers: Option<&Bound<'_, PyAny>>,
) -> PyResult<PyObject> {
    if !(bin_size.is_finite() && bin_size >= 1.0) {
        return Err(PyValueError::new_err("bin_size must be a finite number >= 1"));
    }
    let si = build_scenario_inputs(inputs, dir, network, demand, config, signals, patches)?;
    let obs_records: Option<Vec<ObservedRecord>> = match observed {
        Some(obs) if !obs.is_none() => {
            if let Ok(path_str) = obs.extract::<String>() {
                let p = PathBuf::from(&path_str);
                let resolved = if p.exists() {
                    p
                } else if let Some(d) = dir {
                    PathBuf::from(d).join(&path_str)
                } else {
                    p
                };
                let text = std::fs::read_to_string(&resolved).map_err(|e| {
                    PyRuntimeError::new_err(format!("cannot read {}: {e}", resolved.display()))
                })?;
                Some(serde_json::from_str(&text).map_err(|e| {
                    PyValueError::new_err(format!("invalid observed JSON: {e}"))
                })?)
            } else {
                Some(parse_from_py(obs)?)
            }
        }
        _ => None,
    };

    let (mut sim, warnings, ctrl_err) =
        prepare_simulation(py, &si, assignment, merge, upstream_capacity, controllers)?;
    let _events = execute_sim(py, &mut sim, &ctrl_err, None)?;

    let report = py.allow_threads(|| -> anyhow::Result<PyVerifyReport> {
        let verification = verify_scenario(&sim.scenario, bin_size);
        let bench = if benchmarks {
            Some(core_run_benchmarks()?)
        } else {
            None
        };
        let reference = obs_records
            .as_deref()
            .map(|recs| compare_with_observations(&sim.scenario, recs));
        Ok(PyVerifyReport {
            verification,
            benchmarks: bench,
            reference,
            warnings,
        })
    })
    .map_err(anyhow_err)?;

    serialize_to_py(py, &report)
}

/// Run the 18 closed-form analytic LWR / pipe / roundabout / signal benchmarks.
#[pyfunction]
fn run_benchmarks(py: Python<'_>) -> PyResult<PyObject> {
    let results = py.allow_threads(core_run_benchmarks).map_err(anyhow_err)?;
    serialize_to_py(py, &results)
}

/// Compare an existing simulated [`Scenario`] result? Exposed via `Simulation.compare_observations`.
/// Register the `stream_core_rust` Python module bindings.
pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add_class::<PySimulation>()?;
    m.add_class::<PyControlView>()?;
    m.add_class::<PyCommand>()?;
    m.add_function(wrap_pyfunction!(check, m)?)?;
    m.add_function(wrap_pyfunction!(run, m)?)?;
    m.add_function(wrap_pyfunction!(verify, m)?)?;
    m.add_function(wrap_pyfunction!(run_benchmarks, m)?)?;
    Ok(())
}
