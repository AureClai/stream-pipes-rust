use pyo3::exceptions::{PyIndexError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyList;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use stream_core::model::LinkID;
use stream_core::signals::{Command, ControlView, Controller};

/// Command returned by a Python controller's `start(view)` / `wake(view)` method.
#[pyclass(name = "Command")]
#[derive(Clone, Debug)]
pub struct PyCommand {
    pub inner: Command,
}

#[pymethods]
impl PyCommand {
    /// Turn signal `group` green or red at time `at` (seconds).
    #[staticmethod]
    fn switch(group: String, at: f64, green: bool) -> Self {
        Self {
            inner: Command::Switch { group, at, green },
        }
    }

    /// Ask the engine to wake this controller again at time `at` (seconds).
    #[staticmethod]
    fn wake_at(at: f64) -> Self {
        Self {
            inner: Command::WakeAt(at),
        }
    }

    /// Update the per-green vehicle cap of `group` (`None` removes the cap).
    #[staticmethod]
    #[pyo3(signature = (group, n=None))]
    fn set_vehicles_per_green(group: String, n: Option<u32>) -> Self {
        Self {
            inner: Command::SetVehiclesPerGreen { group, n },
        }
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.inner)
    }
}

#[derive(Clone, Copy)]
struct LinkSensorSnapshot {
    vehicles: usize,
    queue: usize,
    entry_queue: usize,
    density: f64,
    occupancy: f64,
    entered: u64,
    exited: u64,
}

/// Read-only sensor snapshot passed to Python controllers on `start` / `wake`.
#[pyclass(name = "ControlView")]
pub struct PyControlView {
    time: f64,
    links: Vec<LinkSensorSnapshot>,
    groups: HashMap<String, (usize, bool)>,
}

impl PyControlView {
    pub fn from_view(view: &ControlView<'_>) -> Self {
        let scenario = view.scenario();
        let links = (0..scenario.links.len())
            .map(|id| LinkSensorSnapshot {
                vehicles: view.vehicles(id),
                queue: view.queue(id),
                entry_queue: view.entry_queue(id),
                density: view.density(id),
                occupancy: view.occupancy(id),
                entered: view.entered(id),
                exited: view.exited(id),
            })
            .collect();
        let mut groups = HashMap::new();
        for g in &scenario.signals.groups {
            if let (Some(idx), Some(green)) = (view.group(&g.id), view.is_green(&g.id)) {
                groups.insert(g.id.clone(), (idx, green));
            }
        }
        Self {
            time: view.time(),
            links,
            groups,
        }
    }

    fn link(&self, link_id: LinkID) -> PyResult<&LinkSensorSnapshot> {
        self.links.get(link_id).ok_or_else(|| {
            PyIndexError::new_err(format!("link_id {link_id} out of range (0..{})", self.links.len()))
        })
    }
}

#[pymethods]
impl PyControlView {
    fn time(&self) -> f64 {
        self.time
    }

    fn group(&self, id: &str) -> Option<usize> {
        self.groups.get(id).map(|&(idx, _)| idx)
    }

    fn is_green(&self, id: &str) -> Option<bool> {
        self.groups.get(id).map(|&(_, green)| green)
    }

    fn vehicles(&self, link_id: LinkID) -> PyResult<usize> {
        Ok(self.link(link_id)?.vehicles)
    }

    fn queue(&self, link_id: LinkID) -> PyResult<usize> {
        Ok(self.link(link_id)?.queue)
    }

    fn entry_queue(&self, link_id: LinkID) -> PyResult<usize> {
        Ok(self.link(link_id)?.entry_queue)
    }

    fn density(&self, link_id: LinkID) -> PyResult<f64> {
        Ok(self.link(link_id)?.density)
    }

    fn occupancy(&self, link_id: LinkID) -> PyResult<f64> {
        Ok(self.link(link_id)?.occupancy)
    }

    fn entered(&self, link_id: LinkID) -> PyResult<u64> {
        Ok(self.link(link_id)?.entered)
    }

    fn exited(&self, link_id: LinkID) -> PyResult<u64> {
        Ok(self.link(link_id)?.exited)
    }
}

fn extract_commands(res: &Bound<'_, PyAny>) -> PyResult<Vec<Command>> {
    if res.is_none() {
        return Ok(Vec::new());
    }
    let list = res.downcast::<PyList>().map_err(|_| {
        PyValueError::new_err("controller start/wake must return a list of Command objects")
    })?;
    let mut cmds = Vec::with_capacity(list.len());
    for item in list.iter() {
        let cmd = item.extract::<PyCommand>().map_err(|_| {
            PyValueError::new_err("controller commands must be instances of stream_core_rust.Command")
        })?;
        cmds.push(cmd.inner);
    }
    Ok(cmds)
}

/// Bridges a Python controller object (`start(view)` / `wake(view)`) into Rust's [`Controller`].
pub struct PyControllerAdapter {
    obj: PyObject,
    error: Arc<Mutex<Option<PyErr>>>,
}

impl PyControllerAdapter {
    pub fn new(obj: PyObject, error: Arc<Mutex<Option<PyErr>>>) -> Self {
        Self { obj, error }
    }

    fn call_method(&mut self, method: &str, view: &ControlView<'_>) -> Vec<Command> {
        if self.error.lock().unwrap().is_some() {
            return Vec::new();
        }
        let snapshot = PyControlView::from_view(view);
        Python::with_gil(|py| {
            let py_view = match Py::new(py, snapshot) {
                Ok(v) => v,
                Err(e) => {
                    *self.error.lock().unwrap() = Some(e);
                    return Vec::new();
                }
            };
            let result = self.obj.call_method1(py, method, (py_view,));
            match result {
                Ok(val) => match extract_commands(val.bind(py)) {
                    Ok(cmds) => cmds,
                    Err(e) => {
                        *self.error.lock().unwrap() = Some(e);
                        Vec::new()
                    }
                },
                Err(e) => {
                    *self.error.lock().unwrap() = Some(e);
                    Vec::new()
                }
            }
        })
    }
}

impl Controller for PyControllerAdapter {
    fn start(&mut self, view: &ControlView) -> Vec<Command> {
        self.call_method("start", view)
    }

    fn wake(&mut self, view: &ControlView) -> Vec<Command> {
        self.call_method("wake", view)
    }
}
