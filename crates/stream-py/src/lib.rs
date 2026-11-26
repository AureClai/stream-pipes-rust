use pyo3::prelude::*;
use stream_core::simulation::Simulation as CoreSimulation;

#[pyclass(name = "Simulation")]
pub struct PySimulation {
    #[allow(dead_code)]
    inner: CoreSimulation,
}

#[pymethods]
impl PySimulation {
    // TODO: Expose load, run, and verification methods to Python.
}

/// Register the `stream_core_rust` Python module bindings.
pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PySimulation>()?;
    Ok(())
}
