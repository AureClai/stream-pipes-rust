pub mod model;
pub mod simulation;
pub mod assignment;
pub mod io;
pub mod validation;
pub mod analysis;
pub mod diagnostics;
pub mod xt_analysis;
pub mod verification;
pub mod benchmarks;
pub mod patch;
pub mod reference;

#[cfg(feature = "python")]
use pyo3::prelude::*;

#[cfg(feature = "python")]
#[pymodule]
fn stream_core_rust(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<simulation::Simulation>()?;
    Ok(())
}
