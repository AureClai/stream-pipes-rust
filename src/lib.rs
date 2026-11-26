pub use stream_core::*;

#[cfg(feature = "python")]
use pyo3::prelude::*;

#[cfg(feature = "python")]
#[pymodule]
fn stream_core_rust(m: &Bound<'_, PyModule>) -> PyResult<()> {
    stream_py::register(m)
}
