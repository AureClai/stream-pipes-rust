//! Facade crate re-exporting [`stream_core`] (plus the optional `_native`
//! Python extension module when built with `--features python`).
//!
//! ```
//! use stream_core_rust::api::API_VERSION;
//! use stream_core_rust::benchmarks::run_benchmarks;
//!
//! assert_eq!(API_VERSION, 1);
//! assert_eq!(run_benchmarks().unwrap().len(), 18);
//! ```

pub use stream_core::*;

#[cfg(feature = "python")]
use pyo3::prelude::*;

#[cfg(feature = "python")]
#[pymodule]
fn _native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    stream_py::register(m)
}
