use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{PyBool, PyDict, PyFloat, PyInt, PyList, PyNone, PyString, PyTuple};
use serde::de::DeserializeOwned;
use serde::Serialize;
use serde_json::{Map, Number, Value};

/// Convert an arbitrary Python JSON-like object (`None`, `bool`, `int`, `float`,
/// `str`, `list`/`tuple`, `dict`) into a `serde_json::Value`.
pub fn py_to_json(obj: &Bound<'_, PyAny>) -> PyResult<Value> {
    if obj.is_instance_of::<PyNone>() {
        return Ok(Value::Null);
    }
    // Check bool before int because Python `bool` subclasses `int`.
    if obj.is_instance_of::<PyBool>() {
        return Ok(Value::Bool(obj.extract::<bool>()?));
    }
    if obj.is_instance_of::<PyInt>() {
        if let Ok(i) = obj.extract::<i64>() {
            return Ok(Value::Number(i.into()));
        }
        let u = obj.extract::<u64>()?;
        return Ok(Value::Number(u.into()));
    }
    if obj.is_instance_of::<PyFloat>() {
        let f = obj.extract::<f64>()?;
        let n = Number::from_f64(f)
            .ok_or_else(|| PyValueError::new_err(format!("non-finite float {f} is not valid JSON")))?;
        return Ok(Value::Number(n));
    }
    if obj.is_instance_of::<PyString>() {
        return Ok(Value::String(obj.extract::<String>()?));
    }
    if let Ok(list) = obj.downcast::<PyList>() {
        let mut out = Vec::with_capacity(list.len());
        for item in list.iter() {
            out.push(py_to_json(&item)?);
        }
        return Ok(Value::Array(out));
    }
    if let Ok(tuple) = obj.downcast::<PyTuple>() {
        let mut out = Vec::with_capacity(tuple.len());
        for item in tuple.iter() {
            out.push(py_to_json(&item)?);
        }
        return Ok(Value::Array(out));
    }
    if let Ok(dict) = obj.downcast::<PyDict>() {
        let mut map = Map::with_capacity(dict.len());
        for (k, v) in dict.iter() {
            let key = k.extract::<String>().map_err(|_| {
                PyValueError::new_err("JSON object keys must be strings")
            })?;
            map.insert(key, py_to_json(&v)?);
        }
        return Ok(Value::Object(map));
    }
    Err(PyValueError::new_err(format!(
        "unsupported value of type '{}' (expected JSON-compatible dict/list/primitive)",
        obj.get_type().name()?
    )))
}

/// Convert a `serde_json::Value` into native Python objects (`dict`, `list`, primitives).
pub fn json_to_py(py: Python<'_>, value: &Value) -> PyResult<PyObject> {
    match value {
        Value::Null => Ok(py.None()),
        Value::Bool(b) => Ok((*b).into_py(py)),
        Value::Number(n) => {
            if let Some(i) = n.as_i64() {
                Ok(i.into_py(py))
            } else if let Some(u) = n.as_u64() {
                Ok(u.into_py(py))
            } else if let Some(f) = n.as_f64() {
                Ok(f.into_py(py))
            } else {
                Err(PyValueError::new_err("unsupported JSON number"))
            }
        }
        Value::String(s) => Ok(s.into_py(py)),
        Value::Array(arr) => {
            let mut items = Vec::with_capacity(arr.len());
            for v in arr {
                items.push(json_to_py(py, v)?);
            }
            Ok(PyList::new_bound(py, items).into_py(py))
        }
        Value::Object(map) => {
            let dict = PyDict::new_bound(py);
            for (k, v) in map {
                dict.set_item(k, json_to_py(py, v)?)?;
            }
            Ok(dict.into_py(py))
        }
    }
}

pub fn serialize_to_py<T: Serialize>(py: Python<'_>, value: &T) -> PyResult<PyObject> {
    let json = serde_json::to_value(value)
        .map_err(|e| PyValueError::new_err(format!("serialization error: {e}")))?;
    json_to_py(py, &json)
}

pub fn parse_from_py<T: DeserializeOwned>(obj: &Bound<'_, PyAny>) -> PyResult<T> {
    let val = py_to_json(obj)?;
    serde_json::from_value(val)
        .map_err(|e| PyValueError::new_err(format!("invalid input structure: {e}")))
}
