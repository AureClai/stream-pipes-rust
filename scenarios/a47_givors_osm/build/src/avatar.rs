//! Cerema AVATAR count points and their corrected ("fixed") measures.

use std::collections::BTreeMap;
use std::path::Path;

pub struct CountPoint {
    pub id: u32,
    pub name: String,
    pub road: String,
    pub destination: String,
    pub pos: (f64, f64),
}

/// One measure: flow (veh/h) and speed (km/h) at `t` seconds after midnight,
/// and whether AVATAR measured it or imputed it ("predicted").
#[derive(Clone, Copy)]
pub struct Sample {
    pub t: f64,
    pub flow: Option<f64>,
    pub speed: Option<f64>,
    pub flow_measured: bool,
    pub speed_measured: bool,
}

fn unquote(s: &str) -> String {
    s.trim().trim_matches('"').to_string()
}

pub fn count_points(path: &Path) -> Vec<CountPoint> {
    let text = std::fs::read_to_string(path).expect("read count points");
    let mut v = Vec::new();
    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.split(';').collect();
        if f.len() < 5 {
            continue;
        }
        let p = f[4].trim().trim_start_matches("POINT (").trim_end_matches(')');
        let mut it = p.split_whitespace().map(|x| x.parse::<f64>().unwrap_or(f64::NAN));
        let (Some(lon), Some(lat)) = (it.next(), it.next()) else { continue };
        v.push(CountPoint {
            id: f[0].trim().parse().unwrap_or(0),
            name: unquote(f[1]),
            road: unquote(f[2]),
            destination: unquote(f[3]),
            pos: (lon, lat),
        });
    }
    v
}

/// Samples per count point, sorted by time.
pub fn measures(path: &Path) -> BTreeMap<u32, Vec<Sample>> {
    let text = std::fs::read_to_string(path).expect("read measures");
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().unwrap_or_default().split(';').collect();
    let col = |name: &str| header.iter().position(|h| *h == name).unwrap_or_else(|| panic!("no column {name}"));
    let (c_id, c_t, c_q, c_v) = (col("count_point_id"), col("measure_datetime"), col("flow[veh/h]"), col("speed[km/h]"));
    let (c_qp, c_vp) = (col("flow_predicted"), col("speed_predicted"));
    let mut out: BTreeMap<u32, Vec<Sample>> = BTreeMap::new();
    for line in lines {
        let f: Vec<&str> = line.split(';').collect();
        if f.len() <= c_vp {
            continue;
        }
        // 2026-06-11T07:06:00+02:00 -> local seconds after midnight.
        let t = &f[c_t];
        let (h, m): (f64, f64) = (t[11..13].parse().unwrap_or(0.0), t[14..16].parse().unwrap_or(0.0));
        let num = |s: &str| s.trim().parse::<f64>().ok();
        let flow = num(f[c_q]);
        let speed = num(f[c_v]);
        out.entry(f[c_id].trim().parse().unwrap_or(0)).or_default().push(Sample {
            t: h * 3600.0 + m * 60.0,
            flow,
            speed,
            flow_measured: flow.is_some() && f[c_qp].trim() == "0",
            speed_measured: speed.is_some() && f[c_vp].trim() == "0",
        });
    }
    for v in out.values_mut() {
        v.sort_by(|a, b| a.t.total_cmp(&b.t));
    }
    out
}

/// Sampling step of a series, seconds (360 for 6-min data, 3600 hourly).
pub fn step(samples: &[Sample]) -> f64 {
    samples.windows(2).map(|w| w[1].t - w[0].t).filter(|d| *d > 0.0).fold(f64::INFINITY, f64::min)
}

/// Samples whose period lies in [t0, t1).
pub fn in_window(samples: &[Sample], t0: f64, t1: f64) -> Vec<Sample> {
    let st = step(samples);
    samples.iter().copied().filter(|s| s.t >= t0 - 1e-6 && s.t + st <= t1 + 1e-6).collect()
}

/// Mean flow over [t0, t1): from the samples covering it; hourly series
/// cover a 30-min window by the hour that contains it.
pub fn flow_over(samples: &[Sample], t0: f64, t1: f64, measured_only: bool) -> Option<(f64, usize, usize)> {
    let st = step(samples);
    let sel: Vec<Sample> = if st > t1 - t0 + 1e-6 {
        samples.iter().copied().filter(|s| s.t <= t0 + 1e-6 && s.t + st >= t1 - 1e-6).collect()
    } else {
        in_window(samples, t0, t1)
    };
    let expected = if st > t1 - t0 + 1e-6 { 1 } else { ((t1 - t0) / st).round() as usize };
    let usable: Vec<f64> = sel.iter().filter(|s| !measured_only || s.flow_measured).filter_map(|s| s.flow).collect();
    if usable.is_empty() {
        return None;
    }
    let measured = sel.iter().filter(|s| s.flow_measured).count();
    Some((usable.iter().sum::<f64>() / usable.len() as f64, measured, expected))
}
