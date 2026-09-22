//! Patch system — "variant = base + patches".
//!
//! A `Patch` is an ordered list of operations over the compiled scenario
//! inputs. Topology and demand ops are merged at compile time; link-attribute
//! ops are time-windowed and pre-resolved here into a `LinkStateChange`
//! schedule that the simulation applies mid-run via `ApplyPatch` events.
//!
//! Resolution model: for every link touched by windowed ops, the window
//! boundaries are collected and, at each boundary, the *effective* attributes
//! are computed as the base network values overlaid by every window active at
//! that instant, in (patch order, op order) — later wins. Each emitted entry
//! is self-contained (it carries every ever-touched field), so the runtime
//! never needs the base network and window ends revert naturally.

use crate::model::{
    Demand, LinkAttrs, LinkID, LinkStateChange, NodeID, PipeIdx, PipeSpec, ALL_CLASSES,
};
use anyhow::{anyhow, Context, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Patch {
    pub name: String,
    #[serde(default)]
    pub description: String,
    pub ops: Vec<PatchOp>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum PatchOp {
    /// Time-windowed attribute override. `from`/`until` are absolute
    /// simulation seconds; absent = the corresponding horizon bound.
    SetLink {
        link: LinkID,
        #[serde(default)]
        from: Option<f64>,
        #[serde(default)]
        until: Option<f64>,
        set: LinkAttrs,
    },
    /// Sugar: the pipe's class mask becomes 0 in the window — no new entrants,
    /// occupants drain normally (mesoscopic planned-closure semantics).
    ClosePipe {
        link: LinkID,
        pipe: PipeIdx,
        #[serde(default)]
        from: Option<f64>,
        #[serde(default)]
        until: Option<f64>,
    },
    /// Sugar: every pipe's class mask becomes 0 in the window. A window that
    /// covers the whole horizon additionally excludes the link from the
    /// assignment graph ("removal" without renumbering ids).
    CloseLink {
        link: LinkID,
        #[serde(default)]
        from: Option<f64>,
        #[serde(default)]
        until: Option<f64>,
    },
    /// Compile-time topology merge. `id` must continue the contiguous node
    /// id range. `node_type`: "Entry" | "Exit" | "Internal" (default).
    AddNode {
        id: NodeID,
        #[serde(default)]
        node_type: Option<String>,
        x: f64,
        y: f64,
    },
    /// Compile-time topology merge of a full GeoJSON LineString feature
    /// (same property schema as network.geojson). The link id must continue
    /// the contiguous link id range. `active_from` = Some(t) closes the link
    /// from the horizon start until t (it exists topologically from t = 0 —
    /// static assignment limitation).
    AddLink {
        feature: serde_json::Value,
        #[serde(default)]
        active_from: Option<f64>,
    },
    /// Compile-time demand scaling: multiplies `flow` and `count` of every
    /// demand entry matching the filters whose period intersects
    /// [from, until). Partial overlaps scale the whole entry (warned).
    ScaleDemand {
        factor: f64,
        #[serde(default)]
        origin: Option<NodeID>,
        #[serde(default)]
        destination: Option<NodeID>,
        #[serde(default)]
        class: Option<String>,
        #[serde(default)]
        from: Option<f64>,
        #[serde(default)]
        until: Option<f64>,
    },
    /// Compile-time: append an OD demand entry.
    AddDemand { demand: Demand },
}

/// Output of `apply_patches`: merged compile-time inputs plus the runtime
/// schedule handed to the simulation through `Scenario::link_schedule`.
#[derive(Debug, Clone)]
pub struct CompiledPatches {
    pub network: serde_json::Value,
    pub demand: serde_json::Value,
    pub schedule: Vec<LinkStateChange>,
    pub assignment_excluded: Vec<LinkID>,
    pub warnings: Vec<String>,
}

// ── Internal working types ───────────────────────────────────────────────────

/// Base (unpatched) parameters of one link, parsed with the SAME defaults as
/// `io::compile_scenario_from_network_value` so effective values match what
/// the compiler produces.
struct BaseLink {
    speed: f64,
    capacity: f64,
    fd_u: f64,
    fd_w: f64,
    fd_kx: f64,
    fd_c: f64,
    num_lanes: u8,
    friction: f64,
    pipe_specs: Vec<PipeSpec>,
    moves: HashMap<LinkID, Vec<PipeIdx>>,
}

impl BaseLink {
    fn n_pipes(&self) -> usize {
        self.pipe_specs.len().max(1)
    }
    fn base_masks(&self) -> Vec<u64> {
        if self.pipe_specs.is_empty() {
            vec![ALL_CLASSES]
        } else {
            self.pipe_specs.iter().map(|s| s.class_mask).collect()
        }
    }
}

#[derive(Clone)]
enum MaskOp {
    Pipe(PipeIdx),
    All,
    Explicit(Vec<u64>),
}

struct Window {
    from: f64,
    until: f64,
    attrs: LinkAttrs, // class_masks stripped into `mask`
    mask: Option<MaskOp>,
}

#[derive(Default)]
struct Touched {
    speed: bool,
    capacity: bool,
    fd_u: bool,
    fd_w: bool,
    fd_kx: bool,
    fd_c: bool,
    num_lanes: bool,
    friction: bool,
    pipe_specs: bool,
    moves: bool,
    masks: bool,
}

impl Touched {
    fn union(&mut self, w: &Window) {
        self.speed |= w.attrs.speed.is_some();
        self.capacity |= w.attrs.capacity.is_some();
        self.fd_u |= w.attrs.fd_u.is_some();
        self.fd_w |= w.attrs.fd_w.is_some();
        self.fd_kx |= w.attrs.fd_kx.is_some();
        self.fd_c |= w.attrs.fd_c.is_some();
        self.num_lanes |= w.attrs.num_lanes.is_some();
        self.friction |= w.attrs.friction.is_some();
        self.pipe_specs |= w.attrs.pipe_specs.is_some();
        self.moves |= w.attrs.moves.is_some();
        self.masks |= w.mask.is_some();
    }
}

fn prop_f64(props: &serde_json::Value, key: &str) -> Option<f64> {
    props.get(key).and_then(|v| v.as_f64())
}

fn parse_base_link(props: &serde_json::Value, id: LinkID, classes: &[String]) -> Result<BaseLink> {
    let speed = prop_f64(props, "speed").unwrap_or(10.0);
    let num_lanes = props.get("lanes").and_then(|v| v.as_u64()).unwrap_or(1);
    if !(1..=255).contains(&num_lanes) {
        return Err(anyhow!("Link {}: 'lanes' must be in 1..=255", id));
    }
    let capacity = prop_f64(props, "capacity").unwrap_or(1.0);
    let pipe_specs = match props.get("pipes") {
        Some(v) => crate::io::parse_pipe_specs(v, classes, id)?,
        None => Vec::new(),
    };
    let moves = match props.get("moves") {
        Some(v) => crate::io::parse_moves(v, id)?,
        None => HashMap::new(),
    };
    Ok(BaseLink {
        speed,
        capacity,
        fd_u: prop_f64(props, "fd_u").unwrap_or(speed),
        fd_w: prop_f64(props, "fd_w").unwrap_or(5.0),
        fd_kx: prop_f64(props, "fd_kx").unwrap_or(0.2),
        fd_c: prop_f64(props, "fd_c").unwrap_or(capacity / num_lanes as f64),
        num_lanes: num_lanes as u8,
        friction: prop_f64(props, "friction").unwrap_or(1.0),
        pipe_specs,
        moves,
    })
}

/// Effective attributes at one boundary: base overlaid by the active windows
/// in order (later wins). Only ever-touched fields are emitted so untouched
/// parameters are never rewritten.
fn effective_attrs(base: &BaseLink, active: &[&Window], touched: &Touched) -> LinkAttrs {
    fn last<T: Copy>(active: &[&Window], get: impl Fn(&Window) -> Option<T>) -> Option<T> {
        active.iter().rev().find_map(|w| get(w))
    }
    let mut out = LinkAttrs::default();
    if touched.speed {
        out.speed = Some(last(active, |w| w.attrs.speed).unwrap_or(base.speed));
    }
    if touched.capacity {
        out.capacity = Some(last(active, |w| w.attrs.capacity).unwrap_or(base.capacity));
    }
    if touched.fd_u {
        out.fd_u = Some(last(active, |w| w.attrs.fd_u).unwrap_or(base.fd_u));
    }
    if touched.fd_w {
        out.fd_w = Some(last(active, |w| w.attrs.fd_w).unwrap_or(base.fd_w));
    }
    if touched.fd_kx {
        out.fd_kx = Some(last(active, |w| w.attrs.fd_kx).unwrap_or(base.fd_kx));
    }
    if touched.fd_c {
        out.fd_c = Some(last(active, |w| w.attrs.fd_c).unwrap_or(base.fd_c));
    }
    if touched.num_lanes {
        out.num_lanes = Some(last(active, |w| w.attrs.num_lanes).unwrap_or(base.num_lanes));
    }
    if touched.friction {
        out.friction = Some(last(active, |w| w.attrs.friction).unwrap_or(base.friction));
    }
    if touched.pipe_specs {
        out.pipe_specs = Some(
            active
                .iter()
                .rev()
                .find_map(|w| w.attrs.pipe_specs.clone())
                .unwrap_or_else(|| base.pipe_specs.clone()),
        );
    }
    if touched.moves {
        out.moves = Some(
            active
                .iter()
                .rev()
                .find_map(|w| w.attrs.moves.clone())
                .unwrap_or_else(|| base.moves.clone()),
        );
    }
    if touched.masks {
        let mut masks = base.base_masks();
        for w in active {
            match &w.mask {
                Some(MaskOp::Pipe(p)) => {
                    if let Some(m) = masks.get_mut(usize::from(*p)) {
                        *m = 0;
                    }
                }
                Some(MaskOp::All) => masks.iter_mut().for_each(|m| *m = 0),
                Some(MaskOp::Explicit(v)) => masks.clone_from(v),
                None => {}
            }
        }
        out.class_masks = Some(masks);
    }
    out
}

/// Merge `patches` into the scenario inputs. See module doc for the model.
pub fn apply_patches(
    network: serde_json::Value,
    demand: serde_json::Value,
    config: &serde_json::Value,
    patches: &[Patch],
) -> Result<CompiledPatches> {
    let mut network = network;
    let mut warnings: Vec<String> = Vec::new();

    // Horizon and classes — same defaults as io::compile_scenario_from_network_value.
    let start = config
        .get("start_time")
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);
    let duration = config
        .get("duration")
        .and_then(|v| v.as_f64())
        .unwrap_or(3600.0);
    let end = start + duration;
    let classes: Vec<String> = config
        .get("classes")
        .and_then(|v| serde_json::from_value(v.clone()).ok())
        .unwrap_or_else(|| vec!["car".to_string()]);

    let mut demand: Vec<Demand> =
        serde_json::from_value(demand).context("Failed to parse demand JSON")?;

    // ── Index the base network ──
    let features = network
        .get_mut("features")
        .and_then(|f| f.as_array_mut())
        .ok_or_else(|| anyhow!("Network must be a GeoJSON FeatureCollection"))?;

    let mut base_links: HashMap<LinkID, BaseLink> = HashMap::new();
    let mut node_ids: Vec<NodeID> = Vec::new();
    for feature in features.iter() {
        let geom_type = feature
            .get("geometry")
            .and_then(|g| g.get("type"))
            .and_then(|t| t.as_str())
            .unwrap_or("");
        let props = feature
            .get("properties")
            .cloned()
            .unwrap_or(serde_json::Value::Null);
        match geom_type {
            "Point" => {
                let id = props
                    .get("id")
                    .and_then(|v| v.as_u64())
                    .ok_or_else(|| anyhow!("Node feature missing id"))?
                    as usize;
                node_ids.push(id);
            }
            "LineString" => {
                let id = props
                    .get("id")
                    .and_then(|v| v.as_u64())
                    .ok_or_else(|| anyhow!("Link feature missing id"))?
                    as usize;
                base_links.insert(id, parse_base_link(&props, id, &classes)?);
            }
            _ => {}
        }
    }

    // ── Pass 1: compile-time ops (topology + demand), in (patch, op) order ──
    // Windowed AddLink activations are collected here and joined with pass 2.
    let mut pending_windows: Vec<(LinkID, Window)> = Vec::new();
    for patch in patches {
        let ctx = || format!("patch '{}'", patch.name);
        for op in &patch.ops {
            match op {
                PatchOp::AddNode {
                    id,
                    node_type,
                    x,
                    y,
                } => {
                    let next = node_ids.iter().max().map_or(0, |m| m + 1);
                    if *id != next {
                        return Err(anyhow!(
                            "{}: add_node id {} must continue the contiguous range (next free id: {})",
                            ctx(), id, next
                        ));
                    }
                    node_ids.push(*id);
                    features.push(serde_json::json!({
                        "type": "Feature",
                        "geometry": {"type": "Point", "coordinates": [x, y]},
                        "properties": {"id": id, "type": node_type.as_deref().unwrap_or("Internal")}
                    }));
                }
                PatchOp::AddLink {
                    feature,
                    active_from,
                } => {
                    let props = feature
                        .get("properties")
                        .ok_or_else(|| anyhow!("{}: add_link feature missing properties", ctx()))?;
                    let id = props
                        .get("id")
                        .and_then(|v| v.as_u64())
                        .ok_or_else(|| anyhow!("{}: add_link feature missing id", ctx()))?
                        as usize;
                    let next = base_links.keys().max().map_or(0, |m| m + 1);
                    if id != next {
                        return Err(anyhow!(
                            "{}: add_link id {} must continue the contiguous range (next free id: {})",
                            ctx(), id, next
                        ));
                    }
                    for key in ["node_up", "node_down"] {
                        let n =
                            props.get(key).and_then(|v| v.as_u64()).ok_or_else(|| {
                                anyhow!("{}: add_link feature missing {}", ctx(), key)
                            })? as usize;
                        if !node_ids.contains(&n) {
                            return Err(anyhow!(
                                "{}: add_link references unknown node {}",
                                ctx(),
                                n
                            ));
                        }
                    }
                    let is_linestring = feature
                        .get("geometry")
                        .and_then(|g| g.get("type"))
                        .and_then(|t| t.as_str())
                        == Some("LineString");
                    if !is_linestring {
                        return Err(anyhow!(
                            "{}: add_link feature geometry must be a LineString",
                            ctx()
                        ));
                    }
                    base_links.insert(id, parse_base_link(props, id, &classes)?);
                    features.push(feature.clone());
                    if let Some(t) = active_from {
                        if !t.is_finite() {
                            return Err(anyhow!("{}: add_link active_from must be finite", ctx()));
                        }
                        if *t >= end {
                            warnings.push(format!(
                                "{}: link {} activates at {} — after the horizon end {}; it will stay closed",
                                ctx(), id, t, end
                            ));
                        }
                        pending_windows.push((
                            id,
                            Window {
                                from: start,
                                until: t.min(end),
                                attrs: LinkAttrs::default(),
                                mask: Some(MaskOp::All),
                            },
                        ));
                    }
                }
                PatchOp::ScaleDemand {
                    factor,
                    origin,
                    destination,
                    class,
                    from,
                    until,
                } => {
                    if !factor.is_finite() || *factor < 0.0 {
                        return Err(anyhow!("{}: scale_demand factor must be ≥ 0", ctx()));
                    }
                    let w_from = from.unwrap_or(f64::NEG_INFINITY);
                    let w_until = until.unwrap_or(f64::INFINITY);
                    let mut matched = 0usize;
                    for d in demand.iter_mut() {
                        if origin.is_some_and(|o| o != d.origin) {
                            continue;
                        }
                        if destination.is_some_and(|dst| dst != d.destination) {
                            continue;
                        }
                        if let Some(name) = class {
                            let d_class = d.class.as_deref().unwrap_or(&classes[0]);
                            if d_class != name {
                                continue;
                            }
                        }
                        // Period must intersect the window.
                        if d.period_end <= w_from || d.period_start >= w_until {
                            continue;
                        }
                        if d.period_start < w_from || d.period_end > w_until {
                            warnings.push(format!(
                                "{}: demand entry {}→{} [{}, {}] only partially overlaps the scaling window — the whole entry is scaled",
                                ctx(), d.origin, d.destination, d.period_start, d.period_end
                            ));
                        }
                        d.flow *= factor;
                        d.count *= factor;
                        matched += 1;
                    }
                    if matched == 0 {
                        warnings.push(format!("{}: scale_demand matched no demand entries", ctx()));
                    }
                }
                PatchOp::AddDemand { demand: d } => demand.push(d.clone()),
                // Windowed ops handled in pass 2.
                PatchOp::SetLink { .. } | PatchOp::ClosePipe { .. } | PatchOp::CloseLink { .. } => {
                }
            }
        }
    }

    // ── Pass 2: windowed ops → per-link window lists, in (patch, op) order ──
    let mut windows: HashMap<LinkID, Vec<Window>> = HashMap::new();
    let mut assignment_excluded: Vec<LinkID> = Vec::new();

    let clamp_window = |from: &Option<f64>,
                        until: &Option<f64>,
                        what: &str,
                        warnings: &mut Vec<String>|
     -> Result<Option<(f64, f64)>> {
        let f = from.unwrap_or(start);
        let u = until.unwrap_or(end);
        if !f.is_finite() && from.is_some() || !u.is_finite() && until.is_some() {
            return Err(anyhow!("{}: window bounds must be finite", what));
        }
        if u <= f {
            return Err(anyhow!(
                "{}: window until ({}) must be > from ({})",
                what,
                u,
                f
            ));
        }
        let f = f.max(start);
        let u = u.min(end);
        if u <= f {
            warnings.push(format!(
                "{}: window lies outside the simulation horizon — ignored",
                what
            ));
            return Ok(None);
        }
        Ok(Some((f, u)))
    };

    for patch in patches {
        for (i, op) in patch.ops.iter().enumerate() {
            let what = format!("patch '{}' op {}", patch.name, i);
            match op {
                PatchOp::SetLink {
                    link,
                    from,
                    until,
                    set,
                } => {
                    let base = base_links
                        .get(link)
                        .ok_or_else(|| anyhow!("{}: unknown link {}", what, link))?;
                    if let Some(specs) = &set.pipe_specs {
                        if base.pipe_specs.is_empty() {
                            return Err(anyhow!(
                                "{}: cannot set pipe_specs on link {} — it has no base pipe partition (the default pipe's capacity semantics differ)",
                                what, link
                            ));
                        }
                        if specs.len() != base.pipe_specs.len() {
                            return Err(anyhow!(
                                "{}: pipe count of link {} must not change mid-run ({} → {})",
                                what,
                                link,
                                base.pipe_specs.len(),
                                specs.len()
                            ));
                        }
                    }
                    if let Some(masks) = &set.class_masks {
                        if masks.len() != base.n_pipes() {
                            return Err(anyhow!(
                                "{}: class_masks length {} does not match link {}'s pipe count {}",
                                what,
                                masks.len(),
                                link,
                                base.n_pipes()
                            ));
                        }
                    }
                    if let Some(l) = set.num_lanes {
                        if l == 0 {
                            return Err(anyhow!("{}: num_lanes must be ≥ 1", what));
                        }
                    }
                    let Some((f, u)) = clamp_window(from, until, &what, &mut warnings)? else {
                        continue;
                    };
                    let mut attrs = set.clone();
                    let mask = attrs.class_masks.take().map(MaskOp::Explicit);
                    windows.entry(*link).or_default().push(Window {
                        from: f,
                        until: u,
                        attrs,
                        mask,
                    });
                }
                PatchOp::ClosePipe {
                    link,
                    pipe,
                    from,
                    until,
                } => {
                    let base = base_links
                        .get(link)
                        .ok_or_else(|| anyhow!("{}: unknown link {}", what, link))?;
                    if usize::from(*pipe) >= base.n_pipes() {
                        return Err(anyhow!(
                            "{}: pipe {} out of range for link {} ({} pipes)",
                            what,
                            pipe,
                            link,
                            base.n_pipes()
                        ));
                    }
                    let Some((f, u)) = clamp_window(from, until, &what, &mut warnings)? else {
                        continue;
                    };
                    windows.entry(*link).or_default().push(Window {
                        from: f,
                        until: u,
                        attrs: LinkAttrs::default(),
                        mask: Some(MaskOp::Pipe(*pipe)),
                    });
                }
                PatchOp::CloseLink { link, from, until } => {
                    if !base_links.contains_key(link) {
                        return Err(anyhow!("{}: unknown link {}", what, link));
                    }
                    let Some((f, u)) = clamp_window(from, until, &what, &mut warnings)? else {
                        continue;
                    };
                    if f <= start && u >= end && !assignment_excluded.contains(link) {
                        assignment_excluded.push(*link);
                    }
                    windows.entry(*link).or_default().push(Window {
                        from: f,
                        until: u,
                        attrs: LinkAttrs::default(),
                        mask: Some(MaskOp::All),
                    });
                }
                _ => {}
            }
        }
    }
    for (link, w) in pending_windows {
        if w.until > w.from {
            windows.entry(link).or_default().push(w);
        }
    }

    // ── Resolve windows into the self-contained boundary schedule ──
    let mut schedule: Vec<LinkStateChange> = Vec::new();
    for (&link_id, wins) in &windows {
        let base = &base_links[&link_id];
        let mut touched = Touched::default();
        for w in wins {
            touched.union(w);
        }
        let mut boundaries: Vec<f64> = Vec::new();
        for w in wins {
            boundaries.push(w.from);
            if w.until < end {
                boundaries.push(w.until); // revert boundary
            }
        }
        boundaries.sort_by(|a, b| a.partial_cmp(b).expect("finite boundaries"));
        boundaries.dedup();
        for t in boundaries {
            let active: Vec<&Window> = wins.iter().filter(|w| w.from <= t && t < w.until).collect();
            schedule.push(LinkStateChange {
                time: t,
                link_id,
                attrs: effective_attrs(base, &active, &touched),
            });
        }
    }
    schedule.sort_by(|a, b| {
        a.time
            .partial_cmp(&b.time)
            .expect("finite schedule times")
            .then(a.link_id.cmp(&b.link_id))
    });
    assignment_excluded.sort_unstable();

    if !schedule.is_empty() {
        warnings.push(
            "dynamic patches present: assignment is static — vehicles do not reroute in response to mid-run changes".to_string(),
        );
    }

    Ok(CompiledPatches {
        network,
        demand: serde_json::to_value(demand).context("Failed to re-serialize demand")?,
        schedule,
        assignment_excluded,
        warnings,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn network_json() -> serde_json::Value {
        serde_json::json!({
            "type": "FeatureCollection",
            "features": [
                {"type": "Feature", "geometry": {"type": "Point", "coordinates": [0, 0]},
                 "properties": {"id": 0, "type": "Entry"}},
                {"type": "Feature", "geometry": {"type": "Point", "coordinates": [1, 0]},
                 "properties": {"id": 1, "type": "Exit"}},
                {"type": "Feature", "geometry": {"type": "LineString", "coordinates": [[0,0],[1,0]]},
                 "properties": {"id": 0, "node_up": 0, "node_down": 1, "length": 100.0,
                                "speed": 10.0, "lanes": 2, "capacity": 1.0,
                                "fd_u": 10.0, "fd_w": 5.0, "fd_kx": 0.02, "fd_c": 0.5}}
            ]
        })
    }

    fn config_json() -> serde_json::Value {
        serde_json::json!({"start_time": 0.0, "duration": 3600.0})
    }

    fn demand_json() -> serde_json::Value {
        serde_json::json!([
            {"period_start": 0.0, "period_end": 600.0, "origin": 0, "destination": 1,
             "flow": 360.0, "count": 60.0}
        ])
    }

    fn patch(ops: Vec<PatchOp>) -> Patch {
        Patch {
            name: "test".into(),
            description: String::new(),
            ops,
        }
    }

    #[test]
    fn empty_patch_list_is_inert() {
        let out = apply_patches(network_json(), demand_json(), &config_json(), &[]).unwrap();
        assert!(out.schedule.is_empty());
        assert!(out.assignment_excluded.is_empty());
        assert!(out.warnings.is_empty());
        assert_eq!(out.network, network_json());
        assert_eq!(out.demand, demand_json());
    }

    #[test]
    fn windowed_set_link_emits_boundary_and_revert() {
        let p = patch(vec![PatchOp::SetLink {
            link: 0,
            from: Some(600.0),
            until: Some(1200.0),
            set: LinkAttrs {
                capacity: Some(0.5),
                ..Default::default()
            },
        }]);
        let out = apply_patches(network_json(), demand_json(), &config_json(), &[p]).unwrap();
        assert_eq!(out.schedule.len(), 2);
        assert_eq!(out.schedule[0].time, 600.0);
        assert_eq!(out.schedule[0].attrs.capacity, Some(0.5));
        // Revert entry carries the base value — self-contained.
        assert_eq!(out.schedule[1].time, 1200.0);
        assert_eq!(out.schedule[1].attrs.capacity, Some(1.0));
        assert!(
            out.schedule[1].attrs.speed.is_none(),
            "untouched fields stay absent"
        );
    }

    #[test]
    fn overlapping_windows_later_wins_and_layers_revert() {
        // A: capacity 0.5 on [600, 1800); B: capacity 0.25 on [1200, 1500).
        let p = patch(vec![
            PatchOp::SetLink {
                link: 0,
                from: Some(600.0),
                until: Some(1800.0),
                set: LinkAttrs {
                    capacity: Some(0.5),
                    ..Default::default()
                },
            },
            PatchOp::SetLink {
                link: 0,
                from: Some(1200.0),
                until: Some(1500.0),
                set: LinkAttrs {
                    capacity: Some(0.25),
                    ..Default::default()
                },
            },
        ]);
        let out = apply_patches(network_json(), demand_json(), &config_json(), &[p]).unwrap();
        let caps: Vec<(f64, f64)> = out
            .schedule
            .iter()
            .map(|c| (c.time, c.attrs.capacity.unwrap()))
            .collect();
        assert_eq!(
            caps,
            vec![(600.0, 0.5), (1200.0, 0.25), (1500.0, 0.5), (1800.0, 1.0)]
        );
    }

    #[test]
    fn close_pipe_zeroes_only_that_mask() {
        let p = patch(vec![PatchOp::ClosePipe {
            link: 0,
            pipe: 0,
            from: Some(100.0),
            until: Some(200.0),
        }]);
        // Give the link two pipes first.
        let mut net = network_json();
        net["features"][2]["properties"]["pipes"] = serde_json::json!([{"lanes": 1}, {"lanes": 1}]);
        let out = apply_patches(net, demand_json(), &config_json(), &[p]).unwrap();
        assert_eq!(out.schedule.len(), 2);
        assert_eq!(
            out.schedule[0].attrs.class_masks,
            Some(vec![0, ALL_CLASSES])
        );
        assert_eq!(
            out.schedule[1].attrs.class_masks,
            Some(vec![ALL_CLASSES, ALL_CLASSES])
        );
    }

    #[test]
    fn whole_horizon_close_link_excludes_from_assignment() {
        let p = patch(vec![PatchOp::CloseLink {
            link: 0,
            from: None,
            until: None,
        }]);
        let out = apply_patches(network_json(), demand_json(), &config_json(), &[p]).unwrap();
        assert_eq!(out.assignment_excluded, vec![0]);
        // Eager entry at the horizon start with all masks zeroed.
        assert_eq!(out.schedule[0].time, 0.0);
        assert_eq!(out.schedule[0].attrs.class_masks, Some(vec![0]));
    }

    #[test]
    fn add_link_requires_contiguous_id() {
        let p = patch(vec![PatchOp::AddLink {
            feature: serde_json::json!({
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": [[0,0],[1,0]]},
                "properties": {"id": 5, "node_up": 0, "node_down": 1, "length": 100.0}
            }),
            active_from: None,
        }]);
        let err = apply_patches(network_json(), demand_json(), &config_json(), &[p])
            .unwrap_err()
            .to_string();
        assert!(err.contains("next free id: 1"), "got: {err}");
    }

    #[test]
    fn add_link_with_activation_closes_until_t() {
        let p = patch(vec![PatchOp::AddLink {
            feature: serde_json::json!({
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": [[0,0],[1,0]]},
                "properties": {"id": 1, "node_up": 0, "node_down": 1, "length": 100.0,
                               "speed": 10.0, "lanes": 1, "capacity": 0.5}
            }),
            active_from: Some(1800.0),
        }]);
        let out = apply_patches(network_json(), demand_json(), &config_json(), &[p]).unwrap();
        // Network gained the feature.
        assert_eq!(out.network["features"].as_array().unwrap().len(), 4);
        // Closed [0, 1800), reopens at 1800.
        assert_eq!(out.schedule.len(), 2);
        assert_eq!(out.schedule[0].time, 0.0);
        assert_eq!(out.schedule[0].attrs.class_masks, Some(vec![0]));
        assert_eq!(out.schedule[1].time, 1800.0);
        assert_eq!(out.schedule[1].attrs.class_masks, Some(vec![ALL_CLASSES]));
    }

    #[test]
    fn scale_demand_filters_and_scales() {
        let p = patch(vec![PatchOp::ScaleDemand {
            factor: 1.5,
            origin: Some(0),
            destination: None,
            class: None,
            from: None,
            until: None,
        }]);
        let out = apply_patches(network_json(), demand_json(), &config_json(), &[p]).unwrap();
        let d: Vec<Demand> = serde_json::from_value(out.demand).unwrap();
        assert!((d[0].flow - 540.0).abs() < 1e-9);
        assert!((d[0].count - 90.0).abs() < 1e-9);
    }

    #[test]
    fn scale_demand_outside_window_is_untouched() {
        let p = patch(vec![PatchOp::ScaleDemand {
            factor: 2.0,
            origin: None,
            destination: None,
            class: None,
            from: Some(700.0),
            until: Some(900.0),
        }]);
        let out = apply_patches(network_json(), demand_json(), &config_json(), &[p]).unwrap();
        let d: Vec<Demand> = serde_json::from_value(out.demand).unwrap();
        assert!(
            (d[0].flow - 360.0).abs() < 1e-9,
            "period [0,600) does not intersect [700,900)"
        );
        assert!(out.warnings.iter().any(|w| w.contains("matched no demand")));
    }

    #[test]
    fn pipe_count_change_is_rejected() {
        let p = patch(vec![PatchOp::SetLink {
            link: 0,
            from: None,
            until: None,
            set: LinkAttrs {
                pipe_specs: Some(vec![
                    PipeSpec {
                        lanes: 1,
                        class_mask: ALL_CLASSES,
                    },
                    PipeSpec {
                        lanes: 1,
                        class_mask: ALL_CLASSES,
                    },
                ]),
                ..Default::default()
            },
        }]);
        let err = apply_patches(network_json(), demand_json(), &config_json(), &[p])
            .unwrap_err()
            .to_string();
        assert!(err.contains("pipe"), "got: {err}");
    }

    #[test]
    fn unknown_link_is_rejected() {
        let p = patch(vec![PatchOp::CloseLink {
            link: 42,
            from: None,
            until: None,
        }]);
        let err = apply_patches(network_json(), demand_json(), &config_json(), &[p])
            .unwrap_err()
            .to_string();
        assert!(err.contains("unknown link 42"), "got: {err}");
    }

    #[test]
    fn patch_json_round_trip() {
        let json = r#"{
            "name": "close-right-lane",
            "description": "Right lane closed 8:00-9:00",
            "ops": [
                {"op": "close_pipe", "link": 4, "pipe": 0, "from": 28800, "until": 32400},
                {"op": "set_link", "link": 4, "from": 28800, "until": 32400,
                 "set": {"friction": 0.85}},
                {"op": "scale_demand", "factor": 1.2}
            ]
        }"#;
        let p: Patch = serde_json::from_str(json).expect("patch JSON must parse");
        assert_eq!(p.ops.len(), 3);
        let back = serde_json::to_string(&p).unwrap();
        let p2: Patch = serde_json::from_str(&back).unwrap();
        assert_eq!(p2.name, p.name);
    }
}
