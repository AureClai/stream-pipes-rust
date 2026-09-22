use crate::model::{
    Demand, FundamentalDiagram, Link, Node, NodeType, PipeIdx, PipeSpec, Scenario, ALL_CLASSES,
};
use anyhow::{anyhow, Context, Result};
use geojson::{GeoJson, Value};
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

/// Resolve a list of class names to a permission bitmask against the
/// scenario's class list.
pub(crate) fn class_mask_from_names(
    names: &[serde_json::Value],
    classes: &[String],
    link_id: usize,
) -> Result<u64> {
    let mut mask = 0u64;
    for name_val in names {
        let name = name_val
            .as_str()
            .ok_or_else(|| anyhow!("Link {}: pipe class entries must be strings", link_id))?;
        let idx = classes.iter().position(|c| c == name).ok_or_else(|| {
            anyhow!(
                "Link {}: unknown vehicle class '{}' (declared classes: {:?})",
                link_id,
                name,
                classes
            )
        })?;
        mask |= 1u64 << idx;
    }
    Ok(mask)
}

/// Parse the optional per-link pipe schema:
///   "pipes": [{"lanes": 1, "classes": ["bus"]}, {"lanes": 2}]
pub(crate) fn parse_pipe_specs(
    val: &serde_json::Value,
    classes: &[String],
    link_id: usize,
) -> Result<Vec<PipeSpec>> {
    let arr = val
        .as_array()
        .ok_or_else(|| anyhow!("Link {}: 'pipes' must be an array", link_id))?;
    arr.iter()
        .map(|p| {
            let lanes = p
                .get("lanes")
                .and_then(|v| v.as_u64())
                .filter(|&l| (1..=255).contains(&l))
                .ok_or_else(|| {
                    anyhow!(
                        "Link {}: each pipe needs an integer 'lanes' in 1..=255",
                        link_id
                    )
                })?;
            let class_mask = match p.get("classes").and_then(|v| v.as_array()) {
                Some(names) => class_mask_from_names(names, classes, link_id)?,
                None => ALL_CLASSES,
            };
            Ok(PipeSpec {
                lanes: lanes as u8,
                class_mask,
            })
        })
        .collect()
}

/// Parse the optional movement restriction:
///   "moves": {"<out_link_id>": [0, 1]}
pub(crate) fn parse_moves(
    val: &serde_json::Value,
    link_id: usize,
) -> Result<std::collections::HashMap<usize, Vec<PipeIdx>>> {
    let obj = val
        .as_object()
        .ok_or_else(|| anyhow!("Link {}: 'moves' must be an object", link_id))?;
    let mut moves = std::collections::HashMap::new();
    for (key, pipes_val) in obj {
        let out_link: usize = key
            .parse()
            .map_err(|_| anyhow!("Link {}: 'moves' key '{}' is not a link id", link_id, key))?;
        let pipes: Vec<PipeIdx> = pipes_val
            .as_array()
            .ok_or_else(|| {
                anyhow!(
                    "Link {}: 'moves' values must be arrays of pipe indices",
                    link_id
                )
            })?
            .iter()
            .map(|v| {
                v.as_u64()
                    .filter(|&i| i < 256)
                    .map(|i| i as PipeIdx)
                    .ok_or_else(|| anyhow!("Link {}: invalid pipe index in 'moves'", link_id))
            })
            .collect::<Result<_>>()?;
        moves.insert(out_link, pipes);
    }
    Ok(moves)
}

pub fn load_scenario<P: AsRef<Path>>(path: P) -> Result<Scenario> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);
    let scenario = serde_json::from_reader(reader)?;
    Ok(scenario)
}

pub fn compile_scenario<P: AsRef<Path>>(
    network_path: P,
    demand_path: P,
    config_path: P,
) -> Result<Scenario> {
    let demand_file = File::open(demand_path).context("Failed to open demand file")?;
    let demand_val: serde_json::Value = serde_json::from_reader(BufReader::new(demand_file))?;

    let config_file = File::open(config_path).context("Failed to open config file")?;
    let config_val: serde_json::Value = serde_json::from_reader(BufReader::new(config_file))?;

    compile_scenario_from_values(network_path, demand_val, config_val)
}

/// Build a Scenario from a GeoJSON network file and in-memory demand/config JSON values.
/// Useful when demand and config are already parsed (e.g. from an HTTP request body).
pub fn compile_scenario_from_values<P: AsRef<Path>>(
    network_path: P,
    demand_val: serde_json::Value,
    config_val: serde_json::Value,
) -> Result<Scenario> {
    let network_file = File::open(network_path).context("Failed to open network file")?;
    let reader = BufReader::new(network_file);
    let network_val: serde_json::Value =
        serde_json::from_reader(reader).context("Failed to parse network JSON")?;
    compile_scenario_from_network_value(network_val, demand_val, config_val)
}

/// Fully values-based compilation: all three inputs are in-memory JSON.
/// Used by the patch pipeline, where the network has been merged/overlaid
/// before compilation.
pub fn compile_scenario_from_network_value(
    network_val: serde_json::Value,
    demand_val: serde_json::Value,
    config_val: serde_json::Value,
) -> Result<Scenario> {
    // 1. Parse Network (GeoJSON)
    let geojson = GeoJson::from_json_value(network_val).context("Failed to parse GeoJSON")?;

    let collection = match geojson {
        GeoJson::FeatureCollection(fc) => fc,
        _ => return Err(anyhow!("Network file must be a FeatureCollection")),
    };

    // Vehicle classes (needed while parsing pipe class permissions).
    let classes: Vec<String> = config_val
        .get("classes")
        .and_then(|v| serde_json::from_value(v.clone()).ok())
        .unwrap_or_else(|| vec!["car".to_string()]);

    let mut nodes = Vec::new();
    let mut links = Vec::new();

    for feature in collection.features {
        let props = feature
            .properties
            .ok_or(anyhow!("Feature missing properties"))?;
        let geom = feature
            .geometry
            .ok_or(anyhow!("Feature missing geometry"))?;

        match geom.value {
            Value::Point(coords) => {
                let id = props
                    .get("id")
                    .and_then(|v| v.as_u64())
                    .ok_or(anyhow!("Node missing ID"))? as usize;
                let type_str = props
                    .get("type")
                    .and_then(|v| v.as_str())
                    .unwrap_or("Internal");
                let node_type = match type_str {
                    "Entry" => NodeType::Entry,
                    "Exit" => NodeType::Exit,
                    _ => NodeType::Internal,
                };
                let x = coords
                    .first()
                    .copied()
                    .ok_or(anyhow!("Node missing x coordinate"))?;
                let y = coords
                    .get(1)
                    .copied()
                    .ok_or(anyhow!("Node missing y coordinate"))?;
                nodes.push(Node {
                    id,
                    node_type,
                    incoming_links: Vec::new(),
                    outgoing_links: Vec::new(),
                    points: (x, y),
                    signals: Vec::new(),
                });
            }
            Value::LineString(coords) => {
                let id = props
                    .get("id")
                    .and_then(|v| v.as_u64())
                    .ok_or(anyhow!("Link missing ID"))? as usize;
                let u = props
                    .get("node_up")
                    .and_then(|v| v.as_u64())
                    .ok_or(anyhow!("Link missing node_up"))? as usize;
                let v = props
                    .get("node_down")
                    .and_then(|v| v.as_u64())
                    .ok_or(anyhow!("Link missing node_down"))? as usize;

                let length = props.get("length").and_then(|v| v.as_f64()).unwrap_or(0.0);
                let speed = props.get("speed").and_then(|v| v.as_f64()).unwrap_or(10.0);
                let lanes_u64 = props.get("lanes").and_then(|v| v.as_u64()).unwrap_or(1);
                if !(1..=255).contains(&lanes_u64) {
                    return Err(anyhow!(
                        "Link {}: 'lanes' must be in 1..=255 (got {})",
                        id,
                        lanes_u64
                    ));
                }
                let lanes = lanes_u64 as u8;
                let capacity = props
                    .get("capacity")
                    .and_then(|v| v.as_f64())
                    .unwrap_or(1.0);

                let fd = FundamentalDiagram {
                    u: props.get("fd_u").and_then(|v| v.as_f64()).unwrap_or(speed),
                    w: props.get("fd_w").and_then(|v| v.as_f64()).unwrap_or(5.0),
                    kx: props.get("fd_kx").and_then(|v| v.as_f64()).unwrap_or(0.2),
                    c: props
                        .get("fd_c")
                        .and_then(|v| v.as_f64())
                        .unwrap_or(capacity / (lanes as f64)),
                };

                let points: Vec<(f64, f64)> = coords
                    .iter()
                    .filter_map(|p| {
                        let x = p.first().copied()?;
                        let y = p.get(1).copied()?;
                        Some((x, y))
                    })
                    .collect();

                let mut link = Link::new(id, u, v, length, speed, lanes, capacity, fd, points);
                if let Some(pipes_val) = props.get("pipes") {
                    link.pipe_specs = parse_pipe_specs(pipes_val, &classes, id)?;
                }
                if let Some(moves_val) = props.get("moves") {
                    link.moves = parse_moves(moves_val, id)?;
                }
                if let Some(f) = props.get("friction").and_then(|v| v.as_f64()) {
                    link.friction = f;
                }
                links.push(link);
            }
            _ => {}
        }
    }

    // Sort by ID so that node.id == Vec index after the sort
    nodes.sort_by_key(|n| n.id);
    links.sort_by_key(|l| l.id);

    // Build node index map and infer topology
    let mut node_idx_map = std::collections::HashMap::new();
    for (i, n) in nodes.iter().enumerate() {
        node_idx_map.insert(n.id, i);
    }
    for link in &links {
        if let Some(&idx) = node_idx_map.get(&link.node_up) {
            nodes[idx].outgoing_links.push(link.id);
        }
        if let Some(&idx) = node_idx_map.get(&link.node_down) {
            nodes[idx].incoming_links.push(link.id);
        }
    }

    // 2. Parse Demand from value
    let demand: Vec<Demand> =
        serde_json::from_value(demand_val).context("Failed to parse demand JSON")?;

    // 3. Parse Config from value
    let start_time = config_val
        .get("start_time")
        .and_then(|v| v.as_f64())
        .unwrap_or(0.0);
    let duration = config_val
        .get("duration")
        .and_then(|v| v.as_f64())
        .unwrap_or(3600.0);

    Ok(Scenario {
        nodes,
        links,
        vehicles: Vec::new(),
        demand,
        start_time,
        duration,
        classes,
        link_schedule: Vec::new(),
        assignment_excluded: Vec::new(),
    })
}

/// Full variant pipeline: merge `patches` into the inputs, compile, and
/// attach the runtime schedule. Returns the scenario plus human-readable
/// warnings (surfaced in the run response). With an empty patch list this is
/// exactly `compile_scenario_from_network_value`.
pub fn compile_with_patches(
    network_val: serde_json::Value,
    demand_val: serde_json::Value,
    config_val: serde_json::Value,
    patches: &[crate::patch::Patch],
) -> Result<(Scenario, Vec<String>)> {
    if patches.is_empty() {
        let scenario = compile_scenario_from_network_value(network_val, demand_val, config_val)?;
        return Ok((scenario, Vec::new()));
    }
    let compiled = crate::patch::apply_patches(network_val, demand_val, &config_val, patches)?;
    let mut scenario =
        compile_scenario_from_network_value(compiled.network, compiled.demand, config_val)?;
    scenario.link_schedule = compiled.schedule;
    scenario.assignment_excluded = compiled.assignment_excluded;
    Ok((scenario, compiled.warnings))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario_dir(name: &str) -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("scenarios")
            .join(name)
    }

    fn bottleneck_values() -> (serde_json::Value, serde_json::Value) {
        let dir = scenario_dir("bottleneck");
        let demand: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(dir.join("demand.json")).unwrap())
                .unwrap();
        let config: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(dir.join("config.json")).unwrap())
                .unwrap();
        (demand, config)
    }

    #[test]
    fn compile_succeeds_on_valid_input() {
        let dir = scenario_dir("bottleneck");
        let (demand, config) = bottleneck_values();
        let result = compile_scenario_from_values(dir.join("network.geojson"), demand, config);
        assert!(
            result.is_ok(),
            "should compile bottleneck: {:?}",
            result.err()
        );
        let s = result.unwrap();
        assert!(!s.nodes.is_empty(), "should have nodes");
        assert!(!s.links.is_empty(), "should have links");
        assert!(!s.demand.is_empty(), "should have demand");
    }

    #[test]
    fn compile_nodes_and_links_sorted_by_id() {
        let dir = scenario_dir("bottleneck");
        let (demand, config) = bottleneck_values();
        let s = compile_scenario_from_values(dir.join("network.geojson"), demand, config).unwrap();

        for (i, node) in s.nodes.iter().enumerate() {
            assert_eq!(
                node.id, i,
                "node at position {} has id {} (NodeID == index invariant violated)",
                i, node.id
            );
        }
        for (i, link) in s.links.iter().enumerate() {
            assert_eq!(link.id, i, "link at position {} has id {}", i, link.id);
        }
    }

    #[test]
    fn compile_topology_correctly_inferred() {
        let dir = scenario_dir("bottleneck");
        let (demand, config) = bottleneck_values();
        let s = compile_scenario_from_values(dir.join("network.geojson"), demand, config).unwrap();

        // Every link should be registered in its upstream node's outgoing_links
        // and its downstream node's incoming_links.
        for link in &s.links {
            let up = &s.nodes[link.node_up];
            let down = &s.nodes[link.node_down];
            assert!(
                up.outgoing_links.contains(&link.id),
                "link {} missing from node {} outgoing_links",
                link.id,
                link.node_up
            );
            assert!(
                down.incoming_links.contains(&link.id),
                "link {} missing from node {} incoming_links",
                link.id,
                link.node_down
            );
        }
    }

    #[test]
    fn compile_fails_on_missing_network_file() {
        let (demand, config) = bottleneck_values();
        let result = compile_scenario_from_values(
            std::path::PathBuf::from("nonexistent_network.geojson"),
            demand,
            config,
        );
        assert!(result.is_err());
    }

    #[test]
    fn compile_fails_on_invalid_demand_shape() {
        let dir = scenario_dir("bottleneck");
        let (_, config) = bottleneck_values();
        // Demand must be a JSON array; an object should be rejected.
        let bad_demand = serde_json::json!({ "origin": 0, "destination": 1 });
        let result = compile_scenario_from_values(dir.join("network.geojson"), bad_demand, config);
        assert!(result.is_err());
    }

    #[test]
    fn load_scenario_round_trips() {
        // load_scenario reads a pre-compiled scenario JSON — use the bottleneck fixture
        // (last_run.json is not a full Scenario, so we compile and round-trip via serde).
        let dir = scenario_dir("bottleneck");
        let (demand, config) = bottleneck_values();
        let original =
            compile_scenario_from_values(dir.join("network.geojson"), demand, config).unwrap();

        let tmp = tempfile_path();
        let f = std::fs::File::create(&tmp).unwrap();
        serde_json::to_writer(f, &original).unwrap();

        let loaded = load_scenario(&tmp).expect("round-trip load should succeed");
        assert_eq!(loaded.nodes.len(), original.nodes.len());
        assert_eq!(loaded.links.len(), original.links.len());
        assert_eq!(loaded.demand.len(), original.demand.len());
        assert_eq!(loaded.start_time, original.start_time);
        assert_eq!(loaded.duration, original.duration);

        std::fs::remove_file(&tmp).ok();
    }

    fn tempfile_path() -> std::path::PathBuf {
        std::env::temp_dir().join(format!("stream_io_test_{}.json", std::process::id()))
    }
}

#[cfg(test)]
mod pipe_io_tests {
    use super::*;

    fn write_temp_network(pipes_prop: &str) -> std::path::PathBuf {
        let json = format!(
            r#"{{
            "type": "FeatureCollection",
            "features": [
                {{"type": "Feature", "geometry": {{"type": "Point", "coordinates": [0, 0]}},
                  "properties": {{"id": 0, "type": "Entry"}}}},
                {{"type": "Feature", "geometry": {{"type": "Point", "coordinates": [1, 0]}},
                  "properties": {{"id": 1, "type": "Exit"}}}},
                {{"type": "Feature", "geometry": {{"type": "LineString", "coordinates": [[0,0],[1,0]]}},
                  "properties": {{"id": 0, "node_up": 0, "node_down": 1, "length": 300.0,
                                  "speed": 25.0, "lanes": 3, "capacity": 1.5,
                                  "friction": 0.8,
                                  "moves": {{"7": [0]}},
                                  {pipes} }}}}
            ]
        }}"#,
            pipes = pipes_prop
        );
        let path = std::env::temp_dir().join(format!(
            "stream_pipes_io_test_{}_{}.geojson",
            std::process::id(),
            pipes_prop.len()
        ));
        std::fs::write(&path, json).unwrap();
        path
    }

    fn config_with_classes() -> serde_json::Value {
        serde_json::json!({"duration": 600, "classes": ["car", "bus"]})
    }

    #[test]
    fn parses_pipes_moves_friction_and_classes() {
        let path =
            write_temp_network(r#""pipes": [{"lanes": 1, "classes": ["bus"]}, {"lanes": 2}]"#);
        let s = compile_scenario_from_values(&path, serde_json::json!([]), config_with_classes())
            .expect("network with pipes should compile");
        std::fs::remove_file(&path).ok();

        assert_eq!(s.classes, vec!["car", "bus"]);
        let l = &s.links[0];
        assert_eq!(l.pipe_specs.len(), 2);
        assert_eq!(l.pipe_specs[0].lanes, 1);
        assert_eq!(l.pipe_specs[0].class_mask, 0b10, "bus is class index 1");
        assert_eq!(l.pipe_specs[1].lanes, 2);
        assert_eq!(l.pipe_specs[1].class_mask, ALL_CLASSES);
        assert_eq!(l.moves.get(&7), Some(&vec![0u8]));
        assert!((l.friction - 0.8).abs() < 1e-12);
    }

    #[test]
    fn unknown_pipe_class_is_rejected() {
        let path = write_temp_network(r#""pipes": [{"lanes": 3, "classes": ["tramway"]}]"#);
        let res = compile_scenario_from_values(&path, serde_json::json!([]), config_with_classes());
        std::fs::remove_file(&path).ok();
        let err = res.expect_err("unknown class must be rejected").to_string();
        assert!(
            err.contains("tramway"),
            "error should name the class: {err}"
        );
    }

    #[test]
    fn demand_class_is_parsed() {
        let path = write_temp_network(r#""pipes": [{"lanes": 3}]"#);
        let demand = serde_json::json!([
            {"period_start": 0, "period_end": 60, "origin": 0, "destination": 1,
             "flow": 60.0, "count": 1.0, "class": "bus"}
        ]);
        let s = compile_scenario_from_values(&path, demand, config_with_classes()).unwrap();
        std::fs::remove_file(&path).ok();
        assert_eq!(s.demand[0].class.as_deref(), Some("bus"));
    }
}
