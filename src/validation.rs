use crate::model::{Link, LinkID, NodeID, NodeType, Scenario};
use anyhow::{bail, Result};
use std::collections::{HashMap, HashSet};

pub trait Validate {
    fn validate(&self) -> Result<()>;
}

/// Resolve an optional class name against `classes`, defaulting to class `0`.
pub(crate) fn resolve_class_id(
    classes: &[String],
    class: Option<&str>,
    context: &str,
) -> Result<usize> {
    match class {
        Some(name) => classes.iter().position(|c| c == name).ok_or_else(|| {
            anyhow::anyhow!(
                "{}: unknown vehicle class '{}' (declared classes: {:?})",
                context,
                name,
                classes
            )
        }),
        None => Ok(0),
    }
}

/// Validate that `path` connects `origin` to `destination` through known links
/// and has at least one pipe usable by `class_id` on every link/movement.
pub(crate) fn validate_route(
    label: &str,
    origin: NodeID,
    destination: NodeID,
    class_id: usize,
    class_label: &str,
    path: &[LinkID],
    links_by_id: &HashMap<LinkID, &Link>,
    node_ids: &HashSet<NodeID>,
    allow_empty: bool,
) -> Result<()> {
    if !node_ids.contains(&origin) {
        bail!("{}: missing Origin Node {}", label, origin);
    }
    if !node_ids.contains(&destination) {
        bail!("{}: missing Destination Node {}", label, destination);
    }
    if path.is_empty() {
        if allow_empty {
            return Ok(());
        }
        bail!("{}: explicit path must not be empty", label);
    }

    for &lid in path {
        if !links_by_id.contains_key(&lid) {
            bail!("{}: path references missing Link {}", label, lid);
        }
    }

    // Connected path from the origin to the destination: the engine
    // moves a vehicle onto its next link at the node where it stands,
    // so a gap in the path would teleport it.
    let first = path[0];
    let last = *path.last().expect("non-empty path");
    if links_by_id[&first].node_up != origin {
        bail!(
            "{}: path starts at node {} (link {}) but its origin is node {}",
            label,
            links_by_id[&first].node_up,
            first,
            origin
        );
    }
    if links_by_id[&last].node_down != destination {
        bail!(
            "{}: path ends at node {} (link {}) but its destination is node {}",
            label,
            links_by_id[&last].node_down,
            last,
            destination
        );
    }
    for w in path.windows(2) {
        if links_by_id[&w[0]].node_down != links_by_id[&w[1]].node_up {
            bail!(
                "{}: path is not connected: link {} ends at node {} but link {} starts at node {}",
                label,
                w[0],
                links_by_id[&w[0]].node_down,
                w[1],
                links_by_id[&w[1]].node_up
            );
        }
    }

    // Class/movement reachability: every path link must have a pipe
    // usable by the class for its onward movement, otherwise vehicles
    // would silently gridlock their pipe at run time.
    for (i, &lid) in path.iter().enumerate() {
        let next = path.get(i + 1).copied();
        if !links_by_id[&lid].serves(class_id, next) {
            bail!(
                "{} (class '{}'): link {} has no pipe usable by this class{}",
                label,
                class_label,
                lid,
                next.map_or(String::new(), |n| format!(" for the movement toward link {}", n)),
            );
        }
    }
    Ok(())
}

impl Validate for Scenario {
    fn validate(&self) -> Result<()> {
        // 0. Ids index the engine's vectors directly: they must be 0..n in
        // order (io sorts them; gaps or duplicates would silently send a
        // vehicle onto the wrong link).
        for (i, node) in self.nodes.iter().enumerate() {
            if node.id != i {
                bail!(
                    "Node ids must be 0..{} in order: node at position {} has id {}",
                    self.nodes.len(),
                    i,
                    node.id
                );
            }
        }
        for (i, link) in self.links.iter().enumerate() {
            if link.id != i {
                bail!(
                    "Link ids must be 0..{} in order: link at position {} has id {}",
                    self.links.len(),
                    i,
                    link.id
                );
            }
        }

        // 1. Validate Nodes
        let mut node_ids = HashSet::new();
        for node in &self.nodes {
            if !node_ids.insert(node.id) {
                bail!("Duplicate Node ID: {}", node.id);
            }

            // Verify Type vs Connectivity
            match node.node_type {
                NodeType::Entry => {
                    if !node.incoming_links.is_empty() {
                        bail!("Entry Node {} has incoming links", node.id);
                    }
                }
                NodeType::Exit => {
                    if !node.outgoing_links.is_empty() {
                        bail!("Exit Node {} has outgoing links", node.id);
                    }
                }
                NodeType::Internal => {
                    // Generally internal nodes have both, but dead-ends might exist in valid subgraphs?
                    // Let's strict check for now
                    if node.incoming_links.is_empty() && node.outgoing_links.is_empty() {
                        // Isolated node?
                    }
                }
            }
            if let Some(spec) = &node.roundabout {
                if node.node_type != NodeType::Internal {
                    bail!("Roundabout node {} must be an internal node", node.id);
                }
                crate::roundabout::validate_spec(spec, node)?;
            }
            if let Some(spec) = &node.junction {
                if node.node_type != NodeType::Internal || node.roundabout.is_some() {
                    bail!("Junction node {} must be an ordinary internal node", node.id);
                }
                crate::junction::validate_spec(spec, node)?;
            }
        }

        // 2. Validate Links
        let mut link_ids = HashSet::new();
        for link in &self.links {
            if !link_ids.insert(link.id) {
                bail!("Duplicate Link ID: {}", link.id);
            }

            if !node_ids.contains(&link.node_up) {
                bail!(
                    "Link {} references missing Upstream Node {}",
                    link.id,
                    link.node_up
                );
            }
            if !node_ids.contains(&link.node_down) {
                bail!(
                    "Link {} references missing Downstream Node {}",
                    link.id,
                    link.node_down
                );
            }

            if link.speed <= 0.0 {
                bail!("Link {} has invalid speed {}", link.id, link.speed);
            }
            if link.capacity <= 0.0 {
                bail!("Link {} has invalid capacity {}", link.id, link.capacity);
            }
            if link.num_lanes == 0 {
                bail!("Link {} has 0 lanes", link.id);
            }

            // Pipe partition
            if !link.pipe_specs.is_empty() {
                let lane_sum: u32 = link.pipe_specs.iter().map(|p| u32::from(p.lanes)).sum();
                if lane_sum != u32::from(link.num_lanes) {
                    bail!(
                        "Link {}: pipe lanes sum to {} but the link has {} lanes",
                        link.id,
                        lane_sum,
                        link.num_lanes
                    );
                }
                if link.pipe_specs.iter().any(|p| p.lanes == 0) {
                    bail!("Link {} has a pipe with 0 lanes", link.id);
                }
                if link.pipe_specs.iter().any(|p| p.class_mask == 0) {
                    bail!("Link {} has a pipe that allows no vehicle class", link.id);
                }
                if link.pipe_specs.len() > 255 {
                    bail!("Link {} has more than 255 pipes", link.id);
                }
            }
            let n_pipes = link.pipe_specs.len().max(1);
            for (&out_link, pipe_idxs) in &link.moves {
                if pipe_idxs.iter().any(|&p| usize::from(p) >= n_pipes) {
                    bail!(
                        "Link {}: movement to link {} references pipe index out of range (link has {} pipes)",
                        link.id, out_link, n_pipes
                    );
                }
                if pipe_idxs.is_empty() {
                    bail!(
                        "Link {}: movement to link {} allows no pipe",
                        link.id,
                        out_link
                    );
                }
            }
            if !(link.friction > 0.0 && link.friction <= 1.0) {
                bail!(
                    "Link {} has friction {} outside (0, 1]",
                    link.id,
                    link.friction
                );
            }
        }

        // Movement keys must be actual out-links of the downstream node
        for link in &self.links {
            if link.moves.is_empty() {
                continue;
            }
            let Some(down) = self.nodes.iter().find(|n| n.id == link.node_down) else {
                continue; // missing node already reported above
            };
            for &out_link in link.moves.keys() {
                if !down.outgoing_links.contains(&out_link) {
                    bail!(
                        "Link {}: movement key {} is not an outgoing link of node {}",
                        link.id,
                        out_link,
                        link.node_down
                    );
                }
            }
        }

        if self.classes.is_empty() || self.classes.len() > 64 {
            bail!(
                "Scenario must define between 1 and 64 vehicle classes (got {})",
                self.classes.len()
            );
        }

        let links_by_id: HashMap<LinkID, &Link> =
            self.links.iter().map(|l| (l.id, l)).collect();

        // 3. Validate Vehicles
        for veh in &self.vehicles {
            let class_label = self
                .classes
                .get(veh.class_id)
                .map(String::as_str)
                .unwrap_or("unknown");
            validate_route(
                &format!("Vehicle {}", veh.id),
                veh.origin,
                veh.destination,
                veh.class_id,
                class_label,
                &veh.path,
                &links_by_id,
                &node_ids,
                true,
            )?;
        }

        // 3b. Validate explicit routes on route-based demand entries
        for (i, dem) in self.demand.iter().enumerate() {
            if let Some(path) = &dem.path {
                let label = format!("Demand {} ({}→{})", i, dem.origin, dem.destination);
                let class_id = resolve_class_id(&self.classes, dem.class.as_deref(), &label)?;
                let class_label = &self.classes[class_id];
                validate_route(
                    &label,
                    dem.origin,
                    dem.destination,
                    class_id,
                    class_label,
                    path,
                    &links_by_id,
                    &node_ids,
                    false,
                )?;
            }
        }

        // Signals, plans and tolls (empty without signals.json).
        if !self.signals.is_empty() {
            crate::signals::validate(&self.signals, self)?;
        }

        // 4. Validate the dynamic-patch schedule (empty for unpatched runs)
        for (i, change) in self.link_schedule.iter().enumerate() {
            let what = format!("Schedule entry {} (link {})", i, change.link_id);
            if !change.time.is_finite() {
                bail!("{}: non-finite time", what);
            }
            let Some(link) = self.links.iter().find(|l| l.id == change.link_id) else {
                bail!("{}: unknown link", what);
            };
            let n_pipes = link.pipe_specs.len().max(1);
            let a = &change.attrs;
            if let Some(masks) = &a.class_masks {
                if masks.len() != n_pipes {
                    bail!(
                        "{}: class_masks length {} != pipe count {}",
                        what,
                        masks.len(),
                        n_pipes
                    );
                }
            }
            if let Some(specs) = &a.pipe_specs {
                if specs.len() != n_pipes {
                    bail!(
                        "{}: pipe count must not change mid-run ({} → {})",
                        what,
                        n_pipes,
                        specs.len()
                    );
                }
                if specs.iter().any(|p| p.lanes == 0) {
                    bail!("{}: a pipe has 0 lanes", what);
                }
            }
            if a.num_lanes == Some(0) {
                bail!("{}: num_lanes must be ≥ 1", what);
            }
            if a.speed.is_some_and(|v| v <= 0.0) {
                bail!("{}: speed must be > 0", what);
            }
            if a.capacity.is_some_and(|v| v <= 0.0) {
                bail!("{}: capacity must be > 0", what);
            }
            if a.friction.is_some_and(|v| !(v > 0.0 && v <= 1.0)) {
                bail!("{}: friction outside (0, 1]", what);
            }
        }
        for &lid in &self.assignment_excluded {
            if !link_ids.contains(&lid) {
                bail!("assignment_excluded references missing link {}", lid);
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Demand, FundamentalDiagram, Link, Node, NodeType, Scenario, Vehicle};

    fn fd() -> FundamentalDiagram {
        FundamentalDiagram {
            u: 10.0,
            w: 5.0,
            kx: 0.2,
            c: 1.0,
        }
    }

    /// Entry(0) → [Link 0] → Internal(1) → [Link 1] → Exit(2)
    fn valid_scenario() -> Scenario {
        Scenario {
            nodes: vec![
                Node {
                    id: 0,
                    node_type: NodeType::Entry,
                    incoming_links: vec![],
                    outgoing_links: vec![0],
                    points: (0.0, 0.0),
                    signals: vec![], roundabout: None, junction: None,
                },
                Node {
                    id: 1,
                    node_type: NodeType::Internal,
                    incoming_links: vec![0],
                    outgoing_links: vec![1],
                    points: (1.0, 0.0),
                    signals: vec![], roundabout: None, junction: None,
                },
                Node {
                    id: 2,
                    node_type: NodeType::Exit,
                    incoming_links: vec![1],
                    outgoing_links: vec![],
                    points: (2.0, 0.0),
                    signals: vec![], roundabout: None, junction: None,
                },
            ],
            links: vec![
                Link::new(0, 0, 1, 1000.0, 10.0, 1, 1.0, fd(), vec![]),
                Link::new(1, 1, 2, 1000.0, 10.0, 1, 1.0, fd(), vec![]),
            ],
            vehicles: vec![],
            demand: vec![],
            start_time: 0.0,
            duration: 3600.0,
            classes: vec!["car".to_string()],
            link_schedule: Vec::new(),
            assignment_excluded: Vec::new(), signals: Default::default(), signal_log: Vec::new(),
        }
    }

    #[test]
    fn valid_scenario_passes() {
        assert!(valid_scenario().validate().is_ok());
    }

    #[test]
    fn duplicate_node_id_fails() {
        let mut s = valid_scenario();
        s.nodes[1].id = 0;
        assert!(s.validate().is_err());
    }

    #[test]
    fn duplicate_link_id_fails() {
        let mut s = valid_scenario();
        s.links[1].id = 0;
        assert!(s.validate().is_err());
    }

    #[test]
    fn entry_node_with_incoming_link_fails() {
        let mut s = valid_scenario();
        s.nodes[0].incoming_links.push(99);
        assert!(s.validate().is_err());
    }

    #[test]
    fn exit_node_with_outgoing_link_fails() {
        let mut s = valid_scenario();
        s.nodes[2].outgoing_links.push(99);
        assert!(s.validate().is_err());
    }

    #[test]
    fn link_referencing_missing_node_fails() {
        let mut s = valid_scenario();
        s.links[0].node_up = 99;
        assert!(s.validate().is_err());
    }

    #[test]
    fn link_with_zero_speed_fails() {
        let mut s = valid_scenario();
        s.links[0].speed = 0.0;
        assert!(s.validate().is_err());
    }

    #[test]
    fn link_with_negative_capacity_fails() {
        let mut s = valid_scenario();
        s.links[0].capacity = -1.0;
        assert!(s.validate().is_err());
    }

    #[test]
    fn link_with_zero_lanes_fails() {
        let mut s = valid_scenario();
        s.links[0].num_lanes = 0;
        assert!(s.validate().is_err());
    }

    #[test]
    fn vehicle_with_missing_link_in_path_fails() {
        let mut s = valid_scenario();
        s.vehicles.push(Vehicle::new(0, 0, vec![99], 0.0, 0, 2));
        assert!(s.validate().is_err());
    }

    #[test]
    fn scenario_with_no_vehicles_passes() {
        let mut s = valid_scenario();
        s.demand.push(Demand {
            period_start: 0.0,
            period_end: 3600.0,
            origin: 0,
            destination: 2,
            flow: 100.0,
            count: 100.0,
            class: None,
            path: None,
        });
        assert!(s.validate().is_ok());
    }

    #[test]
    fn route_based_demand_validates_connectivity_and_endpoints() {
        let mut s = valid_scenario();
        s.demand.push(Demand {
            period_start: 0.0,
            period_end: 3600.0,
            origin: 0,
            destination: 2,
            flow: 100.0,
            count: 100.0,
            class: None,
            path: Some(vec![0, 1]),
        });
        assert!(s.validate().is_ok());

        // Empty path rejected
        s.demand[0].path = Some(vec![]);
        assert!(s.validate().is_err());

        // Wrong origin rejected
        s.demand[0].path = Some(vec![1]);
        assert!(s.validate().is_err());

        // Missing link rejected
        s.demand[0].path = Some(vec![0, 99]);
        assert!(s.validate().is_err());

        // Unknown class rejected
        s.demand[0].path = Some(vec![0, 1]);
        s.demand[0].class = Some("tram".to_string());
        assert!(s.validate().is_err());
    }
}
