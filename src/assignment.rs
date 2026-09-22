use crate::model::{LinkID, NodeID, Scenario, Vehicle};
use petgraph::algo::astar;
use petgraph::graph::{DiGraph, NodeIndex};
use std::collections::HashMap;

pub struct NetworkGraph {
    pub graph: DiGraph<NodeID, (LinkID, f64)>, // NodeID, (LinkID, Cost/TravelTime)
    pub node_map: HashMap<NodeID, NodeIndex>,
}

impl NetworkGraph {
    pub fn from_scenario(scenario: &Scenario) -> Self {
        let mut graph = DiGraph::new();
        let mut node_map = HashMap::new();

        // Add Nodes
        for node in &scenario.nodes {
            let idx = graph.add_node(node.id);
            node_map.insert(node.id, idx);
        }

        // Add Edges — links closed for the whole horizon by a patch are
        // excluded so routing never uses them ("removal" without renumbering).
        for link in &scenario.links {
            if scenario.assignment_excluded.contains(&link.id) {
                continue;
            }
            if let (Some(&u), Some(&v)) =
                (node_map.get(&link.node_up), node_map.get(&link.node_down))
            {
                let cost = link.length / link.speed; // Free flow travel time
                graph.add_edge(u, v, (link.id, cost));
            }
        }

        Self { graph, node_map }
    }

    pub fn shortest_path(&self, from: NodeID, to: NodeID) -> Option<(f64, Vec<LinkID>)> {
        let start_idx = *self.node_map.get(&from)?;
        let end_idx = *self.node_map.get(&to)?;

        // Use A* (heuristic = 0 for Dijkstra)
        let path_res = astar(
            &self.graph,
            start_idx,
            |finish| finish == end_idx,
            |e| e.weight().1, // Cost
            |_| 0.0,          // Heuristic (can assume 0 if no geometry info ready)
        );

        if let Some((cost, nodes)) = path_res {
            // Convert node sequence to link sequence
            let mut links = Vec::new();
            for i in 0..nodes.len() - 1 {
                let u = nodes[i];
                let v = nodes[i + 1];
                let edge = self.graph.find_edge(u, v).unwrap();
                links.push(self.graph[edge].0);
            }
            Some((cost, links))
        } else {
            None
        }
    }
}

pub fn assign_demand(scenario: &mut Scenario) -> anyhow::Result<usize> {
    let graph = NetworkGraph::from_scenario(scenario);
    let mut new_vehicles = Vec::new();
    let mut veh_id_counter = scenario.vehicles.len();

    // Process existing vehicles (re-route)
    for veh in &mut scenario.vehicles {
        if let Some((_, path)) = graph.shortest_path(veh.origin, veh.destination) {
            veh.path = path;
        }
    }

    // Process Demand
    for dem in &scenario.demand {
        if dem.count <= 0.0 {
            continue;
        }

        // Resolve the vehicle class name against the declared classes.
        let class_id = match &dem.class {
            Some(name) => scenario
                .classes
                .iter()
                .position(|c| c == name)
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "Demand {}→{}: unknown vehicle class '{}' (declared classes: {:?})",
                        dem.origin,
                        dem.destination,
                        name,
                        scenario.classes
                    )
                })?,
            None => 0,
        };

        // Calculate Path once per OD pair (Static Assignment)
        if let Some((_, path)) = graph.shortest_path(dem.origin, dem.destination) {
            // Class/movement reachability: every link of the path must have at
            // least one pipe usable by this class for its onward movement.
            // (Routing itself is class-blind — class-aware shortest paths are
            // a documented follow-up; this turns silent gridlock into an error.)
            for (i, &lid) in path.iter().enumerate() {
                let next = path.get(i + 1).copied();
                let ok = scenario
                    .links
                    .iter()
                    .find(|l| l.id == lid)
                    .is_some_and(|l| l.serves(class_id, next));
                if !ok {
                    anyhow::bail!(
                        "Demand {}→{} (class '{}'): link {} has no pipe usable by this class \
                         for its movement{} — adjust pipe classes/moves or the route",
                        dem.origin,
                        dem.destination,
                        dem.class.as_deref().unwrap_or(&scenario.classes[0]),
                        lid,
                        next.map_or(String::new(), |n| format!(" toward link {}", n)),
                    );
                }
            }

            let duration = dem.period_end - dem.period_start;
            if duration <= 0.0 {
                continue;
            }

            let count = dem.count as usize;
            let headway = duration / (count as f64);

            for k in 0..count {
                let start_time = dem.period_start + (k as f64) * headway;
                new_vehicles.push(Vehicle::new(
                    veh_id_counter,
                    class_id,
                    path.clone(),
                    start_time,
                    dem.origin,
                    dem.destination,
                ));
                veh_id_counter += 1;
            }
        } else {
            eprintln!(
                "Warning: No path found for Demand {} -> {}",
                dem.origin, dem.destination
            );
        }
    }

    let added = new_vehicles.len();
    scenario.vehicles.extend(new_vehicles);
    Ok(added)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Demand, FundamentalDiagram, Link, Node, NodeType, Scenario};

    fn fd() -> FundamentalDiagram {
        FundamentalDiagram {
            u: 10.0,
            w: 5.0,
            kx: 0.2,
            c: 1.0,
        }
    }

    /// Entry(0) → [Link 0] → Internal(1) → [Link 1] → Exit(2)
    fn chain_scenario() -> Scenario {
        Scenario {
            nodes: vec![
                Node {
                    id: 0,
                    node_type: NodeType::Entry,
                    incoming_links: vec![],
                    outgoing_links: vec![0],
                    points: (0.0, 0.0),
                    signals: vec![],
                },
                Node {
                    id: 1,
                    node_type: NodeType::Internal,
                    incoming_links: vec![0],
                    outgoing_links: vec![1],
                    points: (1.0, 0.0),
                    signals: vec![],
                },
                Node {
                    id: 2,
                    node_type: NodeType::Exit,
                    incoming_links: vec![1],
                    outgoing_links: vec![],
                    points: (2.0, 0.0),
                    signals: vec![],
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
            assignment_excluded: Vec::new(),
        }
    }

    #[test]
    fn graph_has_correct_node_and_edge_count() {
        let s = chain_scenario();
        let g = NetworkGraph::from_scenario(&s);
        assert_eq!(g.graph.node_count(), 3);
        assert_eq!(g.graph.edge_count(), 2);
    }

    #[test]
    fn shortest_path_on_chain_uses_both_links() {
        let s = chain_scenario();
        let g = NetworkGraph::from_scenario(&s);
        let (_, links) = g
            .shortest_path(0, 2)
            .expect("path from 0 to 2 should exist");
        assert_eq!(links, vec![0, 1]);
    }

    #[test]
    fn shortest_path_cost_equals_sum_of_travel_times() {
        let s = chain_scenario();
        let g = NetworkGraph::from_scenario(&s);
        // Each link: length=1000m, speed=10m/s → travel_time=100s
        let (cost, _) = g.shortest_path(0, 2).unwrap();
        assert!(
            (cost - 200.0).abs() < 1e-9,
            "expected cost 200.0, got {}",
            cost
        );
    }

    #[test]
    fn no_path_on_reverse_direction_returns_none() {
        let s = chain_scenario();
        let g = NetworkGraph::from_scenario(&s);
        // The graph is directed Entry→Exit; reverse has no path
        assert!(g.shortest_path(2, 0).is_none());
    }

    #[test]
    fn assign_demand_generates_correct_vehicle_count() {
        let mut s = chain_scenario();
        s.demand.push(Demand {
            period_start: 0.0,
            period_end: 3600.0,
            origin: 0,
            destination: 2,
            flow: 100.0,
            count: 100.0,
            class: None,
        });
        let added = assign_demand(&mut s).expect("assignment should succeed");
        assert_eq!(added, 100);
        assert_eq!(s.vehicles.len(), 100);
    }

    #[test]
    fn assign_demand_vehicles_follow_shortest_path() {
        let mut s = chain_scenario();
        s.demand.push(Demand {
            period_start: 0.0,
            period_end: 3600.0,
            origin: 0,
            destination: 2,
            flow: 5.0,
            count: 5.0,
            class: None,
        });
        assign_demand(&mut s).expect("assignment should succeed");
        for v in &s.vehicles {
            assert_eq!(v.path, vec![0, 1]);
            assert_eq!(v.origin, 0);
            assert_eq!(v.destination, 2);
        }
    }

    #[test]
    fn assign_demand_vehicles_evenly_distributed_in_time() {
        let mut s = chain_scenario();
        s.demand.push(Demand {
            period_start: 0.0,
            period_end: 100.0,
            origin: 0,
            destination: 2,
            flow: 10.0,
            count: 10.0,
            class: None,
        });
        assign_demand(&mut s).expect("assignment should succeed");
        // headway = 100s / 10 = 10s per vehicle
        for (i, v) in s.vehicles.iter().enumerate() {
            let expected = i as f64 * 10.0;
            assert!(
                (v.start_time - expected).abs() < 1e-9,
                "vehicle {}: start_time {:.2}, expected {:.2}",
                i,
                v.start_time,
                expected
            );
        }
    }

    #[test]
    fn assign_demand_skips_zero_count_entry() {
        let mut s = chain_scenario();
        s.demand.push(Demand {
            period_start: 0.0,
            period_end: 3600.0,
            origin: 0,
            destination: 2,
            flow: 0.0,
            count: 0.0,
            class: None,
        });
        let added = assign_demand(&mut s).expect("assignment should succeed");
        assert_eq!(added, 0);
        assert!(s.vehicles.is_empty());
    }
}

#[cfg(test)]
mod class_tests {
    use super::*;
    use crate::model::{Demand, FundamentalDiagram, Link, Node, NodeType, PipeSpec, Scenario};

    fn two_class_scenario() -> Scenario {
        let fd = FundamentalDiagram {
            u: 10.0,
            w: 5.0,
            kx: 0.2,
            c: 1.0,
        };
        Scenario {
            nodes: vec![
                Node {
                    id: 0,
                    node_type: NodeType::Entry,
                    incoming_links: vec![],
                    outgoing_links: vec![0],
                    points: (0.0, 0.0),
                    signals: vec![],
                },
                Node {
                    id: 1,
                    node_type: NodeType::Exit,
                    incoming_links: vec![0],
                    outgoing_links: vec![],
                    points: (1.0, 0.0),
                    signals: vec![],
                },
            ],
            links: vec![Link::new(0, 0, 1, 1000.0, 10.0, 2, 1.0, fd, vec![])],
            vehicles: vec![],
            demand: vec![],
            start_time: 0.0,
            duration: 3600.0,
            classes: vec!["car".to_string(), "bus".to_string()],
            link_schedule: Vec::new(),
            assignment_excluded: Vec::new(),
        }
    }

    fn demand(class: Option<&str>) -> Demand {
        Demand {
            period_start: 0.0,
            period_end: 60.0,
            origin: 0,
            destination: 1,
            flow: 60.0,
            count: 3.0,
            class: class.map(String::from),
        }
    }

    #[test]
    fn demand_class_resolves_to_class_id() {
        let mut s = two_class_scenario();
        s.demand.push(demand(Some("bus")));
        assign_demand(&mut s).unwrap();
        assert!(s.vehicles.iter().all(|v| v.class_id == 1));
    }

    #[test]
    fn unknown_demand_class_errors() {
        let mut s = two_class_scenario();
        s.demand.push(demand(Some("hovercraft")));
        let err = assign_demand(&mut s)
            .expect_err("unknown class")
            .to_string();
        assert!(err.contains("hovercraft"), "{err}");
    }

    #[test]
    fn class_locked_out_of_every_pipe_errors() {
        let mut s = two_class_scenario();
        // Both pipes car-only: a bus can never traverse link 0.
        s.links[0].pipe_specs = vec![
            PipeSpec {
                lanes: 1,
                class_mask: 0b01,
            },
            PipeSpec {
                lanes: 1,
                class_mask: 0b01,
            },
        ];
        s.demand.push(demand(Some("bus")));
        let err = assign_demand(&mut s)
            .expect_err("unreachable path")
            .to_string();
        assert!(err.contains("link 0"), "{err}");
    }
}
