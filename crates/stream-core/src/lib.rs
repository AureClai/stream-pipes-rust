//! Event-driven dynamic traffic network simulator using Newell's simplified
//! kinematic-wave (LWR) model and Daganzo's multi-pipe extension.
//!
//! ## Public Modules
//!
//! - [`api`] — stable JSON-in / JSON-out entry points (`check`, `compile`, `run`, `collect`).
//! - [`model`] — core network, pipe, vehicle, demand, and scenario data model.
//! - [`simulation`] — discrete-event kinematic-wave solver (`Simulation`).
//! - [`patch`] — non-destructive variant overlays (`"variant = base + patches"`).
//! - [`signals`] — signal groups, fixed-time plans, tolls, and the [`signals::Controller`] trait.
//! - [`verification`] — self-verification of simulated results against theoretical LWR laws.
//! - [`benchmarks`] — closed-form analytic LWR benchmark suite (`run_benchmarks`).
//! - [`mod@reference`] — comparison against observed loop/probe measurements (`compare_with_observations`).
//! - [`analysis`] / [`xt_analysis`] — post-run KPIs, CVCs, skims, and Newell/Edie XT diagrams.
//!
//! ## Quick Start
//!
//! ```
//! use stream_core::model::{
//!     FundamentalDiagram, Link, Node, NodeType, Scenario, Vehicle, VehicleState,
//! };
//! use stream_core::simulation::Simulation;
//! use stream_core::verification::verify_scenario;
//!
//! let fd = FundamentalDiagram { u: 10.0, w: 5.0, kx: 0.15, c: 0.5 };
//! let nodes = vec![
//!     Node {
//!         id: 0,
//!         node_type: NodeType::Entry,
//!         incoming_links: vec![],
//!         outgoing_links: vec![0],
//!         points: (0.0, 0.0),
//!         signals: vec![],
//!         roundabout: None,
//!         junction: None,
//!     },
//!     Node {
//!         id: 1,
//!         node_type: NodeType::Exit,
//!         incoming_links: vec![0],
//!         outgoing_links: vec![],
//!         points: (500.0, 0.0),
//!         signals: vec![],
//!         roundabout: None,
//!         junction: None,
//!     },
//! ];
//! let links = vec![Link::new(0, 0, 1, 500.0, 10.0, 1, 0.5, fd, vec![(0.0, 0.0), (500.0, 0.0)])];
//! let vehicles = vec![Vehicle {
//!     id: 0,
//!     class_id: 0,
//!     path: vec![0],
//!     start_time: 0.0,
//!     origin: 0,
//!     destination: 1,
//!     state: VehicleState::QueuedAtEntry,
//!     current_link_idx: 0,
//!     node_times: vec![],
//!     pipes_taken: vec![],
//!     ring_exits: vec![],
//! }];
//! let scenario = Scenario {
//!     nodes,
//!     links,
//!     vehicles,
//!     demand: vec![],
//!     start_time: 0.0,
//!     duration: 100.0,
//!     classes: vec!["car".into()],
//!     link_schedule: vec![],
//!     assignment_excluded: vec![],
//!     signals: Default::default(),
//!     signal_log: vec![],
//! };
//!
//! let mut sim = Simulation::new(scenario);
//! sim.run().unwrap();
//! assert_eq!(sim.scenario.vehicles[0].node_times, vec![0.0, 50.0]);
//! assert_eq!(verify_scenario(&sim.scenario, 60.0).summary.checks_fail, 0);
//! ```

pub mod analysis;
pub mod api;
pub mod assignment;
pub mod benchmarks;
pub mod diagnostics;
pub mod io;
pub mod junction;
pub mod model;
pub mod patch;
pub mod reference;
pub mod roundabout;
pub mod signals;
pub mod simulation;
pub mod validation;
pub mod verification;
pub mod xt_analysis;
