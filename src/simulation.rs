use crate::model::{
    Link, LinkID, NodeID, PipeIdx, Scenario, StorageAvailability, VehID, VehicleState,
};
use crate::roundabout::{Bound, PointKind, RingVehicle, RoundaboutState};
use crate::signals::{Aspect, Command, ControlView, Controller, SignalRuntime, SwitchRecord};
use crate::junction::JunctionState;
use anyhow::Result;
#[cfg(feature = "python")]
use pyo3::prelude::*;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum EventType {
    /// Dynamic patch boundary: apply `Scenario::link_schedule[veh_id]` to its
    /// link. System event — `veh_id` is the schedule index, not a vehicle.
    ApplyPatch,
    VehicleEntry,
    VehicleArrival,
    LinkExit,
    LinkReady,
    /// Roundabout wake-up: a ring constraint at point `pipe` of node
    /// `link_id` expires. `veh_id` = NO_VEHICLE.
    RingReady,
    /// A ring vehicle reaches point `pipe` of roundabout node `link_id`.
    RingArrival,
    /// Signal group `link_id` turns green (`pipe` = 1) or red (`pipe` = 0).
    SignalSwitch,
    /// External controller `link_id` asked to be woken now.
    ControlWake,
}

/// All four event types are (link, pipe)-addressed: `pipe` selects the LWR
/// stream within the link the event refers to (always 0 on single-pipe links).
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Event {
    pub time: f64,
    pub event_type: EventType,
    pub link_id: LinkID,
    pub veh_id: VehID,
    pub pipe: PipeIdx,
}

impl Eq for Event {}

impl EventType {
    fn order_key(self) -> u8 {
        match self {
            // Patches apply first among simultaneous events so the new
            // physics governs everything at that timestamp. Unpatched runs
            // contain no ApplyPatch events, so the relative order of the
            // other four keys — and the golden fixture checksums — are
            // untouched.
            // Ring events slot in without changing the relative order of the
            // other types: scenarios without a roundabout never emit them.
            // Controllers act, then switches apply, before any vehicle moves
            // at that instant. Scenarios without signals emit neither.
            EventType::ApplyPatch => 0,
            EventType::ControlWake => 1,
            EventType::SignalSwitch => 2,
            EventType::LinkReady => 3,
            EventType::RingReady => 4,
            EventType::LinkExit => 5,
            EventType::RingArrival => 6,
            EventType::VehicleArrival => 7,
            EventType::VehicleEntry => 8,
        }
    }
}

impl Ord for Event {
    /// Min-heap on time with deterministic tie-breaking so that simultaneous
    /// events — e.g. all demand released at t = 0 — are processed in a stable,
    /// reproducible order. At equal times, capacity-freeing events (LinkReady,
    /// LinkExit) are processed before capacity-consuming ones (arrivals,
    /// entries), then vehicle id preserves demand FIFO.
    ///
    /// `pipe` must remain the LAST key: single-pipe scenarios have pipe == 0
    /// everywhere, so every comparison — and therefore the whole heap order —
    /// is bit-identical to the single-stream engine (fixture gate).
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .time
            .partial_cmp(&self.time)
            .unwrap_or(Ordering::Equal)
            .then_with(|| {
                other
                    .event_type
                    .order_key()
                    .cmp(&self.event_type.order_key())
            })
            .then_with(|| other.veh_id.cmp(&self.veh_id))
            .then_with(|| other.link_id.cmp(&self.link_id))
            .then_with(|| other.pipe.cmp(&self.pipe))
    }
}

// ── Pipe choice (lane assignment) ─────────────────────────────────────────────

/// Outcome of trying to place a vehicle into a pipe of its next link.
#[derive(Debug, Clone, Copy, PartialEq)]
enum PipeChoice {
    /// This pipe admits the vehicle right now.
    AdmitNow(PipeIdx),
    /// No eligible pipe admits now; the earliest becomes feasible at this time.
    WakeAt(PipeIdx, f64),
    /// All eligible pipes are storage-Blocked (no release scheduled yet) —
    /// a future LinkExit on one of them re-triggers the transfer.
    Blocked,
    /// No pipe satisfies class ∧ movement. Scenario coding error (validation
    /// rejects this at compile time; kept as a runtime guard).
    NoEligible,
}

/// Pipes of `link` a vehicle of `class_id` may use, given its movement at the
/// downstream node (`next_link`; `None` = trip ends on this link, so only the
/// class filter applies). Ascending pipe index (keep-right preference).
fn eligible_pipes(link: &Link, class_id: usize, next_link: Option<LinkID>) -> Vec<PipeIdx> {
    let by_move = next_link.and_then(|nl| link.moves.get(&nl));
    (0..link.pipes.len() as u8)
        .filter(|&p| link.pipes[usize::from(p)].allows_class(class_id))
        .filter(|&p| by_move.is_none_or(|v| v.contains(&p)))
        .collect()
}

/// Same test as `!eligible_pipes(..).is_empty()`, without allocating.
fn has_eligible_pipe(link: &Link, class_id: usize, next_link: Option<LinkID>) -> bool {
    let by_move = next_link.and_then(|nl| link.moves.get(&nl));
    (0..link.pipes.len() as u8).any(|p| {
        link.pipes[usize::from(p)].allows_class(class_id) && by_move.is_none_or(|v| v.contains(&p))
    })
}

/// Choice point 1 — entry-node enqueue. Discretionary rule: least-occupied
/// eligible pipe (occupancy = vehicles on pipe + queued at entry, integer
/// compare), ties broken toward the lowest index (keep-right).
fn choose_pipe_for_entry(
    link: &Link,
    class_id: usize,
    next_link: Option<LinkID>,
) -> Option<PipeIdx> {
    eligible_pipes(link, class_id, next_link)
        .into_iter()
        .min_by_key(|&p| {
            let pipe = &link.pipes[usize::from(p)];
            pipe.vehicles_on_pipe + pipe.entry_queue.len()
        })
}

/// Choice point 2 — node transfer, late binding: evaluated at transfer time
/// against the pipes' current storage and headway clocks.
fn choose_pipe_for_transfer(
    link: &Link,
    class_id: usize,
    next_link: Option<LinkID>,
    now: f64,
) -> PipeChoice {
    let eligible = eligible_pipes(link, class_id, next_link);
    if eligible.is_empty() {
        return PipeChoice::NoEligible;
    }

    // Admittable right now → least occupied, keep-right ties.
    let admit_now = eligible
        .iter()
        .copied()
        .filter(|&p| {
            let pipe = &link.pipes[usize::from(p)];
            pipe.storage_availability(now) == StorageAvailability::Available
                && now >= pipe.next_available_entry_time
        })
        .min_by_key(|&p| link.pipes[usize::from(p)].vehicles_on_pipe);
    if let Some(p) = admit_now {
        return PipeChoice::AdmitNow(p);
    }

    // Otherwise: earliest feasible wake-up among pipes with a known time.
    let mut best: Option<(f64, PipeIdx)> = None;
    for &p in &eligible {
        let pipe = &link.pipes[usize::from(p)];
        let storage_time = match pipe.storage_availability(now) {
            StorageAvailability::Available => now,
            StorageAvailability::AvailableAt(t) => t,
            StorageAvailability::Blocked => continue,
        };
        let t = storage_time.max(pipe.next_available_entry_time);
        if best.is_none_or(|(bt, _)| t < bt) {
            best = Some((t, p));
        }
    }
    match best {
        Some((t, p)) => PipeChoice::WakeAt(p, t),
        None => PipeChoice::Blocked,
    }
}

// ── Merge rule ────────────────────────────────────────────────────────────────

/// How a node shares its outflow between incoming links that all have
/// vehicles waiting.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum MergeRule {
    /// Incoming links are tried in id order: under saturation the lowest id
    /// takes every slot it can use. The historical rule — and the default, so
    /// every existing scenario runs bit-identically.
    #[default]
    InOrder,
    /// Deterministic weighted fair merge: each incoming link counts the
    /// vehicles it passed while another incoming link was also waiting; the
    /// node tries first the link with the smallest count / capacity. Under
    /// sustained saturation the outflow splits in proportion to the incoming
    /// capacities (Daganzo's capacity-proportional merge); a link alone takes
    /// all the supply.
    CapacityProportional,
}

impl MergeRule {
    /// Read `"merge"` from a scenario config: `"in_order"` (default) or
    /// `"capacity"`.
    pub fn from_config(config: &serde_json::Value) -> Result<Self> {
        match config.get("merge").and_then(|v| v.as_str()) {
            None | Some("in_order") => Ok(MergeRule::InOrder),
            Some("capacity") => Ok(MergeRule::CapacityProportional),
            Some(other) => anyhow::bail!("unknown merge rule '{other}' (expected \"in_order\" or \"capacity\")"),
        }
    }
}

/// Placeholder vehicle id for system-originated events (LinkReady) that do not
/// reference a real vehicle — distinct from any valid index.
const NO_VEHICLE: VehID = VehID::MAX;

/// Sentinel veh_id marking a LinkReady as a *discharge-gate* wake-up
/// (friction / upstream-capacity), so `handle_link_ready` clears the matching
/// dedup marker and not the admission one. Participates in the existing
/// veh_id tie-break: at an identical (time, type), gate wakes order before
/// admission wakes (MAX − 1 < MAX) — a fixed, documented position. Runs with
/// φ = 1 and upstream_capacity = false never emit this sentinel, so the
/// single-stream event order is untouched (fixture gate).
const EXIT_WAKE: VehID = VehID::MAX - 1;

/// Effective friction multiplier for a pipe's discharge: when any *sibling*
/// pipe of the link has spilled back (storage constraint binding), the pipe
/// discharges at C·φ instead of C — the rubbernecking / late-inserter proxy.
/// φ = 1.0 (default) disables the coupling entirely.
fn friction_phi(link: &Link, pipe_idx: PipeIdx, now: f64) -> f64 {
    if link.friction >= 1.0 || link.pipes.len() < 2 {
        return 1.0;
    }
    let spilled_sibling = link
        .pipes
        .iter()
        .enumerate()
        .any(|(i, p)| i != usize::from(pipe_idx) && p.is_spilled(now));
    if spilled_sibling {
        link.friction
    } else {
        1.0
    }
}

impl PartialOrd for Event {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[cfg_attr(feature = "python", pyclass)]
pub struct Simulation {
    pub scenario: Scenario,
    pub events: BinaryHeap<Event>,
    pub current_time: f64,
    /// Apply the upstream supply term (1/C headway of the incoming link) at
    /// node transfers — Python's `General["ActiveUpStreamCapacity"]`.
    /// Defaults to `false`, matching the Python default.
    pub upstream_capacity: bool,
    /// How nodes share their outflow between waiting incoming links.
    pub merge_rule: MergeRule,
    /// Per link: vehicles it passed at its downstream node while another
    /// incoming link of that node was waiting (CapacityProportional only).
    merge_served: Vec<f64>,
    /// Ring state of every roundabout node, indexed by node id (None for
    /// ordinary nodes). Public so tests and verification can read the
    /// passages recorded at each ring point.
    pub roundabouts: Vec<Option<Box<RoundaboutState>>>,
    /// Signal groups and tolls at run time (empty without signals.json).
    pub signals: SignalRuntime,
    /// True when signals, tolls or controllers exist; false keeps every
    /// signal check off the hot path (and runs bit-identical).
    signals_active: bool,
    /// External controllers (each taken out while it runs).
    controllers: Vec<Option<Box<dyn Controller>>>,
    controllers_started: bool,
    /// Cumulative vehicles that entered / left each link (controller sensors).
    link_entered: Vec<u64>,
    link_exited: Vec<u64>,
    /// Priority-junction state per node id (None for other nodes).
    pub junctions: Vec<Option<Box<JunctionState>>>,
    junctions_active: bool,
}

#[cfg(feature = "python")]
#[pymethods]
impl Simulation {
    // TODO: Implement methods exposed to Python (load, run, etc.)
    // For now, just pure Rust methods or internal logic
}

impl Simulation {
    pub fn new(scenario: Scenario) -> Self {
        // NodeID must equal the Vec index: simulation logic indexes nodes directly by ID.
        // io::compile_scenario_from_values sorts nodes by ID before building the Scenario,
        // so this invariant holds for all normally-constructed scenarios.
        for (i, node) in scenario.nodes.iter().enumerate() {
            assert_eq!(
                node.id, i,
                "NodeID must equal Vec index: node at position {} has id {}. \
                 Ensure nodes are sorted and IDs are contiguous starting from 0.",
                i, node.id
            );
        }
        let n_links = scenario.links.len();
        let mut sim = Simulation {
            scenario,
            events: BinaryHeap::new(),
            current_time: 0.0,
            upstream_capacity: false,
            merge_rule: MergeRule::InOrder,
            merge_served: vec![0.0; n_links],
            roundabouts: Vec::new(),
            signals: SignalRuntime::default(),
            signals_active: false,
            controllers: Vec::new(),
            controllers_started: false,
            link_entered: vec![0; n_links],
            link_exited: vec![0; n_links],
            junctions: Vec::new(),
            junctions_active: false,
        };
        sim.junctions = sim
            .scenario
            .nodes
            .iter()
            .map(|n| n.junction.as_ref().map(|spec| Box::new(JunctionState::new(n, spec))))
            .collect();
        sim.junctions_active = sim.junctions.iter().any(Option::is_some);
        sim.signals = SignalRuntime::new(&sim.scenario.signals, &sim.scenario);
        sim.signals_active = !sim.signals.is_empty();
        sim.scenario.signal_log.clear();
        if sim.signals_active {
            sim.current_time = sim.scenario.start_time;
            sim.seed_signals();
        }
        sim.roundabouts = sim
            .scenario
            .nodes
            .iter()
            .map(|n| n.roundabout.as_ref().map(|spec| Box::new(RoundaboutState::new(n.id, spec))))
            .collect();
        // Build runtime pipes here (not in Link::new) so that FD parameters
        // adjusted after construction — as several tests do — are respected.
        for link in &mut sim.scenario.links {
            link.materialize_pipes();
        }
        sim.seed_patch_schedule();
        sim.initialize_events();
        sim
    }

    /// Apply schedule entries at/before the horizon start eagerly (initial
    /// patched state) and push the rest as `ApplyPatch` events. No-op when the
    /// schedule is empty — unpatched runs stay bit-identical (fixture gate).
    fn seed_patch_schedule(&mut self) {
        let start = self.scenario.start_time;
        let horizon_end = start + self.scenario.duration;
        for idx in 0..self.scenario.link_schedule.len() {
            let change = self.scenario.link_schedule[idx].clone();
            if change.link_id >= self.scenario.links.len() {
                continue;
            }
            if change.time <= start {
                let link = &mut self.scenario.links[change.link_id];
                link.apply_attrs(&change.attrs);
                link.refresh_pipes_in_place(change.attrs.class_masks.as_deref());
            } else if change.time <= horizon_end {
                self.events.push(Event {
                    time: change.time,
                    event_type: EventType::ApplyPatch,
                    link_id: change.link_id,
                    // Schedule index: unique, so the veh_id tie-break orders
                    // simultaneous patches deterministically by schedule order.
                    veh_id: idx,
                    pipe: 0,
                });
            }
        }
    }

    fn initialize_events(&mut self) {
        self.current_time = self.scenario.start_time;
        for (i, veh) in self.scenario.vehicles.iter_mut().enumerate() {
            veh.node_times = Vec::with_capacity(veh.path.len() + 1);
            if !veh.path.is_empty() {
                let link_idx = veh.path[0];
                // TODO: Ensure link_idx is valid
                self.events.push(Event {
                    time: veh.start_time,
                    event_type: EventType::VehicleEntry,
                    link_id: link_idx,
                    veh_id: i,
                    // Placeholder: the pipe is chosen when the entry fires.
                    pipe: 0,
                });
                veh.state = VehicleState::QueuedAtEntry;
            }
        }
    }

    pub fn run(&mut self) -> Result<usize> {
        self.run_with_progress(0, |_, _| true)
    }

    /// Same as `run`, but calls `on_progress(sim_time, events_processed)`
    /// every `every` events (never when `every` is 0). Returning `false`
    /// cancels the run, which then returns an error.
    /// The callback only observes, so a completed run is bit-identical to `run`.
    pub fn run_with_progress<F>(&mut self, every: usize, mut on_progress: F) -> Result<usize>
    where
        F: FnMut(f64, usize) -> bool,
    {
        let mut events_processed = 0;
        let max_time = self.scenario.start_time + self.scenario.duration;
        self.start_controllers();

        while let Some(event) = self.events.pop() {
            if event.time > max_time {
                break;
            }
            self.current_time = event.time;
            events_processed += 1;
            if every > 0 && events_processed % every == 0 && !on_progress(event.time, events_processed) {
                anyhow::bail!("simulation cancelled at t = {:.1} s", event.time);
            }

            match event.event_type {
                EventType::ApplyPatch => self.handle_apply_patch(event.veh_id),
                EventType::VehicleEntry => self.handle_vehicle_entry(event.link_id, event.veh_id),
                EventType::VehicleArrival => {
                    self.handle_vehicle_arrival(event.link_id, event.pipe, event.veh_id)
                }
                EventType::LinkExit => self.handle_link_exit(event.link_id, event.pipe),
                EventType::LinkReady => {
                    self.handle_link_ready(event.link_id, event.pipe, event.veh_id)
                }
                EventType::RingReady => self.handle_ring_ready(event.link_id, event.pipe, event.time),
                EventType::RingArrival => {
                    self.handle_ring_arrival(event.link_id, event.pipe, event.veh_id)
                }
                EventType::SignalSwitch => self.handle_signal_switch(event.link_id, Aspect::from_code(event.pipe)),
                EventType::ControlWake => self.call_controller(event.link_id, false),
            }
        }
        Ok(events_processed)
    }

    /// Earliest future schedule boundary for `link_id`, if any. The schedule
    /// is sorted by time, so the first match is the next change.
    fn next_schedule_time_for_link(&self, link_id: LinkID) -> Option<f64> {
        self.scenario
            .link_schedule
            .iter()
            .find(|c| c.link_id == link_id && c.time > self.current_time)
            .map(|c| c.time)
    }

    /// Apply schedule entry `schedule_idx` to its link mid-run, then rescan
    /// exactly like `handle_link_ready`: the new physics may unblock (or
    /// re-block) entries and transfers on both adjacent nodes.
    fn handle_apply_patch(&mut self, schedule_idx: usize) {
        let Some(change) = self.scenario.link_schedule.get(schedule_idx).cloned() else {
            return;
        };
        let link_id = change.link_id;
        if link_id >= self.scenario.links.len() {
            return;
        }

        {
            let link = &mut self.scenario.links[link_id];
            link.apply_attrs(&change.attrs);
            link.refresh_pipes_in_place(change.attrs.class_masks.as_deref());
            // Old markers reference wake-up times computed under the old
            // params: a duplicate LinkReady is harmless, a suppressed one
            // strands vehicles.
            for pipe in &mut link.pipes {
                pipe.last_scheduled_ready_time = None;
                pipe.last_scheduled_exit_ready_time = None;
            }
        }

        // Re-dispatch entry-queue vehicles whose class the pipe no longer
        // allows (they have not physically entered — only committed): each
        // re-runs the entry choice under the new masks, FIFO order preserved.
        // Vehicles already ON a pipe are never touched — they drain.
        let n_pipes = self.scenario.links[link_id].pipes.len();
        let mut redispatch: Vec<VehID> = Vec::new();
        {
            let Scenario {
                links, vehicles, ..
            } = &mut self.scenario;
            let link = &mut links[link_id];
            for pipe in &mut link.pipes {
                if pipe.entry_queue.is_empty() {
                    continue;
                }
                let queue = std::mem::take(&mut pipe.entry_queue);
                for veh_id in queue {
                    let allowed = vehicles
                        .get(veh_id)
                        .is_some_and(|v| pipe.allows_class(v.class_id));
                    if allowed {
                        pipe.entry_queue.push_back(veh_id);
                    } else {
                        redispatch.push(veh_id);
                    }
                }
            }
        }
        for veh_id in redispatch {
            self.handle_vehicle_entry(link_id, veh_id);
        }

        // Full rescan (the LinkReady pattern): entry queues, then transfers
        // feeding into and draining out of this link.
        for p in 0..n_pipes as PipeIdx {
            self.process_pipe_entry_queue(link_id, p);
        }
        let node_up = self.scenario.links[link_id].node_up;
        let node_down = self.scenario.links[link_id].node_down;
        self.process_node_transfer(node_up);
        self.process_node_transfer(node_down);
    }

    fn handle_vehicle_entry(&mut self, link_id: LinkID, veh_id: VehID) {
        if link_id >= self.scenario.links.len() {
            return;
        }
        if veh_id >= self.scenario.vehicles.len() {
            return;
        }

        // Choice point 1: commit the vehicle to a pipe at enqueue time.
        // Movement constraint: the pipe must serve the vehicle's next out-link.
        let (class_id, next_link) = {
            let v = &self.scenario.vehicles[veh_id];
            (v.class_id, v.path.get(1).copied())
        };
        let link = &self.scenario.links[link_id];
        let Some(p) = choose_pipe_for_entry(link, class_id, next_link) else {
            // No pipe satisfies class ∧ movement. With a dynamic patch
            // pending on this link (e.g. a closure window), retry when the
            // link next changes state; otherwise this is a scenario coding
            // error (validation rejects it) — the vehicle stays unserved and
            // shows up in the demand_served verification check.
            if let Some(t) = self.next_schedule_time_for_link(link_id) {
                self.events.push(Event {
                    time: t,
                    event_type: EventType::VehicleEntry,
                    link_id,
                    veh_id,
                    pipe: 0,
                });
            }
            return;
        };
        self.scenario.links[link_id].pipes[usize::from(p)]
            .entry_queue
            .push_back(veh_id);
        self.process_pipe_entry_queue(link_id, p);
    }

    fn process_pipe_entry_queue(&mut self, link_id: LinkID, pipe_idx: PipeIdx) {
        loop {
            let link = &mut self.scenario.links[link_id];
            let travel_time = link.length / link.speed;
            let pipe = &mut link.pipes[usize::from(pipe_idx)];
            let Some(&veh_id) = pipe.entry_queue.front() else {
                break;
            };
            if veh_id >= self.scenario.vehicles.len() {
                pipe.entry_queue.pop_front();
                continue;
            }

            // Demand may enter the pipe when both LWR supply terms hold:
            //  - storage:  entry(n) ≥ exit(n − dn) + L/w   (Newell / Lax-Hopf)
            //  - capacity: inflow headway ≥ 1/C
            let storage = pipe.storage_availability(self.current_time);
            let flow_ok = self.current_time >= pipe.next_available_entry_time;

            if storage == StorageAvailability::Available && flow_ok {
                pipe.entry_queue.pop_front();
                pipe.admit(self.current_time);
                self.link_entered[link_id] += 1;
                if pipe.capacity > 0.0 {
                    pipe.next_available_entry_time = self.current_time + 1.0 / pipe.capacity;
                }

                let veh = &mut self.scenario.vehicles[veh_id];
                veh.state = VehicleState::Moving;
                if veh.node_times.is_empty() {
                    // Passage time through the entry node = physical entry to
                    // the first link (matches Python's NodeTimes semantics).
                    veh.node_times.push(self.current_time);
                }
                veh.pipes_taken.push(pipe_idx);
                if self.junctions_active {
                    self.junction_expect(link_id, veh_id);
                }

                self.events.push(Event {
                    time: self.current_time + travel_time,
                    event_type: EventType::VehicleArrival,
                    link_id,
                    veh_id,
                    pipe: pipe_idx,
                });
            } else {
                // Blocked: wake up at the earliest time both constraints can hold.
                // If storage is Blocked (no slot released yet), a future
                // LinkExit will re-trigger this queue — no event to schedule.
                let mut ready = match storage {
                    StorageAvailability::AvailableAt(t) => Some(t),
                    StorageAvailability::Available => Some(self.current_time),
                    StorageAvailability::Blocked => None,
                };
                if let Some(r) = ready {
                    if !flow_ok {
                        ready = Some(r.max(pipe.next_available_entry_time));
                    }
                }
                if let Some(t) = ready.filter(|&t| t > self.current_time) {
                    if pipe.last_scheduled_ready_time != Some(t) {
                        pipe.last_scheduled_ready_time = Some(t);
                        self.events.push(Event {
                            time: t,
                            event_type: EventType::LinkReady,
                            link_id,
                            veh_id: NO_VEHICLE,
                            pipe: pipe_idx,
                        });
                    }
                }
                break;
            }
        }
    }

    fn handle_vehicle_arrival(&mut self, link_id: LinkID, pipe_idx: PipeIdx, veh_id: VehID) {
        if link_id >= self.scenario.links.len() {
            return;
        }
        self.scenario.links[link_id].pipes[usize::from(pipe_idx)]
            .waiting_queue
            .push_back(veh_id);
        let node_id = self.scenario.links[link_id].node_down;
        if self.junctions_active {
            if let Some(js) = self.junctions.get_mut(node_id).and_then(|j| j.as_deref_mut()) {
                let v = &self.scenario.vehicles[veh_id];
                js.arrived(link_id, v.path.get(v.current_link_idx + 1).copied());
            }
        }
        self.process_node_transfer(node_id);
    }

    fn handle_link_exit(&mut self, link_id: LinkID, pipe_idx: PipeIdx) {
        if link_id >= self.scenario.links.len() {
            return;
        }

        // Mirrors Python's SupplyTimes["Downstream"] update in execute_and_update_event:
        // when vehicle n exits, the slot it frees becomes usable at the entry for
        // vehicle (n + dn), where dn = kx·L·lanes(pipe), only after the backward
        // wave has travelled the link (L/w). This is the Lax-Hopf supply
        // constraint of the LWR model, applied per vehicle via the release queue.
        {
            let pipe = &mut self.scenario.links[link_id].pipes[usize::from(pipe_idx)];
            debug_assert!(
                pipe.vehicles_on_pipe > 0,
                "LinkExit on link {} pipe {} with no vehicles — double exit or miscount",
                link_id,
                pipe_idx
            );
            self.link_exited[link_id] += 1;
            if pipe.vehicles_on_pipe > 0 {
                pipe.vehicles_on_pipe -= 1;
            }
            if pipe.storage_veh.is_some() {
                if pipe.pending_release_discards > 0 {
                    // Post-shrink over-fill drain: this exit only reduces the
                    // over-occupancy — no slot opens at the entry.
                    pipe.pending_release_discards -= 1;
                } else {
                    let release = self.current_time + pipe.wave_delay;
                    pipe.release_times.push_back(release);
                }
            }
        }

        // Retry the entry queue: if the head is now storage-blocked with a known
        // release time, this schedules the LinkReady at the wave arrival.
        self.process_pipe_entry_queue(link_id, pipe_idx);

        let node_id = self.scenario.links[link_id].node_up;
        self.process_node_transfer(node_id);
    }

    fn handle_link_ready(&mut self, link_id: LinkID, pipe_idx: PipeIdx, wake_id: VehID) {
        if link_id >= self.scenario.links.len() {
            return;
        }

        // Re-arm the dedup marker OF THIS WAKE FAMILY ONLY: this LinkReady
        // has fired, so a future wake-up of the same kind must be schedulable
        // again. Clearing the other family's marker here would re-open its
        // dedup while its event is still pending — the duplicate would rescan
        // and regenerate this one, without bound (tests/friction.rs).
        // (Duplicate LinkReady events within one family are harmless —
        // processing is idempotent — but a suppressed one could strand a
        // blocked vehicle.)
        {
            let pipe = &mut self.scenario.links[link_id].pipes[usize::from(pipe_idx)];
            if wake_id == EXIT_WAKE {
                pipe.last_scheduled_exit_ready_time = None;
            } else {
                pipe.last_scheduled_ready_time = None;
            }
        }

        self.process_pipe_entry_queue(link_id, pipe_idx);

        // A constraint on this link expired: retry transfers feeding into it
        // (node upstream) and transfers draining it (node downstream — needed
        // when the upstream-capacity term was the binding constraint).
        let node_up = self.scenario.links[link_id].node_up;
        let node_down = self.scenario.links[link_id].node_down;
        self.process_node_transfer(node_up);
        self.process_node_transfer(node_down);
    }

    fn process_node_transfer(&mut self, node_id: NodeID) {
        if node_id >= self.scenario.nodes.len() {
            return;
        }
        if self.junctions_active && self.junctions.get(node_id).is_some_and(Option::is_some) {
            self.process_junction(node_id);
            return;
        }
        if self.roundabouts.get(node_id).is_some_and(Option::is_some) {
            self.process_roundabout(node_id);
            return;
        }
        let incoming_links = self.scenario.nodes[node_id].incoming_links.clone();
        if self.merge_rule == MergeRule::CapacityProportional && incoming_links.len() > 1 {
            self.process_node_transfer_fair(&incoming_links);
            return;
        }

        for in_link_idx in incoming_links {
            if in_link_idx >= self.scenario.links.len() {
                continue;
            }
            let n_pipes = self.scenario.links[in_link_idx].pipes.len();

            for in_pipe in 0..n_pipes as PipeIdx {
                self.try_transfer_pipe_head(in_link_idx, in_pipe);
            }
        }
    }

    /// Capacity-proportional merge: try every incoming pipe head once, as the
    /// in-order sweep does, but always next the one whose link has passed the
    /// fewest vehicles per unit of capacity under competition. Ties go to the
    /// lower (link, pipe), so the rule is deterministic.
    fn process_node_transfer_fair(&mut self, incoming_links: &[LinkID]) {
        let mut pending: Vec<(LinkID, PipeIdx)> = incoming_links
            .iter()
            .filter(|&&l| l < self.scenario.links.len())
            .flat_map(|&l| (0..self.scenario.links[l].pipes.len() as PipeIdx).map(move |p| (l, p)))
            .collect();
        let waiting = |sim: &Self, l: LinkID| sim.scenario.links[l].pipes.iter().any(|p| !p.waiting_queue.is_empty());
        let weight = |sim: &Self, l: LinkID| {
            let c: f64 = sim.scenario.links[l].pipes.iter().map(|p| p.capacity).sum();
            if c > 0.0 { c } else { f64::MIN_POSITIVE }
        };
        while !pending.is_empty() {
            let (k, _) = pending
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    let ka = self.merge_served[a.0] / weight(self, a.0);
                    let kb = self.merge_served[b.0] / weight(self, b.0);
                    ka.total_cmp(&kb).then(a.cmp(b))
                })
                .expect("pending is not empty");
            let (link, pipe) = pending.remove(k);
            let competing = incoming_links.iter().any(|&o| o != link && o < self.scenario.links.len() && waiting(self, o));
            let before = self.scenario.links[link].pipes[usize::from(pipe)].waiting_queue.len();
            self.try_transfer_pipe_head(link, pipe);
            let passed = self.scenario.links[link].pipes[usize::from(pipe)].waiting_queue.len() < before;
            if passed && competing {
                self.merge_served[link] += 1.0;
            }
        }
    }

    /// Attempt to move the head vehicle of (in_link, in_pipe) through the node.
    /// Strict FIFO within a pipe: vehicles cannot overtake inside one stream;
    /// cross-pipe overtaking is free by construction (each pipe has its own head).
    fn try_transfer_pipe_head(&mut self, in_link_idx: LinkID, in_pipe: PipeIdx) {
        let Some(&veh_id) = self.scenario.links[in_link_idx].pipes[usize::from(in_pipe)]
            .waiting_queue
            .front()
        else {
            return;
        };
        if veh_id >= self.scenario.vehicles.len() {
            self.scenario.links[in_link_idx].pipes[usize::from(in_pipe)]
                .waiting_queue
                .pop_front();
            return;
        }

        // Upstream supply term: the incoming pipe cannot discharge faster
        // than its own capacity (Python's SupplyTimes["UpCapacity"], applied
        // only when ActiveUpStreamCapacity is set). Friction makes the term
        // active regardless of the flag: a spilled sibling pipe throttles this
        // pipe's discharge to C·φ (rubbernecking).
        //
        // The gate is armed at DISCHARGE time via `exit_gate_active` — do not
        // re-derive the friction state here: a sibling recovering between two
        // discharges must not bypass an already-armed throttle.
        {
            let p_in = &self.scenario.links[in_link_idx].pipes[usize::from(in_pipe)];
            if p_in.exit_gate_active {
                let t_exit = p_in.next_available_exit_time;
                if self.current_time < t_exit {
                    let p_in = &mut self.scenario.links[in_link_idx].pipes[usize::from(in_pipe)];
                    // Discharge-gate wake: its own dedup marker and its own
                    // sentinel, so it never stomps (nor is stomped by) the
                    // admission-wake dedup on the same pipe.
                    if p_in.last_scheduled_exit_ready_time != Some(t_exit) {
                        p_in.last_scheduled_exit_ready_time = Some(t_exit);
                        self.events.push(Event {
                            time: t_exit,
                            event_type: EventType::LinkReady,
                            link_id: in_link_idx,
                            veh_id: EXIT_WAKE,
                            pipe: in_pipe,
                        });
                    }
                    return;
                }
            }
        }

        // Friction at THIS discharge instant — used when arming the next gate.
        let eff_phi = friction_phi(
            &self.scenario.links[in_link_idx],
            in_pipe,
            self.current_time,
        );

        // Movement of the head vehicle: next link, and the movement after it
        // (constrains which pipes of the next link are eligible).
        let (class_id, next_link_opt, next_next_opt) = {
            let v = &self.scenario.vehicles[veh_id];
            let next = v.path.get(v.current_link_idx + 1).copied();
            let next_next = v.path.get(v.current_link_idx + 2).copied();
            (v.class_id, next, next_next)
        };

        // Signals and tolls: a red group or a busy toll lane holds the head
        // (and its pipe, FIFO). Skipped entirely without signals.json.
        let controlled = if self.signals_active {
            match self.signal_gate(in_link_idx, in_pipe, veh_id, class_id, next_link_opt) {
                Some(c) => c,
                None => return,
            }
        } else {
            Controlled::default()
        };
        // Priority rules: a minor movement waits for its gap.
        let junction_move = if self.junctions_active {
            match self.junction_gate(in_link_idx, in_pipe, veh_id, next_link_opt) {
                Some(m) => m,
                None => return,
            }
        } else {
            None
        };

        if let Some(next_link_idx) = next_link_opt {
            if next_link_idx >= self.scenario.links.len() {
                return;
            }

            // Choice point 2 — late binding against current pipe states.
            let choice = choose_pipe_for_transfer(
                &self.scenario.links[next_link_idx],
                class_id,
                next_next_opt,
                self.current_time,
            );

            match choice {
                PipeChoice::AdmitNow(target) => {
                    let travel_time = {
                        let l_next = &self.scenario.links[next_link_idx];
                        l_next.length / l_next.speed
                    };
                    {
                        let p_next =
                            &mut self.scenario.links[next_link_idx].pipes[usize::from(target)];
                        p_next.admit(self.current_time);
                        if p_next.capacity > 0.0 {
                            p_next.next_available_entry_time =
                                self.current_time + 1.0 / p_next.capacity;
                        }
                    }

                    self.scenario.links[in_link_idx].pipes[usize::from(in_pipe)]
                        .waiting_queue
                        .pop_front();
                    {
                        // One-shot friction: the discharge headway armed here
                        // reflects the sibling spill state at this instant.
                        let p_in =
                            &mut self.scenario.links[in_link_idx].pipes[usize::from(in_pipe)];
                        if p_in.capacity > 0.0 {
                            p_in.next_available_exit_time =
                                self.current_time + 1.0 / (p_in.capacity * eff_phi);
                            p_in.exit_gate_active = self.upstream_capacity || eff_phi < 1.0;
                        }
                    }
                    {
                        let v = &mut self.scenario.vehicles[veh_id];
                        v.current_link_idx += 1;
                        v.state = VehicleState::Moving;
                        v.node_times.push(self.current_time);
                        v.pipes_taken.push(target);
                    }
                    self.link_entered[next_link_idx] += 1;
                    if self.signals_active {
                        self.signal_passed(controlled, in_link_idx, in_pipe);
                    }
                    if self.junctions_active {
                        self.junction_passed(in_link_idx, in_pipe, junction_move);
                        self.junction_expect(next_link_idx, veh_id);
                    }

                    self.events.push(Event {
                        time: self.current_time + travel_time,
                        event_type: EventType::VehicleArrival,
                        link_id: next_link_idx,
                        veh_id,
                        pipe: target,
                    });
                    self.events.push(Event {
                        time: self.current_time,
                        event_type: EventType::LinkExit,
                        link_id: in_link_idx,
                        veh_id,
                        pipe: in_pipe,
                    });
                }
                PipeChoice::WakeAt(target, t) => {
                    if t > self.current_time {
                        let p_next =
                            &mut self.scenario.links[next_link_idx].pipes[usize::from(target)];
                        if p_next.last_scheduled_ready_time != Some(t) {
                            p_next.last_scheduled_ready_time = Some(t);
                            self.events.push(Event {
                                time: t,
                                event_type: EventType::LinkReady,
                                link_id: next_link_idx,
                                veh_id: NO_VEHICLE,
                                pipe: target,
                            });
                        }
                    }
                }
                // Blocked: a future LinkExit on an eligible pipe re-triggers
                // this node. NoEligible: scenario coding error (validation
                // rejects it); the vehicle waits forever rather than jumping
                // into a forbidden pipe.
                PipeChoice::Blocked | PipeChoice::NoEligible => {}
            }
        } else {
            // Destination reached — exit the network (unconstrained exit supply)
            self.scenario.links[in_link_idx].pipes[usize::from(in_pipe)]
                .waiting_queue
                .pop_front();
            {
                let p_in = &mut self.scenario.links[in_link_idx].pipes[usize::from(in_pipe)];
                if p_in.capacity > 0.0 {
                    p_in.next_available_exit_time =
                        self.current_time + 1.0 / (p_in.capacity * eff_phi);
                    p_in.exit_gate_active = self.upstream_capacity || eff_phi < 1.0;
                }
            }
            {
                let v = &mut self.scenario.vehicles[veh_id];
                v.state = VehicleState::Exited;
                v.node_times.push(self.current_time);
            }
            if self.signals_active {
                self.signal_passed(controlled, in_link_idx, in_pipe);
            }
            if self.junctions_active {
                self.junction_passed(in_link_idx, in_pipe, junction_move);
            }
            self.events.push(Event {
                time: self.current_time,
                event_type: EventType::LinkExit,
                link_id: in_link_idx,
                veh_id,
                pipe: in_pipe,
            });
        }
    }
}

// ── Signals, tolls and external controllers ───────────────────────────────────
//
// See `signals.rs`. Fixed-time plans are expanded into `SignalSwitch` events
// when the simulation is built; external controllers add switches at run
// time through `Command`s. A switch to green re-triggers the group's node.

/// What a discharge went through, to update the group and toll state.
#[derive(Debug, Clone, Copy, Default)]
struct Controlled {
    group: Option<usize>,
    toll: bool,
    /// Passed as a sneaker, just after its group turned red.
    sneak: bool,
}

impl Simulation {
    /// Register an external controller. It is started when the run begins.
    pub fn add_controller(&mut self, controller: Box<dyn Controller>) {
        self.controllers.push(Some(controller));
        self.signals_active = true;
    }

    /// Initial group states and the fixed-time switches over the horizon.
    fn seed_signals(&mut self) {
        let start = self.scenario.start_time;
        let end = start + self.scenario.duration;
        let timelines = crate::signals::expand(&self.scenario.signals, start, end);
        for (g, timeline) in timelines.into_iter().enumerate() {
            for (i, (t, aspect)) in timeline.into_iter().enumerate() {
                if i == 0 {
                    let grp = &mut self.signals.groups[g];
                    grp.green = aspect.passes();
                    grp.capped = aspect == Aspect::Green;
                    self.scenario.signal_log.push(SwitchRecord {
                        group: g,
                        time: t,
                        green: aspect.passes(),
                        capped: aspect != Aspect::Off,
                    });
                } else {
                    self.push_switch(g, t, aspect);
                }
            }
        }
    }

    fn push_switch(&mut self, group: usize, time: f64, aspect: Aspect) {
        self.events.push(Event {
            time,
            event_type: EventType::SignalSwitch,
            link_id: group,
            veh_id: NO_VEHICLE,
            pipe: aspect.code(),
        });
    }

    fn handle_signal_switch(&mut self, group: usize, aspect: Aspect) {
        let Some(g) = self.signals.groups.get_mut(group) else {
            return;
        };
        let green = aspect.passes();
        let capped = aspect == Aspect::Green;
        if g.green == green && (!green || g.capped == capped) {
            return;
        }
        // A new green, or a change between plan green and dark signal,
        // starts a fresh per-green count.
        g.served = 0;
        g.capped = capped;
        let turned = g.green != green;
        g.green = green;
        if !green {
            g.red_since = self.current_time;
            g.sneak_left = g.sneakers;
        }
        let node = g.node;
        self.scenario.signal_log.push(SwitchRecord {
            group,
            time: self.current_time,
            green,
            capped: aspect != Aspect::Off,
        });
        if !turned {
            // Plan green ↔ dark signal: the cap may have just been lifted.
            self.process_node_transfer(node);
            return;
        }
        // Green releases the group's queue; red can release the movements
        // that give way to it at a priority junction (permitted turns).
        let junction = self.junctions_active && self.junctions.get(node).is_some_and(Option::is_some);
        if green || junction {
            self.process_node_transfer(node);
        }
    }

    /// May the head vehicle of (link, pipe) pass now? None = it waits (a
    /// wake-up is scheduled when the wait has a known end).
    fn signal_gate(
        &mut self,
        link: LinkID,
        pipe: PipeIdx,
        veh: VehID,
        class: usize,
        next: Option<LinkID>,
    ) -> Option<Controlled> {
        let group = self.signals.group_of(link, next);
        if let Some(g) = group {
            // Red, or the per-green cap is used up: the next switch to green
            // re-triggers the node.
            if !self.signals.open(g) {
                // A permitted vehicle (one that gives way at this priority
                // junction) already at the line may still clear just after
                // the red: the HCM sneakers.
                let at_line_before_red = {
                    let l = &self.scenario.links[link];
                    let v = &self.scenario.vehicles[veh];
                    let entered = v.link_entry_time(v.current_link_idx).unwrap_or(self.current_time);
                    entered + l.length / l.speed <= self.signals.groups[g].red_since
                };
                let permitted = self.signals.sneak_open(g, self.current_time)
                    && at_line_before_red
                    && !self.signals.groups[g].green
                    && self.junction_yields(link, next);
                if permitted {
                    return Some(Controlled { group, toll: false, sneak: true });
                }
                return None;
            }
        }
        let mut toll = false;
        if let Some(service) = self.signals.toll_service(link, pipe, class) {
            let l = &self.scenario.links[link];
            let v = &self.scenario.vehicles[veh];
            // Arrival at the booth = free-flow arrival at the link end.
            let entered = v.link_entry_time(v.current_link_idx).unwrap_or(self.current_time);
            let arrival = entered + l.length / l.speed;
            let last = self.signals.toll_lanes.get(&(link, pipe)).map_or(f64::NEG_INFINITY, |t| t.last_departure);
            let ready = arrival.max(last) + service;
            if self.current_time < ready {
                let p = &mut self.scenario.links[link].pipes[usize::from(pipe)];
                if p.last_scheduled_exit_ready_time != Some(ready) {
                    p.last_scheduled_exit_ready_time = Some(ready);
                    self.events.push(Event {
                        time: ready,
                        event_type: EventType::LinkReady,
                        link_id: link,
                        veh_id: EXIT_WAKE,
                        pipe,
                    });
                }
                return None;
            }
            toll = true;
        }
        Some(Controlled { group, toll, sneak: false })
    }

    fn signal_passed(&mut self, c: Controlled, link: LinkID, pipe: PipeIdx) {
        let now = self.current_time;
        // Earliest time the next vehicle of this pipe may follow.
        let mut next_ready = f64::NEG_INFINITY;
        if let Some(g) = c.group {
            self.signals.groups[g].served += 1;
            if c.sneak {
                let grp = &mut self.signals.groups[g];
                grp.sneak_left = grp.sneak_left.saturating_sub(1);
            }
            // A queue released by a green leaves at the saturation flow, the
            // pipe capacity: enforce the discharge headway armed just before
            // (1/(C·φ)) even when the upstream-capacity term is off.
            let p = &mut self.scenario.links[link].pipes[usize::from(pipe)];
            p.exit_gate_active = true;
            next_ready = p.next_available_exit_time;
        }
        if c.toll {
            if let Some(lane) = self.signals.toll_lanes.get_mut(&(link, pipe)) {
                lane.last_departure = now;
            }
            // The next vehicle, if already at the booth, is served from now.
            let class_and_arrival = self.scenario.links[link].pipes[usize::from(pipe)]
                .waiting_queue
                .front()
                .map(|&v| {
                    let veh = &self.scenario.vehicles[v];
                    let l = &self.scenario.links[link];
                    let entered = veh.link_entry_time(veh.current_link_idx).unwrap_or(now);
                    (veh.class_id, entered + l.length / l.speed)
                });
            if let Some((class, arrival)) = class_and_arrival {
                if let Some(service) = self.signals.toll_service(link, pipe, class) {
                    next_ready = next_ready.max(arrival.max(now) + service);
                }
            }
        }
        // The vehicle now at the head needs its own wake-up: nothing else
        // re-triggers this node when the headway or the service ends.
        let p = &mut self.scenario.links[link].pipes[usize::from(pipe)];
        if next_ready > now && !p.waiting_queue.is_empty() && p.last_scheduled_exit_ready_time != Some(next_ready) {
            p.last_scheduled_exit_ready_time = Some(next_ready);
            self.events.push(Event {
                time: next_ready,
                event_type: EventType::LinkReady,
                link_id: link,
                veh_id: EXIT_WAKE,
                pipe,
            });
        }
    }

    fn start_controllers(&mut self) {
        if self.controllers_started {
            return;
        }
        self.controllers_started = true;
        for i in 0..self.controllers.len() {
            self.call_controller(i, true);
        }
    }

    fn call_controller(&mut self, i: usize, first: bool) {
        let Some(mut c) = self.controllers.get_mut(i).and_then(Option::take) else {
            return;
        };
        let commands = {
            let view = ControlView {
                time: self.current_time,
                scenario: &self.scenario,
                signals: &self.signals,
                entered: &self.link_entered,
                exited: &self.link_exited,
            };
            if first {
                c.start(&view)
            } else {
                c.wake(&view)
            }
        };
        self.controllers[i] = Some(c);
        for cmd in commands {
            self.apply_command(i, cmd);
        }
    }

    fn apply_command(&mut self, controller: usize, cmd: Command) {
        let now = self.current_time;
        match cmd {
            Command::Switch { group, at, green } => {
                if let Some(g) = self.signals.group_index(&group) {
                    let aspect = if green { Aspect::Green } else { Aspect::Red };
                    self.push_switch(g, if at.is_finite() { at.max(now) } else { now }, aspect);
                }
            }
            // Only future wake-ups: a controller asking for "now" again
            // would loop forever at one instant.
            Command::WakeAt(t) => {
                if t.is_finite() && t > now {
                    self.events.push(Event {
                        time: t,
                        event_type: EventType::ControlWake,
                        link_id: controller,
                        veh_id: NO_VEHICLE,
                        pipe: 0,
                    });
                }
            }
            Command::SetVehiclesPerGreen { group, n } => {
                if let Some(g) = self.signals.group_index(&group) {
                    self.signals.groups[g].vehicles_per_green = n.filter(|&n| n > 0);
                    let node = self.signals.groups[g].node;
                    self.process_node_transfer(node);
                }
            }
        }
    }
}

// ── Priority junctions ────────────────────────────────────────────────────────
//
// See `junction.rs`. A junction node serves its heads in priority order
// (rank 1 first) and repeats after every departure, so a minor head always
// sees the major vehicles that could go before it.

/// Outcome of the junction gate before any state is changed.
enum JunctionWait {
    Go(Option<usize>),
    Until(f64),
    Event,
}

impl Simulation {
    /// A vehicle just entered `link`: if the link ends at a junction, record
    /// its arrival there (free-flow, the same expression as its
    /// `VehicleArrival`).
    fn junction_expect(&mut self, link: LinkID, veh: VehID) {
        let l = &self.scenario.links[link];
        let node = l.node_down;
        let arrival = self.current_time + l.length / l.speed;
        let v = &self.scenario.vehicles[veh];
        let next = v.path.get(v.current_link_idx + 1).copied();
        if let Some(js) = self.junctions.get_mut(node).and_then(|j| j.as_deref_mut()) {
            js.expect(link, arrival, next);
        }
    }

    /// Does the movement `link → next` give way to another one at its
    /// node (a permitted movement)?
    fn junction_yields(&self, link: LinkID, next: Option<LinkID>) -> bool {
        let node = self.scenario.links[link].node_down;
        self.junctions.get(node).and_then(|j| j.as_deref()).is_some_and(|js| {
            js.movement(link, next).is_some_and(|m| !js.moves[m].yields.is_empty())
        })
    }

    fn junction_passed(&mut self, link: LinkID, pipe: PipeIdx, movement: Option<usize>) {
        let node = self.scenario.links[link].node_down;
        let now = self.current_time;
        if let Some(js) = self.junctions.get_mut(node).and_then(|j| j.as_deref_mut()) {
            js.passed(movement, link, pipe, now);
        }
    }

    /// Serve the heads of a junction in priority order until none moves.
    fn process_junction(&mut self, node_id: NodeID) {
        loop {
            let mut heads: Vec<(u8, LinkID, PipeIdx)> = Vec::new();
            {
                let Some(js) = self.junctions[node_id].as_deref() else { return };
                for &l in &self.scenario.nodes[node_id].incoming_links {
                    let Some(link) = self.scenario.links.get(l) else { continue };
                    for (p, pipe) in link.pipes.iter().enumerate() {
                        if let Some(&v) = pipe.waiting_queue.front() {
                            let veh = &self.scenario.vehicles[v];
                            let next = veh.path.get(veh.current_link_idx + 1).copied();
                            heads.push((js.rank_of(l, next), l, p as PipeIdx));
                        }
                    }
                }
            }
            heads.sort_unstable();
            let mut progress = false;
            for (_, l, p) in heads {
                let before = self.scenario.links[l].pipes[usize::from(p)].waiting_queue.len();
                self.try_transfer_pipe_head(l, p);
                if self.scenario.links[l].pipes[usize::from(p)].waiting_queue.len() < before {
                    progress = true;
                    break;
                }
            }
            if !progress {
                break;
            }
        }
    }

    /// May the head of (link, pipe), bound for `next`, go now under the
    /// priority rules? `Some(movement)` = yes (record it on departure),
    /// None = it waits (with a wake-up when the wait has a known end).
    fn junction_gate(&mut self, link: LinkID, pipe: PipeIdx, veh: VehID, next: Option<LinkID>) -> Option<Option<usize>> {
        let node = self.scenario.links[link].node_down;
        let now = self.current_time;
        let wait = {
            let Some(js) = self.junctions.get(node).and_then(|j| j.as_deref()) else {
                return Some(None);
            };
            let Some(m) = js.movement(link, next) else {
                return Some(None);
            };
            if js.moves[m].yields.is_empty() {
                JunctionWait::Go(Some(m))
            } else {
                // A movement held by a red signal does not count as a conflict.
                let red = |n: usize| -> bool {
                    self.signals_active
                        && self
                            .signals
                            .group_of(js.moves[n].from, Some(js.moves[n].to))
                            .is_some_and(|g| !self.signals.groups[g].green)
                };
                let l = &self.scenario.links[link];
                let v = &self.scenario.vehicles[veh];
                let arrival = v.link_entry_time(v.current_link_idx).unwrap_or(now) + l.length / l.speed;
                let t = js.cheap_bound(m, link, pipe, arrival, now);
                if t > now {
                    JunctionWait::Until(t)
                } else if js.approaching_conflict(m, now, &red) {
                    // The approaching vehicle's arrival re-triggers the node.
                    JunctionWait::Event
                } else {
                    // A better-ranked vehicle already at the line that can go
                    // soon keeps its priority; one stuck behind a full exit
                    // does not.
                    let lag = js.moves[m].lag;
                    let blocked = js.moves[m].yields.iter().any(|&n| {
                        if red(n) {
                            return false;
                        }
                        let (from, to) = (js.moves[n].from, js.moves[n].to);
                        self.scenario.links[from].pipes.iter().any(|p| {
                            p.waiting_queue.front().is_some_and(|&hv| {
                                let h = &self.scenario.vehicles[hv];
                                if h.path.get(h.current_link_idx + 1) != Some(&to) {
                                    return false;
                                }
                                let nn = h.path.get(h.current_link_idx + 2).copied();
                                match choose_pipe_for_transfer(&self.scenario.links[to], h.class_id, nn, now) {
                                    PipeChoice::AdmitNow(_) => true,
                                    PipeChoice::WakeAt(_, tw) => tw < now + lag,
                                    PipeChoice::Blocked | PipeChoice::NoEligible => false,
                                }
                            })
                        })
                    });
                    if blocked {
                        JunctionWait::Event
                    } else {
                        JunctionWait::Go(Some(m))
                    }
                }
            }
        };
        match wait {
            JunctionWait::Go(m) => Some(m),
            JunctionWait::Event => None,
            JunctionWait::Until(t) => {
                let p = &mut self.scenario.links[link].pipes[usize::from(pipe)];
                if p.last_scheduled_exit_ready_time != Some(t) {
                    p.last_scheduled_exit_ready_time = Some(t);
                    self.events.push(Event {
                        time: t,
                        event_type: EventType::LinkReady,
                        link_id: link,
                        veh_id: EXIT_WAKE,
                        pipe,
                    });
                }
                None
            }
        }
    }
}

// ── Roundabouts ───────────────────────────────────────────────────────────────
//
// The ring of a roundabout node is solved in `process_roundabout`, which the
// ordinary triggers reach through `process_node_transfer`: an approach
// arrival or gate wake (node_down), an exit-link slot release or wake
// (node_up). Ring points have their own events, `RingArrival` (free-flow
// arrival at the next point) and `RingReady` (a ring constraint expires).
// See `roundabout.rs` for the rules.

impl Simulation {
    fn handle_ring_arrival(&mut self, node_id: NodeID, point: PipeIdx, veh_id: VehID) {
        let Some(Some(st)) = self.roundabouts.get_mut(node_id) else {
            return;
        };
        st.arrive(usize::from(point), veh_id);
        self.process_roundabout(node_id);
    }

    fn handle_ring_ready(&mut self, node_id: NodeID, point: PipeIdx, time: f64) {
        if let Some(Some(st)) = self.roundabouts.get_mut(node_id) {
            st.clear_pending(usize::from(point), time);
        }
        self.process_roundabout(node_id);
    }

    /// Wake the roundabout at `t` for point `p` (one event per distinct time).
    fn schedule_ring_ready(&mut self, st: &mut RoundaboutState, p: usize, t: f64) {
        if t > self.current_time && st.add_pending(p, t) {
            self.events.push(Event {
                time: t,
                event_type: EventType::RingReady,
                link_id: st.node,
                veh_id: NO_VEHICLE,
                pipe: p as PipeIdx,
            });
        }
    }

    fn schedule_ring_arrival(&mut self, st: &RoundaboutState, from: usize, veh: VehID) {
        let next = st.geom.next(from);
        self.events.push(Event {
            time: self.current_time + st.travel_time(from, next),
            event_type: EventType::RingArrival,
            link_id: st.node,
            veh_id: veh,
            pipe: next as PipeIdx,
        });
    }

    /// Move everything that can move through the roundabout now. The ring
    /// goes first and is settled before any entry is considered, so that a
    /// vehicle that can pass a point right away is seen in motion by the
    /// entries' lag check; then the entries, in the order of circulation.
    /// Repeat while anything passed (a passage can free storage upstream).
    fn process_roundabout(&mut self, node_id: NodeID) {
        let Some(mut st) = self.roundabouts.get_mut(node_id).and_then(Option::take) else {
            return;
        };
        loop {
            let mut progress = false;
            for p in 0..st.geom.len() {
                while self.try_pass_ring(&mut st, p) {
                    progress = true;
                }
            }
            if progress {
                continue;
            }
            for p in 0..st.geom.len() {
                let point = st.geom.points[p];
                if point.kind != PointKind::Merge {
                    continue;
                }
                let Some(in_link) = st.spec.arms[point.arm].in_link else {
                    continue;
                };
                if in_link >= self.scenario.links.len() {
                    continue;
                }
                let n_pipes = self.scenario.links[in_link].pipes.len();
                for pipe in 0..n_pipes as PipeIdx {
                    while self.try_enter_ring(&mut st, p, in_link, pipe) {
                        progress = true;
                    }
                }
            }
            if !progress {
                break;
            }
        }
        self.roundabouts[node_id] = Some(st);
    }

    /// Let the vehicle waiting at the head of ring point `p` pass it, if the
    /// ring capacity, the storage downstream and (at its exit) the exit link
    /// allow. Returns true when it passed.
    fn try_pass_ring(&mut self, st: &mut RoundaboutState, p: usize) -> bool {
        let now = self.current_time;
        let Some(&veh) = st.hold[p].front() else {
            return false;
        };
        let Some(&rv) = st.on_ring.get(&veh) else {
            st.hold[p].pop_front();
            return true;
        };
        // At its exit point a vehicle leaves, unless no pipe of its exit link
        // accepts it any more (closed by a patch after it entered): then it
        // goes round again rather than stopping and blocking the ring. The
        // exit link's state is not part of the ring version, so heads at
        // their exit point are never served from the cache.
        let at_exit = p == rv.exit_pt;
        let leaving = at_exit && {
            let v = &self.scenario.vehicles[veh];
            let out = v.path.get(v.current_link_idx + 1).copied();
            let next_next = v.path.get(v.current_link_idx + 2).copied();
            out.filter(|&l| l < self.scenario.links.len())
                .is_some_and(|l| has_eligible_pipe(&self.scenario.links[l], v.class_id, next_next))
        };
        if !at_exit && st.ring_head_waits(p, veh, now) {
            return false;
        }
        // The ring headway alone may already say "later": wake up then and
        // evaluate the storage at that time (same passage time, less work).
        let cap = st.capacity_bound(p);
        let bound = if cap > now || leaving {
            Bound::At(cap)
        } else if at_exit {
            Bound::At(cap).and(st.storage_bound_lap(p, Some(veh)))
        } else {
            Bound::At(cap).and(st.storage_bound(p, rv.exit_pt, Some(veh)))
        };
        let t = match bound {
            Bound::Blocked => {
                if !at_exit {
                    st.remember_ring_bound(p, veh, bound);
                }
                return false;
            }
            Bound::At(t) => t,
        };
        if t > now {
            if !at_exit {
                st.remember_ring_bound(p, veh, bound);
            }
            self.schedule_ring_ready(st, p, t);
            return false;
        }

        if !leaving {
            st.hold[p].pop_front();
            st.record(p, veh, now, true);
            if let Some(r) = st.on_ring.get_mut(&veh) {
                r.last_pt = p;
                r.last_time = now;
                r.in_transit = true;
            }
            self.schedule_ring_arrival(st, p, veh);
            return true;
        }

        // Exit point: leave by the next link of the path.
        let (class_id, out, next_next) = {
            let v = &self.scenario.vehicles[veh];
            (
                v.class_id,
                v.path.get(v.current_link_idx + 1).copied(),
                v.path.get(v.current_link_idx + 2).copied(),
            )
        };
        let Some(out) = out.filter(|&l| l < self.scenario.links.len()) else {
            return false;
        };
        match choose_pipe_for_transfer(&self.scenario.links[out], class_id, next_next, now) {
            PipeChoice::AdmitNow(target) => {
                let travel_time = {
                    let l = &self.scenario.links[out];
                    l.length / l.speed
                };
                {
                    let p_next = &mut self.scenario.links[out].pipes[usize::from(target)];
                    p_next.admit(now);
                    self.link_entered[out] += 1;
                    if p_next.capacity > 0.0 {
                        p_next.next_available_entry_time = now + 1.0 / p_next.capacity;
                    }
                }
                {
                    let v = &mut self.scenario.vehicles[veh];
                    v.current_link_idx += 1;
                    v.state = VehicleState::Moving;
                    v.pipes_taken.push(target);
                    v.ring_exits.push((v.current_link_idx, now));
                }
                if self.junctions_active {
                    self.junction_expect(out, veh);
                }
                self.events.push(Event {
                    time: now + travel_time,
                    event_type: EventType::VehicleArrival,
                    link_id: out,
                    veh_id: veh,
                    pipe: target,
                });
                st.hold[p].pop_front();
                st.on_ring.remove(&veh);
                st.record(p, veh, now, true);
                true
            }
            PipeChoice::WakeAt(target, t) => {
                if t > now {
                    let p_next = &mut self.scenario.links[out].pipes[usize::from(target)];
                    if p_next.last_scheduled_ready_time != Some(t) {
                        p_next.last_scheduled_ready_time = Some(t);
                        self.events.push(Event {
                            time: t,
                            event_type: EventType::LinkReady,
                            link_id: out,
                            veh_id: NO_VEHICLE,
                            pipe: target,
                        });
                    }
                }
                false
            }
            // Blocked: a LinkExit on the exit link re-triggers this node.
            PipeChoice::Blocked | PipeChoice::NoEligible => false,
        }
    }

    /// Let the head of approach (in_link, pipe) enter the ring at merge
    /// point `m`. Returns true when it entered (or ended its trip here).
    fn try_enter_ring(&mut self, st: &mut RoundaboutState, m: usize, in_link: LinkID, pipe: PipeIdx) -> bool {
        let now = self.current_time;
        let Some(&veh) = self.scenario.links[in_link].pipes[usize::from(pipe)]
            .waiting_queue
            .front()
        else {
            return false;
        };
        if veh >= self.scenario.vehicles.len() {
            self.scenario.links[in_link].pipes[usize::from(pipe)].waiting_queue.pop_front();
            return true;
        }
        // Known to wait for the ring: nothing on the approach side can change
        // that (the gate is checked again once the ring lets it go).
        let key = (in_link, pipe);
        if st.entry_waits(key, veh, now) {
            return false;
        }

        // Discharge gate of the approach pipe (friction / upstream capacity),
        // exactly as at an ordinary node.
        {
            let p_in = &mut self.scenario.links[in_link].pipes[usize::from(pipe)];
            if p_in.exit_gate_active && now < p_in.next_available_exit_time {
                let t_exit = p_in.next_available_exit_time;
                if p_in.last_scheduled_exit_ready_time != Some(t_exit) {
                    p_in.last_scheduled_exit_ready_time = Some(t_exit);
                    self.events.push(Event {
                        time: t_exit,
                        event_type: EventType::LinkReady,
                        link_id: in_link,
                        veh_id: EXIT_WAKE,
                        pipe,
                    });
                }
                return false;
            }
        }
        let eff_phi = friction_phi(&self.scenario.links[in_link], pipe, now);

        let out = {
            let v = &self.scenario.vehicles[veh];
            v.path.get(v.current_link_idx + 1).copied()
        };
        let exit_pt = match out {
            Some(out) => match st.exit_point_of(out) {
                // Its exit accepts nobody of its class right now (closed by
                // a patch): wait on the approach, as at an ordinary node.
                // The patch reopening the link re-triggers this node.
                Some(_)
                    if {
                        let v = &self.scenario.vehicles[veh];
                        let next_next = v.path.get(v.current_link_idx + 2).copied();
                        !has_eligible_pipe(&self.scenario.links[out], v.class_id, next_next)
                    } =>
                {
                    return false
                }
                Some(x) => x,
                // Not an exit of this roundabout: validation rejects such a
                // path; the vehicle waits rather than teleporting.
                None => return false,
            },
            None => {
                // Trip ends on the approach: leave the network here.
                self.scenario.links[in_link].pipes[usize::from(pipe)].waiting_queue.pop_front();
                self.arm_discharge_gate(in_link, pipe, eff_phi);
                let v = &mut self.scenario.vehicles[veh];
                v.state = VehicleState::Exited;
                v.node_times.push(now);
                self.events.push(Event {
                    time: now,
                    event_type: EventType::LinkExit,
                    link_id: in_link,
                    veh_id: veh,
                    pipe,
                });
                return true;
            }
        };

        match st.entry_bound(m, key, exit_pt, now) {
            b @ Bound::Blocked => {
                st.remember_entry_bound(key, veh, b);
                return false;
            }
            b @ Bound::At(t) if t > now => {
                st.remember_entry_bound(key, veh, b);
                self.schedule_ring_ready(st, m, t);
                return false;
            }
            Bound::At(_) => {}
        }

        self.scenario.links[in_link].pipes[usize::from(pipe)].waiting_queue.pop_front();
        self.arm_discharge_gate(in_link, pipe, eff_phi);
        {
            let v = &mut self.scenario.vehicles[veh];
            v.state = VehicleState::Moving;
            // Passage through the node = leaving the approach.
            v.node_times.push(now);
        }
        self.events.push(Event {
            time: now,
            event_type: EventType::LinkExit,
            link_id: in_link,
            veh_id: veh,
            pipe,
        });
        st.on_ring.insert(veh, RingVehicle { last_pt: m, last_time: now, exit_pt, in_transit: true });
        st.record(m, veh, now, false);
        st.last_entry.insert(key, now);
        self.schedule_ring_arrival(st, m, veh);
        true
    }

    /// Arm the discharge headway of an incoming pipe after a vehicle left it
    /// (same one-shot friction rule as `try_transfer_pipe_head`).
    fn arm_discharge_gate(&mut self, link: LinkID, pipe: PipeIdx, eff_phi: f64) {
        let p_in = &mut self.scenario.links[link].pipes[usize::from(pipe)];
        if p_in.capacity > 0.0 {
            p_in.next_available_exit_time = self.current_time + 1.0 / (p_in.capacity * eff_phi);
            p_in.exit_gate_active = self.upstream_capacity || eff_phi < 1.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{FundamentalDiagram, Link, Node, NodeType, Scenario, Vehicle};

    /// Entry(0) →[L0]→ Internal(1) →[L1]→ Exit(2)
    ///
    /// L0: 100 m, u = 10 m/s (10 s traverse), w = 5 m/s (L/w = 20 s wave),
    ///     kx = 0.02 veh/m/lane, `lanes_l0` lanes → storage = 2·lanes_l0 veh,
    ///     capacity 10 veh/s (0.1 s inflow headway — effectively unconstrained).
    /// L1: 100 m, huge storage; `c1` (veh/s) is the bottleneck discharge rate.
    fn two_link_scenario(lanes_l0: u8, c1: f64, n_veh: usize) -> Scenario {
        let fd0 = FundamentalDiagram {
            u: 10.0,
            w: 5.0,
            kx: 0.02,
            c: 10.0,
        };
        let fd1 = FundamentalDiagram {
            u: 10.0,
            w: 5.0,
            kx: 10.0,
            c: c1,
        };
        let vehicles = (0..n_veh)
            .map(|i| Vehicle::new(i, 0, vec![0, 1], 0.0, 0, 2))
            .collect();
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
                Link::new(0, 0, 1, 100.0, 10.0, lanes_l0, 10.0, fd0, vec![]),
                Link::new(1, 1, 2, 100.0, 10.0, 1, c1, fd1, vec![]),
            ],
            vehicles,
            demand: vec![],
            start_time: 0.0,
            duration: 100_000.0,
            classes: vec!["car".to_string()],
            link_schedule: Vec::new(),
            assignment_excluded: Vec::new(), signals: Default::default(), signal_log: Vec::new(),
        }
    }

    fn run(scenario: Scenario) -> Simulation {
        let mut sim = Simulation::new(scenario);
        sim.run().expect("simulation should complete");
        sim
    }

    fn assert_time(actual: f64, expected: f64, msg: &str) {
        assert!(
            (actual - expected).abs() < 1e-9,
            "{msg}: expected {expected}, got {actual}"
        );
    }

    #[test]
    fn storage_counts_lanes() {
        let s = two_link_scenario(2, 0.01, 1);
        // dn = kx · L · lanes = 0.02 · 100 · 2 = 4 vehicles
        assert_eq!(s.links[0].storage_veh(), Some(4));
        let s1 = two_link_scenario(1, 0.01, 1);
        assert_eq!(s1.links[0].storage_veh(), Some(2));
    }

    /// The core Newell/Lax-Hopf check: with L0 storage = 4, vehicle 4 may only
    /// enter L0 once vehicle 0 has exited (t = 10 s, when it transfers to L1)
    /// plus the backward wave time L/w = 20 s → entry at exactly t = 30 s.
    /// Vehicle 5 waits for vehicle 1's exit (t = 110, capacity-gated by L1's
    /// 100 s headway) + 20 s → entry at exactly t = 130 s.
    #[test]
    fn backward_wave_delays_entry_by_l_over_w() {
        let sim = run(two_link_scenario(2, 0.01, 6));
        let v = &sim.scenario.vehicles;

        // Vehicles 0–3 fill the storage immediately (0.1 s inflow headway).
        assert_time(v[0].node_times[0], 0.0, "veh 0 entry");
        assert_time(v[3].node_times[0], 0.3, "veh 3 entry");

        // Vehicle 0 transfers to L1 at its free-flow arrival (t = 10).
        assert_time(v[0].node_times[1], 10.0, "veh 0 passage at node 1");

        // Vehicle 4: slot freed at t = 10, usable at t = 10 + 100/5 = 30.
        assert_time(v[4].node_times[0], 30.0, "veh 4 entry (backward wave)");

        // Vehicle 1 discharges into L1 at t = 10 + 1/c1 = 110 (capacity headway).
        assert_time(v[1].node_times[1], 110.0, "veh 1 passage at node 1");

        // Vehicle 5: slot freed at t = 110, usable at t = 130.
        assert_time(v[5].node_times[0], 130.0, "veh 5 entry (backward wave)");
    }

    /// Non-integer storage: kx = 0.017 → dn = 0.017·100·2 = 3.4 → 4 whole
    /// slots, and the wave delay gains the fractional-slot correction
    /// (dn_ceil − dn)/capacity = 0.6/10 = 0.06 s on top of L/w = 20 s.
    #[test]
    fn fractional_storage_wave_correction() {
        let mut s = two_link_scenario(2, 0.01, 6);
        s.links[0].fd.kx = 0.017;
        assert_eq!(s.links[0].storage_veh(), Some(4));
        assert_time(
            s.links[0].wave_delay(),
            20.06,
            "wave delay incl. fractional correction",
        );

        let sim = run(s);
        let v = &sim.scenario.vehicles;
        // Slot freed by vehicle 0's exit at t = 10 becomes usable at 10 + 20.06.
        assert_time(v[4].node_times[0], 30.06, "veh 4 entry (fractional wave)");
    }

    /// With one lane, L0 stores only 2 vehicles — the same wave timing must
    /// now apply to vehicle 2 instead of vehicle 4.
    #[test]
    fn lanes_change_spillback_onset() {
        let sim = run(two_link_scenario(1, 0.01, 3));
        let v = &sim.scenario.vehicles;
        assert_time(v[1].node_times[0], 0.1, "veh 1 enters freely");
        assert_time(v[2].node_times[0], 30.0, "veh 2 blocked until wave arrives");
    }

    /// Discharge into the bottleneck link must be spaced by exactly 1/C.
    #[test]
    fn bottleneck_discharge_at_capacity() {
        let sim = run(two_link_scenario(2, 0.01, 4));
        let v = &sim.scenario.vehicles;
        for (i, veh) in v.iter().enumerate() {
            assert_time(
                veh.node_times[1],
                10.0 + 100.0 * i as f64,
                &format!("veh {i} discharge into L1"),
            );
        }
    }

    /// FIFO must hold everywhere: passage times at every node are
    /// non-decreasing in vehicle id (all vehicles share one path).
    #[test]
    fn fifo_preserved_and_all_vehicles_exit() {
        let sim = run(two_link_scenario(2, 0.5, 20));
        let v = &sim.scenario.vehicles;
        for veh in v {
            assert_eq!(
                veh.state,
                VehicleState::Exited,
                "veh {} did not exit",
                veh.id
            );
            assert_eq!(
                veh.node_times.len(),
                3,
                "veh {} incomplete node_times",
                veh.id
            );
        }
        for node in 0..3 {
            for pair in v.windows(2) {
                assert!(
                    pair[0].node_times[node] <= pair[1].node_times[node],
                    "FIFO violated at node {node} between veh {} and {}",
                    pair[0].id,
                    pair[1].id
                );
            }
        }
    }

    // ── Multi-pipe (Phase 2) ──────────────────────────────────────────────────

    use crate::model::PipeSpec;

    /// Entry(0) →[L0: 2 pipes]→ Internal(1) →[L1]→ Exit(2) / [L2]→ Exit(3).
    /// L0: 100 m, u = 10, w = 5, kx = 0.02/lane → per-1-lane pipe storage = 2,
    /// fd.c = 5 per lane (0.2 s pipe headway — effectively unconstrained).
    fn two_pipe_scenario(n_veh: usize) -> Scenario {
        let fd0 = FundamentalDiagram {
            u: 10.0,
            w: 5.0,
            kx: 0.02,
            c: 5.0,
        };
        let fd_out = FundamentalDiagram {
            u: 10.0,
            w: 5.0,
            kx: 10.0,
            c: 10.0,
        };
        let vehicles = (0..n_veh)
            .map(|i| Vehicle::new(i, 0, vec![0, 1], 0.0, 0, 2))
            .collect();
        let mut l0 = Link::new(0, 0, 1, 100.0, 10.0, 2, 10.0, fd0, vec![]);
        l0.pipe_specs = vec![
            PipeSpec {
                lanes: 1,
                class_mask: crate::model::ALL_CLASSES,
            },
            PipeSpec {
                lanes: 1,
                class_mask: crate::model::ALL_CLASSES,
            },
        ];
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
                    outgoing_links: vec![1, 2],
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
                Node {
                    id: 3,
                    node_type: NodeType::Exit,
                    incoming_links: vec![2],
                    outgoing_links: vec![],
                    points: (2.0, 1.0),
                    signals: vec![], roundabout: None, junction: None,
                },
            ],
            links: vec![
                l0,
                Link::new(1, 1, 2, 100.0, 10.0, 1, 10.0, fd_out.clone(), vec![]),
                Link::new(2, 1, 3, 100.0, 10.0, 1, 10.0, fd_out, vec![]),
            ],
            vehicles,
            demand: vec![],
            start_time: 0.0,
            duration: 100_000.0,
            classes: vec!["car".to_string(), "bus".to_string()],
            link_schedule: Vec::new(),
            assignment_excluded: Vec::new(), signals: Default::default(), signal_log: Vec::new(),
        }
    }

    /// Entry choice: least-occupied eligible pipe, keep-right on ties, fully
    /// deterministic — vehicles alternate pipes 0, 1, 0, 1, …
    #[test]
    fn entry_choice_least_occupied_keep_right() {
        let sim = run(two_pipe_scenario(6));
        let taken: Vec<u8> = sim
            .scenario
            .vehicles
            .iter()
            .map(|v| v.pipes_taken[0])
            .collect();
        assert_eq!(
            taken,
            vec![0, 1, 0, 1, 0, 1],
            "expected alternating keep-right fill"
        );
    }

    /// A bus-only pipe rejects cars; buses use it.
    #[test]
    fn bus_only_pipe_rejects_cars() {
        let mut s = two_pipe_scenario(4);
        s.links[0].pipe_specs[1].class_mask = 0b10; // pipe 1 = class 1 (bus) only
        s.vehicles[2].class_id = 1; // one bus among cars
        let sim = run(s);
        let v = &sim.scenario.vehicles;
        assert_eq!(v[0].pipes_taken[0], 0, "car must use pipe 0");
        assert_eq!(v[1].pipes_taken[0], 0, "car must use pipe 0");
        assert_eq!(v[2].pipes_taken[0], 1, "bus goes to the empty bus pipe");
        assert_eq!(v[3].pipes_taken[0], 0, "car must use pipe 0");
    }

    /// Movement restriction: vehicles diverging to link 2 may only use pipe 0.
    #[test]
    fn moves_restriction_routes_divergers() {
        let mut s = two_pipe_scenario(6);
        s.links[0].moves.insert(2, vec![0]); // exit movement: rightmost pipe only
        s.links[0].moves.insert(1, vec![1]); // through movement: left pipe only
                                             // Vehicles 0, 2, 4 exit via link 2; vehicles 1, 3, 5 go through via link 1.
        for i in [0usize, 2, 4] {
            s.vehicles[i].path = vec![0, 2];
            s.vehicles[i].destination = 3;
        }
        let sim = run(s);
        for v in &sim.scenario.vehicles {
            let expected = if v.path[1] == 2 { 0 } else { 1 };
            assert_eq!(
                v.pipes_taken[0], expected,
                "veh {} (movement to link {}) took pipe {}",
                v.id, v.path[1], v.pipes_taken[0]
            );
        }
    }

    /// The shoulder-exit case: pipe 0 (exit movement) spills back while pipe 1
    /// (through movement) flows at exact free-flow — and the spillback wave on
    /// pipe 0 has exactly the single-pipe timing (entry gated at exit + L/w).
    #[test]
    fn pipe_spillback_isolated_from_sibling() {
        let mut s = two_pipe_scenario(6);
        s.links[0].moves.insert(2, vec![0]);
        s.links[0].moves.insert(1, vec![1]);
        // Exit link 2 becomes a tiny bottleneck: headway 100 s.
        s.links[2].capacity = 0.01;
        s.links[2].fd.c = 0.01;
        // Vehicles 0..4 exit-bound (pipe 0, storage 2); 4..6 through (pipe 1,
        // storage 2 as well — keep the through count within it so free flow
        // is the correct expectation).
        for i in 0..4usize {
            s.vehicles[i].path = vec![0, 2];
            s.vehicles[i].destination = 3;
        }
        let sim = run(s);
        let v = &sim.scenario.vehicles;

        // Through vehicles: unimpeded — enter pipe 1 at 0.2 s headway (fd.c = 5
        // per lane) and traverse at exactly L/u = 10 s.
        for (k, i) in (4..6usize).enumerate() {
            let t_entry = v[i].node_times[0];
            assert_time(t_entry, 0.2 * k as f64, &format!("through veh {i} entry"));
            assert_time(
                v[i].node_times[1],
                t_entry + 10.0,
                &format!("through veh {i} node passage"),
            );
        }

        // Exit-bound vehicles: pipe 0 stores 2; veh 0 and 1 enter at 0 / 0.2.
        assert_time(v[0].node_times[0], 0.0, "exit veh 0 entry");
        assert_time(v[1].node_times[0], 0.2, "exit veh 1 entry");
        // Veh 0 transfers into L2 at its arrival (t = 10), freeing a slot whose
        // backward wave reaches the entry at 10 + 100/5 = 30 → veh 2 enters at 30.
        assert_time(v[0].node_times[1], 10.0, "exit veh 0 passage");
        assert_time(v[2].node_times[0], 30.0, "exit veh 2 entry (wave-gated)");
        // Veh 1 discharges at 10 + 1/C = 110 → veh 3 enters at 130.
        assert_time(v[1].node_times[1], 110.0, "exit veh 1 passage");
        assert_time(v[3].node_times[0], 130.0, "exit veh 3 entry (wave-gated)");

        // Cross-pipe overtaking: every through vehicle passes the node before
        // exit-bound veh 1 does.
        for i in 4..6usize {
            assert!(
                v[i].node_times[1] < v[1].node_times[1],
                "through veh {i} should overtake the queued exit stream"
            );
        }
    }

    /// Per-pipe strict FIFO: within each pipe of L0, node passages follow
    /// entry order.
    #[test]
    fn per_pipe_fifo_holds() {
        let sim = run(two_pipe_scenario(10));
        for pipe in 0..2u8 {
            let mut last = f64::NEG_INFINITY;
            for v in &sim.scenario.vehicles {
                if v.pipes_taken[0] == pipe {
                    assert!(
                        v.node_times[1] >= last,
                        "FIFO violated in pipe {pipe} at veh {}",
                        v.id
                    );
                    last = v.node_times[1];
                }
            }
        }
    }

    /// Friction φ: while the exit pipe is spilled, the through pipe discharges
    /// at C·φ — through passages at the node are spaced by 1/(C·0.5) = 0.4 s
    /// instead of the arrival spacing 0.2 s.
    #[test]
    fn friction_throttles_sibling_discharge() {
        let build = |phi: f64| {
            let mut s = two_pipe_scenario(6);
            s.links[0].moves.insert(2, vec![0]);
            s.links[0].moves.insert(1, vec![1]);
            s.links[0].friction = phi;
            s.links[2].capacity = 0.01;
            s.links[2].fd.c = 0.01;
            for i in 0..4usize {
                s.vehicles[i].path = vec![0, 2];
                s.vehicles[i].destination = 3;
            }
            s
        };

        // Control: φ = 1 → through passages follow arrivals exactly (10.0, 10.2).
        let sim = run(build(1.0));
        let v = &sim.scenario.vehicles;
        assert_time(v[4].node_times[1], 10.0, "no-friction through veh 4");
        assert_time(v[5].node_times[1], 10.2, "no-friction through veh 5");

        // φ = 0.5: pipe 0 is spilled from t = 0.2 on, so pipe 1 discharges at
        // C·φ = 2.5 veh/s → the second through passage is gated to 10.0 + 0.4.
        let sim = run(build(0.5));
        let v = &sim.scenario.vehicles;
        assert_time(
            v[4].node_times[1],
            10.0,
            "friction through veh 4 (first, ungated)",
        );
        assert_time(
            v[5].node_times[1],
            10.4,
            "friction through veh 5 (gated to C·φ)",
        );
    }
    /// kx ≤ 0 disables the storage constraint entirely.
    #[test]
    fn zero_kx_means_unlimited_storage() {
        let mut s = two_link_scenario(2, 10.0, 10);
        s.links[0].fd.kx = 0.0;
        assert_eq!(s.links[0].storage_veh(), None);
        let sim = run(s);
        // All 10 vehicles enter L0 back-to-back at the 0.1 s capacity headway.
        for (i, veh) in sim.scenario.vehicles.iter().enumerate() {
            assert_time(veh.node_times[0], 0.1 * i as f64, &format!("veh {i} entry"));
        }
    }
}
