use crate::model::{
    Link, LinkID, NodeID, PipeIdx, Scenario, StorageAvailability, VehID, VehicleState,
};
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
            EventType::ApplyPatch => 0,
            EventType::LinkReady => 1,
            EventType::LinkExit => 2,
            EventType::VehicleArrival => 3,
            EventType::VehicleEntry => 4,
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
        let mut sim = Simulation {
            scenario,
            events: BinaryHeap::new(),
            current_time: 0.0,
            upstream_capacity: false,
        };
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
        let mut events_processed = 0;
        let max_time = self.scenario.start_time + self.scenario.duration;

        while let Some(event) = self.events.pop() {
            if event.time > max_time {
                break;
            }
            self.current_time = event.time;
            events_processed += 1;

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
        let incoming_links = self.scenario.nodes[node_id].incoming_links.clone();

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
                Link::new(0, 0, 1, 100.0, 10.0, lanes_l0, 10.0, fd0, vec![]),
                Link::new(1, 1, 2, 100.0, 10.0, 1, c1, fd1, vec![]),
            ],
            vehicles,
            demand: vec![],
            start_time: 0.0,
            duration: 100_000.0,
            classes: vec!["car".to_string()],
            link_schedule: Vec::new(),
            assignment_excluded: Vec::new(),
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
                    signals: vec![],
                },
                Node {
                    id: 1,
                    node_type: NodeType::Internal,
                    incoming_links: vec![0],
                    outgoing_links: vec![1, 2],
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
                Node {
                    id: 3,
                    node_type: NodeType::Exit,
                    incoming_links: vec![2],
                    outgoing_links: vec![],
                    points: (2.0, 1.0),
                    signals: vec![],
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
            assignment_excluded: Vec::new(),
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
