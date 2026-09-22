use serde::{Deserialize, Serialize};
use std::collections::{HashMap, VecDeque};

pub type NodeID = usize;
pub type LinkID = usize;
pub type VehID = usize;
pub type PipeIdx = u8;

/// All vehicle classes allowed (bit per class index, up to 64 classes).
pub const ALL_CLASSES: u64 = u64::MAX;

fn all_classes_mask() -> u64 {
    ALL_CLASSES
}

fn default_friction() -> f64 {
    1.0
}

fn default_classes() -> Vec<String> {
    vec!["car".to_string()]
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NodeType {
    Internal = 0,
    Entry = 1,
    Exit = 2,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FundamentalDiagram {
    pub u: f64,
    pub w: f64,
    pub kx: f64,
    pub c: f64,
}

/// Result of testing the Newell storage (supply) constraint at a pipe entry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StorageAvailability {
    /// A storage slot is available now.
    Available,
    /// The next slot is already released downstream; the backward wave
    /// reaches the entry at this absolute time.
    AvailableAt(f64),
    /// No slot has been released yet — a future LinkExit must free one.
    Blocked,
}

// ── Pipes ─────────────────────────────────────────────────────────────────────
//
// A link is partitioned into ordered pipes (index 0 = rightmost lane; per-lane
// granularity when every pipe has lanes = 1). Each pipe is an independent
// kinematic-wave stream — Daganzo's (1997) special-lanes multi-pipe theory:
// strict FIFO *within* a pipe is physically correct; overtaking happens only
// *between* pipes.

/// Serialized pipe specification (network input).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PipeSpec {
    pub lanes: u8,
    /// Vehicle-class permission bitmask: bit i set ⇔ class index i allowed.
    #[serde(default = "all_classes_mask")]
    pub class_mask: u64,
}

/// Runtime per-pipe LWR stream. Rebuilt from `PipeSpec`s at simulation start —
/// never serialized.
#[derive(Debug, Clone)]
pub struct Pipe {
    pub lanes: u8,
    pub class_mask: u64,

    // ── Derived FD scalars, frozen at materialization ──
    /// Discharge capacity of this pipe (veh/s).
    pub capacity: f64,
    /// Exact storage dn = kx · L · lanes (veh).
    pub storage_exact: f64,
    /// Discrete storage (ceil of dn); `None` disables the constraint (kx ≤ 0).
    pub storage_veh: Option<usize>,
    /// Backward wave delay L/w + fractional-slot correction (s).
    pub wave_delay: f64,

    // ── Solver state ──
    pub waiting_queue: VecDeque<VehID>,
    pub entry_queue: VecDeque<VehID>,
    pub vehicles_on_pipe: usize,
    /// Cumulative count of vehicles that ever entered the pipe (N-curve at x = 0).
    pub entered_count: usize,
    /// Slot-release times: for each vehicle that exited, the absolute time at
    /// which its freed slot becomes usable at the entry (exit time + wave delay).
    /// Front = oldest unconsumed release. Bounded by the pipe's storage.
    pub release_times: VecDeque<f64>,
    pub next_available_entry_time: f64,
    /// Number of upcoming slot releases to swallow instead of enqueueing —
    /// set by `reanchor_storage` when a mid-run patch shrinks the storage
    /// below the current occupancy: the first `occupancy − dn` exits only
    /// reduce the over-fill and must not admit anyone. Always 0 in unpatched
    /// runs.
    pub pending_release_discards: usize,
    /// Earliest time the next vehicle may discharge from the downstream end
    /// (1/C headway of this pipe — Python's "UpCapacity" supply term).
    pub next_available_exit_time: f64,
    /// True when `next_available_exit_time` must actually be enforced: armed
    /// at discharge time (upstream-capacity flag on, or friction < 1 at that
    /// instant). Prevents a later sibling recovery from silently bypassing an
    /// already-armed friction throttle.
    pub exit_gate_active: bool,
    /// Dedup marker for scheduled *admission* LinkReady wake-ups. Written
    /// from TWO sites (entry-queue wake, transfer WakeAt as a destination)
    /// that derive the same clock from the same pipe state — a collision
    /// between them costs at most a duplicate event, never a lost wake-up,
    /// because `handle_link_ready` resets the marker and rescans everything
    /// (entry queue + both adjacent nodes). Do not "optimize" that rescan
    /// away, and do NOT add writers with a different clock: the discharge
    /// gate has its own marker below precisely because sharing this slot
    /// lets the two wake families stomp each other's dedup and regenerate
    /// duplicates without bound through the rescan cascade
    /// (tests/friction.rs).
    pub last_scheduled_ready_time: Option<f64>,
    /// Dedup marker for the *discharge-gate* wake-up (friction φ < 1 or the
    /// upstream-capacity flag): the time the pipe's armed exit headway
    /// expires. Separate from `last_scheduled_ready_time` — see above.
    pub last_scheduled_exit_ready_time: Option<f64>,
}

impl Pipe {
    fn new(
        lanes: u8,
        class_mask: u64,
        capacity: f64,
        length: f64,
        fd: &FundamentalDiagram,
    ) -> Self {
        let storage_exact = length * fd.kx * f64::from(lanes);
        let storage_veh = if fd.kx <= 0.0 {
            None
        } else {
            Some((storage_exact.ceil() as usize).max(1))
        };
        let base = if fd.w > 0.0 { length / fd.w } else { 0.0 };
        let frac = storage_exact.ceil() - storage_exact;
        let corr = if capacity > 0.0 { frac / capacity } else { 0.0 };
        Self {
            lanes,
            class_mask,
            capacity,
            storage_exact,
            storage_veh,
            wave_delay: base + corr,
            waiting_queue: VecDeque::new(),
            entry_queue: VecDeque::new(),
            vehicles_on_pipe: 0,
            entered_count: 0,
            release_times: VecDeque::new(),
            next_available_entry_time: 0.0,
            pending_release_discards: 0,
            next_available_exit_time: 0.0,
            exit_gate_active: false,
            last_scheduled_ready_time: None,
            last_scheduled_exit_ready_time: None,
        }
    }

    /// True when `class_id` may use this pipe.
    pub fn allows_class(&self, class_id: usize) -> bool {
        class_id < 64 && (self.class_mask >> class_id) & 1 == 1
    }

    /// Newell's supply constraint at the pipe entry:
    ///   entry_time(n) ≥ exit_time(n − dn) + L/w
    /// The first dn vehicles enter the initially-empty pipe unconstrained;
    /// afterwards each entry consumes the oldest slot release.
    pub fn storage_availability(&self, now: f64) -> StorageAvailability {
        let Some(dn) = self.storage_veh else {
            return StorageAvailability::Available;
        };
        if self.entered_count < dn {
            return StorageAvailability::Available;
        }
        match self.release_times.front() {
            Some(&t) if t <= now => StorageAvailability::Available,
            Some(&t) => StorageAvailability::AvailableAt(t),
            None => StorageAvailability::Blocked,
        }
    }

    /// Record a vehicle physically entering the pipe, consuming a storage
    /// slot. Call only after `storage_availability` returned `Available`.
    pub fn admit(&mut self, now: f64) {
        if let Some(dn) = self.storage_veh {
            if self.entered_count >= dn {
                let front = self.release_times.pop_front();
                debug_assert!(
                    matches!(front, Some(t) if t <= now),
                    "admit() called without an available storage slot: front release = {:?}, now = {}",
                    front,
                    now
                );
            }
        }
        self.entered_count += 1;
        self.vehicles_on_pipe += 1;
    }

    /// True when the storage constraint is currently binding (spillback):
    /// the pipe has consumed its free slots and the next release is not yet
    /// usable. Drives the friction (rubbernecking) coupling between pipes.
    pub fn is_spilled(&self, now: f64) -> bool {
        self.storage_veh.is_some_and(|dn| self.entered_count >= dn)
            && self.storage_availability(now) != StorageAvailability::Available
    }

    /// Recompute the frozen FD scalars in place, preserving all solver queues
    /// (mid-run patch application). If the discrete storage changes, the
    /// release ledger is re-anchored (see `reanchor_storage`); otherwise
    /// in-flight backward waves keep their timestamps — waves already launched
    /// under the old physics stay valid.
    pub fn refresh_frozen(
        &mut self,
        lanes: u8,
        class_mask: u64,
        capacity: f64,
        length: f64,
        fd: &FundamentalDiagram,
    ) {
        let old_storage = self.storage_veh;
        self.lanes = lanes;
        self.class_mask = class_mask;
        self.capacity = capacity;
        self.storage_exact = length * fd.kx * f64::from(lanes);
        self.storage_veh = if fd.kx <= 0.0 {
            None
        } else {
            Some((self.storage_exact.ceil() as usize).max(1))
        };
        let base = if fd.w > 0.0 { length / fd.w } else { 0.0 };
        let frac = self.storage_exact.ceil() - self.storage_exact;
        let corr = if capacity > 0.0 { frac / capacity } else { 0.0 };
        self.wave_delay = base + corr;
        if self.storage_veh != old_storage {
            self.reanchor_storage();
        }
    }

    /// Re-anchor the Newell release ledger to the pipe's current occupancy:
    /// treat the pipe as freshly loaded with its present vehicles. Growth
    /// (added lane) admits into the new empty space immediately; shrink makes
    /// `storage_availability` return `Blocked` — and when the pipe is
    /// over-occupied (occupancy > new dn), the first `occupancy − dn` exits
    /// only reduce the over-fill (their releases are swallowed), so the next
    /// admission waits until occupancy actually falls below the new storage.
    /// Backward waves in flight at the boundary are discarded — a bounded
    /// one-time transient.
    pub fn reanchor_storage(&mut self) {
        self.entered_count = self.vehicles_on_pipe;
        self.release_times.clear();
        self.pending_release_discards = match self.storage_veh {
            Some(dn) => self.vehicles_on_pipe.saturating_sub(dn),
            None => 0,
        };
    }
}

// ── Dynamic patches (time-windowed link overrides) ───────────────────────────

/// Partial link-attribute override. `None` = keep the current value. Every
/// schedule entry emitted by `patch::apply_patches` is self-contained: it
/// carries, for each field any patch window ever touches on that link, the
/// full effective value at that boundary (falling back to the base network
/// value when no window is active) — so applying entries in order never needs
/// the base network.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct LinkAttrs {
    #[serde(default)]
    pub speed: Option<f64>,
    /// Whole-link capacity (veh/s) — same convention as `Link.capacity`.
    #[serde(default)]
    pub capacity: Option<f64>,
    #[serde(default)]
    pub fd_u: Option<f64>,
    #[serde(default)]
    pub fd_w: Option<f64>,
    /// Per-lane jam density, as everywhere in the engine.
    #[serde(default)]
    pub fd_kx: Option<f64>,
    /// Per-lane capacity, as everywhere in the engine.
    #[serde(default)]
    pub fd_c: Option<f64>,
    #[serde(default)]
    pub num_lanes: Option<u8>,
    #[serde(default)]
    pub friction: Option<f64>,
    /// Replacement pipe partition — the pipe COUNT must not change mid-run
    /// (validated at compile time; queues live on pipes).
    #[serde(default)]
    pub pipe_specs: Option<Vec<PipeSpec>>,
    #[serde(default)]
    pub moves: Option<HashMap<LinkID, Vec<PipeIdx>>>,
    /// Per-pipe class-mask override (index-parallel to pipes). Drives lane
    /// closures: mask 0 = no new entrants (drain semantics).
    #[serde(default)]
    pub class_masks: Option<Vec<u64>>,
}

/// One pre-resolved point of the dynamic-patch timeline: at `time`, the link's
/// effective attributes become `attrs`. Produced by `patch::apply_patches`,
/// consumed by `Simulation` as `ApplyPatch` events (event `veh_id` = index
/// into `Scenario::link_schedule`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LinkStateChange {
    pub time: f64,
    pub link_id: LinkID,
    pub attrs: LinkAttrs,
}

// ── Link ──────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Link {
    pub id: LinkID,
    pub node_up: NodeID,
    pub node_down: NodeID,
    pub length: f64,
    pub speed: f64,
    pub num_lanes: u8,
    pub capacity: f64,
    pub fd: FundamentalDiagram,
    pub points: Vec<(f64, f64)>,
    pub priority: f64,

    /// Pipe partition, index 0 = rightmost lane group. Empty ⇒ default single
    /// pipe spanning all `num_lanes`, all classes (single-stream behaviour).
    #[serde(default)]
    pub pipe_specs: Vec<PipeSpec>,
    /// Movement restriction: out-link id → pipe indices that serve it.
    /// Missing key = movement reachable from every pipe.
    #[serde(default)]
    pub moves: HashMap<LinkID, Vec<PipeIdx>>,
    /// Friction φ ∈ (0, 1]: when a sibling pipe on this link has spilled back,
    /// other pipes discharge with headway 1/(C·φ). 1.0 disables the coupling.
    #[serde(default = "default_friction")]
    pub friction: f64,

    /// Runtime pipes — rebuilt by `materialize_pipes` at simulation start.
    #[serde(skip)]
    pub pipes: Vec<Pipe>,
}

impl Link {
    pub fn new(
        id: LinkID,
        node_up: NodeID,
        node_down: NodeID,
        length: f64,
        speed: f64,
        num_lanes: u8,
        capacity: f64,
        fd: FundamentalDiagram,
        points: Vec<(f64, f64)>,
    ) -> Self {
        Self {
            id,
            node_up,
            node_down,
            length,
            speed,
            num_lanes,
            capacity,
            fd,
            points,
            priority: 1.0,
            pipe_specs: Vec::new(),
            moves: HashMap::new(),
            friction: 1.0,
            pipes: Vec::new(),
        }
    }

    /// Build the runtime pipes from `pipe_specs`. Called from `Simulation::new`
    /// (idempotent — resets all solver state).
    ///
    /// Back-compat invariant: with no explicit specs the single default pipe
    /// copies `self.capacity` verbatim (NOT fd.c × lanes — GeoJSON may set
    /// fd_c independently), so every headway and wave-delay value is
    /// bit-identical to the single-stream engine.
    pub fn materialize_pipes(&mut self) {
        // Backstop for callers that skip validate(): PipeIdx is u8, more than
        // 255 pipes would silently wrap every index cast in the engine.
        assert!(
            self.pipe_specs.len() <= 255,
            "Link {}: at most 255 pipes are supported (got {})",
            self.id,
            self.pipe_specs.len()
        );
        self.pipes = if self.pipe_specs.is_empty() {
            vec![Pipe::new(
                self.num_lanes,
                ALL_CLASSES,
                self.capacity,
                self.length,
                &self.fd,
            )]
        } else {
            self.pipe_specs
                .iter()
                .map(|spec| {
                    let capacity = self.fd.c * f64::from(spec.lanes);
                    Pipe::new(spec.lanes, spec.class_mask, capacity, self.length, &self.fd)
                })
                .collect()
        };
    }

    /// Exact storage of the whole link in vehicles: dn = kx · L · lanes.
    /// (Param-derived facade — valid as the Σ over pipes.)
    pub fn storage_exact(&self) -> f64 {
        self.length * self.fd.kx * f64::from(self.num_lanes)
    }

    /// Discrete whole-link storage, rounded up as in the Python reference.
    /// `None` disables the constraint (kx ≤ 0 → unlimited storage).
    pub fn storage_veh(&self) -> Option<usize> {
        if self.fd.kx <= 0.0 {
            return None;
        }
        Some((self.storage_exact().ceil() as usize).max(1))
    }

    /// True when a vehicle of `class_id` that will continue to `next_link`
    /// (None = trip ends here) can use at least one pipe of this link.
    /// Based on `pipe_specs`, so it works before materialization — used by
    /// validation and assignment to reject unreachable paths early.
    pub fn serves(&self, class_id: usize, next_link: Option<LinkID>) -> bool {
        let n = self.pipe_specs.len().max(1);
        let by_move = next_link.and_then(|nl| self.moves.get(&nl));
        (0..n).any(|p| {
            let mask = self.pipe_specs.get(p).map_or(ALL_CLASSES, |s| s.class_mask);
            let class_ok = class_id < 64 && (mask >> class_id) & 1 == 1;
            let move_ok = by_move.is_none_or(|v| v.contains(&(p as PipeIdx)));
            class_ok && move_ok
        })
    }

    /// Overlay a partial attribute override onto this link's parameters
    /// (mid-run patch application). Does NOT touch the runtime pipes — call
    /// `refresh_pipes_in_place` afterwards to propagate to the frozen scalars.
    pub fn apply_attrs(&mut self, attrs: &LinkAttrs) {
        if let Some(v) = attrs.speed {
            self.speed = v;
        }
        if let Some(v) = attrs.capacity {
            self.capacity = v;
        }
        if let Some(v) = attrs.fd_u {
            self.fd.u = v;
        }
        if let Some(v) = attrs.fd_w {
            self.fd.w = v;
        }
        if let Some(v) = attrs.fd_kx {
            self.fd.kx = v;
        }
        if let Some(v) = attrs.fd_c {
            self.fd.c = v;
        }
        if let Some(v) = attrs.num_lanes {
            self.num_lanes = v;
        }
        if let Some(v) = attrs.friction {
            self.friction = v;
        }
        if let Some(specs) = &attrs.pipe_specs {
            // Pipe count is immutable mid-run (validated at compile time);
            // degrade gracefully in release rather than corrupt queue state.
            debug_assert_eq!(
                specs.len().max(1),
                self.pipes.len().max(1),
                "Link {}: patch must not change the pipe count",
                self.id
            );
            if specs.len().max(1) == self.pipes.len().max(1) {
                self.pipe_specs = specs.clone();
            }
        }
        if let Some(moves) = &attrs.moves {
            self.moves = moves.clone();
        }
    }

    /// Recompute the frozen scalars of the existing runtime pipes IN PLACE,
    /// preserving all solver queues — the mid-run counterpart of
    /// `materialize_pipes`. `class_masks` optionally overrides the per-pipe
    /// permission masks (index-parallel); pipes keep their current mask when
    /// absent. The default single pipe keeps copying `self.capacity` verbatim
    /// (back-compat invariant of `materialize_pipes`).
    pub fn refresh_pipes_in_place(&mut self, class_masks: Option<&[u64]>) {
        let mask_for = |i: usize, current: u64| -> u64 {
            class_masks
                .and_then(|m| m.get(i).copied())
                .unwrap_or(current)
        };
        if self.pipe_specs.is_empty() {
            if let Some(pipe) = self.pipes.first_mut() {
                let mask = mask_for(0, pipe.class_mask);
                pipe.refresh_frozen(self.num_lanes, mask, self.capacity, self.length, &self.fd);
            }
        } else {
            debug_assert_eq!(
                self.pipe_specs.len(),
                self.pipes.len(),
                "Link {}: pipe_specs/pipes length mismatch in refresh",
                self.id
            );
            for (i, spec) in self.pipe_specs.iter().enumerate() {
                let Some(pipe) = self.pipes.get_mut(i) else {
                    break;
                };
                let mask = mask_for(i, pipe.class_mask);
                let capacity = self.fd.c * f64::from(spec.lanes);
                pipe.refresh_frozen(spec.lanes, mask, capacity, self.length, &self.fd);
            }
        }
    }

    /// Whole-link backward wave delay: L/w plus the fractional-slot correction.
    ///
    /// DELIBERATE DEVIATION from the Python reference: main_simulation_meso.py
    /// computes `dt + (dn_ceil − dn) * FD["C"]`, which is dimensionally
    /// inconsistent (vehicles × veh/s ≠ seconds). The intended correction is
    /// the time for the discharge to advance by the fractional vehicle:
    /// frac / capacity (seconds).
    pub fn wave_delay(&self) -> f64 {
        let base = if self.fd.w > 0.0 {
            self.length / self.fd.w
        } else {
            0.0
        };
        let dn = self.storage_exact();
        let frac = dn.ceil() - dn;
        let corr = if self.capacity > 0.0 {
            frac / self.capacity
        } else {
            0.0
        };
        base + corr
    }
}

// ── Nodes / vehicles / demand ────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Node {
    pub id: NodeID,
    pub node_type: NodeType,
    pub incoming_links: Vec<LinkID>,
    pub outgoing_links: Vec<LinkID>,
    pub points: (f64, f64),
    pub signals: Vec<usize>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum VehicleState {
    #[default]
    QueuedAtEntry,
    Moving,
    Waiting,
    Exited,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Vehicle {
    pub id: VehID,
    pub class_id: usize,
    pub path: Vec<LinkID>,
    pub start_time: f64,
    pub origin: NodeID,
    pub destination: NodeID,

    #[serde(skip)]
    pub state: VehicleState,
    #[serde(skip)]
    pub current_link_idx: usize,
    #[serde(skip)]
    pub node_times: Vec<f64>,
    /// Pipe taken on each entered link — index-parallel to the prefix of
    /// `path` the vehicle has entered. Drives per-pipe analysis.
    #[serde(skip)]
    pub pipes_taken: Vec<PipeIdx>,
}

impl Vehicle {
    pub fn new(
        id: VehID,
        class_id: usize,
        path: Vec<LinkID>,
        start_time: f64,
        origin: NodeID,
        destination: NodeID,
    ) -> Self {
        Self {
            id,
            class_id,
            path,
            start_time,
            origin,
            destination,
            state: VehicleState::default(),
            current_link_idx: 0,
            node_times: Vec::new(),
            pipes_taken: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Demand {
    pub period_start: f64,
    pub period_end: f64,
    pub origin: NodeID,
    pub destination: NodeID,
    pub flow: f64, // vehicles per hour? or total vehicles? Let's say flow (veh/h) or count. Python uses count/duration * 3600. Let's store count.
    pub count: f64,
    /// Vehicle class name, resolved against `Scenario.classes` at assignment.
    /// `None` = first class.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub class: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Scenario {
    pub nodes: Vec<Node>,
    pub links: Vec<Link>,
    pub vehicles: Vec<Vehicle>,
    pub demand: Vec<Demand>,
    pub start_time: f64,
    pub duration: f64,
    /// Vehicle class names; index = class_id, ≤ 64 entries.
    #[serde(default = "default_classes")]
    pub classes: Vec<String>,
    /// Pre-resolved dynamic-patch timeline, sorted by (time, link_id).
    /// Empty for unpatched scenarios — the engine is then bit-identical to
    /// the schedule-free engine (fixture gate).
    #[serde(default)]
    pub link_schedule: Vec<LinkStateChange>,
    /// Links excluded from the assignment graph (closed for the whole
    /// horizon by a patch — "removed" without renumbering ids).
    #[serde(default)]
    pub assignment_excluded: Vec<LinkID>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fd() -> FundamentalDiagram {
        FundamentalDiagram {
            u: 10.0,
            w: 5.0,
            kx: 0.02,
            c: 0.5,
        }
    }

    fn link() -> Link {
        Link::new(0, 0, 1, 100.0, 10.0, 3, 1.7, fd(), vec![])
    }

    #[test]
    fn default_materialization_is_single_pipe_copying_link_capacity() {
        let mut l = link();
        l.materialize_pipes();
        assert_eq!(l.pipes.len(), 1);
        let p = &l.pipes[0];
        assert_eq!(p.lanes, 3);
        assert_eq!(p.class_mask, ALL_CLASSES);
        // Back-compat invariant: capacity copied verbatim (1.7 != fd.c * lanes = 1.5)
        assert_eq!(p.capacity, 1.7);
        assert_eq!(p.storage_exact, l.storage_exact());
        assert_eq!(p.storage_veh, l.storage_veh());
        assert_eq!(p.wave_delay, l.wave_delay());
    }

    #[test]
    fn explicit_specs_derive_per_pipe_scalars() {
        let mut l = link();
        l.pipe_specs = vec![
            PipeSpec {
                lanes: 1,
                class_mask: 0b10,
            }, // class 1 only (e.g. bus)
            PipeSpec {
                lanes: 2,
                class_mask: ALL_CLASSES,
            },
        ];
        l.materialize_pipes();
        assert_eq!(l.pipes.len(), 2);
        // Pipe capacity = fd.c * lanes
        assert!((l.pipes[0].capacity - 0.5).abs() < 1e-12);
        assert!((l.pipes[1].capacity - 1.0).abs() < 1e-12);
        // Storage: kx * L * lanes = 0.02*100*1 = 2 and 0.02*100*2 = 4
        assert_eq!(l.pipes[0].storage_veh, Some(2));
        assert_eq!(l.pipes[1].storage_veh, Some(4));
        // Wave delay: integer dn -> exactly L/w = 20 s
        assert!((l.pipes[0].wave_delay - 20.0).abs() < 1e-12);
        // Class permissions
        assert!(l.pipes[0].allows_class(1));
        assert!(!l.pipes[0].allows_class(0));
        assert!(l.pipes[1].allows_class(0));
    }

    #[test]
    fn materialization_is_idempotent_and_resets_state() {
        let mut l = link();
        l.materialize_pipes();
        l.pipes[0].entered_count = 7;
        l.pipes[0].entry_queue.push_back(1);
        l.materialize_pipes();
        assert_eq!(l.pipes[0].entered_count, 0);
        assert!(l.pipes[0].entry_queue.is_empty());
    }

    #[test]
    fn old_scenario_json_deserializes_with_defaults() {
        // A pre-pipe Link JSON: no pipe_specs / moves / friction fields.
        let json = r#"{
            "id": 0, "node_up": 0, "node_down": 1, "length": 100.0,
            "speed": 10.0, "num_lanes": 2, "capacity": 1.0,
            "fd": {"u": 10.0, "w": 5.0, "kx": 0.1, "c": 0.5},
            "points": [], "priority": 1.0
        }"#;
        let l: Link = serde_json::from_str(json).expect("old link JSON must load");
        assert!(l.pipe_specs.is_empty());
        assert!(l.moves.is_empty());
        assert_eq!(l.friction, 1.0);

        // A pre-pipe Demand JSON: no class field.
        let json = r#"{"period_start":0.0,"period_end":10.0,"origin":0,"destination":1,"flow":1.0,"count":5.0}"#;
        let d: Demand = serde_json::from_str(json).expect("old demand JSON must load");
        assert_eq!(d.class, None);
    }
}
