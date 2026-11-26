//! Traffic signals, tolls, and the interface for external controllers.
//!
//! The engine holds the *mechanics* only:
//!
//! - **signal groups** controlling movements (incoming link → outgoing link)
//!   at a node. A red group holds the head vehicle of a pipe whose movement
//!   it controls; the vehicles behind it in that pipe wait too (FIFO), so a
//!   red left turn blocks a shared lane but not a dedicated pocket (pipes +
//!   moves). A group may admit at most N vehicles per green (ramp metering);
//! - **fixed-time plans**, which are data: per period, a cycle, an offset and
//!   the green windows of each group within the cycle;
//! - **tolls**: a deterministic server per pipe, with a service time per
//!   vehicle class (departure = max(arrival, previous departure) + service);
//! - the **controller interface**: any control algorithm (ALINEA, actuated
//!   or adaptive control…) lives outside the engine, behind [`Controller`].
//!   The engine wakes a controller at the times it asks for, shows it the
//!   sensors ([`ControlView`]) and applies the commands it returns.
//!
//! Outside every plan period a group is in its `off` state (green by
//! default: a dark signal lets traffic through).

use std::collections::BTreeMap;

use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};

use crate::model::{LinkID, NodeID, PipeIdx, Scenario};

// ── Input: signals.json ───────────────────────────────────────────────────────

fn default_sneakers() -> u32 {
    2
}

fn yes() -> bool {
    true
}

fn is_true(b: &bool) -> bool {
    *b
}

fn default_clearance() -> f64 {
    3.0
}

/// Everything `signals.json` declares. Empty for a scenario without
/// signals, which then runs exactly as before.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SignalNetwork {
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub groups: Vec<SignalGroup>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub plans: Vec<FixedTimePlan>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tolls: Vec<Toll>,
}

impl SignalNetwork {
    pub fn is_empty(&self) -> bool {
        self.groups.is_empty() && self.tolls.is_empty()
    }
}

/// A movement through a node: from an incoming link to an outgoing link
/// (`to` absent = every movement from `from`, trips ending there included).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Movement {
    pub from: LinkID,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub to: Option<LinkID>,
}

/// State of a group outside every plan period.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OffState {
    #[default]
    Green,
    Red,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SignalGroup {
    pub id: String,
    pub node: NodeID,
    pub movements: Vec<Movement>,
    /// At most this many vehicles pass per green (ramp metering: 1).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vehicles_per_green: Option<u32>,
    #[serde(default)]
    pub off: OffState,
    /// Permitted vehicles (movements that give way at a priority junction)
    /// already at the line that may still clear after the group turns red
    /// (HCM "sneakers"), within `clearance` seconds of the red.
    #[serde(default = "default_sneakers")]
    pub sneakers: u32,
    #[serde(default = "default_clearance")]
    pub clearance: f64,
}

/// A fixed-time plan for one period: cycles of `cycle` seconds starting at
/// `from + offset + k·cycle`; in each cycle, group g is green over the
/// windows `greens[g]` = [[start, end], …] (seconds from the cycle start,
/// 0 ≤ start < end ≤ start + cycle; an end past `cycle` runs into the next
/// cycle). Groups of the plan's `groups` without windows stay red.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FixedTimePlan {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    pub from: f64,
    pub until: f64,
    pub cycle: f64,
    #[serde(default)]
    pub offset: f64,
    /// Groups this plan controls (all groups listed in `greens` are added).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub groups: Vec<String>,
    pub greens: BTreeMap<String, Vec<[f64; 2]>>,
}

impl FixedTimePlan {
    /// Every group the plan controls.
    pub fn controlled(&self) -> Vec<&str> {
        let mut g: Vec<&str> = self.groups.iter().map(String::as_str).collect();
        for k in self.greens.keys() {
            if !g.contains(&k.as_str()) {
                g.push(k);
            }
        }
        g
    }
}

/// A toll booth at the downstream end of `link`: each pipe (all, or those
/// listed) serves one vehicle at a time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Toll {
    pub link: LinkID,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pipes: Option<Vec<PipeIdx>>,
    /// Service time (s) of a vehicle whose class is not in `by_class`.
    pub service_time: f64,
    /// Service time per class name.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub by_class: BTreeMap<String, f64>,
}

/// Parse and check `signals.json` against a compiled scenario.
pub fn compile(value: serde_json::Value, scenario: &Scenario) -> Result<SignalNetwork> {
    let net: SignalNetwork = serde_json::from_value(value).context("invalid signals.json")?;
    validate(&net, scenario)?;
    Ok(net)
}

/// Structural checks of a signal network against its scenario.
pub fn validate(net: &SignalNetwork, s: &Scenario) -> Result<()> {
    let mut ids = std::collections::BTreeSet::new();
    let mut owner: BTreeMap<(LinkID, Option<LinkID>), &str> = BTreeMap::new();
    for g in &net.groups {
        if !ids.insert(g.id.as_str()) {
            bail!("signal group '{}' declared twice", g.id);
        }
        let Some(node) = s.nodes.get(g.node) else {
            bail!("signal group '{}': unknown node {}", g.id, g.node);
        };
        if node.roundabout.is_some() {
            bail!("signal group '{}': node {} is a roundabout (signalised roundabouts are not supported yet)", g.id, g.node);
        }
        if g.movements.is_empty() {
            bail!("signal group '{}' controls no movement", g.id);
        }
        if !(g.clearance.is_finite() && g.clearance >= 0.0) {
            bail!("signal group '{}': clearance must be ≥ 0", g.id);
        }
        if g.vehicles_per_green == Some(0) {
            bail!("signal group '{}': vehicles_per_green must be ≥ 1", g.id);
        }
        for m in &g.movements {
            if !node.incoming_links.contains(&m.from) {
                bail!("signal group '{}': link {} does not end at node {}", g.id, m.from, g.node);
            }
            if let Some(to) = m.to {
                if !node.outgoing_links.contains(&to) {
                    bail!("signal group '{}': link {} does not start at node {}", g.id, to, g.node);
                }
            }
            // One group per movement; an "every movement" entry conflicts
            // with any specific one on the same link.
            let clash = owner.get(&(m.from, m.to)).or_else(|| {
                if m.to.is_none() {
                    owner.iter().find(|((f, _), _)| *f == m.from).map(|(_, g)| g)
                } else {
                    owner.get(&(m.from, None))
                }
            });
            if let Some(other) = clash {
                bail!("movement {} → {:?} is controlled by both '{}' and '{}'", m.from, m.to, other, g.id);
            }
            owner.insert((m.from, m.to), &g.id);
        }
    }
    for (i, p) in net.plans.iter().enumerate() {
        let what = p.name.clone().unwrap_or_else(|| format!("plan {i}"));
        if !(p.cycle.is_finite() && p.cycle > 0.0) {
            bail!("{what}: cycle must be > 0");
        }
        if !(p.from.is_finite() && p.until.is_finite() && p.until > p.from) {
            bail!("{what}: needs from < until");
        }
        for g in p.controlled() {
            if !ids.contains(g) {
                bail!("{what}: unknown signal group '{g}'");
            }
        }
        for (g, windows) in &p.greens {
            for &[a, b] in windows {
                if !(a.is_finite() && b.is_finite() && 0.0 <= a && a < b && b <= a + p.cycle) {
                    bail!("{what}: group '{g}' green [{a}, {b}] must satisfy 0 ≤ start < end ≤ start + cycle");
                }
            }
        }
    }
    // Two plans may not control one group over overlapping periods.
    for (i, p) in net.plans.iter().enumerate() {
        for q in &net.plans[i + 1..] {
            if p.from < q.until && q.from < p.until {
                if let Some(g) = p.controlled().into_iter().find(|g| q.controlled().contains(g)) {
                    bail!("signal group '{g}' is in two plans over overlapping periods");
                }
            }
        }
    }
    for t in &net.tolls {
        let Some(link) = s.links.get(t.link) else {
            bail!("toll: unknown link {}", t.link);
        };
        if s.nodes.get(link.node_down).is_some_and(|n| n.roundabout.is_some()) {
            bail!("toll on link {}: it ends at a roundabout (not supported yet)", t.link);
        }
        let n_pipes = link.pipe_specs.len().max(1);
        if let Some(ps) = &t.pipes {
            if ps.iter().any(|&p| usize::from(p) >= n_pipes) {
                bail!("toll on link {}: pipe index out of range ({} pipes)", t.link, n_pipes);
            }
        }
        if !(t.service_time.is_finite() && t.service_time >= 0.0) {
            bail!("toll on link {}: service_time must be ≥ 0", t.link);
        }
        for (c, &v) in &t.by_class {
            if !s.classes.contains(c) {
                bail!("toll on link {}: unknown class '{c}'", t.link);
            }
            if !(v.is_finite() && v >= 0.0) {
                bail!("toll on link {}: service time of '{c}' must be ≥ 0", t.link);
            }
        }
    }
    Ok(())
}

// ── Fixed-time expansion ──────────────────────────────────────────────────────

/// What a group shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Aspect {
    Red,
    /// Green of a plan (or of a controller): the per-green cap applies.
    Green,
    /// Dark signal, outside every plan period with `off: green`: traffic
    /// passes as on green, with no per-green cap.
    Off,
}

impl Aspect {
    pub fn passes(self) -> bool {
        self != Aspect::Red
    }
    pub(crate) fn code(self) -> u8 {
        match self {
            Aspect::Red => 0,
            Aspect::Green => 1,
            Aspect::Off => 2,
        }
    }
    pub(crate) fn from_code(c: u8) -> Self {
        match c {
            1 => Aspect::Green,
            2 => Aspect::Off,
            _ => Aspect::Red,
        }
    }
}

/// The aspect changes (time, aspect) of every group over [start, end], from
/// the fixed-time plans and the `off` states. Each list starts with the
/// aspect at `start`.
pub fn expand(net: &SignalNetwork, start: f64, end: f64) -> Vec<Vec<(f64, Aspect)>> {
    net.groups
        .iter()
        .map(|g| {
            // Plan greens inside plan periods, and the gaps between periods
            // as dark-signal intervals when the group is green when off.
            let mut periods: Vec<(f64, f64)> = Vec::new();
            let mut intervals: Vec<(f64, f64, Aspect)> = Vec::new();
            for p in net.plans.iter().filter(|p| p.controlled().contains(&g.id.as_str())) {
                let (from, until) = (p.from.max(start), p.until.min(end));
                if from >= until {
                    continue;
                }
                periods.push((from, until));
                let Some(windows) = p.greens.get(&g.id) else { continue };
                let base = p.from + p.offset;
                // First cycle whose windows can reach `from`.
                let mut k = ((from - base) / p.cycle).floor() - 1.0;
                loop {
                    let c0 = base + k * p.cycle;
                    if c0 >= until {
                        break;
                    }
                    for &[a, b] in windows {
                        let (s0, s1) = ((c0 + a).max(from), (c0 + b).min(until));
                        if s0 < s1 {
                            intervals.push((s0, s1, Aspect::Green));
                        }
                    }
                    k += 1.0;
                }
            }
            if g.off == OffState::Green {
                periods.sort_by(|a, b| a.0.total_cmp(&b.0));
                let mut t = start;
                for &(a, b) in &periods {
                    if a > t {
                        intervals.push((t, a, Aspect::Off));
                    }
                    t = t.max(b);
                }
                if t < end {
                    intervals.push((t, end, Aspect::Off));
                }
            }
            timeline(intervals, start, end)
        })
        .collect()
}

/// Turn intervals into aspect changes starting at `start`. Overlapping or
/// touching intervals of one aspect merge; a dark interval and a plan green
/// never do, so the cap is reset where a plan begins or ends.
fn timeline(mut intervals: Vec<(f64, f64, Aspect)>, start: f64, end: f64) -> Vec<(f64, Aspect)> {
    intervals.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.2.code().cmp(&b.2.code())));
    let mut merged: Vec<(f64, f64, Aspect)> = Vec::new();
    for (a, b, k) in intervals {
        match merged.last_mut() {
            Some(last) if last.2 == k && a <= last.1 => last.1 = last.1.max(b),
            _ => merged.push((a, b, k)),
        }
    }
    let mut out: Vec<(f64, Aspect)> = vec![(start, Aspect::Red)];
    for (a, b, k) in merged {
        out.push((a.max(start), k));
        out.push((b, Aspect::Red));
    }
    // At one instant the last change wins (a green starting where the
    // previous one ends); changes at or after the horizon end never apply.
    let mut norm: Vec<(f64, Aspect)> = Vec::new();
    for (t, k) in out {
        if t != start && t >= end {
            continue;
        }
        match norm.last_mut() {
            Some(last) if last.0 == t => last.1 = k,
            _ => norm.push((t, k)),
        }
    }
    norm.dedup_by(|b, a| a.1 == b.1);
    norm
}

// ── Runtime ───────────────────────────────────────────────────────────────────

/// A switch actually applied during a run.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SwitchRecord {
    pub group: usize,
    pub time: f64,
    pub green: bool,
    /// False for a dark-signal (off-state) green: no per-green cap.
    #[serde(default = "yes", skip_serializing_if = "is_true")]
    pub capped: bool,
}

/// Run-time state of one signal group.
#[derive(Debug, Clone)]
pub struct GroupState {
    pub id: String,
    pub node: NodeID,
    pub green: bool,
    /// The green comes from a plan or a controller (cap applies), not from
    /// the dark off state.
    pub capped: bool,
    pub vehicles_per_green: Option<u32>,
    /// Vehicles passed since the group last turned green.
    pub served: u32,
    pub sneakers: u32,
    pub clearance: f64,
    /// Time of the last switch to red, and the sneakers still allowed.
    pub red_since: f64,
    pub sneak_left: u32,
}

/// Run-time state of one toll lane.
#[derive(Debug, Clone, Copy, Default)]
pub struct TollLane {
    pub last_departure: f64,
}

/// Everything the engine needs at run time; built once per simulation.
#[derive(Debug, Clone, Default)]
pub struct SignalRuntime {
    pub groups: Vec<GroupState>,
    /// (from link, to link or None) → group.
    by_movement: BTreeMap<(LinkID, Option<LinkID>), usize>,
    /// Per link: has at least one controlled movement.
    controlled_link: Vec<bool>,
    /// Per (link, pipe): toll service times per class (None = no toll).
    tolls: BTreeMap<(LinkID, PipeIdx), Vec<f64>>,
    pub toll_lanes: BTreeMap<(LinkID, PipeIdx), TollLane>,
}

impl SignalRuntime {
    pub fn new(net: &SignalNetwork, s: &Scenario) -> Self {
        let mut rt = SignalRuntime {
            controlled_link: vec![false; s.links.len()],
            ..Default::default()
        };
        for (i, g) in net.groups.iter().enumerate() {
            rt.groups.push(GroupState {
                id: g.id.clone(),
                node: g.node,
                green: true,
                capped: false,
                vehicles_per_green: g.vehicles_per_green,
                served: 0,
                sneakers: g.sneakers,
                clearance: g.clearance,
                red_since: f64::NEG_INFINITY,
                sneak_left: 0,
            });
            for m in &g.movements {
                rt.by_movement.insert((m.from, m.to), i);
                if let Some(c) = rt.controlled_link.get_mut(m.from) {
                    *c = true;
                }
            }
        }
        for t in &net.tolls {
            let n_pipes = s.links[t.link].pipe_specs.len().max(1);
            let pipes: Vec<PipeIdx> = match &t.pipes {
                Some(p) => p.clone(),
                None => (0..n_pipes as PipeIdx).collect(),
            };
            let per_class: Vec<f64> = s
                .classes
                .iter()
                .map(|c| t.by_class.get(c).copied().unwrap_or(t.service_time))
                .collect();
            for p in pipes {
                rt.tolls.insert((t.link, p), per_class.clone());
                rt.toll_lanes.insert((t.link, p), TollLane { last_departure: f64::NEG_INFINITY });
            }
        }
        rt
    }

    pub fn is_empty(&self) -> bool {
        self.groups.is_empty() && self.tolls.is_empty()
    }

    /// Group controlling the movement `from → to`, if any.
    pub fn group_of(&self, from: LinkID, to: Option<LinkID>) -> Option<usize> {
        if !self.controlled_link.get(from).copied().unwrap_or(false) {
            return None;
        }
        self.by_movement
            .get(&(from, to))
            .or_else(|| self.by_movement.get(&(from, None)))
            .copied()
    }

    /// A permitted vehicle may still clear after the red: the group turned
    /// red less than `clearance` ago and has sneakers left.
    pub fn sneak_open(&self, group: usize, now: f64) -> bool {
        let g = &self.groups[group];
        !g.green && g.sneak_left > 0 && now < g.red_since + g.clearance
    }

    /// May a vehicle take the movement controlled by `group` now?
    pub fn open(&self, group: usize) -> bool {
        let g = &self.groups[group];
        g.green && (!g.capped || g.vehicles_per_green.is_none_or(|n| g.served < n))
    }

    /// Service time of a vehicle of `class` at the toll of (link, pipe).
    pub fn toll_service(&self, link: LinkID, pipe: PipeIdx, class: usize) -> Option<f64> {
        self.tolls.get(&(link, pipe)).map(|v| v.get(class).copied().unwrap_or(0.0))
    }

    pub fn group_index(&self, id: &str) -> Option<usize> {
        self.groups.iter().position(|g| g.id == id)
    }
}

// ── External controllers ──────────────────────────────────────────────────────

/// What a controller asks the engine to do.
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    /// Turn `group` green or red at time `at` (now if `at` is past).
    Switch { group: String, at: f64, green: bool },
    /// Wake this controller again at time `at`.
    WakeAt(f64),
    /// Change the per-green vehicle cap of `group` (None = no cap).
    SetVehiclesPerGreen { group: String, n: Option<u32> },
}

/// A control algorithm, compiled outside the engine. It must be
/// deterministic for runs to stay reproducible.
pub trait Controller: Send {
    /// Called once at the start of the horizon.
    fn start(&mut self, view: &ControlView) -> Vec<Command>;
    /// Called at every time this controller asked for with `WakeAt`.
    fn wake(&mut self, view: &ControlView) -> Vec<Command>;
}

/// Read-only sensors offered to controllers.
pub struct ControlView<'a> {
    pub(crate) time: f64,
    pub(crate) scenario: &'a Scenario,
    pub(crate) signals: &'a SignalRuntime,
    pub(crate) entered: &'a [u64],
    pub(crate) exited: &'a [u64],
}

impl ControlView<'_> {
    /// Current simulation time (s).
    pub fn time(&self) -> f64 {
        self.time
    }
    /// Index of a signal group, by id.
    pub fn group(&self, id: &str) -> Option<usize> {
        self.signals.group_index(id)
    }
    pub fn is_green(&self, id: &str) -> Option<bool> {
        self.group(id).map(|g| self.signals.groups[g].green)
    }
    /// Vehicles on `link` (moving or queued at its downstream end).
    pub fn vehicles(&self, link: LinkID) -> usize {
        self.scenario.links[link].pipes.iter().map(|p| p.vehicles_on_pipe).sum()
    }
    /// Vehicles waiting at the downstream end of `link`.
    pub fn queue(&self, link: LinkID) -> usize {
        self.scenario.links[link].pipes.iter().map(|p| p.waiting_queue.len()).sum()
    }
    /// Vehicles waiting upstream of `link`, not yet admitted.
    pub fn entry_queue(&self, link: LinkID) -> usize {
        self.scenario.links[link].pipes.iter().map(|p| p.entry_queue.len()).sum()
    }
    /// Mean density on `link` (veh/m).
    pub fn density(&self, link: LinkID) -> f64 {
        let l = &self.scenario.links[link];
        self.vehicles(link) as f64 / l.length
    }
    /// Occupancy proxy in [0, 1]: density over jam density.
    pub fn occupancy(&self, link: LinkID) -> f64 {
        let l = &self.scenario.links[link];
        let jam = l.fd.kx * f64::from(l.num_lanes);
        if jam > 0.0 {
            (self.density(link) / jam).min(1.0)
        } else {
            0.0
        }
    }
    /// Cumulative vehicles that entered / left `link` since the start.
    pub fn entered(&self, link: LinkID) -> u64 {
        self.entered[link]
    }
    pub fn exited(&self, link: LinkID) -> u64 {
        self.exited[link]
    }
    /// The scenario (network, parameters) for anything else read-only.
    pub fn scenario(&self) -> &Scenario {
        self.scenario
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn group(id: &str) -> SignalGroup {
        SignalGroup { id: id.into(), node: 0, movements: vec![], vehicles_per_green: None, off: OffState::Green, sneakers: 0, clearance: 0.0 }
    }

    #[test]
    fn fixed_time_plan_expands_to_alternating_switches() {
        let net = SignalNetwork {
            groups: vec![group("a"), group("b")],
            plans: vec![FixedTimePlan {
                name: None,
                from: 0.0,
                until: 120.0,
                cycle: 60.0,
                offset: 10.0,
                groups: vec![],
                greens: [("a".to_string(), vec![[0.0, 20.0]]), ("b".to_string(), vec![[25.0, 55.0]])]
                    .into_iter()
                    .collect(),
            }],
            tolls: vec![],
        };
        let sw = expand(&net, 0.0, 200.0);
        // a: cycles start at 10, 70 (and −50 → its window ends at −30).
        use Aspect::{Green, Off, Red};
        assert_eq!(sw[0], vec![(0.0, Red), (10.0, Green), (30.0, Red), (70.0, Green), (90.0, Red), (120.0, Off)]);
        // b: the cycle starting at −50 is green over [0, 5); then [35, 65),
        // and [95, 120) runs into the off state (green) after the plan.
        // The plan green [95, 120) and the dark signal after it stay apart, so
        // the per-green cap is lifted at 120.
        assert_eq!(sw[1], vec![(0.0, Green), (5.0, Red), (35.0, Green), (65.0, Red), (95.0, Green), (120.0, Off)]);
    }

    #[test]
    fn windows_running_past_the_cycle_end_wrap() {
        let net = SignalNetwork {
            groups: vec![group("a")],
            plans: vec![FixedTimePlan {
                name: None,
                from: 0.0,
                until: 100.0,
                cycle: 50.0,
                offset: 0.0,
                groups: vec![],
                greens: [("a".to_string(), vec![[40.0, 60.0]])].into_iter().collect(),
            }],
            tolls: vec![],
        };
        let mut a = net.groups[0].clone();
        a.off = OffState::Red;
        let net = SignalNetwork { groups: vec![a], ..net };
        // Cycle −50 contributes [−10, 10); cycle 0: [40, 60); cycle 50: [90, 100).
        assert_eq!(
            expand(&net, 0.0, 100.0)[0],
            vec![(0.0, Aspect::Green), (10.0, Aspect::Red), (40.0, Aspect::Green), (60.0, Aspect::Red), (90.0, Aspect::Green)]
        );
    }
}
