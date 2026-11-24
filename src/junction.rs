//! Priority junctions: minor movements give way to the major ones.
//!
//! An ordinary node with a `junction` object applies priority rules between
//! its movements (incoming link → outgoing link):
//!
//! - every movement is classified from the link bearings (right, through,
//!   left, U-turn) and ranked as in the HCM two-way-stop method: rank 1 the
//!   major through and right, rank 2 the major left and the minor right,
//!   rank 3 the minor through, rank 4 the minor left (U-turns as lefts);
//! - two movements conflict when they share their exit (merge) or when
//!   their paths cross. Lanes are placed on a small circle around the node
//!   (right-hand traffic: an arm's incoming lane just counter-clockwise of
//!   the arm, its outgoing lane just clockwise) and two paths cross when
//!   their end points interleave on that circle. Movements from the same
//!   approach never conflict (they share the approach's FIFO pipes);
//! - a movement gives way to every conflicting movement of a better rank.
//!   Its head vehicle may go at t when, for each of them: `lead` has passed
//!   since its last vehicle, none of its vehicles can reach the node before
//!   t + `lag` (approaching vehicles arrive at known free-flow times;
//!   vehicles already at the line count when they can move), and when
//!   `t_f` has passed since the previous departure from the same lane and,
//!   at a stop, the vehicle has stood `stop_time` at the line. With
//!   lead = t_f/2 and lag = t_c − t_f, a gap g admits
//!   floor((g − t_0)/t_f) + 1 vehicles, t_0 = t_c − t_f/2 (Siegloch).
//!
//! Signals combine with it: a red movement holds its vehicles and does not
//! count as a conflict, so a left turn in a green phase is *permitted*
//! (it gives way to the opposing through) and *protected* when the
//! opposing through is red.

use std::collections::{BTreeMap, VecDeque};

use anyhow::{anyhow, bail, Result};
use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::model::{Link, LinkID, Node, PipeIdx};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Turn {
    Right,
    Through,
    Left,
    UTurn,
}

/// Control of the minor approaches.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Control {
    #[default]
    Yield,
    Stop,
}

/// One movement of a junction, resolved (compiled scenario form).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JunctionMovement {
    pub from: LinkID,
    pub to: LinkID,
    pub turn: Turn,
    /// 1 (major through/right) … 4 (minor left).
    pub rank: u8,
    /// t_c and t_f (s); unused for rank 1.
    #[serde(default)]
    pub critical_gap: f64,
    #[serde(default)]
    pub follow_up: f64,
    /// Time a vehicle must stand at the line before going (stop sign).
    #[serde(default)]
    pub stop_time: f64,
    /// Movements (from, to) it gives way to.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub yields_to: Vec<[LinkID; 2]>,
}

impl JunctionMovement {
    pub fn lead(&self) -> f64 {
        self.follow_up / 2.0
    }
    pub fn lag(&self) -> f64 {
        (self.critical_gap - self.follow_up).max(0.0)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JunctionSpec {
    pub control: Control,
    /// Links of the major road (incoming ones define major approaches).
    pub major: Vec<LinkID>,
    pub movements: Vec<JunctionMovement>,
}

impl JunctionSpec {
    pub fn movement(&self, from: LinkID, to: LinkID) -> Option<&JunctionMovement> {
        self.movements.iter().find(|m| m.from == from && m.to == to)
    }
}

// ── Defaults (HCM 6, two-way stop control, two-lane major road) ─────────────

/// (t_c, t_f) for a movement of `rank` and `turn`.
pub fn default_gaps(rank: u8, turn: Turn) -> (f64, f64) {
    match (rank, turn) {
        (1, _) => (0.0, 0.0),
        (2, Turn::Left | Turn::UTurn) => (4.1, 2.2), // major left
        (2, _) => (6.2, 3.3),                        // minor right
        (3, _) => (6.5, 4.0),                        // minor through
        _ => (7.1, 3.5),                             // minor left
    }
}

pub const DEFAULT_STOP_TIME: f64 = 1.0;

// ── Geometry ──────────────────────────────────────────────────────────────────

fn wrap(a: f64) -> f64 {
    let t = a.rem_euclid(std::f64::consts::TAU);
    if t > std::f64::consts::PI {
        t - std::f64::consts::TAU
    } else {
        t
    }
}

/// Turn from an approach whose link points (from the node) at `in_bearing`
/// to an exit at `out_bearing` (radians, counter-clockwise from east).
pub fn classify(in_bearing: f64, out_bearing: f64) -> Turn {
    let travel_in = in_bearing + std::f64::consts::PI;
    let d = wrap(out_bearing - travel_in).to_degrees();
    if d.abs() >= 150.0 {
        Turn::UTurn
    } else if d > 45.0 {
        Turn::Left
    } else if d < -45.0 {
        Turn::Right
    } else {
        Turn::Through
    }
}

/// Rank of a movement: major approach or not, and its turn.
pub fn rank(major_approach: bool, turn: Turn) -> u8 {
    match (major_approach, turn) {
        (true, Turn::Through | Turn::Right) => 1,
        (true, _) => 2,
        (false, Turn::Right) => 2,
        (false, Turn::Through) => 3,
        (false, _) => 4,
    }
}

/// Offset of a lane from its arm on the circle (rad).
const LANE_OFFSET: f64 = 0.02;

/// True when the strict counter-clockwise arc from `a` to `b` contains `x`.
fn on_arc(a: f64, b: f64, x: f64) -> bool {
    let span = (b - a).rem_euclid(std::f64::consts::TAU);
    let off = (x - a).rem_euclid(std::f64::consts::TAU);
    off > 0.0 && off < span
}

/// Do the paths (in1 → out1) and (in2 → out2) cross? Points are positions
/// on the circle around the node.
fn cross(in1: f64, out1: f64, in2: f64, out2: f64) -> bool {
    on_arc(in1, out1, in2) != on_arc(in1, out1, out2)
}

// ── Input (GeoJSON node property) ─────────────────────────────────────────────

fn num(v: &Value, k: &str) -> Option<f64> {
    v.get(k).and_then(Value::as_f64)
}

/// Build the compiled spec of `node` from its `junction` object: classify
/// every movement from the link bearings, rank it, find its conflicts,
/// then apply the explicit per-movement overrides.
pub fn resolve_spec(raw: &Value, node: &Node, links: &[Link]) -> Result<JunctionSpec> {
    let id = node.id;
    if !raw.is_object() {
        bail!("Junction node {id}: 'junction' must be an object");
    }
    let major: Vec<LinkID> = raw
        .get("major")
        .and_then(Value::as_array)
        .ok_or_else(|| anyhow!("Junction node {id}: give 'major', the links of the major road"))?
        .iter()
        .map(|v| v.as_u64().map(|x| x as usize).ok_or_else(|| anyhow!("Junction node {id}: 'major' must list link ids")))
        .collect::<Result<_>>()?;
    let control = match raw.get("control").and_then(Value::as_str) {
        None | Some("yield") => Control::Yield,
        Some("stop") => Control::Stop,
        Some(other) => bail!("Junction node {id}: 'control' must be \"yield\" or \"stop\" (got \"{other}\")"),
    };
    let stop_time = num(raw, "stop_time").unwrap_or(match control {
        Control::Stop => DEFAULT_STOP_TIME,
        Control::Yield => 0.0,
    });
    let link = |lid: LinkID| -> Result<&Link> {
        links.iter().find(|l| l.id == lid).ok_or_else(|| anyhow!("Junction node {id}: unknown link {lid}"))
    };
    let bearing = |lid: LinkID, incoming: bool| -> Result<f64> {
        crate::roundabout::link_bearing(link(lid)?, node, incoming)
            .map_err(|_| anyhow!("Junction node {id}: link {lid} has no geometry to classify its movements; give it a LineString"))
    };
    let ins: Vec<(LinkID, f64)> = node.incoming_links.iter().map(|&l| Ok((l, bearing(l, true)?))).collect::<Result<_>>()?;
    let outs: Vec<(LinkID, f64)> = node.outgoing_links.iter().map(|&l| Ok((l, bearing(l, false)?))).collect::<Result<_>>()?;

    // Movements with their circle end points.
    struct Draft {
        m: JunctionMovement,
        p_in: f64,
        p_out: f64,
    }
    let mut drafts = Vec::new();
    for &(a, ta) in &ins {
        for &(b, tb) in &outs {
            let turn = classify(ta, tb);
            let r = rank(major.contains(&a), turn);
            let (critical_gap, follow_up) = default_gaps(r, turn);
            let stop = if r > 1 && !major.contains(&a) { stop_time } else { 0.0 };
            drafts.push(Draft {
                m: JunctionMovement { from: a, to: b, turn, rank: r, critical_gap, follow_up, stop_time: stop, yields_to: vec![] },
                p_in: ta + LANE_OFFSET,
                p_out: tb - LANE_OFFSET,
            });
        }
    }
    for i in 0..drafts.len() {
        let mut yields = Vec::new();
        for (j, n) in drafts.iter().enumerate() {
            let m = &drafts[i];
            if i == j || n.m.rank >= m.m.rank || n.m.from == m.m.from {
                continue;
            }
            if n.m.to == m.m.to || cross(m.p_in, m.p_out, n.p_in, n.p_out) {
                yields.push([n.m.from, n.m.to]);
            }
        }
        drafts[i].m.yields_to = yields;
    }
    let mut movements: Vec<JunctionMovement> = drafts.into_iter().map(|d| d.m).collect();

    // Explicit overrides.
    if let Some(list) = raw.get("movements") {
        let list = list.as_array().ok_or_else(|| anyhow!("Junction node {id}: 'movements' must be a list"))?;
        for o in list {
            let from = o.get("from").and_then(Value::as_u64).map(|v| v as usize);
            let to = o.get("to").and_then(Value::as_u64).map(|v| v as usize);
            let (Some(from), Some(to)) = (from, to) else {
                bail!("Junction node {id}: a movement override needs 'from' and 'to'");
            };
            let m = movements
                .iter_mut()
                .find(|m| m.from == from && m.to == to)
                .ok_or_else(|| anyhow!("Junction node {id}: no movement {from} → {to}"))?;
            if let Some(r) = o.get("rank").and_then(Value::as_u64) {
                m.rank = r as u8;
            }
            if let Some(v) = num(o, "critical_gap") {
                m.critical_gap = v;
            }
            if let Some(v) = num(o, "follow_up") {
                m.follow_up = v;
            }
            if let Some(v) = num(o, "stop_time") {
                m.stop_time = v;
            }
            if let Some(y) = o.get("yields_to") {
                let y = y.as_array().ok_or_else(|| anyhow!("Junction node {id}: 'yields_to' must be a list of [from, to]"))?;
                m.yields_to = y
                    .iter()
                    .map(|p| {
                        let a = p.get(0).and_then(Value::as_u64);
                        let b = p.get(1).and_then(Value::as_u64);
                        match (a, b) {
                            (Some(a), Some(b)) => Ok([a as usize, b as usize]),
                            _ => Err(anyhow!("Junction node {id}: 'yields_to' entries are [from, to]")),
                        }
                    })
                    .collect::<Result<_>>()?;
            }
        }
    }
    Ok(JunctionSpec { control, major, movements })
}

/// Structural checks of a compiled spec.
pub fn validate_spec(spec: &JunctionSpec, node: &Node) -> Result<()> {
    let id = node.id;
    if spec.major.is_empty() {
        bail!("Junction node {id}: 'major' is empty");
    }
    for l in &spec.major {
        if !node.incoming_links.contains(l) && !node.outgoing_links.contains(l) {
            bail!("Junction node {id}: major link {l} does not touch the node");
        }
    }
    for m in &spec.movements {
        if !node.incoming_links.contains(&m.from) || !node.outgoing_links.contains(&m.to) {
            bail!("Junction node {id}: movement {} → {} does not go through the node", m.from, m.to);
        }
        for (name, v) in [("critical_gap", m.critical_gap), ("follow_up", m.follow_up), ("stop_time", m.stop_time)] {
            if !(v.is_finite() && v >= 0.0) {
                bail!("Junction node {id}: movement {} → {}: '{name}' must be ≥ 0", m.from, m.to);
            }
        }
        if !m.yields_to.is_empty() && m.critical_gap < m.follow_up {
            bail!("Junction node {id}: movement {} → {}: critical_gap must be ≥ follow_up", m.from, m.to);
        }
        for &[a, b] in &m.yields_to {
            if spec.movement(a, b).is_none() {
                bail!("Junction node {id}: movement {} → {} gives way to unknown movement {a} → {b}", m.from, m.to);
            }
            if a == m.from {
                bail!("Junction node {id}: movement {} → {} cannot give way to a movement of its own approach", m.from, m.to);
            }
        }
    }
    Ok(())
}

// ── Runtime ───────────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct MoveRt {
    pub from: LinkID,
    pub to: LinkID,
    pub rank: u8,
    pub lead: f64,
    pub lag: f64,
    pub follow_up: f64,
    pub stop_time: f64,
    /// Indices of the movements it gives way to.
    pub yields: Vec<usize>,
}

/// Run-time state of one junction.
#[derive(Debug, Clone)]
pub struct JunctionState {
    pub node: usize,
    pub moves: Vec<MoveRt>,
    index: BTreeMap<(LinkID, LinkID), usize>,
    /// Per movement: time of its last vehicle at the node.
    pub last_pass: Vec<f64>,
    /// Per incoming link: vehicles on their way to the node, (arrival time
    /// at the node, next link), in arrival order.
    pub upcoming: BTreeMap<LinkID, VecDeque<(f64, Option<LinkID>)>>,
    /// Per (incoming link, pipe): time of the last departure.
    pub last_departure: BTreeMap<(LinkID, PipeIdx), f64>,
}

impl JunctionState {
    pub fn new(node: &Node, spec: &JunctionSpec) -> Self {
        let index: BTreeMap<(LinkID, LinkID), usize> =
            spec.movements.iter().enumerate().map(|(i, m)| ((m.from, m.to), i)).collect();
        let moves = spec
            .movements
            .iter()
            .map(|m| MoveRt {
                from: m.from,
                to: m.to,
                rank: m.rank,
                lead: m.lead(),
                lag: m.lag(),
                follow_up: m.follow_up,
                stop_time: m.stop_time,
                yields: m.yields_to.iter().filter_map(|&[a, b]| index.get(&(a, b)).copied()).collect(),
            })
            .collect::<Vec<_>>();
        let n = moves.len();
        JunctionState {
            node: node.id,
            moves,
            index,
            last_pass: vec![f64::NEG_INFINITY; n],
            upcoming: node.incoming_links.iter().map(|&l| (l, VecDeque::new())).collect(),
            last_departure: BTreeMap::new(),
        }
    }

    pub fn movement(&self, from: LinkID, to: Option<LinkID>) -> Option<usize> {
        to.and_then(|t| self.index.get(&(from, t)).copied())
    }

    /// Rank of a head's movement (a trip ending here, or an unknown
    /// movement, goes as rank 1: it conflicts with nobody).
    pub fn rank_of(&self, from: LinkID, to: Option<LinkID>) -> u8 {
        self.movement(from, to).map_or(1, |m| self.moves[m].rank)
    }

    pub fn expect(&mut self, link: LinkID, arrival: f64, next: Option<LinkID>) {
        if let Some(q) = self.upcoming.get_mut(&link) {
            q.push_back((arrival, next));
        }
    }

    /// A vehicle bound for `next` reached the node by `link`: drop its
    /// record (the earliest one, matching `next` among simultaneous ones).
    pub fn arrived(&mut self, link: LinkID, next: Option<LinkID>) {
        if let Some(q) = self.upcoming.get_mut(&link) {
            let Some(&(t0, _)) = q.front() else { return };
            let i = q.iter().take_while(|e| e.0 == t0).position(|e| e.1 == next).unwrap_or(0);
            q.remove(i);
        }
    }

    pub fn passed(&mut self, movement: Option<usize>, link: LinkID, pipe: PipeIdx, now: f64) {
        if let Some(m) = movement {
            self.last_pass[m] = now;
        }
        self.last_departure.insert((link, pipe), now);
    }

    /// Earliest time the head of (link, pipe), on movement `m`, which
    /// reached the line at `arrival`, may go — from the cheap terms only:
    /// lead after each conflicting passage, follow-up after the previous
    /// departure of its lane, stop time at the line. The lead holds even
    /// behind a movement that has just turned red: its last vehicle is
    /// physically through the node.
    pub fn cheap_bound(&self, m: usize, link: LinkID, pipe: PipeIdx, arrival: f64, now: f64) -> f64 {
        let mv = &self.moves[m];
        let mut t = now;
        let last = self.last_departure.get(&(link, pipe)).copied().unwrap_or(f64::NEG_INFINITY);
        if !mv.yields.is_empty() {
            t = t.max(last + mv.follow_up);
        }
        t = t.max(arrival.max(last) + mv.stop_time);
        for &n in &mv.yields {
            t = t.max(self.last_pass[n] + mv.lead);
        }
        t
    }

    /// Does a vehicle approaching on a movement that `m` gives way to reach
    /// the node before t + lag?
    pub fn approaching_conflict(&self, m: usize, t: f64, red: &dyn Fn(usize) -> bool) -> bool {
        let mv = &self.moves[m];
        let horizon = t + mv.lag;
        mv.yields.iter().any(|&n| {
            if red(n) {
                return false;
            }
            let nm = &self.moves[n];
            self.upcoming.get(&nm.from).is_some_and(|q| {
                q.iter().take_while(|&&(arr, _)| arr < horizon).any(|&(_, next)| next == Some(nm.to))
            })
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    #[test]
    fn classifies_turns_from_bearings() {
        // Approach from the south (its link points south from the node).
        let south = -FRAC_PI_2;
        assert_eq!(classify(south, FRAC_PI_2), Turn::Through); // leaves north
        assert_eq!(classify(south, 0.0), Turn::Right); // leaves east
        assert_eq!(classify(south, PI), Turn::Left); // leaves west
        assert_eq!(classify(south, south), Turn::UTurn);
    }

    /// Four-leg crossing, major road east-west: arms E (0), N (π/2),
    /// W (π), S (−π/2). Movement (a, b) = from arm a to arm b.
    fn four_leg() -> Vec<(usize, usize, Turn, u8, f64, f64)> {
        let arms = [0.0, FRAC_PI_2, PI, -FRAC_PI_2];
        let major = [true, false, true, false];
        let mut v = Vec::new();
        for a in 0..4 {
            for b in 0..4 {
                if a == b {
                    continue;
                }
                let t = classify(arms[a], arms[b]);
                v.push((a, b, t, rank(major[a], t), arms[a] + LANE_OFFSET, arms[b] - LANE_OFFSET));
            }
        }
        v
    }

    fn yields(a: usize, b: usize) -> Vec<(usize, usize)> {
        let v = four_leg();
        let m = v.iter().find(|x| x.0 == a && x.1 == b).unwrap();
        let mut out: Vec<(usize, usize)> = v
            .iter()
            .filter(|n| n.3 < m.3 && n.0 != m.0 && (n.1 == m.1 || cross(m.4, m.5, n.4, n.5)))
            .map(|n| (n.0, n.1))
            .collect();
        out.sort();
        out
    }

    #[test]
    fn conflict_matrix_matches_the_hcm_two_way_stop() {
        let (e, n, w, s) = (0, 1, 2, 3);
        // Ranks.
        let v = four_leg();
        let r = |a, b| v.iter().find(|x| x.0 == a && x.1 == b).unwrap().3;
        assert_eq!((r(w, e), r(w, s), r(w, n)), (1, 1, 2)); // eastbound through, right, left
        assert_eq!((r(s, e), r(s, n), r(s, w)), (2, 3, 4)); // northbound right, through, left
        // Major left (eastbound, W → N) gives way to the opposing through
        // (E → W) and the opposing right (E → N).
        assert_eq!(yields(w, n), vec![(e, n), (e, w)]);
        // Minor right (S → E) gives way to the eastbound through only.
        assert_eq!(yields(s, e), vec![(w, e)]);
        // Minor through (S → N) gives way to both major throughs, both major
        // lefts, and the westbound right (E → N, same exit).
        assert_eq!(yields(s, n), vec![(e, n), (e, w), (e, s), (w, e), (w, n)]);
        // Minor left (S → W) gives way to every major movement crossing it,
        // the opposing minor through and right.
        let l = yields(s, w);
        for must in [(e, w), (w, e), (w, n), (e, s), (n, s), (n, w)] {
            assert!(l.contains(&must), "minor left must give way to {must:?}: {l:?}");
        }
    }
}
