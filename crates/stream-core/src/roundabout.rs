//! Roundabout node: the whole ring is solved inside one node.
//!
//! The ring is one FIFO kinematic-wave stream that closes on itself (a pipe
//! of `lanes` lanes, no overtaking). It is resolved only at the points where
//! something happens: for every arm i, the exit point X_i, then (one splitter
//! island further) the entry point M_i. A vehicle from arm a to arm b passes
//! M_a, X_{a+1}, M_{a+1}, …, X_b.
//!
//! To pass a point P a vehicle needs, besides FIFO order at P:
//! - free flow from its previous point (scheduled as a `RingArrival`);
//! - the ring capacity: one passage per 1/C at P;
//! - Newell's storage constraint against *every* downstream point Q up to
//!   its exit, not only the next one. With d = arc P→Q, K = ceil(kx·lanes·d)
//!   and c the vehicles ahead of it, between P and Q, that will pass Q: if
//!   c ≥ K it waits for a passage at Q, otherwise it may pass at
//!   T_Q[N_Q − (K − c)] + d/w + (K − kx·lanes·d)/C. This is the pipe rule
//!   (`storage_availability` + `wave_delay`) with sources and sinks, and the
//!   bound taken over all Q keeps the result independent of how many points
//!   the ring has;
//! - at its exit point, a place on the exit link (the ordinary transfer
//!   rule). A vehicle that cannot leave holds its point and everybody behind
//!   it: the queue spills back around the ring.
//!
//! An approach vehicle enters at M_i when no ring vehicle waits at M_i, when
//! `lead` has passed since the last circulating vehicle, when no circulating
//! vehicle in motion can reach M_i within `lag`, when `t_f` has passed since
//! the last entry from its lane (pipe), and when the storage constraint
//! holds at M_i. With lead + lag = t_0 = t_c − t_f/2, a gap g admits
//! exactly floor((g − t_0)/t_f) + 1 vehicles (Siegloch) when t_f ≥ 1/C. The
//! nominal split is lead = t_f/2, lag = t_c − t_f; lead never drops below
//! the ring headway 1/C (see `RoundaboutSpec::lead`).

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use anyhow::{anyhow, bail, Result};
use serde_json::Value;

use crate::model::{
    FundamentalDiagram, Link, LinkID, Node, NodeID, PipeIdx, RoundaboutArm, RoundaboutSpec, VehID,
};

// ── Defaults (HCM 6, single-lane entry facing one circulating lane) ─────────

pub const DEFAULT_CRITICAL_GAP: f64 = 4.976;
pub const DEFAULT_FOLLOW_UP: f64 = 2.609;
pub const DEFAULT_SPEED: f64 = 25.0 / 3.0; // 30 km/h
pub const DEFAULT_W: f64 = 5.0;
pub const DEFAULT_KX: f64 = 0.15;
pub const DEFAULT_ISLAND: f64 = 6.0;

/// Largest number of arms: two ring points per arm must fit the u8 point
/// index carried by ring events.
pub const MAX_ARMS: usize = 127;

// ── GeoJSON input ─────────────────────────────────────────────────────────────

fn num(obj: &Value, key: &str) -> Option<f64> {
    obj.get(key).and_then(Value::as_f64)
}

/// Build the compiled `RoundaboutSpec` of `node` from the `roundabout`
/// object of its GeoJSON properties. Arms not given explicitly are inferred
/// from the bearing of each link around the node point; arms given without
/// a position get the position of their links.
pub fn resolve_spec(raw: &Value, node: &Node, links: &[Link]) -> Result<RoundaboutSpec> {
    let id = node.id;
    if !raw.is_object() {
        bail!("Roundabout node {id}: 'roundabout' must be an object");
    }
    let circumference = match (num(raw, "circumference"), num(raw, "diameter")) {
        (Some(c), _) => c,
        (None, Some(d)) => std::f64::consts::PI * d,
        (None, None) => bail!("Roundabout node {id}: give 'diameter' or 'circumference'"),
    };
    let lanes_u = raw.get("lanes").and_then(Value::as_u64).unwrap_or(1);
    if !(1..=255).contains(&lanes_u) {
        bail!("Roundabout node {id}: 'lanes' must be in 1..=255 (got {lanes_u})");
    }
    let speed = num(raw, "speed").unwrap_or(DEFAULT_SPEED);
    let u = num(raw, "fd_u").unwrap_or(speed);
    let w = num(raw, "fd_w").unwrap_or(DEFAULT_W);
    let kx = num(raw, "fd_kx").unwrap_or(DEFAULT_KX);
    let c = num(raw, "fd_c").unwrap_or(if u + w > 0.0 { kx * u * w / (u + w) } else { 0.0 });
    let ccw = match raw.get("direction").and_then(Value::as_str) {
        None | Some("ccw") => true,
        Some("cw") => false,
        Some(other) => bail!("Roundabout node {id}: 'direction' must be \"ccw\" or \"cw\" (got \"{other}\")"),
    };
    let radius = circumference / std::f64::consts::TAU;
    let position_of = |angle: f64| -> f64 {
        let theta = if ccw { angle } else { -angle };
        (theta.rem_euclid(std::f64::consts::TAU)) * radius
    };

    let link = |lid: LinkID| -> Result<&Link> {
        links
            .iter()
            .find(|l| l.id == lid)
            .ok_or_else(|| anyhow!("Roundabout node {id}: unknown link {lid}"))
    };

    let arms: Vec<RoundaboutArm> = match raw.get("arms") {
        Some(list) => {
            let list = list
                .as_array()
                .ok_or_else(|| anyhow!("Roundabout node {id}: 'arms' must be a list"))?;
            let mut arms = Vec::with_capacity(list.len());
            for a in list {
                let in_link = a.get("in").and_then(Value::as_u64).map(|v| v as usize);
                let out_link = a.get("out").and_then(Value::as_u64).map(|v| v as usize);
                let position = match num(a, "position") {
                    Some(p) => p.rem_euclid(circumference),
                    None => {
                        let mut angles = Vec::new();
                        if let Some(l) = in_link {
                            angles.push(link_bearing(link(l)?, node, true)?);
                        }
                        if let Some(l) = out_link {
                            angles.push(link_bearing(link(l)?, node, false)?);
                        }
                        if angles.is_empty() {
                            bail!("Roundabout node {id}: an arm without links needs a 'position'");
                        }
                        position_of(circular_mean(&angles))
                    }
                };
                arms.push(RoundaboutArm { in_link, out_link, position });
            }
            arms
        }
        None => infer_arms(node, links)?
            .into_iter()
            .map(|(in_link, out_link, angle)| RoundaboutArm {
                in_link,
                out_link,
                position: position_of(angle),
            })
            .collect(),
    };
    let mut arms = arms;
    arms.sort_by(|a, b| a.position.total_cmp(&b.position));

    Ok(RoundaboutSpec {
        circumference,
        lanes: lanes_u as u8,
        speed,
        fd: FundamentalDiagram { u, w, kx, c },
        critical_gap: num(raw, "critical_gap").unwrap_or(DEFAULT_CRITICAL_GAP),
        follow_up: num(raw, "follow_up").unwrap_or(DEFAULT_FOLLOW_UP),
        island: num(raw, "island").unwrap_or(DEFAULT_ISLAND),
        arms,
    })
}

/// Bearing (rad, counter-clockwise from east) of a link seen from the node:
/// toward the link's point next to the node. Longitude differences are
/// scaled by cos(latitude) when the coordinates look geographic; this local
/// linear map keeps the angular order of the arms either way.
pub fn link_bearing(link: &Link, node: &Node, incoming: bool) -> Result<f64> {
    let (x0, y0) = node.points;
    let toward = if incoming {
        // Last point before the node.
        link.points.iter().rev().find(|p| **p != node.points).copied()
    } else {
        link.points.iter().find(|p| **p != node.points).copied()
    };
    let (x1, y1) = toward.ok_or_else(|| {
        anyhow!(
            "Roundabout node {}: link {} has no geometry to place it around the node; give it a LineString or list the arms with their 'position'",
            node.id,
            link.id
        )
    })?;
    let geographic = x0.abs() <= 180.0 && y0.abs() <= 90.0;
    let scale = if geographic { y0.to_radians().cos().max(1e-6) } else { 1.0 };
    Ok((y1 - y0).atan2((x1 - x0) * scale))
}

fn circular_mean(angles: &[f64]) -> f64 {
    let (s, c) = angles
        .iter()
        .fold((0.0, 0.0), |(s, c), a| (s + a.sin(), c + a.cos()));
    s.atan2(c)
}

fn angle_gap(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(std::f64::consts::TAU);
    d.min(std::f64::consts::TAU - d)
}

/// Pair the node's incoming and outgoing links into arms by bearing: each
/// entry takes the closest free exit within 45°; the rest are one-way arms.
/// Returns (in, out, bearing), unsorted.
pub fn infer_arms(node: &Node, links: &[Link]) -> Result<Vec<(Option<LinkID>, Option<LinkID>, f64)>> {
    let find = |lid: LinkID| links.iter().find(|l| l.id == lid);
    let mut ins: Vec<(LinkID, f64)> = Vec::new();
    for &lid in &node.incoming_links {
        let l = find(lid).ok_or_else(|| anyhow!("Roundabout node {}: unknown link {lid}", node.id))?;
        ins.push((lid, link_bearing(l, node, true)?));
    }
    let mut outs: Vec<(LinkID, f64)> = Vec::new();
    for &lid in &node.outgoing_links {
        let l = find(lid).ok_or_else(|| anyhow!("Roundabout node {}: unknown link {lid}", node.id))?;
        outs.push((lid, link_bearing(l, node, false)?));
    }
    // Greedy by smallest gap, deterministic on ties (link ids).
    let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
    for (i, &(_, a)) in ins.iter().enumerate() {
        for (j, &(_, b)) in outs.iter().enumerate() {
            let g = angle_gap(a, b);
            if g <= std::f64::consts::FRAC_PI_4 {
                pairs.push((g, i, j));
            }
        }
    }
    pairs.sort_by(|x, y| x.0.total_cmp(&y.0).then((x.1, x.2).cmp(&(y.1, y.2))));
    let mut in_used = vec![false; ins.len()];
    let mut out_used = vec![false; outs.len()];
    let mut arms = Vec::new();
    for (_, i, j) in pairs {
        if in_used[i] || out_used[j] {
            continue;
        }
        in_used[i] = true;
        out_used[j] = true;
        arms.push((Some(ins[i].0), Some(outs[j].0), circular_mean(&[ins[i].1, outs[j].1])));
    }
    for (i, &(lid, a)) in ins.iter().enumerate() {
        if !in_used[i] {
            arms.push((Some(lid), None, a));
        }
    }
    for (j, &(lid, b)) in outs.iter().enumerate() {
        if !out_used[j] {
            arms.push((None, Some(lid), b));
        }
    }
    Ok(arms)
}

/// Structural checks on a compiled spec; `node` gives the links it must cover.
pub fn validate_spec(spec: &RoundaboutSpec, node: &Node) -> Result<()> {
    let id = node.id;
    let positive = [
        ("circumference", spec.circumference),
        ("speed", spec.speed),
        ("fd_w", spec.fd.w),
        ("fd_c", spec.fd.c),
        ("follow_up", spec.follow_up),
    ];
    for (name, v) in positive {
        if !(v.is_finite() && v > 0.0) {
            bail!("Roundabout node {id}: '{name}' must be > 0 (got {v})");
        }
    }
    if !(spec.island.is_finite() && spec.island >= 0.0) {
        bail!("Roundabout node {id}: 'island' must be ≥ 0 (got {})", spec.island);
    }
    if spec.critical_gap < spec.follow_up {
        bail!(
            "Roundabout node {id}: 'critical_gap' ({}) must be ≥ 'follow_up' ({})",
            spec.critical_gap,
            spec.follow_up
        );
    }
    if spec.arms.is_empty() || spec.arms.len() > MAX_ARMS {
        bail!("Roundabout node {id}: needs 1..={MAX_ARMS} arms (got {})", spec.arms.len());
    }
    // Arm spacing: each arm's exit and entry points must stay in order.
    let n = spec.arms.len();
    for k in 0..n {
        let a = spec.arms[k].position;
        if !(0.0..spec.circumference).contains(&a) {
            bail!("Roundabout node {id}: arm position {a} outside [0, circumference)");
        }
        let b = if k + 1 < n { spec.arms[k + 1].position } else { spec.arms[0].position + spec.circumference };
        if n > 1 && b - a <= spec.island {
            bail!(
                "Roundabout node {id}: arms at {a:.1} m and {:.1} m are closer than the island ({} m)",
                b.rem_euclid(spec.circumference),
                spec.island
            );
        }
    }
    if n == 1 && spec.island >= spec.circumference {
        bail!("Roundabout node {id}: island longer than the ring");
    }
    let mut seen_in = BTreeSet::new();
    let mut seen_out = BTreeSet::new();
    for arm in &spec.arms {
        if let Some(l) = arm.in_link {
            if !node.incoming_links.contains(&l) {
                bail!("Roundabout node {id}: arm entry link {l} does not end at this node");
            }
            if !seen_in.insert(l) {
                bail!("Roundabout node {id}: link {l} used by two arms");
            }
        }
        if let Some(l) = arm.out_link {
            if !node.outgoing_links.contains(&l) {
                bail!("Roundabout node {id}: arm exit link {l} does not start at this node");
            }
            if !seen_out.insert(l) {
                bail!("Roundabout node {id}: link {l} used by two arms");
            }
        }
    }
    for l in &node.incoming_links {
        if !seen_in.contains(l) {
            bail!("Roundabout node {id}: incoming link {l} belongs to no arm");
        }
    }
    for l in &node.outgoing_links {
        if !seen_out.contains(l) {
            bail!("Roundabout node {id}: outgoing link {l} belongs to no arm");
        }
    }
    if seen_in.is_empty() || seen_out.is_empty() {
        bail!("Roundabout node {id}: needs at least one entry and one exit");
    }
    Ok(())
}

// ── Ring geometry ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointKind {
    Exit,
    Merge,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RingPoint {
    /// Position along the ring, in [0, circumference).
    pub pos: f64,
    pub kind: PointKind,
    pub arm: usize,
}

#[derive(Debug, Clone)]
pub struct RingGeometry {
    pub circumference: f64,
    /// Points in the direction of circulation.
    pub points: Vec<RingPoint>,
    pub exit_point: Vec<usize>,
    pub merge_point: Vec<usize>,
}

impl RingGeometry {
    pub fn new(spec: &RoundaboutSpec) -> Self {
        let c = spec.circumference;
        let mut points = Vec::with_capacity(2 * spec.arms.len());
        for (k, arm) in spec.arms.iter().enumerate() {
            let x = (arm.position - spec.island / 2.0).rem_euclid(c);
            let m = (arm.position + spec.island / 2.0).rem_euclid(c);
            points.push(RingPoint { pos: x, kind: PointKind::Exit, arm: k });
            points.push(RingPoint { pos: m, kind: PointKind::Merge, arm: k });
        }
        // Order of circulation, starting from the first arm's exit point so
        // that X_k precedes M_k even when an arm straddles position 0.
        let origin = points[0].pos;
        points.sort_by(|a, b| {
            let ka = (a.pos - origin).rem_euclid(c);
            let kb = (b.pos - origin).rem_euclid(c);
            ka.total_cmp(&kb).then((a.arm, a.kind as u8).cmp(&(b.arm, b.kind as u8)))
        });
        let n_arms = spec.arms.len();
        let mut exit_point = vec![0; n_arms];
        let mut merge_point = vec![0; n_arms];
        for (i, p) in points.iter().enumerate() {
            match p.kind {
                PointKind::Exit => exit_point[p.arm] = i,
                PointKind::Merge => merge_point[p.arm] = i,
            }
        }
        RingGeometry { circumference: c, points, exit_point, merge_point }
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn next(&self, p: usize) -> usize {
        (p + 1) % self.points.len()
    }

    /// Ring distance from point `from` to point `to` in the direction of
    /// circulation, in (0, circumference]; a full lap when `from == to`.
    pub fn arc(&self, from: usize, to: usize) -> f64 {
        let d = (self.points[to].pos - self.points[from].pos).rem_euclid(self.circumference);
        if d <= 0.0 {
            self.circumference
        } else {
            d
        }
    }

    /// Index steps from `from` to `to` going forward, in 1..=len (a full lap
    /// when equal).
    pub fn steps(&self, from: usize, to: usize) -> usize {
        let n = self.points.len();
        let s = (to + n - from) % n;
        if s == 0 {
            n
        } else {
            s
        }
    }

    /// True when `q` lies strictly after `from` and at or before `to` along
    /// the ring (`from == to` spans a full lap).
    pub fn within(&self, from: usize, q: usize, to: usize) -> bool {
        let sq = self.steps(from, q);
        sq <= self.steps(from, to) && q != from
    }

    /// Path length from entry point `merge` to exit point `exit`.
    pub fn path_length(&self, merge: usize, exit: usize) -> f64 {
        self.arc(merge, exit)
    }
}

// ── Runtime state ─────────────────────────────────────────────────────────────

/// A vehicle on the ring.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RingVehicle {
    /// Last ring point passed and when.
    pub last_pt: usize,
    pub last_time: f64,
    /// Point where it leaves the ring.
    pub exit_pt: usize,
    /// True between a passage and the arrival at the next point; false while
    /// it waits at a point.
    pub in_transit: bool,
}

/// One recorded passage at a ring point (kept for verification and tests).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Passage {
    pub time: f64,
    pub veh: VehID,
    /// True for a vehicle already circulating; false for one entering here.
    pub circulating: bool,
}

/// Outcome of evaluating whether a vehicle may pass a ring point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Bound {
    /// May pass at this time (≤ now ⇒ right away).
    At(f64),
    /// Must wait for some other ring event (a passage, an arrival).
    Blocked,
}

impl Bound {
    fn max(self, t: f64) -> Bound {
        match self {
            Bound::At(a) => Bound::At(a.max(t)),
            Bound::Blocked => Bound::Blocked,
        }
    }
    pub fn and(self, other: Bound) -> Bound {
        match other {
            Bound::At(t) => self.max(t),
            Bound::Blocked => Bound::Blocked,
        }
    }
}

/// The vehicles on a ring. A handful at most (bounded by the ring storage),
/// so a flat vector beats any map; order is irrelevant to every rule (they
/// only count or test for existence).
#[derive(Debug, Clone, Default)]
pub struct RingSet {
    items: Vec<(VehID, RingVehicle)>,
}

impl RingSet {
    pub fn get(&self, veh: &VehID) -> Option<&RingVehicle> {
        self.items.iter().find(|(v, _)| v == veh).map(|(_, r)| r)
    }
    pub fn get_mut(&mut self, veh: &VehID) -> Option<&mut RingVehicle> {
        self.items.iter_mut().find(|(v, _)| v == veh).map(|(_, r)| r)
    }
    pub fn insert(&mut self, veh: VehID, rv: RingVehicle) {
        match self.get_mut(&veh) {
            Some(r) => *r = rv,
            None => self.items.push((veh, rv)),
        }
    }
    pub fn remove(&mut self, veh: &VehID) -> Option<RingVehicle> {
        let i = self.items.iter().position(|(v, _)| v == veh)?;
        Some(self.items.swap_remove(i).1)
    }
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
    pub fn len(&self) -> usize {
        self.items.len()
    }
    pub fn iter(&self) -> impl Iterator<Item = (&VehID, &RingVehicle)> {
        self.items.iter().map(|(v, r)| (v, r))
    }
    pub fn values(&self) -> impl Iterator<Item = &RingVehicle> {
        self.items.iter().map(|(_, r)| r)
    }
}

/// A map over a handful of keys (one per approach lane): a flat vector with
/// linear search, no allocation once warm.
#[derive(Debug, Clone)]
pub struct SmallMap<K, V> {
    items: Vec<(K, V)>,
}

impl<K, V> Default for SmallMap<K, V> {
    fn default() -> Self {
        SmallMap { items: Vec::new() }
    }
}

impl<K: PartialEq, V> SmallMap<K, V> {
    pub fn get(&self, key: &K) -> Option<&V> {
        self.items.iter().find(|(k, _)| k == key).map(|(_, v)| v)
    }
    pub fn insert(&mut self, key: K, value: V) {
        match self.items.iter_mut().find(|(k, _)| *k == key) {
            Some((_, v)) => *v = value,
            None => self.items.push((key, value)),
        }
    }
}

/// Constants of one ordered pair of ring points (p, q), computed once.
#[derive(Debug, Clone, Copy, Default)]
struct PairConst {
    /// Index steps from p to q going forward, in 1..=n.
    steps: usize,
    /// Storage of the arc p → q in whole vehicles, K = ceil(kx·lanes·d) ≥ 1.
    k: usize,
    /// d / w.
    wave: f64,
    /// (K − kx·lanes·d) / C, the fractional-slot correction.
    frac: f64,
    /// d / u_ring.
    travel: f64,
}

#[derive(Debug, Clone)]
pub struct RoundaboutState {
    pub node: NodeID,
    pub spec: RoundaboutSpec,
    pub geom: RingGeometry,
    /// kx · lanes (veh/m).
    kxl: f64,
    /// 1/C, lead and lag, frozen.
    headway: f64,
    lead: f64,
    lag: f64,
    /// Row-major n × n table of point-pair constants.
    pairs: Vec<PairConst>,
    pub arm_of_in: BTreeMap<LinkID, usize>,
    pub arm_of_out: BTreeMap<LinkID, usize>,
    /// Per point: ring vehicles arrived and waiting to pass, FIFO.
    pub hold: Vec<VecDeque<VehID>>,
    /// Per point: the most recent passage times (bounded by the storage).
    log: Vec<VecDeque<f64>>,
    log_cap: usize,
    /// Per point: passages ever.
    pub passed: Vec<usize>,
    pub last_pass: Vec<f64>,
    pub last_circ_pass: Vec<f64>,
    /// Last entry time per approach (link, pipe), for the follow-up time.
    pub last_entry: SmallMap<(LinkID, PipeIdx), f64>,
    pub on_ring: RingSet,
    /// Per point: pending `RingReady` wake-up times (bit patterns).
    pending_ready: Vec<Vec<u64>>,
    /// Per point: every passage, in order.
    pub trace: Vec<Vec<Passage>>,
    /// Bumped on every change of the ring state (a passage, an arrival at a
    /// point). A bound computed at one version holds until the next.
    pub version: u64,
    /// Last waiting bound computed for the head of each ring point and of
    /// each approach lane: (version, head vehicle, bound).
    ring_cache: Vec<Option<(u64, VehID, Bound)>>,
    entry_cache: SmallMap<(LinkID, PipeIdx), (u64, VehID, Bound)>,
    /// Scratch for `storage_bound` (difference array over the steps).
    scratch: std::cell::RefCell<Vec<i32>>,
}

/// True when a bound cached for `veh` at the current `version` still holds
/// at `now`. A bound only depends on the ring state (which bumps the
/// version) and on `now` through max(now, ·): "blocked" stays blocked and
/// "not before t > now" stays t (its wake-up is already scheduled) until the
/// ring changes.
fn still_waits(cached: Option<&(u64, VehID, Bound)>, version: u64, veh: VehID, now: f64) -> bool {
    match cached {
        Some(&(v, h, b)) if v == version && h == veh => match b {
            Bound::Blocked => true,
            Bound::At(t) => t > now,
        },
        _ => false,
    }
}

impl RoundaboutState {
    pub fn new(node: NodeID, spec: &RoundaboutSpec) -> Self {
        let geom = RingGeometry::new(spec);
        let n = geom.len();
        let kxl = spec.fd.kx.max(0.0) * f64::from(spec.lanes);
        let log_cap = (kxl * spec.circumference).ceil() as usize + 2;
        let cap = spec.capacity();
        // Same arithmetic, in the same order, as the bound evaluated on the
        // fly: the results are bit-identical.
        let mut pairs = vec![PairConst::default(); n * n];
        for p in 0..n {
            for q in 0..n {
                let d = geom.arc(p, q);
                let dn = kxl * d;
                let k = (dn.ceil() as usize).max(1);
                pairs[p * n + q] = PairConst {
                    steps: geom.steps(p, q),
                    k,
                    wave: d / spec.fd.w,
                    frac: (k as f64 - dn) / cap,
                    travel: d / spec.speed,
                };
            }
        }
        let mut arm_of_in = BTreeMap::new();
        let mut arm_of_out = BTreeMap::new();
        for (k, arm) in spec.arms.iter().enumerate() {
            if let Some(l) = arm.in_link {
                arm_of_in.insert(l, k);
            }
            if let Some(l) = arm.out_link {
                arm_of_out.insert(l, k);
            }
        }
        RoundaboutState {
            node,
            spec: spec.clone(),
            geom,
            kxl,
            headway: 1.0 / cap,
            lead: spec.lead(),
            lag: spec.lag(),
            pairs,
            arm_of_in,
            arm_of_out,
            hold: vec![VecDeque::new(); n],
            log: vec![VecDeque::new(); n],
            log_cap,
            passed: vec![0; n],
            last_pass: vec![f64::NEG_INFINITY; n],
            last_circ_pass: vec![f64::NEG_INFINITY; n],
            last_entry: SmallMap::default(),
            on_ring: RingSet::default(),
            pending_ready: vec![Vec::new(); n],
            trace: vec![Vec::new(); n],
            version: 0,
            ring_cache: vec![None; n],
            entry_cache: SmallMap::default(),
            scratch: std::cell::RefCell::new(vec![0; n + 2]),
        }
    }

    /// Register a pending `RingReady` at `t` for point `p`; false when one
    /// is already pending at that exact time.
    pub fn add_pending(&mut self, p: usize, t: f64) -> bool {
        let bits = t.to_bits();
        let set = &mut self.pending_ready[p];
        if set.contains(&bits) {
            return false;
        }
        set.push(bits);
        true
    }

    /// The `RingReady` at `t` for point `p` fired.
    pub fn clear_pending(&mut self, p: usize, t: f64) {
        let bits = t.to_bits();
        let set = &mut self.pending_ready[p];
        if let Some(i) = set.iter().position(|&b| b == bits) {
            set.swap_remove(i);
        }
    }

    /// The head `veh` of ring point `p` is known to wait (unchanged ring).
    pub fn ring_head_waits(&self, p: usize, veh: VehID, now: f64) -> bool {
        still_waits(self.ring_cache[p].as_ref(), self.version, veh, now)
    }

    pub fn remember_ring_bound(&mut self, p: usize, veh: VehID, bound: Bound) {
        self.ring_cache[p] = Some((self.version, veh, bound));
    }

    /// The head `veh` of approach lane `key` is known to wait.
    pub fn entry_waits(&self, key: (LinkID, PipeIdx), veh: VehID, now: f64) -> bool {
        still_waits(self.entry_cache.get(&key), self.version, veh, now)
    }

    pub fn remember_entry_bound(&mut self, key: (LinkID, PipeIdx), veh: VehID, bound: Bound) {
        self.entry_cache.insert(key, (self.version, veh, bound));
    }

    #[inline]
    fn pair(&self, p: usize, q: usize) -> &PairConst {
        &self.pairs[p * self.geom.len() + q]
    }

    /// Forward steps from `from` to `to`, in 1..=n (n for a full lap).
    #[inline]
    fn steps(&self, from: usize, to: usize) -> usize {
        self.pair(from, to).steps
    }

    /// `q` strictly after `from` and at or before `to` (same as
    /// `RingGeometry::within`, from the table).
    #[inline]
    fn within(&self, from: usize, q: usize, to: usize) -> bool {
        q != from && self.steps(from, q) <= self.steps(from, to)
    }

    pub fn speed(&self) -> f64 {
        self.spec.speed
    }

    /// Free-flow time from point `from` to point `to`.
    pub fn travel_time(&self, from: usize, to: usize) -> f64 {
        self.pair(from, to).travel
    }

    /// Exit point for a vehicle leaving by `out_link`.
    pub fn exit_point_of(&self, out_link: LinkID) -> Option<usize> {
        self.arm_of_out.get(&out_link).map(|&a| self.geom.exit_point[a])
    }

    pub fn merge_point_of(&self, in_link: LinkID) -> Option<usize> {
        self.arm_of_in.get(&in_link).map(|&a| self.geom.merge_point[a])
    }

    /// Newell storage bound for a vehicle about to pass point `p`, bound for
    /// `exit_pt`. `me` excludes the vehicle itself from the count.
    ///
    /// For every downstream point q up to the exit (j steps after p), c(q) is
    /// the number of vehicles ahead between p and q that will pass q. A
    /// vehicle whose last point lies a steps after p (0 at p) and whose exit
    /// lies r steps after that point counts for a < j ≤ a + r, so one pass
    /// over the vehicles fills a difference array and a running sum gives
    /// every c(q): O(vehicles + points) instead of O(vehicles × points).
    pub fn storage_bound(&self, p: usize, exit_pt: usize, me: Option<VehID>) -> Bound {
        if p == exit_pt {
            return Bound::At(f64::NEG_INFINITY);
        }
        self.storage_bound_steps(p, self.steps(p, exit_pt), me)
    }

    /// Storage bound for a vehicle at its exit point that cannot leave and
    /// goes round again: every point of a full lap, back to `p`.
    pub fn storage_bound_lap(&self, p: usize, me: Option<VehID>) -> Bound {
        self.storage_bound_steps(p, self.geom.len(), me)
    }

    fn storage_bound_steps(&self, p: usize, len: usize, me: Option<VehID>) -> Bound {
        if self.kxl <= 0.0 {
            return Bound::At(f64::NEG_INFINITY);
        }
        let n = self.geom.len();
        let mut diff = self.scratch.borrow_mut();
        diff[..len + 2].fill(0);
        for (&v, rv) in self.on_ring.iter() {
            if Some(v) == me {
                continue;
            }
            let a = if rv.last_pt == p { 0 } else { self.steps(p, rv.last_pt) };
            if a >= len {
                continue;
            }
            let hi = (a + self.steps(rv.last_pt, rv.exit_pt)).min(len);
            if a < hi {
                diff[a + 1] += 1;
                diff[hi + 1] -= 1;
            }
        }
        let mut bound = Bound::At(f64::NEG_INFINITY);
        let mut ahead: i32 = 0;
        for j in 1..=len {
            ahead += diff[j];
            let q = (p + j) % n;
            let pc = self.pair(p, q);
            let ahead = ahead as usize;
            if ahead >= pc.k {
                return Bound::Blocked;
            }
            let need = pc.k - ahead;
            let log = &self.log[q];
            if need <= log.len() {
                let t = log[log.len() - need] + pc.wave + pc.frac;
                bound = bound.max(t);
            }
        }
        bound
    }

    /// Earliest passage of a circulating vehicle at `p` set by the ring
    /// capacity.
    pub fn capacity_bound(&self, p: usize) -> f64 {
        self.last_pass[p] + self.headway
    }

    /// When the head of approach (link, pipe) may enter at merge point `m`,
    /// bound for `exit_pt`, evaluated at `now`.
    ///
    /// When a cheap term (lead, ring headway, follow-up) already lies in the
    /// future, that time is returned as is: it is a lower bound, the caller
    /// wakes up then and evaluates the rest. Entry times are the same as with
    /// the full bound; only the storage and lag evaluations are saved.
    pub fn entry_bound(&self, m: usize, key: (LinkID, PipeIdx), exit_pt: usize, now: f64) -> Bound {
        // Priority: a ring vehicle waiting at the merge point goes first.
        if !self.hold[m].is_empty() {
            return Bound::Blocked;
        }
        let mut t = now
            .max(self.last_circ_pass[m] + self.lead)
            .max(self.capacity_bound(m));
        if let Some(&last) = self.last_entry.get(&key) {
            t = t.max(last + self.spec.follow_up);
        }
        if t > now {
            return Bound::At(t);
        }
        let b = Bound::At(t).and(self.storage_bound(m, exit_pt, None));
        let Bound::At(t) = b else {
            return Bound::Blocked;
        };
        // Lag: no circulating vehicle in motion may reach M within `lag`.
        let lag = self.lag;
        let conflict = self.on_ring.values().any(|rv| {
            rv.in_transit
                && rv.last_pt != m
                && self.within(rv.last_pt, m, rv.exit_pt)
                && rv.last_time + self.travel_time(rv.last_pt, m) < t + lag
        });
        if conflict {
            Bound::Blocked
        } else {
            Bound::At(t)
        }
    }

    /// Record a passage at `p`.
    pub fn record(&mut self, p: usize, veh: VehID, time: f64, circulating: bool) {
        self.version += 1;
        self.passed[p] += 1;
        self.last_pass[p] = time;
        if circulating {
            self.last_circ_pass[p] = time;
        }
        let log = &mut self.log[p];
        log.push_back(time);
        if log.len() > self.log_cap {
            log.pop_front();
        }
        self.trace[p].push(Passage { time, veh, circulating });
    }

    /// A ring vehicle reached point `p` and waits to pass it.
    pub fn arrive(&mut self, p: usize, veh: VehID) {
        self.version += 1;
        if let Some(rv) = self.on_ring.get_mut(&veh) {
            rv.in_transit = false;
        }
        self.hold[p].push_back(veh);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(arms: &[f64]) -> RoundaboutSpec {
        RoundaboutSpec {
            circumference: 100.0,
            lanes: 1,
            speed: 10.0,
            fd: FundamentalDiagram { u: 10.0, w: 5.0, kx: 0.1, c: 1.0 },
            critical_gap: 4.0,
            follow_up: 2.0,
            island: 4.0,
            arms: arms
                .iter()
                .enumerate()
                .map(|(k, &p)| RoundaboutArm { in_link: Some(2 * k), out_link: Some(2 * k + 1), position: p })
                .collect(),
        }
    }

    #[test]
    fn geometry_orders_exit_before_merge_even_across_zero() {
        let g = RingGeometry::new(&spec(&[1.0, 50.0]));
        let kinds: Vec<(PointKind, usize)> = g.points.iter().map(|p| (p.kind, p.arm)).collect();
        assert_eq!(
            kinds,
            vec![(PointKind::Exit, 0), (PointKind::Merge, 0), (PointKind::Exit, 1), (PointKind::Merge, 1)]
        );
        assert!((g.arc(g.exit_point[0], g.merge_point[0]) - 4.0).abs() < 1e-12);
        assert!((g.arc(g.merge_point[0], g.exit_point[1]) - 45.0).abs() < 1e-12);
        // U-turn: from M_0 all the way round to X_0.
        assert!((g.arc(g.merge_point[0], g.exit_point[0]) - 96.0).abs() < 1e-12);
    }

    #[test]
    fn within_follows_the_direction_of_circulation() {
        let g = RingGeometry::new(&spec(&[0.0, 25.0, 50.0, 75.0]));
        // Points: X0 M0 X1 M1 X2 M2 X3 M3 = 0..8
        assert!(g.within(1, 2, 4));
        assert!(g.within(1, 4, 4));
        assert!(!g.within(1, 5, 4));
        assert!(!g.within(1, 1, 4));
        assert!(g.within(7, 0, 2)); // wraps
        assert!(g.within(1, 0, 0)); // full lap to X0
    }

    #[test]
    fn empty_ring_has_no_storage_bound() {
        let st = RoundaboutState::new(0, &spec(&[0.0, 50.0]));
        assert_eq!(st.storage_bound(1, 2, None), Bound::At(f64::NEG_INFINITY));
    }

    #[test]
    fn storage_blocks_when_the_arc_is_full() {
        // M0 → X1 is 46 m: K = ceil(0.1 · 46) = 5.
        let mut st = RoundaboutState::new(0, &spec(&[0.0, 50.0]));
        let (m0, x1) = (st.geom.merge_point[0], st.geom.exit_point[1]);
        for v in 0..5 {
            st.on_ring.insert(v, RingVehicle { last_pt: m0, last_time: 0.0, exit_pt: x1, in_transit: false });
        }
        assert_eq!(st.storage_bound(m0, x1, None), Bound::Blocked);
        st.on_ring.remove(&4);
        // Four ahead, none passed X1 yet: need 1 more passage than logged.
        assert_eq!(st.storage_bound(m0, x1, None), Bound::At(f64::NEG_INFINITY));
    }

    #[test]
    fn storage_wave_delay_matches_the_pipe_rule() {
        // One passage at X1 at t = 10; empty arc; K = 5 > 1 logged ⇒ free.
        // With 4 vehicles ahead, need = 1 ⇒ bound = 10 + 46/5 + (5 − 4.6)/C.
        let mut st = RoundaboutState::new(0, &spec(&[0.0, 50.0]));
        let (m0, x1) = (st.geom.merge_point[0], st.geom.exit_point[1]);
        st.record(x1, 99, 10.0, true);
        for v in 0..4 {
            st.on_ring.insert(v, RingVehicle { last_pt: m0, last_time: 0.0, exit_pt: x1, in_transit: true });
        }
        let cap = st.spec.capacity();
        let expected = 10.0 + 46.0 / 5.0 + (5.0 - 4.6) / cap;
        match st.storage_bound(m0, x1, None) {
            Bound::At(t) => assert!((t - expected).abs() < 1e-9, "{t} vs {expected}"),
            Bound::Blocked => panic!("should not block"),
        }
    }

    #[test]
    fn entry_waits_for_lead_and_rejects_short_lag() {
        let mut st = RoundaboutState::new(0, &spec(&[0.0, 50.0]));
        let (m0, x1) = (st.geom.merge_point[0], st.geom.exit_point[1]);
        // A circulating vehicle passed M0 at t = 5 ⇒ entry not before 5 + lead.
        st.record(m0, 7, 5.0, true);
        let key = (0, 0);
        match st.entry_bound(m0, key, x1, 5.0) {
            Bound::At(t) => assert!((t - 6.0).abs() < 1e-12, "lead = t_f/2 = 1 s, got {t}"),
            Bound::Blocked => panic!(),
        }
        // A vehicle in motion 15 m upstream of M0 (1.5 s away) is inside the
        // 2 s lag ⇒ blocked.
        let m1 = st.geom.merge_point[1];
        st.on_ring.insert(8, RingVehicle { last_pt: m1, last_time: 10.0 - 4.5, exit_pt: x1, in_transit: true });
        // From M1 (pos 52) to M0 (pos 2) = 50 m = 5 s; arrives at 10.5.
        assert_eq!(st.entry_bound(m0, key, x1, 9.0), Bound::Blocked);
        assert!(matches!(st.entry_bound(m0, key, x1, 7.0), Bound::At(_)));
    }

    #[test]
    fn infers_four_arms_from_bearings() {
        use crate::model::NodeType;
        let node = Node {
            id: 0,
            node_type: NodeType::Internal,
            incoming_links: vec![0, 2, 4, 6],
            outgoing_links: vec![1, 3, 5, 7],
            points: (1000.0, 1000.0),
            signals: vec![],
            roundabout: None, junction: None,
        };
        let fd = FundamentalDiagram { u: 10.0, w: 5.0, kx: 0.15, c: 0.5 };
        let mut links = Vec::new();
        for k in 0..4 {
            let a = k as f64 * std::f64::consts::FRAC_PI_2;
            let far = (1000.0 + 100.0 * a.cos(), 1000.0 + 100.0 * a.sin());
            links.push(Link::new(2 * k, 1, 0, 100.0, 10.0, 1, 0.5, fd.clone(), vec![far, node.points]));
            links.push(Link::new(2 * k + 1, 0, 2, 100.0, 10.0, 1, 0.5, fd.clone(), vec![node.points, far]));
        }
        let raw = serde_json::json!({ "diameter": 40.0 });
        let spec = resolve_spec(&raw, &node, &links).unwrap();
        assert_eq!(spec.arms.len(), 4);
        for (k, arm) in spec.arms.iter().enumerate() {
            assert_eq!(arm.in_link, Some(2 * k));
            assert_eq!(arm.out_link, Some(2 * k + 1));
            let expected = k as f64 * spec.circumference / 4.0;
            assert!((arm.position - expected).abs() < 1e-9);
        }
        validate_spec(&spec, &node).unwrap();
        // Clockwise reverses the order of arms 1 and 3.
        let spec = resolve_spec(&serde_json::json!({ "diameter": 40.0, "direction": "cw" }), &node, &links).unwrap();
        let order: Vec<_> = spec.arms.iter().map(|a| a.in_link).collect();
        assert_eq!(order, vec![Some(0), Some(6), Some(4), Some(2)]);
    }
}
