//! OpenStreetMap extract: nodes, ways, a directed road graph, and the A47
//! carriageways as chained polylines.

use serde_json::Value;
use std::collections::{HashMap, HashSet};

pub type Id = i64;

pub struct Osm {
    pub pos: HashMap<Id, (f64, f64)>,
    pub ways: Vec<Way>,
    /// node -> [(next node, way index)] in the driving direction.
    pub out: HashMap<Id, Vec<(Id, usize)>>,
    pub inc: HashMap<Id, Vec<(Id, usize)>>,
}

pub struct Way {
    pub id: Id,
    pub nodes: Vec<Id>,
    pub tags: Value,
}

impl Way {
    pub fn tag(&self, k: &str) -> &str {
        self.tags[k].as_str().unwrap_or("")
    }
    pub fn highway(&self) -> &str {
        self.tag("highway")
    }
    pub fn is_link(&self) -> bool {
        self.highway().ends_with("_link")
    }
    pub fn is_a47(&self) -> bool {
        self.tag("ref").split(';').any(|r| r.trim() == "A 47") && matches!(self.highway(), "motorway" | "trunk")
    }
    pub fn lanes(&self) -> Option<u8> {
        self.tag("lanes").parse().ok()
    }
    /// km/h
    pub fn maxspeed(&self) -> Option<f64> {
        self.tag("maxspeed").parse().ok()
    }
}

pub fn hav(a: (f64, f64), b: (f64, f64)) -> f64 {
    let r = 6_371_008.8;
    let (p1, p2) = (a.1.to_radians(), b.1.to_radians());
    let dp = p2 - p1;
    let dl = (b.0 - a.0).to_radians();
    let h = (dp / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
    2.0 * r * h.sqrt().asin()
}

pub fn load(path: &std::path::Path) -> Osm {
    let v: Value = serde_json::from_str(&std::fs::read_to_string(path).expect("read osm")).expect("parse osm");
    let mut pos = HashMap::new();
    let mut ways = Vec::new();
    for e in v["elements"].as_array().expect("elements") {
        match e["type"].as_str() {
            Some("node") => {
                pos.insert(e["id"].as_i64().unwrap(), (e["lon"].as_f64().unwrap(), e["lat"].as_f64().unwrap()));
            }
            Some("way") => ways.push(Way {
                id: e["id"].as_i64().unwrap(),
                nodes: e["nodes"].as_array().unwrap().iter().map(|n| n.as_i64().unwrap()).collect(),
                tags: e["tags"].clone(),
            }),
            _ => {}
        }
    }
    let mut out: HashMap<Id, Vec<(Id, usize)>> = HashMap::new();
    let mut inc: HashMap<Id, Vec<(Id, usize)>> = HashMap::new();
    for (wi, w) in ways.iter().enumerate() {
        let oneway = w.tag("oneway");
        let implied = matches!(w.highway(), "motorway" | "motorway_link") || w.tag("junction") == "roundabout";
        let fwd = oneway != "-1";
        let back = oneway == "-1" || (oneway != "yes" && !implied);
        for s in w.nodes.windows(2) {
            if fwd {
                out.entry(s[0]).or_default().push((s[1], wi));
                inc.entry(s[1]).or_default().push((s[0], wi));
            }
            if back {
                out.entry(s[1]).or_default().push((s[0], wi));
                inc.entry(s[0]).or_default().push((s[1], wi));
            }
        }
    }
    Osm { pos, ways, out, inc }
}

/// One carriageway: OSM nodes in driving order with chainage, and for each
/// edge (i, i+1) the way it belongs to.
pub struct Carriageway {
    pub nodes: Vec<Id>,
    pub pts: Vec<(f64, f64)>,
    pub cum: Vec<f64>,
    pub edge_way: Vec<usize>,
}

impl Carriageway {
    pub fn len(&self) -> f64 {
        *self.cum.last().unwrap()
    }
    pub fn at(&self, c: f64) -> (f64, f64) {
        let c = c.clamp(0.0, self.len());
        let i = self.cum.partition_point(|&x| x <= c).clamp(1, self.pts.len() - 1);
        let (a, b) = (self.pts[i - 1], self.pts[i]);
        let seg = self.cum[i] - self.cum[i - 1];
        let t = if seg > 0.0 { (c - self.cum[i - 1]) / seg } else { 0.0 };
        (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1))
    }
    /// Polyline between two chainages.
    pub fn sub(&self, c0: f64, c1: f64) -> Vec<(f64, f64)> {
        let mut v = vec![self.at(c0)];
        for (p, &c) in self.pts.iter().zip(&self.cum) {
            if c > c0 + 0.5 && c < c1 - 0.5 {
                v.push(*p);
            }
        }
        v.push(self.at(c1));
        v
    }
    /// The way under chainage c.
    pub fn way_at(&self, c: f64) -> usize {
        let i = self.cum.partition_point(|&x| x <= c).clamp(1, self.pts.len() - 1);
        self.edge_way[i - 1]
    }
    pub fn chainage_of(&self, node: Id) -> Option<f64> {
        self.nodes.iter().position(|&n| n == node).map(|i| self.cum[i])
    }
}

/// Follow A47 edges from a source node.
fn walk(osm: &Osm, start: Id) -> (Vec<Id>, Vec<usize>) {
    let mut path = vec![start];
    let mut ways = vec![];
    let mut seen = HashSet::from([start]);
    let mut cur = start;
    while let Some(&(n, wi)) = osm.out.get(&cur).into_iter().flatten().find(|(n, wi)| osm.ways[*wi].is_a47() && !seen.contains(n)) {
        path.push(n);
        ways.push(wi);
        seen.insert(n);
        cur = n;
    }
    (path, ways)
}

/// The longest A47 carriageway heading west (`westward`) or east.
pub fn carriageway(osm: &Osm, westward: bool) -> Carriageway {
    let a47: HashSet<Id> = osm.ways.iter().filter(|w| w.is_a47()).flat_map(|w| w.nodes.iter().copied()).collect();
    let has_in = |n: &Id| osm.inc.get(n).into_iter().flatten().any(|(_, wi)| osm.ways[*wi].is_a47());
    let mut best: Option<(Vec<Id>, Vec<usize>)> = None;
    let mut sources: Vec<Id> = a47.into_iter().filter(|n| !has_in(n)).collect();
    sources.sort();
    for s in sources {
        let (path, ways) = walk(osm, s);
        if path.len() < 20 {
            continue;
        }
        let (a, b) = (osm.pos[&path[0]], osm.pos[path.last().unwrap()]);
        if (b.0 < a.0) == westward && best.as_ref().is_none_or(|(p, _)| path.len() > p.len()) {
            best = Some((path, ways));
        }
    }
    let (nodes, edge_way) = best.expect("carriageway");
    let pts: Vec<(f64, f64)> = nodes.iter().map(|n| osm.pos[n]).collect();
    let mut cum = vec![0.0];
    for w in pts.windows(2) {
        cum.push(cum.last().unwrap() + hav(w[0], w[1]));
    }
    Carriageway { nodes, pts, cum, edge_way }
}

/// A ramp leaving or joining a carriageway at one of its nodes.
pub struct Junction {
    pub chainage: f64,
    pub node: Id,
    pub diverge: bool,
    pub way: usize,
}

pub fn junctions(osm: &Osm, cw: &Carriageway) -> Vec<Junction> {
    let mut v = Vec::new();
    for (i, n) in cw.nodes.iter().enumerate() {
        for (m, wi) in osm.out.get(n).into_iter().flatten() {
            if !osm.ways[*wi].is_a47() && cw.nodes.get(i + 1) != Some(m) {
                v.push(Junction { chainage: cw.cum[i], node: *n, diverge: true, way: *wi });
            }
        }
        for (m, wi) in osm.inc.get(n).into_iter().flatten() {
            if !osm.ways[*wi].is_a47() && (i == 0 || cw.nodes.get(i - 1) != Some(m)) {
                v.push(Junction { chainage: cw.cum[i], node: *n, diverge: false, way: *wi });
            }
        }
    }
    v
}

/// The ramp's polyline from its junction (diverge: forward; merge: backward,
/// returned in driving order), up to where it stops being a single ramp:
/// it ends, branches, or meets another road. Returns the OSM ways used.
pub fn ramp(osm: &Osm, j: &Junction, max_len: f64) -> (Vec<(f64, f64)>, Vec<usize>) {
    let mut pts = vec![osm.pos[&j.node]];
    let mut ways = vec![j.way];
    let mut seen = HashSet::from([j.node]);
    let mut cur = j.node;
    let mut way = j.way;
    let mut len = 0.0;
    // First step along the junction's way.
    loop {
        let edges = if j.diverge { osm.out.get(&cur) } else { osm.inc.get(&cur) };
        let cand: Vec<&(Id, usize)> =
            edges.into_iter().flatten().filter(|(m, wi)| !seen.contains(m) && !osm.ways[*wi].is_a47()).collect();
        let next = if cur == j.node {
            cand.iter().find(|(_, wi)| *wi == way).copied()
        } else if cand.len() == 1 && osm.ways[cand[0].1].is_link() {
            Some(cand[0])
        } else {
            None
        };
        let Some(&(n, wi)) = next else { break };
        // Stop where another ramp branches off or joins (the ramp is no longer one stream).
        let other = if j.diverge { osm.inc.get(&n) } else { osm.out.get(&n) };
        let joined = other.into_iter().flatten().filter(|(m, _)| *m != cur).count() > 0;
        len += hav(osm.pos[&cur], osm.pos[&n]);
        pts.push(osm.pos[&n]);
        if wi != way {
            ways.push(wi);
        }
        seen.insert(n);
        cur = n;
        way = wi;
        if joined || len >= max_len {
            break;
        }
    }
    if !j.diverge {
        pts.reverse();
    }
    (pts, ways)
}

/// Project p on a polyline: (chainage, offset metres).
pub fn project(pts: &[(f64, f64)], p: (f64, f64)) -> (f64, f64) {
    let k = p.1.to_radians().cos();
    let r = 6_371_008.8f64;
    let xy = |q: (f64, f64)| (q.0.to_radians() * r * k, q.1.to_radians() * r);
    let q = xy(p);
    let mut best = (f64::INFINITY, 0.0);
    let mut cum = 0.0;
    for w in pts.windows(2) {
        let (a, b) = (xy(w[0]), xy(w[1]));
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l2 = dx * dx + dy * dy;
        let t = if l2 > 0.0 { (((q.0 - a.0) * dx + (q.1 - a.1) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
        let d = ((q.0 - a.0 - t * dx).powi(2) + (q.1 - a.1 - t * dy).powi(2)).sqrt();
        let seg = hav(w[0], w[1]);
        if d < best.0 {
            best = (d, cum + t * seg);
        }
        cum += seg;
    }
    (best.1, best.0)
}

pub fn length(pts: &[(f64, f64)]) -> f64 {
    pts.windows(2).map(|w| hav(w[0], w[1])).sum()
}

/// Which side of the travel direction (at `from`, heading to `to`) a point
/// lies: > 0 right, < 0 left.
pub fn side(from: (f64, f64), to: (f64, f64), p: (f64, f64)) -> f64 {
    let k = from.1.to_radians().cos();
    let (dx, dy) = ((to.0 - from.0) * k, to.1 - from.1);
    let (px, py) = ((p.0 - from.0) * k, p.1 - from.1);
    // Cross product z: positive = left (counter-clockwise).
    -(dx * py - dy * px)
}
