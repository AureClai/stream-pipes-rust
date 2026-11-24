//! Builds `scenarios/a47_givors_osm` from the raw data in its `raw/` folder:
//!
//! - `raw/osm.json`: OpenStreetMap motorways and ramps (Overpass extract);
//! - `raw/avatar_countpoints.csv`, `raw/avatar_2026-06-11.csv`: Cerema AVATAR
//!   count points and their corrected measures for the simulated day.
//!
//! Writes `network.geojson`, `demand.json`, `config.json`, `observed.json`,
//! `stations.geojson` and `BUILD_REPORT.md` next to `raw/`. Offline and
//! deterministic: the same raw files give the same scenario, byte for byte.
//!
//! cargo run --release --manifest-path scenarios/a47_givors_osm/build/Cargo.toml

mod avatar;
mod osm;

use avatar::{CountPoint, Sample};
use osm::{Carriageway, Id, Osm};
use serde_json::{Value, json};
use std::collections::{BTreeMap, HashMap};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

// ── Modelling choices (documented in README.md) ─────────────────────────────

/// Simulated day and horizon (local time, seconds after midnight).
const DAY: &str = "2026-06-11";
const START: f64 = 5.0 * 3600.0;
const END: f64 = 22.0 * 3600.0;
/// Demand and validation windows.
const WINDOW: f64 = 1800.0;
/// Per-lane capacity (veh/s) and jam density (veh/m) of the triangular FD.
const CAPACITY_PER_LANE: f64 = 0.5;
const JAM_DENSITY_PER_LANE: f64 = 0.13;
/// Friction between pipes (paper: flat optimum φ ∈ [0.85, 0.95]).
const FRICTION: f64 = 0.95;
/// Corridor ends beyond the outermost stations.
const END_MARGIN: f64 = 400.0;
/// Points closer than this along a carriageway are one node.
const SNAP: f64 = 25.0;
/// A station farther than this from a road is not on it.
const STATION_OFFSET: f64 = 40.0;
/// Longest ramp followed from its junction.
const RAMP_MAX: f64 = 1500.0;
/// At most this share of the mainline leaves at one off-ramp.
const MAX_EXIT_SHARE: f64 = 0.9;
/// Below this speed (km/h) a station is in a queue.
const CONGESTED_KMH: f64 = 50.0;

#[derive(Clone, Copy, PartialEq, Debug)]
enum Dir {
    Lyon,
    StEtienne,
}

impl Dir {
    fn label(self) -> &'static str {
        match self {
            Dir::Lyon => "vers Lyon",
            Dir::StEtienne => "vers Saint-Étienne",
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
enum NodeKind {
    Entry,
    Exit,
    Internal,
}

struct Node {
    pos: (f64, f64),
    kind: NodeKind,
    label: String,
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Kind {
    Mainline,
    OffRamp,
    OnRamp,
}

struct Link {
    up: usize,
    down: usize,
    pts: Vec<(f64, f64)>,
    lanes: u8,
    /// km/h, from OSM maxspeed.
    limit_kmh: f64,
    /// m/s, free-flow speed of the FD.
    fd_u: f64,
    kind: Kind,
    dir: Dir,
    label: String,
    osm_ways: Vec<Id>,
    /// downstream link → pipes that may take it.
    moves: Vec<(usize, Vec<u8>)>,
    /// Chainage of the link on its carriageway (mainline) or of its junction (ramps).
    ch0: f64,
    ch1: f64,
}

/// A station on the network.
struct Station {
    cp: usize,
    link: usize,
    dir: Option<Dir>,
    chainage: f64,
    offset: f64,
}

struct Net {
    nodes: Vec<Node>,
    links: Vec<Link>,
}

impl Net {
    fn node(&mut self, pos: (f64, f64), kind: NodeKind, label: &str) -> usize {
        self.nodes.push(Node { pos, kind, label: label.to_string() });
        self.nodes.len() - 1
    }
}

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).parent().expect("scenario folder").to_path_buf()
}

/// The ramp label OSM gives: its destination, else its name.
fn ramp_label(osm: &Osm, ways: &[usize], diverge: bool) -> String {
    for &wi in ways {
        let w = &osm.ways[wi];
        for k in ["destination", "destination:ref", "name", "ref"] {
            let v = w.tag(k);
            if !v.is_empty() {
                return format!("{} {}", if diverge { "sortie" } else { "entrée" }, v.replace(';', ", "));
            }
        }
    }
    if diverge { "sortie".into() } else { "entrée".into() }
}

/// Build one direction's mainline links and ramps.
fn build_direction(
    osm: &Osm,
    net: &mut Net,
    dir: Dir,
    cw: &Carriageway,
    c0: f64,
    c1: f64,
    station_chainages: &[f64],
    report: &mut String,
) -> Vec<usize> {
    let juncs: Vec<osm::Junction> = osm::junctions(osm, cw)
        .into_iter()
        .filter(|j| j.chainage > c0 + SNAP && j.chainage <= c1 + 1e-6)
        .filter(|j| j.chainage < c1 - 1e-6 || j.diverge)
        .collect();

    // Break points: ends, junctions, lane changes.
    let mut breaks: Vec<f64> = vec![c0, c1];
    breaks.extend(juncs.iter().map(|j| j.chainage));
    let lanes_of = |wi: usize| osm.ways[wi].lanes().unwrap_or(2);
    for i in 1..cw.edge_way.len() {
        if lanes_of(cw.edge_way[i]) != lanes_of(cw.edge_way[i - 1]) {
            let c = cw.cum[i];
            if c > c0 && c < c1 {
                breaks.push(c);
            }
        }
    }
    breaks.sort_by(f64::total_cmp);
    // One station per link: split halfway between two stations that nothing
    // else separates, so each has the free-flow speed it measures.
    let mut st: Vec<f64> = station_chainages.iter().copied().filter(|c| *c > c0 && *c < c1).collect();
    st.sort_by(f64::total_cmp);
    for w in st.windows(2) {
        if !breaks.iter().any(|b| *b > w[0] && *b < w[1]) {
            breaks.push((w[0] + w[1]) / 2.0);
        }
    }
    breaks.sort_by(f64::total_cmp);
    // Snap: keep junctions over lane changes when they are close.
    let is_junction = |c: f64| juncs.iter().any(|j| (j.chainage - c).abs() < 1e-6) || c == c0 || c == c1;
    let mut kept: Vec<f64> = Vec::new();
    for c in breaks {
        match kept.last() {
            Some(&last) if c - last < SNAP => {
                if is_junction(c) && !is_junction(last) {
                    *kept.last_mut().unwrap() = c;
                }
            }
            _ => kept.push(c),
        }
    }

    // Nodes and mainline links.
    let mut node_at: Vec<(f64, usize)> = Vec::new();
    for (i, &c) in kept.iter().enumerate() {
        let kind = if i == 0 {
            NodeKind::Entry
        } else if i == kept.len() - 1 && dir == Dir::StEtienne {
            NodeKind::Exit
        } else {
            NodeKind::Internal
        };
        let label = match (i == 0, i == kept.len() - 1) {
            (true, _) => format!("A47 {} (début)", dir.label()),
            (_, true) => format!("A47 {} (fin)", dir.label()),
            _ => String::new(),
        };
        node_at.push((c, net.node(cw.at(c), kind, &label)));
    }
    let mut mainline = Vec::new();
    for w in node_at.windows(2) {
        let ((ca, na), (cb, nb)) = (w[0], w[1]);
        let mid = (ca + cb) / 2.0;
        let way = &osm.ways[cw.way_at(mid)];
        let mut ways: Vec<Id> = Vec::new();
        for (i, &wi) in cw.edge_way.iter().enumerate() {
            let (e0, e1) = (cw.cum[i], cw.cum[i + 1]);
            if e1 > ca + 0.5 && e0 < cb - 0.5 && !ways.contains(&osm.ways[wi].id) {
                ways.push(osm.ways[wi].id);
            }
        }
        net.links.push(Link {
            up: na,
            down: nb,
            pts: cw.sub(ca, cb),
            lanes: way.lanes().unwrap_or(2),
            limit_kmh: way.maxspeed().unwrap_or(90.0),
            fd_u: 0.0,
            kind: Kind::Mainline,
            dir,
            label: format!("A47 {}", dir.label()),
            osm_ways: ways,
            moves: Vec::new(),
            ch0: ca,
            ch1: cb,
        });
        mainline.push(net.links.len() - 1);
    }
    let node_for = |c: f64| node_at.iter().min_by(|a, b| (a.0 - c).abs().total_cmp(&(b.0 - c).abs())).map(|x| x.1).unwrap();

    // Ramps.
    let _ = writeln!(report, "\n### Bretelles {}\n\n| Lien | Type | Libellé OSM | Longueur | Voies |\n|---|---|---|---|---|", dir.label());
    for j in &juncs {
        let (pts, ways) = osm::ramp(osm, j, RAMP_MAX);
        let len = osm::length(&pts);
        if len < 30.0 {
            continue;
        }
        let w0 = &osm.ways[ways[0]];
        let lanes = w0.lanes().unwrap_or(1);
        let limit = w0.maxspeed().unwrap_or(if w0.highway() == "motorway_link" { 70.0 } else { 50.0 });
        let label = ramp_label(osm, &ways, j.diverge);
        let at = node_for(j.chainage);
        let end = if j.diverge { *pts.last().unwrap() } else { pts[0] };
        let far = net.node(end, if j.diverge { NodeKind::Exit } else { NodeKind::Entry }, &label);
        let (up, down) = if j.diverge { (at, far) } else { (far, at) };
        let mut pts = pts;
        // Start (or end) exactly on the junction node.
        if j.diverge {
            pts[0] = net.nodes[at].pos;
        } else {
            *pts.last_mut().unwrap() = net.nodes[at].pos;
        }
        net.links.push(Link {
            up,
            down,
            pts,
            lanes,
            limit_kmh: limit,
            fd_u: limit / 3.6,
            kind: if j.diverge { Kind::OffRamp } else { Kind::OnRamp },
            dir,
            label: label.clone(),
            osm_ways: ways.iter().map(|&wi| osm.ways[wi].id).collect(),
            moves: Vec::new(),
            ch0: j.chainage,
            ch1: j.chainage,
        });
        let id = net.links.len() - 1;
        let _ = writeln!(
            report,
            "| {id} | {} | {label} | {len:.0} m | {lanes} |",
            if j.diverge { "sortie" } else { "entrée" }
        );
    }

    // Movement restrictions at diverges: the exit takes the right pipes; when
    // the mainline loses lanes there, those lanes lead only to the exit.
    for &m in &mainline {
        let down = net.links[m].down;
        let lanes = net.links[m].lanes;
        let outs: Vec<usize> = (0..net.links.len()).filter(|&l| net.links[l].up == down).collect();
        let exits: Vec<usize> = outs.iter().copied().filter(|&l| net.links[l].kind == Kind::OffRamp).collect();
        if exits.is_empty() || lanes < 2 {
            continue;
        }
        let next_main = outs.iter().copied().find(|&l| net.links[l].kind == Kind::Mainline);
        // Rightmost first.
        let from = net.nodes[net.links[m].up].pos;
        let to = net.nodes[down].pos;
        let mut ordered = exits.clone();
        if let Some(n) = next_main {
            ordered.push(n);
        }
        let far_point = |l: usize| {
            let p = &net.links[l].pts;
            p[(p.len() - 1).min(3)]
        };
        ordered.sort_by(|&a, &b| osm::side(from, to, far_point(b)).total_cmp(&osm::side(from, to, far_point(a))));
        let mut next_pipe: u8 = 0;
        let n = ordered.len();
        for (k, &l) in ordered.iter().enumerate() {
            let want = if k == n - 1 {
                lanes - next_pipe.min(lanes - 1)
            } else {
                net.links[l].lanes.min(lanes - 1 - next_pipe.min(lanes - 1)).max(1)
            };
            let first = next_pipe.min(lanes - 1);
            let pipes: Vec<u8> = (first..(first + want).min(lanes)).collect();
            next_pipe = (first + want).min(lanes - 1);
            // A through lane that does not drop keeps every pipe.
            let pipes = if net.links[l].kind == Kind::Mainline && net.links[l].lanes >= lanes {
                (0..lanes).collect()
            } else {
                pipes
            };
            net.links[m].moves.push((l, pipes));
        }
    }
    mainline
}

/// Free-flow speed (m/s) of a station: median measured speed over the
/// simulated hours, leaving out congested samples. Daytime traffic, not the
/// faster night, sets the speed the model is compared with.
fn free_speed(samples: &[Sample]) -> Option<f64> {
    let mut v: Vec<f64> = samples
        .iter()
        .filter(|s| s.speed_measured && s.t >= START && s.t < END)
        .filter_map(|s| s.speed)
        .filter(|v| *v > CONGESTED_KMH)
        .collect();
    if v.len() < 10 {
        return None;
    }
    v.sort_by(f64::total_cmp);
    Some(v[v.len() / 2] / 3.6)
}

/// Mean flow change (veh/h) to a threshold beyond which a station is suspect.
const SCREEN_DELTA: f64 = 150.0;
/// A window departs from its pair's usual flow change by more than this
/// (veh/h, or this share of the upstream flow, whichever is larger).
const SCREEN_WINDOW_ABS: f64 = 300.0;
const SCREEN_WINDOW_REL: f64 = 0.25;

/// Mean measured speed of a station over a window.
fn speed_over(samples: &[Sample], t0: f64, t1: f64) -> Option<f64> {
    let v: Vec<f64> = avatar::in_window(samples, t0, t1).iter().filter(|x| x.speed_measured).filter_map(|x| x.speed).collect();
    (!v.is_empty()).then(|| v.iter().sum::<f64>() / v.len() as f64)
}

/// Physical-impossibility test, per window: with both stations of a pair in
/// free flow (no queue can store or release vehicles), the flow change
/// between them stays what the ramps make it all day. A window that departs
/// from the pair's median change by far, with the upstream station agreeing
/// with its own upstream neighbour, is a counting fault at the downstream
/// station. Returns (station id, window start).
fn screen_windows(
    stations: &[Station],
    cps: &[CountPoint],
    meas: &BTreeMap<u32, Vec<Sample>>,
    excluded: &[u32],
    report: &mut String,
) -> Vec<(u32, f64)> {
    let mut out = Vec::new();
    for d in [Dir::Lyon, Dir::StEtienne] {
        let mut st: Vec<&Station> =
            stations.iter().filter(|s| s.dir == Some(d) && !excluded.contains(&cps[s.cp].id)).collect();
        st.sort_by(|a, b| a.chainage.total_cmp(&b.chainage));
        for w in st.windows(2) {
            let (a, b) = (&meas[&cps[w[0].cp].id], &meas[&cps[w[1].cp].id]);
            let mut deltas: Vec<(f64, f64, f64)> = Vec::new(); // (t, delta, q_up)
            let mut t = START;
            while t + WINDOW <= END + 1e-6 {
                if let (Some((qa, _, _)), Some((qb, _, _)), Some(va), Some(vb)) = (
                    avatar::flow_over(a, t, t + WINDOW, true),
                    avatar::flow_over(b, t, t + WINDOW, true),
                    speed_over(a, t, t + WINDOW),
                    speed_over(b, t, t + WINDOW),
                ) && va > CONGESTED_KMH
                    && vb > CONGESTED_KMH
                {
                    deltas.push((t, qb - qa, qa));
                }
                t += WINDOW;
            }
            if deltas.len() < 5 {
                continue;
            }
            // The usual change scales with the traffic: compare shares of the
            // upstream flow, not vehicles.
            let mut ratios: Vec<f64> = deltas.iter().filter(|x| x.2 > 0.0).map(|x| x.1 / x.2).collect();
            ratios.sort_by(f64::total_cmp);
            let ratio = ratios[ratios.len() / 2];
            for (t, delta, qa) in deltas {
                let median = ratio * qa;
                if (delta - median).abs() > SCREEN_WINDOW_ABS.max(SCREEN_WINDOW_REL * qa) {
                    out.push((cps[w[1].cp].id, t));
                    let _ = writeln!(
                        report,
                        "- {} {} {} : {:+.0} véh/h depuis {} (habituel {:+.0}), en fluide — fenêtre écartée de la validation et de la demande.",
                        cps[w[1].cp].name,
                        d.label(),
                        hhmm(t),
                        delta,
                        cps[w[0].cp].name,
                        median
                    );
                }
            }
        }
    }
    out
}

fn screen_stations(
    net: &Net,
    stations: &[Station],
    cps: &[CountPoint],
    meas: &BTreeMap<u32, Vec<Sample>>,
    report: &mut String,
) -> Vec<u32> {
    let mut out = Vec::new();
    let _ = writeln!(report, "\n### Dépistage des stations\n");
    for d in [Dir::Lyon, Dir::StEtienne] {
        let mut st: Vec<&Station> = stations.iter().filter(|s| s.dir == Some(d)).collect();
        st.sort_by(|a, b| a.chainage.total_cmp(&b.chainage));
        let ramps_between = |a: f64, b: f64| {
            net.links.iter().any(|l| l.dir == d && l.kind != Kind::Mainline && l.ch0 > a && l.ch0 <= b)
        };
        // Mean (downstream − upstream) over the windows where both are measured.
        let mean_delta = |a: &Station, b: &Station| {
            let (sa, sb) = (&meas[&cps[a.cp].id], &meas[&cps[b.cp].id]);
            let mut v = Vec::new();
            let mut t = START;
            while t + WINDOW <= END + 1e-6 {
                if let (Some((qa, _, _)), Some((qb, _, _))) =
                    (avatar::flow_over(sa, t, t + WINDOW, true), avatar::flow_over(sb, t, t + WINDOW, true))
                {
                    v.push(qb - qa);
                }
                t += WINDOW;
            }
            (!v.is_empty()).then(|| v.iter().sum::<f64>() / v.len() as f64)
        };
        for k in 1..st.len().saturating_sub(1) {
            let (a, s, b) = (st[k - 1], st[k], st[k + 1]);
            if ramps_between(a.chainage, s.chainage) || ramps_between(s.chainage, b.chainage) {
                continue;
            }
            if let (Some(up), Some(down)) = (mean_delta(a, s), mean_delta(s, b))
                && up.abs() > SCREEN_DELTA
                && down.abs() > SCREEN_DELTA
                && up.signum() != down.signum()
            {
                let _ = writeln!(
                    report,
                    "- **{}** ({} {}) écartée de la construction de la demande : {:+.0} véh/h depuis {}, puis {:+.0} véh/h jusqu'à {}, sans bretelle entre elles. Elle est aussi écartée de `observed.json` : une donnée qui échoue au dépistage ne juge pas le modèle.",
                    cps[s.cp].name,
                    cps[s.cp].id,
                    d.label(),
                    up,
                    cps[a.cp].name,
                    down,
                    cps[b.cp].name
                );
                out.push(cps[s.cp].id);
            }
        }
    }
    if out.is_empty() {
        let _ = writeln!(report, "Aucune station écartée.");
    }
    out
}

fn round7(p: (f64, f64)) -> Value {
    json!([(p.0 * 1e7).round() / 1e7, (p.1 * 1e7).round() / 1e7])
}

fn write_json(path: &Path, v: &Value) {
    let mut buf = Vec::new();
    let fmt = serde_json::ser::PrettyFormatter::with_indent(b" ");
    let mut ser = serde_json::Serializer::with_formatter(&mut buf, fmt);
    serde::Serialize::serialize(v, &mut ser).expect("serialize");
    buf.push(b'\n');
    std::fs::write(path, buf).expect("write");
}

fn hhmm(t: f64) -> String {
    format!("{:02}:{:02}", (t / 3600.0) as u32, ((t % 3600.0) / 60.0) as u32)
}

fn main() {
    let dir = root();
    let raw = dir.join("raw");
    let osm = osm::load(&raw.join("osm.json"));
    let cps = avatar::count_points(&raw.join("avatar_countpoints.csv"));
    let meas = avatar::measures(&raw.join(format!("avatar_{DAY}.csv")));
    let mut report = String::new();
    let _ = writeln!(report, "# A47 Givors (OSM) — rapport de construction\n\nGénéré par `build/` à partir de `raw/`. Jour simulé : {DAY}, {}–{}.\n", hhmm(START), hhmm(END));

    let east = osm::carriageway(&osm, false);
    let west = osm::carriageway(&osm, true);
    // Corridor ends: around the westmost stations (91/92), and the A7 junction.
    let has_data = |cp: &CountPoint| meas.get(&cp.id).is_some_and(|s| s.iter().any(|x| x.flow.is_some()));
    let westmost = |cw: &Carriageway| {
        cps.iter()
            .filter(|c| c.road == "A0047" && has_data(c))
            .map(|c| osm::project(&cw.pts, c.pos))
            .filter(|(_, off)| *off < STATION_OFFSET)
            .map(|(ch, _)| ch)
            .collect::<Vec<f64>>()
    };
    let e_st = westmost(&east);
    let w_st = westmost(&west);
    let e0 = (e_st.iter().copied().fold(f64::INFINITY, f64::min) - END_MARGIN).max(0.0);
    let e1 = east.len();
    let w0 = 0.0;
    let w1 = (w_st.iter().copied().fold(0.0, f64::max) + END_MARGIN).min(west.len());

    let mut net = Net { nodes: Vec::new(), links: Vec::new() };
    let _ = writeln!(report, "## Réseau\n\nChaussées OSM : vers Lyon {:.0} m retenus sur {:.0} m, vers Saint-Étienne {:.0} m sur {:.0} m.", e1 - e0, east.len(), w1 - w0, west.len());
    // Mainline station chainages per carriageway (to give each its own link).
    let station_ch = |cw: &Carriageway, dest: &str| -> Vec<f64> {
        cps.iter()
            .filter(|c| c.road == "A0047" && c.destination == dest && has_data(c))
            .map(|c| osm::project(&cw.pts, c.pos))
            .filter(|(_, off)| *off < STATION_OFFSET)
            .map(|(ch, _)| ch)
            .collect()
    };
    let main_e = build_direction(&osm, &mut net, Dir::Lyon, &east, e0, e1, &station_ch(&east, "Vers Lyon"), &mut report);
    let main_w = build_direction(&osm, &mut net, Dir::StEtienne, &west, w0, w1, &station_ch(&west, "Vers St-Etienne"), &mut report);
    // The Lyon carriageway ends at the A7 junction: its last node splits.
    if let Some(&last) = main_e.last() {
        let d = net.links[last].down;
        net.nodes[d].label = "bifurcation A7 / A46".into();
    }

    // Stations on the network.
    let mut stations: Vec<Station> = Vec::new();
    for (k, cp) in cps.iter().enumerate() {
        if !has_data(cp) {
            continue;
        }
        let mainline_dir = match (cp.road.as_str(), cp.destination.as_str()) {
            ("A0047", "Vers Lyon") | ("A0047", "Vers A7") => Some(Dir::Lyon),
            ("A0047", "Vers St-Etienne") => Some(Dir::StEtienne),
            _ => None,
        };
        let candidates: Vec<usize> = (0..net.links.len())
            .filter(|&l| match mainline_dir {
                Some(d) => net.links[l].kind == Kind::Mainline && net.links[l].dir == d,
                None => net.links[l].kind != Kind::Mainline,
            })
            .collect();
        let best = candidates
            .iter()
            .map(|&l| (l, osm::project(&net.links[l].pts, cp.pos)))
            .min_by(|a, b| a.1.1.total_cmp(&b.1.1));
        // Same margin for ramps: a wider one let MYR69.73, next to the Lyon
        // station 72, snap onto a Saint-Étienne ramp 43 m away.
        let limit = STATION_OFFSET;
        match best {
            Some((l, (ch, off))) if off < limit => {
                stations.push(Station { cp: k, link: l, dir: mainline_dir, chainage: net.links[l].ch0 + ch, offset: off })
            }
            _ => {
                let _ = writeln!(
                    report,
                    "\nStation {} ({}, {:.5}, {:.5}) écartée : à {:.0} m du réseau (lien le plus proche : {}).",
                    cp.name,
                    cp.id,
                    cp.pos.0,
                    cp.pos.1,
                    best.map_or(f64::NAN, |b| b.1.1),
                    best.map_or(usize::MAX, |b| b.0)
                );
            }
        }
    }

    // Free-flow speeds: from the nearest station of the same direction and
    // speed limit; else the limit plus 5 %.
    for l in 0..net.links.len() {
        if net.links[l].kind != Kind::Mainline {
            continue;
        }
        let (d, lim, mid) = (net.links[l].dir, net.links[l].limit_kmh, (net.links[l].ch0 + net.links[l].ch1) / 2.0);
        // The link's own station, else the nearest with the same speed limit
        // (screened-out stations included: their speeds are sound).
        let near = stations
            .iter()
            .filter(|s| s.dir == Some(d) && (s.link == l || net.links[s.link].limit_kmh == lim))
            .filter_map(|s| {
                let rank = if s.link == l { 0.0 } else { 1.0 + (s.chainage - mid).abs() };
                free_speed(&meas[&cps[s.cp].id]).map(|u| (rank, u))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0));
        net.links[l].fd_u = near.map_or(lim * 1.05 / 3.6, |x| x.1);
    }

    // ── Screening ─────────────────────────────────────────────────────────
    let excluded = screen_stations(&net, &stations, &cps, &meas, &mut report);
    let _ = writeln!(report, "\n### Dépistage des fenêtres\n");
    let bad_windows = screen_windows(&stations, &cps, &meas, &excluded, &mut report);
    if bad_windows.is_empty() {
        let _ = writeln!(report, "Aucune fenêtre écartée.");
    }
    let is_bad = |id: u32, t: f64| bad_windows.iter().any(|(i, w)| *i == id && (*w - t).abs() < 1.0);

    // ── Observations ──────────────────────────────────────────────────────
    let mut observed = Vec::new();
    let mut n_windows = (0usize, 0usize);
    let _ = writeln!(report, "\n## Stations\n\n| Station | AVATAR | Sens | Lien | Écart au tracé | Fenêtres mesurées |\n|---|---|---|---|---|---|");
    // A ramp with several stations is validated against the one that drives
    // its demand (most measured samples); the others are cross-checks.
    let measured_count = |s: &Station| meas[&cps[s.cp].id].iter().filter(|x| x.flow_measured).count();
    let primary = |s: &Station| {
        s.dir.is_some()
            || stations
                .iter()
                .filter(|o| o.link == s.link && o.dir.is_none())
                .max_by_key(|o| measured_count(o))
                .is_some_and(|o| o.cp == s.cp)
    };
    for s in &stations {
        let cp = &cps[s.cp];
        let samples = &meas[&cp.id];
        if !primary(s) {
            let main = stations.iter().filter(|o| o.link == s.link && primary(o)).map(|o| &cps[o.cp]).next();
            let mean = |c: &CountPoint| {
                let v: Vec<f64> = meas[&c.id].iter().filter(|x| x.flow_measured).filter_map(|x| x.flow).collect();
                v.iter().sum::<f64>() / v.len().max(1) as f64
            };
            if let Some(m) = main {
                let _ = writeln!(
                    report,
                    "| {} | {} | bretelle (contrôle croisé) | {} | {:.0} m | écartée : {:.0} véh/h contre {:.0} pour {} sur la même bretelle |",
                    cp.name, cp.id, s.link, s.offset, mean(cp), mean(m), m.name
                );
            }
            continue;
        }
        let st = avatar::step(samples);
        let win = if st >= 3600.0 { 3600.0 } else { WINDOW };
        let (mut kept, mut total) = (0, 0);
        let mut t = START;
        while t + win <= END + 1e-6 {
            total += 1;
            // Data that failed the screening does not judge the model.
            if is_bad(cp.id, t) || excluded.contains(&cp.id) {
                t += win;
                continue;
            }
            if let Some((q, measured, expected)) = avatar::flow_over(samples, t, t + win, true) {
                if measured == expected {
                    let speeds: Vec<f64> = avatar::in_window(samples, t, t + win)
                        .iter()
                        .filter(|x| x.speed_measured)
                        .filter_map(|x| x.speed)
                        .collect();
                    let mut o = json!({
                        "link_id": s.link, "t_start": t, "t_end": t + win,
                        "flow_veh_h": (q * 10.0).round() / 10.0,
                        "label": format!("{} {}", cp.name, s.dir.map_or("bretelle", |d| d.label())),
                    });
                    if speeds.len() == expected && st < 3600.0 {
                        o["speed_kmh"] = json!((speeds.iter().sum::<f64>() / speeds.len() as f64 * 10.0).round() / 10.0);
                    }
                    observed.push(o);
                    kept += 1;
                }
            }
            t += win;
        }
        n_windows.0 += kept;
        n_windows.1 += total;
        let _ = writeln!(
            report,
            "| {} | {} | {} | {} | {:.0} m | {kept}/{total} |",
            cp.name,
            cp.id,
            s.dir.map_or("bretelle", |d| d.label()),
            s.link,
            s.offset
        );
    }
    let _ = writeln!(report, "\nFenêtres d'observation retenues (entièrement mesurées) : {}/{}. Les valeurs imputées par AVATAR sont exclues.", n_windows.0, n_windows.1);

    // ── Demand ────────────────────────────────────────────────────────────
    let mut demand = Vec::new();
    let _ = writeln!(report, "\n## Demande\n\nFenêtres de {} min. Débits de section courante : stations ; bretelles mesurées : stations (pas horaire) ; bretelles non mesurées : conservation entre deux stations quand une seule est inconnue, sinon 0 (signalé).\n", WINDOW / 60.0);
    let mut ramp_source: BTreeMap<usize, (String, f64, usize)> = BTreeMap::new();
    let mut residuals: Vec<f64> = Vec::new();
    // Flow change between consecutive stations, per pair (screening).
    let mut pair_flows: BTreeMap<(&str, i64, i64), Vec<f64>> = BTreeMap::new();
    for (d, main) in [(Dir::Lyon, &main_e), (Dir::StEtienne, &main_w)] {
        let origin_main = net.links[main[0]].up;
        let ramps: Vec<usize> =
            (0..net.links.len()).filter(|&l| net.links[l].dir == d && net.links[l].kind != Kind::Mainline).collect();
        let dir_stations: Vec<&Station> = {
            let mut v: Vec<&Station> = stations.iter().filter(|s| s.dir == Some(d)).collect();
            v.sort_by(|a, b| a.chainage.total_cmp(&b.chainage));
            v
        };
        // The ramp's best station: the one with the most measured samples.
        let ramp_station = |r: usize| {
            stations
                .iter()
                .filter(|s| s.link == r && s.dir.is_none())
                .max_by_key(|s| meas[&cps[s.cp].id].iter().filter(|x| x.flow_measured).count())
        };
        // Final exits: the downstream end of the mainline (and its split branches).
        let last = *main.last().unwrap();
        let end_node = net.links[last].down;
        let end_exits: Vec<usize> = ramps.iter().copied().filter(|&r| net.links[r].up == end_node).collect();
        let mut cum: HashMap<(usize, usize), (f64, f64)> = HashMap::new(); // OD -> (cumulated veh, emitted)
        let mut t = START;
        while t + WINDOW <= END + 1e-6 {
            // Measured values only: AVATAR's imputed values never enter the demand.
            let flows: Vec<(f64, f64)> = dir_stations
                .iter()
                .filter(|s| !excluded.contains(&cps[s.cp].id) && !is_bad(cps[s.cp].id, t))
                .filter_map(|s| avatar::flow_over(&meas[&cps[s.cp].id], t, t + WINDOW, true).map(|(q, _, _)| (s.chainage, q)))
                .collect();
            for w in flows.windows(2) {
                pair_flows.entry((d.label(), w[0].0.round() as i64, w[1].0.round() as i64)).or_default().push(w[1].1 - w[0].1);
            }
            if flows.is_empty() {
                t += WINDOW;
                continue;
            }
            // Ramp flows: measured, then resolved by conservation.
            let mut q_ramp: HashMap<usize, f64> = HashMap::new();
            let mut src: HashMap<usize, &str> = HashMap::new();
            for &r in &ramps {
                if end_exits.contains(&r) {
                    continue;
                }
                if let Some(s) = ramp_station(r)
                    && let Some((q, _, _)) = avatar::flow_over(&meas[&cps[s.cp].id], t, t + WINDOW, true)
                {
                    q_ramp.insert(r, q);
                    src.insert(r, "mesurée");
                }
            }
            let segment = |a: f64, b: f64| -> Vec<usize> {
                ramps.iter().copied().filter(|&r| !end_exits.contains(&r) && net.links[r].ch0 > a && net.links[r].ch0 <= b).collect()
            };
            for w in flows.windows(2) {
                let ((ca, qa), (cb, qb)) = (w[0], w[1]);
                let seg = segment(ca, cb);
                let mut r = qb - qa;
                for &x in &seg {
                    if let Some(q) = q_ramp.get(&x) {
                        r += if net.links[x].kind == Kind::OffRamp { *q } else { -*q };
                    }
                }
                let unknown: Vec<usize> = seg.iter().copied().filter(|x| !q_ramp.contains_key(x)).collect();
                if unknown.is_empty() {
                    residuals.push(r);
                    continue;
                }
                let ons: Vec<usize> = unknown.iter().copied().filter(|&x| net.links[x].kind == Kind::OnRamp).collect();
                let offs: Vec<usize> = unknown.iter().copied().filter(|&x| net.links[x].kind == Kind::OffRamp).collect();
                for &x in &unknown {
                    let v = match net.links[x].kind {
                        Kind::OnRamp if r > 0.0 => r / ons.len() as f64,
                        Kind::OffRamp if r < 0.0 => -r / offs.len() as f64,
                        _ => 0.0,
                    };
                    q_ramp.insert(x, v);
                    src.insert(x, if unknown.len() == 1 { "conservation" } else { "conservation (partagée)" });
                }
            }
            for &x in &ramps {
                if !end_exits.contains(&x) && !q_ramp.contains_key(&x) {
                    q_ramp.insert(x, 0.0);
                    src.insert(x, "non observée : 0");
                }
            }
            // Boundary entry: the first station's flow, corrected for the ramps before it.
            let (c_first, q_first) = flows[0];
            let mut q_entry = q_first;
            for x in segment(f64::NEG_INFINITY, c_first) {
                q_entry += if net.links[x].kind == Kind::OffRamp { q_ramp[&x] } else { -q_ramp[&x] };
            }
            let q_entry = q_entry.max(0.0);

            // Origins and the events met downstream, in chainage order.
            let mut events: Vec<(f64, usize)> = ramps.iter().copied().filter(|r| !end_exits.contains(r)).map(|r| (net.links[r].ch0, r)).collect();
            events.sort_by(|a, b| a.0.total_cmp(&b.0));
            // (origin node, flow, share still on the mainline)
            let mut on: Vec<(usize, f64, f64)> = vec![(origin_main, q_entry, 1.0)];
            let mut od: Vec<((usize, usize), f64)> = Vec::new();
            for (_, r) in &events {
                let q = q_ramp[r];
                match net.links[*r].kind {
                    Kind::OnRamp => on.push((net.links[*r].up, q, 1.0)),
                    Kind::OffRamp => {
                        let main_q: f64 = on.iter().map(|o| o.1 * o.2).sum();
                        let p = if main_q > 0.0 { (q / main_q).min(MAX_EXIT_SHARE) } else { 0.0 };
                        for o in on.iter_mut() {
                            od.push(((o.0, net.links[*r].down), o.1 * o.2 * p));
                            o.2 *= 1.0 - p;
                        }
                    }
                    Kind::Mainline => {}
                }
            }
            // What remains leaves at the end: through the split branches by
            // their lanes, else at the end node.
            let total_lanes: f64 = end_exits.iter().map(|&x| f64::from(net.links[x].lanes)).sum();
            for o in &on {
                if end_exits.is_empty() {
                    od.push(((o.0, end_node), o.1 * o.2));
                } else {
                    for &x in &end_exits {
                        od.push(((o.0, net.links[x].down), o.1 * o.2 * f64::from(net.links[x].lanes) / total_lanes));
                    }
                }
            }
            for &x in &end_exits {
                src.insert(x, "répartition par voies");
            }
            for (k, v) in &src {
                let e = ramp_source.entry(*k).or_insert((v.to_string(), 0.0, 0));
                if e.0 != *v && !e.0.contains(v) {
                    e.0 = format!("{} / {v}", e.0);
                }
                e.1 += q_ramp.get(k).copied().unwrap_or(0.0);
                e.2 += 1;
            }
            // Cumulative rounding: no vehicle lost to truncation.
            let mut merged: BTreeMap<(usize, usize), f64> = BTreeMap::new();
            for (k, q) in od {
                *merged.entry(k).or_default() += q;
            }
            for ((o, dd), q) in merged {
                let veh = q * WINDOW / 3600.0;
                let e = cum.entry((o, dd)).or_insert((0.0, 0.0));
                e.0 += veh;
                let count = (e.0.round() - e.1).max(0.0);
                e.1 += count;
                if count > 0.0 {
                    demand.push(json!({
                        "period_start": t, "period_end": t + WINDOW,
                        "origin": o, "destination": dd,
                        "flow": count * 3600.0 / WINDOW, "count": count,
                    }));
                }
            }
            t += WINDOW;
        }
    }
    let _ = writeln!(report, "| Bretelle | Libellé | Débit moyen | Source |\n|---|---|---|---|");
    for (r, (s, sum, n)) in &ramp_source {
        let _ = writeln!(report, "| {r} | {} | {:.0} véh/h | {s} |", net.links[*r].label, sum / (*n).max(1) as f64);
    }
    if !residuals.is_empty() {
        let mean_abs = residuals.iter().map(|r| r.abs()).sum::<f64>() / residuals.len() as f64;
        let _ = writeln!(report, "\nContrôle de conservation entre stations sans bretelle inconnue : écart moyen absolu {mean_abs:.0} véh/h sur {} fenêtres.", residuals.len());
    }
    let _ = writeln!(report, "\n### Contrôle station par station\n\nVariation de débit entre deux stations consécutives (aval − amont), moyenne sur les fenêtres où les deux sont mesurées.\n\n| Sens | Amont | Aval | Bretelles entre | Δ moyen | |Δ| moyen |\n|---|---|---|---|---|---|");
    for ((d, a, b), v) in &pair_flows {
        let name = |c: i64| {
            stations
                .iter()
                .filter(|s| s.dir.is_some_and(|x| x.label() == *d) && (s.chainage.round() as i64) == c)
                .map(|s| cps[s.cp].name.trim_start_matches("MYR69.").to_string())
                .collect::<Vec<_>>()
                .join("/")
        };
        let between = (0..net.links.len())
            .filter(|&l| net.links[l].kind != Kind::Mainline && net.links[l].dir.label() == *d)
            .filter(|&l| net.links[l].ch0 > *a as f64 && net.links[l].ch0 <= *b as f64)
            .map(|l| l.to_string())
            .collect::<Vec<_>>()
            .join(", ");
        let mean = v.iter().sum::<f64>() / v.len() as f64;
        let mean_abs = v.iter().map(|x| x.abs()).sum::<f64>() / v.len() as f64;
        let _ = writeln!(report, "| {d} | {} | {} | {} | {mean:+.0} | {mean_abs:.0} |", name(*a), name(*b), if between.is_empty() { "—".into() } else { between });
    }
    let vehicles: f64 = demand.iter().map(|d| d["count"].as_f64().unwrap_or(0.0)).sum();
    let _ = writeln!(report, "\nDemande : {} lignes, {vehicles:.0} véhicules.", demand.len());

    // ── Downstream boundary: the A7 junction ──────────────────────────────
    // The queue that fills the Lyon carriageway in the morning starts beyond
    // the network, at the A7 / A46 junction. While the last Lyon station is in
    // that queue, its measured flow is the junction's discharge: the last
    // Lyon link gets that capacity (patch `gate`).
    let last_lyon = *main_e.last().unwrap();
    let gate_station = stations
        .iter()
        .filter(|s| s.dir == Some(Dir::Lyon) && !excluded.contains(&cps[s.cp].id))
        .filter(|s| meas[&cps[s.cp].id].iter().any(|x| x.flow_measured))
        .max_by(|a, b| a.chainage.total_cmp(&b.chainage));
    let mut ops = Vec::new();
    if let Some(gs) = gate_station {
        let lanes = f64::from(net.links[last_lyon].lanes);
        let samples = &meas[&cps[gs.cp].id];
        let step = avatar::step(samples);
        for x in samples.iter().filter(|x| x.t >= START && x.t < END) {
            if let (true, true, Some(v), Some(q)) = (x.speed_measured, x.flow_measured, x.speed, x.flow)
                && v < CONGESTED_KMH
            {
                // Every pipe is spilled in the queue, so friction throttles each
                // to C·φ: divide by φ for the link to discharge what was measured.
                let c = q / 3600.0 / lanes / FRICTION;
                ops.push(json!({"op": "set_link", "link": last_lyon, "from": x.t, "until": x.t + step,
                                "set": {"fd_c": (c * 1e4).round() / 1e4}}));
            }
        }
        let _ = writeln!(
            report,
            "\n## Condition aval (patch `gate`)\n\nLa station {} est congestionnée (< {CONGESTED_KMH} km/h) sur {} pas de 6 min : pendant ces pas, le lien {last_lyon} (fin de l'A47 vers Lyon) prend pour capacité le débit qu'elle mesure. Sans ce patch, le goulot de la jonction A7, hors du réseau, n'existe pas.",
            cps[gs.cp].name,
            ops.len()
        );
    }
    std::fs::create_dir_all(dir.join("patches")).expect("patches dir");
    write_json(
        &dir.join("patches").join("gate.json"),
        &json!({"name": "gate", "description": format!(
            "Downstream boundary at the A7 / A46 junction: while station {} (last on the Lyon carriageway) is congested, the end link discharges at the flow it measures. Built from Cerema AVATAR, {DAY}.",
            gate_station.map_or("?".to_string(), |s| cps[s.cp].name.clone())), "ops": ops}),
    );

    // ── Write ─────────────────────────────────────────────────────────────
    let mut features = Vec::new();
    for (i, n) in net.nodes.iter().enumerate() {
        let t = match n.kind {
            NodeKind::Entry => "Entry",
            NodeKind::Exit => "Exit",
            NodeKind::Internal => "Internal",
        };
        let mut props = json!({"id": i, "type": t});
        if !n.label.is_empty() {
            props["label"] = json!(n.label);
        }
        features.push(json!({"type": "Feature", "geometry": {"type": "Point", "coordinates": round7(n.pos)}, "properties": props}));
    }
    // The final node of the Lyon carriageway only splits: an Internal node.
    let _ = writeln!(report, "\n## Liens\n\n| Lien | Sens | Type | Longueur | Voies | Limite | u (km/h) |\n|---|---|---|---|---|---|---|");
    for (i, l) in net.links.iter().enumerate() {
        let length = osm::length(&l.pts);
        let lanes = f64::from(l.lanes);
        let c = CAPACITY_PER_LANE;
        let u = l.fd_u;
        let w = c / (JAM_DENSITY_PER_LANE - c / u);
        let mut props = json!({
            "id": i, "node_up": l.up, "node_down": l.down,
            "length": (length * 10.0).round() / 10.0,
            "speed": (u * 1000.0).round() / 1000.0,
            "lanes": l.lanes,
            "capacity": c * lanes,
            "fd_u": (u * 1000.0).round() / 1000.0,
            "fd_w": (w * 1000.0).round() / 1000.0,
            "fd_kx": JAM_DENSITY_PER_LANE,
            "fd_c": c,
        });
        if l.lanes >= 2 {
            props["pipes"] = json!(vec![json!({"lanes": 1}); usize::from(l.lanes)]);
            props["friction"] = json!(FRICTION);
        }
        if !l.moves.is_empty() {
            let mut m = serde_json::Map::new();
            for (to, pipes) in &l.moves {
                m.insert(to.to_string(), json!(pipes));
            }
            props["moves"] = Value::Object(m);
        }
        props["kind"] = json!(match l.kind {
            Kind::Mainline => "mainline",
            Kind::OffRamp => "off-ramp",
            Kind::OnRamp => "on-ramp",
        });
        props["direction"] = json!(l.dir.label());
        props["label"] = json!(l.label);
        props["speed_limit_kmh"] = json!(l.limit_kmh);
        props["osm_ways"] = json!(l.osm_ways);
        let coords: Vec<Value> = l.pts.iter().map(|p| round7(*p)).collect();
        features.push(json!({"type": "Feature", "geometry": {"type": "LineString", "coordinates": coords}, "properties": props}));
        let _ = writeln!(
            report,
            "| {i} | {} | {:?} | {length:.0} m | {} | {:.0} | {:.0} |",
            l.dir.label(),
            l.kind,
            l.lanes,
            l.limit_kmh,
            u * 3.6
        );
    }
    write_json(&dir.join("network.geojson"), &json!({"type": "FeatureCollection", "features": features}));
    write_json(&dir.join("demand.json"), &Value::Array(demand));
    let n_observed = observed.len();
    write_json(&dir.join("observed.json"), &Value::Array(observed));
    // Capacity-proportional merge: the Givors-Montrond on-ramp forces its way
    // into the morning queue, which in-order service cannot reproduce.
    write_json(&dir.join("config.json"), &json!({"start_time": START, "duration": END - START, "classes": ["car"], "merge": "capacity"}));
    let st_features: Vec<Value> = stations
        .iter()
        .map(|s| {
            let cp = &cps[s.cp];
            json!({"type": "Feature", "geometry": {"type": "Point", "coordinates": round7(cp.pos)},
                   "properties": {"avatar_id": cp.id, "name": cp.name, "road": cp.road, "destination": cp.destination,
                                   "link_id": s.link, "offset_m": (s.offset * 10.0).round() / 10.0}})
        })
        .collect();
    write_json(&dir.join("stations.geojson"), &json!({"type": "FeatureCollection", "features": st_features}));
    std::fs::write(dir.join("BUILD_REPORT.md"), report).expect("write report");
    println!(
        "{} nodes, {} links, {} stations, {} observations, {vehicles:.0} vehicles",
        net.nodes.len(),
        net.links.len(),
        stations.len(),
        n_observed
    );
}
