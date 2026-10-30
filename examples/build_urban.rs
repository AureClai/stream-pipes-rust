//! Build `scenarios/urban`: a synthetic town that exercises every node
//! mechanism at once.
//!
//! A 3 × 3 grid of intersections 300 m apart (local metric coordinates,
//! origin far from (0, 0) so they are never read as degrees). The
//! east–west boulevard crosses the middle row; the north–south avenue the
//! middle column. Grid node G(i, j) sits at (1000 + 300 i, 1000 + 300 j):
//!
//! ```text
//!            north (toll)
//!   G6 ─────── G7 ─────── G8        G6, G0, G2: priority, yield
//!   │          │          │         G7: priority, yield (avenue major)
//!   G3 ═══════ G4 ═══════ G5 ══ M ══ east     G3: roundabout
//!   (round-    (signals:   (signals,   M: on-ramp merge, ramp metered
//!    about)     3 phases)   green wave)  G8: ordinary node (no rules)
//!   │          │          │         G1: priority, stop (avenue major)
//!   G0 ─────── G1 ─────── G2
//! ```
//!
//! - boulevard: a bus lane plus a general lane each way, a left-turn
//!   pocket on the approaches of G4 (pipes + moves), 50 km/h;
//! - G4: three phases (boulevard through and right, protected boulevard
//!   lefts, avenue with permitted lefts), plans changing at 30 min;
//! - G5: two phases, offset for a green wave from G4, permitted lefts;
//! - on-ramp R → M metered (one vehicle per green) between 15 and 45 min;
//! - a two-lane toll on the northern entry of the avenue;
//! - buses along the boulevard (class "bus"), the PT step to come.
//!
//!   cargo run --release --no-default-features --example build_urban

use serde_json::{json, Value};
use std::collections::BTreeMap;

type P = (f64, f64);

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Boulevard,
    Avenue,
    Street,
}

struct Net {
    nodes: Vec<Value>,
    links: Vec<Value>,
    pos: Vec<P>,
    /// (from node, to node) → link id
    link_of: BTreeMap<(usize, usize), usize>,
}

impl Net {
    fn node(&mut self, p: P, props: Value) -> usize {
        let id = self.nodes.len();
        let mut props = props;
        props["id"] = json!(id);
        self.nodes.push(json!({
            "type": "Feature",
            "geometry": { "type": "Point", "coordinates": [p.0, p.1] },
            "properties": props,
        }));
        self.pos.push(p);
        id
    }

    fn link(&mut self, a: usize, b: usize, kind: Kind, lanes: u8, extra: Value) -> usize {
        let id = self.links.len();
        let (pa, pb) = (self.pos[a], self.pos[b]);
        let length = ((pb.0 - pa.0).powi(2) + (pb.1 - pa.1).powi(2)).sqrt();
        let (speed, c) = match kind {
            Kind::Boulevard => (13.9, 0.5),
            Kind::Avenue => (13.9, 0.5),
            Kind::Street => (8.3, 0.45),
        };
        let mut props = json!({
            "id": id, "node_up": a, "node_down": b, "length": length, "speed": speed,
            "lanes": lanes, "capacity": c * f64::from(lanes), "fd_kx": 0.15, "fd_w": 5.0, "fd_c": c,
        });
        if let Value::Object(m) = extra {
            for (k, v) in m {
                props[k] = v;
            }
        }
        self.links.push(json!({
            "type": "Feature",
            "geometry": { "type": "LineString", "coordinates": [[pa.0, pa.1], [pb.0, pb.1]] },
            "properties": props,
        }));
        self.link_of.insert((a, b), id);
        id
    }

    fn l(&self, a: usize, b: usize) -> usize {
        self.link_of[&(a, b)]
    }

    fn set_link_prop(&mut self, id: usize, key: &str, v: Value) {
        self.links[id]["properties"][key] = v;
    }
}

fn main() -> anyhow::Result<()> {
    let mut net = Net { nodes: vec![], links: vec![], pos: vec![], link_of: BTreeMap::new() };
    let gp = |i: usize, j: usize| (1000.0 + 300.0 * i as f64, 1000.0 + 300.0 * j as f64);

    // Grid nodes 0..9 (id = 3j + i); properties filled in below.
    for j in 0..3 {
        for i in 0..3 {
            net.node(gp(i, j), json!({}));
        }
    }
    let g = |i: usize, j: usize| 3 * j + i;

    // External arms: (grid node, far point, name). The east boulevard arm
    // goes through the merge node M.
    let mut arms: Vec<(usize, P, &str)> = vec![
        (g(0, 0), (700.0, 1000.0), "west_south_street"),
        (g(0, 0), (1000.0, 700.0), "south_west_street"),
        (g(1, 0), (1300.0, 700.0), "avenue_south"),
        (g(2, 0), (1600.0, 700.0), "south_east_street"),
        (g(2, 0), (1900.0, 1000.0), "east_south_street"),
        (g(0, 1), (700.0, 1300.0), "boulevard_west"),
        (g(0, 2), (700.0, 1600.0), "west_north_street"),
        (g(0, 2), (1000.0, 1900.0), "north_west_street"),
        (g(1, 2), (1300.0, 1900.0), "avenue_north"),
        (g(2, 2), (1600.0, 1900.0), "north_east_street"),
        (g(2, 2), (1900.0, 1600.0), "east_north_street"),
    ];
    let merge = net.node((1900.0, 1300.0), json!({}));
    arms.push((merge, (2200.0, 1300.0), "boulevard_east"));

    // Entry and exit node per arm, at the far point.
    let mut entry_of: BTreeMap<&str, usize> = BTreeMap::new();
    let mut exit_of: BTreeMap<&str, usize> = BTreeMap::new();
    for &(_, p, name) in &arms {
        entry_of.insert(name, net.node(p, json!({ "type": "Entry", "name": format!("{name} in") })));
        exit_of.insert(name, net.node(p, json!({ "type": "Exit", "name": format!("{name} out") })));
    }
    let ramp = net.node((1850.0, 1150.0), json!({ "type": "Entry", "name": "on-ramp" }));

    // Bus lane + general lane.
    let bus_pipes = json!([ { "lanes": 1, "classes": ["bus"] }, { "lanes": 1 } ]);

    // Grid links, both directions.
    for j in 0..3 {
        for i in 0..3 {
            for (di, dj) in [(1usize, 0usize), (0, 1)] {
                let (i2, j2) = (i + di, j + dj);
                if i2 > 2 || j2 > 2 {
                    continue;
                }
                let horizontal = dj == 0;
                let kind = if horizontal && j == 1 {
                    Kind::Boulevard
                } else if !horizontal && i == 1 {
                    Kind::Avenue
                } else {
                    Kind::Street
                };
                for (a, b) in [(g(i, j), g(i2, j2)), (g(i2, j2), g(i, j))] {
                    if kind == Kind::Boulevard {
                        let into_centre = b == g(1, 1);
                        let lanes = if into_centre { 3 } else { 2 };
                        let pipes = if into_centre {
                            json!([ { "lanes": 1, "classes": ["bus"] }, { "lanes": 1 }, { "lanes": 1 } ])
                        } else {
                            bus_pipes.clone()
                        };
                        net.link(a, b, kind, lanes, json!({ "pipes": pipes }));
                    } else {
                        net.link(a, b, kind, 1, json!({}));
                    }
                }
            }
        }
    }
    // External links.
    for &(gn, _, name) in &arms {
        let kind = match name {
            "boulevard_west" | "boulevard_east" => Kind::Boulevard,
            "avenue_south" | "avenue_north" => Kind::Avenue,
            _ => Kind::Street,
        };
        let (e_in, e_out) = (entry_of[name], exit_of[name]);
        match name {
            "boulevard_west" => {
                net.link(e_in, gn, kind, 2, json!({ "pipes": bus_pipes }));
                net.link(gn, e_out, kind, 2, json!({ "pipes": bus_pipes }));
            }
            "avenue_north" => {
                // Two-lane toll plaza approach.
                net.link(e_in, gn, kind, 2, json!({}));
                net.link(gn, e_out, kind, 1, json!({}));
            }
            "boulevard_east" => {
                net.link(e_in, gn, kind, 2, json!({}));
                net.link(gn, e_out, kind, 2, json!({}));
            }
            _ => {
                net.link(e_in, gn, kind, 1, json!({}));
                net.link(gn, e_out, kind, 1, json!({}));
            }
        }
    }
    // East boulevard between G5 and M, and the on-ramp.
    net.link(g(2, 1), merge, Kind::Boulevard, 2, json!({}));
    net.link(merge, g(2, 1), Kind::Boulevard, 2, json!({}));
    let ramp_link = net.link(ramp, merge, Kind::Street, 1, json!({}));

    // Movement lanes on the approaches of the centre G4: bus lane for the
    // through buses, general lane for through and right, pocket for the left.
    let c = g(1, 1);
    for (from, through, right, left) in [
        (g(0, 1), g(2, 1), g(1, 0), g(1, 2)), // eastbound
        (g(2, 1), g(0, 1), g(1, 2), g(1, 0)), // westbound
    ] {
        let approach = net.l(from, c);
        let mut moves = serde_json::Map::new();
        moves.insert(net.l(c, through).to_string(), json!([0, 1]));
        moves.insert(net.l(c, right).to_string(), json!([1]));
        moves.insert(net.l(c, left).to_string(), json!([2]));
        moves.insert(net.l(c, from).to_string(), json!([2]));
        net.set_link_prop(approach, "moves", Value::Object(moves));
    }

    // Node rules.
    let set = |net: &mut Net, n: usize, key: &str, v: Value| net.nodes[n]["properties"][key] = v;
    let both = |net: &Net, a: usize, b: usize| vec![net.l(a, b), net.l(b, a)];
    // G3 roundabout.
    set(&mut net, g(0, 1), "type", json!("Roundabout"));
    set(&mut net, g(0, 1), "roundabout", json!({ "diameter": 30.0 }));
    // G1: avenue major, stop on the street.
    let mut major = both(&net, g(1, 0), g(1, 1));
    major.extend([net.l(entry_of["avenue_south"], g(1, 0)), net.l(g(1, 0), exit_of["avenue_south"])]);
    set(&mut net, g(1, 0), "junction", json!({ "major": major, "control": "stop" }));
    // G7: avenue major, yield.
    let mut major = both(&net, g(1, 2), g(1, 1));
    major.extend([net.l(entry_of["avenue_north"], g(1, 2)), net.l(g(1, 2), exit_of["avenue_north"])]);
    set(&mut net, g(1, 2), "junction", json!({ "major": major }));
    // Corners G0, G2, G6: the grid street along the column is major.
    for (n, other) in [(g(0, 0), g(0, 1)), (g(2, 0), g(2, 1)), (g(0, 2), g(0, 1))] {
        let ext = arms
            .iter()
            .find(|a| a.0 == n && (a.1 .0 - net.pos[n].0).abs() < 1.0)
            .map(|a| a.2)
            .unwrap();
        let mut major = both(&net, n, other);
        major.extend([net.l(entry_of[ext], n), net.l(n, exit_of[ext])]);
        set(&mut net, n, "junction", json!({ "major": major }));
    }
    // G4 and G5: priority rules under the signals (boulevard major), for
    // the permitted turns.
    let blvd = |net: &Net, n: usize, a: usize, b: usize| {
        let mut m = both(net, n, a);
        m.extend(both(net, n, b));
        m
    };
    let m4 = blvd(&net, c, g(0, 1), g(2, 1));
    set(&mut net, c, "junction", json!({ "major": m4 }));
    let m5 = blvd(&net, g(2, 1), g(1, 1), merge);
    set(&mut net, g(2, 1), "junction", json!({ "major": m5 }));

    // ── Signals ────────────────────────────────────────────────────────────
    let (w, e, s, n) = (g(0, 1), g(2, 1), g(1, 0), g(1, 2));
    let mv = |net: &Net, from: usize, to: Option<usize>, node: usize| match to {
        Some(t) => json!({ "from": net.l(from, node), "to": net.l(node, t) }),
        None => json!({ "from": net.l(from, node) }),
    };
    let groups = json!([
        { "id": "centre_boulevard", "node": c, "movements": [
            mv(&net, w, Some(e), c), mv(&net, w, Some(s), c), mv(&net, e, Some(w), c), mv(&net, e, Some(n), c) ] },
        { "id": "centre_left", "node": c, "movements": [
            mv(&net, w, Some(n), c), mv(&net, e, Some(s), c), mv(&net, w, Some(w), c), mv(&net, e, Some(e), c) ] },
        { "id": "centre_avenue", "node": c, "movements": [ mv(&net, s, None, c), mv(&net, n, None, c) ] },
        { "id": "east_boulevard", "node": e, "movements": [ mv(&net, c, None, e), mv(&net, merge, None, e) ] },
        { "id": "east_cross", "node": e, "movements": [ mv(&net, g(2, 0), None, e), mv(&net, g(2, 2), None, e) ] },
        { "id": "ramp", "node": merge, "movements": [ { "from": ramp_link } ], "vehicles_per_green": 1 }
    ]);
    let plans = json!([
        { "name": "first half hour", "from": 0, "until": 1800, "cycle": 80, "offset": 0,
          "greens": { "centre_boulevard": [[0, 32]], "centre_left": [[35, 45]], "centre_avenue": [[48, 77]] } },
        { "name": "second half hour", "from": 1800, "until": 3600, "cycle": 100, "offset": 0,
          "greens": { "centre_boulevard": [[0, 42]], "centre_left": [[45, 57]], "centre_avenue": [[60, 97]] } },
        { "name": "east, first half hour", "from": 0, "until": 1800, "cycle": 80, "offset": 22,
          "greens": { "east_boulevard": [[0, 45]], "east_cross": [[48, 77]] } },
        { "name": "east, second half hour", "from": 1800, "until": 3600, "cycle": 100, "offset": 22,
          "greens": { "east_boulevard": [[0, 58]], "east_cross": [[61, 97]] } },
        { "name": "ramp metering", "from": 900, "until": 2700, "cycle": 4, "greens": { "ramp": [[0, 1]] } }
    ]);
    let tolls = json!([
        { "link": net.l(entry_of["avenue_north"], g(1, 2)), "service_time": 6.0, "by_class": { "bus": 2.0 } }
    ]);
    let signals = json!({ "groups": groups, "plans": plans, "tolls": tolls });

    // ── Demand ─────────────────────────────────────────────────────────────
    let names: Vec<&str> = arms.iter().map(|a| a.2).collect();
    let mut demand = Vec::new();
    let mut od = |o: &str, d: &str, q: f64, class: Option<&str>| {
        let mut e = json!({
            "period_start": 0.0, "period_end": 3600.0,
            "origin": entry_of[o], "destination": exit_of[d], "flow": q, "count": q,
        });
        if let Some(c) = class {
            e["class"] = json!(c);
        }
        demand.push(e);
    };
    od("boulevard_west", "boulevard_east", 420.0, None);
    od("boulevard_east", "boulevard_west", 420.0, None);
    od("avenue_south", "avenue_north", 240.0, None);
    od("avenue_north", "avenue_south", 240.0, None);
    // Every other pair, lightly: all movements of all nodes get traffic.
    for &o in &names {
        for &d in &names {
            if o != d {
                od(o, d, 8.0, None);
            }
        }
    }
    // Buses along the boulevard (a line every 5 minutes each way).
    od("boulevard_west", "boulevard_east", 12.0, Some("bus"));
    od("boulevard_east", "boulevard_west", 12.0, Some("bus"));
    // On-ramp traffic joins the eastbound boulevard.
    demand.push(json!({
        "period_start": 0.0, "period_end": 3600.0,
        "origin": ramp, "destination": exit_of["boulevard_east"], "flow": 380.0, "count": 380.0,
    }));

    // ── Write ──────────────────────────────────────────────────────────────
    let mut features = net.nodes.clone();
    features.extend(net.links.clone());
    let network = json!({ "type": "FeatureCollection", "features": features });
    let config = json!({ "start_time": 0.0, "duration": 3600.0, "classes": ["car", "bus"] });
    let dir = std::path::Path::new("scenarios/urban");
    std::fs::create_dir_all(dir)?;
    let write = |name: &str, v: &Value| -> anyhow::Result<()> {
        std::fs::write(dir.join(name), serde_json::to_string_pretty(v)? + "\n")?;
        Ok(())
    };
    write("network.geojson", &network)?;
    write("demand.json", &Value::Array(demand))?;
    write("config.json", &config)?;
    write("signals.json", &signals)?;
    println!(
        "scenarios/urban: {} nodes, {} links, {} demand entries",
        net.nodes.len(),
        net.links.len(),
        names.len() * (names.len() - 1) + 7
    );
    Ok(())
}
