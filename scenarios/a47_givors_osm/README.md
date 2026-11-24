# A47 Givors — built from OpenStreetMap and Cerema AVATAR

The A47 motorway through Givors (Rhône, France), both directions, from
Saint-Romain-en-Gier (stations 91/92) to the A7 / A46 junction, on Thursday
**11 June 2026, 05:00–22:00**. Every interchange is where it is on the ground,
every link is as long as its drawn line, and every number the model is judged
on was measured, not imputed.

It replaces nothing: `../a47_givors` is the scenario of the second preprint
and stays as published.

## What is in it

| | |
|---|---|
| Network | 10.2 km per direction, 55 nodes, 53 links: 35 mainline links and 18 ramps |
| Interchanges | Givors-Ouest, Givors-Montrond, Givors-Centre, Les Plaines / Grigny, Chasse-sur-Rhône, A7 / A46 split |
| Lanes | from OSM `lanes`, one pipe per lane; the 3-lane stretches are the auxiliary lanes (`turn:lanes` *slight_right* / *merge_to_left*) |
| Stations | 25 AVATAR count points on the network (13 mainline at 6 min, ramps hourly) |
| Demand | 88 611 vehicles, 30-min OD flows estimated from the counts |
| Patch `gate` | the A7 junction's measured morning discharge, as a downstream boundary |

## Build

```sh
cargo run --release --manifest-path scenarios/a47_givors_osm/build/Cargo.toml
```

Offline and deterministic: it reads `raw/` and writes `network.geojson`,
`demand.json`, `config.json`, `observed.json`, `stations.geojson`,
`patches/gate.json` and [`BUILD_REPORT.md`](BUILD_REPORT.md), which lists every
choice it made (ramps, stations, sources of each ramp flow, screening). The raw
data were fetched with:

```sh
# OpenStreetMap: motorways and ramps (Overpass API)
[out:json];way["highway"~"^(motorway|motorway_link|trunk|trunk_link|primary_link|secondary_link|tertiary_link)$"](45.540,4.670,45.605,4.810);(._;>;);out body;
# Cerema AVATAR: count points and corrected 6-min measures (no account needed)
https://avatar.cerema.fr/api/countpoints/download?count_point_ids=<ids>
https://avatar.cerema.fr/api/fixed_measures/download?count_point_ids=<ids>&start_time=2026-06-11T00:00:00&end_time=2026-06-12T00:00:00&time_zone=Europe/Paris
```

## Run

```sh
cargo run --release --no-default-features --example scenario_report -- scenarios/a47_givors_osm gate
```

## Modelling choices

- **Network.** The two A47 carriageways are walked in OSM; links break at every
  ramp junction, every lane-count change, and halfway between two stations
  that nothing else separates (so each station has its own link). Ramps follow
  their OSM ways from the junction until they branch or meet another road.
- **Fundamental diagram.** Triangular, per lane: capacity 0.5 veh/s
  (1 800 veh/h), jam density 0.13 veh/m, backward wave set by the two
  (`w = c / (kx − c/u)`). Free-flow speed `u` is the median measured daytime
  speed of the link's station (congested samples left out), else of the
  nearest station under the same speed limit; ramps take their limit.
- **Merges.** Capacity-proportional (see *Results*).
- **Lanes and movements.** At a diverge the exit is served by the right-hand
  pipes; where the mainline loses a lane there (a deceleration lane), that
  lane leads only to the exit. At the A7 / A46 split, the right pipe leads to
  A7 south and the two left ones to A46 / A7 north, as OSM's lanes do.
  Friction φ = 0.95.
- **Demand.** Per 30-min window and direction: mainline flows from the
  stations, ramp flows from their own station when it measured the window,
  else by conservation between the two stations around the ramp when it is the
  only unknown, else 0 (flagged). Origins are the upstream end and the
  on-ramps; each off-ramp takes the same share of every vehicle passing it;
  what remains leaves at the end (split by lanes at the A7 / A46 split).
  Vehicles are rounded cumulatively, so none is lost to truncation.
- **Downstream boundary (`gate`).** The morning queue (06:30–08:30, speeds
  down to 20 km/h from A2 back to 32) starts beyond the network, at the
  A7 junction. While station A2 is in that queue, the last Lyon link gets its
  measured flow as capacity (divided by φ: in a queue every pipe is spilled).

## Data screening

Following the companion paper's protocol, data must pass before they judge the
model. AVATAR marks imputed values (`flow_predicted`): they never enter the
demand or `observed.json`. On this day the mainline stations were 99 %
measured; 92, 81–84, 77, 78, 63, A5 and the two stations near the A7 had no
measured value at all. The build then excludes, and reports:

- **station 31** (vers Saint-Étienne): +300 veh/h from 51, then −413 to 71, with
  no ramp on either side — a counting fault, out of the demand and of the
  validation;
- **station A2, 15:30–17:00**: half its usual flow at 61–66 km/h while 62 just
  upstream flows normally — a lost lane loop, those four windows are out;
- **ramp stations**: one per ramp is used (the most measured); MYR69.A3 on the
  Givors-Centre exit disagrees with 2224 by 20 % and is listed as a cross-check;
  MYR69.73 and .74 lie off the modelled ramps.

## Results (`gate`)

The scenario runs with the capacity-proportional merge (`"merge": "capacity"`
in `config.json`); the engine's historical in-order merge is shown for
comparison (`MERGE=in_order` with `scenario_report`).

| | capacity merge | in-order merge |
|---|---|---|
| Flow windows with GEH < 5 | **444 / 492 (90 %)** | 451 / 492 (92 %) |
| Mean speed error at 32 / 42 / 52 (morning queue) | **17 / 17 / 16 %** | 36 / 35 / 37 % |
| Mean speed error, other stations | 1–12 % | 1–11 % |
| Verification | 164 pass, 4 warn, 0 fail, 11 not exercised | 167 pass, 1 warn, 0 fail |
| Run time | 2.8 M events, 1.0 s | 2.7 M events, 0.8 s |

**The morning queue.** At 06:30–08:30 the Givors-Montrond on-ramp forces about
1 000 veh/h into the queue and the mainline upstream collapses to ~1 100 veh/h
at 20 km/h (stations 42, 52, 32). Served in order, a merge gives the mainline
every slot: the queue stays short, the ramp's vehicles wait off the network
and flood out after 08:30. Shared by capacity (2/3 mainline, 1/3 ramp), the
queue grows back towards 32 as measured, and the speed error there halves.
The ramp's measured share (~45 %) is still above its capacity share.

**The warnings** are all `flow_conservation`, at the nodes the queue covers
(2, 3, 5) and at the A7 split (10). That check compares, per 5-min bin, the
flow *leaving* the incoming links with the flow *leaving* the outgoing ones,
so a queue filling or draining the outgoing link reads as imbalance; at the
node itself conservation is exact (every passage is one instantaneous
transfer).

## Known limits

- **Congested merges.** The capacity-proportional merge brings the morning
  queue back, but the ramp's measured share (~45 %) is above its capacity
  share (1/3), and speeds at 32, 42 and 52 are still 16–17 % off.
- **Unobserved ramps.** Chasse-sur-Rhône (both directions) and the entries
  between A2 and the A7 have no station: their flows are 0. They lie outside
  every observed pair, so the validation does not depend on them.
- **The A7 / A46 split** is by lanes (1/3 south, 2/3 north): no station
  measures it.
- **One class** (cars). AVATAR also counts trucks (`flow_trucks`).

## Licences

- **Network**: derived from OpenStreetMap, © OpenStreetMap contributors,
  under the [Open Database Licence](https://opendatacommons.org/licenses/odbl/)
  (`network.geojson` and `raw/osm.json` included).
- **Traffic data**: Cerema AVATAR, data of the road operators (DIR Centre-Est,
  Métropole de Lyon). The platform states Licence Ouverte 2.0 unless otherwise
  noted; check before redistribution.
