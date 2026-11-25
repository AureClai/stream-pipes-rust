# JSON Format Reference

This document describes all JSON input and output formats used by `stream-cli` (`stream-pipes-rust`).

---

## Overview

| File / Command | Role | Root Type |
|---|---|---|
| `network.geojson` (`build --network`) | Source network topology & link physics | GeoJSON `FeatureCollection` |
| `demand.json` (`build --demand`) | Source OD demand periods | Array of `Demand` |
| `config.json` (`build --config`) | Simulation horizon & vehicle classes | Object |
| `scenario.json` (`build -o`, `run -i`, `verify -i`) | Compiled, validated scenario artifact | `Scenario` object |
| `results.json` (`run -o`) | Simulation run summary & binned statistics | `RunOutput` object |
| `report.json` (`verify -o`) | Physics & conservation verification report | `VerificationReport` object |

---

## 1. Input JSON Formats

### 1.1 Source Inputs (`stream-cli build`)

`stream-cli build` compiles three source files into a single `scenario.json`:

```bash
stream-cli build --network network.geojson --demand demand.json --config config.json -o scenario.json
```

#### `config.json`

| Field | Type | Default | Description |
|---|---|---|---|
| `start_time` | `number` (`f64`) | `0.0` | Simulation start time (seconds). |
| `duration` | `number` (`f64`) | `3600.0` | Simulation duration (seconds). |
| `classes` | `string[]` | `["car"]` | Ordered vehicle class names (max 64). Index `i` maps to `class_id` and bit `i` in pipe class masks. |

Example:
```json
{
  "start_time": 0.0,
  "duration": 3600.0,
  "classes": ["car", "bus"]
}
```

#### `demand.json`

JSON array of `Demand` objects:

| Field | Type | Default | Description |
|---|---|---|---|
| `period_start` | `number` (`f64`) | required | Start of demand period (seconds). |
| `period_end` | `number` (`f64`) | required | End of demand period (seconds). |
| `origin` | `integer` (`usize`) | required unless `path` is set | Origin `NodeID`. When omitted with `path`, inferred from `path[0].node_up`. |
| `destination` | `integer` (`usize`) | required unless `path` is set | Destination `NodeID`. When omitted with `path`, inferred from `path[last].node_down`. |
| `flow` | `number` (`f64`) | derived if `count` is set | Flow rate (veh/h). Derived as `count * 3600 / duration` when omitted. |
| `count` | `number` (`f64`) | derived if `flow` is set | Number of vehicles to generate in `[period_start, period_end)`. Derived as `flow * duration / 3600` when omitted. |
| `class` | `string \| null` | `null` | Vehicle class name (must exist in `config.json` `classes`). `null` or omitted uses class index `0`. |
| `path` | `integer[] \| null` | `null` | Optional explicit route (ordered list of `LinkID`s). When provided, assignment injects vehicles along this exact route instead of computing shortest paths. |

Example:
```json
[
  {
    "period_start": 0.0,
    "period_end": 3600.0,
    "origin": 0,
    "destination": 2,
    "flow": 1000.0,
    "count": 1000.0
  },
  {
    "period_start": 0.0,
    "period_end": 3600.0,
    "flow": 400.0,
    "path": [0, 3, 2]
  }
]
```

#### `network.geojson`

A GeoJSON `FeatureCollection` containing:
- **`Point` features** (nodes): `coordinates` is `[x, y]`.
- **`LineString` features** (links): `coordinates` is `[[x0, y0], [x1, y1], ...]`.

**Node (`Point`) `properties`:**

| Property | Type | Default | Description |
|---|---|---|---|
| `id` | `integer` (`usize`) | required | Unique node ID. During compilation, nodes are sorted by `id` (and `id` must match array index). |
| `type` | `string` | `"Internal"` | `"Entry"`, `"Exit"`, or `"Internal"`. |

**Link (`LineString`) `properties`:**

| Property | Type | Default | Description |
|---|---|---|---|
| `id` | `integer` (`usize`) | required | Unique link ID. During compilation, links are sorted by `id`. |
| `node_up` | `integer` (`usize`) | required | Upstream `NodeID`. |
| `node_down` | `integer` (`usize`) | required | Downstream `NodeID`. |
| `length` | `number` (`f64`) | `0.0` | Link length (meters). |
| `speed` | `number` (`f64`) | `10.0` | Free-flow speed (m/s). |
| `lanes` | `integer` (`1..=255`) | `1` | Total number of lanes. |
| `capacity` | `number` (`f64`) | `1.0` | Whole-link discharge capacity (veh/s). |
| `fd_u` | `number` (`f64`) | `speed` | Triangular FD free-flow speed `u` (m/s). |
| `fd_w` | `number` (`f64`) | `5.0` | Triangular FD backward wave speed `w` (m/s). |
| `fd_kx` | `number` (`f64`) | `0.2` | Per-lane jam density `kx` (veh/m/lane). |
| `fd_c` | `number` (`f64`) | `capacity / lanes` | Per-lane capacity `c` (veh/s/lane). |
| `pipes` | `PipeInput[]` | omitted (single pipe) | Optional lane-group partition (index `0` = rightmost). Each entry has `lanes` (`1..=255`, required) and optional `classes` (`string[]`; omitted allows all classes). |
| `moves` | `object` | `{}` | Optional movement restrictions: `{"<out_link_id>": [pipe_idx, ...]}`. Missing key means all pipes can reach that downstream link. |
| `friction` | `number` (`f64`) | `1.0` | Cross-pipe spillback friction $\varphi \in (0, 1]$. `1.0` disables coupling. |

---

### 1.2 Compiled Scenario (`scenario.json`)

Used as output of `stream-cli build` and input to `stream-cli run -i` / `stream-cli verify -i`.

#### `Scenario`

| Field | Type | Default | Description |
|---|---|---|---|
| `nodes` | `Node[]` | required | Network nodes, sorted by `id`. |
| `links` | `Link[]` | required | Network links, sorted by `id`. |
| `vehicles` | `Vehicle[]` | required | Pre-assigned vehicles (may be `[]` when running with `--assignment`). |
| `demand` | `Demand[]` | required | OD demand entries (same schema as `demand.json`). |
| `start_time` | `number` (`f64`) | required | Simulation start time (seconds). |
| `duration` | `number` (`f64`) | required | Simulation duration (seconds). |
| `classes` | `string[]` | `["car"]` | Vehicle class names. |
| `link_schedule` | `LinkStateChange[]` | `[]` | Pre-resolved dynamic patch timeline, sorted by `(time, link_id)`. |
| `assignment_excluded` | `integer[]` | `[]` | Link IDs excluded from assignment routing. |

#### `Node`

| Field | Type | Description |
|---|---|---|
| `id` | `integer` (`usize`) | Node ID. |
| `node_type` | `"Internal" \| "Entry" \| "Exit"` | Node role in the network. |
| `incoming_links` | `integer[]` | IDs of incoming links. |
| `outgoing_links` | `integer[]` | IDs of outgoing links. |
| `points` | `[number, number]` | `[x, y]` coordinates. |
| `signals` | `integer[]` | Signal identifiers (reserved). |

#### `Link`

| Field | Type | Default | Description |
|---|---|---|---|
| `id` | `integer` (`usize`) | required | Link ID. |
| `node_up` | `integer` (`usize`) | required | Upstream node ID. |
| `node_down` | `integer` (`usize`) | required | Downstream node ID. |
| `length` | `number` (`f64`) | required | Link length (m). |
| `speed` | `number` (`f64`) | required | Free-flow speed (m/s). |
| `num_lanes` | `integer` (`u8`) | required | Number of lanes. |
| `capacity` | `number` (`f64`) | required | Whole-link capacity (veh/s). |
| `fd` | `FundamentalDiagram` | required | `{"u": f64, "w": f64, "kx": f64, "c": f64}`. |
| `points` | `[number, number][]` | required | Polyline vertices. |
| `priority` | `number` (`f64`) | required | Merge priority weight. |
| `pipe_specs` | `PipeSpec[]` | `[]` | Pipe partition; empty means a single pipe covering all `num_lanes` and all classes. |
| `moves` | `object` | `{}` | Map from stringified downstream `LinkID` to array of allowed `PipeIdx` (`u8`). |
| `friction` | `number` (`f64`) | `1.0` | Spillback coupling coefficient $\varphi \in (0, 1]$. |

#### `PipeSpec`

| Field | Type | Default | Description |
|---|---|---|---|
| `lanes` | `integer` (`u8`) | required | Number of lanes in this pipe. |
| `class_mask` | `integer` (`u64`) | `u64::MAX` | Bitmask of allowed vehicle classes (bit `i` set $\iff$ `classes[i]` allowed). |

#### `Vehicle`

| Field | Type | Description |
|---|---|---|
| `id` | `integer` (`usize`) | Unique vehicle ID. |
| `class_id` | `integer` (`usize`) | Index into `Scenario.classes`. |
| `path` | `integer[]` | Sequence of `LinkID`s from origin to destination. |
| `start_time` | `number` (`f64`) | Scheduled entry time (seconds). |
| `origin` | `integer` (`usize`) | Origin `NodeID`. |
| `destination` | `integer` (`usize`) | Destination `NodeID`. |

> Note: Runtime fields (`state`, `current_link_idx`, `node_times`, `pipes_taken`) are marked `#[serde(skip)]` and are not serialized in `Scenario`.

#### `LinkStateChange` (in `link_schedule`)

| Field | Type | Description |
|---|---|---|
| `time` | `number` (`f64`) | Event timestamp (seconds) when `attrs` take effect. |
| `link_id` | `integer` (`usize`) | Target `LinkID`. |
| `attrs` | `LinkAttrs` | Object with optional fields (`speed`, `capacity`, `fd_u`, `fd_w`, `fd_kx`, `fd_c`, `num_lanes`, `friction`, `pipe_specs`, `moves`, `class_masks`). |

---

## 2. Output JSON Formats

### 2.1 Simulation Results (`stream-cli run -o <FILE>`)

Generated by:
```bash
stream-cli run -i scenario.json [--assignment] [--bin-size 60] [--no-vehicles] -o results.json
```

#### `RunOutput`

| Field | Type | Description |
|---|---|---|
| `api_version` | `integer` (`u32`) | Schema version of `RunOutput` (`1`). |
| `engine_version` | `string` | Crate version of `stream-pipes-rust`. |
| `start_time` | `number` (`f64`) | Simulation start time (seconds). |
| `duration` | `number` (`f64`) | Simulation horizon duration (seconds). |
| `bin_size` | `number` (`f64`) | Aggregation bin width used for `link_stats`, `pipe_stats`, and `link_cvcs` (seconds). |
| `classes` | `string[]` | Vehicle class names (`class_id` lookup table). |
| `events_processed` | `integer` (`usize`) | Total discrete events processed by the simulator. |
| `vehicles_total` | `integer` (`usize`) | Total number of scheduled vehicles in the scenario. |
| `vehicles_arrived` | `integer` (`usize`) | Vehicles that reached their destination within the horizon. |
| `vehicle_kpis` | `VehicleKpis` | Pre-computed summary vehicle and network KPIs. |
| `warnings` | `string[]` | Compilation / patch warnings (empty for `stream-cli run`). |
| `link_stats` | `LinkStats[]` | Per-link binned series, sorted by `link_id`. |
| `link_cvcs` | `LinkCvc[]` | Pre-computed entry/exit cumulative volume curves per link, sorted by `link_id`. |
| `pipe_stats` | `PipeStats[]` | Per-pipe series for multi-pipe links only, sorted by `(link_id, pipe)`. |
| `vehicles` | `VehicleResult[]` | Per-vehicle trip trajectories (empty `[]` when `--no-vehicles` is passed). |

> Note: `stream-server` uses `RunResponse` (map-keyed `link_stats` / `pipe_stats` plus `scenario_name`, `patches_applied`, `warnings`, and `run_at_unix`).

#### `VehicleKpis`

| Field | Type | Description |
|---|---|---|
| `total` | `integer` (`usize`) | Scheduled vehicles (`vehicles_total`). |
| `entered` | `integer` (`usize`) | Vehicles that physically entered the network. |
| `arrived` | `integer` (`usize`) | Vehicles that reached their destination. |
| `in_network` | `integer` (`usize`) | Vehicles still on the network at horizon end (`entered - arrived`). |
| `queued_at_entry` | `integer` (`usize`) | Vehicles still waiting at an entry at horizon end (`total - entered`). |
| `completion_rate` | `number` (`f64`) | Fraction of scheduled vehicles that arrived (`arrived / total`). |
| `total_vkt_veh_km` | `number` (`f64`) | Total vehicle-kilometres over completed link traversals (veh·km). |
| `total_vht_veh_h` | `number` (`f64`) | Total in-network vehicle-hours during the horizon (veh·h). |
| `total_entry_wait_veh_h` | `number` (`f64`) | Total entry-queue waiting time during the horizon (veh·h). |
| `total_delay_veh_h` | `number` (`f64`) | Total in-network delay across arrived vehicles (veh·h). |
| `mean_travel_time_s` | `number` (`f64`) | Mean in-network travel time of arrived vehicles (s). |
| `p50_travel_time_s` | `number` (`f64`) | Median in-network travel time of arrived vehicles (s). |
| `p95_travel_time_s` | `number` (`f64`) | 95th percentile in-network travel time of arrived vehicles (s). |
| `mean_free_flow_time_s` | `number` (`f64`) | Mean free-flow travel time of arrived vehicles (s). |
| `mean_delay_s` | `number` (`f64`) | Mean in-network delay of arrived vehicles (s). |
| `mean_entry_wait_s` | `number` (`f64`) | Mean entry-queue wait across entered vehicles (s). |
| `max_entry_wait_s` | `number` (`f64`) | Maximum entry-queue wait across entered vehicles (s). |
| `mean_speed_kmh` | `number` (`f64`) | Space-mean speed across arrived vehicles (km/h). |

#### `LinkCvc`

| Field | Type | Description |
|---|---|---|
| `link_id` | `integer` (`usize`) | Link ID. |
| `t` | `number[]` | Sample timestamps (`n_bins + 1` points from `start_time` to `start_time + duration`). |
| `entry` | `integer[]` | Cumulative number of vehicles that entered the link by each timestamp in `t`. |
| `exit` | `integer[]` | Cumulative number of vehicles that exited the link by each timestamp in `t`. |

#### `LinkStats`

| Field | Type | Description |
|---|---|---|
| `link_id` | `integer` (`usize`) | Link ID. |
| `flow` | `TimeSeries` | Binned exit flow (veh/h). |
| `density` | `TimeSeries` | Binned Edie-style link density (veh/km). |
| `speed` | `TimeSeries` | Binned space-mean speed `flow / density` (km/h). |
| `kx_veh_km` | `number` (`f64`) | Whole-link jam density (`fd.kx * num_lanes * 1000`, veh/km). |
| `capacity_veh_h` | `number` (`f64`) | Whole-link capacity (`capacity * 3600`, veh/h). |

#### `PipeStats`

| Field | Type | Description |
|---|---|---|
| `link_id` | `integer` (`usize`) | Link ID. |
| `pipe` | `integer` (`u8`) | Pipe index on the link (`0` = rightmost). |
| `lanes` | `integer` (`u8`) | Number of lanes in this pipe. |
| `flow` | `TimeSeries` | Binned pipe exit flow (veh/h). |
| `density` | `TimeSeries` | Binned pipe density (veh/km). |
| `speed` | `TimeSeries` | Binned pipe space-mean speed (km/h). |
| `kx_veh_km` | `number` (`f64`) | Pipe jam density (`fd.kx * pipe.lanes * 1000`, veh/km). |
| `capacity_veh_h` | `number` (`f64`) | Pipe capacity (`pipe.capacity * 3600`, veh/h). |

#### `TimeSeries`

| Field | Type | Description |
|---|---|---|
| `t` | `number[]` | Bin start timestamps in seconds (`start_time + i * bin_size`). |
| `values` | `number[]` | Metric values corresponding to each bin in `t`. |

Example (`results.json` excerpt):
```json
{
  "events_processed": 10868,
  "duration_ms": 43,
  "vehicles_count": 2000,
  "status": "success",
  "link_stats": {
    "0": {
      "link_id": 0,
      "flow": { "t": [0.0, 60.0], "values": [1980.0, 2000.0] },
      "density": { "t": [0.0, 60.0], "values": [27.5, 27.8] },
      "speed": { "t": [0.0, 60.0], "values": [72.0, 72.0] },
      "kx_veh_km": 400.0,
      "capacity_veh_h": 7200.0
    }
  }
}
```

---

### 2.2 Verification Report (`stream-cli verify -o <FILE>`)

Generated by:
```bash
stream-cli verify -i scenario.json [--assignment] [--bin-size 300] -o report.json
```

#### `VerificationReport`

| Field | Type | Description |
|---|---|---|
| `summary` | `VerificationSummary` | Aggregate counts across all checked elements and checks. |
| `elements` | `ElementReport[]` | Per-element check results. |

#### `VerificationSummary`

| Field | Type | Description |
|---|---|---|
| `elements_checked` | `integer` (`usize`) | Total elements evaluated. |
| `elements_pass` | `integer` (`usize`) | Elements whose worst check status is `pass`. |
| `elements_warn` | `integer` (`usize`) | Elements whose worst check status is `warn`. |
| `elements_fail` | `integer` (`usize`) | Elements whose worst check status is `fail`. |
| `elements_not_exercised` | `integer` (`usize`) | Elements with all checks `not_exercised`. |
| `checks_total` | `integer` (`usize`) | Total individual checks executed. |
| `checks_fail` | `integer` (`usize`) | Total checks with status `fail`. |

#### `ElementReport`

| Field | Type | Description |
|---|---|---|
| `element_type` | `"link" \| "node" \| "entry"` | Type of network element. |
| `element_id` | `integer` (`usize`) | Link or Node ID. |
| `pipe` | `integer \| null` | Pipe index for per-pipe link checks; `null` for whole-link, node, or entry reports. |
| `label` | `string` | Human-readable element label. |
| `status` | `"pass" \| "warn" \| "fail" \| "not_exercised"` | Worst status among `checks`. |
| `checks` | `Check[]` | Individual physics/conservation checks for this element. |

#### `Check`

| Field | Type | Description |
|---|---|---|
| `id` | `string` | Machine-readable check identifier (e.g., `"free_flow_traversal"`, `"inflow_capacity"`, `"storage_bound"`, `"backward_wave"`, `"fd_adherence"`, `"flow_conservation"`, `"fifo_discipline"`, `"demand_served"`). |
| `name` | `string` | Human-readable check name. |
| `status` | `"pass" \| "warn" \| "fail" \| "not_exercised"` | Check outcome. |
| `measured` | `number \| null` | Value measured from simulation output. |
| `expected` | `number \| null` | Theoretical/expected value from element parameters. |
| `unit` | `string` | Unit of `measured` and `expected`. |
| `error_pct` | `number \| null` | Percentage error when both `measured` and `expected` are defined. |
| `detail` | `string` | Explanation of what was compared and on how much data. |
