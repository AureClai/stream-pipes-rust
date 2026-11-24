# 6. Traffic Signals & Tolls

> Note: Traffic signals and tolls are implemented on the `feat/signals` branch and compiled with `stream-cli build --signals signals.json`.

A `signals.json` file defines **signal groups**, **fixed-time plans**, and **tolls** at network nodes and links.

## CLI Usage

Pass `--signals` to `stream-cli build`:

```bash
stream-cli build \
  --network scenarios/urban/network.geojson \
  --demand scenarios/urban/demand.json \
  --config scenarios/urban/config.json \
  --signals scenarios/urban/signals.json \
  -o scenario.json
```

---

## `signals.json` Structure

Top-level object (`SignalNetwork`):

| Field | Type | Default | Description |
|---|---|---|---|
| `groups` | `SignalGroup[]` | `[]` | Signal groups controlling node movements. |
| `plans` | `FixedTimePlan[]` | `[]` | Time-windowed fixed-time signal plans. |
| `tolls` | `Toll[]` | `[]` | Deterministic service booths on link exits. |

---

### Signal Groups (`groups`)

Each `SignalGroup` controls one or more movements at a single node:

| Field | Type | Default | Description |
|---|---|---|---|
| `id` | `string` | required | Unique group name (referenced by `plans`). |
| `node` | `integer` (`usize`) | required | `NodeID` where this group is located (cannot be a roundabout). |
| `movements` | `Movement[]` | required | Non-empty list of `{ "from": LinkID, "to": LinkID? }`. Omitting `"to"` controls every movement from `"from"`. |
| `vehicles_per_green` | `integer \| null` | `null` | Optional cap on vehicles admitted per green phase (e.g. `1` for ramp metering). Must be $\ge 1$. |
| `off` | `"green" \| "red"` | `"green"` | State outside every active plan period (`"green"` = dark signal lets traffic through). |
| `sneakers` | `integer` (`u32`) | `2` | Permitted-turn vehicles already at the stop line that may still clear after red. |
| `clearance` | `number` (`f64`) | `3.0` | Clearance window (seconds) after red during which `sneakers` may clear. |

**FIFO and turn pockets:**
A red signal holds the head vehicle of any pipe whose movement it controls. On a shared lane, vehicles behind it wait (FIFO); on a link with dedicated turn pockets (`pipes` + `moves`), a red left-turn pocket does not block through lanes.

---

### Fixed-Time Plans (`plans`)

Each `FixedTimePlan` schedules signal cycles over `[from, until)`:

| Field | Type | Default | Description |
|---|---|---|---|
| `name` | `string \| null` | `null` | Optional human-readable plan name. |
| `from` | `number` (`f64`) | required | Period start time (seconds). |
| `until` | `number` (`f64`) | required | Period end time (seconds). |
| `cycle` | `number` (`f64`) | required | Cycle length (seconds). |
| `offset` | `number` (`f64`) | `0.0` | Cycle offset from `from` (seconds). |
| `groups` | `string[]` | `[]` | Groups controlled by this plan (all keys in `greens` are included automatically; listed groups without windows stay red). |
| `greens` | `object` | required | Map from group `id` to `[[start, end], ...]` green windows within `[0, cycle)` (or extending past `cycle` into the next cycle). |

---

### Tolls (`tolls`)

A `Toll` models a deterministic server at the downstream end of a link:

| Field | Type | Default | Description |
|---|---|---|---|
| `link` | `integer` (`usize`) | required | Target `LinkID`. |
| `pipes` | `integer[] \| null` | `null` | Optional pipe indices with toll booths (`null` = all pipes). |
| `service_time` | `number` (`f64`) | required | Default service time (seconds) per vehicle. |
| `by_class` | `object` | `{}` | Optional map from vehicle class name to class-specific service time (seconds). |

---

### Example (`signals.json`)

```json
{
  "groups": [
    {
      "id": "main_through",
      "node": 4,
      "movements": [{ "from": 10, "to": 14 }, { "from": 15, "to": 11 }]
    },
    {
      "id": "main_left",
      "node": 4,
      "movements": [{ "from": 10, "to": 16 }]
    },
    {
      "id": "ramp_meter",
      "node": 9,
      "movements": [{ "from": 50 }],
      "vehicles_per_green": 1
    }
  ],
  "plans": [
    {
      "name": "morning",
      "from": 0,
      "until": 1800,
      "cycle": 80,
      "offset": 0,
      "greens": {
        "main_through": [[0, 35]],
        "main_left": [[38, 50]]
      }
    }
  ],
  "tolls": [
    {
      "link": 2,
      "service_time": 6.0,
      "by_class": { "bus": 2.0 }
    }
  ]
}
```
