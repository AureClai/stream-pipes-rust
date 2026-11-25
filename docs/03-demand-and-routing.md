# 3. Demand & Routing

There are two ways to feed demand into `stream-cli`.

---

## Option A: OD Demand + Built-in Assignment (`--assignment`)

Use `demand.json` to specify origin–destination demand periods:

```json
[
  {
    "period_start": 0.0,
    "period_end": 3600.0,
    "origin": 0,
    "destination": 2,
    "flow": 1000.0,
    "count": 1000.0,
    "class": "car"
  }
]
```

Then run with `--assignment`:

```bash
stream-cli run -i scenario.json --assignment -o results.json
```

**How built-in assignment works:**
- For entries without `path`, routes each OD pair by shortest free-flow travel time (`length / speed`) using Dijkstra/A*.
- For entries with `path`, injects vehicles directly along that sequence of `LinkID`s (validated for connectivity and class/movement reachability).
- Uniformly spaces `count` vehicles over `[period_start, period_end)`.
- Respects `assignment_excluded` links and pipe class/movement reachability.
- Assignment is **static** (routes are fixed at `t = 0` and do not dynamically reroute around mid-run patches).

---

## Option B: Route-Based Demand (`path` in `demand.json`)

If an external assignment tool (UE, logit, transit lines, dynamic assignment) already chose routes and flows, supply an explicit `path` on each entry in `demand.json`:

```json
[
  {
    "period_start": 0.0,
    "period_end": 3600.0,
    "flow": 600.0,
    "path": [0, 1, 3]
  },
  {
    "period_start": 0.0,
    "period_end": 3600.0,
    "flow": 400.0,
    "path": [0, 2, 3]
  }
]
```

- `origin` and `destination` are optional when `path` is given (inferred from the first link's `node_up` and the last link's `node_down`; if provided, they must match).
- `count` may be omitted when `flow` is given (`count = flow × (period_end − period_start) / 3600`), and vice versa.
- Route-based entries can be mixed freely with OD-only entries in the same `demand.json`, and work with `stream-cli run --assignment`, `stream-cli verify --assignment`, `ScaleDemand` patches, and `api::run`.

---

## Option C: Pre-Assigned Vehicles (`vehicles` in `scenario.json`)

If you want to control each individual vehicle's departure timestamp rather than uniform headways, you can bypass `assign_demand` completely:

1. Compile the base network (`stream-cli build ... -o scenario.json`) with an empty `demand.json` (`[]`).
2. Populate the `vehicles` array in `scenario.json` with your pre-routed vehicles:
   ```json
   "vehicles": [
     {
       "id": 0,
       "class_id": 0,
       "path": [0, 2, 5],
       "start_time": 12.5,
       "origin": 0,
       "destination": 4
     }
   ]
   ```
   - `path`: ordered list of `LinkID`s from `origin` to `destination`.
   - `class_id`: 0-based index into `Scenario.classes`.
   - `start_time`: departure time in seconds.
3. Run **without** `--assignment`:
   ```bash
   stream-cli run -i scenario.json -o results.json
   ```

`stream-cli` will validate every vehicle's `path` (connectivity, origin/destination match, class/movement reachability) and simulate those exact trajectories.
