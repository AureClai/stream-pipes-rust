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
- Routes each OD pair by shortest free-flow travel time (`length / speed`) using Dijkstra/A*.
- Uniformly spaces `count` vehicles over `[period_start, period_end)`.
- Respects `assignment_excluded` links and pipe class/movement reachability.
- Assignment is **static** (routes are fixed at `t = 0` and do not dynamically reroute around mid-run patches).

---

## Option B: External Route Assignment (`vehicles` in `scenario.json`)

If you run your own route-assignment algorithm externally, you can bypass `assign_demand` completely:

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
