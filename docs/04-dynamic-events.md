# 4. Dynamic Events (Mid-Run Parameter Changes)

The simulation engine supports predetermined, time-windowed changes to link parameters during a run (such as capacity drops from an accident, speed limits, or lane closures).

## How It Works

`Scenario.link_schedule` holds a time-sorted list of `LinkStateChange` events. During simulation, each entry triggers an `ApplyPatch` event at `time`, updating the link's effective attributes and refreshing its pipes in place (preserving existing queues).

Supported dynamic attributes in `LinkAttrs`:
- `capacity` (whole-link capacity, veh/s)
- `fd_c` (per-lane capacity, veh/s/lane)
- `speed`, `fd_u`, `fd_w`, `fd_kx`
- `num_lanes`, `friction`
- `pipe_specs`, `moves`, `class_masks` (note: the number of pipes on a link cannot change mid-run)

## Example: Capacity Drop from `t = 1800` to `t = 4200`

In `scenario.json`, add entries to `link_schedule` (sorted by `(time, link_id)`):

```json
"link_schedule": [
  {
    "time": 1800.0,
    "link_id": 3,
    "attrs": {
      "capacity": 0.5,
      "fd_c": 0.25
    }
  },
  {
    "time": 4200.0,
    "link_id": 3,
    "attrs": {
      "capacity": 1.0,
      "fd_c": 0.5
    }
  }
]
```

At `t = 1800 s`, link `3` drops to half capacity, and at `t = 4200 s` it is restored to its nominal values.

> **Note:** The underlying `patch` module (`SetLink`, `ClosePipe`, `CloseLink`) compiles time windows `[from, until)` into `link_schedule` automatically when used via `stream-server` or the Rust library (`compile_with_patches`). Exposing `--patches` directly on `stream-cli` is planned; until then, populate `link_schedule` in `scenario.json`.
