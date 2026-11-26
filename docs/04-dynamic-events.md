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

## CLI Usage (`--patches`)

You can pass one or more patch JSON files directly to `stream-cli build`, `run`, or `verify`:

```bash
# Bake patches into a compiled scenario.json (supports all PatchOp types, including add_node / add_link)
stream-cli build \
  --network network.geojson --demand demand.json --config config.json \
  --patches patches/closure.json patches/demand_peak.json \
  -o scenario.json

# Or run/verify directly from a scenario directory using patch names in patches/<name>.json
stream-cli run -i scenarios/a47_givors_osm --patches gate --assignment -o results.json
stream-cli verify -i scenarios/a47_givors_osm --patches gate --assignment

# Or apply demand/windowed patches on top of an existing scenario.json
stream-cli run -i scenario.json --patches patches/incident.json --assignment -o results.json
```

> **Note:** In directory mode (`-i <DIR>`), `--merge` defaults to `config.json`'s `"merge"` setting unless overridden on the CLI. Topology patches (`add_node`, `add_link`) require source inputs (`stream-cli build --patches` or directory mode `-i <DIR>`). When patching only `fd_c` or only `capacity`, `apply_attrs` keeps the sibling field synchronized; note that `fd_c`/`capacity` limits throttle entry into downstream links, so gating traffic leaving at an `Exit` node requires a downstream stop-bar link (as in `scenarios/a47_givors_osm` and `scenarios/a15_papendrecht`).
