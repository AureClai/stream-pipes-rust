# 8. Python Bindings (`stream-pipes` / `stream_core_rust`)

`stream-pipes` provides native PyO3 Python bindings (`import stream_core_rust`) backed by the Rust simulation engine. Because the extension is built with `abi3-py39`, a single compiled wheel works across Python 3.9 and newer without Python-side serialization overhead.

---

## Installation

### From a local clone (with `pip` or `maturin`)

Requires a Rust toolchain (Rust ≥ 1.82) and Python ≥ 3.9:

```bash
# Install directly via pip (uses pyproject.toml / maturin build backend)
pip install .

# Or for editable/local development in a virtualenv:
pip install maturin
maturin develop --release
```

### Directly from GitHub

```bash
pip install "git+https://github.com/AureClai/stream-pipes-rust.git"
```

> **Note:** Pre-built wheels on PyPI (`pip install stream-pipes` without a Rust compiler) will be available once wheels are published on release. Package distribution name is `stream-pipes`; Python import name is `stream_core_rust`.

---

## Quickstart

```python
import stream_core_rust as sc

# 1. Validate a scenario folder (no simulation run)
report = sc.check(dir="scenarios/diverge")
assert report["errors"] == []

# 2. Simulate and collect link/pipe stats, CVCs, turning counts, queue summaries, OD skims
out = sc.run(dir="scenarios/diverge", bin_size=60.0)
print("Arrived:", out["vehicles_arrived"], "/", out["vehicles_total"])
print("Mean speed (km/h):", out["vehicle_kpis"]["mean_speed_kmh"])

# 3. Audit physical invariants + all 18 closed-form analytic benchmarks
ver = sc.verify(dir="scenarios/urban", benchmarks=True)
assert ver["summary"]["checks_fail"] == 0
```

You can also pass patches and `merge` options directly:

```python
out = sc.run(
    dir="scenarios/a47_givors_osm",
    patches=["gate"],
    bin_size=300.0,
)
```

---

## In-Memory Workflows & Route-Based Demand

All functions (`check`, `run`, `verify`, `Simulation.from_inputs`) accept standard Python `dict` and `list` objects directly in memory, avoiding JSON file round-trips inside iterative loops (such as Dynamic Traffic Assignment or 4-step model iterations):

```python
import json
from pathlib import Path
import stream_core_rust as sc

base = Path("scenarios/diverge")
network = json.loads((base / "network.geojson").read_text())
config = json.loads((base / "config.json").read_text())

# Explicit route-based demand (path = list of link IDs)
demand = [
    {"period_start": 0.0, "period_end": 1800.0, "flow": 360.0, "path": [0, 1]},
    {"period_start": 0.0, "period_end": 1800.0, "flow": 180.0, "path": [0, 2]},
]

out = sc.run(network=network, demand=demand, config=config, keep_vehicles=False)
for skim in out["od_skims"]:
    print(f"{skim['origin']} -> {skim['destination']}: {skim['mean_travel_time_s']:.2f} s")
```

---

## Interactive `Simulation` Handle

Use `Simulation` when you want step-by-step control over building, attaching controllers, running, auditing, and comparing against observed detector counts:

```python
from stream_core_rust import Simulation

sim = Simulation.from_dir("scenarios/a47_givors_osm", patches=["gate"])
events = sim.run()

out = sim.collect(events_processed=events, bin_size=300.0, keep_vehicles=False)
ver = sim.verify(bin_size=300.0)
ref = sim.compare_observations(observed_records)  # list of ObservedRecord dicts
print("Flow RMSE (veh/h):", ref["flow_rmse_veh_h"], "Mean GEH:", ref["flow_mean_geh"])
```

---

## Python Signal & Metering Controllers

You can write closed-loop traffic control laws (e.g. ALINEA ramp metering or actuated phases) in Python and pass them via `controllers=[...]` or `sim.add_controller(...)`.

A controller implements `start(self, view: ControlView) -> list[Command]` and `wake(self, view: ControlView) -> list[Command]`:

```python
from stream_core_rust import Command, ControlView, Simulation

class SimpleMeter:
    def __init__(self, group: str, detector_link: int, period: float = 60.0):
        self.group = group
        self.detector = detector_link
        self.period = period

    def start(self, view: ControlView) -> list[Command]:
        return [Command.wake_at(self.period)]

    def wake(self, view: ControlView) -> list[Command]:
        t = view.time()
        occ = view.occupancy(self.detector)
        # Example: cap to 1 veh/green when downstream occupancy exceeds 14%
        cap = 1 if occ > 0.14 else None
        return [
            Command.set_vehicles_per_green(self.group, cap),
            Command.wake_at(t + self.period),
        ]
```

### `ControlView` Sensors

| Method | Return | Description |
|---|---|---|
| `view.time()` | `float` | Current simulation time (s). |
| `view.group(id)` | `int \| None` | Signal group index by ID. |
| `view.is_green(id)` | `bool \| None` | Current signal aspect for group `id`. |
| `view.vehicles(link_id)` | `int` | Vehicles currently on `link_id`. |
| `view.queue(link_id)` | `int` | Vehicles queued at the downstream end of `link_id`. |
| `view.entry_queue(link_id)` | `int` | Vehicles waiting upstream of `link_id`. |
| `view.density(link_id)` | `float` | Mean density on `link_id` (veh/m). |
| `view.occupancy(link_id)` | `float` | Occupancy proxy in `[0, 1]` (`density / jam_density`). |
| `view.entered(link_id)` | `int` | Cumulative vehicles that entered `link_id`. |
| `view.exited(link_id)` | `int` | Cumulative vehicles that exited `link_id`. |

### `Command` Builders

- `Command.switch(group, at, green)` — Switch `group` green (`True`) or red (`False`) at time `at`.
- `Command.wake_at(at)` — Schedule the next `wake(view)` call at time `at`.
- `Command.set_vehicles_per_green(group, n=None)` — Set per-green cap (`None` removes cap).

---

## Progress & Cancellation

Pass a `progress` callback to `sc.run(...)`; returning `False` cancels the simulation and raises `RuntimeError`:

```python
def on_progress(evt: dict) -> bool:
    if evt["stage"] == "simulating":
        print(f"t = {evt['sim_time']:.0f}s ({evt['events']} events)")
    return True

out = sc.run(dir="scenarios/a47_givors_osm", patches=["gate"], progress=on_progress)
```

---

## Typing & API Reference

Type stubs (`__init__.pyi`) and `py.typed` are bundled in `python/stream_core_rust/` for IDE autocomplete and `mypy` / `pyright` checking.

Top-level functions:
- `check(inputs=None, *, dir=None, network=None, demand=None, config=None, signals=None, patches=None) -> dict`
- `run(..., assignment=True, merge=None, upstream_capacity=False, bin_size=60.0, keep_vehicles=False, controllers=None, progress=None) -> dict`
- `verify(..., assignment=True, merge=None, upstream_capacity=False, bin_size=300.0, benchmarks=False, observed=None, controllers=None) -> dict`
- `run_benchmarks() -> list[dict]`
