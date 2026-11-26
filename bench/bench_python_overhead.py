"""Benchmark `stream_core_rust` Python bindings overhead vs pure-Rust solver time.

Measures:
1. `Simulation.from_dir(...)` compile + assign time in Python
2. `sim.run()` solver-only time (GIL released via `py.allow_threads`)
3. `sc.run(dir=...)` full end-to-end call (compile + assign + run + KPI marshaling to Python dicts)

Usage (after installing `stream-pipes` via `pip install .` or `maturin develop --release`):
    python bench/bench_python_overhead.py
"""

from __future__ import annotations

import statistics
import time
from pathlib import Path

import stream_core_rust as sc
from stream_core_rust import Simulation

ROOT = Path(__file__).resolve().parent.parent
SCENARIOS = ROOT / "scenarios"


def bench_scenario(name: str, patches: list[str] | None = None, repeats: int = 5) -> None:
    path = str(SCENARIOS / name)
    patch_arg = patches or []

    init_times = []
    run_times = []
    e2e_times = []
    n_events = 0
    n_veh = 0

    for _ in range(repeats):
        t0 = time.perf_counter()
        sim = Simulation.from_dir(path, patches=patch_arg)
        t1 = time.perf_counter()
        n_events = sim.run()
        t2 = time.perf_counter()
        n_veh = sim.vehicles_count

        init_times.append((t1 - t0) * 1000.0)
        run_times.append((t2 - t1) * 1000.0)

        t3 = time.perf_counter()
        _ = sc.run(dir=path, patches=patch_arg, keep_vehicles=False)
        t4 = time.perf_counter()
        e2e_times.append((t4 - t3) * 1000.0)

    print(
        f"{name:<16} | {n_veh:>6} veh | {n_events:>7} events | "
        f"init {statistics.median(init_times):>6.2f} ms | "
        f"sim.run {statistics.median(run_times):>6.2f} ms | "
        f"sc.run (full) {statistics.median(e2e_times):>6.2f} ms"
    )


def main() -> None:
    print(f"stream_core_rust v{sc.__version__} — Python bindings benchmark (median of 5 runs)\n")
    bench_scenario("bottleneck")
    bench_scenario("diverge")
    bench_scenario("grid")
    bench_scenario("urban")
    bench_scenario("a47_givors_osm", patches=["gate"], repeats=3)


if __name__ == "__main__":
    main()
