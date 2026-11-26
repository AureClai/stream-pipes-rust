"""Tests for the `stream_core_rust` PyO3 Python bindings."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

import stream_core_rust as sc
from stream_core_rust import Command, ControlView, Simulation

ROOT = Path(__file__).resolve().parent.parent.parent
SCENARIOS = ROOT / "scenarios"


def test_check_directory_and_invalid_input() -> None:
    report = sc.check(dir=str(SCENARIOS / "diverge"))
    assert report["errors"] == []
    assert report["links"] == 3
    assert report["nodes"] == 4
    assert report["demand_vehicles"] > 0

    bad = sc.check(
        network={"type": "Point", "coordinates": [0, 0]},
        demand=[],
        config={"duration": 100.0},
    )
    assert len(bad["errors"]) > 0


def test_run_directory_and_study_kpis() -> None:
    out = sc.run(dir=str(SCENARIOS / "diverge"), bin_size=60.0)
    assert out["vehicles_total"] == 2000
    assert out["vehicles_arrived"] == 1956
    assert out["vehicle_kpis"]["in_network"] == 44
    assert out["vehicles"] == []
    assert len(out["link_stats"]) == 3
    assert len(out["link_cvcs"]) == 3
    assert len(out["turning_counts"]) == 2
    assert len(out["queue_summaries"]) == 3
    assert len(out["od_skims"]) == 2
    assert out["vehicle_kpis"]["mean_speed_kmh"] > 0.0


def test_in_memory_route_based_demand() -> None:
    base = SCENARIOS / "diverge"
    network = json.loads((base / "network.geojson").read_text(encoding="utf-8"))
    config = json.loads((base / "config.json").read_text(encoding="utf-8"))
    demand = [
        {"period_start": 0.0, "period_end": 1800.0, "flow": 360.0, "path": [0, 1]},
        {"period_start": 0.0, "period_end": 1800.0, "flow": 180.0, "path": [0, 2]},
    ]

    out = sc.run(
        network=network,
        demand=demand,
        config=config,
        keep_vehicles=True,
    )
    assert out["vehicles_total"] == 270
    assert sum(1 for v in out["vehicles"] if v["path"] == [0, 1]) == 180
    assert sum(1 for v in out["vehicles"] if v["path"] == [0, 2]) == 90


def test_verify_urban_and_benchmarks() -> None:
    report = sc.verify(dir=str(SCENARIOS / "urban"), benchmarks=True)
    assert report["summary"]["checks_fail"] == 0
    assert len(report["benchmarks"]) == 18
    assert all(b["status"] == "pass" for b in report["benchmarks"])


def test_run_benchmarks_standalone() -> None:
    results = sc.run_benchmarks()
    assert len(results) == 18
    assert all(r["status"] == "pass" for r in results)


def test_python_controller_hook() -> None:
    network = {
        "type": "FeatureCollection",
        "features": [
            {"type": "Feature", "geometry": {"type": "Point", "coordinates": [0.0, 0.0]}, "properties": {"id": 0, "type": "Entry"}},
            {"type": "Feature", "geometry": {"type": "Point", "coordinates": [100.0, 0.0]}, "properties": {"id": 1, "type": "Internal"}},
            {"type": "Feature", "geometry": {"type": "Point", "coordinates": [200.0, 0.0]}, "properties": {"id": 2, "type": "Exit"}},
            {
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [100.0, 0.0]]},
                "properties": {"id": 0, "node_up": 0, "node_down": 1, "length": 100.0, "speed": 10.0, "lanes": 1, "capacity": 1.0, "fd_u": 10.0, "fd_w": 5.0, "fd_kx": 0.2, "fd_c": 1.0},
            },
            {
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": [[100.0, 0.0], [200.0, 0.0]]},
                "properties": {"id": 1, "node_up": 1, "node_down": 2, "length": 100.0, "speed": 10.0, "lanes": 1, "capacity": 1.0, "fd_u": 10.0, "fd_w": 5.0, "fd_kx": 0.2, "fd_c": 1.0},
            },
        ],
    }
    demand = [{"period_start": 0.0, "period_end": 60.0, "origin": 0, "destination": 2, "flow": 1800.0}]
    config = {"start_time": 0.0, "duration": 120.0}
    signals = {
        "groups": [{"id": "g0", "node": 1, "movements": [{"from": 0, "to": 1}]}],
        "plans": [],
        "tolls": [],
    }

    class ToggleController:
        def __init__(self) -> None:
            self.wakes: list[float] = []

        def start(self, view: ControlView) -> list[Command]:
            assert view.group("g0") == 0
            return [
                Command.switch("g0", 0.0, False),
                Command.wake_at(20.0),
            ]

        def wake(self, view: ControlView) -> list[Command]:
            self.wakes.append(view.time())
            _ = view.occupancy(0)
            return [Command.switch("g0", 20.0, True)]

    ctrl = ToggleController()
    sim = Simulation.from_inputs(network, demand, config, signals=signals)
    sim.add_controller(ctrl)
    events = sim.run()
    assert events > 0
    assert ctrl.wakes == [20.0]
    log = sim.signal_log()
    assert any(entry["time"] == 0.0 and entry["green"] is False for entry in log)
    assert any(entry["time"] == 20.0 and entry["green"] is True for entry in log)
    out = sim.collect(events, bin_size=60.0)
    assert out["vehicles_arrived"] == 30


def test_controller_exception_propagates() -> None:
    base = SCENARIOS / "diverge"
    network = json.loads((base / "network.geojson").read_text(encoding="utf-8"))
    config = json.loads((base / "config.json").read_text(encoding="utf-8"))
    demand = json.loads((base / "demand.json").read_text(encoding="utf-8"))

    class BrokenController:
        def start(self, view: ControlView) -> list[Command]:
            raise RuntimeError("boom from python controller")

        def wake(self, view: ControlView) -> list[Command]:
            return []

    with pytest.raises(RuntimeError, match="boom from python controller"):
        sc.run(network=network, demand=demand, config=config, controllers=[BrokenController()])


def test_progress_callback_cancellation() -> None:
    with pytest.raises(RuntimeError, match="cancelled"):
        sc.run(dir=str(SCENARIOS / "diverge"), progress=lambda _p: False)
