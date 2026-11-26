"""Iterative in-memory Dynamic Traffic Assignment (MSA) with `stream_core_rust`.

Demonstrates how `pipe-stream` acts as a fast Dynamic Network Loading (DNL)
kernel inside an external Python route-choice loop:
- Total OD demand (2,400 veh/h for 1 h) splits across two routes from node 0 to node 2:
    * Route A (link 0): short 1,000 m direct link, 25 m/s (40 s free flow), capacity 0.5 veh/s (1,800 veh/h)
    * Route B (link 1): longer 1,500 m bypass link, 25 m/s (60 s free flow), capacity 1.0 veh/s (3,600 veh/h)
- At each iteration k, Method of Successive Averages (MSA) shifts share `1/k`
  toward the cheaper route based on simulated travel time (`mean_total_time_s`),
  passing updated route-based `demand` directly in memory.

Usage:
    python python/examples/in_memory_dta.py
"""

from __future__ import annotations

import time
import stream_core_rust as sc

HORIZON = 3600.0
TOTAL_FLOW = 2400.0


def build_network() -> tuple[dict, dict]:
    network = {
        "type": "FeatureCollection",
        "features": [
            {"type": "Feature", "geometry": {"type": "Point", "coordinates": [0.0, 0.0]}, "properties": {"id": 0, "type": "Entry"}},
            {"type": "Feature", "geometry": {"type": "Point", "coordinates": [1000.0, 0.0]}, "properties": {"id": 1, "type": "Exit"}},
            {
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [1000.0, 0.0]]},
                "properties": {
                    "id": 0, "node_up": 0, "node_down": 1, "length": 1000.0,
                    "speed": 25.0, "lanes": 1, "capacity": 0.5,
                    "fd_u": 25.0, "fd_w": 5.0, "fd_kx": 0.15, "fd_c": 0.5,
                },
            },
            {
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": [[0.0, 0.0], [500.0, 300.0], [1000.0, 0.0]]},
                "properties": {
                    "id": 1, "node_up": 0, "node_down": 1, "length": 1500.0,
                    "speed": 25.0, "lanes": 2, "capacity": 1.0,
                    "fd_u": 25.0, "fd_w": 5.0, "fd_kx": 0.15, "fd_c": 0.5,
                },
            },
        ],
    }
    config = {"start_time": 0.0, "duration": HORIZON + 1800.0}
    return network, config


def route_mean_total_times(vehicles: list[dict]) -> tuple[float, float]:
    by_route: dict[int, list[float]] = {0: [], 1: []}
    for v in vehicles:
        nt = v["node_times"]
        if len(nt) >= 2:
            by_route[v["path"][0]].append(nt[-1] - v["start_time"])
    t0 = sum(by_route[0]) / len(by_route[0]) if by_route[0] else 40.0
    t1 = sum(by_route[1]) / len(by_route[1]) if by_route[1] else 60.0
    return t0, t1


def main() -> None:
    network, config = build_network()
    share_a = 1.0  # start all-or-nothing on the shorter route

    print("iter | share A | flow A (veh/h) | flow B (veh/h) | mean time A | mean time B | runtime")
    t_start = time.perf_counter()
    for k in range(1, 13):
        flow_a = round(TOTAL_FLOW * share_a, 2)
        flow_b = round(TOTAL_FLOW - flow_a, 2)
        demand = [
            {"period_start": 0.0, "period_end": HORIZON, "flow": flow_a, "path": [0]},
            {"period_start": 0.0, "period_end": HORIZON, "flow": flow_b, "path": [1]},
        ]
        t0_run = time.perf_counter()
        out = sc.run(network=network, demand=demand, config=config, keep_vehicles=True)
        dt_ms = (time.perf_counter() - t0_run) * 1000.0

        tt_a, tt_b = route_mean_total_times(out["vehicles"])
        print(f"{k:>4} | {share_a:>7.3f} | {flow_a:>14.1f} | {flow_b:>14.1f} | {tt_a:>9.2f} s | {tt_b:>9.2f} s | {dt_ms:>5.2f} ms")

        target_a = 1.0 if tt_a <= tt_b else 0.0
        step = 1.0 / (k + 1)
        share_a = (1.0 - step) * share_a + step * target_a

    print(f"\nCompleted 12 DTA iterations in {(time.perf_counter() - t_start) * 1000.0:.1f} ms total.")


if __name__ == "__main__":
    main()
