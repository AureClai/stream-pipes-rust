"""ALINEA ramp metering in pure Python using `stream_core_rust`.

Mirrors `examples/ramp_metering_alinea.rs` without writing any Rust code:
- Two-lane motorway (link 0, 1 km) and one-lane on-ramp (link 1, 300 m) merge
  at node 2 into link 2 (500 m), ending at a lane-drop bottleneck (link 3).
- Mainline 1,500 veh/h + ramp 500 veh/h exceed the 1,800 veh/h bottleneck.
- An external Python `AlineaController` reads `view.occupancy(2)` every 60 s
  and emits `Command.switch` / `Command.wake_at` events.

Usage:
    python python/examples/ramp_metering_alinea.py
"""

from __future__ import annotations

from stream_core_rust import Command, ControlView, Simulation

HORIZON = 3600.0


class AlineaController:
    """Closed-loop ALINEA ramp meter implemented in Python."""

    def __init__(
        self,
        group: str,
        detector: int,
        period: float = 60.0,
        target: float = 0.14,
        gain: float = 7000.0,
        rate: float = 900.0,
        min_rate: float = 200.0,
        max_rate: float = 1800.0,
        green: float = 1.0,
    ) -> None:
        self.group = group
        self.detector = detector
        self.period = period
        self.target = target
        self.gain = gain
        self.rate = rate
        self.min_rate = min_rate
        self.max_rate = max_rate
        self.green = green
        self.history: list[tuple[float, float, float]] = []

    def _step(self, view: ControlView) -> list[Command]:
        t = view.time()
        occ = view.occupancy(self.detector)
        self.rate = max(
            self.min_rate,
            min(self.max_rate, self.rate + self.gain * (self.target - occ)),
        )
        self.history.append((t, occ, self.rate))

        cycle = 3600.0 / self.rate
        cmds: list[Command] = []
        g = t
        while g < t + self.period:
            cmds.append(Command.switch(self.group, g, True))
            cmds.append(Command.switch(self.group, g + self.green, False))
            g += cycle
        cmds.append(Command.wake_at(t + self.period))
        return cmds

    def start(self, view: ControlView) -> list[Command]:
        return self._step(view)

    def wake(self, view: ControlView) -> list[Command]:
        return self._step(view)


def build_inputs() -> tuple[dict, list[dict], dict, dict]:
    def pt(nid: int, ntype: str) -> dict:
        return {
            "type": "Feature",
            "geometry": {"type": "Point", "coordinates": [float(nid * 500), 0.0]},
            "properties": {"id": nid, "type": ntype},
        }

    def lnk(lid: int, u: int, v: int, length: float, speed: float, lanes: int, cap: float, c_lane: float) -> dict:
        return {
            "type": "Feature",
            "geometry": {"type": "LineString", "coordinates": [[float(u * 500), 0.0], [float(v * 500), 0.0]]},
            "properties": {
                "id": lid,
                "node_up": u,
                "node_down": v,
                "length": length,
                "speed": speed,
                "lanes": lanes,
                "capacity": cap,
                "fd_u": speed,
                "fd_w": 5.0,
                "fd_kx": 0.125,
                "fd_c": c_lane,
            },
        }

    network = {
        "type": "FeatureCollection",
        "features": [
            pt(0, "Entry"),
            pt(1, "Entry"),
            pt(2, "Internal"),
            pt(3, "Internal"),
            pt(4, "Exit"),
            lnk(0, 0, 2, 1000.0, 25.0, 2, 1.0, 0.5),
            lnk(1, 1, 2, 300.0, 15.0, 1, 0.5, 0.5),
            lnk(2, 2, 3, 500.0, 25.0, 2, 1.0, 0.5),
            lnk(3, 3, 4, 1000.0, 25.0, 2, 0.5, 0.25),
        ],
    }
    demand = [
        {"period_start": 0.0, "period_end": HORIZON, "origin": 0, "destination": 4, "flow": 1500.0, "path": [0, 2, 3]},
        {"period_start": 0.0, "period_end": HORIZON, "origin": 1, "destination": 4, "flow": 500.0, "path": [1, 2, 3]},
    ]
    config = {"start_time": 0.0, "duration": HORIZON * 2.0, "merge": "capacity"}
    signals = {
        "groups": [
            {
                "id": "ramp",
                "node": 2,
                "movements": [{"from": 1}],
                "vehicles_per_green": 1,
                "off": "red",
                "sneakers": 0,
                "clearance": 0.0,
            }
        ],
        "plans": [],
        "tolls": [],
    }
    return network, demand, config, signals


def main() -> None:
    network, demand, config, signals = build_inputs()

    plain = Simulation.from_inputs(network, demand, config)
    plain_events = plain.run()
    plain_out = plain.collect(plain_events, bin_size=300.0)

    ctrl = AlineaController(group="ramp", detector=2)
    metered = Simulation.from_inputs(network, demand, config, signals=signals)
    metered.add_controller(ctrl)
    metered_events = metered.run()
    metered_out = metered.collect(metered_events, bin_size=300.0)

    for label, out in [("no metering", plain_out), ("ALINEA", metered_out)]:
        by_origin = {s["origin"]: s for s in out["od_skims"]}
        m = by_origin[0]
        r = by_origin[1]
        total_h = (
            m["mean_total_time_s"] * m["vehicles_arrived"]
            + r["mean_total_time_s"] * r["vehicles_arrived"]
        ) / 3600.0
        print(
            f"{label:<12} | mainline {m['vehicles_arrived']:>4} veh, mean {m['mean_total_time_s']:>6.1f} s "
            f"| ramp {r['vehicles_arrived']:>3} veh, mean {r['mean_total_time_s']:>6.1f} s "
            f"| total {total_h:>6.1f} veh·h"
        )

    print("\n  t (s) | occupancy L2 | set point | ramp rate (veh/h)")
    for t, occ, rate in ctrl.history:
        if int(t) % 600 == 0:
            print(f"{t:>7.0f} | {occ * 100:>11.1f} % | {14.0:>7.1f} % | {rate:>6.0f}")


if __name__ == "__main__":
    main()
