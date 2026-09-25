"""Blocked off-ramp: can a lane-resolved model keep the mainline moving?

The question the pipe partition exists to answer, put to four engines on one
scenario and one set of vehicles.

  Network   500 m three-lane approach, splitting into a three-lane through
            branch and a one-lane off-ramp. The ramp ends at a gate.
  Blockage  the gate discharges one vehicle per 100 s. In the LWR engines it
            is a link of capacity 0.01 veh/s; in SUMO micro it is a traffic
            light with 2 s of green per 100 s cycle, the usual physical cause
            of an off-ramp queue.
  Demand    through 3 600 veh/h, exit-bound 360 veh/h (9 %), over 30 minutes.

Engines:
  1. Stream Python          one stream per link, strict FIFO  (the ancestor)
  2. pipe-stream, 1 pipe    the same model, this implementation
  3. pipe-stream, pipes     one-lane exit pipe + two-lane through pipe, phi
  4. SUMO micro             car-following with explicit lanes (the witness)
  5. SUMO meso              queue per edge segment, laneless

What to look for: the exit queue is served identically by every engine (the
ramp is the bottleneck and nothing changes it), so the whole question is what
happens to the *through* traffic beside it.

    pip install eclipse-sumo numpy'<2'
    cargo build --release --no-default-features --example bench_runner
    python bench/diverge_blocked.py --stream-python ../stream-python
"""

import argparse
import copy
import json
import os
import statistics
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compare_with_stream_python as base  # noqa: E402
import compare_with_sumo as sumobench  # noqa: E402

ROOT = base.ROOT

# Worked example of the paper: per-lane triangular FD.
FD = {"u": 25.0, "w": 5.0, "kx": 0.127, "C": 0.55}
GATE_C = 0.01           # veh/s: one vehicle per 100 s
APPROACH_M = 500.0
BRANCH_M = 500.0
GATE_M = 50.0
DURATION = 1800.0
THROUGH_VPH = 3600.0
EXIT_VPH = 360.0
PHI = 0.95
CYCLE_S, GREEN_S = 100.0, 2.0   # SUMO gate signal, same 0.01 veh/s service


def scenario_inputs(template):
    """Stream Python ``Inputs``: entry 1, diverge 2, through exit 3, gate 4, ramp exit 5."""
    inputs = base._base(template, DURATION)
    nodes = {1: {}, 2: {}, 3: {}, 4: {}, 5: {}}
    links = {
        1: base._link(1, 2, APPROACH_M, 3, FD["u"], base._pts(0, 0, APPROACH_M, 0)),
        2: base._link(2, 3, BRANCH_M, 3, FD["u"],
                      base._pts(APPROACH_M, 0, APPROACH_M + BRANCH_M, 0)),
        3: base._link(2, 4, BRANCH_M - GATE_M, 1, FD["u"],
                      base._pts(APPROACH_M, 0, APPROACH_M + BRANCH_M - GATE_M, -BRANCH_M)),
        4: base._link(4, 5, GATE_M, 1, FD["u"],
                      base._pts(APPROACH_M + BRANCH_M - GATE_M, -BRANCH_M,
                                APPROACH_M + BRANCH_M, -BRANCH_M)),
    }
    for lid, link in links.items():
        link["FD"] = dict(FD)
        if lid == 4:                      # the gate: one vehicle per 100 s
            link["FD"]["C"] = GATE_C
    base._wire(nodes, links)
    inputs.update(Nodes=nodes, Links=links, Entries={1: {}}, Exits={3: {}, 5: {}},
                  Demand=np.array([[1, 1, 1, 3, THROUGH_VPH],
                                   [1, 1, 1, 5, EXIT_VPH]]))
    return inputs


def with_pipes(scenario, phi=PHI):
    """Same scenario, approach partitioned: pipe 0 = shoulder (exit), pipe 1 = through."""
    out = copy.deepcopy(scenario)
    approach, through, ramp = 0, 1, 2      # link ids after to_pipe_stream's remap
    link = out["links"][approach]
    link["pipe_specs"] = [{"lanes": 1}, {"lanes": 2}]
    link["moves"] = {str(ramp): [0], str(through): [1]}
    link["friction"] = phi
    return out


# ── SUMO with a signalised gate ──────────────────────────────────────────────

def write_sumo(scenario, workdir, name):
    stem = os.path.join(workdir, name)
    gate_node = scenario["links"][3]["node_up"]      # upstream node of the gate link

    nodes = ET.Element("nodes")
    for n in scenario["nodes"]:
        x, y = n["points"]
        kind = "traffic_light" if n["id"] == gate_node else "priority"
        ET.SubElement(nodes, "node", id=f"n{n['id']}", x=repr(float(x)),
                      y=repr(float(y)), type=kind)
    edges = ET.Element("edges")
    for l in scenario["links"]:
        shape = " ".join(f"{x!r},{y!r}" for x, y in l["points"])
        ET.SubElement(edges, "edge", id=f"e{l['id']}", **{"from": f"n{l['node_up']}"},
                      to=f"n{l['node_down']}", numLanes=str(l["num_lanes"]),
                      speed=repr(l["speed"]), length=repr(l["length"]), shape=shape)
    # Explicit connections: the shoulder lane feeds the ramp and the through
    # branch; the two left lanes feed the through branch only. Exit-bound
    # vehicles must therefore reach lane 0, as at a real off-ramp.
    cons = ET.Element("connections")
    for from_lane, to_lane in ((0, 0), (1, 1), (2, 2)):
        ET.SubElement(cons, "connection", **{"from": "e0", "to": "e1"},
                      fromLane=str(from_lane), toLane=str(to_lane))
    ET.SubElement(cons, "connection", **{"from": "e0", "to": "e2"},
                  fromLane="0", toLane="0")
    ET.SubElement(cons, "connection", **{"from": "e2", "to": "e3"},
                  fromLane="0", toLane="0")
    for el, suffix in ((nodes, "nod"), (edges, "edg"), (cons, "con")):
        ET.ElementTree(el).write(f"{stem}.{suffix}.xml")
    subprocess.run([sumobench.tool("netconvert"),
                    "--node-files", f"{stem}.nod.xml",
                    "--edge-files", f"{stem}.edg.xml",
                    "--connection-files", f"{stem}.con.xml",
                    "--output-file", f"{stem}.net.xml",
                    "--no-turnarounds", "true", "--no-internal-links", "true",
                    "--no-warnings", "true"],
                   check=True, stdout=subprocess.DEVNULL)

    # The gate signal: 2 s green per 100 s, matching the 0.01 veh/s gate link.
    add = ET.Element("additional")
    tl = ET.SubElement(add, "tlLogic", id=f"n{gate_node}", type="static",
                       programID="gate", offset="0")
    ET.SubElement(tl, "phase", duration=repr(GREEN_S), state="G")
    ET.SubElement(tl, "phase", duration=repr(CYCLE_S - GREEN_S), state="r")
    ET.ElementTree(add).write(f"{stem}.add.xml")

    kx, c, u = FD["kx"], FD["C"], FD["u"]
    tau = 1.0 / c - (1.0 / kx) / u
    routes = ET.Element("routes")
    ET.SubElement(routes, "vType", id="car", length=repr(1.0 / kx - 2.5),
                  minGap="2.5", tau=repr(tau), sigma="0",
                  speedFactor="1", speedDev="0")
    for v in sorted(scenario["vehicles"], key=lambda v: (v["start_time"], v["id"])):
        veh = ET.SubElement(routes, "vehicle", id=str(v["id"]), type="car",
                            depart=repr(v["start_time"]), departLane="best",
                            departSpeed="max")
        ET.SubElement(veh, "route", edges=" ".join(f"e{e}" for e in v["path"]))
    ET.ElementTree(routes).write(f"{stem}.rou.xml")
    return stem


def run_sumo(stem, meso):
    tag = "meso" if meso else "micro"
    trips = f"{stem}.{tag}.tripinfo.xml"
    cmd = [sumobench.tool("sumo"), "-n", f"{stem}.net.xml", "-r", f"{stem}.rou.xml",
           "-a", f"{stem}.add.xml", "--begin", "0", "--end", repr(DURATION),
           "--step-length", "1", "--tripinfo-output", trips,
           "--no-step-log", "true", "--no-warnings", "true", "--seed", "42",
           "--time-to-teleport", "-1"]
    if meso:
        # Without junction control the mesoscopic model ignores traffic lights
        # outright, and the gate would not block at all.
        cmd += ["--mesosim", "true", "--meso-junction-control", "true"]
    subprocess.run(cmd, check=True, capture_output=True, text=True)
    return trips


# ── Metrics ──────────────────────────────────────────────────────────────────

def kinds(scenario):
    """vehicle id -> 'through' | 'exit', by route."""
    return {v["id"]: ("exit" if len(v["path"]) == 3 else "through")
            for v in scenario["vehicles"]}


def free_flow(kind):
    metres = APPROACH_M + (BRANCH_M if kind == "through" else BRANCH_M)
    return metres / FD["u"]


def summarise(label, per_vehicle, scenario):
    """per_vehicle: id -> (travel time, or None if it never finished)."""
    kind = kinds(scenario)
    out = {"engine": label}
    for k in ("through", "exit"):
        done = [t for vid, t in per_vehicle.items() if kind[vid] == k and t is not None]
        total = sum(1 for vid in kind if kind[vid] == k)
        out[k] = {
            "completed": len(done),
            "demanded": total,
            "throughput_vph": 3600.0 * len(done) / DURATION,
            "mean_tt": float(np.mean(done)) if done else None,
            "mean_delay": float(np.mean(done)) - free_flow(k) if done else None,
        }
    return out


def lwr_travel_times(node_times, scenario):
    out = {}
    for v, times in zip(scenario["vehicles"], node_times):
        done = len(times) == len(v["path"]) + 1
        out[v["id"]] = (times[-1] - v["start_time"]) if done else None
    return out


def sumo_travel_times(trips, scenario):
    out = {v["id"]: None for v in scenario["vehicles"]}
    for trip in ET.parse(trips).getroot().iter("tripinfo"):
        out[int(trip.get("id"))] = float(trip.get("arrival")) - float(trip.get("depart"))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--stream-python", default=os.path.join(ROOT, "..", "stream-python"))
    ap.add_argument("--out", default=os.path.join(ROOT, "bench", "results"))
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--phi-sweep", type=float, nargs="*",
                    default=[1.0, 0.95, 0.9, 0.8, 0.7, 0.6, 0.5],
                    help="friction values to run with the same partition")
    args = ap.parse_args()

    sys.path.insert(0, os.path.abspath(args.stream_python))
    if not os.path.exists(base.RUNNER):
        sys.exit("bench_runner not built: cargo build --release "
                 "--no-default-features --example bench_runner")
    os.makedirs(args.out, exist_ok=True)
    template = np.load(os.path.join(args.stream_python, "example", "inputs.npy"),
                       allow_pickle=True).item()

    S0 = base.prepare_python(scenario_inputs(template), 42)
    scenario, vids = base.to_pipe_stream(S0)
    print(f"{len(scenario['vehicles'])} vehicles "
          f"({sum(1 for k in kinds(scenario).values() if k == 'exit')} exit-bound)")

    rows = []

    S_py, _ = base.run_python(copy.deepcopy(S0), 42)
    py_tt = {}
    for i, vid in enumerate(vids):
        t = np.asarray(S_py["Vehicles"][vid]["NodeTimes"], dtype=float)
        v = scenario["vehicles"][i]
        done = len(t) == len(v["path"]) + 1 and np.all(t > 0)
        py_tt[v["id"]] = (t[-1] - v["start_time"]) if done else None
    rows.append(summarise("Stream Python (1 stream)", py_tt, scenario))

    single = base.run_rust(scenario, args.out, "diverge_blocked_1pipe", args.repeats)
    rows.append(summarise("pipe-stream (1 pipe)",
                          lwr_travel_times(single["node_times"], scenario), scenario))

    piped = with_pipes(scenario)
    multi = base.run_rust(piped, args.out, "diverge_blocked_pipes", args.repeats)
    rows.append(summarise(f"pipe-stream (pipes, phi={PHI})",
                          lwr_travel_times(multi["node_times"], piped), piped))

    stem = write_sumo(scenario, args.out, "diverge_blocked")
    for meso in (False, True):
        trips = run_sumo(stem, meso)
        rows.append(summarise(f"SUMO {'meso' if meso else 'micro'}",
                              sumo_travel_times(trips, scenario), scenario))

    sweep = []
    for phi in args.phi_sweep:
        s = with_pipes(scenario, phi)
        r = base.run_rust(s, args.out, f"diverge_blocked_phi{phi}", 1)
        row = summarise(f"pipe-stream (phi={phi:g})",
                        lwr_travel_times(r["node_times"], s), s)
        row["phi"] = phi
        sweep.append(row)

    def show(rs):
        for r in rs:
            delay = r["through"]["mean_delay"]
            exit_delay = r["exit"]["mean_delay"]
            print(f"{r['engine']:<30}{r['through']['throughput_vph']:>14.0f}"
                  f"{(delay if delay is not None else float('nan')):>14.1f}s"
                  f"{r['exit']['completed']:>13}"
                  f"{(exit_delay if exit_delay is not None else float('nan')):>11.1f}s")

    head = (f"{'engine':<30}{'through veh/h':>14}{'through delay':>15}"
            f"{'exit served':>13}{'exit delay':>12}")
    print("\n" + head)
    print("-" * len(head))
    show(rows)
    if sweep:
        print("\nfriction sweep (same partition, phi only):")
        print("-" * len(head))
        show(sweep)
        micro = next((r for r in rows if r["engine"] == "SUMO micro"), None)
        if micro and micro["through"]["mean_delay"] is not None:
            target = micro["through"]["mean_delay"]
            best = min(sweep, key=lambda r: abs((r["through"]["mean_delay"] or 0) - target))
            print(f"\nclosest to SUMO micro's {target:.1f} s through delay: "
                  f"phi = {best['phi']:g} ({best['through']['mean_delay']:.1f} s, "
                  f"{best['through']['throughput_vph']:.0f} veh/h against "
                  f"{micro['through']['throughput_vph']:.0f})")

    path = os.path.join(args.out, "diverge_blocked.json")
    with open(path, "w") as f:
        json.dump({"setup": {"through_vph": THROUGH_VPH, "exit_vph": EXIT_VPH,
                             "duration_s": DURATION, "gate_veh_per_s": GATE_C,
                             "phi": PHI, "fd": FD, "cycle_s": CYCLE_S,
                             "green_s": GREEN_S},
                   "results": rows, "phi_sweep": sweep}, f, indent=2)
    print(f"\nWritten to {path}")


if __name__ == "__main__":
    main()
