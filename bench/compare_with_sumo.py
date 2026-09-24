"""Benchmark pipe-stream against Eclipse SUMO (microscopic and mesoscopic).

Same scenarios and same vehicles as ``compare_with_stream_python.py``: the
demand is assigned once by Stream Python (paths, entry times), then exported
to pipe-stream and to SUMO (plain nodes/edges built by ``netconvert``, one
route per vehicle). SUMO runs twice: microscopic (Krauss car-following,
1 s step) and mesoscopic (``--mesosim``, queue-based). The engines do not
share a traffic model, so they are compared on aggregate outputs only:
vehicles completed and mean travel times.

Usage (from the stream-pipes-rust root)::

    pip install eclipse-sumo            # provides sumo and netconvert
    cargo build --release --no-default-features --example bench_runner
    python3 bench/compare_with_sumo.py --stream-python ../stream-python

See ``bench/README.md``.
"""

import argparse
import json
import os
import platform
import re
import shutil
import statistics
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compare_with_stream_python as base  # noqa: E402

ROOT = base.ROOT


def tool(name):
    """sumo / netconvert: $SUMO_HOME/bin, then the pip package, then PATH."""
    if os.environ.get("SUMO_HOME"):
        path = os.path.join(os.environ["SUMO_HOME"], "bin", name)
        if os.path.exists(path):
            return path
    try:
        import sumo  # the eclipse-sumo pip package
        path = os.path.join(sumo.SUMO_HOME, "bin", name)
        if os.path.exists(path):
            return path
    except ImportError:
        pass
    path = shutil.which(name)
    if not path:
        sys.exit(f"{name} not found: pip install eclipse-sumo, or set SUMO_HOME")
    return path


# ── Export to SUMO ───────────────────────────────────────────────────────────

def write_sumo(scenario, workdir, name):
    """Plain XML network + routes carrying the assigned vehicles."""
    stem = os.path.join(workdir, name)
    nodes = ET.Element("nodes")
    for n in scenario["nodes"]:
        x, y = n["points"]
        ET.SubElement(nodes, "node", id=f"n{n['id']}", x=repr(float(x)),
                      y=repr(float(y)), type="priority")
    edges = ET.Element("edges")
    for l in scenario["links"]:
        shape = " ".join(f"{x!r},{y!r}" for x, y in l["points"])
        ET.SubElement(edges, "edge", id=f"e{l['id']}", **{"from": f"n{l['node_up']}"},
                      to=f"n{l['node_down']}", numLanes=str(l["num_lanes"]),
                      speed=repr(l["speed"]), length=repr(l["length"]), shape=shape)
    ET.ElementTree(nodes).write(f"{stem}.nod.xml")
    ET.ElementTree(edges).write(f"{stem}.edg.xml")
    subprocess.run([tool("netconvert"), "--node-files", f"{stem}.nod.xml",
                    "--edge-files", f"{stem}.edg.xml", "--output-file", f"{stem}.net.xml",
                    "--no-turnarounds", "true", "--no-internal-links", "true",
                    "--no-warnings", "true"],
                   check=True, stdout=subprocess.DEVNULL)

    # One vehicle type calibrated on the scenario's (median) triangular
    # fundamental diagram: jam spacing length + minGap = 1/kx, and Krauss
    # saturation headway tau + (length + minGap)/u = 1/C, so a lane carries
    # the same capacity as in Stream. No driver imperfection (sigma) and no
    # speed distribution: every vehicle drives at the speed limit, as in Stream.
    kx, c, u = (statistics.median(l["fd"][k] for l in scenario["links"])
                for k in ("kx", "c", "u"))
    tau = 1.0 / c - (1.0 / kx) / u
    routes = ET.Element("routes")
    ET.SubElement(routes, "vType", id="car", length=repr(1.0 / kx - 2.5),
                  minGap="2.5", tau=repr(tau), sigma="0",
                  speedFactor="1", speedDev="0")
    order = sorted(scenario["vehicles"], key=lambda v: (v["start_time"], v["id"]))
    for v in order:
        veh = ET.SubElement(routes, "vehicle", id=str(v["id"]), type="car",
                            depart=repr(v["start_time"]), departLane="best",
                            departSpeed="max")
        ET.SubElement(veh, "route", edges=" ".join(f"e{e}" for e in v["path"]))
    ET.ElementTree(routes).write(f"{stem}.rou.xml")
    return stem


def run_sumo(stem, scenario, meso, repeats):
    """Median SUMO simulation time (its own 'Duration' statistic, which
    excludes network loading) and the per-vehicle trip info of the last run."""
    begin = scenario["start_time"]
    end = begin + scenario["duration"]
    tag = "meso" if meso else "micro"
    trips = f"{stem}.{tag}.tripinfo.xml"
    cmd = [tool("sumo"), "-n", f"{stem}.net.xml", "-r", f"{stem}.rou.xml",
           "--begin", repr(begin), "--end", repr(end), "--step-length", "1",
           "--tripinfo-output", trips, "--duration-log.statistics", "true",
           "--no-step-log", "true", "--no-warnings", "true", "--seed", "42",
           # No teleporting: a jam must stay a jam, as in the other engines.
           "--time-to-teleport", "-1"]
    if meso:
        cmd += ["--mesosim", "true"]
    sim_s, wall_s = [], []
    for _ in range(repeats):
        t = time.perf_counter()
        out = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
        wall_s.append(time.perf_counter() - t)
        m = re.search(r"Duration:\s*([\d.,]+)\s*(ms|s)", out)
        if not m:
            sys.exit(f"cannot read SUMO duration from:\n{out}")
        value = float(m.group(1).replace(",", ""))
        sim_s.append(value / 1000 if m.group(2) == "ms" else value)
    return statistics.median(sim_s), statistics.median(wall_s), trips, out


def sumo_metrics(trips, scenario):
    start = {str(v["id"]): v["start_time"] for v in scenario["vehicles"]}
    tt, tt_od, steps = [], [], 0
    for trip in ET.parse(trips).getroot().iter("tripinfo"):
        arrival, depart = float(trip.get("arrival")), float(trip.get("depart"))
        tt.append(arrival - depart)
        tt_od.append(arrival - start[trip.get("id")])
    return {"completed": len(tt),
            "mean_tt": float(np.mean(tt)) if tt else None,
            "mean_tt_from_demand": float(np.mean(tt_od)) if tt_od else None}


def rust_metrics(rust, scenario):
    tt, tt_od = [], []
    for v, times in zip(scenario["vehicles"], rust["node_times"]):
        if len(times) == len(v["path"]) + 1:
            tt.append(times[-1] - times[0])
            tt_od.append(times[-1] - v["start_time"])
    return {"completed": len(tt),
            "mean_tt": float(np.mean(tt)) if tt else None,
            "mean_tt_from_demand": float(np.mean(tt_od)) if tt_od else None}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--stream-python", default=os.path.join(ROOT, "..", "stream-python"),
                    help="path to a stream-python checkout (scenario building and assignment)")
    ap.add_argument("--scenarios", nargs="*", default=list(base.SCENARIOS),
                    choices=list(base.SCENARIOS))
    ap.add_argument("--repeats", type=int, default=5, help="runs per engine (median)")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default=os.path.join(ROOT, "bench", "results"))
    args = ap.parse_args()

    sys.path.insert(0, os.path.abspath(args.stream_python))
    if not os.path.exists(base.RUNNER):
        sys.exit("bench_runner not built: cargo build --release --no-default-features "
                 "--example bench_runner")
    os.makedirs(args.out, exist_ok=True)
    template = np.load(os.path.join(args.stream_python, "example", "inputs.npy"),
                       allow_pickle=True).item()
    version = subprocess.run([tool("sumo"), "--version"], capture_output=True,
                             text=True).stdout.splitlines()[0]

    results = []
    for name in args.scenarios:
        print(f"── {name}", flush=True)
        S0 = base.prepare_python(base.SCENARIOS[name](template), args.seed)
        scenario, _ = base.to_pipe_stream(S0)
        rust = base.run_rust(scenario, args.out, name, args.repeats)
        ru_s = statistics.median(a + b for a, b in zip(rust["init_s"], rust["run_s"]))
        stem = write_sumo(scenario, args.out, name)
        row = {"scenario": name, "nodes": len(scenario["nodes"]),
               "links": len(scenario["links"]), "vehicles": len(scenario["vehicles"]),
               "ru_s": ru_s, "rust": rust_metrics(rust, scenario)}
        for mode in ("micro", "meso"):
            sim_s, wall_s, trips, out = run_sumo(stem, scenario, mode == "meso",
                                                 args.repeats)
            row[f"{mode}_s"], row[f"{mode}_wall_s"] = sim_s, wall_s
            row[mode] = sumo_metrics(trips, scenario)
            waiting = re.search(r"Waiting:\s*(\d+)", out)
            row[mode]["not_inserted"] = int(waiting.group(1)) if waiting else None
        results.append(row)
        print(f"   pipe-stream {ru_s * 1e3:8.2f} ms | SUMO micro {row['micro_s']:7.3f} s "
              f"(×{row['micro_s'] / ru_s:,.0f}) | SUMO meso {row['meso_s']:7.3f} s "
              f"(×{row['meso_s'] / ru_s:,.0f}) | mean tt {row['rust']['mean_tt']:.1f} / "
              f"{row['micro']['mean_tt']:.1f} / {row['meso']['mean_tt']:.1f} s", flush=True)

    meta = {"sumo": version, "python": platform.python_version(),
            "platform": platform.platform(), "seed": args.seed, "repeats": args.repeats}
    path = os.path.join(args.out, "sumo_results.json")
    with open(path, "w") as f:
        json.dump({"meta": meta, "results": results}, f, indent=2)
    print(f"Results written to {path}")


if __name__ == "__main__":
    main()
