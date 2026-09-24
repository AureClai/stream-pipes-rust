"""Benchmark Stream Python (reference engine) against pipe-stream (Rust).

Both engines simulate *exactly the same vehicles*: the scenario is built as a
Stream Python ``Inputs`` dict, completed and assigned by Stream Python
(shortest paths, entry times), then exported to a compiled pipe-stream
scenario with those vehicles. Stream Python's event loop
(``main_simulation_meso``) is timed in-process; pipe-stream is timed by the
``bench_runner`` example (``Simulation::new`` + ``run``), which also returns
every vehicle's node passage times so the trajectories can be compared.

Usage (from the stream-pipes-rust root)::

    cargo build --release --no-default-features --example bench_runner
    python3 bench/compare_with_stream_python.py --stream-python ../stream-python

See ``bench/README.md`` for the scenario list and how to read the results.
"""

import argparse
import contextlib
import copy
import io
import json
import os
import platform
import random
import statistics
import subprocess
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUNNER = os.path.join(ROOT, "target", "release", "examples", "bench_runner")

FD = {"u": 25.0, "C": 0.5, "kx": 0.15, "w": 5.0}
LINK_DEFAULTS = {"Capacity": None, "road_type": 1, "Priority": None, "name": None}


# ── Scenario builders (Stream Python ``Inputs`` dicts) ───────────────────────

def _base(template, duration, periods_end=None):
    inputs = {
        "RoadTypes": template["RoadTypes"],
        "VehicleClass": {1: {"Name": "VL"}},
        "Regulations": {},
        "General": {"SimulationDuration": [0.0, float(duration)],
                    "ActiveUpStreamCapacity": False},
        "Periods": {1: {"start": 0}, 2: {"start": periods_end or duration}},
    }
    return inputs


def _pts(x1, y1, x2, y2):
    return np.array([[x1, x2], [y1, y2]], dtype=float)


def _link(up, down, length, lanes, speed, pts):
    return {**LINK_DEFAULTS, "NodeUpID": up, "NodeDownID": down,
            "Length": float(length), "NumLanes": lanes, "Speed": float(speed),
            "FD": dict(FD), "Points": pts}


def _wire(nodes, links):
    """Fill IncomingLinksID / OutgoingLinksID and node types from links."""
    for nid in nodes:
        nodes[nid].update({"IncomingLinksID": [], "OutgoingLinksID": []})
    for lid in sorted(links):
        nodes[links[lid]["NodeUpID"]]["OutgoingLinksID"].append(lid)
        nodes[links[lid]["NodeDownID"]]["IncomingLinksID"].append(lid)
    for n in nodes.values():
        n["IncomingLinksID"] = np.array(n["IncomingLinksID"], dtype=int)
        n["OutgoingLinksID"] = np.array(n["OutgoingLinksID"], dtype=int)
        n["NumIncomingLinks"] = len(n["IncomingLinksID"])
        n["NumOutgoingLinks"] = len(n["OutgoingLinksID"])


def chain(template, n_links, flow=1500.0, duration=3600.0, length=100.0):
    """Linear corridor of ``n_links`` short links, free flow."""
    inputs = _base(template, duration)
    nodes = {i: {} for i in range(n_links + 1)}
    links = {i: _link(i, i + 1, length, 1, 25.0,
                      _pts(i * length, 0, (i + 1) * length, 0))
             for i in range(n_links)}
    _wire(nodes, links)
    inputs.update(Nodes=nodes, Links=links, Entries={0: {}},
                  Exits={n_links: {}},
                  Demand=np.array([[1, 1, 0, n_links, flow]]))
    return inputs


def bottleneck(template):
    """2-lane link feeding a 1-lane link, demand above the 1-lane capacity."""
    inputs = _base(template, 3600.0)
    nodes = {1: {}, 2: {}, 3: {}}
    links = {1: _link(1, 2, 1000, 2, 25, _pts(0, 0, 1000, 0)),
             2: _link(2, 3, 1000, 1, 25, _pts(1000, 0, 2000, 0))}
    _wire(nodes, links)
    inputs.update(Nodes=nodes, Links=links, Entries={1: {}}, Exits={3: {}},
                  Demand=np.array([[1, 1, 1, 3, 2500.0]]))
    return inputs


def diverge(template):
    """2-lane link splitting into two 1-lane branches, both near capacity."""
    inputs = _base(template, 3600.0)
    nodes = {1: {}, 2: {}, 3: {}, 4: {}}
    links = {1: _link(1, 2, 1000, 2, 25, _pts(0, 0, 1000, 0)),
             2: _link(2, 3, 1000, 1, 25, _pts(1000, 0, 2000, 0)),
             3: _link(2, 4, 1000, 1, 25, _pts(1000, 0, 1000, 1000))}
    _wire(nodes, links)
    inputs.update(Nodes=nodes, Links=links, Entries={1: {}},
                  Exits={3: {}, 4: {}},
                  Demand=np.array([[1, 1, 1, 3, 800.0], [1, 1, 1, 4, 800.0]]))
    return inputs


def merge(template):
    """Two 1-lane on-ramps merging into a 1-lane link (congested merge)."""
    inputs = _base(template, 3600.0)
    nodes = {1: {}, 2: {}, 3: {}, 4: {}}
    links = {1: _link(1, 3, 1000, 1, 25, _pts(0, 0, 1000, 0)),
             2: _link(2, 3, 1000, 1, 25, _pts(0, 500, 1000, 0)),
             3: _link(3, 4, 1000, 1, 25, _pts(1000, 0, 2000, 0))}
    _wire(nodes, links)
    inputs.update(Nodes=nodes, Links=links, Entries={1: {}, 2: {}},
                  Exits={4: {}},
                  Demand=np.array([[1, 1, 1, 4, 1000.0], [1, 1, 2, 4, 1000.0]]))
    return inputs


def grid(template, size, flow=1500.0, duration=3600.0):
    """size×size bidirectional Manhattan grid, two crossing diagonal flows."""
    inputs = _base(template, duration)
    nodes = {r * size + c: {} for r in range(size) for c in range(size)}
    links, lid = {}, 1
    for r in range(size):
        for c in range(size):
            u = r * size + c
            for dr, dc in ((0, 1), (1, 0)):
                if r + dr >= size or c + dc >= size:
                    continue
                v = (r + dr) * size + c + dc
                for a, b in ((u, v), (v, u)):
                    ra, ca = divmod(a, size)
                    rb, cb = divmod(b, size)
                    links[lid] = _link(a, b, 200, 1, 15,
                                       _pts(ca * 200, ra * 200, cb * 200, rb * 200))
                    lid += 1
    _wire(nodes, links)
    border = {n: {} for n in nodes
              if n // size in (0, size - 1) or n % size in (0, size - 1)}
    last = size * size - 1
    inputs.update(Nodes=nodes, Links=links, Entries=dict(border),
                  Exits=dict(border),
                  Demand=np.array([[1, 1, 0, last, flow],
                                   [1, 1, size - 1, last - size + 1, flow]]))
    return inputs


def real_network(template):
    """The 12-node motorway interchange shipped with Stream Python."""
    return copy.deepcopy(template)


SCENARIOS = {
    "bottleneck": bottleneck,
    "diverge": diverge,
    "merge": merge,
    "real_network": real_network,
    "chain_10": lambda t: chain(t, 10),
    "chain_50": lambda t: chain(t, 50),
    "chain_100": lambda t: chain(t, 100),
    "grid_3x3": lambda t: grid(t, 3),
    "grid_5x5": lambda t: grid(t, 5),
    "grid_8x8": lambda t: grid(t, 8),
}


# ── Stream Python side ───────────────────────────────────────────────────────

def prepare_python(inputs, seed):
    from stream.initialization.validate_and_complete_scenario import validate_and_complete_scenario
    from stream.initialization.assignment import assignment
    from stream.initialization.initialize_simulation import initialize_simulation

    random.seed(seed)
    np.random.seed(seed)
    with contextlib.redirect_stdout(io.StringIO()):
        S = validate_and_complete_scenario(inputs)
        S = assignment(S)
        S = initialize_simulation(S)
    return S


def run_python(S, seed):
    from stream.simulation.main_simulation_meso import main_simulation_meso

    random.seed(seed)  # congested merges draw from np.random
    np.random.seed(seed)
    end = float(S["General"]["SimulationDuration"][1])
    with contextlib.redirect_stdout(io.StringIO()):
        t = time.perf_counter()
        S = main_simulation_meso(S, end)
        elapsed = time.perf_counter() - t
    return S, elapsed


class _FirstCandidate:
    """``np.random`` stand-in: the Daganzo draw always picks the first
    congested candidate, i.e. the lowest incoming-link index."""

    @staticmethod
    def rand():
        return 0.0


@contextlib.contextmanager
def aligned_python():
    """Diagnostic variant of Stream Python with pipe-stream's two node rules.

    1. Deterministic merge: when every candidate at a merge is congested, the
       lowest incoming-link index passes (Stream Python draws at random with
       the Daganzo coefficients).
    2. Causal passages: a vehicle cannot cross a node before the previous
       vehicle of the same incoming link. With ``ActiveUpStreamCapacity``
       off, Stream Python has no such bound, so behind a head vehicle blocked
       at a diverge the next one can be dated *before* the current event
       (processed after it, but time-stamped earlier).

    Used only for the agreement columns, never for timings.
    """
    import stream.simulation.main_simulation_meso as M

    select, supply = M.select_next_event, M.next_supply_time

    def deterministic_select(*args, **kwargs):
        saved = M.np.random
        M.np.random = _FirstCandidate
        try:
            return select(*args, **kwargs)
        finally:
            M.np.random = saved

    def causal_supply(Events, NodeID, General):
        out = supply(Events, NodeID, General)
        up = np.array(out["Up"], dtype=float)
        for ex in Events[NodeID]["Exits"].values():
            for prev, t in zip(ex["PreviousLinkID"], ex["Time"]):
                up[int(prev)] = max(up[int(prev)], t)
        out["Up"] = up
        return out

    M.select_next_event, M.next_supply_time = deterministic_select, causal_supply
    try:
        yield
    finally:
        M.select_next_event, M.next_supply_time = select, supply


# ── Export to pipe-stream ────────────────────────────────────────────────────

def to_pipe_stream(S):
    """Compiled pipe-stream scenario carrying Stream Python's vehicles."""
    node_ids = sorted(S["Nodes"])
    link_ids = sorted(S["Links"])
    nmap = {nid: i for i, nid in enumerate(node_ids)}
    lmap = {lid: i for i, lid in enumerate(link_ids)}

    links = []
    for lid in link_ids:
        l = S["Links"][lid]
        pts = np.asarray(l["Points"], dtype=float)
        links.append({
            "id": lmap[lid], "node_up": nmap[l["NodeUpID"]],
            "node_down": nmap[l["NodeDownID"]], "length": float(l["Length"]),
            "speed": float(l["Speed"]), "num_lanes": int(l["NumLanes"]),
            "capacity": float(l["Capacity"]),
            "fd": {k.lower(): float(l["FD"][k]) for k in ("u", "w", "kx", "C")},
            "points": [[float(x), float(y)] for x, y in zip(pts[0], pts[1])],
            "priority": 1.0,
        })

    nodes = []
    for nid in node_ids:
        n = S["Nodes"][nid]
        inc = [lmap[x] for x in n["IncomingLinksID"]]
        out = [lmap[x] for x in n["OutgoingLinksID"]]
        kind = "Entry" if not inc else "Exit" if not out else "Internal"
        p = links[out[0]]["points"][0] if out else links[inc[0]]["points"][-1]
        nodes.append({"id": nmap[nid], "node_type": kind, "incoming_links": inc,
                      "outgoing_links": out, "points": p, "signals": []})

    # pipe-stream indexes vehicles by id: ids must be 0..n-1.
    vids = sorted(S["Vehicles"])
    vehicles = []
    for i, vid in enumerate(vids):
        v = S["Vehicles"][vid]
        path = [lmap[x] for x in v["Path"]]
        vehicles.append({"id": i, "class_id": 0, "path": path,
                         "start_time": float(v["NetworkArrivalTime"]),
                         "origin": links[path[0]]["node_up"],
                         "destination": links[path[-1]]["node_down"]})

    start, end = (float(x) for x in S["General"]["SimulationDuration"])
    scenario = {"nodes": nodes, "links": links, "vehicles": vehicles,
                "demand": [], "start_time": start, "duration": end - start}
    return scenario, vids


def run_rust(scenario, workdir, name, repeats):
    src = os.path.join(workdir, f"{name}.scenario.json")
    dst = os.path.join(workdir, f"{name}.rust.json")
    with open(src, "w") as f:
        json.dump(scenario, f)
    subprocess.run([RUNNER, src, dst, str(repeats)], check=True)
    with open(dst) as f:
        return json.load(f)


# ── Comparison ───────────────────────────────────────────────────────────────

def compare(S_py, vids, rust):
    py_times = [np.asarray(S_py["Vehicles"][v]["NodeTimes"], dtype=float) for v in vids]
    ru_times = [np.asarray(t, dtype=float) for t in rust["node_times"]]
    starts = [float(S_py["Vehicles"][v]["NetworkArrivalTime"]) for v in vids]
    n_nodes = [len(t) for t in py_times]

    def done(t, n):
        return len(t) == n and np.all(t > 0)

    py_passages = sum(int(np.count_nonzero(t > 0)) for t in py_times)
    ru_passages = sum(len(t) for t in ru_times)
    diffs, py_tt, ru_tt, both_tt, py_od, ru_od = [], [], [], [], [], []
    for p, r, n, t0 in zip(py_times, ru_times, n_nodes, starts):
        k = min(int(np.count_nonzero(p > 0)), len(r))
        if k:
            diffs.append(np.abs(p[:k] - r[:k]))
        if done(p, n):
            py_tt.append(p[-1] - p[0])
            py_od.append(p[-1] - t0)
        if len(r) == n:
            ru_tt.append(r[-1] - r[0])
            ru_od.append(r[-1] - t0)
        if done(p, n) and len(r) == n:
            both_tt.append((p[-1] - p[0], r[-1] - r[0]))
    d = np.concatenate(diffs) if diffs else np.zeros(0)
    return {
        "vehicles": len(vids),
        "py_completed": len(py_tt), "ru_completed": len(ru_tt),
        "py_passages": py_passages, "ru_passages": ru_passages,
        "py_mean_tt": float(np.mean(py_tt)) if py_tt else None,
        "ru_mean_tt": float(np.mean(ru_tt)) if ru_tt else None,
        # Including the wait at the entry before admission on the network.
        "py_mean_tt_from_demand": float(np.mean(py_od)) if py_od else None,
        "ru_mean_tt_from_demand": float(np.mean(ru_od)) if ru_od else None,
        "passage_mae": float(d.mean()) if d.size else None,
        "passage_max_err": float(d.max()) if d.size else None,
        "passage_exact_pct": float(100 * np.mean(d < 1e-6)) if d.size else None,
        "tt_mae": float(np.mean([abs(a - b) for a, b in both_tt])) if both_tt else None,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--stream-python", default=os.path.join(ROOT, "..", "stream-python"),
                    help="path to a stream-python checkout")
    ap.add_argument("--scenarios", nargs="*", default=list(SCENARIOS),
                    choices=list(SCENARIOS))
    ap.add_argument("--repeats", type=int, default=5, help="pipe-stream repeats (median)")
    ap.add_argument("--py-repeats", type=int, default=1, help="Stream Python repeats (median)")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--aligned", action="store_true",
                    help="also compare against Stream Python with pipe-stream's "
                         "merge and causality rules (see aligned_python)")
    ap.add_argument("--out", default=os.path.join(ROOT, "bench", "results"))
    args = ap.parse_args()

    sys.path.insert(0, os.path.abspath(args.stream_python))
    if not os.path.exists(RUNNER):
        sys.exit("bench_runner not built: cargo build --release --no-default-features "
                 "--example bench_runner")
    os.makedirs(args.out, exist_ok=True)
    template = np.load(os.path.join(args.stream_python, "example", "inputs.npy"),
                       allow_pickle=True).item()

    results = []
    for name in args.scenarios:
        print(f"── {name}", flush=True)
        S0 = prepare_python(SCENARIOS[name](template), args.seed)
        scenario, vids = to_pipe_stream(S0)

        py_s, S_py = [], None
        for _ in range(args.py_repeats):
            S_py, t = run_python(copy.deepcopy(S0), args.seed)
            py_s.append(t)
        rust = run_rust(scenario, args.out, name, args.repeats)

        ru_run = statistics.median(rust["run_s"])
        ru_total = statistics.median(a + b for a, b in zip(rust["init_s"], rust["run_s"]))
        py = statistics.median(py_s)
        row = {
            "scenario": name, "nodes": len(scenario["nodes"]),
            "links": len(scenario["links"]), "py_s": py,
            "ru_run_s": ru_run, "ru_total_s": ru_total,
            "speedup": py / ru_total if ru_total > 0 else None,
            "ru_events": rust["events"],
            **compare(S_py, vids, rust),
        }
        if args.aligned:
            with aligned_python():
                S_al, _ = run_python(copy.deepcopy(S0), args.seed)
            row["aligned"] = compare(S_al, vids, rust)
        results.append(row)
        print(f"   Python {py:9.3f} s | pipe-stream {ru_total * 1e3:9.3f} ms "
              f"| ×{row['speedup']:,.0f} | {row['vehicles']} veh "
              f"| passage MAE {row['passage_mae']:.3g} s", flush=True)

    meta = {
        "python": platform.python_version(), "numpy": np.__version__,
        "platform": platform.platform(), "cpu": platform.processor() or platform.machine(),
        "seed": args.seed, "repeats": args.repeats, "py_repeats": args.py_repeats,
    }
    with open(os.path.join(args.out, "results.json"), "w") as f:
        json.dump({"meta": meta, "results": results}, f, indent=2)
    print(f"Results written to {os.path.join(args.out, 'results.json')}")


if __name__ == "__main__":
    main()
