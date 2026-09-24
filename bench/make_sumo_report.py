"""Render bench/results/sumo_results.json as the Markdown tables of bench/SUMO.md."""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def fmt(v, spec):
    return "—" if v is None else format(v, spec)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "results", "sumo_results.json")
    with open(path) as f:
        data = json.load(f)
    meta, rows = data["meta"], data["results"]
    py = {}
    py_path = os.path.join(os.path.dirname(path), "results.json")
    if os.path.exists(py_path):
        with open(py_path) as f:
            py = {r["scenario"]: r["py_s"] for r in json.load(f)["results"]}

    print(f"{meta['sumo']}; {meta['platform']}. Median of {meta['repeats']} runs per engine.\n")

    print("### Speed\n")
    print("| Scenario | Nodes | Links | Vehicles | pipe-stream | SUMO meso | SUMO micro | "
          "Stream Python | meso / pipe-stream | micro / pipe-stream |")
    print("|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|")
    for r in rows:
        p = py.get(r["scenario"])
        print(f"| `{r['scenario']}` | {r['nodes']} | {r['links']} | {r['vehicles']:,} | "
              f"{r['ru_s'] * 1e3:.2f} ms | {r['meso_s']:.2f} s | {r['micro_s']:.2f} s | "
              f"{fmt(p, '.2f')}{' s' if p is not None else ''} | "
              f"**×{r['meso_s'] / r['ru_s']:,.0f}** | **×{r['micro_s'] / r['ru_s']:,.0f}** |")

    print("\n### Outputs\n")
    print("| Scenario | Completed (pipe-stream / meso / micro) | Not inserted at end "
          "(meso / micro) | Mean travel time (pipe-stream / meso / micro) | "
          "Incl. entry wait (pipe-stream / meso / micro) |")
    print("|---|--:|--:|--:|--:|")
    for r in rows:
        a, m, u = r["rust"], r["meso"], r["micro"]
        print(f"| `{r['scenario']}` | {a['completed']:,} / {m['completed']:,} / "
              f"{u['completed']:,} | {fmt(m['not_inserted'], ',')} / "
              f"{fmt(u['not_inserted'], ',')} | "
              f"{fmt(a['mean_tt'], '.1f')} / {fmt(m['mean_tt'], '.1f')} / "
              f"{fmt(u['mean_tt'], '.1f')} s | {fmt(a['mean_tt_from_demand'], '.1f')} / "
              f"{fmt(m['mean_tt_from_demand'], '.1f')} / "
              f"{fmt(u['mean_tt_from_demand'], '.1f')} s |")


if __name__ == "__main__":
    main()
