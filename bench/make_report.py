"""Render bench/results/results.json as the Markdown tables of bench/RESULTS.md."""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def fmt(v, spec):
    return "—" if v is None else format(v, spec)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "results", "results.json")
    with open(path) as f:
        data = json.load(f)
    meta, rows = data["meta"], data["results"]

    print(f"Machine: {meta['cpu']}, {meta['platform']}; Python {meta['python']}, "
          f"NumPy {meta['numpy']}. pipe-stream: median of {meta['repeats']} runs; "
          f"Stream Python: median of {meta['py_repeats']} run(s); seed {meta['seed']}.\n")

    print("### Speed\n")
    print("| Scenario | Nodes | Links | Vehicles | Node passages | Stream Python | "
          "pipe-stream | Speed-up | Python passages/s | pipe-stream passages/s |")
    print("|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|")
    for r in rows:
        print(f"| `{r['scenario']}` | {r['nodes']} | {r['links']} | {r['vehicles']:,} | "
              f"{r['ru_passages']:,} | {r['py_s']:.2f} s | {r['ru_total_s'] * 1e3:.2f} ms | "
              f"**×{r['speedup']:,.0f}** | {r['py_passages'] / r['py_s']:,.0f} | "
              f"{r['ru_passages'] / r['ru_total_s']:,.0f} |")

    print("\n### Agreement\n")
    print("| Scenario | Completed (Py / Rust) | Mean travel time (Py / Rust) | "
          "Incl. entry wait (Py / Rust) | Passage-time MAE | Max error | Exact passages |")
    print("|---|--:|--:|--:|--:|--:|--:|")
    for r in rows:
        print(f"| `{r['scenario']}` | {r['py_completed']:,} / {r['ru_completed']:,} | "
              f"{fmt(r['py_mean_tt'], '.1f')} / {fmt(r['ru_mean_tt'], '.1f')} s | "
              f"{fmt(r['py_mean_tt_from_demand'], '.1f')} / "
              f"{fmt(r['ru_mean_tt_from_demand'], '.1f')} s | "
              f"{fmt(r['passage_mae'], '.3g')} s | {fmt(r['passage_max_err'], '.3g')} s | "
              f"{fmt(r['passage_exact_pct'], '.1f')} % |")

    if any("aligned" in r for r in rows):
        print("\n### Agreement with Stream Python aligned on pipe-stream's node rules\n")
        print("Deterministic merge + causal passages (`--aligned`, see "
              "`aligned_python` in the driver).\n")
        print("| Scenario | Completed (Py / Rust) | Mean travel time (Py / Rust) | "
              "Passage-time MAE | Max error | Exact passages |")
        print("|---|--:|--:|--:|--:|--:|")
        for r in rows:
            a = r.get("aligned")
            if not a:
                continue
            print(f"| `{r['scenario']}` | {a['py_completed']:,} / {a['ru_completed']:,} | "
                  f"{fmt(a['py_mean_tt'], '.1f')} / {fmt(a['ru_mean_tt'], '.1f')} s | "
                  f"{fmt(a['passage_mae'], '.3g')} s | {fmt(a['passage_max_err'], '.3g')} s | "
                  f"{fmt(a['passage_exact_pct'], '.1f')} % |")


if __name__ == "__main__":
    main()
