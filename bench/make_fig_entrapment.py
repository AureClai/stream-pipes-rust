"""Generate the paired-contrast figure of the paper (FIFO entrapment vs pipes).

Reads the CSV written by the `diverge_entrapment` example, so every number
printed on the figure comes from the solver run, not from the caption.

    cargo run --release --no-default-features --example diverge_entrapment \
        > entrapment_trace.csv
    python bench/make_fig_entrapment.py entrapment_trace.csv -o docs/figures/fig6.pdf

Left panel  : cumulative through passages at the diverge, both runs.
Right panel : vehicles departed but not yet past the diverge.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scenarios" / "ngsim"))
import paper_style  # noqa: E402

HORIZON = 1800.0  # s of demand; the run itself extends to 2000 s
RED = "#d62728"
BLUE = paper_style.BLUE
GRAY = paper_style.GRAY


class Row(dict):
    """One CSV row with the numeric fields parsed."""


def load(path: Path) -> list[Row]:
    # utf-8-sig: tolerate the BOM a PowerShell redirect puts on the header line.
    with path.open(newline="", encoding="utf-8-sig") as fh:
        rows = []
        for raw in csv.DictReader(fh):
            row = Row(raw)
            for key in ("t_depart", "t_entry", "t_discharge"):
                row[key] = float(raw[key])  # "NaN" parses to nan
            rows.append(row)
    if not rows:
        raise SystemExit(f"{path}: no rows — did the example write to stdout?")
    return rows


def cumulative(times: list[float], grid: list[float]) -> list[int]:
    """Number of `times` at or below each grid point (times may hold NaN)."""
    ordered = sorted(t for t in times if not math.isnan(t))
    out, i = [], 0
    for t in grid:
        while i < len(ordered) and ordered[i] <= t:
            i += 1
        out.append(i)
    return out


def held(rows: list[Row], grid: list[float]) -> list[int]:
    """Vehicles departed but not yet past the diverge, at each grid point."""
    departed = cumulative([r["t_depart"] for r in rows], grid)
    passed = cumulative([r["t_discharge"] for r in rows], grid)
    return [d - p for d, p in zip(departed, passed)]


def select(rows: list[Row], run: str, kind: str | None = None) -> list[Row]:
    return [r for r in rows if r["run"] == run and (kind is None or r["kind"] == kind)]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("trace", type=Path, help="CSV from the diverge_entrapment example")
    ap.add_argument("-o", "--out", type=Path, default=Path("docs/figures/fig6.pdf"))
    args = ap.parse_args()

    rows = load(args.trace)
    grid = [i * 2.0 for i in range(int(HORIZON / 2.0) + 1)]

    single_thru = select(rows, "single", "through")
    pipes_thru = select(rows, "pipes", "through")
    pipes_exit = select(rows, "pipes", "exit")

    paper_style.apply()
    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(11, 4.1))

    # ---- left: cumulative through passages at the diverge --------------------
    single_n = cumulative([r["t_discharge"] for r in single_thru], grid)
    pipes_n = cumulative([r["t_discharge"] for r in pipes_thru], grid)
    ax_left.step(grid, pipes_n, where="post", color=BLUE, label="pipes, $\\varphi$ = 0.95")
    ax_left.step(grid, single_n, where="post", color=RED, label="single stream")
    ax_left.set_title("cumulative through passages at the diverge", loc="left")
    ax_left.set_xlabel("time (s)")
    ax_left.set_ylabel("vehicles")
    ax_left.annotate(
        f"{pipes_n[-1]:,} passed — 3 600 veh/h, zero delay".replace(",", " "),
        xy=(HORIZON, pipes_n[-1]), xytext=(-12, -30),
        textcoords="offset points", ha="right", color=BLUE, fontsize=9,
    )
    ax_left.annotate(
        f"{single_n[-1]} passed — 360 veh/h exactly\n"
        r"= $(q_\mathrm{thru}/q_\mathrm{exit})\,/\,h_\mathrm{ramp}$",
        xy=(HORIZON, single_n[-1]), xytext=(-12, 26),
        textcoords="offset points", ha="right", color=RED, fontsize=9,
    )
    ax_left.legend(loc="upper left")

    # ---- right: vehicles held upstream of the diverge ------------------------
    single_held = held(select(rows, "single"), grid)
    pipes_exit_held = held(pipes_exit, grid)
    pipes_thru_held = held(pipes_thru, grid)
    ax_right.step(grid, single_held, where="post", color=RED,
                  label="single stream — everyone")
    ax_right.step(grid, pipes_exit_held, where="post", color=BLUE,
                  label="pipes — exit-bound (confined to its pipe)")
    ax_right.step(grid, pipes_thru_held, where="post", color=GRAY, linewidth=1.4,
                  label="pipes — through (free-flow transit)")
    ax_right.set_title("vehicles departed but not yet past the diverge", loc="left")
    ax_right.set_xlabel("time (s)")
    ax_right.set_ylabel("vehicles")
    for series, color, dy in ((single_held, RED, 6), (pipes_exit_held, BLUE, 6),
                             (pipes_thru_held, GRAY, 8)):
        ax_right.annotate(
            f"{series[-1]:,}".replace(",", " "),
            xy=(HORIZON, series[-1]), xytext=(6, dy),
            textcoords="offset points", color=color, fontsize=9,
        )
    ax_right.legend(loc="upper left")

    for ax in (ax_left, ax_right):
        ax.set_xlim(0, HORIZON * 1.06)
        ax.set_ylim(bottom=0)

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out)

    n_single_thru_held = held(single_thru, grid)[-1]
    print(f"wrote {args.out}")
    print(f"  through passed by t={HORIZON:.0f}s: single {single_n[-1]}, pipes {pipes_n[-1]}")
    print(f"  held at t={HORIZON:.0f}s: single {single_held[-1]} "
          f"({n_single_thru_held} through + {single_held[-1] - n_single_thru_held} exit-bound), "
          f"pipes {pipes_exit_held[-1]} exit-bound + {pipes_thru_held[-1]} through in transit")


if __name__ == "__main__":
    main()
