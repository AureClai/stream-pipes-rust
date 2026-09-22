"""Generate docs/figures/fig_ngsim_behaviour.pdf — NGSIM behavioural panel.

(a) ECDF of the exit lane-commitment distance, measured per vehicle from
    the final mainline lane change to that vehicle's own entry into the
    exit lane, with the G1 300-500 m floor band and the observation
    (censoring) limit of the 640 m site.
(b) Lane-change intensity per 100 m bin, per 15-min period.

Recomputes from the raw pages via analyze_trajectories' functions so the
figure and the metrics JSON cannot drift apart. Styled by paper_style to
match the paper's exported figures.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import paper_style
from analyze_trajectories import debounce_lanes, load

paper_style.apply()
BLUE = paper_style.BLUE
ORANGE = paper_style.ORANGE
GRAY = paper_style.GRAY
DEBOUNCE = 2.0

base = Path(__file__).parent
df = load(base / "data" / "raw", "us-101")

# --- commitment distances (same logic as commitment_metrics, raw values) ---
dists, gores = [], []
lc_events = []
t0 = df["t"].min()
main = set(range(1, 7))
for _, g in df.groupby("traj", sort=False):
    runs = debounce_lanes(g, DEBOUNCE)
    lanes = [r[2] for r in runs]
    for r0, r1 in zip(runs, runs[1:]):
        if r0[2] in main and r1[2] in main:
            lc_events.append((int((r1[0] - t0) // 900), r1[1]))
    if 8 not in lanes:
        continue
    first_exit = lanes.index(8)
    gores.append(runs[first_exit][1])
    last_main = None
    for r0, r1 in zip(runs, runs[1:]):
        if r0[2] <= 5 and r1[2] > 5:
            last_main = r1
    if last_main is not None:
        dists.append(runs[first_exit][1] - last_main[1])

gore = float(np.median(gores))
commit = np.sort(np.array([d for d in dists if d >= 0]))
censor = gore  # commitments longer than the upstream extent are unobservable

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.4, 3.0))

# (a) ECDF
ecdf_y = np.arange(1, len(commit) + 1) / len(commit)
ax1.axvspan(300, 500, color=BLUE, alpha=0.10, lw=0)
ax1.text(400, 0.07, "G1 floor\n300–500 m", ha="center", fontsize=8.5,
         color=BLUE)
ax1.axvline(censor, ls="--", lw=1.0, color="#999999")
ax1.text(censor - 10, 0.52, f"observation limit ({censor:.0f} m)",
         rotation=90, va="center", ha="right", fontsize=8, color="#888888")
ax1.step(commit, ecdf_y, where="post", color=BLUE)
med = float(np.median(commit))
p90 = float(np.percentile(commit, 90))
ax1.plot([med], [0.5], "o", ms=5, color=BLUE)
ax1.annotate(f"median {med:.0f} m", (med, 0.5), textcoords="offset points",
             xytext=(-8, 5), ha="right", fontsize=8.5)
ax1.plot([p90], [0.9], "s", ms=5, color=BLUE)
ax1.annotate(f"p90 {p90:.0f} m", (p90, 0.9), textcoords="offset points",
             xytext=(-8, 3), ha="right", fontsize=8.5)
ax1.set_xlabel("commitment distance to own exit-lane entry (m)")
ax1.set_ylabel("ECDF")
ax1.set_xlim(0, 520)
ax1.set_ylim(0, 1.02)
ax1.set_title(f"(a) Exit commitment, n = {len(commit)}")

# (b) LC intensity per 100 m bin, periods 0-2
bins = np.arange(0, 800, 100)
for period, color in zip(range(3), [BLUE, ORANGE, GRAY]):
    ys = [e[1] for e in lc_events if e[0] == period]
    counts, _ = np.histogram(ys, bins=bins)
    rate = counts / 0.1 / 15.0  # events / km / min
    ax2.plot(bins[:-1] + 50, rate, "-o", ms=4.5, color=color,
             label=f"{['07:50', '08:05', '08:20'][period]} period")
ax2.axvline(gore, ls="--", lw=1.0, color="#999999")
ax2.text(gore + 12, 22, "off-ramp\ngore", fontsize=8, color="#888888")
ax2.set_xlabel("longitudinal position (m)")
ax2.set_ylabel("lane changes / km / min")
ax2.legend()
ax2.set_title("(b) Lane-change intensity")

fig.tight_layout()
out = base.parent.parent / "docs" / "figures" / "fig_ngsim_behaviour.pdf"
fig.savefig(out, bbox_inches="tight")
print(f"saved {out}; n_commit={len(commit)}, median={med:.1f}, p90={p90:.1f}, "
      f"gore={gore:.1f}")
