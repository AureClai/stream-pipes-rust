"""Generate docs/figures/fig_us101_replication.pdf — US-101 replication vs
observation.

(a) Cumulative vehicles past the 300 m station: observed vs simulated,
    with the warm-up window marked and the 5-minute-window GEH annotated.
(b) Per-lane flow shares at the same station: observed lanes vs simulated
    pipes (pipe 0 = auxiliary/rightmost ... pipe 5 = median).

Inputs: data/out/us101_sim_input.json (observed series + metadata) and
        data/out/us101_sim_trace.csv (engine output).
Styled by paper_style: observed = blue, simulated = orange, matching the
paper's exported field-validation figures.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import paper_style

paper_style.apply()
BLUE = paper_style.BLUE
ORANGE = paper_style.ORANGE
WARMUP_MIN = 5

base = Path(__file__).parent
inp = json.loads((base / "data" / "out" / "us101_sim_input.json").read_text())
sim = pd.read_csv(base / "data" / "out" / "us101_sim_trace.csv")
sim = sim[np.isfinite(sim["t_300m"])]

obs_minute = {int(k): v for k, v in inp["observed_300m"]["minute_total"].items()}
obs_by_lane: dict[tuple[int, int], int] = {}
for key, n in inp["observed_300m"]["minute_by_lane"].items():
    m, lane = (int(x) for x in key.split("_"))
    obs_by_lane[(m, lane)] = n

last_min = max(obs_minute)
minutes = np.arange(0, last_min + 1)
obs_counts = np.array([obs_minute.get(int(m), 0) for m in minutes])

sim["minute"] = (sim["t_300m"] // 60).astype(int)
sim_minute = sim.groupby("minute").size()
sim_counts = np.array([sim_minute.get(int(m), 0) for m in minutes])

# GEH on 5-minute windows (veh/h), warm-up excluded
WIN = 5
starts = np.arange(WARMUP_MIN, last_min + 1 - WIN + 1, WIN)
m_flow, o_flow = [], []
for s in starts:
    sel = (minutes >= s) & (minutes < s + WIN)
    m_flow.append(sim_counts[sel].sum() * 60.0 / WIN)
    o_flow.append(obs_counts[sel].sum() * 60.0 / WIN)
m_flow, o_flow = np.array(m_flow), np.array(o_flow)
geh = np.sqrt(2 * (m_flow - o_flow) ** 2 / (m_flow + o_flow))
geh_mean, geh_max = float(np.mean(geh)), float(np.max(geh))
geh_under5 = int((geh < 5).sum())
n_windows = len(geh)

fig, (ax1, ax2) = plt.subplots(
    1, 2, figsize=(7.4, 3.0), gridspec_kw={"width_ratios": [1.25, 1.0]}
)

# (a) cumulative curves — observed blue, simulated orange (paper convention)
ax1.axvspan(0, WARMUP_MIN, color="#000000", alpha=0.06, lw=0)
ax1.text(WARMUP_MIN / 2, 0.55 * obs_counts.cumsum()[-1], "warm-up",
         rotation=90, ha="center", va="center", fontsize=8.5,
         color="#888888")
ax1.plot(minutes + 1, obs_counts.cumsum(), color=BLUE, lw=2.6,
         label="observed")
ax1.plot(minutes + 1, sim_counts.cumsum(), color=ORANGE, lw=2.0,
         label="simulated")
ax1.set_xlabel("minute of recording")
ax1.set_ylabel("cumulative vehicles past 300 m")
ax1.legend(loc="lower right")
ax1.set_title(
    f"(a) Throughput at 300 m — GEH mean {geh_mean:.1f}, "
    f"{geh_under5}/{n_windows} < 5")

# (b) per-lane shares, warm-up excluded; obs lane 6-k <-> sim pipe k
obs_lane_tot = np.zeros(6)
for (m, lane), n in obs_by_lane.items():
    if m >= WARMUP_MIN and 1 <= lane <= 6:
        obs_lane_tot[lane - 1] += n
obs_share = obs_lane_tot / obs_lane_tot.sum()

sim_after = sim[sim["minute"] >= WARMUP_MIN]
sim_pipe_tot = np.zeros(6)
for pipe, n in sim_after.groupby("pipe_300m").size().items():
    sim_pipe_tot[int(pipe)] += n
sim_share = sim_pipe_tot / sim_pipe_tot.sum()

labels = ["aux", "lane 5", "lane 4", "lane 3", "lane 2", "median"]
obs_ordered = [obs_share[5], obs_share[4], obs_share[3],
               obs_share[2], obs_share[1], obs_share[0]]
sim_ordered = [sim_share[0], sim_share[1], sim_share[2],
               sim_share[3], sim_share[4], sim_share[5]]
x = np.arange(6)
ax2.bar(x - 0.19, obs_ordered, 0.36, color=BLUE, label="observed")
ax2.bar(x + 0.19, sim_ordered, 0.36, color=ORANGE, label="simulated")
ax2.set_xticks(x)
ax2.set_xticklabels(labels, fontsize=8.5)
ax2.set_ylabel("flow share at 300 m")
ax2.set_ylim(0, 0.30)
ax2.legend()
ax2.set_title("(b) Per-lane distribution")

fig.tight_layout()
out = base.parent.parent / "docs" / "figures" / "fig_us101_replication.pdf"
fig.savefig(out, bbox_inches="tight")

print(json.dumps({
    "geh_mean": round(geh_mean, 2),
    "geh_max": round(geh_max, 2),
    "geh_under5": geh_under5,
    "windows_scored": n_windows,
    "obs_shares": [round(v, 3) for v in obs_ordered],
    "sim_shares": [round(v, 3) for v in sim_ordered],
    "sim_vehicles_past_300": int(len(sim_after)),
}, indent=1))
print(f"saved {out}")
