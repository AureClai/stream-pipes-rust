"""Build the US-101 simulation input from the NGSIM trajectories.

Produces data/out/us101_sim_input.json containing:
  - per-vehicle demand reconstructed from observation (entry time, origin
    main/ramp, destination through/exit) -- no demand model, no calibration;
  - FD estimates read off the data's own envelope (u, c per lane);
  - the measured off-ramp gore and on-ramp merge positions;
  - the downstream boundary gate: observed minute discharge at y = 630 m,
    imposed as a time-windowed capacity (the paper's boundary-gate recipe);
  - the observed comparison series at the 300 m station (cumulative count
    and per-lane minute flows), so the figure reads one file.

Vehicles already inside the section at a recording boundary (first
observation upstream of neither entry) are excluded from injection and
counted; the comparison therefore starts after a warm-up window.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd

from analyze_trajectories import debounce_lanes, load

BASE = Path(__file__).parent
STATION = 300.0
GATE_STATION = 630.0
ENTRY_MAX_Y = 30.0
DEBOUNCE = 2.0

df = load(BASE / "data" / "raw", "us-101")
t0 = float(df["t"].min())
df["tt"] = df["t"] - t0
t_end = float(df["tt"].max())

vehicles = []
midstart = 0
gore_estimates = []
merge_estimates = []
crossings_300 = []   # (tt, lane)
crossings_gate = []  # tt
offramp_counts = []  # tt of lane-8 crossings at y=450

for traj, g in df.groupby("traj", sort=False):
    y = g["y"].values
    tt = g["tt"].values
    lanes_raw = g["lane_id"].values.astype(int)
    runs = debounce_lanes(g, DEBOUNCE)
    lane_seq = [r[2] for r in runs]

    is_exit = 8 in lane_seq
    if is_exit:
        first_exit = lane_seq.index(8)
        gore_estimates.append(runs[first_exit][1])
    if 7 in lane_seq:
        # merge point ~ where the vehicle leaves the on-ramp lane
        i7 = lane_seq.index(7)
        if i7 + 1 < len(runs):
            merge_estimates.append(runs[i7 + 1][1])

    # observed crossings for the comparison series
    for station, sink in ((STATION, crossings_300), (GATE_STATION, crossings_gate)):
        idx = np.nonzero((y[:-1] < station) & (y[1:] >= station))[0]
        if len(idx):
            i = idx[0]
            frac = (station - y[i]) / max(y[i + 1] - y[i], 1e-9)
            t_cross = tt[i] + frac * (tt[i + 1] - tt[i])
            if station == STATION:
                sink.append((t_cross, int(lanes_raw[i + 1])))
            else:
                sink.append(t_cross)
    idx = np.nonzero((y[:-1] < 450.0) & (y[1:] >= 450.0))[0]
    if len(idx) and lanes_raw[min(idx[0] + 1, len(lanes_raw) - 1)] == 8:
        offramp_counts.append(tt[idx[0]])

    # demand classification
    origin = None
    if lane_seq[0] == 7:
        origin = "ramp"
        start = float(tt[0])
    elif y[0] < ENTRY_MAX_Y and 1 <= lane_seq[0] <= 6:
        origin = "main"
        start = float(tt[0])
    else:
        midstart += 1
        continue
    vehicles.append(
        {"origin": origin, "exit": bool(is_exit), "start_time": round(start, 2)}
    )

gore = float(np.median(gore_estimates))
merge = float(np.median(merge_estimates))

# FD envelope estimates
speeds = df["speed"].values
u_est = float(np.percentile(speeds[speeds > 0], 99))
c300 = pd.DataFrame(crossings_300, columns=["tt", "lane"])
c300["minute"] = (c300["tt"] // 60).astype(int)
lane_minute = c300.groupby(["minute", "lane"]).size()
c_lane_est = float(np.percentile(lane_minute.values, 98)) / 60.0  # veh/s/lane

# boundary gate: observed minute discharge at 630 m (veh/s whole carriageway)
gate = pd.Series(crossings_gate)
gate_min = gate.groupby((gate // 60).astype(int)).size()
gate_schedule = [
    {"minute": int(m), "capacity_veh_s": round(float(n) / 60.0, 4)}
    for m, n in gate_min.items()
]

# observed comparison series at 300 m
obs_min_total = c300.groupby("minute").size()
obs_series = {
    "minute_total": {int(m): int(n) for m, n in obs_min_total.items()},
    "minute_by_lane": {
        f"{int(m)}_{int(l)}": int(n) for (m, l), n in lane_minute.items()
    },
}

out = {
    "t0_epoch_s": t0,
    "duration_s": round(t_end + 600.0, 1),
    "gore_y_m": round(gore, 1),
    "merge_y_m": round(merge, 1),
    "u_est_ms": round(u_est, 2),
    "c_lane_est_veh_s": round(c_lane_est, 4),
    "n_midstart_excluded": midstart,
    "vehicles": sorted(vehicles, key=lambda v: v["start_time"]),
    "gate_schedule": gate_schedule,
    "observed_300m": obs_series,
}
dest = BASE / "data" / "out" / "us101_sim_input.json"
dest.parent.mkdir(parents=True, exist_ok=True)
dest.write_text(json.dumps(out, indent=1))
print(
    f"vehicles injected: {len(vehicles)} (midstart excluded: {midstart}); "
    f"u={u_est:.1f} m/s, c_lane={c_lane_est:.3f} veh/s, "
    f"merge={merge:.0f} m, gore={gore:.0f} m -> {dest}"
)
