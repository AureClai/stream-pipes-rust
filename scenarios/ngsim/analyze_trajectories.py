"""NGSIM trajectory analysis: virtual per-lane loops + behavioral metrics.

Usage:
    python analyze_trajectories.py --location us-101
        [--stations 100,300,500] [--debounce 2.0] [--late-threshold 150]

Reads the pages downloaded by fetch_ngsim.py, converts to metric units,
splits recording periods, debounces lane assignments, then computes:

  N1  virtual per-lane loops: minute flow (veh/h) + mean speed per station
  N2  lane-commitment distance of exit-bound vehicles (US-101: lane 8)
  N3  late-insertion share (last change within --late-threshold m of gore)
  N4  lane-change intensity per 100 m bin (events / km / min)

Outputs: data/out/<loc>_loops.csv and data/out/<loc>_metrics.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

FT = 0.3048
USECOLS = ["vehicle_id", "global_time", "local_y", "v_vel", "v_class", "lane_id"]
MAINLINE = {"us-101": list(range(1, 7)), "i-80": list(range(1, 7))}
EXIT_LANE = {"us-101": 8}


def load(raw: Path, location: str) -> pd.DataFrame:
    pages = sorted(raw.glob(f"{location}_p*.csv"))
    if not pages:
        raise SystemExit(f"no pages for {location} in {raw}; run fetch_ngsim.py first")
    df = pd.concat(
        (pd.read_csv(p, usecols=USECOLS) for p in pages), ignore_index=True
    )
    df["t"] = df["global_time"] / 1000.0
    df["y"] = df["local_y"] * FT
    df["speed"] = df["v_vel"] * FT
    df = df.drop(columns=["global_time", "local_y", "v_vel"])
    df = df.sort_values(["vehicle_id", "t"], kind="stable").reset_index(drop=True)

    # Vehicle ids restart each recording period while time runs continuously,
    # so the same id can cover several distinct vehicles. Split trajectories
    # on kinematic discontinuities: id change, time jump, or backward
    # position jump — a real vehicle does neither mid-trajectory.
    id_change = df["vehicle_id"].ne(df["vehicle_id"].shift())
    dt = df["t"].diff()
    dy = df["y"].diff()
    df["traj"] = (id_change | (dt > 1.0) | (dy < -30.0)).cumsum()

    # Reporting period = 15-min window since recording start.
    df["period"] = ((df["t"] - df["t"].min()) // 900).astype(int)
    return df


def debounce_lanes(g: pd.DataFrame, min_dur: float) -> list[tuple[float, float, int]]:
    """Run-length encode a vehicle's lane series, merging runs < min_dur s.

    Returns [(t_start, y_at_start, lane), ...] of stable lane runs.
    """
    t = g["t"].values
    y = g["y"].values
    lane = g["lane_id"].values.astype(int)
    runs: list[list] = []  # [t0, y0, lane, t_end]
    for i in range(len(t)):
        if runs and runs[-1][2] == lane[i]:
            runs[-1][3] = t[i]
        else:
            runs.append([t[i], y[i], lane[i], t[i]])
    # merge too-short runs into their predecessor (flicker suppression)
    stable: list[list] = []
    for r in runs:
        if stable and (r[3] - r[0]) < min_dur:
            stable[-1][3] = r[3]
            continue
        if stable and stable[-1][2] == r[2]:
            stable[-1][3] = r[3]
        else:
            stable.append(r)
    return [(r[0], r[1], r[2]) for r in stable]


def virtual_loops(df: pd.DataFrame, stations: list[float]) -> pd.DataFrame:
    """First upstream->downstream crossing of each station per trajectory."""
    t0 = df["t"].min()
    rows = []
    for traj, g in df.groupby("traj", sort=False):
        y = g["y"].values
        t = g["t"].values
        v = g["speed"].values
        lane = g["lane_id"].values.astype(int)
        for s in stations:
            idx = np.nonzero((y[:-1] < s) & (y[1:] >= s))[0]
            if len(idx) == 0:
                continue
            i = idx[0]
            frac = (s - y[i]) / max(y[i + 1] - y[i], 1e-9)
            rows.append(
                {
                    "station_m": s,
                    "t_cross": t[i] + frac * (t[i + 1] - t[i]),
                    "lane": lane[i + 1],
                    "speed": v[i] + frac * (v[i + 1] - v[i]),
                    "traj": traj,
                }
            )
    cross = pd.DataFrame(rows)
    cross["period"] = ((cross["t_cross"] - t0) // 900).astype(int)
    cross["minute"] = (cross["t_cross"] // 60).astype(int)
    loops = (
        cross.groupby(["period", "station_m", "lane", "minute"])
        .agg(flow_veh_h=("traj", "count"), speed_ms=("speed", "mean"))
        .reset_index()
    )
    loops["flow_veh_h"] *= 60  # veh/min -> veh/h
    return loops


def commitment_metrics(
    df: pd.DataFrame, location: str, min_dur: float, late_m: float
) -> dict:
    exit_lane = EXIT_LANE.get(location)
    if exit_lane is None:
        return {"status": "not-exercised (no off-ramp lane at this site)"}
    dists, gore_estimates = [], []
    for _, g in df.groupby("traj", sort=False):
        runs = debounce_lanes(g, min_dur)
        lanes = [r[2] for r in runs]
        if exit_lane not in lanes:
            continue
        first_exit = lanes.index(exit_lane)
        gore_estimates.append(runs[first_exit][1])
        # last change from a pure mainline lane (1-5) toward the exit path
        last_main = None
        for r0, r1 in zip(runs, runs[1:]):
            if r0[2] <= 5 and r1[2] > 5:
                last_main = r1
        if last_main is not None:
            # per-vehicle: from the last mainline lane change to the point
            # where THIS vehicle enters the exit lane (not a pooled gore)
            dists.append(runs[first_exit][1] - last_main[1])
    if not dists:
        return {"status": "not-exercised (no exit-bound vehicles found)"}
    gore = float(np.median(gore_estimates))  # censoring reference only
    commit = np.array([d for d in dists if d >= 0])
    return {
        "status": "run",
        "n_exit_vehicles": int(len(dists)),
        "gore_y_m_est": round(gore, 1),
        "commitment_distance_m": {
            "median": round(float(np.median(commit)), 1),
            "p25": round(float(np.percentile(commit, 25)), 1),
            "p75": round(float(np.percentile(commit, 75)), 1),
            "p90": round(float(np.percentile(commit, 90)), 1),
        },
        "late_insertion_share": round(float((commit < late_m).mean()), 3),
        "late_threshold_m": late_m,
    }


def lane_change_intensity(df: pd.DataFrame, location: str, min_dur: float) -> list[dict]:
    main = set(MAINLINE.get(location, range(1, 9)))
    t0 = df["t"].min()
    events = []
    for _, g in df.groupby("traj", sort=False):
        runs = debounce_lanes(g, min_dur)
        for r0, r1 in zip(runs, runs[1:]):
            if r0[2] in main and r1[2] in main:
                # (period at change, y where the new lane starts)
                events.append((int((r1[0] - t0) // 900), r1[1]))
    ev = pd.DataFrame(events, columns=["period", "y"])
    out = []
    for period, g in ev.groupby("period"):
        dur_min = (df.loc[df["period"] == period, "t"].max()
                   - df.loc[df["period"] == period, "t"].min()) / 60
        binned = (g["y"] // 100).astype(int).value_counts().sort_index()
        out.append(
            {
                "period": int(period),
                "duration_min": round(dur_min, 1),
                "lc_per_km_min_by_100m_bin": {
                    f"{int(b) * 100}-{int(b) * 100 + 100}m":
                        round(n / 0.1 / dur_min, 1)
                    for b, n in binned.items()
                },
            }
        )
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    base = Path(__file__).parent
    ap.add_argument("--location", required=True)
    ap.add_argument("--raw", type=Path, default=base / "data" / "raw")
    ap.add_argument("--out", type=Path, default=base / "data" / "out")
    ap.add_argument("--stations", default="100,300,500")
    ap.add_argument("--debounce", type=float, default=2.0)
    ap.add_argument("--late-threshold", type=float, default=150.0)
    args = ap.parse_args()

    stations = [float(s) for s in args.stations.split(",")]
    df = load(args.raw, args.location)
    args.out.mkdir(parents=True, exist_ok=True)

    loops = virtual_loops(df, stations)
    loops.to_csv(args.out / f"{args.location}_loops.csv", index=False)

    shares = {}
    for (period, station), g in loops.groupby(["period", "station_m"]):
        lane_mean = g.groupby("lane")["flow_veh_h"].mean()
        shares[f"period{period}_station{int(station)}m"] = {
            f"lane{int(l)}": round(float(v / lane_mean.sum()), 3)
            for l, v in lane_mean.items()
        }
    metrics = {
        "location": args.location,
        "periods": int(df["period"].nunique()),
        "vehicles": int(df["traj"].nunique()),
        "n1_lane_flow_shares": shares,
        "n2_n3_commitment": commitment_metrics(
            df, args.location, args.debounce, args.late_threshold
        ),
        "n4_lane_change_intensity": lane_change_intensity(
            df, args.location, args.debounce
        ),
        "debounce_s": args.debounce,
    }
    (args.out / f"{args.location}_metrics.json").write_text(
        json.dumps(metrics, indent=2)
    )
    print(json.dumps({k: metrics[k] for k in
                      ("location", "periods", "vehicles", "n2_n3_commitment")},
                     indent=2))
    print(f"full metrics: {args.out / f'{args.location}_metrics.json'}")


if __name__ == "__main__":
    main()
