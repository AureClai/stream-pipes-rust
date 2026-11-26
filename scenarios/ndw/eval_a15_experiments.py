"""Evaluate NDW Experiments E1-E4 on the A15 Papendrecht simulation results.

E1: Lane-flow distribution (pipe-stream keep-right vs NDW per-lane shares).
E2: Per-pipe split & spillback isolation at 0778ra_1 (5L) + vwa0780ra (2L).
E3: Friction sweep (phi in {1.00, 0.95, 0.90, 0.85, 0.80}) on 0778ra_1.
E4: Per-station 5-minute GEH & speed benchmark summary.
"""

from __future__ import annotations

import json
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[2]
SCENARIO_DIR = ROOT / "scenarios" / "a15_papendrecht"
SCREENED_CSV = ROOT / "scenarios" / "ndw" / "data" / "screened" / "series_per_lane_minute.csv"
DEFAULT_RESULTS = Path(r"C:\Users\ACI\.gemini\antigravity\brain\4207a357-b398-4bf7-b719-8aaa7478e7f5\scratch\a15_results.json")
STREAM_CLI = ROOT / "target" / "release" / "stream-cli.exe"


def evaluate(results_path: Path = DEFAULT_RESULTS) -> None:
    res = json.loads(results_path.read_text(encoding="utf-8"))
    df = pd.read_csv(SCREENED_CSV)
    df["ts"] = pd.to_datetime(df["timestamp"], utc=True)

    print("=== Summary KPIs ===")
    vk = res.get("vehicle_kpis", {})
    for k in ("total", "arrived", "completion_rate", "mean_speed_kmh", "mean_delay_s", "total_delay_veh_h"):
        print(f"  {k}: {vk.get(k)}")

    print("\n=== Queue & Spillback Summaries (links with queue or delay) ===")
    for q in res.get("queue_summaries", []):
        if q.get("spillback_duration_s", 0) > 0 or q.get("mean_delay_s", 0) > 0.5 or q["link_id"] in (2, 3, 4, 11, 14, 15):
            print(
                f"  link {q['link_id']:2d}: max_occ={q['max_occupancy']:3d}/{q['storage_veh']:.1f} "
                f"({q['max_occupancy_ratio']*100:5.1f}%), spill={q['spillback_duration_s']:6.1f} s, "
                f"mean_delay={q['mean_delay_s']:5.1f} s"
            )

    # E1: Lane shares on Link 0 (0764ra, 3L) & Link 2 (0778ra_1, 5L)
    # Pipe 0 = rightmost -> NDW highest lane number
    pipe_map = {(ps["link_id"], ps["pipe"]): ps for ps in res.get("pipe_stats", [])}
    t_axis = np.array(pipe_map[(0, 0)]["flow"]["t"], dtype=float)
    peak_mask = (t_axis >= 50700.0) & (t_axis < 60900.0)

    print("\n=== Experiment E1: Lane-Flow Shares (Sim Pipe vs NDW Loop Mean veh/h) ===")
    for site, lid, n_pipes in [
        ("RWS01_MONIBAS_0151hrr0764ra", 0, 3),
        ("RWS01_MONIBAS_0151hrr0778ra_1", 2, 5),
        ("RWS01_MONIBAS_0151hrr0782ra", 5, 3),
    ]:
        obs_site = (
            df[(df["site_id"] == site) & (df["ts"] >= "2026-11-26 14:05:00+00:00") & (df["ts"] < "2026-11-26 16:55:00+00:00")]
            .groupby("lane")["flow_veh_h"]
            .mean()
        )
        print(f"  {site} (link {lid}):")
        sim_flows = []
        for p in range(n_pipes):
            arr = np.array(pipe_map[(lid, p)]["flow"]["values"], dtype=float)[peak_mask]
            sim_flows.append(float(arr.mean()))
        tot_sim = sum(sim_flows)
        tot_obs = float(obs_site.sum())
        for p in range(n_pipes):
            ndw_lane = f"lane{n_pipes - p}"
            qo = float(obs_site.get(ndw_lane, 0.0))
            qs = sim_flows[p]
            print(
                f"    pipe {p} <-> {ndw_lane}: sim={qs:6.1f} veh/h ({100*qs/max(tot_sim,1):4.1f}%) | "
                f"obs={qo:6.1f} veh/h ({100*qo/max(tot_obs,1):4.1f}%)"
            )

    # E2: Per-pipe speeds during spillback episode [16:05..16:45Z = 57900..60300]
    print("\n=== Experiment E2: Spillback Window [16:05-16:45Z] Per-Pipe Speeds on Link 2 (0778ra_1) & Link 3 ===")
    spill_mask = (t_axis >= 57900.0) & (t_axis < 60300.0)
    for p in range(5):
        ndw_lane = f"lane{5 - p}"
        sp_arr = np.array(pipe_map[(2, p)]["speed"]["values"], dtype=float)[spill_mask]
        obs_sub = df[
            (df["site_id"] == "RWS01_MONIBAS_0151hrr0778ra_1")
            & (df["lane"] == ndw_lane)
            & (df["ts"] >= "2026-11-26 16:05:00+00:00")
            & (df["ts"] < "2026-11-26 16:45:00+00:00")
        ]
        obs_pos = obs_sub[obs_sub["speed_kmh"] > 0]["speed_kmh"]
        print(
            f"  Link 2 pipe {p} ({ndw_lane}): sim mean={sp_arr.mean():5.1f} min={sp_arr.min():5.1f} km/h | "
            f"obs mean={obs_pos.mean():5.1f} min={obs_pos.min():5.1f} km/h"
        )
    link_map = {ls["link_id"]: ls for ls in res.get("link_stats", [])}
    l3_sp = np.array(link_map[3]["speed"]["values"], dtype=float)[spill_mask]
    obs_l3 = df[
        (df["site_id"] == "RWS01_MONIBAS_0150vwa0780ra")
        & (df["lane"] == "lane1")
        & (df["ts"] >= "2026-11-26 16:05:00+00:00")
        & (df["ts"] < "2026-11-26 16:45:00+00:00")
    ]
    obs_l3_pos = obs_l3[obs_l3["speed_kmh"] > 0]["speed_kmh"]
    print(
        f"  Link 3 (vwa0780ra L1):  sim mean={l3_sp.mean():5.1f} min={l3_sp.min():5.1f} km/h | "
        f"obs mean={obs_l3_pos.mean():5.1f} min={obs_l3_pos.min():5.1f} km/h"
    )

    # E3: Friction sweep on Link 2 (0778ra_1)
    print("\n=== Experiment E3: Friction Sweep (phi) on Link 2 (0778ra_1) ===")
    base_net = json.loads((SCENARIO_DIR / "network.geojson").read_text(encoding="utf-8"))
    with tempfile.TemporaryDirectory() as td:
        td_path = Path(td)
        for name in ("config.json", "demand.json", "observed.json"):
            (td_path / name).write_text((SCENARIO_DIR / name).read_text(encoding="utf-8"), encoding="utf-8")
        (td_path / "patches").mkdir()
        (td_path / "patches" / "gate.json").write_text((SCENARIO_DIR / "patches" / "gate.json").read_text(encoding="utf-8"), encoding="utf-8")
        for phi in (1.00, 0.95, 0.90, 0.85, 0.80, 0.70, 0.60, 0.50):
            net = json.loads(json.dumps(base_net))
            for feat in net["features"]:
                if feat["geometry"]["type"] == "LineString" and feat["properties"]["id"] == 2:
                    feat["properties"]["friction"] = phi
            (td_path / "network.geojson").write_text(json.dumps(net), encoding="utf-8")
            out_r = td_path / f"res_{phi:.2f}.json"
            subprocess.run(
                [str(STREAM_CLI), "run", "-i", str(td_path), "--assignment", "--patches", "gate", "--bin-size", "60", "--no-vehicles", "-o", str(out_r)],
                check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            rr = json.loads(out_r.read_text(encoding="utf-8"))
            pm = {(ps["link_id"], ps["pipe"]): ps for ps in rr["pipe_stats"]}
            p_thru_q = sum(float(np.array(pm[(2, k)]["flow"]["values"])[spill_mask].mean()) for k in (2, 3, 4))
            p_thru_v = sum(float(np.array(pm[(2, k)]["speed"]["values"])[spill_mask].mean()) for k in (2, 3, 4)) / 3.0
            p0_v = float(np.array(pm[(2, 0)]["speed"]["values"])[spill_mask].mean())
            l2_spill = next(q["spillback_duration_s"] for q in rr["queue_summaries"] if q["link_id"] == 2)
            print(f"  phi={phi:.2f}: L2 spill={l2_spill:5.1f} s | mainline(P2..4) q={p_thru_q:6.1f} veh/h, v={p_thru_v:5.1f} km/h | P0(L5) v={p0_v:5.1f} km/h | mean_delay={rr['vehicle_kpis']['mean_delay_s']:.2f} s")


if __name__ == "__main__":
    evaluate()
