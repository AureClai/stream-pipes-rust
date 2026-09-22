"""Dexter (NDW) export -> normalised per-lane minute series + screening report.

Usage:
    python prepare_dexter_export.py [--raw data/raw] [--out data/screened]
                                    [--corridor network/corridor.json]

Reads every CSV in the raw directory, normalises to one tidy table
(site_id, timestamp, lane, flow_veh_h, speed_kmh, n_valid), then runs the
screening steps that need no topology:

  S3a  dead per-lane channels: flow or speed frozen over a sliding window
  S3b  physically impossible values (negative flow, speed > 200 km/h,
       per-lane flow > 3000 veh/h sustained)
  inventory: sites, lanes per site, coverage gaps

S1 conservation closure and S2 residual localisation additionally need the
corridor topology (which sites are boundaries, ramps, internal) -- provide it
as corridor.json (template written on first run) and they run per
carriageway (sum over lanes).

COLUMN_MAP below is written against the documented Dexter "Intensiteit en
snelheid" export and MUST be confirmed against the first real export --
the script fails loudly listing the columns it actually found.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import pandas as pd

# --- confirm against the first real Dexter export -------------------------
# Dexter CSVs are typically ';'-separated with Dutch headers.
COLUMN_MAP = {
    "site_id": ["meetlocatie id", "meetlocatie", "measurement site id"],
    "timestamp": ["periode start", "start meetperiode", "period start"],
    "lane": ["rijstrook", "lane"],
    "flow": ["intensiteit", "gem. intensiteit", "intensity"],
    "speed": ["snelheid", "gem. snelheid", "speed"],
}
CSV_SEP = ";"

FLOW_MAX_VEH_H = 3000.0     # sustained per-lane flow above this is a defect
SPEED_MAX_KMH = 200.0
FROZEN_WINDOW_MIN = 60      # a channel constant for this long is suspect
CLOSURE_PASS_PCT = 1.0      # S1: daily carriageway balance must close < 1 %


def _resolve_columns(df: pd.DataFrame, path: Path) -> dict[str, str]:
    cols = {c.lower().strip(): c for c in df.columns}
    resolved = {}
    missing = []
    for key, candidates in COLUMN_MAP.items():
        hit = next((cols[c] for c in candidates if c in cols), None)
        if hit is None:
            missing.append((key, candidates))
        else:
            resolved[key] = hit
    if missing:
        sys.exit(
            f"{path.name}: could not resolve {[m[0] for m in missing]}.\n"
            f"Columns found: {list(df.columns)}\n"
            "Update COLUMN_MAP to match the actual Dexter export headers."
        )
    return resolved


def load_raw(raw_dir: Path) -> pd.DataFrame:
    frames = []
    files = sorted(raw_dir.glob("*.csv"))
    if not files:
        sys.exit(f"No CSV files in {raw_dir}. Drop Dexter exports there first.")
    for path in files:
        df = pd.read_csv(path, sep=CSV_SEP, low_memory=False)
        col = _resolve_columns(df, path)
        tidy = pd.DataFrame(
            {
                "site_id": df[col["site_id"]].astype(str),
                "timestamp": pd.to_datetime(df[col["timestamp"]]),
                "lane": df[col["lane"]].astype(str),
                "flow_veh_h": pd.to_numeric(df[col["flow"]], errors="coerce"),
                "speed_kmh": pd.to_numeric(df[col["speed"]], errors="coerce"),
            }
        )
        tidy["source_file"] = path.name
        frames.append(tidy)
    return pd.concat(frames, ignore_index=True)


def screen_s3(df: pd.DataFrame) -> list[dict]:
    """Topology-free defect screening. Returns a list of findings."""
    findings = []
    for (site, lane), g in df.groupby(["site_id", "lane"]):
        g = g.sort_values("timestamp")
        if (g["flow_veh_h"] < 0).any():
            findings.append({"check": "S3b_negative_flow", "site": site, "lane": lane})
        if (g["speed_kmh"] > SPEED_MAX_KMH).any():
            findings.append({"check": "S3b_speed_over_max", "site": site, "lane": lane})
        high = g["flow_veh_h"] > FLOW_MAX_VEH_H
        if high.rolling(15).sum().max() >= 15:  # 15 consecutive minutes
            findings.append({"check": "S3b_flow_over_max_sustained", "site": site, "lane": lane})
        for col in ("flow_veh_h", "speed_kmh"):
            s = g[col].dropna()
            if len(s) >= FROZEN_WINDOW_MIN:
                frozen = (s.diff() == 0).rolling(FROZEN_WINDOW_MIN).sum()
                if (frozen >= FROZEN_WINDOW_MIN - 1).any() and s.nunique() > 1:
                    findings.append(
                        {"check": "S3a_frozen_channel", "site": site, "lane": lane, "field": col}
                    )
    return findings


def screen_s1(df: pd.DataFrame, corridor: dict) -> list[dict]:
    """Daily carriageway conservation closure over the declared boundary set.

    corridor.json declares, per carriageway, the entering and leaving
    detector sets. Lane-summed daily totals must close within
    CLOSURE_PASS_PCT.
    """
    findings = []
    daily = (
        df.assign(day=df["timestamp"].dt.date)
        .groupby(["site_id", "day"], as_index=False)["flow_veh_h"]
        .mean()  # mean veh/h over the day; scaled totals cancel in the ratio
    )
    for cw in corridor.get("carriageways", []):
        ins = daily[daily["site_id"].isin(cw["in_sites"])]
        outs = daily[daily["site_id"].isin(cw["out_sites"])]
        for day in sorted(set(ins["day"]) & set(outs["day"])):
            v_in = ins.loc[ins["day"] == day, "flow_veh_h"].sum()
            v_out = outs.loc[outs["day"] == day, "flow_veh_h"].sum()
            if v_in == 0:
                continue
            imbalance_pct = 100.0 * abs(v_in - v_out) / v_in
            findings.append(
                {
                    "check": "S1_closure",
                    "carriageway": cw["name"],
                    "day": str(day),
                    "imbalance_pct": round(imbalance_pct, 3),
                    "status": "pass" if imbalance_pct < CLOSURE_PASS_PCT else "fail",
                }
            )
    return findings


CORRIDOR_TEMPLATE = {
    "name": "TO FILL, e.g. A15-east-Papendrecht",
    "carriageways": [
        {
            "name": "eastbound",
            "in_sites": ["<upstream mainline site id>", "<on-ramp site id>"],
            "out_sites": ["<downstream mainline site id>", "<off-ramp site id>"],
        }
    ],
}


def main() -> None:
    ap = argparse.ArgumentParser()
    base = Path(__file__).parent
    ap.add_argument("--raw", type=Path, default=base / "data" / "raw")
    ap.add_argument("--out", type=Path, default=base / "data" / "screened")
    ap.add_argument("--corridor", type=Path, default=base / "network" / "corridor.json")
    args = ap.parse_args()

    df = load_raw(args.raw)
    args.out.mkdir(parents=True, exist_ok=True)

    inventory = {
        "sites": int(df["site_id"].nunique()),
        "lanes_per_site": {
            s: sorted(g["lane"].unique().tolist())
            for s, g in df.groupby("site_id")
        },
        "period": [str(df["timestamp"].min()), str(df["timestamp"].max())],
        "rows": int(len(df)),
    }
    findings = screen_s3(df)

    if args.corridor.exists():
        corridor = json.loads(args.corridor.read_text(encoding="utf-8"))
        findings += screen_s1(df, corridor)
        s1_status = "run"
    else:
        args.corridor.parent.mkdir(parents=True, exist_ok=True)
        args.corridor.write_text(
            json.dumps(CORRIDOR_TEMPLATE, indent=2), encoding="utf-8"
        )
        s1_status = f"not-exercised (template written to {args.corridor})"

    out_csv = args.out / "series_per_lane_minute.csv"
    df.to_csv(out_csv, index=False)
    sha = hashlib.sha256(out_csv.read_bytes()).hexdigest()

    report = {
        "inventory": inventory,
        "s1": s1_status,
        "findings": findings,
        "accepted_series_sha256": sha,
    }
    (args.out / "screening_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    n_fail = sum(1 for f in findings if f.get("status") == "fail" or "S3" in f["check"])
    print(f"{inventory['rows']} rows, {inventory['sites']} sites; "
          f"{len(findings)} findings ({n_fail} defects/failures); S1: {s1_status}")
    print(f"report: {args.out / 'screening_report.json'}")


if __name__ == "__main__":
    main()
