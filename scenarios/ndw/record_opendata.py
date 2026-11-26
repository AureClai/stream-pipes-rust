"""Poll NDW Open Data (`snelheden_en_intensiteiten_meetgegevens.xml.gz`) and record
1-minute per-lane flow & speed series for the A15 Papendrecht corridor.

Usage:
    # 1. Build the index->lane map once from config_meetlocaties.xml.gz:
    python record_opendata.py --init-map

    # 2. Record live 1-minute per-lane snapshots (e.g. 240 min = 4 h):
    python record_opendata.py --duration-minutes 240

    # Or ingest a single local XML snapshot (testing / one-off):
    python record_opendata.py --local-xml path/to/snelheden_en_intensiteiten_meetgegevens.xml

Outputs a ';'-separated CSV (`data/raw/a15_eb_1min_lanes.csv`) directly
compatible with `prepare_dexter_export.py`.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import io
import json
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

BASE = Path(__file__).parent
RAW_DIR = BASE / "data" / "raw"
CONFIG_URL = "https://opendata.ndw.nu/snelheden_en_intensiteiten_configuratie_meetlocaties.xml.gz"
DATA_URL = "https://opendata.ndw.nu/snelheden_en_intensiteiten_meetgegevens.xml.gz"
CONFIG_GZ = RAW_DIR / "config_meetlocaties.xml.gz"
SITE_MAP_JSON = RAW_DIR / "a15_site_map.json"
OUT_CSV = RAW_DIR / "a15_eb_1min_lanes.csv"

# A15 Eastbound (km 72.5 -> 81.2): Alblasserdam -> Exit 23 Papendrecht -> Exit 24 Sliedrecht-West
A15_EB_SITES = [
    "RWS01_MONIBAS_0151hrr0725ra",
    "RWS01_MONIBAS_0151hrr0729ra",
    "RWS01_MONIBAS_0151hrr0732ra",
    "RWS01_MONIBAS_0151hrr0735ra",
    "RWS01_MONIBAS_0151hrr0737ra",
    "RWS01_MONIBAS_0151hrr0739ra",
    "RWS01_MONIBAS_0151hrr0740ra",
    "RWS01_MONIBAS_0151hrr0742ra",
    "RWS01_MONIBAS_0151hrr0746ra",
    "RWS01_MONIBAS_0151hrr0750ra",
    "RWS01_MONIBAS_0151hrr0753ra",
    "RWS01_MONIBAS_0151hrr0758ra",
    "RWS01_MONIBAS_0151hrr0764ra",
    "RWS01_MONIBAS_0151hrr0770ra",
    "RWS01_MONIBAS_0151hrr0773ra",
    "RWS01_MONIBAS_0151hrr0776ra",
    "RWS01_MONIBAS_0151hrr0778ra_1",
    "RWS01_MONIBAS_0150vwa0780ra",   # Exit 23 Papendrecht off-ramp
    "RWS01_MONIBAS_0151hrr0782ra",
    "RWS01_MONIBAS_0151hrr0784ra",
    "GEO0K_K_RWSTI358186",           # Entry 23 Papendrecht on-ramp (015vwb078800)
    "RWS01_MONIBAS_0151hrr0790ra",
    "RWS01_MONIBAS_0151hrr0794ra",
    "RWS01_MONIBAS_0151hrr0797ra",
    "RWS01_MONIBAS_0151hrr0801ra",
    "GEO0C_Z_RWSTI358373",           # Exit 24 Sliedrecht-West off-ramp (015vwa080517)
    "RWS01_MONIBAS_0151hrr0806ra",
    "RWS01_MONIBAS_0151hrr0808ra",
    "RWS01_MONIBAS_0151hrr0812ra",
]


def _local(tag: str) -> str:
    return tag.split("}", 1)[-1]


def build_site_map(config_gz: Path, targets: set[str]) -> dict[str, dict]:
    """Extract total (all-length) flow/speed index -> lane mappings for `targets`."""
    if not config_gz.exists():
        print(f"Downloading {CONFIG_URL} -> {config_gz} ...", flush=True)
        config_gz.parent.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(CONFIG_URL, config_gz)

    site_map: dict[str, dict] = {}
    with gzip.open(config_gz, "rb") as f:
        for _, elem in ET.iterparse(f, events=("end",)):
            if _local(elem.tag) != "measurementSite":
                continue
            sid = elem.attrib.get("id")
            if sid in targets:
                name = sid
                n_lanes = 0
                slots: dict[str, dict[str, str]] = {}  # index -> {lane, kind}
                for child in elem:
                    ct = _local(child.tag)
                    if ct == "measurementSiteName":
                        name = "".join(child.itertext()).strip()
                    elif ct == "measurementSiteNumberOfLanes":
                        n_lanes = int("".join(child.itertext()).strip() or "0")
                    elif ct == "measurementSpecificCharacteristics":
                        idx = child.attrib.get("index", "")
                        has_length = any(_local(s.tag) == "lengthCharacteristic" for s in child.iter())
                        if has_length:
                            continue
                        val_type = None
                        lane = None
                        for sub in child.iter():
                            st = _local(sub.tag)
                            if st == "specificMeasurementValueType" and sub.text:
                                val_type = sub.text.strip()
                            elif st == "laneNumber" and sub.text:
                                lane = sub.text.strip()
                        if val_type in ("trafficFlow", "trafficSpeed") and lane:
                            slots[idx] = {
                                "lane": lane,
                                "kind": "flow" if val_type == "trafficFlow" else "speed",
                            }
                site_map[sid] = {"name": name, "lanes": n_lanes, "slots": slots}
            elem.clear()
    return site_map


def parse_snapshot(stream, site_map: dict[str, dict]) -> list[tuple[str, str, str, str, str]]:
    """Extract (site_id, timestamp, lane, flow_veh_h, speed_kmh) rows from an XML stream."""
    targets = set(site_map)
    rows: list[tuple[str, str, str, str, str]] = []
    for _, elem in ET.iterparse(stream, events=("end",)):
        if _local(elem.tag) != "siteMeasurements":
            continue
        sid = None
        ts = ""
        per_lane: dict[str, dict[str, str]] = {}
        for child in elem:
            ct = _local(child.tag)
            if ct == "measurementSiteReference":
                sid = child.attrib.get("id")
            elif ct == "measurementTimeDefault":
                for sub in child:
                    if _local(sub.tag) == "timeValue" and sub.text:
                        ts = sub.text.strip()
            elif ct == "physicalQuantity" and sid in targets:
                idx = child.attrib.get("index", "")
                slot = site_map[sid]["slots"].get(idx)
                if not slot:
                    continue
                val = None
                for desc in child.iter():
                    dt = _local(desc.tag)
                    if slot["kind"] == "flow" and dt == "vehicleFlowRate" and desc.text:
                        val = desc.text.strip()
                        break
                    if slot["kind"] == "speed" and dt == "speed" and desc.text:
                        raw_v = desc.text.strip()
                        val = "" if raw_v == "-1.0" else raw_v
                        break
                if val is not None:
                    per_lane.setdefault(slot["lane"], {})[slot["kind"]] = val
        if sid in targets and ts:
            for lane in sorted(per_lane, key=lambda s: int(s)):
                vals = per_lane[lane]
                rows.append((sid, ts, f"lane{lane}", vals.get("flow", ""), vals.get("speed", "")))
        elem.clear()
    return rows


def load_seen_keys(csv_path: Path) -> set[tuple[str, str, str]]:
    seen: set[tuple[str, str, str]] = set()
    if not csv_path.exists():
        return seen
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.reader(f, delimiter=";")
        next(reader, None)
        for row in reader:
            if len(row) >= 3:
                seen.add((row[0], row[1], row[2]))
    return seen


def append_rows(
    csv_path: Path,
    rows: list[tuple[str, str, str, str, str]],
    seen: set[tuple[str, str, str]],
) -> int:
    new_rows = [r for r in rows if (r[0], r[1], r[2]) not in seen]
    if not new_rows:
        return 0
    write_header = not csv_path.exists()
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("a", encoding="utf-8", newline="") as f:
        writer = csv.writer(f, delimiter=";")
        if write_header:
            writer.writerow(["meetlocatie id", "periode start", "rijstrook", "intensiteit", "snelheid"])
        writer.writerows(new_rows)
    for r in new_rows:
        seen.add((r[0], r[1], r[2]))
    return len(new_rows)


def fetch_once(site_map: dict[str, dict]) -> list[tuple[str, str, str, str, str]]:
    req = urllib.request.Request(DATA_URL, headers={"User-Agent": "stream-pipes-ndw-collector/1.0"})
    with urllib.request.urlopen(req, timeout=30) as resp:
        payload = resp.read()
    with gzip.GzipFile(fileobj=io.BytesIO(payload)) as gz:
        return parse_snapshot(gz, site_map)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--init-map", action="store_true", help="Build a15_site_map.json and exit.")
    ap.add_argument("--local-xml", type=Path, help="Ingest a single local XML(.gz) file instead of polling.")
    ap.add_argument("--duration-minutes", type=float, default=240.0, help="Total polling duration in minutes.")
    ap.add_argument("--interval", type=float, default=30.0, help="Poll interval in seconds.")
    args = ap.parse_args()

    if args.init_map or not SITE_MAP_JSON.exists():
        site_map = build_site_map(CONFIG_GZ, set(A15_EB_SITES))
        SITE_MAP_JSON.parent.mkdir(parents=True, exist_ok=True)
        SITE_MAP_JSON.write_text(json.dumps(site_map, indent=2), encoding="utf-8")
        print(f"Mapped {len(site_map)}/{len(A15_EB_SITES)} A15 sites -> {SITE_MAP_JSON}")
        missing = sorted(set(A15_EB_SITES) - set(site_map))
        if missing:
            print(f"Missing from config XML: {missing}", file=sys.stderr)
        if args.init_map:
            return
    else:
        site_map = json.loads(SITE_MAP_JSON.read_text(encoding="utf-8"))

    seen = load_seen_keys(OUT_CSV)
    if args.local_xml:
        opener = gzip.open if args.local_xml.suffix == ".gz" else open
        with opener(args.local_xml, "rb") as f:
            rows = parse_snapshot(f, site_map)
        added = append_rows(OUT_CSV, rows, seen)
        print(f"Ingested {args.local_xml.name}: +{added} rows (total {len(seen)}) -> {OUT_CSV}")
        return

    deadline = time.monotonic() + args.duration_minutes * 60.0
    print(f"Recording {len(site_map)} sites -> {OUT_CSV} for {args.duration_minutes:.0f} min (poll every {args.interval:.0f}s)...", flush=True)
    while time.monotonic() < deadline:
        try:
            rows = fetch_once(site_map)
            added = append_rows(OUT_CSV, rows, seen)
            if added:
                ts_sample = max(r[1] for r in rows)
                off = [f"{r[4]}km/h" for r in rows if r[0] == "RWS01_MONIBAS_0150vwa0780ra"]
                main_pre = [f"{r[4]}km/h" for r in rows if r[0] == "RWS01_MONIBAS_0151hrr0770ra"]
                now_utc = datetime.now(timezone.utc).strftime("%H:%M:%S")
                print(f"[{now_utc} UTC] +{added} rows (total {len(seen)}) ts={ts_sample} | 0770={main_pre} vwa0780={off}", flush=True)
        except Exception as exc:
            print(f"[warn] poll failed: {exc}", file=sys.stderr, flush=True)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(args.interval, remaining))
    print(f"Done. Total rows saved: {len(seen)} in {OUT_CSV}")


if __name__ == "__main__":
    main()
