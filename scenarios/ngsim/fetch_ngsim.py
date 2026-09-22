"""Paginated, resumable NGSIM downloader (US DOT Socrata open data).

Usage:
    python fetch_ngsim.py --location us-101 [--page-size 500000] [--out data/raw]

No account or token required. Pages are ordered by :id (stable), written as
<location>_p####.csv via a .part temp file, and recorded in a manifest so an
interrupted download resumes at the first incomplete page.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.parse
import urllib.request
from pathlib import Path

BASE = "https://data.transportation.gov/resource/8ect-6jqj.csv"
RETRIES = 4
TIMEOUT_S = 300


def page_url(location: str, limit: int, offset: int) -> str:
    params = {
        "$where": f"location='{location}'",
        "$order": ":id",
        "$limit": str(limit),
        "$offset": str(offset),
    }
    return BASE + "?" + urllib.parse.urlencode(params)


def fetch_page(url: str, dest: Path) -> int:
    """Download one page to dest. Returns the number of data rows written."""
    last_err: Exception | None = None
    for attempt in range(RETRIES):
        try:
            tmp = dest.with_suffix(".part")
            req = urllib.request.Request(url, headers={"Accept": "text/csv"})
            with urllib.request.urlopen(req, timeout=TIMEOUT_S) as resp, \
                    open(tmp, "wb") as f:
                while chunk := resp.read(1 << 20):
                    f.write(chunk)
            rows = sum(1 for _ in open(tmp, "rb")) - 1  # minus header
            tmp.replace(dest)
            return max(rows, 0)
        except Exception as e:  # noqa: BLE001 - retry any transport error
            last_err = e
            wait = 2 ** attempt * 5
            print(f"  attempt {attempt + 1} failed ({e}); retrying in {wait}s")
            time.sleep(wait)
    raise RuntimeError(f"page download failed after {RETRIES} attempts: {last_err}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--location", required=True,
                    choices=["us-101", "i-80", "peachtree", "lankershim"])
    ap.add_argument("--page-size", type=int, default=500_000)
    ap.add_argument("--out", type=Path, default=Path(__file__).parent / "data" / "raw")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    manifest_path = args.out / f"{args.location}_manifest.json"
    manifest = (
        json.loads(manifest_path.read_text()) if manifest_path.exists()
        else {"location": args.location, "page_size": args.page_size, "pages": {}}
    )
    if manifest["page_size"] != args.page_size:
        sys.exit("page-size differs from an existing manifest; "
                 "delete the raw pages + manifest to restart.")

    page = 0
    total = sum(manifest["pages"].values())
    while True:
        dest = args.out / f"{args.location}_p{page:04d}.csv"
        key = str(page)
        if key in manifest["pages"]:
            rows = manifest["pages"][key]
        else:
            print(f"page {page} (offset {page * args.page_size}) ...")
            rows = fetch_page(
                page_url(args.location, args.page_size, page * args.page_size), dest
            )
            manifest["pages"][key] = rows
            manifest_path.write_text(json.dumps(manifest, indent=2))
            total += rows
            print(f"  {rows} rows ({total} cumulative)")
        if rows < args.page_size:  # short page = last page
            break
        page += 1

    print(f"done: {total} rows in {page + 1} pages -> {args.out}")


if __name__ == "__main__":
    main()
