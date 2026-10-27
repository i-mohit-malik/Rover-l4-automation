#!/usr/bin/env python3
"""Download the newest timeline log from the box into 04_SAFETY_AND_TESTING/Test_Results/.

    python tools/fetch_log.py --box 6 --note manual-lap1
    python tools/fetch_log.py --box 6 --out D:/somewhere/else

The laptop must be on the box Wi-Fi (CAS-Box-NN). Uses the /log endpoints described in
docs/logging.md. The box writes one run_<n>.csv per boot and per START RUN; "newest" is the
highest n.
"""
from __future__ import annotations

import argparse
import datetime as dt
import re
import sys
from pathlib import Path

import requests

# L4_Rover/03_ENGINEERING/Rover-l4-automation/tools/ -> L4_Rover/04_SAFETY_AND_TESTING/Test_Results/
RUNS = Path(__file__).resolve().parents[3] / "04_SAFETY_AND_TESTING" / "Test_Results"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--box", type=int, required=True, help="kit number (BOX_ID), 1..10")
    ap.add_argument("--note", default="run", help="short note for the file name, e.g. manual-lap1")
    ap.add_argument("--file", help="log file name on the box; default: newest")
    ap.add_argument("--out", type=Path, default=RUNS, help=f"target folder; default: {RUNS}")
    args = ap.parse_args()

    base = f"http://192.168.{10 + args.box}.1"
    try:
        r = requests.get(f"{base}/log", timeout=5)
    except requests.RequestException as e:
        sys.exit(f"box not reachable at {base}: {e}")
    if r.status_code == 404:
        sys.exit("the box firmware has no /log endpoint yet (Logger not implemented)")
    r.raise_for_status()

    name = args.file
    if not name:
        names = re.findall(r"run_\d+\.csv", r.text)
        if not names:
            sys.exit("no log files on the box")
        name = max(names, key=lambda n: int(re.search(r"\d+", n).group()))  # run_10 after run_9

    data = requests.get(f"{base}/log", params={"get": name}, timeout=30)
    data.raise_for_status()

    args.out.mkdir(parents=True, exist_ok=True)
    safe = re.sub(r"[^A-Za-z0-9_-]+", "-", args.note).strip("-") or "run"
    out = args.out / f"{dt.datetime.now():%Y-%m-%d_%H%M}_{safe}.csv"
    out.write_bytes(data.content)
    print(f"{name} ({len(data.content)} bytes) -> {out}")
    print(f"next: add a row to {args.out / 'test_log.md'} and run tools/plot_run.py on it")


if __name__ == "__main__":
    main()
