#!/usr/bin/env python3
"""Plot one timeline log (docs/logging.md, row format v1 or v2) to a PNG beside the CSV.

    python tools/plot_run.py ../../04_SAFETY_AND_TESTING/Test_Results/2026-10-26_1430_manual-lap1.csv
    python tools/plot_run.py ../../04_SAFETY_AND_TESTING/Test_Results/*.csv --show

Panels, all against t_ms (seconds on the axis):
  1. ultrasonic distances (cm) for front / left / right / back / front-angled
  2. LiDAR sector minima (cm) front / left / right / back (v2: also front-right / front-left / lane)
  3. yaw (deg); v2: heading reference dashed
  4. v2 only: cross-track offset (cm), left wall distance (cm), poles counted
  5. commanded l / r
State changes (MANUAL, LEG1, WAIT, TURN, SETTLE, LEG2) are drawn as vertical lines, rows with a
note as markers. Old CampusLogger files (campus_run.csv, header time_ms,elapsed_s,...) are read
too: their columns are renamed to the v2 names.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

US_COLS = ["us_f", "us_l", "us_r", "us_b", "us_fa"]
LID_COLS = ["lid_f", "lid_l", "lid_r", "lid_b", "lid_fr", "lid_fl", "lid_corr"]
REQUIRED = ["t_ms", "state", "l", "r"] + US_COLS + ["yaw"]

# CampusLogger sketch (26.10.2026) column -> v2 column. lidar_* there are single degrees, lid_*
# in v2 are +-10 deg sector minima: close enough for a plot, not for a fine comparison.
CAMPUS_COLUMNS = {
    "time_ms": "t_ms", "phase": "state", "drive_l": "l", "drive_r": "r",
    "us_front_cm": "us_f", "us_left_cm": "us_l", "us_right_cm": "us_r", "us_back_cm": "us_b",
    "us_45deg_cm": "us_fa", "yaw_deg": "yaw", "pitch_deg": "pitch", "roll_deg": "roll",
    "lidar_front_cm": "lid_f", "lidar_left90_cm": "lid_l", "lidar_right90_cm": "lid_r",
    "lidar_fr45_cm": "lid_fr", "lidar_fl45_cm": "lid_fl", "lidar_corridor_cm": "lid_corr",
    "heading_ref_deg": "heading_ref", "yaw_rate_dps": "yaw_rate", "cross_track_cm": "cross_track",
    "imu_cal_sgam": "imu_cal", "pole_count": "poles", "pole_x_cm": "pole_x", "pole_y_cm": "last_pole_y",
    "pole_d_cm": "pole_d", "pole_fit_cm": "pole_fit", "cliff_baseline_cm": "cliff_base",
}


def load(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, comment="#")
    if "time_ms" in df.columns and "t_ms" not in df.columns:
        df = df.rename(columns=CAMPUS_COLUMNS)
    missing = [c for c in REQUIRED if c not in df.columns]
    if missing:
        sys.exit(f"{path.name}: missing columns {missing}; see docs/logging.md")
    df["t_s"] = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0
    return df


def plot(df: pd.DataFrame, title: str) -> plt.Figure:
    has_line = any(c in df.columns for c in ("cross_track", "wall_cm", "poles"))
    n = 5 if has_line else 4
    fig, axes = plt.subplots(n, 1, sharex=True, figsize=(12, 2.5 * n))
    t = df["t_s"]

    ax = axes[0]
    for c in US_COLS:
        ax.plot(t, df[c], label=c, linewidth=0.9)
    ax.set_ylabel("ultrasonic [cm]")
    ax.legend(loc="upper right", ncol=5, fontsize=8)

    ax = axes[1]
    for c in LID_COLS:
        if c in df.columns:
            ax.plot(t, df[c].astype(float).replace(0.0, float("nan")), label=c, linewidth=0.9)
    ax.set_ylabel("LiDAR min [cm]")
    ax.legend(loc="upper right", ncol=7, fontsize=8)

    ax = axes[2]
    ax.plot(t, df["yaw"], color="tab:purple", linewidth=0.9, label="yaw")
    if "heading_ref" in df.columns:
        ax.plot(t, df["heading_ref"], color="tab:purple", linestyle="--", linewidth=0.8, alpha=0.7,
                label="heading_ref")
        ax.legend(loc="upper right", fontsize=8)
    ax.set_ylabel("yaw [deg]")

    if has_line:
        ax = axes[3]
        if "cross_track" in df.columns:
            ax.plot(t, df["cross_track"], label="cross_track (+ = left)", linewidth=0.9)
        if "wall_cm" in df.columns:
            ax.plot(t, pd.to_numeric(df["wall_cm"], errors="coerce"), label="wall_cm", linewidth=0.9)
        ax.set_ylabel("line [cm]")
        if "poles" in df.columns:
            ax2 = ax.twinx()
            ax2.step(t, df["poles"], where="post", color="black", linewidth=0.9, label="poles")
            ax2.set_ylabel("poles")
        ax.legend(loc="upper left", fontsize=8)

    ax = axes[-1]
    ax.step(t, df["l"], where="post", label="l", linewidth=0.9)
    ax.step(t, df["r"], where="post", label="r", linewidth=0.9)
    ax.set_ylabel("command")
    ax.set_xlabel("t [s]")
    ax.legend(loc="upper right", fontsize=8)

    # state changes and notes on every panel
    changes = df.index[df["state"].ne(df["state"].shift())]
    for i in changes:
        for ax in axes:
            ax.axvline(t[i], color="grey", alpha=0.4, linewidth=0.8)
        axes[0].annotate(str(df.at[i, "state"]), (t[i], axes[0].get_ylim()[1]),
                         fontsize=7, rotation=90, va="top")
    if "note" in df.columns:
        notes = df[df["note"].notna() & (df["note"].astype(str).str.len() > 0)]
        for i, row in notes.iterrows():
            axes[-1].plot(row["t_s"], 0, marker="v", color="red")
            axes[-1].annotate(str(row["note"]), (row["t_s"], 0), fontsize=7, color="red",
                             xytext=(0, 6), textcoords="offset points", ha="center")

    fig.suptitle(title)
    fig.tight_layout()
    return fig


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", type=Path)
    ap.add_argument("--show", action="store_true", help="open a window instead of only saving the PNG")
    args = ap.parse_args()

    for path in args.csv:
        df = load(path)
        fig = plot(df, path.stem)
        out = path.with_suffix(".png")
        fig.savefig(out, dpi=120)
        print(f"{path.name}: {len(df)} rows, {df['t_s'].iloc[-1]:.1f} s -> {out.name}")
        if args.show:
            plt.show()
        plt.close(fig)


if __name__ == "__main__":
    main()
