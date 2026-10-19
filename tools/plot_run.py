#!/usr/bin/env python3
"""Plot one timeline log (docs/logging.md, row format v1) to a PNG beside the CSV.

    python tools/plot_run.py ../../04_SAFETY_AND_TESTING/Test_Results/2026-10-26_1430_manual-lap1.csv
    python tools/plot_run.py ../../04_SAFETY_AND_TESTING/Test_Results/*.csv --show

Panels, all against t_ms (seconds on the axis):
  1. ultrasonic distances (cm) for front / left / right / back / front-angled
  2. LiDAR sector minima (cm) front / left / right / back
  3. yaw (deg)
  4. commanded l / r
State changes are drawn as vertical lines, rows with a note as markers.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

US_COLS = ["us_f", "us_l", "us_r", "us_b", "us_fa"]
LID_COLS = ["lid_f", "lid_l", "lid_r", "lid_b"]
REQUIRED = ["t_ms", "state", "l", "r"] + US_COLS + ["yaw"]


def load(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path, comment="#")
    missing = [c for c in REQUIRED if c not in df.columns]
    if missing:
        sys.exit(f"{path.name}: missing columns {missing}; see docs/logging.md")
    df["t_s"] = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0
    return df


def plot(df: pd.DataFrame, title: str) -> plt.Figure:
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(12, 10))
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
    ax.legend(loc="upper right", ncol=4, fontsize=8)

    ax = axes[2]
    ax.plot(t, df["yaw"], color="tab:purple", linewidth=0.9)
    ax.set_ylabel("yaw [deg]")

    ax = axes[3]
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
            axes[3].plot(row["t_s"], 0, marker="v", color="red")
            axes[3].annotate(str(row["note"]), (row["t_s"], 0), fontsize=7, color="red",
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
