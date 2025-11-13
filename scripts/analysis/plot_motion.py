#!/usr/bin/env python3
"""Plot surge/sway/heave from a motion CSV."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motion_csv", type=Path, help="Input CSV (t,surge,sway,heave).")
    parser.add_argument(
        "--stride",
        type=int,
        default=10,
        help="Plot every Nth sample (default: 10).",
    )
    parser.add_argument(
        "--title",
        default=None,
        help="Custom plot title.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="PNG output path.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    data = np.genfromtxt(args.motion_csv, delimiter=",", skip_header=1)
    if data.ndim != 2 or data.shape[1] < 4:
        raise ValueError("Expected columns: timestamp_s,surge,sway,heave")
    slim = data[:: max(1, args.stride)]
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover
        raise RuntimeError("matplotlib required for plotting") from exc
    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    labels = ["Surge (m)", "Sway (m)", "Heave (m)"]
    colors = ["#1f77b4", "#2ca02c", "#d62728"]
    for ax, label, idx, color in zip(axes, labels, range(1, 4), colors):
        ax.plot(slim[:, 0] / 3600.0, slim[:, idx], linewidth=0.5, color=color)
        ax.set_ylabel(label)
        ax.grid(alpha=0.2, linewidth=0.4)
    axes[-1].set_xlabel("Time since start (hours)")
    fig.suptitle(args.title or args.motion_csv.name)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=200)
    plt.close(fig)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
