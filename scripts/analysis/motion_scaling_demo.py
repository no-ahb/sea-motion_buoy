#!/usr/bin/env python3
"""Generate alternative motor-scaling previews from a motion CSV."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, Tuple

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motion_csv", type=Path, help="Input 10 Hz motion CSV.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for outputs (default: alongside motion CSV).",
    )
    parser.add_argument(
        "--prefix",
        default="bno_000",
        help="Prefix for generated files.",
    )
    parser.add_argument(
        "--stride",
        type=int,
        default=10,
        help="Plot every Nth sample to keep PNG size reasonable (default: 10).",
    )
    parser.add_argument(
        "--crop-window-sec",
        type=float,
        default=600.0,
        help="Rolling STD window (s) used to detect in-water segment.",
    )
    parser.add_argument(
        "--crop-std-threshold",
        type=float,
        default=0.05,
        help="STD threshold (m) for in-water detection.",
    )
    parser.add_argument(
        "--crop-prepad-sec",
        type=float,
        default=60.0,
        help="Seconds to include before detected in-water window.",
    )
    parser.add_argument(
        "--crop-postpad-sec",
        type=float,
        default=60.0,
        help="Seconds to include after detected in-water window.",
    )
    return parser.parse_args()


def load_motion(path: Path) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=",", skip_header=1)
    if data.ndim != 2 or data.shape[1] < 4:
        raise ValueError("Motion CSV must have at least 4 columns (t,surge,sway,heave)")
    return data


def percentile_scale(values: np.ndarray, lo: float, hi: float, knee: float = 0.05) -> np.ndarray:
    lo_v = np.percentile(values, lo)
    hi_v = np.percentile(values, hi)
    span = hi_v - lo_v if hi_v != lo_v else 1.0
    norm = np.clip((values - lo_v) / span, 0.0, 1.0)
    if knee <= 0:
        return norm
    return 0.5 + np.arctan((norm - 0.5) / knee) / math.pi


def rolling_mean(values: np.ndarray, window_samples: int) -> np.ndarray:
    if window_samples <= 1:
        return values.copy()
    kernel = np.ones(window_samples, dtype=np.float64) / float(window_samples)
    return np.convolve(values, kernel, mode="same")


def rolling_std(values: np.ndarray, window_samples: int) -> np.ndarray:
    if window_samples <= 1:
        return np.abs(values - np.mean(values))
    mean = rolling_mean(values, window_samples)
    mean_sq = rolling_mean(values ** 2, window_samples)
    var = np.clip(mean_sq - mean ** 2, 0.0, None)
    return np.sqrt(var)


def detect_spike_window(
    t: np.ndarray,
    heave: np.ndarray,
    percentile: float = 99.5,
    deploy_window: float = 600.0,
    retrieve_window: float = 3600.0,
    margin: float = 30.0,
) -> Tuple[float, float]:
    if heave.size < 2:
        return float(t[0]), float(t[-1])
    abs_delta = np.abs(np.diff(heave))
    threshold = np.percentile(abs_delta, percentile)
    spike_idx = np.where(abs_delta >= threshold)[0]
    if spike_idx.size == 0:
        return float(t[0]), float(t[-1])
    spike_times = t[spike_idx]
    start_candidates = spike_times[spike_times < deploy_window]
    end_candidates = spike_times[spike_times > retrieve_window]
    start_time = (
        float(start_candidates.max() + margin) if start_candidates.size else float(t[0])
    )
    end_time = (
        float(end_candidates.min() - margin) if end_candidates.size else float(t[-1])
    )
    if start_time >= end_time:
        return float(t[0]), float(t[-1])
    return start_time, end_time


def detect_window_std(
    t: np.ndarray,
    heave: np.ndarray,
    window_sec: float,
    std_threshold: float,
    prepad_sec: float,
    postpad_sec: float,
) -> Tuple[float, float]:
    if window_sec <= 0 or std_threshold <= 0 or heave.size < 2:
        return float(t[0]), float(t[-1])
    dt = np.median(np.diff(t))
    if not math.isfinite(dt) or dt <= 0:
        return float(t[0]), float(t[-1])
    window_samples = max(1, int(round(window_sec / dt)))
    stds = rolling_std(heave, window_samples)
    mask = stds >= std_threshold
    indices = np.where(mask)[0]
    if indices.size == 0:
        return float(t[0]), float(t[-1])
    start_idx = indices[0]
    end_idx = indices[-1] + window_samples
    start_time = t[start_idx] - prepad_sec
    end_time = t[min(len(t) - 1, end_idx)] + postpad_sec
    return max(start_time, float(t[0])), min(end_time, float(t[-1]))


def segment_bounds(t: np.ndarray, segment_sec: float) -> np.ndarray:
    if segment_sec <= 0:
        return np.array([t[0], t[-1]])
    start = t[0]
    end = t[-1]
    segments = [start]
    cur = start
    while cur < end:
        cur += segment_sec
        segments.append(min(cur, end))
    return np.array(segments)


def apply_scalings(
    t: np.ndarray,
    surge: np.ndarray,
    sway: np.ndarray,
    heave: np.ndarray,
    fs: float,
) -> Dict[str, Dict[str, np.ndarray]]:
    strategies: Dict[str, Dict[str, np.ndarray]] = {}
    data = {"surge": surge, "sway": sway, "heave": heave}

    # 1) Percentile + soft knee (0.5/99.5)
    strat1 = {
        axis: percentile_scale(values, 0.01, 99.99, knee=0.05)
        for axis, values in data.items()
    }
    strategies["percentile_soft"] = strat1

    # 2) Dual-band center (30 min low band)
    low_win = int(max(1, round(1800.0 * fs)))
    dual = {}
    for axis, values in data.items():
        low = rolling_mean(values, low_win)
        high = values - low
        low_norm = percentile_scale(low, 2.0, 98.0, knee=0.08)
        high_norm = percentile_scale(high, 15.0, 85.0, knee=0.04)
        combined = np.clip(
            0.5
            + 0.35 * (low_norm - 0.5)
            + 0.65 * (high_norm - 0.5),
            0.0,
            1.0,
        )
        dual[axis] = combined
    strategies["dual_band"] = dual

    # 3) Rolling STD modulation
    std_win = int(max(1, round(300.0 * fs)))
    base = {
        axis: percentile_scale(values, 2.0, 98.0, knee=0.05)
        for axis, values in data.items()
    }
    std_mod = {}
    for axis, values in data.items():
        std = rolling_std(values, std_win)
        std_norm = percentile_scale(std, 5.0, 95.0, knee=0.1)
        std_mod[axis] = np.clip(base[axis] * (0.5 + 0.5 * std_norm), 0.0, 1.0)
    strategies["rolling_std"] = std_mod

    # 4) Percentile remap + bias (0.1/99.9 to tanh curve)
    bias = {}
    for axis, values in data.items():
        lo = np.percentile(values, 0.1)
        hi = np.percentile(values, 99.9)
        span = hi - lo if hi != lo else 1.0
        z = np.clip((values - lo) / span, 0.0, 1.0)
        z = 2.0 * z - 1.0
        remap = 0.5 + 0.5 * np.tanh(z * 1.5)
        bias[axis] = np.clip(remap, 0.0, 1.0)
    strategies["percentile_bias"] = bias

    return strategies


def write_strategy_outputs(
    out_dir: Path,
    prefix: str,
    strategy_name: str,
    t: np.ndarray,
    strategy: Dict[str, np.ndarray],
    stride: int,
) -> None:
    out_csv = out_dir / f"{prefix}_{strategy_name}.csv"
    header = "timestamp_s,surge_norm,sway_norm,heave_norm\n"
    rel_t = t - t[0]
    arr = np.column_stack([rel_t, strategy["surge"], strategy["sway"], strategy["heave"]])
    np.savetxt(out_csv, arr, delimiter=",", header=header.strip(), comments="")
    print(f"Wrote {out_csv}")

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover
        print(f"Plotting skipped ({exc})")
        return
    slim = arr[:: max(1, stride)]
    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    labels = ["Surge", "Sway", "Heave"]
    colors = ["#1f77b4", "#2ca02c", "#d62728"]
    for ax, label, idx, color in zip(axes, labels, range(1, 4), colors):
        ax.plot(slim[:, 0] / 3600.0, slim[:, idx], linewidth=0.5, color=color)
        ax.set_ylabel(f"{label} (0-1)")
        ax.set_ylim(-0.05, 1.05)
        ax.grid(alpha=0.2, linewidth=0.4)
    axes[-1].set_xlabel("Time since deployment (hours)")
    fig.suptitle(f"{prefix}: {strategy_name}")
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out_png = out_dir / f"{prefix}_{strategy_name}.png"
    fig.savefig(out_png, dpi=200)
    plt.close(fig)
    print(f"Wrote {out_png}")


def main() -> None:
    args = parse_args()
    data = load_motion(args.motion_csv)
    out_dir = args.output_dir or args.motion_csv.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    t = data[:, 0]
    surge = data[:, 1]
    sway = data[:, 2]
    heave = data[:, 3]
    start, end = detect_window_std(
        t,
        heave,
        window_sec=args.crop_window_sec,
        std_threshold=args.crop_std_threshold,
        prepad_sec=args.crop_prepad_sec,
        postpad_sec=args.crop_postpad_sec,
    )
    spike_start, spike_end = detect_spike_window(
        t,
        heave,
        percentile=99.5,
        deploy_window=600.0,
        retrieve_window=t[-1] - 600.0,
        margin=30.0,
    )
    start = max(start, spike_start)
    end = min(end, spike_end)
    mask = (t >= start) & (t <= end)
    if not np.any(mask):
        raise ValueError("Automatic crop removed all samples; adjust thresholds.")
    t_crop = t[mask]
    surge_crop = surge[mask]
    sway_crop = sway[mask]
    heave_crop = heave[mask]
    fs = 1.0 / np.median(np.diff(t_crop))
    strategies = apply_scalings(t_crop, surge_crop, sway_crop, heave_crop, fs)
    for name, strategy in strategies.items():
        write_strategy_outputs(out_dir, args.prefix, name, t_crop, strategy, args.stride)


if __name__ == "__main__":
    main()
