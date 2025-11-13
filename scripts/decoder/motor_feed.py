#!/usr/bin/env python3
"""Post-process reconstructed motion CSVs into motor-friendly feeds.

Steps performed:
1. Automatically crop out deployment / retrieval spikes based on heave transients.
2. Normalize surge/sway/heave jointly into [0, 1] (optionally respecting user-supplied global min/max).
3. Apply fade-in/fade-out envelopes so motors ramp smoothly.
4. Optionally emit a diagnostic plot showing the envelope and per-axis tracks.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Tuple

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "motion_csv",
        type=Path,
        help="Input 10 Hz motion CSV from sea-movement_decoder (--reconstruct-csv output).",
    )
    parser.add_argument(
        "--output-csv",
        type=Path,
        required=True,
        help="Where to write the normalized + faded feed (CSV).",
    )
    parser.add_argument(
        "--output-plot",
        type=Path,
        help="Optional diagnostic PNG (requires matplotlib).",
    )
    parser.add_argument(
        "--deploy-window",
        type=float,
        default=600.0,
        help="Consider spikes before this time (s) as deployment artefacts (default: 600).",
    )
    parser.add_argument(
        "--retrieve-window",
        type=float,
        default=2400.0,
        help="Consider spikes after this time (s) as retrieval artefacts (default: 2400).",
    )
    parser.add_argument(
        "--margin",
        type=float,
        default=25.0,
        help="Seconds to pad away from detected spike windows when cropping (default: 25).",
    )
    parser.add_argument(
        "--fade-seconds",
        type=float,
        default=10.0,
        help="Length of fade-in/fade-out envelopes in seconds (default: 10).",
    )
    parser.add_argument(
        "--percentile",
        type=float,
        default=99.5,
        help="Percentile of |Δheave| to treat as a spike (default: 99.5).",
    )
    parser.add_argument(
        "--fallback-start",
        type=float,
        default=220.0,
        help="Fallback start time (s) if no early spikes are found (default: 220).",
    )
    parser.add_argument(
        "--fallback-end",
        type=float,
        default=2580.0,
        help="Fallback end time (s) if no late spikes are found (default: 2580).",
    )
    parser.add_argument(
        "--global-min",
        type=float,
        help="Force this value as the shared min before normalization (spans deployments).",
    )
    parser.add_argument(
        "--global-max",
        type=float,
        help="Force this value as the shared max before normalization (spans deployments).",
    )
    parser.add_argument(
        "--hampel-k",
        type=int,
        default=9,
        help="Window (samples, odd) for Hampel spike removal (0 disables, default: 9).",
    )
    parser.add_argument(
        "--hampel-t",
        type=float,
        default=3.0,
        help="Sigma multiplier for Hampel threshold (default: 3.0).",
    )
    parser.add_argument(
        "--smooth-samples",
        type=int,
        default=11,
        help="Moving-average window (samples) after Hampel (0 disables, default: 11).",
    )
    parser.add_argument(
        "--max-step",
        type=float,
        default=0.02,
        help="Max per-sample change after smoothing (normalized units, default: 0.02).",
    )
    parser.add_argument(
        "--crop-window-sec",
        type=float,
        default=120.0,
        help="Rolling window (s) used to detect in-water segments via std threshold.",
    )
    parser.add_argument(
        "--crop-std-threshold",
        type=float,
        default=0.05,
        help="Std (m) threshold for the rolling window to treat as in-water motion.",
    )
    parser.add_argument(
        "--crop-prepad-sec",
        type=float,
        default=30.0,
        help="Seconds to include before detected in-water window.",
    )
    parser.add_argument(
        "--crop-postpad-sec",
        type=float,
        default=30.0,
        help="Seconds to include after detected in-water window.",
    )
    return parser.parse_args()


def _load_motion(path: Path) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"{path} not found")
    data = np.genfromtxt(path, delimiter=",", skip_header=1)
    if data.ndim != 2 or data.shape[1] < 4:
        raise ValueError("Motion CSV must have at least four columns (t, surge, sway, heave)")
    return data


def _detect_window(
    t: np.ndarray,
    heave: np.ndarray,
    percentile: float,
    deploy_window: float,
    retrieve_window: float,
    margin: float,
    fallback_start: float,
    fallback_end: float,
) -> Tuple[float, float]:
    if heave.size < 2:
        return float(t[0]), float(t[-1])
    abs_delta = np.abs(np.diff(heave))
    try:
        threshold = np.percentile(abs_delta, percentile)
    except Exception as exc:
        raise ValueError(f"Failed to compute percentile {percentile}: {exc}") from exc
    spike_idx = np.where(abs_delta >= threshold)[0]
    if spike_idx.size == 0:
        return fallback_start, fallback_end
    spike_times = t[spike_idx]
    start_candidates = spike_times[spike_times < deploy_window]
    end_candidates = spike_times[spike_times > retrieve_window]
    start_time = (
        float(start_candidates.max() + margin)
        if start_candidates.size
        else fallback_start
    )
    end_time = (
        float(end_candidates.min() - margin)
        if end_candidates.size
        else fallback_end
    )
    if start_time >= end_time:
        return float(t[0]), float(t[-1])
    return start_time, end_time


def _detect_window_std(
    t: np.ndarray,
    heave: np.ndarray,
    window_sec: float,
    std_threshold: float,
    prepad_sec: float,
    postpad_sec: float,
) -> Tuple[float, float] | None:
    if window_sec <= 0 or std_threshold <= 0 or heave.size < 2:
        return None
    dt = np.median(np.diff(t))
    if dt <= 0 or not math.isfinite(dt):
        return None
    window_samples = max(1, int(round(window_sec / dt)))
    if window_samples >= heave.size:
        return None
    stds = _rolling_std(heave, window_samples)
    mask = stds >= std_threshold
    true_idx = np.where(mask)[0]
    if true_idx.size == 0:
        return None
    start_idx = true_idx[0]
    end_idx = true_idx[-1] + window_samples
    start_time = t[start_idx] - prepad_sec
    end_time = t[min(len(t) - 1, end_idx)] + postpad_sec
    start_time = max(start_time, float(t[0]))
    end_time = min(end_time, float(t[-1]))
    if start_time >= end_time:
        return None
    return start_time, end_time


def _build_envelope(t: np.ndarray, fade_seconds: float) -> Tuple[np.ndarray, int]:
    n = t.size
    if n == 0 or fade_seconds <= 0:
        return np.ones(n, dtype=np.float64), 0
    if n == 1:
        return np.array([1.0], dtype=np.float64), 0
    dt = np.median(np.diff(t))
    dt = dt if math.isfinite(dt) and dt > 0 else 0.1
    fade_samples = min(n // 2, max(1, int(round(fade_seconds / dt))))
    envelope = np.ones(n, dtype=np.float64)
    fade_in = np.linspace(0.0, 1.0, fade_samples, endpoint=True)
    fade_out = np.linspace(1.0, 0.0, fade_samples, endpoint=True)
    envelope[:fade_samples] = fade_in
    envelope[-fade_samples:] = np.minimum(envelope[-fade_samples:], fade_out)
    return envelope, fade_samples


def _sma(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or values.size == 0:
        return values
    if window % 2 == 0:
        window += 1  # ensure symmetry
    pad = window // 2
    padded = np.pad(values, pad, mode="edge")
    kernel = np.ones(window, dtype=np.float64) / float(window)
    return np.convolve(padded, kernel, mode="valid")


def _rolling_std(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or values.size == 0:
        return np.abs(values - np.mean(values))
    kernel = np.ones(window, dtype=np.float64) / float(window)
    mean = np.convolve(values, kernel, mode="same")
    mean_sq = np.convolve(values ** 2, kernel, mode="same")
    var = np.clip(mean_sq - mean ** 2, 0.0, None)
    return np.sqrt(var)


def _hampel(values: np.ndarray, window: int, threshold: float) -> np.ndarray:
    if window <= 1 or values.size == 0:
        return values
    if window % 2 == 0:
        raise ValueError("Hampel window must be odd")
    pad = window // 2
    padded = np.pad(values, pad, mode="edge")
    from numpy.lib.stride_tricks import sliding_window_view

    win = sliding_window_view(padded, window)
    med = np.median(win, axis=1)
    mad = np.median(np.abs(win - med[:, None]), axis=1)
    sigma = 1.4826 * (mad + 1e-12)
    y = values.copy()
    mask = np.abs(values - med) > threshold * sigma
    y[mask] = med[mask]
    return y


def _slew_limit(values: np.ndarray, max_step: float) -> np.ndarray:
    if max_step <= 0 or values.size == 0:
        return values
    y = np.empty_like(values)
    y[0] = values[0]
    for idx in range(1, values.size):
        lower = y[idx - 1] - max_step
        upper = y[idx - 1] + max_step
        y[idx] = float(np.clip(values[idx], lower, upper))
    return y


def main() -> int:
    args = parse_args()
    data = _load_motion(args.motion_csv)
    t = data[:, 0]
    surge = data[:, 1]
    sway = data[:, 2]
    heave = data[:, 3]

    start_time = end_time = None
    std_window = _detect_window_std(
        t,
        heave,
        window_sec=args.crop_window_sec,
        std_threshold=args.crop_std_threshold,
        prepad_sec=args.crop_prepad_sec,
        postpad_sec=args.crop_postpad_sec,
    )
    spike_window = _detect_window(
        t,
        heave,
        percentile=args.percentile,
        deploy_window=args.deploy_window,
        retrieve_window=args.retrieve_window,
        margin=args.margin,
        fallback_start=args.fallback_start,
        fallback_end=args.fallback_end,
    )
    if std_window is not None:
        start_time, end_time = std_window
    else:
        start_time, end_time = spike_window
    if spike_window is not None:
        s_spike, e_spike = spike_window
        start_time = max(start_time, s_spike)
        end_time = min(end_time, e_spike)
    mask = (t >= start_time) & (t <= end_time)
    if not np.any(mask):
        raise RuntimeError("Cropping removed all samples; adjust percentile or window settings.")
    seg_t = t[mask]
    seg_surge = surge[mask]
    seg_sway = sway[mask]
    seg_heave = heave[mask]

    combined = np.concatenate([seg_surge, seg_sway, seg_heave])
    min_all = args.global_min if args.global_min is not None else float(combined.min())
    max_all = args.global_max if args.global_max is not None else float(combined.max())
    if not math.isfinite(min_all) or not math.isfinite(max_all):
        raise ValueError("Global min/max must be finite.")
    if max_all <= min_all:
        raise ValueError("global max must be greater than global min for normalization.")
    denom = max_all - min_all
    surge_norm = (seg_surge - min_all) / denom
    sway_norm = (seg_sway - min_all) / denom
    heave_norm = (seg_heave - min_all) / denom

    # Playback-only conditioning (does not touch archived data)
    if args.hampel_k and args.hampel_k > 1:
        surge_norm = _hampel(surge_norm, args.hampel_k, args.hampel_t)
        sway_norm = _hampel(sway_norm, args.hampel_k, args.hampel_t)
        heave_norm = _hampel(heave_norm, args.hampel_k, args.hampel_t)

    if args.smooth_samples and args.smooth_samples > 1:
        surge_norm = _sma(surge_norm, args.smooth_samples)
        sway_norm = _sma(sway_norm, args.smooth_samples)
        heave_norm = _sma(heave_norm, args.smooth_samples)

    if args.max_step and args.max_step > 0:
        surge_norm = _slew_limit(surge_norm, args.max_step)
        sway_norm = _slew_limit(sway_norm, args.max_step)
        heave_norm = _slew_limit(heave_norm, args.max_step)

    envelope, fade_samples = _build_envelope(seg_t, args.fade_seconds)
    surge_faded = surge_norm * envelope
    sway_faded = sway_norm * envelope
    heave_faded = heave_norm * envelope

    header = "timestamp_s,envelope,surge_norm,sway_norm,heave_norm"
    out = np.column_stack([seg_t, envelope, surge_faded, sway_faded, heave_faded])
    np.savetxt(args.output_csv, out, delimiter=",", header=header, comments="")

    if args.output_plot:
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:  # pragma: no cover - optional dependency
            raise RuntimeError("Matplotlib is required for plotting.") from exc
        fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
        axes[0].plot(seg_t, envelope, color="black", linewidth=1.0)
        axes[0].set_ylabel("Envelope")
        axes[0].set_ylim(-0.05, 1.05)
        axes[0].grid(alpha=0.2, linewidth=0.5)
        for ax, values, label, color in zip(
            axes[1:],
            (surge_faded, sway_faded, heave_faded),
            ("Surge 0-1", "Sway 0-1", "Heave 0-1"),
            ("#1f77b4", "#2ca02c", "#d62728"),
        ):
            ax.plot(seg_t, values, color=color, linewidth=0.8)
            ax.set_ylabel(label)
            ax.set_ylim(-0.05, 1.05)
            ax.grid(alpha=0.2, linewidth=0.5)
        axes[-1].set_xlabel("Absolute time (s since recording start)")
        fig.suptitle(
            f"Motor feed window {start_time:.1f}-{end_time:.1f} s "
            f"(norm [{min_all:.3f},{max_all:.3f}], fade {args.fade_seconds:.1f}s)"
        )
        fig.tight_layout(rect=(0, 0, 1, 0.93))
        fig.savefig(args.output_plot, dpi=200)

    print(f"Window: {start_time:.3f} s → {end_time:.3f} s ({seg_t.size} samples)")
    print(f"Normalization: global min={min_all:.6f}, max={max_all:.6f}")
    print(f"Fade: {fade_samples} samples (~{args.fade_seconds:.2f} s)")
    print(f"Wrote CSV: {args.output_csv}")
    if args.output_plot:
        print(f"Wrote plot: {args.output_plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
