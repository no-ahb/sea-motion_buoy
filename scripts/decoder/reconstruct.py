"""Motion reconstitution helpers for sea-movement decoder."""

from __future__ import annotations

import math
from typing import Dict, Tuple

import numpy as np


def quat_to_r_xyzw(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Rotation matrix that maps sensor-frame vectors into the world frame."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def rotate_series_to_world(
    lx: np.ndarray,
    ly: np.ndarray,
    lz: np.ndarray,
    qi: np.ndarray,
    qj: np.ndarray,
    qk: np.ndarray,
    qr: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Rotate linear acceleration samples into the world frame."""
    n = lx.shape[0]
    out = np.empty((n, 3), dtype=np.float64)
    for idx in range(n):
        r = quat_to_r_xyzw(qi[idx], qj[idx], qk[idx], qr[idx])
        out[idx, :] = r @ np.array([lx[idx], ly[idx], lz[idx]], dtype=np.float64)
    return out[:, 0], out[:, 1], out[:, 2]


def _butter_filter(acc: np.ndarray, fs: float, cutoff: float, kind: str) -> np.ndarray:
    from scipy.signal import butter, filtfilt  # type: ignore

    nyq = fs * 0.5
    normalized = cutoff / nyq
    b, a = butter(4, normalized, btype=kind)
    return filtfilt(b, a, acc, method="gust")


def bandlimit(acc: np.ndarray, fs: float, f_hp: float = 0.05, f_lp: float = 3.5) -> np.ndarray:
    """Apply high-pass then low-pass filtering to bound the usable band."""
    try:
        x = _butter_filter(acc, fs, f_hp, "highpass") if f_hp > 0 else acc
        x = _butter_filter(x, fs, f_lp, "lowpass") if f_lp > 0 else x
        return x
    except Exception:
        # FIR fallback: remove slow drift via moving-average subtraction, then smooth.
        if f_hp > 0:
            win_hp = int(max(fs / f_hp, 10 * fs))
            hp_kernel = np.ones(win_hp, dtype=np.float64) / float(win_hp)
            x = acc - np.convolve(acc, hp_kernel, mode="same")
        else:
            x = acc.copy()
        if f_lp > 0:
            win_lp = max(int(fs / (2.0 * f_lp)), 5)
            lp_kernel = np.ones(win_lp, dtype=np.float64) / float(win_lp)
            x = np.convolve(x, lp_kernel, mode="same")
        return x


def _numpy_detrend(values: np.ndarray, kind: str) -> np.ndarray:
    if kind == "constant":
        return values - np.mean(values)
    if kind == "linear":
        x = np.arange(values.size, dtype=np.float64)
        coeffs = np.polyfit(x, values, 1)
        trend = coeffs[0] * x + coeffs[1]
        return values - trend
    raise ValueError(f"Unsupported detrend kind: {kind}")


def _detrend(values: np.ndarray, kind: str) -> np.ndarray:
    try:
        from scipy.signal import detrend as scipy_detrend  # type: ignore

        return scipy_detrend(values, type=kind)
    except Exception:
        return _numpy_detrend(values, kind)


def _integrate(acc: np.ndarray, fs: float) -> np.ndarray:
    if acc.size == 0:
        return np.array([], dtype=np.float64)
    if acc.size == 1:
        return np.array([0.0], dtype=np.float64)
    dt = 1.0 / fs
    increments = (acc[1:] + acc[:-1]) * 0.5 * dt
    out = np.empty_like(acc)
    out[0] = 0.0
    out[1:] = np.cumsum(increments, dtype=np.float64)
    return out


def integrate_twice(acc: np.ndarray, fs: float) -> np.ndarray:
    """Double integrate acceleration with basic drift suppression."""
    v = _integrate(acc, fs)
    v = _detrend(v, "constant")
    x = _integrate(v, fs)
    x = _detrend(x, "linear")
    return x


def decimate_to_rate(x: np.ndarray, fs: float, out_fs: float = 10.0) -> Tuple[np.ndarray, float]:
    factor = int(round(fs / out_fs))
    if factor < 1:
        raise ValueError("Output sampling rate must be <= input rate")
    if factor == 1:
        return x.copy(), fs
    if not math.isclose(fs / factor, out_fs, rel_tol=1e-6):
        raise ValueError("Input/output sampling rates must be integer-related for decimation")
    try:
        from scipy.signal import firwin, filtfilt  # type: ignore

        cutoff = 0.45 * out_fs
        taps = firwin(numtaps=101, cutoff=cutoff, fs=fs)
        x_filt = filtfilt(taps, [1.0], x)
    except Exception:
        kernel_len = max(5, factor * 2 + 1)
        kernel = np.ones(kernel_len, dtype=np.float64) / float(kernel_len)
        x_filt = np.convolve(x, kernel, mode="same")
    usable = (x_filt.size // factor) * factor
    return x_filt[:usable:factor], out_fs


def reconstruct_motion(
    t_ms: np.ndarray,
    lx: np.ndarray,
    ly: np.ndarray,
    lz: np.ndarray,
    qi: np.ndarray,
    qj: np.ndarray,
    qk: np.ndarray,
    qr: np.ndarray,
    fs: float = 50.0,
    out_fs: float = 10.0,
    hp: float = 0.05,
    lp: float = 3.5,
) -> Dict[str, np.ndarray]:
    """Full pipeline: rotate, band-limit, integrate, decimate."""
    if fs <= 0:
        raise ValueError("Sampling rate must be positive for reconstruction")
    if len(t_ms) == 0:
        raise ValueError("No samples available for reconstruction")
    ax_w, ay_w, az_w = rotate_series_to_world(lx, ly, lz, qi, qj, qk, qr)
    ax_w = bandlimit(ax_w, fs, f_hp=hp, f_lp=lp)
    ay_w = bandlimit(ay_w, fs, f_hp=hp, f_lp=lp)
    az_w = bandlimit(az_w, fs, f_hp=hp, f_lp=lp)
    disp_x = integrate_twice(ax_w, fs)
    disp_y = integrate_twice(ay_w, fs)
    disp_z = integrate_twice(az_w, fs)
    dx, _ = decimate_to_rate(disp_x, fs, out_fs=out_fs)
    dy, _ = decimate_to_rate(disp_y, fs, out_fs=out_fs)
    dz, _ = decimate_to_rate(disp_z, fs, out_fs=out_fs)
    t_s = t_ms * 0.001
    factor = int(round(fs / out_fs))
    samples_out = dx.size
    t10 = t_s[: samples_out * factor : factor]
    return {
        "t10": t10,
        "x10": dx,
        "y10": dy,
        "z10": dz,
        "ax_band": ax_w,
        "ay_band": ay_w,
        "az_band": az_w,
        "fs_in": fs,
        "fs_out": out_fs,
    }


def wave_metrics_from_az(az_w: np.ndarray, fs: float) -> Tuple[float | None, float | None]:
    """Compute Hs and Tp via elevation spectrum from heave acceleration."""
    try:
        from scipy.signal import welch  # type: ignore
    except Exception:
        return None, None
    if fs <= 0:
        raise ValueError("Sampling rate must be positive for wave metrics")
    if az_w.size < 16:
        raise ValueError("Need at least 16 samples for wave metrics")
    nperseg = int(2 * fs * 64)
    f, pzz = welch(az_w, fs=fs, nperseg=min(nperseg, az_w.size))
    if f.size <= 1:
        return None, None
    f = f[1:]
    pzz = pzz[1:]
    omega = 2.0 * math.pi * f
    with np.errstate(divide="ignore", invalid="ignore"):
        s_eta = pzz / (omega ** 4)
    mask = np.isfinite(s_eta)
    if not np.any(mask):
        return None, None
    s_eta = s_eta[mask]
    f = f[mask]
    m0 = np.trapz(s_eta, f)
    hs = 4.0 * math.sqrt(max(m0, 0.0))
    tp_val = 1.0 / f[np.argmax(s_eta)] if f.size else None
    if tp_val is not None and not math.isfinite(tp_val):
        tp_val = None
    return float(hs), (float(tp_val) if tp_val is not None else None)
def _sliding_mean(values: np.ndarray, window_samples: int) -> np.ndarray:
    if window_samples <= 1:
        return np.convolve(values, np.array([1.0]), mode="same")
    kernel = np.ones(window_samples, dtype=np.float64) / float(window_samples)
    return np.convolve(values, kernel, mode="same")


def reconstruct_motion_freq(
    t_ms: np.ndarray,
    lx: np.ndarray,
    ly: np.ndarray,
    lz: np.ndarray,
    qi: np.ndarray,
    qj: np.ndarray,
    qk: np.ndarray,
    qr: np.ndarray,
    fs: float = 50.0,
    out_fs: float = 10.0,
    hp: float = 0.01,
    lp: float = 1.5,
    detrend_window: float = 600.0,
) -> Dict[str, np.ndarray]:
    """
    Frequency-domain reconstruction with explicit band limits.
    """
    if fs <= 0:
        raise ValueError("Sampling rate must be positive for reconstruction")
    if len(t_ms) == 0:
        raise ValueError("No samples available for reconstruction")
    ax_w, ay_w, az_w = rotate_series_to_world(lx, ly, lz, qi, qj, qk, qr)
    ax_w = _freq_integrate_to_disp(ax_w, fs, hp, lp, detrend_window)
    ay_w = _freq_integrate_to_disp(ay_w, fs, hp, lp, detrend_window)
    az_w = _freq_integrate_to_disp(az_w, fs, hp, lp, detrend_window)
    dx, _ = decimate_to_rate(ax_w, fs, out_fs=out_fs)
    dy, _ = decimate_to_rate(ay_w, fs, out_fs=out_fs)
    dz, _ = decimate_to_rate(az_w, fs, out_fs=out_fs)
    t_s = t_ms * 0.001
    factor = int(round(fs / out_fs))
    samples_out = dx.size
    t10 = t_s[: samples_out * factor : factor]
    return {
        "t10": t10,
        "x10": dx,
        "y10": dy,
        "z10": dz,
        "fs_in": fs,
        "fs_out": out_fs,
    }


def _freq_integrate_to_disp(
    acc: np.ndarray,
    fs: float,
    hp: float,
    lp: float,
    detrend_window: float,
) -> np.ndarray:
    n = acc.size
    if n == 0:
        return np.array([], dtype=np.float64)
    acc = acc - np.mean(acc)
    fft = np.fft.rfft(acc)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    disp_fft = np.zeros_like(fft)
    mask = np.ones_like(freqs, dtype=bool)
    if hp > 0:
        mask &= freqs >= hp
    if lp > 0:
        mask &= freqs <= lp
    mask &= freqs > 0
    omega2 = (2.0 * np.pi * freqs[mask]) ** 2
    disp_fft[mask] = -fft[mask] / omega2
    disp = np.fft.irfft(disp_fft, n=n)
    if detrend_window > 0:
        win = max(1, int(round(detrend_window * fs)))
        trend = _sliding_mean(disp, win)
        disp = disp - trend
    return disp
