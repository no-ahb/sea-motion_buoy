import argparse
import csv
import math
import os
import struct
import sys
import binascii
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

HEADER_V1_MAGIC = 0x424E4F31  # 'BNO1'
HEADER_V1_FMT = "<IBHB"
HEADER_V1_SIZE = struct.calcsize(HEADER_V1_FMT)

HEADER_V2_MAGIC = b"BNO2"
HEADER_V2_FMT = "<4sBBHHIIIfffII"
HEADER_V2_SIZE = struct.calcsize(HEADER_V2_FMT)
# Record format: t_ms(4) lx(4) ly(4) lz(4) qi(4) qj(4) qk(4) qr(4) = 32 bytes
RECORD_FORMAT = "<Ifffffff"
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)
CSV_HEADER = ["t_ms", "lx", "ly", "lz", "qi", "qj", "qk", "qr"]


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Decode sea-movement recorder binary blobs."
    )
    parser.add_argument(
        "filename",
        help="Path to .bin file (e.g. recordings/test/.../bno_013.bin)",
    )
    parser.add_argument(
        "--csv",
        metavar="OUT.csv",
        help="Write decoded samples to a CSV file for external plotting.",
    )
    parser.add_argument(
        "--head",
        type=int,
        default=5,
        help="Print this many samples to stdout summary (default: 5).",
    )
    parser.add_argument(
        "--plot",
        metavar="OUT.png",
        help="Generate basic time-series plots (requires matplotlib).",
    )
    parser.add_argument(
        "--downsample",
        type=int,
        default=0,
        help="Downsample to this rate in Hz (e.g. 10). 0=disabled.",
    )
    parser.add_argument(
        "--robust",
        choices=["median", "trimmed", "mean"],
        default="trimmed",
        help="Robust aggregation for downsampling (default: trimmed).",
    )
    parser.add_argument(
        "--highpass",
        type=float,
        default=0.0,
        help="Apply first-order high-pass filter at this cutoff frequency (Hz). 0 = disabled.",
    )
    parser.add_argument(
        "--recon-highpass",
        type=float,
        default=0.05,
        help="High-pass cutoff (Hz) for reconstruction band-limit (default: 0.05).",
    )
    parser.add_argument(
        "--recon-lowpass",
        type=float,
        default=3.5,
        help="Low-pass cutoff (Hz) for reconstruction band-limit (default: 3.5).",
    )
    parser.add_argument(
        "--psd",
        metavar="OUT.csv",
        help="Write acceleration PSD to CSV (requires numpy).",
    )
    parser.add_argument(
        "--psd-plot",
        metavar="OUT.png",
        help="Plot the acceleration PSD (requires numpy and matplotlib).",
    )
    parser.add_argument(
        "--reconstruct-csv",
        metavar="OUT.csv",
        help="Write 10 Hz displacement stream (timestamp_s,surge_m,sway_m,heave_m). Requires numpy.",
    )
    parser.add_argument(
        "--wave-metrics",
        metavar="OUT.csv",
        nargs="?",
        const="",
        help="Compute Hs/Tp from heave acceleration; optionally write results to CSV. Requires numpy and SciPy.",
    )
    parser.add_argument(
        "--reconstruct-freq-csv",
        metavar="OUT.csv",
        help="Write frequency-domain reconstructed displacement to CSV.",
    )
    parser.add_argument(
        "--recon-freq-highpass",
        type=float,
        default=0.01,
        help="Frequency-domain recon high-pass cutoff (Hz, default 0.01).",
    )
    parser.add_argument(
        "--recon-freq-lowpass",
        type=float,
        default=1.5,
        help="Frequency-domain recon low-pass cutoff (Hz, default 1.5).",
    )
    parser.add_argument(
        "--recon-freq-detrend",
        type=float,
        default=600.0,
        help="Frequency-domain recon sliding detrend window (seconds, default 600).",
    )
    parser.add_argument(
        "--verify-crc",
        action="store_true",
        help="Recompute CRC32 over records and compare with header (if present).",
    )
    return parser.parse_args(argv)


def decode_file(path: str) -> Tuple[Dict[str, object], List[Tuple]]:
    with open(path, "rb") as f:
        data = f.read()

    if len(data) < min(HEADER_V1_SIZE, HEADER_V2_SIZE):
        raise ValueError(f"File too short for header (got {len(data)} bytes)")

    records: List[Tuple] = []
    header: Dict[str, object]

    if data[:4] == HEADER_V2_MAGIC:
        if len(data) < HEADER_V2_SIZE:
            raise ValueError("File truncated: incomplete v2 header")
        unpacked = struct.unpack(HEADER_V2_FMT, data[:HEADER_V2_SIZE])
        header_size = unpacked[2]
        if header_size > len(data):
            raise ValueError("Header declares size beyond file length")
        header = {
            "magic": unpacked[0].decode("ascii", errors="ignore"),
            "version": unpacked[1],
            "header_size": header_size,
            "rate_hz": unpacked[3],
            "flags": unpacked[4],
            "start_epoch": unpacked[5],
            "start_ticks": unpacked[6],
            "device_id": unpacked[7],
            "bias": (unpacked[8], unpacked[9], unpacked[10]),
            "total_samples": unpacked[11],
            "crc32": unpacked[12],
            "reserved": None,
        }
        offset = header_size
    else:
        magic_val = struct.unpack("<I", data[:4])[0]
        if magic_val != HEADER_V1_MAGIC:
            raise ValueError(
                f"Unexpected magic 0x{magic_val:08X} (expected 0x{HEADER_V1_MAGIC:08X} for 'BNO1')"
            )
        magic, ver, rate, reserved = struct.unpack(HEADER_V1_FMT, data[:HEADER_V1_SIZE])
        header = {
            "magic": "BNO1",
            "version": ver,
            "header_size": HEADER_V1_SIZE,
            "rate_hz": rate,
            "flags": 0,
            "start_epoch": None,
            "start_ticks": None,
            "device_id": None,
            "bias": (None, None, None),
            "total_samples": None,
            "crc32": None,
            "reserved": reserved,
        }
        offset = HEADER_V1_SIZE

    crc_calc = 0
    for cursor in range(offset, len(data), RECORD_SIZE):
        chunk = data[cursor : cursor + RECORD_SIZE]
        if len(chunk) != RECORD_SIZE:
            print(
                f"Warning: truncated record at byte offset {cursor} "
                f"(expected {RECORD_SIZE} bytes, got {len(chunk)})",
                file=sys.stderr,
            )
            break
        crc_calc = binascii.crc32(chunk, crc_calc) & 0xFFFFFFFF
        records.append(struct.unpack(RECORD_FORMAT, chunk))

    header["crc32_calc"] = crc_calc
    return header, records


def show_summary(header: Dict[str, object], records: List[Tuple], head: int) -> None:
    print(
        f"File header: magic={header['magic']} version={header['version']} rate={header['rate_hz']} Hz"
    )
    if header.get("device_id") is not None:
        bias = header.get("bias", (None, None, None))
        if bias and all(b is not None for b in bias):
            bias_str = "(%.6f, %.6f, %.6f)" % bias
        else:
            bias_str = str(bias)
        print(
            "  device_id=0x%08X flags=0x%04X start_epoch=%s bias=%s"
            % (
                header["device_id"],
                header.get("flags", 0),
                str(header.get("start_epoch")),
                bias_str,
            )
        )
        print(
            "  total_samples(declared)=%s crc32=0x%08X"
            % (
                str(header.get("total_samples")),
                header.get("crc32", 0) or 0,
            )
        )
    else:
        print(f"  legacy header reserved={header.get('reserved')}")

    print(f"Read {len(records)} samples.")
    if not records:
        return
    # Jitter summary (based on t_ms deltas)
    dts = [records[i][0] - records[i-1][0] for i in range(1, len(records))]
    if dts:
        import statistics as stats
        dt_ms = 1000.0 / float(header["rate_hz"]) if header.get("rate_hz") else None
        mean_dt = stats.mean(dts)
        std_dt = stats.pstdev(dts) if len(dts) > 1 else 0.0
        p95 = sorted(dts)[int(0.95 * (len(dts)-1))]
        p99 = sorted(dts)[int(0.99 * (len(dts)-1))]
        dt_min = min(dts); dt_max = max(dts)
        late20 = sum(1 for x in dts if x > 20)
        late50 = sum(1 for x in dts if x > 50)
        missed = 0
        if dt_ms is not None and dt_ms > 0:
            import math
            missed = sum(max(0, int(x // dt_ms) - 1) for x in dts)
        derived_rate = ((len(records) - 1) * 1000.0) / (records[-1][0] - records[0][0]) if len(records) > 1 else 0.0
        print(
            (
                "Timing: dt mean=%.2f ms, std=%.2f, min=%.0f, p95=%.0f, p99=%.0f, max=%.0f; "
                "late20=%d, late50=%d, missed_slots=%d; derived_rate=%.6f Hz"
            )
            % (mean_dt, std_dt, dt_min, p95, p99, dt_max, late20, late50, missed, derived_rate)
        )
    # CRC summary
    if header.get("crc32") is not None:
        print("Header CRC32: 0x%08X, Calculated: 0x%08X" % (header.get("crc32", 0) or 0, header.get("crc32_calc", 0) or 0))
    print("First samples (t_ms, lx, ly, lz, qi, qj, qk, qr):")
    for row in records[: max(head, 0)]:
        print(row)


def write_csv(path: str, rows: Iterable[Tuple]) -> None:
    with open(path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(CSV_HEADER)
        writer.writerows(rows)
    print(f"Wrote CSV: {path}")


def write_plots(path: str, records: List[Tuple]) -> None:
    try:
        import matplotlib  # type: ignore
        matplotlib.use("Agg")  # headless-friendly
        import matplotlib.pyplot as plt  # type: ignore
    except ImportError as exc:  # pragma: no cover - optional dependency
        raise RuntimeError(
            "matplotlib is required for plotting. Install it with 'pip install matplotlib'"
        ) from exc

    if not records:
        raise ValueError("No records to plot.")

    t = [row[0] * 0.001 for row in records]  # seconds
    lin = list(zip(*[row[1:4] for row in records]))
    quat = list(zip(*[row[4:] for row in records]))

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax_lin, ax_quat = axes

    for comp, label in zip(lin, ("lx", "ly", "lz")):
        ax_lin.plot(t, comp, label=label, linewidth=0.7)
    ax_lin.set_ylabel("Linear accel (m/s²)")
    ax_lin.legend(loc="upper right")
    ax_lin.grid(True, alpha=0.2)

    for comp, label in zip(quat, ("qi", "qj", "qk", "qr")):
        ax_quat.plot(t, comp, label=label, linewidth=0.7)
    ax_quat.set_ylabel("Quaternion")
    ax_quat.set_xlabel("Time (s)")
    ax_quat.legend(loc="upper right")
    ax_quat.grid(True, alpha=0.2)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Wrote plot: {path}")


def write_reconstruction_csv(path: str, t_s, x, y, z) -> None:
    with open(path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["timestamp_s", "surge_m", "sway_m", "heave_m"])
        for row in zip(t_s, x, y, z):
            writer.writerow([float(row[0]), float(row[1]), float(row[2]), float(row[3])])
    print(f"Wrote reconstruction CSV: {path}")


def write_wave_metrics_csv(path: str, hs: Optional[float], tp: Optional[float]) -> None:
    with open(path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Hs_m", "Tp_s"])
        writer.writerow(
            [
                "" if hs is None else f"{hs:.6f}",
                "" if tp is None else f"{tp:.6f}",
            ]
        )
    print(f"Wrote wave metrics CSV: {path}")


def _median(values: List[float]) -> float:
    vs = sorted(values)
    n = len(vs)
    return vs[n // 2] if n % 2 == 1 else 0.5 * (vs[n // 2 - 1] + vs[n // 2])


def _trimmed_mean(values: List[float]) -> float:
    if len(values) <= 2:
        return sum(values) / float(len(values))
    vs = sorted(values)[1:-1]
    return sum(vs) / float(len(vs))


def _avg_quaternions(quats: List[Tuple[float, float, float, float]]) -> Tuple[float, float, float, float]:
    # Practical small-window average: sign-align to running sum, sum, then normalize
    import math
    sx = sy = sz = sw = 0.0
    for (x, y, z, w) in quats:
        dot = x * sx + y * sy + z * sz + w * sw
        if dot < 0.0:
            x, y, z, w = -x, -y, -z, -w
        sx += x; sy += y; sz += z; sw += w
    nrm = math.sqrt(sx * sx + sy * sy + sz * sz + sw * sw) or 1.0
    return (sx / nrm, sy / nrm, sz / nrm, sw / nrm)


def downsample_records(records: List[Tuple], rate_in: int, rate_out: int, robust: str) -> List[Tuple]:
    if rate_out <= 0 or rate_out >= rate_in:
        return records
    if rate_in % rate_out != 0:
        raise ValueError(f"Downsample requires integer ratio; {rate_in}/{rate_out} is not integer")
    w = rate_in // rate_out
    out: List[Tuple] = []
    n = len(records)
    agg_fun = {
        "median": _median,
        "trimmed": _trimmed_mean,
        "mean": lambda xs: sum(xs) / float(len(xs)),
    }[robust]
    for i in range(0, n, w):
        bucket = records[i : i + w]
        if len(bucket) < w:
            break
        t_ms = bucket[-1][0]
        lxs = [r[1] for r in bucket]
        lys = [r[2] for r in bucket]
        lzs = [r[3] for r in bucket]
        lx = agg_fun(lxs)
        ly = agg_fun(lys)
        lz = agg_fun(lzs)
        quats = [(r[4], r[5], r[6], r[7]) for r in bucket]
        qi, qj, qk, qr = _avg_quaternions(quats)
        out.append((t_ms, lx, ly, lz, qi, qj, qk, qr))
    return out


def highpass_records(records: List[Tuple], rate: int, cutoff_hz: float) -> List[Tuple]:
    if cutoff_hz <= 0.0 or not records:
        return records
    if rate <= 0:
        raise ValueError("Sampling rate must be positive for high-pass filtering")
    dt = 1.0 / float(rate)
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    alpha = rc / (rc + dt)
    prev_x = [records[0][1], records[0][2], records[0][3]]
    prev_y = [0.0, 0.0, 0.0]
    filtered: List[Tuple] = []
    for row in records:
        lin = []
        for idx in range(3):
            x = row[1 + idx]
            y = alpha * (prev_y[idx] + x - prev_x[idx])
            prev_y[idx] = y
            prev_x[idx] = x
            lin.append(y)
        filtered.append((row[0], lin[0], lin[1], lin[2], row[4], row[5], row[6], row[7]))
    return filtered


def compute_psd(records: List[Tuple], rate: int):
    if rate <= 0:
        raise ValueError("Sampling rate must be positive for PSD computation")
    if not records:
        raise ValueError("No records to analyse")
    try:
        import numpy as np  # type: ignore
    except ImportError as exc:  # pragma: no cover - optional dependency
        raise RuntimeError("numpy is required for PSD computation") from exc

    arr = np.asarray([[r[1], r[2], r[3]] for r in records], dtype=np.float64)
    n = arr.shape[0]
    if n < 8:
        raise ValueError("Not enough samples for PSD (need >= 8)")

    window = np.hanning(n)
    scale = np.sum(window ** 2)
    if scale == 0.0:
        raise ValueError("Window power is zero; cannot compute PSD")

    freqs = np.fft.rfftfreq(n, d=1.0 / rate)
    psd = {}
    for idx, label in enumerate(("lx", "ly", "lz")):
        xw = arr[:, idx] * window
        fft = np.fft.rfft(xw)
        psd[label] = (np.abs(fft) ** 2) / (scale * rate)
    return freqs, psd


def write_psd_csv(path: str, freqs, psd) -> None:
    with open(path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["freq_hz", "psd_lx", "psd_ly", "psd_lz"])
        for idx in range(len(freqs)):
            writer.writerow([
                float(freqs[idx]),
                float(psd["lx"][idx]),
                float(psd["ly"][idx]),
                float(psd["lz"][idx]),
            ])
    print(f"Wrote PSD CSV: {path}")


def write_psd_plot(path: str, freqs, psd) -> None:
    try:
        import matplotlib  # type: ignore
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # type: ignore
    except ImportError as exc:  # pragma: no cover - optional dependency
        raise RuntimeError(
            "matplotlib is required for PSD plotting. Install it with 'pip install matplotlib'"
        ) from exc

    fig, ax = plt.subplots(figsize=(8, 4))
    for label in ("lx", "ly", "lz"):
        ax.plot(freqs, psd[label], label=label)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("PSD (power/Hz)")
    ax.set_yscale("log")
    ax.set_title("Acceleration PSD")
    ax.grid(True, which="both", alpha=0.2)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Wrote PSD plot: {path}")


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)

    if not os.path.exists(args.filename):
        print(f"Error: {args.filename} not found.", file=sys.stderr)
        return 1

    try:
        header, records = decode_file(args.filename)
    except Exception as exc:
        print(f"Decode failed: {exc}", file=sys.stderr)
        return 1

    show_summary(header, records, args.head)

    recon_requested = bool(args.reconstruct_csv) or bool(args.reconstruct_freq_csv) or args.wave_metrics is not None
    if recon_requested:
        if not records:
            print("No samples available for reconstruction; skipping.", file=sys.stderr)
        else:
            try:
                import numpy as np  # type: ignore
            except ImportError as exc:  # pragma: no cover - optional dependency
                print(
                    "Motion reconstruction requires numpy. Install it with 'pip install numpy'.",
                    file=sys.stderr,
                )
                return 1
            try:
                from reconstruct import reconstruct_motion, reconstruct_motion_freq, wave_metrics_from_az  # type: ignore
            except ImportError as exc:  # pragma: no cover - module packaging
                print(f"Reconstruction helpers not found: {exc}", file=sys.stderr)
                return 1
            fs_native = float(header.get("rate_hz") or 0.0)
            if fs_native <= 0.0:
                print("Header missing a valid sampling rate; cannot reconstruct motion.", file=sys.stderr)
                return 1
            arr = np.asarray(records, dtype=np.float64)
            t_ms = arr[:, 0]
            lx = arr[:, 1]
            ly = arr[:, 2]
            lz = arr[:, 3]
            qi = arr[:, 4]
            qj = arr[:, 5]
            qk = arr[:, 6]
            qr = arr[:, 7]
            try:
                motion = reconstruct_motion(
                    t_ms,
                    lx,
                    ly,
                    lz,
                    qi,
                    qj,
                    qk,
                    qr,
                    fs=fs_native,
                    out_fs=10.0,
                    hp=args.recon_highpass,
                    lp=args.recon_lowpass,
                )
            except Exception as exc:
                print(f"Motion reconstruction failed: {exc}", file=sys.stderr)
                return 1
            if args.reconstruct_csv:
                if motion["t10"].size == 0:
                    print("Motion reconstruction produced no samples to write.", file=sys.stderr)
                else:
                    write_reconstruction_csv(
                        args.reconstruct_csv, motion["t10"], motion["x10"], motion["y10"], motion["z10"]
                    )
            if args.wave_metrics is not None:
                try:
                    hs, tp = wave_metrics_from_az(motion["az_band"], motion["fs_in"])
                except Exception as exc:
                    print(f"Wave metric computation failed: {exc}", file=sys.stderr)
                else:
                    if hs is None and tp is None:
                        print(
                            "Wave metrics unavailable (requires scipy.signal.welch).",
                            file=sys.stderr,
                        )
                    else:
                        tp_display = "n/a" if tp is None else f"{tp:.3f}"
                        hs_display = "n/a" if hs is None else f"{hs:.3f}"
                        print(f"Wave metrics: Hs={hs_display} m, Tp={tp_display} s")
                        if args.wave_metrics:
                            write_wave_metrics_csv(args.wave_metrics, hs, tp)
            if args.reconstruct_freq_csv:
                try:
                    motion_freq = reconstruct_motion_freq(
                        t_ms,
                        lx,
                        ly,
                        lz,
                        qi,
                        qj,
                        qk,
                        qr,
                        fs=fs_native,
                        out_fs=10.0,
                        hp=args.recon_freq_highpass,
                        lp=args.recon_freq_lowpass,
                        detrend_window=args.recon_freq_detrend,
                    )
                except Exception as exc:
                    print(f"Frequency-domain reconstruction failed: {exc}", file=sys.stderr)
                    return 1
                if motion_freq["t10"].size == 0:
                    print("Frequency-domain reconstruction produced no samples to write.", file=sys.stderr)
                else:
                    write_reconstruction_csv(
                        args.reconstruct_freq_csv,
                        motion_freq["t10"],
                        motion_freq["x10"],
                        motion_freq["y10"],
                        motion_freq["z10"],
                    )

    effective_rate = header["rate_hz"]

    # Optional downsampling step
    if args.downsample and args.downsample > 0:
        try:
            ds_records = downsample_records(records, rate_in=effective_rate, rate_out=args.downsample, robust=args.robust)
        except Exception as exc:
            print(f"Downsample failed: {exc}", file=sys.stderr)
            return 1
        records = ds_records
        effective_rate = args.downsample

    if args.highpass and args.highpass > 0.0:
        try:
            records = highpass_records(records, rate=effective_rate, cutoff_hz=args.highpass)
        except Exception as exc:
            print(f"High-pass filter failed: {exc}", file=sys.stderr)
            return 1

    # Optional CRC verify (after decode)
    if args.verify_crc and header.get("crc32") is not None:
        calc = header.get("crc32_calc")
        exp = header.get("crc32")
        if calc != exp:
            print("CRC verify: FAIL (expected 0x%08X, got 0x%08X)" % (exp or 0, calc or 0), file=sys.stderr)
        else:
            print("CRC verify: OK (0x%08X)" % (calc or 0))

    psd_data = None
    if args.psd or args.psd_plot:
        try:
            psd_data = compute_psd(records, effective_rate)
        except Exception as exc:
            print(f"PSD computation failed: {exc}", file=sys.stderr)
            return 1

    if args.csv:
        write_csv(args.csv, records)

    if args.plot:
        try:
            write_plots(args.plot, records)
        except Exception as exc:
            print(f"Plotting failed: {exc}", file=sys.stderr)
            return 1

    if args.psd and psd_data:
        freqs, psd_axes = psd_data
        write_psd_csv(args.psd, freqs, psd_axes)

    if args.psd_plot and psd_data:
        freqs, psd_axes = psd_data
        try:
            write_psd_plot(args.psd_plot, freqs, psd_axes)
        except Exception as exc:
            print(f"PSD plotting failed: {exc}", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
