import argparse
import csv
import os
import struct
import sys
from typing import Iterable, List, Sequence, Tuple

# Header: magic(4) ver(1) rate(2) reserved(1) = 8 bytes
HEADER_FORMAT = "<IBHB"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
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
    return parser.parse_args(argv)


def decode_file(path: str) -> Tuple[Tuple[int, int, int, int], List[Tuple]]:
    with open(path, "rb") as f:
        data = f.read()

    if len(data) < HEADER_SIZE:
        raise ValueError(f"File too short for header (got {len(data)} bytes)")

    magic, ver, rate, reserved = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
    if magic != 0x424E4F31:
        raise ValueError(f"Unexpected magic 0x{magic:08X} (expected 0x424E4F31 for 'BNO1')")

    records: List[Tuple] = []
    for offset in range(HEADER_SIZE, len(data), RECORD_SIZE):
        chunk = data[offset : offset + RECORD_SIZE]
        if len(chunk) != RECORD_SIZE:
            # Ignore trailing partial chunk; log clue for future debugging.
            print(
                f"Warning: truncated record at byte offset {offset} "
                f"(expected {RECORD_SIZE} bytes, got {len(chunk)})",
                file=sys.stderr,
            )
            break
        records.append(struct.unpack(RECORD_FORMAT, chunk))

    header = (magic, ver, rate, reserved)
    return header, records


def show_summary(header: Tuple[int, int, int, int], records: List[Tuple], head: int) -> None:
    magic, ver, rate, reserved = header
    print(f"File header: magic=0x{magic:08X} ('BNO1'), version={ver}, rate={rate} Hz, reserved={reserved}")
    print(f"Read {len(records)} samples.")
    if not records:
        return
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

    if args.csv:
        write_csv(args.csv, records)

    if args.plot:
        try:
            write_plots(args.plot, records)
        except Exception as exc:
            print(f"Plotting failed: {exc}", file=sys.stderr)
            return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
