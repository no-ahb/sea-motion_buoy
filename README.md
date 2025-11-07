# Ocean Motion Logger & Replayer

This repository now contains the first field-proven stack (recorder + decoder + reconstruction tooling) for the Sea Strings buoy program. A Raspberry Pi Pico 2 with a BNO085 IMU logs 3-axis motion at 50 Hz to microSD; the Python decoder turns those binaries into CSV, PSD, and reconstructed displacement feeds for motor playback.

## Project Overview

This cycle focused on reliability: verified 7 h overnight sessions (e.g., `recordings/test/006_1107 err  overnight/bno_008`) and the full decoding pipeline including PSDs and motion reconstruction.

**Current capabilities (v0.1 “Yutyrannus”):**
- 50 Hz logging with aggressive SD retries (synchronous writer, FS checkpoints every 4 blocks)
- Bias + telemetry metadata in companion `_meta.txt`
- Periodic QA CSV snapshots (accel, gyro, quaternion norm, VSYS) buffered safely
- Decoder exports: CSV, PSD (data + plot), motion reconstruction (10 Hz surge/sway/heave) and optional wave metrics
- Proven 7 h log (bno_008) without errors; remaining SD issues traced to known bad media/cards

## Repository Structure

```
.
├── README.md                     # Project overview (this file)
├── TODO.txt / CHANGELOG.md       # Planning + history
├── scripts/
│   ├── recorder/sea-movement_recorder.py  # Pico firmware (ship this as main.py)
│   ├── decoder/sea-movement_decoder.py    # Python 3 CLI + plots
│   └── library/BOSCH-…/                   # Vendor BNO08X driver snapshot
├── recordings/
│   ├── test/006_1107 err  overnight/      # Active validation runs
│   ├── test/00X_*                         # Other benches / experiments
│   └── production/                        # Holds future deployment drops
├── docs/                                  # Datasheets & research
├── materials, archive, etc.
```

## Data Format

- **Header (v2 / 42 bytes):** magic `BNO2`, version, header size, target rate, flags, start epoch + ticks, device ID, bias (xyz), declared total samples, CRC32.
- **Records (32 bytes each):** `<Ifffffff` (ms timestamp, linear accel xyz, quaternion qi/qj/qk/qr)
- **Companion files:** `_meta.txt` (rates, jitter, SD telemetry), `_qa.csv` (every 250th sensor snapshot), optional `.err` on faults.
  - `lx, ly, lz`: Linear acceleration (gravity/bias removed), m/s²
  - `qi, qj, qk, qr`: Quaternion rotation vector

## Processing Workflow

The recorded data follows this processing pipeline to drive stepper motors:

1. **Parse `.bin`** with `sea-movement_decoder.py`
2. **Optional filters** (`--highpass`, downsample aggregator, PSD export)
3. **Reconstruction** (`--reconstruct-csv`) rotates into world frame, band-limits, double-integrates, decimates to 10 Hz
4. **Wave metrics** (`--wave-metrics`) derive Hs/Tp from heave (SciPy optional)
5. **Motor prep**: normalize 10 Hz surge/sway/heave for playback firmware

**Naming Conventions:**
- On-device: `bno_XXX.bin` + sidecars (`_meta.txt`, `_qa.csv`, optional `.err`)
- Test runs: keep the `bno_###` files in a descriptive folder (e.g., `recordings/test/006_1107 err  overnight/`)
- Production runs: `recordings/production/YYYY-MM-DD_session_###/`

## Quick Start

### Decoding a Recording

```bash
# Basic decode + QA plot + PSD
python3 scripts/decoder/sea-movement_decoder.py \
  "recordings/test/006_1107 err  overnight/bno_008.bin" \
  --plot bno_008_plot.png --psd bno_008_psd.csv --psd-plot bno_008_psd.png

# Full reconstruction to 10 Hz motion stream
python3 scripts/decoder/sea-movement_decoder.py \
  recordings/test/006_1107\ err\ \ overnight/bno_008.bin \
  --reconstruct-csv bno_008_motion_10hz.csv --wave-metrics bno_008_waves.csv
```

### Script Versions

See `CHANGELOG.md` for version history and `TODO.txt` for active development tasks.

## Hardware

See `MATERIALS.md` for the complete bill of materials.

**Core Components (current build):**
- Raspberry Pi Pico 2 + ProtoMate carrier
- BNO085 breakout wired to I2C(0) @ 400 kHz
- SD breakout on SPI(1) (mount @ 1 MHz → 12 MHz runtime)
- LiPo SHIM + 2000 mAh cell (≈8 h at 50 Hz; proven 7 h run)
- Waterproof hull + button/LED pass-throughs (see docs/)

## Development Workflow

1. **Recording:** Upload recorder script to Pico, capture data to SD card
2. **Transfer:** Copy `.bin` files to `recordings/` organized by date/session
3. **Decode:** Run decoder script to verify and convert data
4. **Archive:** Organize successful recordings, update TODO.txt with notes

## Status

**Current Phase:** First shippable logger/decoder bundle
- ✅ Recorder v0.1: synchronous writer, SD retries, QA buffering, INT pacing hook
- ✅ Decoder v0.2: CSV/plot/PSD/reconstruction pipeline + wave metrics hook
- ✅ Field validation: 7 h run `bno_008` (no errors)
- 🔄 Hardware polish: better SD cards, waterproof packaging
- 🔜 Motor playback integration using reconstructed CSVs
