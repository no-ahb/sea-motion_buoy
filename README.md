# Ocean Motion Logger & Replayer

A minimal open-hardware buoy system for capturing and reproducing 3-axis ocean-surface motion.

## Project Overview

This project implements a minimalist offline logger that captures IMU data (BNO085) on a Raspberry Pi Pico 2, storing motion data to microSD for **≈10-hour logging sessions**. The recorded data later drives three stepper motors reproducing heave, surge, and sway motions.

**Key Features:**
- High-frequency motion capture (200 Hz target sampling rate)
- ~10 hour battery runtime on 2000 mAh LiPo
- Offline logging with MicroSD card storage
- Minimalist, cost-effective design
- Data format: binary with 8-byte header + 32-byte records (timestamp, linear acceleration, quaternion)
- Records **linear acceleration** and **quaternion orientation** to microSD

## Repository Structure

```
.
├── README.md                 # This file
├── TODO.txt                  # Active task list (updated by agents)
├── CHANGELOG.md              # Version history of scripts
├── MATERIALS.md              # Bill of materials
├── scripts/                  # Code files
│   ├── recorder/             # Recorder scripts (MicroPython)
│   │   ├── sea-movement-recorder_script.py
│   │   └── bno085_test.py
│   ├── decoder/              # Decoder scripts (Python 3)
│   │   └── sea-movement_decoder.py
│   └── library/              # Third-party libraries
│       └── BOSCH-BNO085-I2C-micropython-library/
├── recordings/               # Recorded data files
│   ├── test/                 # Test recordings
│   │   └── test_runs/        # Organized by test session
│   ├── production/           # Real deployment recordings
│   │   └── 2025-XX-XX_XXX/   # Date-based folders
│   └── decoded/              # Decoded output files
├── docs/                     # Documentation
│   ├── research/             # Research papers and references
│   └── datasheets/           # Component datasheets
└── archive/                  # Old/retired script versions
```

## Firmware Features

- 200 Hz acquisition (linear accel + quaternion) - *target rate, currently 100 Hz*
- 20 s still-bias calibration on boot - *currently 4 s*
- Binary logging in 512-record blocks (~18 KB per flush)
- LED + button control with safe unmount
- Power-safe block flushing and optional CRC (planned)
- Double-buffered SD writing for stable 100–200 Hz sampling (planned)

## Data Format

**Binary File Structure:**
- **Header (8 bytes):** `"BNO1"` magic (4), version (1), sample rate Hz (2), reserved (1)
- **Records (32 bytes each):** `<Ifffffff` = timestamp_ms (4), lx (4), ly (4), lz (4), qi (4), qj (4), qk (4), qr (4)
  - `lx, ly, lz`: Linear acceleration (gravity/bias removed), m/s²
  - `qi, qj, qk, qr`: Quaternion rotation vector

## Processing Workflow

The recorded data follows this processing pipeline to drive stepper motors:

1. **Parse `.bin`** → timestamped samples
2. **Rotate to world frame** using quaternion
3. **Filter** (low-pass ≈ 4–5 Hz; high-pass 0.03–0.10 Hz)
4. **Integrate twice** → displacement
5. **Downsample** (200 → 10–25 Hz)
6. **Normalize 0–1** for stepper control
7. **Export CSV** or stream to motor controller

**Naming Conventions:**
- Recordings: `bno_XXX.bin` (on device) or `YYYY-MM-DD_HHMM_bno_XXX.bin` (archived)
- Test runs: Organized in dated folders under `recordings/test/test_runs/`
- Production runs: `recordings/production/YYYY-MM-DD_session_XXX/`

## Quick Start

### Decoding a Recording

```bash
# From project root
python scripts/decoder/sea-movement_decoder.py recordings/test/test_runs/001_on\ desk/bno_009.bin

# Or with relative path
cd scripts/decoder
python sea-movement_decoder.py ../../recordings/test/test_runs/001_on\ desk/bno_009.bin
```

### Git Workflow

```bash
# View current status
git status

# View active tasks
cat TODO.txt

# Commit changes
git add <files>
git commit -m "Description of changes"

# Push to GitHub (after setting up remote)
git push origin main
```

### Script Versions

See `CHANGELOG.md` for version history and `TODO.txt` for active development tasks.

## Hardware

See `MATERIALS.md` for the complete bill of materials.

**Core Components:**
- Raspberry Pi Pico 2 (with headers)
- BNO085 IMU (9-DOF Orientation Fusion Breakout)
- MicroSD card breakout + 32GB Class 10 A1 card
- LiPo SHIM for Pico
- 2000mAh 3.7V LiPo Battery (provides ~10 hour runtime)
- ProtoMate for Pico
- Flotation hardware (10–15 cm buoy hull, TBD)

## Development Workflow

1. **Recording:** Upload recorder script to Pico, capture data to SD card
2. **Transfer:** Copy `.bin` files to `recordings/` organized by date/session
3. **Decode:** Run decoder script to verify and convert data
4. **Archive:** Organize successful recordings, update TODO.txt with notes

## Status

**Current Phase:** Testing & iteration
- ✅ Basic recorder/decoder working (100 Hz, 4s bias calibration)
- 🔄 Implementing double-buffered writes for stable 200 Hz
- 🔄 Testing data format validation
- 📋 Preparing for 10-hour production sessions
- 📋 Post-processing workflow implementation (quaternion rotation, filtering, integration)

## License

[Add license here if needed]

