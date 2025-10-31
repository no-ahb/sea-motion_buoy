# Ocean Motion Sensing Buoy

A DIY ocean-surface motion sensing buoy for capturing fine-grained wave-induced motion (multi-axis rotations/accelerations) at high sample rates (~10–100 Hz) for extended deployments in coastal waters.

## Project Overview

This project aims to build a minimalist offline logger that captures IMU data (BNO085) on a Raspberry Pi Pico 2, storing motion data to an SD card for periods of 3–4 weeks. The design balances cost, robustness, and data quality for coastal wave monitoring applications.

**Key Features:**
- High-frequency motion capture (100 Hz sampling rate)
- Extended battery operation (targeting 3–4 week deployments)
- Offline logging with MicroSD card storage
- Minimalist, cost-effective design
- Data format: binary with 8-byte header + 32-byte records (timestamp, linear acceleration, quaternion)

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

## Data Format

**Binary File Structure:**
- **Header (8 bytes):** `"BNO1"` magic (4), version (1), sample rate Hz (2), reserved (1)
- **Records (32 bytes each):** `<Ifffffff` = timestamp_ms (4), lx (4), ly (4), lz (4), qi (4), qj (4), qk (4), qr (4)
  - `lx, ly, lz`: Linear acceleration (gravity/bias removed), m/s²
  - `qi, qj, qk, qr`: Quaternion rotation vector

**Naming Conventions:**
- Recordings: `bno_XXX.bin` (on device) or `YYYY-MM-DD_HHMM_bno_XXX.bin` (archived)
- Test runs: Organized in dated folders under `recordings/test/test_runs/`
- Production runs: `recordings/production/YYYY-MM-DD_session_XXX/`

## Quick Start

### Decoding a Recording

```bash
python scripts/decoder/sea-movement_decoder.py recordings/test/test_runs/001_on\ desk/bno_009.bin
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
- 2000mAh 3.7V LiPo Battery
- ProtoMate for Pico
- Flotation hardware (TBD)

## Development Workflow

1. **Recording:** Upload recorder script to Pico, capture data to SD card
2. **Transfer:** Copy `.bin` files to `recordings/` organized by date/session
3. **Decode:** Run decoder script to verify and convert data
4. **Archive:** Organize successful recordings, update TODO.txt with notes

## Status

**Current Phase:** Testing & iteration
- ✅ Basic recorder/decoder working
- 🔄 Testing data format validation
- 📋 Planning for 10-hour production sessions

## License

[Add license here if needed]

