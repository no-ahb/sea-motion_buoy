# Decoder Scripts

Python 3 scripts for decoding binary recording files from the ocean motion logger and preparing data for motor control playback.

## Usage

```bash
# From project root
python scripts/decoder/sea-movement_decoder.py recordings/test/test_runs/001_on\ desk/bno_009.bin

# Or from decoder directory
cd scripts/decoder
python sea-movement_decoder.py ../../recordings/test/test_runs/001_on\ desk/bno_009.bin
```

## Output

The decoder prints:
- File header information (magic, version, sample rate)
- Total number of samples read
- First 5 samples in format: `(t_ms, lx, ly, lz, qi, qj, qk, qr)`

## Processing Workflow

For stepper motor playback, the recorded data follows this pipeline:

1. **Parse `.bin`** → timestamped samples (timestamp, linear accel, quaternion)
2. **Rotate to world frame** using quaternion → project linear accel → compute `a_z` (up vector)
3. **Filter** (low-pass ≈ 4–5 Hz; high-pass 0.03–0.07 Hz to remove drift)
4. **Integrate twice** → displacement (time-domain or frequency-domain)
5. **Downsample** (200 Hz → 10–25 Hz for motor control)
6. **Normalize 0–1** for stepper control signals
7. **Export CSV** or stream to motor controller

## Future Enhancements

### Immediate
- Export to CSV/NumPy formats
- Metadata extraction (recording duration, sample count, effective rate)
- Data validation (quaternion normalization, timestamp continuity)
- Statistics summary (mean, std, range)

### Processing
- Compute **Up vector** from quaternion → project linear accel
- High-pass filtering (0.03–0.07 Hz) for drift removal
- Double integration → displacement reconstruction
- Downsampling and normalization for motor control
- Optional plotting for QA (acceleration, quaternion norm)

