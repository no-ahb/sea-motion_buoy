# Decoder Scripts

Python 3 scripts for decoding binary recording files from the ocean motion logger and preparing data for motor control playback.

## Usage

```bash
# From project root (basic summary)
python scripts/decoder/sea-movement_decoder.py recordings/test/test_runs/001_on\ desk/bno_009.bin

# Decode + downsample to 10 Hz + trimmed mean + PSD plot
python scripts/decoder/sea-movement_decoder.py \
  recordings/test/test_runs/001_on\ desk/bno_009.bin \
  --downsample 10 --robust trimmed --highpass 0.05 \
  --csv out_10hz.csv --plot out_timeseries.png \
  --psd out_psd.csv --psd-plot out_psd.png
```

### Key Options

- `--downsample <Hz>` – aggregate samples to a lower rate (median/trimmed/mean).
- `--highpass <Hz>` – first-order high-pass on linear acceleration.
- `--csv / --plot` – export CSV and time-series plot.
- `--psd / --psd-plot` – write acceleration PSD (requires `numpy`, optionally `matplotlib`).

## Output

The decoder prints:
- Header summary (magic, version, sample rate, device ID, bias, CRC if present)
- Total number of records read
- First `--head` samples `(t_ms, lx, ly, lz, qi, qj, qk, qr)`

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
- [x] Export to CSV
- [x] Metadata extraction (rate, sample count, bias, CRC)
- [x] Optional plotting for QA (time series, PSD)
- [ ] Add statistics summary (mean, std, quaternion norm drift)
- [ ] Timestamp continuity + CRC verification check

### Processing
- Compute **Up vector** from quaternion → project linear accel
- High-pass filtering (0.03–0.07 Hz) for drift removal
- Double integration → displacement reconstruction
- Downsampling and normalization for motor control
- Optional plotting for QA (acceleration, quaternion norm)
