# Decoder Scripts

Python 3 tooling for turning recorder `.bin` files into human-readable CSV, QA plots, PSDs and 10 Hz reconstructed displacement streams. 

## Usage

```bash
# 1) Quick integrity check + QA plots
python3 scripts/decoder/sea-movement_decoder.py \
  "recordings/test/006_1107 err  overnight/bno_008.bin" \
  --plot bno_008_plot.png --psd bno_008_psd.csv --psd-plot bno_008_psd.png

# 2) Full export (CSV + reconstruction + wave metrics)
python3 scripts/decoder/sea-movement_decoder.py \
  recordings/test/006_1107\ err\ \ overnight/bno_008.bin \
  --csv bno_008_decoded.csv --reconstruct-csv bno_008_motion_10hz.csv \
  --wave-metrics bno_008_waves.csv --downsample 10 --robust trimmed --highpass 0.05

# 3) Motor-ready feed (crop + normalize + fade)
RUN_DIR="recordings/test/007_30min sea test"
python3 scripts/decoder/sea-movement_decoder.py \
  "$RUN_DIR/bno_000.bin" \
  --reconstruct-csv "$RUN_DIR/bno_000_motion_10hz.csv"

python3 scripts/decoder/motor_feed.py \
  "$RUN_DIR/bno_000_motion_10hz.csv" \
  --output-csv "$RUN_DIR/bno_000_motor_norm_fade10s.csv" \
  --output-plot "$RUN_DIR/bno_000_motor_norm_fade10s.png"
```

### Key Options

- `--csv` – dump decoded records (after optional downsampling/high-pass)
- `--plot` – simple time-series plot (linear accel + quaternions, needs Matplotlib)
- `--psd`, `--psd-plot` – compute/export PSD (requires NumPy; plot also needs Matplotlib)
- `--downsample <Hz>` – aggregate to lower rate (uses trimmed mean by default)
- `--highpass <Hz>` – simple 1st-order filter before CSV export (jitter cleanup)
- `--reconstruct-csv` – run the NumPy/SciPy pipeline from `reconstruct.py` to produce 10 Hz surge/sway/heave
- `--wave-metrics` – use Welch spectrum on heave to estimate Hs/Tp (SciPy optional)
- `--verify-crc` – recompute CRC32 over the binary payload
- `scripts/decoder/motor_feed.py` – helper to crop out pier transients, normalize surge/sway/heave jointly to 0–1, and apply fade-in/out envelopes (10 s default) before driving motors. Key flags:
  - `--percentile`, `--deploy-window`, `--retrieve-window`, `--margin` tune automatic cropping.
  - `--crop-window-sec`/`--crop-std-threshold` detect the longest in-water interval via rolling std, with `--crop-prepad-sec`/`--crop-postpad-sec` to keep some context.
  - `--fade-seconds` controls the soft-start length.
  - `--hampel-k/--hampel-t` remove narrow mechanical spikes via Hampel filtering (defaults 9 samples / 3σ).
  - `--smooth-samples` applies a small moving average (default 11 samples) after Hampel to soften edges.
  - `--max-step` enforces a slew-rate limit per 10 Hz sample (default 0.02 normalized units).
  - `--global-min`, `--global-max` let you lock consistent normalization across multiple days.

## Output

Every invocation prints:
- Header summary (magic, version, flags, rate, device ID, bias, declared total samples, CRC)
- Timing stats (dt mean/std/min/p95/p99/max, late >20/50 ms, missed slots)
- First `--head` samples (timestamp + linear accel + quaternion)
- Paths to any emitted CSV/plots/PSD/reconstruction files

## Processing Workflow

1. Decode `.bin` into raw samples (linear accel + quaternion)
2. Optional filtering/downsampling for QA
3. Rotation to world frame → heave/surge/sway acceleration
4. Band-limit + double integration to displacement (NumPy) and decimate to 10 Hz
5. Compute PSD or wave statistics as needed
6. Export CSVs / plots for playback firmware or analysis

## Future Enhancements

- Automate per-run summary export (means/std/quaternion drift) for README snapshots
- Integrate Matplotlib displacement plots directly into the decoder (currently scripted separately)
- Add optional BLE/Web serial streaming for quick previews
