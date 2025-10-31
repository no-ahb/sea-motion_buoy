# Decoder Scripts

Python 3 scripts for decoding binary recording files from the motion sensor buoy.

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

## Future Enhancements

- Export to CSV/JSON
- Data validation (quaternion normalization, timestamp continuity)
- Statistics summary (mean, std, range)
- Visualization plots

