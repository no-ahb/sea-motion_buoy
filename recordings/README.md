# Recordings

Working directory for everything captured off the buoy. This now holds the validated 7 h overnight run (`test/006_1107 err  overnight/bno_008`) plus all intermediate experiments.

## Organization

### `test/`
Scratch pads for bring-up, bench experiments, and validation nights. Each folder documents a discrete attempt (e.g. `001_on desk`, `005_predeployTest`, `006_1107 err  overnight`). Keep all raw artifacts—`.bin`, `_meta.txt`, `_qa.csv`, `.err`, plots, reconstructed CSVs—in the same folder so post-mortems have everything.

### `production/`
Reserved for real deployments once hardware leaves the lab. Use `YYYY-MM-DD_session_##/` so data, notes, and photos stay grouped. Goal is ≥8 h per run at 50 Hz.

### `decoded/`
Optional parking lot for shared derivatives (downsampled CSVs, PSD exports) when they don’t belong inside a specific test folder. For reproducibility we currently keep most decoder outputs beside each run (see `006_1107 err  overnight`).

## Naming Conventions

**On SD card:**
- Recorder auto-increments `bno_###.bin` plus `_meta.txt`, `_qa.csv`, and, on errors, `.err`.

**After transfer:**
- Drop the entire folder from the card into `recordings/test/<session>/`.
- Append context to the folder name (`006_1107 err  overnight`) to encode test plan/date.
- For production, optionally add a timestamp prefix when archiving long-term backups.

## File Format

See main README.md for binary format specification.

## Data Management
1. **Record** using `sea-movement_recorder.py` on Pico (button to start/stop).
2. **Transfer** the entire SD folder; never cherry-pick files.
3. **Decode** via `scripts/decoder/sea-movement_decoder.py` (CSV, plots, PSD, reconstruction).
4. **Review** `_meta.txt` + `.err` and log takeaways in `TODO.txt` or session notes.
5. **Backup** to external storage—git history should track metadata/plots but not multi-GB binaries.
