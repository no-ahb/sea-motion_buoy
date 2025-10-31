# Recordings

Directory structure for storing recorded motion data files.

## Organization

### `test/`
Test recordings and validation data. Organized by test session:
- `test_runs/001_on desk/` - Initial desk testing
- `test_runs/002_XXXX/` - Future test sessions

### `production/`
Real deployment recordings from ocean sessions (~10-hour logging sessions). Organized by date:
- `2025-XX-XX_session_001/` - ~10-hour production sessions
- `2025-XX-XX_session_002/` - etc.

Each production session targets approximately 10 hours of continuous recording at 200 Hz, requiring ~2.9 GB storage per session (raw binary).

### `decoded/`
Decoded/converted output files (CSV, JSON, etc.)

## Naming Conventions

**On Device (SD Card):**
- Format: `bno_XXX.bin` (sequential numbering)

**After Transfer (Archived):**
- Test: Keep original name, organize in dated folders
- Production: Optionally rename to `YYYY-MM-DD_HHMM_bno_XXX.bin` for clarity

## File Format

See main README.md for binary format specification.

## Data Management

1. **Record:** Capture data on Pico SD card
2. **Transfer:** Copy `.bin` files to appropriate directory here
3. **Decode:** Run decoder script to verify data integrity
4. **Archive:** Mark successful recordings, update TODO.txt with notes
5. **Backup:** Consider Git LFS or external storage for large production files

