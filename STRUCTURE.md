# Repository Structure

This document explains the repository organization and naming conventions.

## Directory Layout

```
.
├── README.md                    # Main project documentation
├── TODO.txt                     # Active task list (updated by agents)
├── CHANGELOG.md                 # Script version history
├── MATERIALS.md                 # Bill of materials
├── STRUCTURE.md                 # This file
├── GIT_SETUP.md                 # Git/GitHub setup instructions
│
├── scripts/                     # All code files
│   ├── recorder/                # MicroPython scripts for Pico
│   │   ├── sea-movement-recorder_script.py
│   │   ├── bno085_test.py
│   │   └── README.md
│   ├── decoder/                 # Python 3 decoder scripts
│   │   ├── sea-movement_decoder.py
│   │   └── README.md
│   └── library/                 # Third-party libraries
│       └── BOSCH-BNO085-I2C-micropython-library/
│
├── recordings/                  # All recorded data
│   ├── test/                    # Test recordings
│   │   └── test_runs/           # Organized by test session
│   │       └── 001_on desk/     # Example: Initial desk testing
│   ├── production/              # Real deployment recordings
│   │   └── YYYY-MM-DD_session_XXX/  # Date-based sessions
│   └── decoded/                 # Decoded output (CSV, JSON, etc.)
│
├── docs/                        # Documentation
│   ├── research/                # Research papers and references
│   └── datasheets/              # Component datasheets
│
└── archive/                     # Old/retired script versions
```

## Naming Conventions

### Recording Files

**On Device (SD Card):**
- Format: `bno_XXX.bin`
- Sequential numbering (000, 001, 002, ...)
- Maximum 3 digits

**After Transfer (Archived):**
- Test runs: Keep original name, organize in dated folders
- Production runs: Optionally rename for clarity:
  - Format: `YYYY-MM-DD_HHMM_bno_XXX.bin`
  - Example: `2025-01-15_1430_bno_001.bin`

### Script Versions

Scripts are versioned via CHANGELOG.md. When making significant changes:

1. Document in CHANGELOG.md
2. Consider creating archive copy in `archive/` directory
3. Update version/date in script header comments (optional)

### Test Sessions

Test session folders follow pattern:
- `001_XXXX` - Sequential test number + brief description
- Examples:
  - `001_on desk`
  - `002_shake test`
  - `003_outdoor_validation`

### Production Sessions

Production session folders follow pattern:
- `YYYY-MM-DD_session_XXX`
- Examples:
  - `2025-01-15_session_001`
  - `2025-01-15_session_002` (same day, different session)
  - `2025-01-16_session_001` (next day)

## Workflow

### Recording Workflow

1. **Prepare:** Upload recorder script to Pico
2. **Record:** Capture data to SD card (files: `bno_000.bin`, `bno_001.bin`, ...)
3. **Transfer:** Copy files from SD card to appropriate `recordings/` directory
4. **Verify:** Run decoder script to validate data
5. **Organize:** Move to appropriate folder, update TODO.txt with notes
6. **Commit:** Git commit with descriptive message

### Decoding Workflow

1. **Run Decoder:**
   ```bash
   python scripts/decoder/sea-movement_decoder.py recordings/test/test_runs/001_on\ desk/bno_009.bin
   ```
2. **Verify Output:** Check sample count, header info, sample values
3. **Export (Future):** Convert to CSV/JSON if needed
4. **Document:** Update TODO.txt with decoding notes/issues

### Agent Collaboration

When multiple agents work on the repository:

1. **Read TODO.txt** before starting work
2. **Update TODO.txt** with your active tasks:
   ```
   ## Active Tasks
   - [in progress] Agent X: Working on feature Y
   - [pending] Agent Z: Waiting for feature Y
   ```
3. **Commit frequently** with clear messages
4. **Update CHANGELOG.md** for script version changes
5. **Add notes** to TODO.txt if encountering issues

## Tips

- **Keep recordings organized:** Use consistent folder structure
- **Document in TODO.txt:** Agents should note changes/issues
- **Commit regularly:** Small, focused commits are easier to track
- **Tag releases:** Consider git tags for major script versions
- **Backup large files:** Production recordings may need external backup

