# Recorder Scripts

MicroPython scripts for the Raspberry Pi Pico 2 to capture 9-DoF IMU data for ocean motion logging.

## Files

- `sea-movement_recorder.py` - Production recorder (rename to `main.py` on the Pico)
- `functional_baselineScript.py` - Legacy baseline script, retained for reference
- `bno085_test.py` - Test/development script for BNO085 sensor

## Upload to Pico

1. Copy files to Pico (via Thonny or similar):
   - `functional_baselineScript.py` → Rename to `main.py` on Pico (for auto-run on battery)
   - `bno08x.py` library → Pico root directory:
     - From: `scripts/library/BOSCH-BNO085-I2C-micropython-library/lib/bno08x.py`
2. Ensure MicroSD card is mounted and accessible
3. The `main.py` file will automatically execute on boot when powered by battery

## Configuration

Edit these constants in `sea-movement_recorder.py` before uploading as `main.py`:
- `RATE_HZ` – Sampling rate (default: 50 Hz for clean, stable logging)
- `BIAS_SECS` – Still-bias duration (default: 30 s)
- `BIAS_RATE` – Calibration sample rate (default: 25 Hz)
- `BLOCK_RECS` – Records per block (default: 1024 → 32 KiB)
- `I2C_FREQ` – IMU bus frequency (default: 400 kHz)
- `SPI_BAUD` – SD SPI baudrate (default: 24 MHz after mount)
- `LED_PIN`, `BTN_PIN` – Hardware pin assignments

## Operation

**Boot Sequence:**
1. System boots → LED boot pattern
2. 30 s still-bias calibration (device must remain stationary)
3. Ready state → LED double-blink
4. Press button to start recording

**Recording:**
- LED solid = actively recording
- Data buffered in 1024-record blocks (~32 KiB)
- Async writer drains SD queue; periodic GC keeps timing clean
- Press button again to stop and safely unmount SD card

**Files:**
- Saved as `bno_XXX.bin` on SD card plus `bno_XXX_meta.txt`
- v2 header: 42 bytes (magic `BNO2`, version, rate, flags, start epoch, device ID, bias, sample count, CRC32)
- Per-record payload: 32 bytes – `t_ms`, linear accel (m/s²), quaternion (qi,qj,qk,qr)
- Companion `bno_XXX_qa.csv` logs periodic raw accel/gyro snapshots + quaternion norm

## Planned Improvements

- **Add LED error pattern** for SD/IMU init failures.
- **Optional periodic fsync** (every 30–60 s) to bound worst-case data loss.
- **Expose IMU temperature** alongside QA snapshots.
- **Automate PSD-based high-pass selection** once long-run spectra are analysed.
