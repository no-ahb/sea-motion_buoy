# Recorder Scripts

MicroPython scripts for the Raspberry Pi Pico 2 to capture 9-DoF IMU data for ocean motion logging.

## Files

- `functional_baselineScript.py` - Main recorder script with battery monitoring (rename to `main.py` on device for auto-run)
- `sea-movement_recorder.py` - Alternative recorder script
- `bno085_test.py` - Test/development script for BNO085 sensor

## Upload to Pico

1. Copy files to Pico (via Thonny or similar):
   - `functional_baselineScript.py` → Rename to `main.py` on Pico (for auto-run on battery)
   - `bno08x.py` library → Pico root directory:
     - From: `scripts/library/BOSCH-BNO085-I2C-micropython-library/lib/bno08x.py`
2. Ensure MicroSD card is mounted and accessible
3. The `main.py` file will automatically execute on boot when powered by battery

## Configuration

Edit these constants in `functional_baselineScript.py` (before uploading as `main.py`):
- `RATE_HZ` - Sampling rate (default: 100 Hz; target: 200 Hz)
- `BIAS_SECS` - Still-bias calibration duration (default: 4s; target: 20s)
- `BIAS_RATE` - Calibration sample rate (default: 25 Hz)
- `BLOCK_RECS` - Records per flush block (default: 512, ~18 KB)
- `I2C_FREQ` - I2C bus frequency (default: 400 kHz)
- `SPI_BAUD` - SPI bus baudrate for SD card (default: 12 MHz)
- `LED_PIN`, `BTN_PIN` - Pin assignments

## Operation

**Boot Sequence:**
1. System boots → LED blinking pattern
2. 20s still-bias calibration (device must remain stationary)
3. Ready state → LED steady blink
4. Press button to start recording

**Recording:**
- LED solid = actively recording
- Data logged in 512-record blocks (~18 KB)
- Power-safe block flushing (planned: flush at stop or every N blocks)
- Press button again to stop and safely unmount SD card

**Files:**
- Saved as `bno_XXX.bin` on SD card
- Each file contains 8-byte header + 32-byte records
- Effective sample rate logged at session end (planned)

## Planned Improvements

- **Double-buffered SD writing** for stable 100–200 Hz sampling
- **Record actual effective sample rate** at session end
- **Remove per-loop flush()** calls; flush only at stop or every N blocks
- **Optional CRC per block** for data integrity
- **Power optimization** (reduce SD writes, limit LED use during idle)