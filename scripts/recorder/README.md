# Recorder Scripts

MicroPython scripts for the Raspberry Pi Pico 2 to capture IMU data.

## Files

- `sea-movement-recorder_script.py` - Main recorder script
- `bno085_test.py` - Test/development script for BNO085 sensor

## Upload to Pico

1. Copy files to Pico (via Thonny or similar)
2. Copy `bno08x.py` library to Pico:
   - From: `scripts/library/BOSCH-BNO085-I2C-micropython-library/lib/bno08x.py`
   - To: Pico root directory
3. Ensure MicroSD card is mounted and accessible
4. Run `sea-movement-recorder_script.py` on the Pico

## Configuration

Edit these constants in `sea-movement-recorder_script.py`:
- `RATE_HZ` - Sampling rate (default: 100 Hz)
- `I2C_FREQ` - I2C bus frequency
- `SPI_BAUD` - SPI bus baudrate for SD card
- `LED_PIN`, `BTN_PIN` - Pin assignments

## Operation

- LED indicates status (blinking = ready, solid = recording)
- Press button to start/stop recording
- Files saved as `bno_XXX.bin` on SD card

