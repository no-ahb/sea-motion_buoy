# Recorder Scripts

MicroPython firmware for the Raspberry Pi Pico 2 + BNO085 stack. `sea-movement_recorder.py` is now the shipping build that produced the 7 h `bno_008` dataset (50 Hz, synchronous writer, QA buffering, SD retries).

## Files

- `sea-movement_recorder.py` – Production recorder (copy as `main.py` on Pico)
- `functional_baselineScript.py` – Legacy baseline, kept for comparison
- `bno085_test.py` – Quick IMU sanity-check script

## Upload to Pico

1. Copy `scripts/library/BOSCH-BNO085-I2C-micropython-library/lib/bno08x.py` to the Pico root.
2. Open `sea-movement_recorder.py`, adjust config if needed, and save it to the Pico as `main.py`.
2. Ensure MicroSD card is mounted and accessible
3. The `main.py` file will automatically execute on boot when powered by battery

## Configuration

Key knobs (defaults ship-ready):
- `RATE_HZ = 50`, `BIAS_RATE = 25`, `BIAS_SECS = 30`
- `BLOCK_RECS = 1024`, `FLUSH_EVERY_N = 4` (bounds SD loss to ≤4 blocks)
- `SPI_BAUD = 12_000_000` post-mount (1 MHz during mount)
- `MAX_BUFFERS = 6` in sync mode; bump if you re-enable threading
- `QA_FLUSH_INTERVAL = 6` (every ~30 s) – safe buffered writes
- `INT_PIN` can be set to the BNO interrupt pin to allow IRQ pacing (loop waits ≤8 ms)

## Operation

**Boot Sequence:**
1. System boots → LED boot pattern
2. 30 s still-bias calibration (device must remain stationary)
3. Ready state → LED double-blink
4. Press button to start recording

**Recording:**
- LED solid = actively recording
- Button press requests graceful stop (flushes final partial block + QA buffer)
- SD writes are synchronous with retries; `.err` is created if an exception bubbles up
- `_meta.txt` captures jitter stats, SD throughput, VSYS minima, QA count, reason codes

**Files written per run:**
- `bno_###.bin` – binary stream (v2 header + 32 byte records)
- `bno_###_meta.txt` – textual stats (stop reason, jitter, SD usage, QA info)
- `bno_###_qa.csv` – periodic QA snapshots (accel, gyro, quaternion norm, VSYS)
- `bno_###.err` – only when an exception or SD error occurs

## Next up

- LED error pattern for obvious mount failures
- Optional SD rollover when a write error leaves a dirty tail
- IMU temperature logging inside QA CSV
- Configurable reconstruction-friendly downlink (BLE or UART streaming)
