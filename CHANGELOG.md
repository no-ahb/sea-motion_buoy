# Changelog

All notable changes to scripts will be documented in this file.

## [Unreleased]

### Recorder Script
- **Version 1.2** (Current)
  - ✅ Double-buffered SD writing for stable high-rate sampling
    - Two pre-allocated 18 KB buffers (buf_a, buf_b)
    - Buffer switching allows continuous sampling while writing
  - ✅ Effective sample rate tracking
    - Calculates actual achieved sampling rate
    - Writes metadata to sidecar `_meta.txt` file with:
      - Target vs effective rate
      - Total samples, duration, blocks written
  - ✅ Configurable periodic flushing
    - Removed per-block `flush()` calls
    - Added `FLUSH_EVERY_N` config (default: 10 blocks)
    - Final flush on stop for data safety
  - **Version 1.1**: Initial version
    - Basic IMU logging at 100 Hz
    - Records: timestamp, linear acceleration (lx,ly,lz), quaternion (qi,qj,qk,qr)
    - File format: 8-byte header ("BNO1" magic) + 32-byte records
    - Features: Button-triggered start/stop, LED status, buffered writes, bias calibration

### Decoder Script
- Initial version: Basic binary file reader
- Version 1.1: Fixed to skip 8-byte header, use correct 32-byte record size
- Outputs: Sample count, header info, first 5 records for verification

## Format

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

