# Session 2025-11-12:13 – Motor Scaling Notes

Artifacts live in `recordings/production/2025-11-12:13_session_02/`.

## Recon variants
- `bno_000_motion_time.png/csv`: classic time-domain recon (0.05–3.5 Hz band).
- `bno_000_motion_nohp.png/csv`: HP disabled – shows bias/tide drift.
- `bno_000_motion_freq.png/csv`: freq-domain recon (hp=0.01 Hz, lp=1.2 Hz, 15 min detrend).
- `bno_000_motion_freq_hp002.png/csv`: same but hp=0.02 Hz to cut more wobble.

## Scaling previews (all auto-cropped to 10:54–25:04 h)
- `bno_000_scaling_percentile_soft.*`: percentile 0.01/99.99 + soft knee.
- `bno_000_scaling_dual_band.*`: 30 min moving-average center + high-band modulation (weights 0.35/0.65).
- `bno_000_scaling_rolling_std.*`: percentile base multiplied by 5 min rolling STD envelope.
- `bno_000_scaling_percentile_bias.*`: tanh remap of 0.1/99.9 percentiles.

Legacy percentile comparisons: `bno_000_percentile_02_98.*`, `05_95.*`, `00p5_99p5.*` (cropped window).

## Motor feed
`bno_000_motor_norm_fade10s.*` now uses variance + spike crop (653.6–90,279 s), Hampel/SMA/slew, and global min/max on the cropped window.

## Next experiments
- Frequency recon parameters: tweak `--recon-freq-highpass` (e.g. 0.015) to balance drift vs tide feel.
- Plug each scaling CSV into the motor script to audition: percentile_soft (dynamic mid), dual_band (slow bias), rolling_std (energy-driven amplitude), percentile_bias (tanh feel).
- Consider mixing freq recon output with percentile scaling to emphasize overnight swell shifts.
