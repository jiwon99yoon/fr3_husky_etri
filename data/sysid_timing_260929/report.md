# Sim–Real timing diagnostic (rich SysID logs)

Source: `/home/dyros/etri_ws/src/fr3_husky_etri/data/sysid_raw`. Phase: `inertia_sine_f1.00`. Matched active joints: 14.

This is not the 9/11 Consistency dataset: its raw time-series CSVs are not preserved in the repository.

## Method

- Each real/MuJoCo phase is converted to a common 200 Hz phase-relative time grid.
- The first and last 0.25 s are removed to avoid sine-envelope transients.
- `real_delay_vs_mujoco_ms` is the lag maximizing active-joint velocity cross-correlation over ±100 ms. Positive means real is delayed.
- DTW uses standardized active-joint position and a ±100 ms Sakoe–Chiba band. It measures shape mismatch after allowed local time warping; lower is better.

## Aggregate results

| Metric | Median across 14 active joints |
| --- | ---: |
| Real delay relative to MuJoCo | +42.5 ms |
| Peak velocity correlation | 0.9856 |
| Zero-lag position RMSE | 0.5408 deg |
| Lag-compensated position RMSE | 0.4035 deg |
| Amplitude-normalized DTW shape distance | 0.0875 |

See `timing_metrics.csv` for each joint and `velocity_lag_overlay.png` for the alignment diagnostic.
