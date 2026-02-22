# Online Calibration Report

- Generated (UTC): `2026-02-22 03:12:49 UTC`
- Pack path: `/home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/fixtures/online_reference_pack.json`
- Source repository: `https://github.com/matteodalnevo/CV_Project`
- Source commit: `d8de7952782a36c17e5d427cb8ff8649f1703025`
- Source license: `UNSPECIFIED_IN_REPOSITORY`
- Valid scenarios used: `10`
- Search trials: `243`
- Baseline score: `2.1425`
- Best score: `1.8668`
- Score improvement: `12.87%`
- Fitted profile output: `/home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/fixtures/physics_profile_online_fit.json`

## Best Coefficients

| Coefficient | Value |
|---|---:|
| `SlidingFriction` | 0.1600 |
| `RollingFriction` | 0.0080 |
| `BallRestitution` | 0.9600 |
| `RailRestitution` | 0.7900 |
| `PocketEntryAssist` | 0.1600 |

## Top Trials

| Trial | Score | Failed Clips | Sliding | Rolling | Ball Rest | Rail Rest | Pocket Assist |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 12 | 1.8668 | 0 | 0.1600 | 0.0080 | 0.9600 | 0.7900 | 0.1600 |
| 39 | 1.8668 | 0 | 0.1600 | 0.0100 | 0.9600 | 0.7900 | 0.1600 |
| 66 | 1.8668 | 0 | 0.1600 | 0.0120 | 0.9600 | 0.7900 | 0.1600 |
| 11 | 1.8979 | 0 | 0.1600 | 0.0080 | 0.9600 | 0.7900 | 0.1200 |
| 38 | 1.8979 | 0 | 0.1600 | 0.0100 | 0.9600 | 0.7900 | 0.1200 |
| 65 | 1.8979 | 0 | 0.1600 | 0.0120 | 0.9600 | 0.7900 | 0.1200 |
| 10 | 1.9064 | 0 | 0.1600 | 0.0080 | 0.9600 | 0.7900 | 0.0800 |
| 37 | 1.9064 | 0 | 0.1600 | 0.0100 | 0.9600 | 0.7900 | 0.0800 |

## Clip Error Delta (Baseline vs Best)

| Clip | Baseline Score | Best Score | Cue Err Δ (m) | Moved Err Δ | Pocket Err Δ | Last Err Δ |
|---|---:|---:|---:|---:|---:|---:|
| `game1_clip1` | 3.0686 | 4.2696 | -0.5112 | 0.00 | 0.00 | 0.00 |
| `game1_clip2` | 2.5248 | 0.9726 | 0.0000 | 10.00 | 2.00 | 2.00 |
| `game1_clip3` | 0.2965 | 0.6354 | -0.1099 | 0.00 | 0.00 | 0.00 |
| `game1_clip4` | 1.0022 | 1.9536 | -0.5876 | 0.00 | 0.00 | 0.00 |
| `game2_clip1` | 1.3936 | 1.0757 | 0.2381 | -3.00 | 0.00 | 0.00 |
| `game2_clip2` | 1.7646 | 0.9622 | 0.5068 | -4.00 | 1.00 | 1.00 |
| `game3_clip1` | 1.4800 | 1.9574 | -0.1570 | -2.00 | 0.00 | 0.00 |
| `game3_clip2` | 4.3318 | 2.7274 | 0.6018 | 0.00 | 1.00 | 1.00 |
| `game4_clip1` | 2.6928 | 2.0932 | 0.2373 | 2.00 | 0.00 | 0.00 |
| `game4_clip2` | 2.8706 | 2.0207 | 0.0203 | 7.00 | 1.00 | 1.00 |

## Notes
- Calibration uses coarse first/last frame clip metrics, not full trajectory ground truth.
- This fit is a guidance profile for realism direction, not a strict replacement for shot-suite acceptance gates.
- Re-run calibration after importing higher-fidelity external references.
