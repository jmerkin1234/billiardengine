# Validation Report

- Generated (UTC): `2026-02-22 01:50:24 UTC`
- Fixtures: `10`
- Real-world references marked: `0/10`
- Benchmark matrix shots: `24`
- Seed: `1337`
- Perf budget: `6.00 ms/shot`
- Benchmark matrix report: `/home/justin/Pictures/billiardengine/engine-rewrite/docs/BENCHMARK_MATRIX.md`

| Preset | Hz | Passed | Failed | Pass Delta vs physics | Avg Pos Err (m) | Avg Angle Err (deg) | Avg Settle Err (s) | Event Failures |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `physics` | 480 | 10 | 0 | 0 | 0.0000 | 0.000 | 0.000 | 0 |
| `balanced` | 360 | 0 | 10 | -10 | 0.2394 | 48.988 | 0.002 | 4 |
| `debug240` | 240 | 0 | 10 | -10 | 0.2789 | 48.728 | 0.002 | 4 |

## Notes
- Fixture pass/fail uses position, angle, settle-time, and event expectation gates.
- Event gates include first-contact ball id, first-contact time tolerance, and pocket outcome checks.
- `Pass Delta vs physics` is each preset pass-count minus physics pass-count.
- WARNING: `10` fixture(s) are not marked as real-world references.

## Failure Samples
### physics
- none
### balanced
- Straight stop shot: event mismatch: pocket event count expected 9 got 7
- Follow shot: pos=0.2911m angle=106.45deg settle=0.002s
- Draw shot: pos=0.5588m angle=0.00deg settle=0.002s
### debug240
- Straight stop shot: event mismatch: pocket event count expected 9 got 8
- Follow shot: pos=0.5588m angle=0.00deg settle=0.002s
- Draw shot: pos=0.7033m angle=107.54deg settle=0.002s
