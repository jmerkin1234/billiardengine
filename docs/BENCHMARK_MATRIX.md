# Benchmark Matrix Report

- Generated (UTC): `2026-02-21 22:08:45 UTC`
- Shots per preset: `24`
- Seed: `1337`
- Perf budget: `6.00 ms/shot`

| Preset | Hz | Completed | Avg ms | P95 ms | Worst ms | Avg steps | Avg sim (s) | ms/sim-s | RT factor | Budget |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| `physics` | 480 | 24/24 | 209.537 | 266.619 | 277.725 | 5469.4 | 11.395 | 18.389 | 54.38x | warn (3) |
| `balanced` | 360 | 24/24 | 153.834 | 195.009 | 199.259 | 3955.9 | 10.989 | 13.999 | 71.43x | warn (3) |
| `debug240` | 240 | 24/24 | 106.958 | 125.918 | 128.334 | 2732.0 | 11.383 | 9.396 | 106.43x | warn (3) |

## Notes
- `physics` is the default high-fidelity profile.
- `balanced` and `debug240` are manual fallback presets.
- No automatic fidelity downshift is applied by the engine.
