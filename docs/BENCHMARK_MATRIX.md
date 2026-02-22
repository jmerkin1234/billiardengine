# Benchmark Matrix Report

- Generated (UTC): `2026-02-22 01:50:19 UTC`
- Shots per preset: `24`
- Seed: `1337`
- Perf budget: `6.00 ms/shot`

| Preset | Hz | Completed | Avg ms | P95 ms | Worst ms | Avg steps | Avg sim (s) | ms/sim-s | RT factor | Budget |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| `physics` | 480 | 24/24 | 205.096 | 260.314 | 276.131 | 5469.4 | 11.395 | 17.999 | 55.56x | warn (3) |
| `balanced` | 360 | 24/24 | 149.340 | 195.478 | 196.939 | 3955.9 | 10.989 | 13.590 | 73.58x | warn (3) |
| `debug240` | 240 | 24/24 | 106.489 | 124.851 | 130.433 | 2732.0 | 11.383 | 9.355 | 106.89x | warn (3) |

## Notes
- `physics` is the default high-fidelity profile.
- `balanced` and `debug240` are manual fallback presets.
- No automatic fidelity downshift is applied by the engine.
