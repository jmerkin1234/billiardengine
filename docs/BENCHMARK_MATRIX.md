# Benchmark Matrix Report

- Generated (UTC): `2026-02-22 01:34:37 UTC`
- Shots per preset: `24`
- Seed: `1337`
- Perf budget: `6.00 ms/shot`

| Preset | Hz | Completed | Avg ms | P95 ms | Worst ms | Avg steps | Avg sim (s) | ms/sim-s | RT factor | Budget |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| `physics` | 480 | 24/24 | 207.057 | 272.644 | 276.053 | 5469.4 | 11.395 | 18.171 | 55.03x | warn (3) |
| `balanced` | 360 | 24/24 | 153.993 | 193.111 | 202.575 | 3955.9 | 10.989 | 14.014 | 71.36x | warn (3) |
| `debug240` | 240 | 24/24 | 110.525 | 134.699 | 143.240 | 2732.0 | 11.383 | 9.710 | 102.99x | warn (3) |

## Notes
- `physics` is the default high-fidelity profile.
- `balanced` and `debug240` are manual fallback presets.
- No automatic fidelity downshift is applied by the engine.
