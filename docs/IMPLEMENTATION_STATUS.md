# Implementation Status

Date: 2026-02-21

## Completed

- Engine core created in a new location:
  - `/home/justin/Pictures/billiardengine/engine-rewrite/src/BilliardsPhysicsEngine`
- Implemented public contracts:
  - `BallState`, `BallInitState`, `CueShotInput`, `TableGeometry`, `PocketGeometry`, `SimulationFrame`, `PhysicsEvent`, `RackPreset`, `PredictionSettings`
- Implemented canonical tuning source:
  - `PhysicsProfile` class (single source of truth)
- Implemented fixed-step engine:
  - `BilliardsPhysicsWorld` with 240-500Hz-compatible stepping (default 480)
  - ball-ball CCD (quadratic TOI)
  - normal + tangential impulse collision response
  - sliding->rolling cloth model and spin decay
  - cushion rebound with english influence
  - pocket mouth/jaw/drop logic
  - rest detection + `AllBallsStopped` event
- Implemented prediction:
  - shot trajectory prediction and first-contact tracking
  - first-contact metadata output (ball id, contact point, contact time)
- Implemented Unity adapters (drop-in scripts):
  - runtime controller
  - cue strike adapter
  - compatibility shims
  - training/ghost-path helpers
  - first-contact ghost marker overlay in training renderer
- Implemented validation harness:
  - `tests/ShotSuiteRunner`
  - 10-case shot suite fixture and baseline recorder
  - standalone sanity checks (`--sanity`):
    - head-on transfer
    - overlap recovery
    - rail bounce response
    - pocket event emission
    - shot lifecycle events
    - prediction isolation
  - deterministic rerun checks (`--determinism`)
  - randomized stress invariants (`--stress`)
  - benchmark breakdown + perf warnings (`--benchmark`, `--profile`, `--perf-budget-ms`)
  - benchmark matrix markdown report (`--benchmark-matrix`, optional `--benchmark-report`)
  - non-default profile baseline guard (skips fixture compare unless `--force-baseline`)
- Stability fixes:
  - corrected ball-ball normal-velocity sign handling in impulse resolution
  - added iterative post-collision penetration stabilization pass

## Verified

- `dotnet build` succeeds for core and test projects.
- Shot suite runner executes and passes against recorded baseline fixture.
- Sanity checks (6/6) pass without Unity scene/model dependencies.
- Prediction sanity includes first-contact metadata validity checks.
- Determinism check passes across repeated runs.
- Stress check passes at 1000 randomized shots (seed 1337).
- Benchmark profile presets execute with detailed throughput metrics and warning thresholds.
- Benchmark matrix runner writes a docs report table at `docs/BENCHMARK_MATRIX.md`.

## Deferred (Post-V1)

- Jump shots
- Massé
- Squirt/deflection
- Miscues
- Cross-platform lockstep determinism
