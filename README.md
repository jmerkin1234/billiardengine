# Engine Rewrite (Standalone)

Location: `/home/justin/Pictures/billiardengine/engine-rewrite`

This workspace contains a standalone billiards physics engine rewrite for 8ft 8-ball with:

- Pure C# simulation core (`src/BilliardsPhysicsEngine`) with no Unity dependency.
- Unity adapter scripts (`unity-adapter`) for engine-authoritative runtime integration.
- Shot-suite baseline runner (`tests/ShotSuiteRunner`) with tolerance checks.

## Structure

- `src/BilliardsPhysicsEngine`
  - `Core`: ball/table state, cue inputs, events, rack presets
  - `Config`: physics profile (canonical tuning source)
  - `Collision`: CCD utilities
  - `Dynamics`: `BilliardsPhysicsWorld` fixed-step impulse solver
  - `Prediction`: ghost-path trajectory prediction
- `unity-adapter`
  - `Runtime`: `UnityPhysicsRuntimeController`, `UnityCueStrikeAdapter`
  - `Compatibility`: shims for legacy-style ball/spin/pocket access
  - `Training`: training hotkeys + ghost path rendering
- `tests/ShotSuiteRunner`
  - fixture-driven validation harness
  - baseline fixture at `fixtures/shot_suite_baseline.json`

## Build

```bash
dotnet build /home/justin/Pictures/billiardengine/engine-rewrite/src/BilliardsPhysicsEngine/BilliardsPhysicsEngine.csproj
dotnet build /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj
```

## Validation

Record or refresh baseline outcomes:

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --record --allow-record-locked --reference-source SIMULATED_PLACEHOLDER_NEEDS_REAL_CAPTURE --fixtures /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/fixtures/shot_suite_baseline.json
```

Run acceptance suite (`<=3cm`, `<=3deg`, `<=0.25s`):

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --fixtures /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/fixtures/shot_suite_baseline.json
```

Enforce real-world references in CI / gated runs:

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --fixtures /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/fixtures/shot_suite_baseline.json --require-real-reference
```

Run standalone sanity checks (no Unity model required):

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --sanity
```

Current sanity coverage:
- head-on ball-ball transfer
- overlap recovery
- rail bounce response
- pocket event emission
- shot lifecycle events (`ShotStarted`, `AllBallsStopped`)
- prediction isolation (predictor does not mutate live world)

Run deterministic replay check:

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --determinism 3
```

Run randomized stress invariants (no overlap/out-of-bounds/non-finite):

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --stress 1000 --seed 1337
```

Run benchmark with profile presets and perf budget warnings:

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --benchmark 120 --profile physics --perf-budget-ms 6
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --benchmark 120 --profile balanced --perf-budget-ms 6
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --benchmark 120 --profile debug240 --perf-budget-ms 6
```

Run benchmark matrix and write markdown report:

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --benchmark-matrix 120 --perf-budget-ms 6
```

Default report output:
- `/home/justin/Pictures/billiardengine/engine-rewrite/docs/BENCHMARK_MATRIX.md`

Optional custom report path:

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --benchmark-matrix 120 --benchmark-report /absolute/path/report.md
```

Run full validation report (suite deltas by preset + benchmark matrix linkage):

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --validation-report --benchmark-matrix 120 --perf-budget-ms 6
```

Default validation report output:
- `/home/justin/Pictures/billiardengine/engine-rewrite/docs/VALIDATION_REPORT.md`

Build an online reference pack (no local table required) from public clip annotations:

```bash
/home/justin/Pictures/billiardengine/engine-rewrite/tools/online_reference/fetch_and_build_cv_reference_pack.sh
```

Default online reference pack output:
- `/home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/fixtures/online_reference_pack.json`

Generate an online reference summary report:

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --online-reference-report --online-reference-pack /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/fixtures/online_reference_pack.json
```

Default online reference report output:
- `/home/justin/Pictures/billiardengine/engine-rewrite/docs/ONLINE_REFERENCE_REPORT.md`
- source ledger: `/home/justin/Pictures/billiardengine/engine-rewrite/docs/ONLINE_REFERENCE_SOURCES.md`

Run online profile calibration sweep against imported clip metrics:

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --online-fit --online-reference-pack /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/fixtures/online_reference_pack.json
```

Default online fit outputs:
- fitted profile JSON: `/home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/fixtures/physics_profile_online_fit.json`
- calibration report: `/home/justin/Pictures/billiardengine/engine-rewrite/docs/ONLINE_CALIBRATION_REPORT.md`

Optional custom paths:

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --online-fit --online-reference-pack /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/fixtures/online_reference_pack.json --online-fit-output /absolute/path/physics_profile_online_fit.json --online-calibration-report-path /absolute/path/ONLINE_CALIBRATION_REPORT.md
```

Profile presets:
- `physics` (default): 480Hz, highest fidelity.
- `balanced`: 360Hz, moderate fallback.
- `debug240`: 240Hz, manual perf fallback.

Notes:
- Baseline fixture comparison is auto-skipped for non-`physics` presets.
- Add `--force-baseline` to compare fixture expectations on non-default presets.
- Fixtures include locked reference metadata and event expectations (first-contact + pocket outcomes).
- Use `--allow-record-locked` when intentionally rewriting locked fixtures.

## Unity Integration (Adapter)

1. Add the built DLL or source from `src/BilliardsPhysicsEngine` to your Unity project.
2. Copy adapter scripts from `unity-adapter` into Unity `Assets/Scripts`.
3. Add `UnityPhysicsRuntimeController` to a scene object.
4. Assign balls by tag (`CueBall`, `Ball`) or explicit list.
5. Route shot release to `UnityCueStrikeAdapter.SubmitShot(...)`.
6. Optional: add `TrainingModeController` and `GhostPathRenderer`.
   - `GhostPathRenderer` now supports a first-contact marker overlay using prediction metadata.

## Notes

- Core sim is engine-authoritative, fixed-step, and event-driven.
- Prediction output includes first-contact ball id, point, and relative time metadata.
- Ball-ball response includes corrected relative-normal sign handling and post-collision penetration stabilization.
- Jump/massé/squirt/miscue are intentionally deferred (post-V1 backlog).
- Baseline fixture is currently tagged `SIMULATED_PLACEHOLDER_NEEDS_REAL_CAPTURE` and not yet marked as real-world reference data.
- Online pack references are currently coarse first/last frame observations from public clips (not direct fixture pass/fail gates yet).
- Online fit sweep currently searches `SlidingFriction`, `RollingFriction`, `BallRestitution`, `RailRestitution`, and `PocketEntryAssist` across a 243-trial grid.
