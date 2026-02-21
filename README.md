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
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --record --fixtures /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/fixtures/shot_suite_baseline.json
```

Run acceptance suite (`<=3cm`, `<=3deg`, `<=0.25s`):

```bash
dotnet run --project /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/ShotSuiteRunner.csproj -- --fixtures /home/justin/Pictures/billiardengine/engine-rewrite/tests/ShotSuiteRunner/fixtures/shot_suite_baseline.json
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

Profile presets:
- `physics` (default): 480Hz, highest fidelity.
- `balanced`: 360Hz, moderate fallback.
- `debug240`: 240Hz, manual perf fallback.

Notes:
- Baseline fixture comparison is auto-skipped for non-`physics` presets.
- Add `--force-baseline` to compare fixture expectations on non-default presets.

## Unity Integration (Adapter)

1. Add the built DLL or source from `src/BilliardsPhysicsEngine` to your Unity project.
2. Copy adapter scripts from `unity-adapter` into Unity `Assets/Scripts`.
3. Add `UnityPhysicsRuntimeController` to a scene object.
4. Assign balls by tag (`CueBall`, `Ball`) or explicit list.
5. Route shot release to `UnityCueStrikeAdapter.SubmitShot(...)`.
6. Optional: add `TrainingModeController` and `GhostPathRenderer`.

## Notes

- Core sim is engine-authoritative, fixed-step, and event-driven.
- Ball-ball response includes corrected relative-normal sign handling and post-collision penetration stabilization.
- Jump/massé/squirt/miscue are intentionally deferred (post-V1 backlog).
- Baseline fixture currently contains recorded output from this solver build; if solver logic changes, re-record baseline.
