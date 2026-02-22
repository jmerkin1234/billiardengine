# AGENT.md

## Purpose
Working log for ongoing Codex changes in this repository.

## Update Policy
- Keep this file updated whenever code or behavior changes.
- Keep these docs in sync together:
  - `/home/justin/Pictures/billiardengine/engine-rewrite/AGENT.md`
  - `/home/justin/Pictures/billiardengine/engine-rewrite/README.md`
  - `/home/justin/Pictures/billiardengine/engine-rewrite/docs/IMPLEMENTATION_STATUS.md`

## Current Snapshot (2026-02-22)
- Repository: `https://github.com/jmerkin1234/billiardengine`
- Branch: `codex/main/engine-rewrite`
- Scope: standalone 8ft 8-ball physics core + Unity adapter + validation runner
- Recent engine stability work:
  - fixed ball-ball relative normal sign handling in collision impulse resolution
  - added iterative residual penetration solver
  - exposed extra stabilization tuning in `PhysicsProfile`
  - added benchmark profile presets + perf warning thresholds in runner
  - added benchmark matrix runner that writes markdown report tables
  - added first-contact prediction metadata + ghost marker overlay support
  - hardened fixture schema with locked reference metadata + event expectation gates
  - added validation report generator with per-preset pass/fail deltas
  - added online-reference ingestion pipeline (`tools/online_reference`)
  - added online-reference runner mode (`--online-reference-report`) and docs output (`docs/ONLINE_REFERENCE_REPORT.md`)
- Validation status:
  - `dotnet build` passes
  - `--sanity` passes (head-on transfer, overlap recovery, rail bounce, pocket event emission, shot lifecycle events, prediction isolation)
  - `--determinism 3` passes
  - `--stress 1000 --seed 1337` passes
  - baseline suite passes `10/10` against current fixture
  - `--benchmark` reports avg/p50/p95/p99/worst plus sim-step throughput and budget warnings
  - `--benchmark-matrix` writes `/home/justin/Pictures/billiardengine/engine-rewrite/docs/BENCHMARK_MATRIX.md`
  - `--validation-report` writes `/home/justin/Pictures/billiardengine/engine-rewrite/docs/VALIDATION_REPORT.md`
  - `--online-reference-report` writes `/home/justin/Pictures/billiardengine/engine-rewrite/docs/ONLINE_REFERENCE_REPORT.md`
  - `--require-real-reference` currently fails because fixture metadata is placeholder (`SIMULATED_PLACEHOLDER_NEEDS_REAL_CAPTURE`)

## Next Work Queue
- Improve physical realism against external references (beyond self-baseline parity).
- Expand pocket/jaw and rail english acceptance assertions.
- Continue Unity scene integration and gameplay event compatibility verification.

## Change Log
- 2026-02-21:
  - initialized standalone repo and pushed to GitHub
  - completed collision stability pass and re-baselined fixture suite
  - expanded sanity-mode checks with pocket events, shot lifecycle events, and prediction isolation checks
  - added benchmark profile presets (`physics`, `balanced`, `debug240`) and non-default baseline guard (`--force-baseline`)
  - added benchmark matrix command (`--benchmark-matrix`) and markdown report output path (`--benchmark-report`)
  - added prediction first-contact point/time metadata and hooked first-contact marker rendering into `GhostPathRenderer`
- 2026-02-22:
  - added locked reference metadata and event expectation checks to fixture schema
  - added real-world reference enforcement flag (`--require-real-reference`)
  - added full validation report command (`--validation-report`) and docs report output (`docs/VALIDATION_REPORT.md`)
  - added online dataset import script for public 8-ball clip annotations (`build_cv_reference_pack.py`)
  - added fetch+build wrapper (`fetch_and_build_cv_reference_pack.sh`) and generated `tests/ShotSuiteRunner/fixtures/online_reference_pack.json`
  - added online reference summary mode (`--online-reference-report`) and generated `docs/ONLINE_REFERENCE_REPORT.md`
