# AGENT.md

## Purpose
Working log for ongoing Codex changes in this repository.

## Update Policy
- Keep this file updated whenever code or behavior changes.
- Keep these docs in sync together:
  - `/home/justin/Pictures/billiardengine/engine-rewrite/AGENT.md`
  - `/home/justin/Pictures/billiardengine/engine-rewrite/README.md`
  - `/home/justin/Pictures/billiardengine/engine-rewrite/docs/IMPLEMENTATION_STATUS.md`

## Current Snapshot (2026-02-21)
- Repository: `https://github.com/jmerkin1234/billiardengine`
- Branch: `codex/main/engine-rewrite`
- Scope: standalone 8ft 8-ball physics core + Unity adapter + validation runner
- Recent engine stability work:
  - fixed ball-ball relative normal sign handling in collision impulse resolution
  - added iterative residual penetration solver
  - exposed extra stabilization tuning in `PhysicsProfile`
  - added benchmark profile presets + perf warning thresholds in runner
- Validation status:
  - `dotnet build` passes
  - `--sanity` passes (head-on transfer, overlap recovery, rail bounce, pocket event emission, shot lifecycle events, prediction isolation)
  - `--determinism 3` passes
  - `--stress 1000 --seed 1337` passes
  - baseline suite passes `10/10` against current fixture
  - `--benchmark` reports avg/p50/p95/p99/worst plus sim-step throughput and budget warnings

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
