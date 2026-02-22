# Online Reference Sources

Date: 2026-02-22

## Usable Now

- `CV_Project` dataset repository:
  - URL: `https://github.com/matteodalnevo/CV_Project`
  - Retrieved commit: `d8de7952782a36c17e5d427cb8ff8649f1703025`
  - Contains 10 clip samples with:
    - per-clip `gameX_clipY.mp4`
    - first/last annotated bounding boxes
    - first/last segmentation masks
    - table-corner file for homography mapping
  - Current usage in this repo:
    - source for `tests/ShotSuiteRunner/fixtures/online_reference_pack.json`
    - summarized by `docs/ONLINE_REFERENCE_REPORT.md`

## Candidate (Currently Gated)

- `Billiard-dataset` repository:
  - URL: `https://github.com/FJ-Rodriguez-Lozano/Billiard-dataset`
  - Paper citation in repo:
    - DOI: `10.1007/s10489-023-04542-3`
  - Notes:
    - README indicates larger dataset (manual trajectory subset included) hosted on SharePoint.
    - SharePoint link currently returns access denied from this environment (cannot fetch directly).

## Practical Constraint

- Current imported online references are coarse first/last frame observations.
- They are useful for external realism direction and regression visibility, but they are not yet one-to-one replacements for shot-suite numeric pass/fail fixtures.
