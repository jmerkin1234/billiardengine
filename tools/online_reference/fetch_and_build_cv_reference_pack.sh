#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DATASET_URL="${DATASET_URL:-https://github.com/matteodalnevo/CV_Project.git}"
DATASET_DIR="${DATASET_DIR:-/tmp/billiard_online_refs/CV_Project}"
OUTPUT_PATH="${OUTPUT_PATH:-${REPO_ROOT}/tests/ShotSuiteRunner/fixtures/online_reference_pack.json}"

mkdir -p "$(dirname "${DATASET_DIR}")"

if [[ -d "${DATASET_DIR}/.git" ]]; then
  git -C "${DATASET_DIR}" fetch --all --tags --prune
  git -C "${DATASET_DIR}" pull --ff-only
else
  git clone --depth 1 "${DATASET_URL}" "${DATASET_DIR}"
fi

SOURCE_COMMIT="$(git -C "${DATASET_DIR}" rev-parse HEAD)"

python3 "${SCRIPT_DIR}/build_cv_reference_pack.py" \
  --dataset-root "${DATASET_DIR}/data" \
  --source-repo "${DATASET_URL%.git}" \
  --source-commit "${SOURCE_COMMIT}" \
  --output "${OUTPUT_PATH}"

echo "online reference pack generated at: ${OUTPUT_PATH}"
