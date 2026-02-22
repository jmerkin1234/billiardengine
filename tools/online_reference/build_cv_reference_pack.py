#!/usr/bin/env python3
"""Build an online reference pack from the CV_Project billiards dataset.

This importer reads the per-clip first/last frame bounding-box annotations and
table corner coordinates, maps ball centers into normalized table coordinates,
and emits a compact JSON pack that can be consumed by ShotSuiteRunner reporting.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import pathlib
import re
from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np


CATEGORY_NAMES = {
    0: "Background",
    1: "CueBall",
    2: "EightBall",
    3: "Solid",
    4: "Stripe",
    5: "Table",
}

PLAYFIELD_WIDTH_METERS = 1.1176
PLAYFIELD_LENGTH_METERS = 2.2352
MOVE_THRESHOLD_METERS = 0.03


@dataclass(frozen=True)
class Observation:
    category_id: int
    center_px: Tuple[float, float]
    center_norm: Tuple[float, float]
    center_meters: Tuple[float, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build online reference pack JSON from CV_Project data.")
    parser.add_argument(
        "--dataset-root",
        required=True,
        help="Path to CV_Project/data directory.",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output JSON file path.",
    )
    parser.add_argument(
        "--source-repo",
        default="https://github.com/matteodalnevo/CV_Project",
        help="Source repository URL recorded in output metadata.",
    )
    parser.add_argument(
        "--source-commit",
        default="unknown",
        help="Source repository commit hash recorded in output metadata.",
    )
    parser.add_argument(
        "--table-width-m",
        type=float,
        default=PLAYFIELD_WIDTH_METERS,
        help="Playfield width (meters).",
    )
    parser.add_argument(
        "--table-length-m",
        type=float,
        default=PLAYFIELD_LENGTH_METERS,
        help="Playfield length (meters).",
    )
    return parser.parse_args()


def parse_table_corners(table_corners_path: pathlib.Path) -> List[Tuple[float, float]]:
    text = table_corners_path.read_text(encoding="utf-8")
    pairs = re.findall(r"\(\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*\)", text)
    if len(pairs) < 4:
        raise ValueError(f"Expected 4 table corners in {table_corners_path}, found {len(pairs)}")

    corners = [(float(x), float(y)) for x, y in pairs[-4:]]
    return corners


def compute_homography(
    src: Sequence[Tuple[float, float]],
    dst: Sequence[Tuple[float, float]],
) -> np.ndarray:
    if len(src) != 4 or len(dst) != 4:
        raise ValueError("Homography requires exactly 4 source and 4 destination points.")

    a_rows: List[List[float]] = []
    b_vals: List[float] = []
    for (x, y), (u, v) in zip(src, dst):
        a_rows.append([x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y])
        b_vals.append(u)
        a_rows.append([0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y])
        b_vals.append(v)

    a = np.asarray(a_rows, dtype=np.float64)
    b = np.asarray(b_vals, dtype=np.float64)
    h = np.linalg.solve(a, b)
    return np.asarray(
        [
            [h[0], h[1], h[2]],
            [h[3], h[4], h[5]],
            [h[6], h[7], 1.0],
        ],
        dtype=np.float64,
    )


def apply_homography(h: np.ndarray, x: float, y: float) -> Tuple[float, float]:
    vec = h @ np.asarray([x, y, 1.0], dtype=np.float64)
    if abs(vec[2]) < 1e-12:
        raise ValueError("Degenerate homography transform.")
    return (float(vec[0] / vec[2]), float(vec[1] / vec[2]))


def load_bbox_file(
    path: pathlib.Path,
    homography: np.ndarray,
    table_width_m: float,
    table_length_m: float,
) -> List[Observation]:
    observations: List[Observation] = []
    lines = [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    for line in lines:
        parts = line.split()
        if len(parts) < 5:
            continue

        x, y, w, h, category = (
            float(parts[0]),
            float(parts[1]),
            float(parts[2]),
            float(parts[3]),
            int(float(parts[4])),
        )
        center_px = (x + 0.5 * w, y + 0.5 * h)
        norm = apply_homography(homography, center_px[0], center_px[1])
        norm_clamped = (max(0.0, min(1.0, norm[0])), max(0.0, min(1.0, norm[1])))
        meters = ((norm_clamped[0] - 0.5) * table_width_m, (norm_clamped[1] - 0.5) * table_length_m)
        observations.append(
            Observation(
                category_id=category,
                center_px=(round(center_px[0], 4), round(center_px[1], 4)),
                center_norm=(round(norm_clamped[0], 6), round(norm_clamped[1], 6)),
                center_meters=(round(meters[0], 6), round(meters[1], 6)),
            )
        )
    return observations


def partition_by_category(observations: Iterable[Observation]) -> Dict[int, List[Observation]]:
    buckets: Dict[int, List[Observation]] = {}
    for obs in observations:
        buckets.setdefault(obs.category_id, []).append(obs)
    return buckets


def match_by_nearest(
    first: Sequence[Observation],
    last: Sequence[Observation],
) -> Tuple[List[Tuple[Observation, Observation, float]], int, int]:
    if not first and not last:
        return [], 0, 0

    remaining_last = list(last)
    matches: List[Tuple[Observation, Observation, float]] = []

    # Stable ordering keeps output deterministic.
    ordered_first = sorted(first, key=lambda o: (o.center_norm[0], o.center_norm[1]))
    for a in ordered_first:
        if not remaining_last:
            break
        best_i = -1
        best_d = math.inf
        for i, b in enumerate(remaining_last):
            dx = a.center_meters[0] - b.center_meters[0]
            dz = a.center_meters[1] - b.center_meters[1]
            d = math.hypot(dx, dz)
            if d < best_d:
                best_d = d
                best_i = i
        b = remaining_last.pop(best_i)
        matches.append((a, b, best_d))

    unmatched_first = max(0, len(first) - len(matches))
    unmatched_last = len(remaining_last)
    return matches, unmatched_first, unmatched_last


def make_metrics(first: Sequence[Observation], last: Sequence[Observation]) -> Dict[str, float | int]:
    first_by_cat = partition_by_category(first)
    last_by_cat = partition_by_category(last)

    matched_count = 0
    moved_count = 0
    max_disp = 0.0
    total_disp = 0.0
    unmatched_first_total = 0
    unmatched_last_total = 0
    cue_disp = -1.0

    for category_id in sorted(set(first_by_cat.keys()) | set(last_by_cat.keys())):
        matches, unmatched_first, unmatched_last = match_by_nearest(
            first_by_cat.get(category_id, []),
            last_by_cat.get(category_id, []),
        )
        unmatched_first_total += unmatched_first
        unmatched_last_total += unmatched_last

        for a, b, d in matches:
            _ = a, b
            matched_count += 1
            total_disp += d
            max_disp = max(max_disp, d)
            if d >= MOVE_THRESHOLD_METERS:
                moved_count += 1
            if category_id == 1 and cue_disp < 0.0:
                cue_disp = d

    first_count = len(first)
    last_count = len(last)
    pocketed_estimate = max(0, first_count - last_count)
    mean_disp = (total_disp / matched_count) if matched_count > 0 else 0.0

    return {
        "FirstBallCount": first_count,
        "LastBallCount": last_count,
        "MatchedBallCount": matched_count,
        "EstimatedPocketedBallCount": pocketed_estimate,
        "EstimatedMovedBallCount": moved_count,
        "UnmatchedFirstCount": unmatched_first_total,
        "UnmatchedLastCount": unmatched_last_total,
        "CueBallDisplacementMeters": round(cue_disp, 6) if cue_disp >= 0.0 else -1.0,
        "MeanMatchedDisplacementMeters": round(mean_disp, 6),
        "MaxMatchedDisplacementMeters": round(max_disp, 6),
        "MoveThresholdMeters": MOVE_THRESHOLD_METERS,
    }


def serialize_observations(observations: Sequence[Observation]) -> List[dict]:
    return [
        {
            "CategoryId": obs.category_id,
            "CategoryName": CATEGORY_NAMES.get(obs.category_id, f"Category{obs.category_id}"),
            "CenterPx": [obs.center_px[0], obs.center_px[1]],
            "CenterNorm": [obs.center_norm[0], obs.center_norm[1]],
            "CenterMeters": [obs.center_meters[0], obs.center_meters[1]],
        }
        for obs in observations
    ]


def main() -> None:
    args = parse_args()
    dataset_root = pathlib.Path(args.dataset_root).expanduser().resolve()
    output_path = pathlib.Path(args.output).expanduser().resolve()

    if not dataset_root.exists():
        raise FileNotFoundError(f"Dataset root not found: {dataset_root}")

    corners_path = dataset_root / "eight_ball_table" / "Table_corners"
    if not corners_path.exists():
        raise FileNotFoundError(f"Table corners file not found: {corners_path}")

    corners = parse_table_corners(corners_path)
    dst_norm = [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0)]
    homography = compute_homography(corners, dst_norm)

    clip_dirs = sorted(
        [p for p in dataset_root.glob("game*_clip*") if p.is_dir()],
        key=lambda p: p.name,
    )
    if not clip_dirs:
        raise ValueError(f"No clip folders found under {dataset_root}")

    clips = []
    for clip_dir in clip_dirs:
        bbox_dir = clip_dir / "bounding_boxes"
        first_path = bbox_dir / "frame_first_bbox.txt"
        last_path = bbox_dir / "frame_last_bbox.txt"
        if not first_path.exists() or not last_path.exists():
            continue

        first_obs = load_bbox_file(first_path, homography, args.table_width_m, args.table_length_m)
        last_obs = load_bbox_file(last_path, homography, args.table_width_m, args.table_length_m)
        metrics = make_metrics(first_obs, last_obs)

        clips.append(
            {
                "ClipId": clip_dir.name,
                "VideoPath": f"{clip_dir.name}/{clip_dir.name}.mp4",
                "FirstBalls": serialize_observations(first_obs),
                "LastBalls": serialize_observations(last_obs),
                "DerivedMetrics": metrics,
            }
        )

    generated_at = dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

    payload = {
        "Source": {
            "Name": "CV_Project",
            "Repository": args.source_repo,
            "RepositoryCommit": args.source_commit,
            "DatasetPath": str(dataset_root),
            "DatasetLicense": "UNSPECIFIED_IN_REPOSITORY",
            "Notes": (
                "10 clip benchmark with first/last frame annotations. "
                "Used as coarse real-world reference candidates."
            ),
        },
        "GeneratedAtUtc": generated_at,
        "Table": {
            "PlayfieldWidthMeters": args.table_width_m,
            "PlayfieldLengthMeters": args.table_length_m,
            "TableCornersPx": [[round(x, 4), round(y, 4)] for x, y in corners],
            "CornerOrder": ["top_left", "bottom_left", "bottom_right", "top_right"],
        },
        "Clips": clips,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"Wrote online reference pack: {output_path}")
    print(f"Clips: {len(clips)}")


if __name__ == "__main__":
    main()
