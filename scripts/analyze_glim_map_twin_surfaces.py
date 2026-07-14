#!/usr/bin/env python3
"""Check a GLIM map for speed-scaled twin vertical surfaces.

The target map is compared with a same-run control map that did not include
the suspect auxiliary source.  High-speed, low-yaw submaps are matched by
timestamp, rigidly aligned, and vertical-surface residuals are measured.  A
clock offset baked into the map should create a secondary residual mode near
``speed * offset``.  Continuous walls can hide tangential displacement, so a
negative result is reported as "not confirmed", never as proof of no warp.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks
from scipy.spatial import cKDTree


@dataclass
class SubmapInfo:
    path: Path
    stamp_s: float
    speed_mps: float
    yaw_change_rad: float
    world_origin: np.ndarray


def matrix_after(text: str, label: str) -> np.ndarray:
    start = text.index(label)
    lines = text[start:].splitlines()[1:5]
    return np.asarray([[float(value) for value in line.split()] for line in lines])


def read_info(path: Path) -> SubmapInfo | None:
    text = (path / "data.txt").read_text(errors="replace")
    stamps = [float(value) for value in re.findall(r"stamp:\s*([\d.]+)", text)]
    velocities = [
        tuple(map(float, values))
        for values in re.findall(
            r"v_world_imu:\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s+([-+\d.eE]+)", text
        )
    ]
    if not stamps or not velocities:
        return None
    world = matrix_after(text, "T_world_origin:")
    left = matrix_after(text, "T_origin_endpoint_L:")
    right = matrix_after(text, "T_origin_endpoint_R:")
    yaw = lambda transform: math.atan2(transform[1, 0], transform[0, 0])
    yaw_change = math.atan2(
        math.sin(yaw(right) - yaw(left)), math.cos(yaw(right) - yaw(left)
    ))
    speed = np.linalg.norm(np.asarray(velocities)[:, :2], axis=1)
    return SubmapInfo(
        path=path,
        stamp_s=float(np.median(stamps)),
        speed_mps=float(np.median(speed)),
        yaw_change_rad=abs(yaw_change),
        world_origin=world,
    )


def list_infos(dump: Path) -> list[SubmapInfo]:
    infos: list[SubmapInfo] = []
    for path in sorted(dump.glob("[0-9][0-9][0-9][0-9][0-9][0-9]")):
        if not (path / "points_compact.bin").is_file():
            continue
        info = read_info(path)
        if info is not None:
            infos.append(info)
    if not infos:
        raise RuntimeError(f"no readable submaps under {dump}")
    return infos


def select_infos(infos: list[SubmapInfo], count: int) -> list[SubmapInfo]:
    candidates = [
        info for info in infos if info.speed_mps >= 18.0 and info.yaw_change_rad <= 0.04
    ]
    candidates.sort(key=lambda item: (item.yaw_change_rad, -item.speed_mps))
    selected: list[SubmapInfo] = []
    for candidate in candidates:
        position = candidate.world_origin[:2, 3]
        if all(
            np.linalg.norm(position - prior.world_origin[:2, 3]) >= 45.0
            for prior in selected
        ):
            selected.append(candidate)
        if len(selected) >= count:
            break
    if not selected:
        raise RuntimeError("no high-speed low-yaw submaps found")
    return selected


def load_cloud(info: SubmapInfo) -> tuple[np.ndarray, np.ndarray]:
    points = np.fromfile(info.path / "points_compact.bin", dtype=np.float32).reshape(-1, 3)
    normals = np.fromfile(info.path / "normals_compact.bin", dtype=np.float32).reshape(-1, 3)
    rotation = info.world_origin[:3, :3]
    translation = info.world_origin[:3, 3]
    return points @ rotation.T + translation, normals @ rotation.T


def apply_transform(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def align_source_to_target(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    tree = cKDTree(target)
    for _ in range(25):
        moved = apply_transform(source, transform)
        distances, indices = tree.query(moved, k=1, workers=-1)
        keep = np.flatnonzero(distances < 2.0)
        if keep.size < 100:
            break
        keep = keep[np.argsort(distances[keep])[: int(keep.size * 0.8)]]
        a = moved[keep]
        b = target[indices[keep]]
        a_center = a.mean(axis=0)
        b_center = b.mean(axis=0)
        _, _, vt = np.linalg.svd((a - a_center).T @ (b - b_center))
        u, _, vt = np.linalg.svd((a - a_center).T @ (b - b_center))
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        translation = b_center - rotation @ a_center
        delta = np.eye(4)
        delta[:3, :3] = rotation
        delta[:3, 3] = translation
        transform = delta @ transform
        if np.linalg.norm(translation) < 1e-4:
            break
    return transform


def vertical_mask(points: np.ndarray, normals: np.ndarray) -> np.ndarray:
    return (
        np.isfinite(points).all(axis=1)
        & np.isfinite(normals).all(axis=1)
        & (np.abs(normals[:, 2]) < 0.35)
    )


def analyze_pair(
    target: SubmapInfo, control: SubmapInfo, configured_offset_s: float
) -> tuple[dict[str, object], dict[str, np.ndarray]]:
    target_points, target_normals = load_cloud(target)
    control_points, control_normals = load_cloud(control)
    transform = align_source_to_target(control_points, target_points)
    control_aligned = apply_transform(control_points, transform)
    control_normals_aligned = control_normals @ transform[:3, :3].T
    target_vertical = vertical_mask(target_points, target_normals)
    control_vertical = vertical_mask(control_aligned, control_normals_aligned)
    tree = cKDTree(control_aligned[control_vertical])
    distances, _ = tree.query(target_points[target_vertical], k=1, workers=-1)
    distances = distances[np.isfinite(distances) & (distances < 4.0)]

    bins = np.arange(0.0, 4.0001, 0.05)
    counts, edges = np.histogram(distances, bins=bins)
    smooth = gaussian_filter1d(counts.astype(np.float64), 1.5)
    peaks, _ = find_peaks(
        smooth,
        prominence=max(1.0, float(smooth.max()) * 0.015),
        distance=4,
    )
    peak_centers = (edges[:-1] + edges[1:]) * 0.5
    expected = target.speed_mps * abs(configured_offset_s)
    secondary = [
        float(peak_centers[index])
        for index in peaks
        if peak_centers[index] >= 0.4 and abs(peak_centers[index] - expected) <= 0.3
    ]
    p50 = float(np.quantile(distances, 0.5))
    alignment_translation = float(np.linalg.norm(transform[:3, 3]))
    comparison_qualified = (
        abs(control.stamp_s - target.stamp_s) <= 0.35
        and alignment_translation <= 1.5
        and p50 <= 0.5
    )
    metrics: dict[str, object] = {
        "target_submap": target.path.name,
        "control_submap": control.path.name,
        "target_stamp_s": target.stamp_s,
        "control_stamp_s": control.stamp_s,
        "stamp_delta_s": control.stamp_s - target.stamp_s,
        "speed_mps": target.speed_mps,
        "yaw_change_rad": target.yaw_change_rad,
        "configured_offset_s": configured_offset_s,
        "expected_spacing_m": expected,
        "vertical_point_count": int(distances.size),
        "distance_to_control_m": {
            "p50": p50,
            "p90": float(np.quantile(distances, 0.9)),
            "p95": float(np.quantile(distances, 0.95)),
        },
        "histogram_peaks_m": [float(peak_centers[index]) for index in peaks],
        "expected_spacing_peak_m": secondary,
        "raw_expected_peak_detected": bool(secondary),
        "comparison_qualified": comparison_qualified,
        "twin_peak_detected": bool(secondary) and comparison_qualified,
        "alignment_translation_m": alignment_translation,
    }
    plot = {
        "target_points": target_points[target_vertical],
        "control_points": control_aligned[control_vertical],
        "distances": distances,
        "expected": np.asarray([expected]),
    }
    return metrics, plot


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-dir", type=Path, required=True)
    parser.add_argument("--control-map-dir", type=Path, required=True)
    parser.add_argument("--configured-offset", type=float, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--count", type=int, default=5)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    target_infos = list_infos(args.map_dir / "dump")
    control_infos = list_infos(args.control_map_dir / "dump")
    selected = select_infos(target_infos, args.count)
    results: list[dict[str, object]] = []
    plots: list[dict[str, np.ndarray]] = []
    for target in selected:
        control = min(control_infos, key=lambda item: abs(item.stamp_s - target.stamp_s))
        metrics, plot = analyze_pair(target, control, args.configured_offset)
        results.append(metrics)
        plots.append(plot)

    fig, axes = plt.subplots(len(results), 2, figsize=(13, 4.0 * len(results)), constrained_layout=True)
    if len(results) == 1:
        axes = np.asarray([axes])
    for row, (metrics, plot) in enumerate(zip(results, plots)):
        origin = next(info for info in selected if info.path.name == metrics["target_submap"])
        rotation = origin.world_origin[:3, :3]
        translation = origin.world_origin[:3, 3]
        target_local = (plot["target_points"] - translation) @ rotation
        control_local = (plot["control_points"] - translation) @ rotation
        axes[row, 0].scatter(
            control_local[:, 0], control_local[:, 1], s=0.8, alpha=0.3, c="green", label="control"
        )
        axes[row, 0].scatter(
            target_local[:, 0], target_local[:, 1], s=0.8, alpha=0.3, c="magenta", label="target"
        )
        axes[row, 0].set(xlim=(-50, 50), ylim=(-50, 50), aspect="equal")
        axes[row, 0].set_title(
            f"{metrics['target_submap']} v={metrics['speed_mps']:.2f} m/s"
        )
        axes[row, 0].set_xlabel("submap x [m]")
        axes[row, 0].set_ylabel("submap y [m]")
        if row == 0:
            axes[row, 0].legend(markerscale=6)
        axes[row, 1].hist(plot["distances"], bins=np.arange(0, 4.01, 0.05), density=True)
        axes[row, 1].axvline(
            float(plot["expected"][0]), color="red", linestyle="--", label="v*offset"
        )
        axes[row, 1].set(xlim=(0, 4), xlabel="vertical-surface residual to control [m]", ylabel="density")
        axes[row, 1].set_title(
            f"expected {metrics['expected_spacing_m']:.2f} m; twin={metrics['twin_peak_detected']}"
        )
        if row == 0:
            axes[row, 1].legend()
    fig.savefig(args.output_dir / "map_twin_surface_cross_sections.png", dpi=180)
    plt.close(fig)

    report = {
        "schema_version": 1,
        "target_map": str(args.map_dir.resolve()),
        "control_map": str(args.control_map_dir.resolve()),
        "control_semantics": "same Run3; older 30 ms concat window excluded right on normal frames",
        "configured_suspect_offset_s": args.configured_offset,
        "submaps": results,
        "qualified_comparison_count": sum(bool(item["comparison_qualified"]) for item in results),
        "twin_peak_count": sum(bool(item["twin_peak_detected"]) for item in results),
        "decision": "CONFIRMED" if any(item["twin_peak_detected"] for item in results) else "NOT_CONFIRMED",
        "limitation": "continuous walls can absorb tangential displacement; raw pairwise ICP remains clock authority",
    }
    (args.output_dir / "map_wall_summary.json").write_text(json.dumps(report, indent=2) + "\n")
    lines = [
        "# Run 3 map wall / twin-surface check",
        "",
        f"Decision: **{report['decision']}** ({report['twin_peak_count']}/{report['qualified_comparison_count']} qualified high-speed straight comparisons had a secondary residual peak near v*offset; {len(results)} candidates inspected).",
        "",
        "The target map used a +100.04 ms right per-point correction. The control is the same Run 3 map family whose 30 ms merge window normally excluded right, so it supplies a front+left vertical-surface reference.",
        "",
        "| target/control submap | speed | expected v*100.04ms | residual p50/p95 | qualified | twin peak |",
        "|---|---:|---:|---:|---|---|",
    ]
    for item in results:
        distance = item["distance_to_control_m"]
        lines.append(
            f"| {item['target_submap']} / {item['control_submap']} | {item['speed_mps']:.2f} m/s | {item['expected_spacing_m']:.2f} m | {distance['p50']:.2f}/{distance['p95']:.2f} m | {item['comparison_qualified']} | {item['twin_peak_detected']} |"
        )
    lines.extend(
        [
            "",
            "No distinct wall-normal twin mode does not prove the map is clean: on a straight, a time shift moves a continuous wall mainly along its tangent. The map configuration proves the +100.04 ms value was applied; the raw front-vs-right ICP regression is the authoritative clock test.",
            "",
        ]
    )
    (args.output_dir / "MAP_WALL_REPORT.md").write_text("\n".join(lines))
    print(json.dumps({"decision": report["decision"], "qualified_comparison_count": report["qualified_comparison_count"], "twin_peak_count": report["twin_peak_count"], "submaps": results}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
