#!/usr/bin/env python3
"""Run Allen's Tier-0 timing-vs-map diagnostics without ROS replay.

The script joins a GICP debug-only bag to the matching local-ENU
``/gps_p1/filtered_odom`` stream, then answers three questions:

1. Is position error predominantly along the vehicle velocity direction?
2. Do position error and GICP fitness increase with speed?
3. Do merged scan spans come from acquisition phase, even though each Iris
   source has a normal per-point time span?

The third check optionally reads one indexed window from the raw bag.  Clouds
are paired by per-point mid-time; header deltas are reported separately and
are never interpreted as clock offsets.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import sys
import tempfile
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader
from scipy.stats import linregress, pearsonr, spearmanr

from check_luminar_clock_hypothesis import (
    DEFAULT_AUX_TOPICS,
    FRONT_TOPIC,
    RawCloud,
    collect_window,
    nearest_by,
    point_timestamps_ns,
)


DEBUG_PREFIX = "/gicp/localization/debug/"
FLOAT_TOPICS = {
    "fitness": DEBUG_PREFIX + "fitness",
    "gt_pos_err_m_debug": DEBUG_PREFIX + "gt_pos_err_m",
    "scan_time_span_s": DEBUG_PREFIX + "scan_time_span_s",
    "aux0_merge_dt_s": DEBUG_PREFIX + "aux0_merge_dt_s",
    "aux1_merge_dt_s": DEBUG_PREFIX + "aux1_merge_dt_s",
    "aux0_points": DEBUG_PREFIX + "aux0_points",
    "aux1_points": DEBUG_PREFIX + "aux1_points",
    "merged_aux_count": DEBUG_PREFIX + "merged_aux_count",
}
FINAL_POSE_TOPIC = DEBUG_PREFIX + "final_pose"
GT_TOPIC = "/gps_p1/filtered_odom"
ACCEPTED_STATUSES = {"ok", "ok_partial"}


def stamp_s(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


@contextmanager
def readable_rosbag(path: Path) -> Iterator[Path]:
    """Repair promoted-bag filename drift in a temporary directory.

    Some manually promoted result bags prefix the MCAP filename while leaving
    the original relative filename in metadata.yaml.  ROS can inspect these,
    but rosbags correctly refuses the inconsistent directory.  Never mutate
    the archived result: construct a temporary metadata+symlink view instead.
    """

    metadata = path / "metadata.yaml"
    if not metadata.exists():
        yield path
        return
    text = metadata.read_text()
    expected = []
    in_paths = False
    for line in text.splitlines():
        stripped = line.strip()
        if stripped == "relative_file_paths:":
            in_paths = True
            continue
        if in_paths and stripped.startswith("- "):
            expected.append(stripped[2:].strip().strip("'\""))
        elif in_paths and stripped and not stripped.startswith("#"):
            break
    if not expected or all((path / item).exists() for item in expected):
        yield path
        return

    mcap_files = sorted(path.glob("*.mcap"))
    if len(expected) != len(mcap_files):
        raise RuntimeError(
            f"metadata expects {expected}, but {path} contains {mcap_files}"
        )
    with tempfile.TemporaryDirectory(prefix="h1_debug_bag_") as tmp_name:
        tmp = Path(tmp_name)
        shutil.copy2(metadata, tmp / "metadata.yaml")
        for wanted, actual in zip(expected, mcap_files):
            os.symlink(actual.resolve(), tmp / wanted)
        yield tmp


def read_debug_bag(path: Path) -> dict[str, Any]:
    values: dict[str, list[tuple[int, float]]] = {key: [] for key in FLOAT_TOPICS}
    poses: list[dict[str, Any]] = []
    topic_to_key = {topic: key for key, topic in FLOAT_TOPICS.items()}
    with readable_rosbag(path) as readable:
        with AnyReader([readable]) as reader:
            connections = [
                connection
                for connection in reader.connections
                if connection.topic == FINAL_POSE_TOPIC or connection.topic in topic_to_key
            ]
            for connection, record_ns, raw in reader.messages(connections=connections):
                message = reader.deserialize(raw, connection.msgtype)
                if connection.topic == FINAL_POSE_TOPIC:
                    poses.append(
                        {
                            "record_ns": int(record_ns),
                            "stamp_s": stamp_s(message.header.stamp),
                            "position": np.array(
                                [
                                    message.pose.position.x,
                                    message.pose.position.y,
                                    message.pose.position.z,
                                ],
                                dtype=np.float64,
                            ),
                        }
                    )
                else:
                    values[topic_to_key[connection.topic]].append(
                        (int(record_ns), float(message.data))
                    )
    if not poses:
        raise RuntimeError(f"no {FINAL_POSE_TOPIC} messages in {path}")
    core_count = len(poses)
    for key in FLOAT_TOPICS:
        if key == "gt_pos_err_m_debug":
            continue
        if len(values[key]) != core_count:
            raise RuntimeError(f"{key}: {len(values[key])} values != {core_count} poses")
    return {"poses": poses, "values": values}


def read_gt(path: Path, start_s: float, stop_s: float) -> dict[str, np.ndarray]:
    stamps: list[float] = []
    positions: list[tuple[float, float, float]] = []
    quaternions: list[tuple[float, float, float, float]] = []
    velocities: list[tuple[float, float, float]] = []
    with AnyReader([path]) as reader:
        connection = next((c for c in reader.connections if c.topic == GT_TOPIC), None)
        if connection is None:
            raise RuntimeError(f"{path} does not contain {GT_TOPIC}")
        for _, _, raw in reader.messages(
            connections=[connection],
            start=int((start_s - 0.1) * 1e9),
            stop=int((stop_s + 0.1) * 1e9),
        ):
            message = reader.deserialize(raw, connection.msgtype)
            stamps.append(stamp_s(message.header.stamp))
            p = message.pose.pose.position
            q = message.pose.pose.orientation
            v = message.twist.twist.linear
            positions.append((p.x, p.y, p.z))
            quaternions.append((q.x, q.y, q.z, q.w))
            velocities.append((v.x, v.y, v.z))
    if not stamps:
        raise RuntimeError(f"no {GT_TOPIC} samples in requested time range")
    return {
        "stamp_s": np.asarray(stamps, dtype=np.float64),
        "position": np.asarray(positions, dtype=np.float64),
        "quaternion_xyzw": np.asarray(quaternions, dtype=np.float64),
        "velocity_body": np.asarray(velocities, dtype=np.float64),
    }


def rotate_vectors_xyzw(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vectors with unit quaternions in xyzw order."""

    xyz = q[:, :3]
    w = q[:, 3:4]
    norm = np.linalg.norm(q, axis=1, keepdims=True)
    qn_xyz = xyz / norm
    qn_w = w / norm
    uv = np.cross(qn_xyz, v)
    uuv = np.cross(qn_xyz, uv)
    return v + 2.0 * (qn_w * uv + uuv)


def join_frames(
    debug: dict[str, Any],
    gt: dict[str, np.ndarray],
    audit: dict[str, Any],
    query_offsets_s: np.ndarray | None = None,
) -> list[dict[str, Any]]:
    poses = debug["poses"]
    frame_stamps = np.asarray([item["stamp_s"] for item in poses])
    if query_offsets_s is None:
        query_offsets_s = np.zeros_like(frame_stamps)
    query_stamps = frame_stamps + query_offsets_s
    gt_stamps = gt["stamp_s"]
    right = np.searchsorted(gt_stamps, query_stamps, side="left")
    right = np.clip(right, 1, len(gt_stamps) - 1)
    left = right - 1
    denom = gt_stamps[right] - gt_stamps[left]
    alpha = np.divide(
        query_stamps - gt_stamps[left],
        denom,
        out=np.zeros_like(frame_stamps),
        where=denom > 0,
    )
    alpha = np.clip(alpha, 0.0, 1.0)
    gt_position = gt["position"][left] * (1.0 - alpha[:, None]) + gt["position"][right] * alpha[:, None]
    nearest = np.where(
        np.abs(query_stamps - gt_stamps[left]) <= np.abs(query_stamps - gt_stamps[right]),
        left,
        right,
    )
    gt_dt_s = np.abs(query_stamps - gt_stamps[nearest])
    velocity_body = gt["velocity_body"][nearest]
    velocity_world = rotate_vectors_xyzw(gt["quaternion_xyzw"][nearest], velocity_body)
    speed = np.linalg.norm(velocity_body[:, :2], axis=1)
    final_position = np.stack([item["position"] for item in poses])
    error = final_position - gt_position

    direction = np.divide(
        velocity_world,
        np.linalg.norm(velocity_world, axis=1, keepdims=True),
        out=np.full_like(velocity_world, np.nan),
        where=np.linalg.norm(velocity_world, axis=1, keepdims=True) > 1e-6,
    )
    along = np.sum(error * direction, axis=1)
    horizontal_direction = direction[:, :2]
    horizontal_norm = np.linalg.norm(horizontal_direction, axis=1, keepdims=True)
    horizontal_direction = np.divide(
        horizontal_direction,
        horizontal_norm,
        out=np.full_like(horizontal_direction, np.nan),
        where=horizontal_norm > 1e-6,
    )
    lateral_direction = np.column_stack((-horizontal_direction[:, 1], horizontal_direction[:, 0]))
    lateral = np.sum(error[:, :2] * lateral_direction, axis=1)

    status = audit.get("status_sequence", [])
    if len(status) != len(poses):
        status = ["unknown"] * len(poses)

    debug_gt = debug["values"]["gt_pos_err_m_debug"]
    debug_gt_record = np.asarray([item[0] for item in debug_gt], dtype=np.int64)
    debug_gt_value = np.asarray([item[1] for item in debug_gt], dtype=np.float64)
    pose_record = np.asarray([item["record_ns"] for item in poses], dtype=np.int64)
    if debug_gt_record.size:
        indices = np.searchsorted(debug_gt_record, pose_record)
        indices = np.clip(indices, 1, debug_gt_record.size - 1)
        prior = indices - 1
        indices = np.where(
            np.abs(debug_gt_record[prior] - pose_record) <= np.abs(debug_gt_record[indices] - pose_record),
            prior,
            indices,
        )
        debug_gt_dt_s = np.abs(debug_gt_record[indices] - pose_record) * 1e-9
        debug_gt_matched = np.where(debug_gt_dt_s <= 0.02, debug_gt_value[indices], np.nan)
    else:
        debug_gt_dt_s = np.full(len(poses), np.inf)
        debug_gt_matched = np.full(len(poses), np.nan)

    aligned_values = {
        key: np.asarray([item[1] for item in debug["values"][key]], dtype=np.float64)
        for key in FLOAT_TOPICS
        if key != "gt_pos_err_m_debug"
    }
    rows: list[dict[str, Any]] = []
    for i in range(len(poses)):
        computed_norm = float(np.linalg.norm(error[i]))
        rows.append(
            {
                "frame": i,
                "stamp_s": frame_stamps[i],
                "gt_query_stamp_s": query_stamps[i],
                "gt_query_offset_ms": query_offsets_s[i] * 1e3,
                "status": status[i],
                "accepted": status[i] in ACCEPTED_STATUSES,
                "gt_aligned": bool(math.isfinite(debug_gt_matched[i]) and gt_dt_s[i] <= 0.02),
                "gt_nearest_dt_ms": gt_dt_s[i] * 1e3,
                "speed_mps": speed[i],
                "error_x_m": error[i, 0],
                "error_y_m": error[i, 1],
                "error_z_m": error[i, 2],
                "gt_err_m": computed_norm,
                "gt_err_debug_m": debug_gt_matched[i],
                "gt_err_validation_delta_m": computed_norm - debug_gt_matched[i],
                "along_error_m": along[i],
                "lateral_error_m": lateral[i],
                "vertical_error_m": error[i, 2],
                **{key: values[i] for key, values in aligned_values.items()},
            }
        )
    return rows


def finite(values: Sequence[float]) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    return array[np.isfinite(array)]


def summarize_values(values: Sequence[float]) -> dict[str, float | int] | None:
    array = finite(values)
    if not array.size:
        return None
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "p50": float(np.quantile(array, 0.5)),
        "p95": float(np.quantile(array, 0.95)),
        "max": float(np.max(array)),
    }


def correlation(x: Sequence[float], y: Sequence[float]) -> dict[str, float | int] | None:
    x_array = np.asarray(x, dtype=np.float64)
    y_array = np.asarray(y, dtype=np.float64)
    mask = np.isfinite(x_array) & np.isfinite(y_array)
    x_array = x_array[mask]
    y_array = y_array[mask]
    if x_array.size < 3 or np.ptp(x_array) <= 1e-9 or np.ptp(y_array) <= 1e-9:
        return None
    fit = linregress(x_array, y_array)
    return {
        "count": int(x_array.size),
        "pearson_r": float(pearsonr(x_array, y_array).statistic),
        "spearman_r": float(spearmanr(x_array, y_array).statistic),
        "ols_intercept": float(fit.intercept),
        "ols_slope": float(fit.slope),
        "ols_slope_stderr": float(fit.stderr),
        "ols_pvalue": float(fit.pvalue),
    }


def summarize_subset(rows: list[dict[str, Any]], accepted_only: bool) -> dict[str, Any]:
    subset = [
        row
        for row in rows
        if row["gt_aligned"] and (not accepted_only or row["accepted"])
    ]
    moving = [row for row in subset if row["speed_mps"] >= 1.0]
    along_sq = np.square([row["along_error_m"] for row in moving])
    lateral_sq = np.square([row["lateral_error_m"] for row in moving])
    denominator = float(np.sum(along_sq + lateral_sq))
    return {
        "frame_count": len(subset),
        "moving_frame_count": len(moving),
        "speed_mps": summarize_values([row["speed_mps"] for row in subset]),
        "gt_err_m": summarize_values([row["gt_err_m"] for row in subset]),
        "absolute_along_error_m": summarize_values([abs(row["along_error_m"]) for row in moving]),
        "absolute_lateral_error_m": summarize_values([abs(row["lateral_error_m"]) for row in moving]),
        "absolute_vertical_error_m": summarize_values([abs(row["vertical_error_m"]) for row in moving]),
        "along_horizontal_error_energy_fraction": (
            float(np.sum(along_sq) / denominator) if denominator > 0 else None
        ),
        "speed_vs_gt_err": correlation(
            [row["speed_mps"] for row in subset], [row["gt_err_m"] for row in subset]
        ),
        "speed_vs_fitness": correlation(
            [row["speed_mps"] for row in subset], [row["fitness"] for row in subset]
        ),
        "speed_vs_signed_along_error": correlation(
            [row["speed_mps"] for row in moving],
            [row["along_error_m"] for row in moving],
        ),
        "speed_vs_absolute_along_error": correlation(
            [row["speed_mps"] for row in moving],
            [abs(row["along_error_m"]) for row in moving],
        ),
    }


def timing_frame(
    raw_bag: Path, target_s: float
) -> tuple[dict[str, Any], dict[str, np.ndarray], dict[str, Any]]:
    with AnyReader([raw_bag]) as reader:
        topics = {FRONT_TOPIC, *DEFAULT_AUX_TOPICS}
        connections = [c for c in reader.connections if c.topic in topics]
        clouds, _, _ = collect_window(reader, connections, int(target_s * 1e9), int(0.25e9))
    front = nearest_by(clouds[FRONT_TOPIC], target_s * 1e9, lambda x: x.timing.header_time_ns)
    if front is None:
        raise RuntimeError(f"no front cloud near {target_s:.6f}")
    correctly_selected: dict[str, RawCloud] = {FRONT_TOPIC: front}
    actually_selected: dict[str, RawCloud] = {FRONT_TOPIC: front}
    for topic in DEFAULT_AUX_TOPICS:
        correct = nearest_by(
            clouds[topic], front.timing.point_mid_ns, lambda x: x.timing.point_mid_ns
        )
        actual = nearest_by(
            clouds[topic], front.timing.header_time_ns, lambda x: x.timing.header_time_ns
        )
        if correct is None or actual is None:
            raise RuntimeError(f"no {topic} cloud near front point mid-time")
        correctly_selected[topic] = correct
        actually_selected[topic] = actual

    distributions: dict[str, np.ndarray] = {}
    summary: dict[str, Any] = {
        "front_header_stamp_s": front.timing.header_time_ns * 1e-9,
        "actual_concat": {
            "selection_rule": "nearest header within lidar_concat/time_threshold (deployed code)",
            "sources": {},
        },
        "correct_point_time_pairing": {
            "selection_rule": "nearest per-point mid-time to front (diagnostic reference)",
            "sources": {},
        },
    }
    reference = front.timing.point_mid_ns
    for label, selected in (
        ("actual_concat", actually_selected),
        ("correct_point_time_pairing", correctly_selected),
    ):
        merged_min = min(cloud.timing.point_min_ns for cloud in selected.values())
        merged_max = max(cloud.timing.point_max_ns for cloud in selected.values())
        summary[label]["merged_span_ms"] = (merged_max - merged_min) * 1e-6
        for topic, cloud in selected.items():
            relative_ms = (point_timestamps_ns(cloud.message).astype(np.float64) - reference) * 1e-6
            relative_ms = relative_ms[np.isfinite(relative_ms)]
            if label == "actual_concat":
                distributions[topic] = relative_ms
            summary[label]["sources"][topic] = {
                "point_count": int(relative_ms.size),
                "source_span_ms": cloud.timing.span_ms,
                "header_delta_to_front_ms": (
                    cloud.timing.header_time_ns - front.timing.header_time_ns
                )
                * 1e-6,
                "point_mid_delta_to_front_ms": (
                    cloud.timing.point_mid_ns - front.timing.point_mid_ns
                )
                * 1e-6,
                "point_time_relative_to_front_mid_ms": summarize_values(relative_ms),
            }

    # Reconstruct the localization pose-validity time for every debug frame.
    # The deployed code maps primary_min to header time, sorts the actual
    # merged per-point timestamps, and uses the median unique time as T_prior.
    centered_ns = {
        topic: point_timestamps_ns(cloud.message).astype(np.int64) - cloud.timing.point_mid_ns
        for topic, cloud in correctly_selected.items()
    }
    model = {
        "front_mid_from_min_s": (front.timing.point_mid_ns - front.timing.point_min_ns) * 1e-9,
        "centered_s": {topic: values.astype(np.float64) * 1e-9 for topic, values in centered_ns.items()},
        "canonical_header_phase_s": {
            topic: (cloud.timing.header_time_ns - front.timing.header_time_ns) * 1e-9
            for topic, cloud in correctly_selected.items()
            if topic != FRONT_TOPIC
        },
    }
    return summary, distributions, model


def estimate_gt_query_offsets(debug: dict[str, Any], model: dict[str, Any]) -> np.ndarray:
    """Estimate T_prior median-point time from per-frame concat diagnostics."""

    front = model["centered_s"][FRONT_TOPIC] + model["front_mid_from_min_s"]
    aux_points = {
        DEFAULT_AUX_TOPICS[0]: np.asarray(
            [item[1] for item in debug["values"]["aux0_points"]], dtype=np.float64
        ),
        DEFAULT_AUX_TOPICS[1]: np.asarray(
            [item[1] for item in debug["values"]["aux1_points"]], dtype=np.float64
        ),
    }
    aux_dt = {
        DEFAULT_AUX_TOPICS[0]: np.asarray(
            [item[1] for item in debug["values"]["aux0_merge_dt_s"]], dtype=np.float64
        ),
        DEFAULT_AUX_TOPICS[1]: np.asarray(
            [item[1] for item in debug["values"]["aux1_merge_dt_s"]], dtype=np.float64
        ),
    }
    cache: dict[tuple[Any, ...], float] = {}
    offsets = np.empty(len(front) * 0 + len(aux_points[DEFAULT_AUX_TOPICS[0]]), dtype=np.float64)
    for index in range(offsets.size):
        key_parts: list[Any] = []
        pieces = [front]
        for topic in DEFAULT_AUX_TOPICS:
            if aux_points[topic][index] <= 0 or not math.isfinite(aux_dt[topic][index]):
                key_parts.append((topic, None))
                continue
            raw_shift = aux_dt[topic][index] - model["canonical_header_phase_s"][topic]
            # Iris sweeps are 50 ms apart; remove microsecond header jitter so
            # only the physically distinct phase choices create cache entries.
            phase_shift = round(raw_shift / 0.05) * 0.05
            key_parts.append((topic, phase_shift))
            pieces.append(
                model["centered_s"][topic]
                + model["front_mid_from_min_s"]
                + phase_shift
            )
        key = tuple(key_parts)
        if key not in cache:
            merged = np.concatenate(pieces)
            cache[key] = float(np.median(merged))
        offsets[index] = cache[key]
    return offsets


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def make_plots(output: Path, rows: list[dict[str, Any]], distributions: dict[str, np.ndarray]) -> None:
    subset = [row for row in rows if row["gt_aligned"] and row["accepted"]]
    speed = np.asarray([row["speed_mps"] for row in subset])

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), constrained_layout=True)
    axes[0].scatter(speed, [row["gt_err_m"] for row in subset], s=6, alpha=0.25)
    axes[0].set(xlabel="speed [m/s]", ylabel="GT position error [m]", title="GT error vs speed")
    axes[1].scatter(speed, [row["fitness"] for row in subset], s=6, alpha=0.25)
    axes[1].set(xlabel="speed [m/s]", ylabel="GICP fitness", title="Fitness vs speed")
    fig.savefig(output / "tier0_speed_correlations.png", dpi=170)
    plt.close(fig)

    moving = [row for row in subset if row["speed_mps"] >= 1.0]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), constrained_layout=True)
    axes[0].scatter(
        [row["speed_mps"] for row in moving],
        [row["along_error_m"] for row in moving],
        s=6,
        alpha=0.25,
        label="along",
    )
    axes[0].scatter(
        [row["speed_mps"] for row in moving],
        [row["lateral_error_m"] for row in moving],
        s=6,
        alpha=0.20,
        label="lateral",
    )
    axes[0].legend()
    axes[0].set(xlabel="speed [m/s]", ylabel="signed error [m]", title="Along/lateral decomposition")
    axes[1].boxplot(
        [
            [abs(row["along_error_m"]) for row in moving],
            [abs(row["lateral_error_m"]) for row in moving],
            [abs(row["vertical_error_m"]) for row in moving],
        ],
        labels=["along", "lateral", "vertical"],
        showfliers=False,
    )
    axes[1].set(ylabel="absolute error [m]", title="Direction error distribution")
    fig.savefig(output / "tier0_error_decomposition.png", dpi=170)
    plt.close(fig)

    if distributions:
        fig, ax = plt.subplots(figsize=(8, 4.4), constrained_layout=True)
        for topic, values in distributions.items():
            ax.hist(values, bins=100, histtype="step", density=True, linewidth=1.4, label=topic)
        ax.axvline(0.0, color="black", linewidth=0.8)
        ax.set(
            xlabel="per-point time relative to front mid-time [ms]",
            ylabel="density",
            title="One deployed header-nearest merged cloud",
        )
        ax.legend(fontsize=8)
        fig.savefig(output / "tier0_merged_point_time_distribution.png", dpi=170)
        plt.close(fig)


def write_report(path: Path, summary: dict[str, Any]) -> None:
    accepted = summary["subsets"]["accepted_gt_aligned"]
    gt_corr = accepted["speed_vs_gt_err"]
    fit_corr = accepted["speed_vs_fitness"]
    signed = accepted["speed_vs_signed_along_error"]
    timing = summary.get("one_merged_cloud")
    lines = [
        "# Allen H1 Tier-0 report",
        "",
        "## Result",
        "",
        f"- GT-aligned accepted frames: {accepted['frame_count']} ({accepted['moving_frame_count']} at >=1 m/s).",
        f"- Horizontal error energy along-track: {accepted['along_horizontal_error_energy_fraction']:.3f}.",
        f"- Median |along| / |lateral| / |vertical|: {accepted['absolute_along_error_m']['p50']:.3f} / {accepted['absolute_lateral_error_m']['p50']:.3f} / {accepted['absolute_vertical_error_m']['p50']:.3f} m.",
        f"- Speed vs GT error: Pearson r={gt_corr['pearson_r']:.3f}, OLS slope={gt_corr['ols_slope']:.3f} m/(m/s).",
        f"- Speed vs fitness: Pearson r={fit_corr['pearson_r']:.3f}, OLS slope={fit_corr['ols_slope']:.4f} fitness/(m/s).",
        f"- Speed vs signed along error: Pearson r={signed['pearson_r']:.3f}, slope={signed['ols_slope'] * 1e3:.2f} ms-equivalent.",
        "",
        "The speed slope above is only a whole-localizer diagnostic; it is not an aux clock estimate because map error, recovery snaps, and scan matching also contribute. Pairwise front-vs-aux ICP remains authoritative for clock offset.",
    ]
    if timing:
        actual = timing["actual_concat"]
        correct = timing["correct_point_time_pairing"]
        lines.extend(
            [
                "",
                "## Scan-time sanity check",
                "",
                f"Deployed header-nearest concat gives merged span {actual['merged_span_ms']:.3f} ms; correct point-mid pairing gives {correct['merged_span_ms']:.3f} ms; every individual Iris source spans about 49 ms.",
            ]
        )
        for topic, item in actual["sources"].items():
            lines.append(
                f"- `{topic}`: header delta {item['header_delta_to_front_ms']:+.3f} ms; point-mid delta {item['point_mid_delta_to_front_ms']:+.3f} ms; source span {item['source_span_ms']:.3f} ms."
            )
    lines.extend(["", "See `tier0_summary.json`, `tier0_frames.csv`, and the PNG plots for full evidence.", ""])
    path.write_text("\n".join(lines))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--debug-bag", type=Path, required=True)
    parser.add_argument("--debug-audit", type=Path, required=True)
    parser.add_argument("--gt-bag", type=Path, required=True)
    parser.add_argument("--raw-bag", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    debug = read_debug_bag(args.debug_bag)
    audit = json.loads(args.debug_audit.read_text())
    stamps = [item["stamp_s"] for item in debug["poses"]]
    distributions: dict[str, np.ndarray] = {}
    timing: dict[str, Any] | None = None
    query_offsets = np.zeros(len(stamps), dtype=np.float64)
    if args.raw_bag:
        timing, distributions, timing_model = timing_frame(
            args.raw_bag, float(np.median(stamps))
        )
        query_offsets = estimate_gt_query_offsets(debug, timing_model)
    gt = read_gt(
        args.gt_bag,
        float(np.min(np.asarray(stamps) + query_offsets)),
        float(np.max(np.asarray(stamps) + query_offsets)),
    )
    rows = join_frames(debug, gt, audit, query_offsets)

    validation = finite([row["gt_err_validation_delta_m"] for row in rows])
    summary: dict[str, Any] = {
        "schema_version": 1,
        "inputs": {
            "debug_bag": str(args.debug_bag.resolve()),
            "debug_audit": str(args.debug_audit.resolve()),
            "gt_bag": str(args.gt_bag.resolve()),
            "raw_bag": str(args.raw_bag.resolve()) if args.raw_bag else None,
        },
        "core_frame_count": len(rows),
        "gt_alignment_validation": {
            "published_gt_frame_count": int(np.isfinite([row["gt_err_debug_m"] for row in rows]).sum()),
            "computed_minus_published_abs_p50_m": float(np.quantile(np.abs(validation), 0.5)),
            "computed_minus_published_abs_p95_m": float(np.quantile(np.abs(validation), 0.95)),
            "nearest_odom_dt_ms": summarize_values([row["gt_nearest_dt_ms"] for row in rows]),
        },
        "debug_scan_span_s": summarize_values([row["scan_time_span_s"] for row in rows]),
        "estimated_gt_query_offset_ms": summarize_values(
            [row["gt_query_offset_ms"] for row in rows]
        ),
        "subsets": {
            "all_gt_aligned": summarize_subset(rows, accepted_only=False),
            "accepted_gt_aligned": summarize_subset(rows, accepted_only=True),
        },
    }
    if timing:
        summary["one_merged_cloud"] = timing

    write_csv(args.output_dir / "tier0_frames.csv", rows)
    (args.output_dir / "tier0_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    make_plots(args.output_dir, rows, distributions)
    write_report(args.output_dir / "TIER0_REPORT.md", summary)
    print(json.dumps(summary["subsets"]["accepted_gt_aligned"], indent=2))
    print(f"Artifacts: {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
