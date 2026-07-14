#!/usr/bin/env python3
"""Decide whether an auxiliary Luminar clock disagrees with the front clock.

This is an offline, ROS-free implementation of the pairwise-ICP-vs-speed test.
It deliberately does *not* call an 80--90 ms header delta a clock error.  The
timestamp audit reports header phase separately, while the final decision is
made only from geometry:

    ICP along-track correction [m] = intercept [m] + speed [m/s] * tau [s]

The intercept absorbs a constant translational extrinsic error.  ``tau`` is the
constant correction to add to the auxiliary point timestamps and can therefore
be copied, with its sign, to ``lidar_concat/aux_point_time_offsets`` after
review. It must never be inferred from header acquisition phase.

The script uses:

* ``/luminar_front/points`` as the primary cloud;
* ``/luminar_right/points`` and ``/luminar_left/points`` independently;
* ``/atlas/pose_filtered`` for FLU vehicle velocity; and
* ``/atlas/imu_calibrated`` for constant-twist intra-scan deskew.

It reads short indexed windows from the MCAP instead of replaying ROS or making
another full pass over the very large Putnam bags.

Example (Run 5 raw bag, which is the data side of a Run-3-map GICP test):

    python3 scripts/check_luminar_clock_hypothesis.py \
      /media/roar/data1/rosbags/putnam/may_26/run_5/filtered/all \
      --urdf ./av24.urdf \
      --output /tmp/run5_luminar_clock_hypothesis.json

Dependencies: rosbags, numpy, and scipy.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence
from xml.etree import ElementTree

import numpy as np

try:
    from rosbags.highlevel import AnyReader
except ImportError as exc:  # pragma: no cover - exercised only on missing dependency
    raise SystemExit(
        "Missing dependency 'rosbags'. Install it with: python3 -m pip install --user rosbags"
    ) from exc

try:
    from scipy.spatial import cKDTree
    from scipy.stats import pearsonr
except ImportError as exc:  # pragma: no cover - exercised only on missing dependency
    raise SystemExit(
        "Missing dependency 'scipy'. Install it with: python3 -m pip install --user scipy"
    ) from exc


FRONT_TOPIC = "/luminar_front/points"
DEFAULT_AUX_TOPICS = ("/luminar_right/points", "/luminar_left/points")
POSE_TOPIC = "/atlas/pose_filtered"
IMU_TOPIC = "/atlas/imu_calibrated"

POINTFIELD_UINT8 = 2
POINTFIELD_FLOAT32 = 7
POINTFIELD_FLOAT64 = 8


@dataclass(frozen=True)
class CloudTiming:
    topic: str
    log_time_ns: int
    header_time_ns: int
    point_min_ns: int
    point_mid_ns: int
    point_max_ns: int
    point_count: int

    @property
    def span_ms(self) -> float:
        return (self.point_max_ns - self.point_min_ns) * 1e-6


@dataclass
class RawCloud:
    timing: CloudTiming
    message: Any


@dataclass(frozen=True)
class MotionSample:
    velocity_flu: tuple[float, float, float]
    omega_flu: tuple[float, float, float]
    pose_dt_ms: float
    imu_dt_ms: float

    @property
    def speed_mps(self) -> float:
        return math.hypot(self.velocity_flu[0], self.velocity_flu[1])


@dataclass(frozen=True)
class IcpResult:
    converged: bool
    transform: list[list[float]]
    translation_m: tuple[float, float, float]
    rotation_deg: float
    rmse_m: float
    inliers: int
    inlier_ratio: float
    iterations: int
    reason: str


@dataclass(frozen=True)
class FrameResult:
    aux_topic: str
    front_header_s: float
    speed_mps: float
    velocity_flu: tuple[float, float, float]
    omega_flu: tuple[float, float, float]
    front_span_ms: float
    aux_span_ms: float
    selected_header_delta_ms: float
    selected_point_mid_delta_ms: float
    header_nearest_point_mid_delta_ms: float
    along_track_correction_m: float
    lateral_correction_m: float
    vertical_correction_m: float
    icp_rmse_m: float
    icp_inliers: int
    icp_inlier_ratio: float
    icp_rotation_deg: float
    front_points_used: int
    aux_points_used: int


@dataclass(frozen=True)
class TimingObservation:
    front_span_ms: float
    aux_span_ms: float
    selected_header_delta_ms: float
    selected_point_mid_delta_ms: float
    header_nearest_point_mid_delta_ms: float


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _point_field(message: Any, name: str) -> Any:
    for field in message.fields:
        if field.name == name:
            return field
    raise ValueError(f"PointCloud2 has no {name!r} field")


def _point_array(message: Any, field: Any, dtype: str) -> np.ndarray:
    """Return an organized PointCloud2 scalar field as a flattened view."""

    height = int(message.height)
    width = int(message.width)
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    itemsize = np.dtype(dtype).itemsize
    if int(field.offset) + itemsize > point_step:
        raise ValueError(
            f"field {field.name!r} at offset {field.offset} does not fit point_step={point_step}"
        )
    required = (height - 1) * row_step + (width - 1) * point_step + int(field.offset) + itemsize
    if required > len(message.data):
        raise ValueError(
            f"PointCloud2 data is short: need {required} bytes, have {len(message.data)}"
        )
    array = np.ndarray(
        (height, width),
        dtype=dtype,
        buffer=message.data,
        offset=int(field.offset),
        strides=(row_step, point_step),
    )
    return array.reshape(-1) if row_step == width * point_step else array.copy().reshape(-1)


def point_timestamps_ns(message: Any) -> np.ndarray:
    """Decode this repository's Luminar UINT8[8] absolute timestamp contract."""

    field = _point_field(message, "timestamp")
    endian = ">" if bool(message.is_bigendian) else "<"
    if int(field.datatype) == POINTFIELD_UINT8 and int(field.count) == 8:
        return _point_array(message, field, endian + "u8").astype(np.uint64, copy=False)
    if int(field.datatype) == POINTFIELD_FLOAT64 and int(field.count) == 1:
        # The Luminar driver has historically used an 8-byte carrier whose raw
        # bits are uint64 ns even when the declared datatype is FLOAT64.
        return _point_array(message, field, endian + "u8").astype(np.uint64, copy=False)
    raise ValueError(
        "unsupported Luminar timestamp schema: expected UINT8[8] or an 8-byte "
        f"FLOAT64 carrier, got datatype={field.datatype} count={field.count}"
    )


def cloud_timing(topic: str, log_time_ns: int, message: Any) -> CloudTiming:
    stamps = point_timestamps_ns(message)
    valid = stamps[stamps > 0]
    if valid.size == 0:
        raise ValueError(f"{topic}: no nonzero per-point timestamps")
    point_min = int(valid.min())
    point_max = int(valid.max())
    # Mid-range is more stable than the median when scene-dependent returns make
    # the timestamp sample distribution uneven across an Iris sweep.
    point_mid = point_min + (point_max - point_min) // 2
    return CloudTiming(
        topic=topic,
        log_time_ns=int(log_time_ns),
        header_time_ns=_stamp_ns(message.header.stamp),
        point_min_ns=point_min,
        point_mid_ns=point_mid,
        point_max_ns=point_max,
        point_count=int(stamps.size),
    )


def decode_xyz_and_time(message: Any) -> tuple[np.ndarray, np.ndarray]:
    endian = ">" if bool(message.is_bigendian) else "<"
    fields = [_point_field(message, axis) for axis in "xyz"]
    for field in fields:
        if int(field.datatype) != POINTFIELD_FLOAT32 or int(field.count) != 1:
            raise ValueError(
                f"field {field.name!r} is not scalar FLOAT32: "
                f"datatype={field.datatype} count={field.count}"
            )
    xyz = np.column_stack(
        [_point_array(message, field, endian + "f4") for field in fields]
    ).astype(np.float64, copy=False)
    return xyz, point_timestamps_ns(message)


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw matrix, R = Rz(yaw) Ry(pitch) Rx(roll)."""

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def origin_transform(origin: Any | None) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    if origin is None:
        return transform
    xyz = [float(value) for value in origin.attrib.get("xyz", "0 0 0").split()]
    rpy = [float(value) for value in origin.attrib.get("rpy", "0 0 0").split()]
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError("URDF joint origin must contain three xyz and three rpy values")
    transform[:3, :3] = rpy_matrix(*rpy)
    transform[:3, 3] = xyz
    return transform


def load_urdf_extrinsics(
    urdf_path: Path, base_frame: str, sensor_frames: Iterable[str]
) -> dict[str, np.ndarray]:
    """Load T_base_sensor for fixed joints from a URDF tree."""

    root = ElementTree.parse(urdf_path).getroot()
    graph: dict[str, list[tuple[str, np.ndarray]]] = {}
    for joint in root.findall("joint"):
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        if parent_node is None or child_node is None:
            continue
        parent = parent_node.attrib["link"]
        child = child_node.attrib["link"]
        t_parent_child = origin_transform(joint.find("origin"))
        graph.setdefault(child, []).append((parent, t_parent_child))
        graph.setdefault(parent, []).append((child, np.linalg.inv(t_parent_child)))

    def transform_to_base(sensor: str) -> np.ndarray:
        queue: list[tuple[str, np.ndarray]] = [(sensor, np.eye(4))]
        visited: set[str] = set()
        while queue:
            frame, t_frame_sensor = queue.pop(0)
            if frame == base_frame:
                return t_frame_sensor
            if frame in visited:
                continue
            visited.add(frame)
            for neighbor, t_neighbor_frame in graph.get(frame, []):
                if neighbor not in visited:
                    queue.append((neighbor, t_neighbor_frame @ t_frame_sensor))
        raise ValueError(f"URDF has no transform chain from {sensor!r} to {base_frame!r}")

    return {frame: transform_to_base(frame) for frame in sensor_frames}


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def rotate_constant_omega(
    points: np.ndarray, omega: np.ndarray, dt_seconds: np.ndarray
) -> np.ndarray:
    """Apply exp([omega]x * dt) to each row without constructing N matrices."""

    rate = float(np.linalg.norm(omega))
    if rate < 1e-10:
        return points.copy()
    axis = omega / rate
    theta = rate * dt_seconds
    cos_theta = np.cos(theta)[:, None]
    sin_theta = np.sin(theta)[:, None]
    cross = np.cross(np.broadcast_to(axis, points.shape), points)
    dot = points @ axis
    return (
        points * cos_theta
        + cross * sin_theta
        + dot[:, None] * axis[None, :] * (1.0 - cos_theta)
    )


def voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    if voxel_m <= 0.0 or points.size == 0:
        return points
    keys = np.floor(points / voxel_m).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def prepare_cloud(
    raw: RawCloud,
    t_base_sensor: np.ndarray,
    motion: MotionSample,
    *,
    deskew: bool,
    min_range_m: float,
    max_range_m: float,
    min_z_m: float,
    max_z_m: float,
    voxel_m: float,
    max_points: int,
    rng: np.random.Generator,
) -> np.ndarray:
    xyz, stamps_ns = decode_xyz_and_time(raw.message)
    finite = np.isfinite(xyz).all(axis=1) & (stamps_ns > 0)
    radial = np.linalg.norm(xyz[:, :2], axis=1)
    finite &= (radial >= min_range_m) & (radial <= max_range_m)
    xyz = xyz[finite]
    stamps_ns = stamps_ns[finite]
    if xyz.shape[0] == 0:
        return xyz

    points_base = transform_points(xyz, t_base_sensor)
    if deskew:
        # A constant inter-sensor clock error cancels in this relative time.  It
        # remains visible as the inter-cloud ICP translation, which is exactly
        # what the speed regression must estimate.
        center_ns = raw.timing.point_mid_ns
        dt = (stamps_ns.astype(np.int64) - center_ns).astype(np.float64) * 1e-9
        velocity = np.asarray(motion.velocity_flu, dtype=np.float64)
        omega = np.asarray(motion.omega_flu, dtype=np.float64)
        points_base = rotate_constant_omega(points_base, omega, dt)
        points_base += dt[:, None] * velocity[None, :]

    z_mask = (points_base[:, 2] >= min_z_m) & (points_base[:, 2] <= max_z_m)
    points_base = voxel_downsample(points_base[z_mask], voxel_m)
    if max_points > 0 and points_base.shape[0] > max_points:
        indices = np.sort(rng.choice(points_base.shape[0], size=max_points, replace=False))
        points_base = points_base[indices]
    return points_base


def crop_to_sensor_fov(
    points_base: np.ndarray,
    t_base_sensor: np.ndarray,
    half_fov_deg: float,
) -> np.ndarray:
    """Keep base-frame points lying inside another sensor's horizontal FOV."""

    if points_base.size == 0 or half_fov_deg <= 0.0:
        return points_base
    points_sensor = transform_points(points_base, np.linalg.inv(t_base_sensor))
    azimuth = np.arctan2(points_sensor[:, 1], points_sensor[:, 0])
    half_fov_rad = math.radians(half_fov_deg)
    visible = (points_sensor[:, 0] > 0.0) & (np.abs(azimuth) <= half_fov_rad)
    return points_base[visible]


def best_fit_transform(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = target_center - rotation @ source_center
    return transform


def rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def run_icp(
    source: np.ndarray,
    target: np.ndarray,
    *,
    max_iterations: int,
    initial_correspondence_m: float,
    final_correspondence_m: float,
    trim_fraction: float,
    min_inliers: int,
    translation_tolerance_m: float = 1e-4,
    rotation_tolerance_deg: float = 0.01,
) -> IcpResult:
    if source.shape[0] < min_inliers or target.shape[0] < min_inliers:
        return IcpResult(
            False,
            np.eye(4).tolist(),
            (0.0, 0.0, 0.0),
            0.0,
            math.inf,
            0,
            0.0,
            0,
            "too_few_filtered_points",
        )

    tree = cKDTree(target)
    transform = np.eye(4, dtype=np.float64)
    iterations_done = 0
    reason = "max_iterations"
    for iteration in range(max_iterations):
        fraction = iteration / max(1, max_iterations - 1)
        max_distance = initial_correspondence_m + fraction * (
            final_correspondence_m - initial_correspondence_m
        )
        moved = transform_points(source, transform)
        distances, indices = tree.query(moved, k=1, workers=-1)
        valid_indices = np.flatnonzero(np.isfinite(distances) & (distances <= max_distance))
        if valid_indices.size < min_inliers:
            reason = f"too_few_correspondences_at_iteration_{iteration}"
            break
        if 0.0 < trim_fraction < 1.0:
            keep = max(min_inliers, int(valid_indices.size * trim_fraction))
            order = np.argpartition(distances[valid_indices], keep - 1)[:keep]
            valid_indices = valid_indices[order]
        delta = best_fit_transform(moved[valid_indices], target[indices[valid_indices]])
        transform = delta @ transform
        iterations_done = iteration + 1
        if (
            np.linalg.norm(delta[:3, 3]) <= translation_tolerance_m
            and rotation_angle_deg(delta[:3, :3]) <= rotation_tolerance_deg
        ):
            reason = "converged"
            break

    moved = transform_points(source, transform)
    distances, _ = tree.query(moved, k=1, workers=-1)
    mask = np.isfinite(distances) & (distances <= final_correspondence_m)
    inliers = int(mask.sum())
    rmse = float(np.sqrt(np.mean(np.square(distances[mask])))) if inliers else math.inf
    translation = tuple(float(value) for value in transform[:3, 3])
    converged = inliers >= min_inliers and math.isfinite(rmse)
    if converged and reason.startswith("too_few_correspondences"):
        reason = "converged_before_tight_gate"
    return IcpResult(
        converged=converged,
        transform=transform.tolist(),
        translation_m=translation,
        rotation_deg=rotation_angle_deg(transform[:3, :3]),
        rmse_m=rmse,
        inliers=inliers,
        inlier_ratio=inliers / max(1, source.shape[0]),
        iterations=iterations_done,
        reason=reason,
    )


def nearest_by(items: Sequence[Any], value: float, key: Any) -> Any | None:
    return min(items, key=lambda item: abs(key(item) - value), default=None)


def collect_window(
    reader: AnyReader,
    connections: Sequence[Any],
    target_ns: int,
    half_window_ns: int,
) -> tuple[dict[str, list[RawCloud]], list[tuple[int, Any]], list[tuple[int, Any]]]:
    clouds: dict[str, list[RawCloud]] = {
        FRONT_TOPIC: [],
        **{topic: [] for topic in DEFAULT_AUX_TOPICS},
    }
    poses: list[tuple[int, Any]] = []
    imus: list[tuple[int, Any]] = []
    for connection, log_time_ns, rawdata in reader.messages(
        connections=connections,
        start=target_ns - half_window_ns,
        stop=target_ns + half_window_ns,
    ):
        message = reader.deserialize(rawdata, connection.msgtype)
        if connection.topic in clouds:
            try:
                timing = cloud_timing(connection.topic, log_time_ns, message)
            except ValueError as exc:
                print(f"warning: skipping {connection.topic} cloud: {exc}", file=sys.stderr)
                continue
            clouds[connection.topic].append(RawCloud(timing, message))
        elif connection.topic == POSE_TOPIC:
            poses.append((int(log_time_ns), message))
        elif connection.topic == IMU_TOPIC:
            imus.append((int(log_time_ns), message))
    return clouds, poses, imus


def select_motion(
    poses: Sequence[tuple[int, Any]],
    imus: Sequence[tuple[int, Any]],
    reference_ns: int,
    max_dt_ms: float,
) -> MotionSample | None:
    pose_item = nearest_by(poses, reference_ns, lambda item: item[0])
    imu_item = nearest_by(imus, reference_ns, lambda item: item[0])
    if pose_item is None or imu_item is None:
        return None
    pose_dt_ms = abs(pose_item[0] - reference_ns) * 1e-6
    imu_dt_ms = abs(imu_item[0] - reference_ns) * 1e-6
    if pose_dt_ms > max_dt_ms or imu_dt_ms > max_dt_ms:
        return None
    pose = pose_item[1]
    imu = imu_item[1]
    return MotionSample(
        velocity_flu=(float(pose.velflu.x), float(pose.velflu.y), float(pose.velflu.z)),
        omega_flu=(
            float(imu.angular_velocity.x),
            float(imu.angular_velocity.y),
            float(imu.angular_velocity.z),
        ),
        pose_dt_ms=pose_dt_ms,
        imu_dt_ms=imu_dt_ms,
    )


def robust_line_fit(x: np.ndarray, y: np.ndarray, iterations: int = 30) -> tuple[float, float]:
    design = np.column_stack((np.ones_like(x), x))
    beta, *_ = np.linalg.lstsq(design, y, rcond=None)
    for _ in range(iterations):
        residual = y - design @ beta
        scale = 1.4826 * float(np.median(np.abs(residual - np.median(residual))))
        if scale < 1e-9:
            break
        cutoff = 1.345 * scale
        absolute = np.abs(residual)
        weights = np.ones_like(absolute)
        high = absolute > cutoff
        weights[high] = cutoff / absolute[high]
        weighted_design = design * np.sqrt(weights)[:, None]
        weighted_y = y * np.sqrt(weights)
        updated, *_ = np.linalg.lstsq(weighted_design, weighted_y, rcond=None)
        if np.linalg.norm(updated - beta) < 1e-10:
            beta = updated
            break
        beta = updated
    return float(beta[0]), float(beta[1])


def regress_clock_offset(
    frames: Sequence[FrameResult],
    *,
    bootstrap_samples: int,
    seed: int,
    min_frames: int,
    min_speed_span_mps: float,
    effect_threshold_ms: float,
    min_abs_correlation: float,
) -> dict[str, Any]:
    if not frames:
        return {"decision": "INCONCLUSIVE", "reason": "no_accepted_icp_frames", "n": 0}
    speed = np.asarray([frame.speed_mps for frame in frames], dtype=np.float64)
    along = np.asarray([frame.along_track_correction_m for frame in frames], dtype=np.float64)
    speed_span = float(np.ptp(speed))
    if len(frames) < min_frames:
        return {
            "decision": "INCONCLUSIVE",
            "reason": f"need_at_least_{min_frames}_frames",
            "n": len(frames),
            "speed_span_mps": speed_span,
        }
    if speed_span < min_speed_span_mps:
        return {
            "decision": "INCONCLUSIVE",
            "reason": f"speed_span_below_{min_speed_span_mps:g}_mps",
            "n": len(frames),
            "speed_span_mps": speed_span,
        }

    intercept, slope_s = robust_line_fit(speed, along)
    prediction = intercept + slope_s * speed
    residual = along - prediction
    total = float(np.sum(np.square(along - along.mean())))
    r_squared = 1.0 - float(np.sum(np.square(residual))) / total if total > 0.0 else 0.0
    correlation = float(pearsonr(speed, along).statistic) if speed_span > 0.0 else 0.0

    rng = np.random.default_rng(seed)
    slopes: list[float] = []
    indices = np.arange(len(frames))
    for _ in range(bootstrap_samples):
        sample = rng.choice(indices, size=len(indices), replace=True)
        if float(np.ptp(speed[sample])) < min_speed_span_mps * 0.5:
            continue
        _, boot_slope = robust_line_fit(speed[sample], along[sample])
        if math.isfinite(boot_slope):
            slopes.append(boot_slope)
    if len(slopes) < max(100, bootstrap_samples // 4):
        return {
            "decision": "INCONCLUSIVE",
            "reason": "bootstrap_failed_to_cover_speed_range",
            "n": len(frames),
            "speed_span_mps": speed_span,
            "offset_ms": slope_s * 1e3,
        }

    ci_low_s, ci_high_s = np.quantile(slopes, [0.025, 0.975])
    threshold_s = effect_threshold_ms * 1e-3
    outside_equivalence = ci_low_s > threshold_s or ci_high_s < -threshold_s
    inside_equivalence = ci_low_s >= -threshold_s and ci_high_s <= threshold_s

    if outside_equivalence and abs(correlation) >= min_abs_correlation:
        decision = "YES"
        reason = "nonzero_speed_scaling_exceeds_practical_threshold"
    elif inside_equivalence:
        decision = "NO"
        reason = "95pct_ci_is_inside_practical_equivalence_band"
    else:
        decision = "INCONCLUSIVE"
        reason = "confidence_interval_or_speed_correlation_is_not_decisive"

    recommended_offset_s: float | None
    if decision == "YES":
        recommended_offset_s = slope_s
    elif decision == "NO":
        recommended_offset_s = 0.0
    else:
        recommended_offset_s = None

    return {
        "decision": decision,
        "reason": reason,
        "n": len(frames),
        "speed_min_mps": float(speed.min()),
        "speed_max_mps": float(speed.max()),
        "speed_span_mps": speed_span,
        "intercept_m": intercept,
        "offset_ms": slope_s * 1e3,
        "offset_95pct_ci_ms": [float(ci_low_s * 1e3), float(ci_high_s * 1e3)],
        "pearson_r": correlation,
        "r_squared": r_squared,
        "residual_rmse_m": float(np.sqrt(np.mean(np.square(residual)))),
        "effect_threshold_ms": effect_threshold_ms,
        "estimated_aux_time_offset_s": slope_s,
        "recommended_aux_time_offset_s": recommended_offset_s,
    }


def finite_summary(values: Iterable[float]) -> dict[str, float] | None:
    array = np.asarray([value for value in values if math.isfinite(value)], dtype=np.float64)
    if array.size == 0:
        return None
    quantiles = np.quantile(array, [0.05, 0.5, 0.95])
    return {
        "count": int(array.size),
        "p05": float(quantiles[0]),
        "p50": float(quantiles[1]),
        "p95": float(quantiles[2]),
        "min": float(array.min()),
        "max": float(array.max()),
    }


def aggregate_timing(frames: Sequence[FrameResult | TimingObservation]) -> dict[str, Any]:
    return {
        "front_scan_span_ms": finite_summary(frame.front_span_ms for frame in frames),
        "aux_scan_span_ms": finite_summary(frame.aux_span_ms for frame in frames),
        "selected_header_delta_ms": finite_summary(
            frame.selected_header_delta_ms for frame in frames
        ),
        "selected_point_mid_delta_ms": finite_summary(
            frame.selected_point_mid_delta_ms for frame in frames
        ),
        "header_nearest_point_mid_delta_ms": finite_summary(
            frame.header_nearest_point_mid_delta_ms for frame in frames
        ),
    }


def classify_large_offset(regression: dict[str, Any], threshold_ms: float) -> dict[str, str]:
    """Classify the claimed large-offset H1 from a geometric confidence interval."""

    interval = regression.get("offset_95pct_ci_ms")
    if interval is None:
        return {
            "decision": "INCONCLUSIVE",
            "reason": "no_valid_geometric_offset_confidence_interval",
        }
    ci_low_ms, ci_high_ms = (float(interval[0]), float(interval[1]))
    if ci_low_ms > threshold_ms or ci_high_ms < -threshold_ms:
        return {
            "decision": "YES",
            "reason": "95pct_ci_is_entirely_outside_large_offset_boundary",
        }
    if ci_low_ms >= -threshold_ms and ci_high_ms <= threshold_ms:
        return {
            "decision": "NO",
            "reason": "95pct_ci_excludes_a_large_clock_offset",
        }
    return {
        "decision": "INCONCLUSIVE",
        "reason": "95pct_ci_crosses_large_offset_boundary",
    }


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Use pairwise Luminar ICP residual-vs-speed regression to decide whether "
            "aux_point_time_offsets should be nonzero."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("bag", type=Path, help="rosbag2 directory or one MCAP file")
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "av24.urdf",
        help="URDF used as the LiDAR extrinsic source of truth",
    )
    parser.add_argument("--base-frame", default="rear_axle_middle")
    parser.add_argument("--output", type=Path, default=Path("luminar_clock_hypothesis.json"))
    parser.add_argument("--start-offset", type=float, default=15.0, help="seconds after bag start")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to inspect; 0 means to bag end")
    parser.add_argument("--sample-count", type=int, default=60, help="target accepted front frames")
    parser.add_argument(
        "--probe-count",
        type=int,
        default=0,
        help="uniform time probes; 0 chooses 3 x sample-count",
    )
    parser.add_argument("--window", type=float, default=0.22, help="half-width of each indexed read, seconds")
    parser.add_argument("--max-motion-dt-ms", type=float, default=30.0)
    parser.add_argument("--max-point-pair-dt-ms", type=float, default=10.0)
    parser.add_argument("--min-scan-span-ms", type=float, default=30.0)
    parser.add_argument("--max-scan-span-ms", type=float, default=70.0)
    parser.add_argument("--max-yaw-rate", type=float, default=0.35, help="rad/s; reduces turn/extrinsic coupling")
    parser.add_argument(
        "--min-forward-fraction",
        type=float,
        default=0.8,
        help="minimum |forward velocity| / planar speed for moving frames",
    )
    parser.add_argument("--no-deskew", action="store_true", help="disable constant-twist scan deskew")
    parser.add_argument("--min-range", type=float, default=2.0)
    parser.add_argument("--max-range", type=float, default=60.0)
    parser.add_argument(
        "--min-z",
        type=float,
        default=-1.0,
        help="base-frame crop; retains the narrow front/side overlap near the road",
    )
    parser.add_argument("--max-z", type=float, default=4.0)
    parser.add_argument("--voxel", type=float, default=0.25)
    parser.add_argument("--max-points", type=int, default=7000)
    parser.add_argument("--icp-iterations", type=int, default=35)
    parser.add_argument("--initial-correspondence", type=float, default=3.0)
    parser.add_argument("--final-correspondence", type=float, default=0.60)
    parser.add_argument("--trim-fraction", type=float, default=0.75)
    parser.add_argument(
        "--sensor-half-fov-deg",
        type=float,
        default=63.0,
        help="crop both clouds to their common horizontal FOV before ICP",
    )
    parser.add_argument("--min-icp-inliers", type=int, default=80)
    parser.add_argument("--min-icp-inlier-ratio", type=float, default=0.08)
    parser.add_argument("--max-icp-rmse", type=float, default=0.45)
    parser.add_argument("--max-icp-rotation-deg", type=float, default=8.0)
    parser.add_argument("--max-icp-translation", type=float, default=3.0)
    parser.add_argument(
        "--max-icp-cycle-translation",
        type=float,
        default=0.35,
        help="maximum disagreement between forward and inverse reverse ICP, metres",
    )
    parser.add_argument("--max-icp-cycle-rotation-deg", type=float, default=2.0)
    parser.add_argument("--min-regression-frames", type=int, default=15)
    parser.add_argument("--min-speed-span", type=float, default=5.0, help="m/s")
    parser.add_argument(
        "--effect-threshold-ms",
        type=float,
        default=10.0,
        help="fine-grained equivalence boundary for recommending aux_point_time_offsets",
    )
    parser.add_argument(
        "--h1-min-offset-ms",
        type=float,
        default=50.0,
        help=(
            "minimum residual offset counted as the large H1 clock error; 50 ms "
            "conservatively covers the claimed 80-90 ms effect"
        ),
    )
    parser.add_argument("--min-abs-correlation", type=float, default=0.35)
    parser.add_argument("--bootstrap-samples", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=24)
    parser.add_argument("--verbose", action="store_true")
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if not args.bag.exists():
        raise ValueError(f"bag does not exist: {args.bag}")
    if not args.urdf.exists():
        raise ValueError(f"URDF does not exist: {args.urdf}")
    if args.sample_count < 1:
        raise ValueError("--sample-count must be positive")
    if args.window <= 0.0:
        raise ValueError("--window must be positive")
    if not (0.0 < args.trim_fraction <= 1.0):
        raise ValueError("--trim-fraction must be in (0, 1]")
    if args.initial_correspondence < args.final_correspondence:
        raise ValueError("--initial-correspondence must be >= --final-correspondence")
    if not (0.0 < args.sensor_half_fov_deg < 90.0):
        raise ValueError("--sensor-half-fov-deg must be in (0, 90)")
    if args.max_icp_cycle_translation <= 0.0 or args.max_icp_cycle_rotation_deg <= 0.0:
        raise ValueError("ICP cycle-consistency limits must be positive")
    if args.bootstrap_samples < 100:
        raise ValueError("--bootstrap-samples must be at least 100")
    if args.h1_min_offset_ms <= 0.0:
        raise ValueError("--h1-min-offset-ms must be positive")


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    validate_args(args)
    all_topics = (FRONT_TOPIC, *DEFAULT_AUX_TOPICS, POSE_TOPIC, IMU_TOPIC)
    frame_names = tuple(topic.split("/")[1] for topic in (FRONT_TOPIC, *DEFAULT_AUX_TOPICS))
    extrinsics = load_urdf_extrinsics(args.urdf, args.base_frame, frame_names)
    results: dict[str, list[FrameResult]] = {topic: [] for topic in DEFAULT_AUX_TOPICS}
    timing_observations: dict[str, list[TimingObservation]] = {
        topic: [] for topic in DEFAULT_AUX_TOPICS
    }
    rejection_counts: dict[str, int] = {}

    def reject(reason: str) -> None:
        rejection_counts[reason] = rejection_counts.get(reason, 0) + 1

    with AnyReader([args.bag]) as reader:
        topic_connections = [connection for connection in reader.connections if connection.topic in all_topics]
        present = {connection.topic for connection in topic_connections}
        missing = set(all_topics) - present
        if missing:
            raise ValueError(f"bag is missing required topics: {sorted(missing)}")

        start_ns = int(reader.start_time + args.start_offset * 1e9)
        stop_ns = int(reader.end_time)
        if args.duration > 0.0:
            stop_ns = min(stop_ns, int(start_ns + args.duration * 1e9))
        margin_ns = int((args.window + 0.01) * 1e9)
        start_ns += margin_ns
        stop_ns -= margin_ns
        if stop_ns <= start_ns:
            raise ValueError("selected time range is shorter than two read windows")

        probe_count = args.probe_count if args.probe_count > 0 else args.sample_count * 3
        probe_count = max(args.sample_count, probe_count)
        targets = np.linspace(start_ns, stop_ns, probe_count, dtype=np.int64)
        seen_front_headers: set[int] = set()
        rng = np.random.default_rng(args.seed)

        for probe_index, target_ns_value in enumerate(targets, start=1):
            if all(len(items) >= args.sample_count for items in results.values()):
                break
            target_ns = int(target_ns_value)
            clouds, poses, imus = collect_window(
                reader, topic_connections, target_ns, int(args.window * 1e9)
            )
            front = nearest_by(
                clouds[FRONT_TOPIC], target_ns, lambda item: item.timing.header_time_ns
            )
            if front is None:
                reject("no_front_cloud")
                continue
            if front.timing.header_time_ns in seen_front_headers:
                reject("duplicate_front_cloud")
                continue
            if not (args.min_scan_span_ms <= front.timing.span_ms <= args.max_scan_span_ms):
                reject("front_scan_span_out_of_range")
                continue
            motion = select_motion(
                poses, imus, front.timing.header_time_ns, args.max_motion_dt_ms
            )
            if motion is None:
                reject("no_nearby_motion_sample")
                continue
            speed = motion.speed_mps
            if speed > 0.5 and abs(motion.velocity_flu[0]) / speed < args.min_forward_fraction:
                reject("motion_not_predominantly_forward")
                continue
            if abs(motion.omega_flu[2]) > args.max_yaw_rate:
                reject("yaw_rate_too_high")
                continue

            front_points = prepare_cloud(
                front,
                extrinsics["luminar_front"],
                motion,
                deskew=not args.no_deskew,
                min_range_m=args.min_range,
                max_range_m=args.max_range,
                min_z_m=args.min_z,
                max_z_m=args.max_z,
                voxel_m=args.voxel,
                max_points=args.max_points,
                rng=rng,
            )
            if front_points.shape[0] < args.min_icp_inliers:
                reject("front_has_too_few_points")
                continue
            seen_front_headers.add(front.timing.header_time_ns)

            for aux_topic in DEFAULT_AUX_TOPICS:
                if len(results[aux_topic]) >= args.sample_count:
                    continue
                aux_candidates = clouds[aux_topic]
                aux = nearest_by(
                    aux_candidates,
                    front.timing.point_mid_ns,
                    lambda item: item.timing.point_mid_ns,
                )
                header_nearest = nearest_by(
                    aux_candidates,
                    front.timing.header_time_ns,
                    lambda item: item.timing.header_time_ns,
                )
                if aux is None or header_nearest is None:
                    reject(f"{aux_topic}:no_aux_cloud")
                    continue
                point_pair_delta_ms = (
                    aux.timing.point_mid_ns - front.timing.point_mid_ns
                ) * 1e-6
                if abs(point_pair_delta_ms) > args.max_point_pair_dt_ms:
                    reject(f"{aux_topic}:point_time_pair_too_far")
                    continue
                if not (args.min_scan_span_ms <= aux.timing.span_ms <= args.max_scan_span_ms):
                    reject(f"{aux_topic}:scan_span_out_of_range")
                    continue

                timing_observations[aux_topic].append(
                    TimingObservation(
                        front_span_ms=front.timing.span_ms,
                        aux_span_ms=aux.timing.span_ms,
                        selected_header_delta_ms=(
                            aux.timing.header_time_ns - front.timing.header_time_ns
                        )
                        * 1e-6,
                        selected_point_mid_delta_ms=point_pair_delta_ms,
                        header_nearest_point_mid_delta_ms=(
                            header_nearest.timing.point_mid_ns - front.timing.point_mid_ns
                        )
                        * 1e-6,
                    )
                )

                frame_name = aux_topic.split("/")[1]
                aux_points = prepare_cloud(
                    aux,
                    extrinsics[frame_name],
                    motion,
                    deskew=not args.no_deskew,
                    min_range_m=args.min_range,
                    max_range_m=args.max_range,
                    min_z_m=args.min_z,
                    max_z_m=args.max_z,
                    voxel_m=args.voxel,
                    max_points=args.max_points,
                    rng=rng,
                )
                # The front and side Iris units only overlap in a narrow edge
                # sector.  Whole-cloud ICP happily aligns unrelated walls and
                # creates metre-scale false clock offsets, so enforce mutual
                # sensor visibility before asking geometry for a verdict.
                front_overlap = crop_to_sensor_fov(
                    front_points, extrinsics[frame_name], args.sensor_half_fov_deg
                )
                aux_overlap = crop_to_sensor_fov(
                    aux_points, extrinsics["luminar_front"], args.sensor_half_fov_deg
                )
                forward_icp = run_icp(
                    aux_overlap,
                    front_overlap,
                    max_iterations=args.icp_iterations,
                    initial_correspondence_m=args.initial_correspondence,
                    final_correspondence_m=args.final_correspondence,
                    trim_fraction=args.trim_fraction,
                    min_inliers=args.min_icp_inliers,
                )
                reverse_icp = run_icp(
                    front_overlap,
                    aux_overlap,
                    max_iterations=args.icp_iterations,
                    initial_correspondence_m=args.initial_correspondence,
                    final_correspondence_m=args.final_correspondence,
                    trim_fraction=args.trim_fraction,
                    min_inliers=args.min_icp_inliers,
                )
                if not forward_icp.converged:
                    reject(f"{aux_topic}:forward_icp_{forward_icp.reason}")
                    continue
                if not reverse_icp.converged:
                    reject(f"{aux_topic}:reverse_icp_{reverse_icp.reason}")
                    continue

                forward_transform = np.asarray(forward_icp.transform, dtype=np.float64)
                inverse_reverse_transform = np.linalg.inv(
                    np.asarray(reverse_icp.transform, dtype=np.float64)
                )
                cycle_translation_m = float(
                    np.linalg.norm(
                        forward_transform[:3, 3] - inverse_reverse_transform[:3, 3]
                    )
                )
                cycle_rotation_deg = rotation_angle_deg(
                    forward_transform[:3, :3] @ inverse_reverse_transform[:3, :3].T
                )
                if cycle_translation_m > args.max_icp_cycle_translation:
                    reject(f"{aux_topic}:icp_cycle_translation_large")
                    continue
                if cycle_rotation_deg > args.max_icp_cycle_rotation_deg:
                    reject(f"{aux_topic}:icp_cycle_rotation_large")
                    continue

                inlier_ratio = min(forward_icp.inlier_ratio, reverse_icp.inlier_ratio)
                rmse_m = max(forward_icp.rmse_m, reverse_icp.rmse_m)
                rotation_deg = max(forward_icp.rotation_deg, reverse_icp.rotation_deg)
                inliers = min(forward_icp.inliers, reverse_icp.inliers)
                if inlier_ratio < args.min_icp_inlier_ratio:
                    reject(f"{aux_topic}:icp_inlier_ratio_low")
                    continue
                if rmse_m > args.max_icp_rmse:
                    reject(f"{aux_topic}:icp_rmse_high")
                    continue
                # Average the two independently estimated aux->front corrections.
                # The cycle gate above makes this meaningful and rejects cases
                # where the narrow overlap cannot constrain one direction.
                translation = 0.5 * (
                    forward_transform[:3, 3] + inverse_reverse_transform[:3, 3]
                )
                if np.linalg.norm(translation) > args.max_icp_translation:
                    reject(f"{aux_topic}:icp_translation_large")
                    continue
                if rotation_deg > args.max_icp_rotation_deg:
                    reject(f"{aux_topic}:icp_rotation_large")
                    continue

                velocity = np.asarray(motion.velocity_flu, dtype=np.float64)
                planar_speed = float(np.linalg.norm(velocity[:2]))
                if planar_speed > 0.25:
                    along_axis = np.array(
                        [velocity[0] / planar_speed, velocity[1] / planar_speed, 0.0]
                    )
                else:
                    along_axis = np.array([1.0, 0.0, 0.0])
                lateral_axis = np.array([-along_axis[1], along_axis[0], 0.0])
                frame_result = FrameResult(
                    aux_topic=aux_topic,
                    front_header_s=front.timing.header_time_ns * 1e-9,
                    speed_mps=planar_speed,
                    velocity_flu=motion.velocity_flu,
                    omega_flu=motion.omega_flu,
                    front_span_ms=front.timing.span_ms,
                    aux_span_ms=aux.timing.span_ms,
                    selected_header_delta_ms=(
                        aux.timing.header_time_ns - front.timing.header_time_ns
                    )
                    * 1e-6,
                    selected_point_mid_delta_ms=point_pair_delta_ms,
                    header_nearest_point_mid_delta_ms=(
                        header_nearest.timing.point_mid_ns - front.timing.point_mid_ns
                    )
                    * 1e-6,
                    along_track_correction_m=float(translation @ along_axis),
                    lateral_correction_m=float(translation @ lateral_axis),
                    vertical_correction_m=float(translation[2]),
                    icp_rmse_m=rmse_m,
                    icp_inliers=inliers,
                    icp_inlier_ratio=inlier_ratio,
                    icp_rotation_deg=rotation_deg,
                    front_points_used=int(front_overlap.shape[0]),
                    aux_points_used=int(aux_overlap.shape[0]),
                )
                results[aux_topic].append(frame_result)
                if args.verbose:
                    print(
                        f"[{probe_index:03d}/{probe_count}] {aux_topic}: "
                        f"v={planar_speed:5.2f} m/s along={frame_result.along_track_correction_m:+.3f} m "
                        f"header_dt={frame_result.selected_header_delta_ms:+.1f} ms "
                        f"point_dt={point_pair_delta_ms:+.2f} ms rmse={rmse_m:.3f} m "
                        f"cycle={cycle_translation_m:.2f} m/{cycle_rotation_deg:.2f} deg"
                    )

    regressions: dict[str, dict[str, Any]] = {}
    timing_audit: dict[str, dict[str, Any]] = {}
    for index, aux_topic in enumerate(DEFAULT_AUX_TOPICS):
        regressions[aux_topic] = regress_clock_offset(
            results[aux_topic],
            bootstrap_samples=args.bootstrap_samples,
            seed=args.seed + index + 1,
            min_frames=args.min_regression_frames,
            min_speed_span_mps=args.min_speed_span,
            effect_threshold_ms=args.effect_threshold_ms,
            min_abs_correlation=args.min_abs_correlation,
        )
        timing_audit[aux_topic] = aggregate_timing(timing_observations[aux_topic])

    h1_by_aux: dict[str, dict[str, str]] = {}
    for aux_topic in DEFAULT_AUX_TOPICS:
        h1_by_aux[aux_topic] = classify_large_offset(
            regressions[aux_topic], args.h1_min_offset_ms
        )

    h1_decisions = [h1_by_aux[topic]["decision"] for topic in DEFAULT_AUX_TOPICS]
    if "YES" in h1_decisions:
        overall = "YES"
        overall_reason = "at_least_one_aux_has_the_claimed_large_speed_scaled_clock_error"
    elif h1_decisions and all(decision == "NO" for decision in h1_decisions):
        overall = "NO"
        overall_reason = "both_aux_confidence_intervals_exclude_the_claimed_large_clock_error"
    else:
        overall = "INCONCLUSIVE"
        overall_reason = "one_or_more_aux_intervals_do_not_decide_the_large_offset_hypothesis"

    return {
        "schema_version": 2,
        "hypothesis": (
            "H1: at least one auxiliary Luminar has a residual clock disagreement "
            f"of at least {args.h1_min_offset_ms:g} ms versus front"
        ),
        "decision": overall,
        "reason": overall_reason,
        "decision_semantics": {
            "YES": "geometry supports the claimed large speed-scaled clock disagreement",
            "NO": "both 95% intervals exclude the claimed large clock disagreement",
            "INCONCLUSIVE": "do not force a binary answer; collect more/better speed-diverse overlap frames",
        },
        "bag": str(args.bag.resolve()),
        "urdf": str(args.urdf.resolve()),
        "base_frame": args.base_frame,
        "deskew_enabled": not args.no_deskew,
        "h1_test_by_aux": h1_by_aux,
        "regression": regressions,
        "timestamp_audit": timing_audit,
        "accepted_frame_count": {topic: len(results[topic]) for topic in DEFAULT_AUX_TOPICS},
        "rejection_counts": dict(sorted(rejection_counts.items())),
        "frames": {
            topic: [asdict(frame) for frame in results[topic]] for topic in DEFAULT_AUX_TOPICS
        },
        "parameters": {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        },
    }


def print_report(report: dict[str, Any]) -> None:
    print()
    print(f"H1 AUX CLOCK DISAGREEMENT: {report['decision']}")
    print(f"reason: {report['reason']}")
    for topic in DEFAULT_AUX_TOPICS:
        regression = report["regression"][topic]
        h1_test = report["h1_test_by_aux"][topic]
        print()
        print(
            f"{topic}: large-H1={h1_test['decision']} ({h1_test['reason']}), "
            f"fine-offset={regression['decision']} ({regression['reason']})"
        )
        if "offset_ms" in regression:
            ci = regression.get("offset_95pct_ci_ms")
            ci_text = f" [{ci[0]:+.2f}, {ci[1]:+.2f}]" if ci else ""
            print(
                f"  estimated aux point-clock offset = {regression['offset_ms']:+.2f} ms; "
                f"95% CI{ci_text} ms"
            )
            if "intercept_m" in regression:
                print(
                    f"  intercept={regression['intercept_m']:+.3f} m, "
                    f"Pearson r={regression['pearson_r']:+.3f}, "
                    f"R^2={regression['r_squared']:+.3f}, n={regression['n']}"
                )
            if regression.get("recommended_aux_time_offset_s") is None:
                print("  configuration recommendation: none (fine-offset test is inconclusive)")
            else:
                print(
                    "  configuration recommendation: aux_point_time_offset="
                    f"{regression['recommended_aux_time_offset_s']:+.6f} s"
                )
        else:
            print(f"  accepted frames={regression.get('n', 0)}")
        audit = report["timestamp_audit"][topic]
        selected_header = audit.get("selected_header_delta_ms")
        selected_point = audit.get("selected_point_mid_delta_ms")
        header_nearest_point = audit.get("header_nearest_point_mid_delta_ms")
        if selected_header and selected_point and header_nearest_point:
            print(
                "  timing medians: selected header delta="
                f"{selected_header['p50']:+.2f} ms, selected point-mid delta="
                f"{selected_point['p50']:+.2f} ms, header-nearest point-mid delta="
                f"{header_nearest_point['p50']:+.2f} ms"
            )
    print()
    print(f"JSON report: {report['parameters']['output']}")


def main() -> int:
    parser = make_parser()
    args = parser.parse_args()
    try:
        report = analyze(args)
    except (OSError, RuntimeError, ValueError) as exc:
        parser.error(str(exc))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print_report(report)
    return 0 if report["decision"] in {"YES", "NO"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
