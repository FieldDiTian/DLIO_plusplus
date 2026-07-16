#!/usr/bin/env python3
"""Fail-closed Luminar vertical-FOV audit and AV-24 live launch preflight."""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np


FLOAT32 = 7
FLOAT64 = 8
DEFAULT_TOPICS = (
    "/luminar_front/points",
    "/luminar_left/points",
    "/luminar_right/points",
)


def _field_map(msg: Any) -> dict[str, Any]:
    return {field.name: field for field in msg.fields if field.count > 0}


def _field_array(msg: Any, field: Any) -> np.ndarray:
    datatype = int(field.datatype)
    if datatype not in (FLOAT32, FLOAT64):
        raise ValueError(f"field {field.name!r} must be FLOAT32 or FLOAT64")
    endian = ">" if bool(msg.is_bigendian) else "<"
    dtype = np.dtype(endian + ("f4" if datatype == FLOAT32 else "f8"))
    if int(field.offset) + dtype.itemsize > int(msg.point_step):
        raise ValueError(f"field {field.name!r} does not fit within point_step")
    return np.ndarray(
        (int(msg.height), int(msg.width)),
        dtype=dtype,
        buffer=memoryview(msg.data),
        offset=int(field.offset),
        strides=(int(msg.row_step), int(msg.point_step)),
    )


def measure_vertical_fov(
    msg: Any,
    *,
    elevation_field: str = "elevation",
    elevation_in_radians: bool = True,
    min_rays: int = 1000,
    require_elevation_field: bool = True,
) -> dict[str, Any]:
    """Measure commanded scan coverage; XYZ returns are only an opt-in fallback."""
    result: dict[str, Any] = {
        "valid": False,
        "source": None,
        "ray_count": 0,
        "min_elevation_deg": None,
        "max_elevation_deg": None,
        "vertical_fov_deg": None,
        "error": None,
    }
    if int(msg.width) <= 0 or int(msg.height) <= 0 or int(msg.point_step) <= 0:
        result["error"] = "empty PointCloud2 or zero point_step"
        return result
    if int(msg.row_step) < int(msg.width) * int(msg.point_step):
        result["error"] = "PointCloud2 row_step is shorter than width * point_step"
        return result
    required_bytes = int(msg.row_step) * int(msg.height)
    if memoryview(msg.data).nbytes < required_bytes:
        result["error"] = "PointCloud2 data is shorter than row_step * height"
        return result

    fields = _field_map(msg)
    try:
        if elevation_field in fields:
            values = _field_array(msg, fields[elevation_field]).astype(np.float64, copy=False)
            if elevation_in_radians:
                values = np.degrees(values)
            result["source"] = elevation_field
        else:
            if require_elevation_field:
                result["error"] = f"required elevation field {elevation_field!r} is missing"
                return result
            if not all(name in fields for name in ("x", "y", "z")):
                result["error"] = "neither elevation nor complete x/y/z fields are available"
                return result
            x = _field_array(msg, fields["x"]).astype(np.float64, copy=False)
            y = _field_array(msg, fields["y"]).astype(np.float64, copy=False)
            z = _field_array(msg, fields["z"]).astype(np.float64, copy=False)
            horizontal = np.hypot(x, y)
            nonzero = (horizontal >= 0.1) | (np.abs(z) >= 0.1)
            values = np.degrees(np.arctan2(z[nonzero], horizontal[nonzero]))
            result["source"] = "xyz_fallback"
    except (TypeError, ValueError) as exc:
        result["error"] = str(exc)
        return result

    values = np.asarray(values).reshape(-1)
    values = values[np.isfinite(values)]
    result["ray_count"] = int(values.size)
    if values.size < min_rays:
        result["error"] = f"only {values.size} usable rays; require at least {min_rays}"
        return result

    minimum = float(np.min(values))
    maximum = float(np.max(values))
    result.update(
        valid=True,
        min_elevation_deg=minimum,
        max_elevation_deg=maximum,
        vertical_fov_deg=maximum - minimum,
    )
    return result


def _frame_passes(frame: dict[str, Any], minimum_fov_deg: float) -> bool:
    return bool(frame["valid"] and frame["vertical_fov_deg"] + 1e-6 >= minimum_fov_deg)


def _summarize(
    mode: str,
    observations: dict[str, list[dict[str, Any]]],
    minimum_fov_deg: float,
    requested_samples: int,
) -> dict[str, Any]:
    topic_results: dict[str, Any] = {}
    overall_pass = True
    for topic, frames in observations.items():
        spans = [float(frame["vertical_fov_deg"]) for frame in frames if frame["valid"]]
        frame_passes = [_frame_passes(frame, minimum_fov_deg) for frame in frames]
        topic_pass = len(frames) >= requested_samples and bool(frame_passes) and all(frame_passes)
        overall_pass = overall_pass and topic_pass
        topic_results[topic] = {
            "pass": topic_pass,
            "frames_checked": len(frames),
            "frames_requested": requested_samples,
            "passing_frames": sum(frame_passes),
            "minimum_vertical_fov_deg": min(spans) if spans else None,
            "median_vertical_fov_deg": statistics.median(spans) if spans else None,
            "maximum_vertical_fov_deg": max(spans) if spans else None,
            "minimum_elevation_deg": min(
                (frame["min_elevation_deg"] for frame in frames if frame["valid"]),
                default=None,
            ),
            "maximum_elevation_deg": max(
                (frame["max_elevation_deg"] for frame in frames if frame["valid"]),
                default=None,
            ),
            "errors": sorted({frame["error"] for frame in frames if frame["error"]}),
            "measurement_source": sorted(
                {frame["source"] for frame in frames if frame["source"]}
            ),
        }
    return {
        "schema_version": "dlio.lidar_fov_quality.v1",
        "mode": mode,
        "generated_at_unix": time.time(),
        "minimum_required_vertical_fov_deg": minimum_fov_deg,
        "pass": overall_pass,
        "topics": topic_results,
    }


def audit_bag(args: argparse.Namespace) -> dict[str, Any]:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:
        raise RuntimeError("bag mode requires the Python 'rosbags' package") from exc

    bag = Path(args.bag).expanduser().resolve()
    if not bag.exists():
        raise RuntimeError(f"rosbag path does not exist: {bag}")

    observations = {topic: [] for topic in args.topics}
    with AnyReader([bag]) as reader:
        connections = [connection for connection in reader.connections if connection.topic in observations]
        found = {connection.topic for connection in connections}
        missing = sorted(set(args.topics) - found)
        if missing:
            raise RuntimeError(f"missing required PointCloud2 topics: {', '.join(missing)}")

        window_count = max(1, min(args.windows, args.samples))
        per_window = int(math.ceil(args.samples / window_count))
        duration = max(1, reader.end_time - reader.start_time)
        for index in range(window_count):
            start = reader.start_time + (duration * index) // window_count
            stop = reader.start_time + (duration * (index + 1)) // window_count
            window_seen = {topic: 0 for topic in args.topics}
            for connection, _, raw in reader.messages(connections=connections, start=start, stop=stop):
                topic = connection.topic
                if len(observations[topic]) >= args.samples or window_seen[topic] >= per_window:
                    continue
                msg = reader.deserialize(raw, connection.msgtype)
                observations[topic].append(
                    measure_vertical_fov(
                        msg,
                        elevation_field=args.elevation_field,
                        elevation_in_radians=not args.elevation_degrees,
                        min_rays=args.min_rays,
                        require_elevation_field=not args.allow_xyz_fallback,
                    )
                )
                window_seen[topic] += 1
                if all(
                    len(observations[topic]) >= args.samples or window_seen[topic] >= per_window
                    for topic in args.topics
                ):
                    break

    report = _summarize("bag", observations, args.min_fov_deg, args.samples)
    report["bag"] = str(bag)
    report["sampling_windows"] = min(args.windows, args.samples)
    return report


def audit_live(args: argparse.Namespace) -> dict[str, Any]:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    observations = {topic: [] for topic in args.topics}
    failed = False

    class FovPreflight(Node):
        def __init__(self) -> None:
            super().__init__("av24_lidar_fov_quality_check")
            self._cloud_subscriptions = []
            for topic in args.topics:
                self._cloud_subscriptions.append(
                    self.create_subscription(
                        PointCloud2,
                        topic,
                        lambda msg, topic=topic: self.on_cloud(topic, msg),
                        qos_profile_sensor_data,
                    )
                )

        def on_cloud(self, topic: str, msg: Any) -> None:
            nonlocal failed
            if len(observations[topic]) >= args.samples:
                return
            frame = measure_vertical_fov(
                msg,
                elevation_field=args.elevation_field,
                elevation_in_radians=not args.elevation_degrees,
                min_rays=args.min_rays,
                require_elevation_field=not args.allow_xyz_fallback,
            )
            observations[topic].append(frame)
            if not _frame_passes(frame, args.min_fov_deg):
                failed = True
                self.get_logger().fatal(
                    f"LiDAR quality failed on {topic}: {frame}; "
                    f"required vertical FOV >= {args.min_fov_deg:.3f} deg"
                )

    rclpy.init(args=args.ros_args)
    node = FovPreflight()
    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and not failed and time.monotonic() < deadline:
            if all(len(frames) >= args.samples for frames in observations.values()):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    report = _summarize("live", observations, args.min_fov_deg, args.samples)
    report["timeout_sec"] = args.timeout_sec
    if not all(len(frames) >= args.samples for frames in observations.values()):
        report["pass"] = False
        report["error"] = "timed out or failed before all required topic samples arrived"
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Audit Luminar commanded-ray elevation coverage. In live mode this is the "
            "AV-24 fail-closed preflight; --exec starts a launch command only after PASS."
        )
    )
    parser.add_argument("--bag", help="rosbag directory or MCAP file; omit for live mode")
    parser.add_argument("--topics", nargs="+", default=list(DEFAULT_TOPICS))
    parser.add_argument("--min-fov-deg", type=float, default=30.0)
    parser.add_argument("--min-rays", type=int, default=1000)
    parser.add_argument("--samples", type=int, default=3, help="required frames per topic")
    parser.add_argument("--windows", type=int, default=10, help="time-distributed bag windows")
    parser.add_argument("--timeout-sec", type=float, default=15.0, help="live-mode timeout")
    parser.add_argument("--elevation-field", default="elevation")
    parser.add_argument("--elevation-degrees", action="store_true")
    parser.add_argument(
        "--allow-xyz-fallback",
        action="store_true",
        help="permit return-only XYZ inference when elevation is missing (not recommended for Luminar)",
    )
    parser.add_argument("--report-json", help="write the machine-readable report here")
    parser.add_argument(
        "--exec",
        dest="launch_command",
        nargs=argparse.REMAINDER,
        help="live mode only: exec this command after the preflight passes",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args, ros_args = parser.parse_known_args()
    args.ros_args = ros_args
    if args.samples < 1 or args.min_rays < 1 or args.min_fov_deg <= 0.0:
        parser.error("--samples/--min-rays must be positive and --min-fov-deg must be > 0")
    if args.bag and args.launch_command:
        parser.error("--exec is available only in live mode")

    try:
        report = audit_bag(args) if args.bag else audit_live(args)
    except Exception as exc:  # boundary: emit a durable fail-closed report
        report = {
            "schema_version": "dlio.lidar_fov_quality.v1",
            "mode": "bag" if args.bag else "live",
            "pass": False,
            "error": str(exc),
        }
        exit_code = 3
    else:
        exit_code = 0 if report["pass"] else 2

    rendered = json.dumps(report, indent=2, sort_keys=True, allow_nan=False)
    print(rendered)
    if args.report_json:
        output = Path(args.report_json).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")

    if exit_code == 0 and args.launch_command:
        command = list(args.launch_command)
        if command and command[0] == "--":
            command = command[1:]
        if not command:
            parser.error("--exec requires a command")
        os.execvp(command[0], command)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
