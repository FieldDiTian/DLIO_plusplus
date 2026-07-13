#!/usr/bin/env python3
"""Generate one top-down GICP/INS trajectory image for every completed lap.

The standard GICP test records only ``/gicp/localization/debug/*``.  This
post-processor reads three topics emitted by ``gicp_localization``:

* ``trajectory_pose``: the pose actually applied after accept/reject/snap;
* ``ins_pose``: the time-matched Atlas INS pose in the same local-ENU frame;
* ``snap_correction``: PoseArray[estimate_before_snap, INS_target].

Completed laps are inferred from repeated, same-direction crossings of an
automatically selected line on the INS trajectory.  If a short smoke test has
no complete lap, one clearly labelled partial image is generated instead.
Images and a machine-readable manifest are written to the test's deliberately
spelled ``trajtory/`` directory by default.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Iterable, Sequence

os.environ.setdefault(
    'MPLCONFIGDIR',
    str(Path(os.environ.get('TMPDIR', '/tmp')) / 'gicp_lap_trajectory_matplotlib'),
)

try:
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError as exc:  # pragma: no cover - dependency failure path
    print(f'matplotlib and numpy are required: {exc}', file=sys.stderr)
    raise SystemExit(2) from exc

try:
    from rosbags.highlevel import AnyReader
except ImportError as exc:  # pragma: no cover - dependency failure path
    print(f'rosbags is required: {exc}', file=sys.stderr)
    raise SystemExit(2) from exc


TRAJECTORY_TOPIC = '/gicp/localization/debug/trajectory_pose'
INS_TOPIC = '/gicp/localization/debug/ins_pose'
SNAP_TOPIC = '/gicp/localization/debug/snap_correction'

TRAJECTORY_COLOR = '#149c36'  # green
INS_COLOR = '#d62728'         # red
SNAP_COLOR = '#f2c500'        # yellow


@dataclass(frozen=True)
class PosePoint:
    stamp_ns: int
    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class SnapEvent:
    stamp_ns: int
    from_x: float
    from_y: float
    to_x: float
    to_y: float

    @property
    def correction_m(self) -> float:
        return math.hypot(self.to_x - self.from_x, self.to_y - self.from_y)


@dataclass(frozen=True)
class LapInterval:
    index: int
    start_ns: int
    end_ns: int
    complete: bool = True
    segment_kind: str = 'lap'

    @property
    def duration_s(self) -> float:
        return (self.end_ns - self.start_ns) * 1e-9


@dataclass(frozen=True)
class LapDetection:
    method: str
    confidence: str
    anchor_x: float | None
    anchor_y: float | None
    tangent_x: float | None
    tangent_y: float | None
    crossing_times_ns: tuple[int, ...]
    lap_durations_s: tuple[float, ...]
    interval_cv: float | None
    partial_before_s: float
    partial_after_s: float
    note: str


def _stamp_ns(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _deduplicate_points(points: Iterable[PosePoint]) -> list[PosePoint]:
    by_stamp: dict[int, PosePoint] = {}
    for point in points:
        if all(math.isfinite(value) for value in (point.x, point.y, point.z)):
            by_stamp[point.stamp_ns] = point
    return [by_stamp[key] for key in sorted(by_stamp)]


def _deduplicate_snaps(events: Iterable[SnapEvent]) -> list[SnapEvent]:
    by_stamp: dict[int, SnapEvent] = {}
    for event in events:
        values = (event.from_x, event.from_y, event.to_x, event.to_y)
        if all(math.isfinite(value) for value in values):
            by_stamp[event.stamp_ns] = event
    return [by_stamp[key] for key in sorted(by_stamp)]


def load_debug_bag(
    bag_path: Path,
) -> tuple[list[PosePoint], list[PosePoint], list[SnapEvent], dict[str, object]]:
    """Load the three plot-evidence topics from an MCAP/rosbag2 directory."""
    bag_path = bag_path.expanduser().resolve()
    if not bag_path.exists():
        raise FileNotFoundError(f'debug bag does not exist: {bag_path}')

    trajectory: list[PosePoint] = []
    ins: list[PosePoint] = []
    snaps: list[SnapEvent] = []
    frames: dict[str, set[str]] = {
        TRAJECTORY_TOPIC: set(),
        INS_TOPIC: set(),
        SNAP_TOPIC: set(),
    }
    recorded_topic_counts = {
        TRAJECTORY_TOPIC: 0,
        INS_TOPIC: 0,
        SNAP_TOPIC: 0,
    }

    with AnyReader([bag_path]) as reader:
        selected = [
            connection
            for connection in reader.connections
            if connection.topic in frames
        ]
        present = {connection.topic for connection in selected}
        for connection in selected:
            recorded_topic_counts[connection.topic] += int(connection.msgcount)
        # A no-snap run may have no MCAP connection for the event topic at all.
        # trajectory_pose and ins_pose are mandatory continuous evidence; an
        # absent snap_correction connection is valid and means zero snap events.
        missing = {TRAJECTORY_TOPIC, INS_TOPIC} - present
        if missing:
            missing_text = ', '.join(sorted(missing))
            raise RuntimeError(
                'debug bag is missing required trajectory evidence topic(s): '
                f'{missing_text}'
            )

        for connection, _record_ns, rawdata in reader.messages(connections=selected):
            msg = reader.deserialize(rawdata, connection.msgtype)
            frame_id = str(msg.header.frame_id)
            frames[connection.topic].add(frame_id)
            stamp_ns = _stamp_ns(msg.header.stamp)

            if connection.topic == TRAJECTORY_TOPIC:
                position = msg.pose.position
                trajectory.append(
                    PosePoint(stamp_ns, position.x, position.y, position.z)
                )
            elif connection.topic == INS_TOPIC:
                position = msg.pose.position
                ins.append(PosePoint(stamp_ns, position.x, position.y, position.z))
            else:
                if len(msg.poses) != 2:
                    raise RuntimeError(
                        f'{SNAP_TOPIC} at {stamp_ns} ns has {len(msg.poses)} poses; '
                        'expected [pre-snap estimate, INS target]'
                    )
                before = msg.poses[0].position
                target = msg.poses[1].position
                snaps.append(
                    SnapEvent(
                        stamp_ns,
                        before.x,
                        before.y,
                        target.x,
                        target.y,
                    )
                )

    trajectory = _deduplicate_points(trajectory)
    ins = _deduplicate_points(ins)
    snaps = _deduplicate_snaps(snaps)
    if len(trajectory) < 2:
        raise RuntimeError(f'{TRAJECTORY_TOPIC} has fewer than two finite samples')
    if len(ins) < 2:
        raise RuntimeError(f'{INS_TOPIC} has fewer than two finite samples')

    frame_lists = {topic: sorted(values) for topic, values in frames.items()}
    pose_frames = frames[TRAJECTORY_TOPIC] | frames[INS_TOPIC] | frames[SNAP_TOPIC]
    if len(pose_frames) > 1:
        raise RuntimeError(
            'trajectory evidence topics are not in one common frame: '
            + json.dumps(frame_lists, sort_keys=True)
        )

    metadata: dict[str, object] = {
        'bag': str(bag_path),
        'frames': frame_lists,
        'topic_counts': recorded_topic_counts,
        'finite_unique_sample_counts': {
            TRAJECTORY_TOPIC: len(trajectory),
            INS_TOPIC: len(ins),
            SNAP_TOPIC: len(snaps),
        },
    }
    return trajectory, ins, snaps, metadata


def _point_arrays(points: Sequence[PosePoint]) -> tuple[np.ndarray, np.ndarray]:
    stamps = np.asarray([point.stamp_ns for point in points], dtype=np.int64)
    xy = np.asarray([(point.x, point.y) for point in points], dtype=np.float64)
    return stamps, xy


def _kinematics(
    stamps_ns: np.ndarray, xy: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    times_s = (stamps_ns - stamps_ns[0]).astype(np.float64) * 1e-9
    if np.any(np.diff(times_s) <= 0.0):
        raise ValueError('INS timestamps must be strictly increasing')

    vx = np.gradient(xy[:, 0], times_s, edge_order=1)
    vy = np.gradient(xy[:, 1], times_s, edge_order=1)
    speed = np.hypot(vx, vy)
    segment_distance = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    cumulative_distance = np.concatenate(([0.0], np.cumsum(segment_distance)))
    return times_s, speed, cumulative_distance


def _candidate_indices(
    speed: np.ndarray,
    min_speed_mps: float,
    max_candidates: int = 180,
) -> np.ndarray:
    valid = np.flatnonzero(speed >= min_speed_mps)
    valid = valid[(valid >= 2) & (valid < len(speed) - 2)]
    if not len(valid):
        return np.asarray([], dtype=np.int64)
    if len(valid) <= max_candidates:
        return valid
    selection = np.linspace(0, len(valid) - 1, max_candidates, dtype=np.int64)
    return valid[selection]


def _tangent_at(xy: np.ndarray, index: int, half_window: int = 5) -> np.ndarray | None:
    lo = max(0, index - half_window)
    hi = min(len(xy) - 1, index + half_window)
    delta = xy[hi] - xy[lo]
    norm = float(np.linalg.norm(delta))
    if norm < 1e-6:
        return None
    return delta / norm


def _merge_near_crossings(
    crossings: list[tuple[int, float]], min_gap_ns: int
) -> list[int]:
    """Merge line chatter, keeping the closest-to-anchor crossing in a group."""
    if not crossings:
        return []
    groups: list[list[tuple[int, float]]] = [[crossings[0]]]
    for crossing in crossings[1:]:
        if crossing[0] - groups[-1][-1][0] < min_gap_ns:
            groups[-1].append(crossing)
        else:
            groups.append([crossing])
    return [min(group, key=lambda item: item[1])[0] for group in groups]


def _line_crossings(
    stamps_ns: np.ndarray,
    xy: np.ndarray,
    anchor: np.ndarray,
    tangent: np.ndarray,
    line_half_width_m: float,
    min_speed_mps: float,
    min_lap_duration_s: float,
) -> list[int]:
    normal = np.asarray((-tangent[1], tangent[0]), dtype=np.float64)
    relative = xy - anchor
    along = relative @ tangent
    crossing_idx = np.flatnonzero((along[:-1] <= 0.0) & (along[1:] > 0.0))
    candidates: list[tuple[int, float]] = []

    for index in crossing_idx:
        denominator = along[index + 1] - along[index]
        if denominator <= 1e-9:
            continue
        alpha = float(np.clip(-along[index] / denominator, 0.0, 1.0))
        crossing_xy = xy[index] + alpha * (xy[index + 1] - xy[index])
        lateral = abs(float((crossing_xy - anchor) @ normal))
        if lateral > line_half_width_m:
            continue

        dt_s = (int(stamps_ns[index + 1]) - int(stamps_ns[index])) * 1e-9
        if dt_s <= 0.0:
            continue
        step = xy[index + 1] - xy[index]
        forward_speed = float(step @ tangent) / dt_s
        if forward_speed < min_speed_mps:
            continue

        stamp_ns = int(round(stamps_ns[index] + alpha * (stamps_ns[index + 1] - stamps_ns[index])))
        candidates.append((stamp_ns, lateral))

    return _merge_near_crossings(
        candidates, int(round(min_lap_duration_s * 1e9))
    )


def _crossing_score(
    crossings: Sequence[int],
    min_lap_duration_s: float,
    max_lap_duration_s: float,
) -> tuple[tuple[float, ...], float | None, tuple[float, ...]] | None:
    if len(crossings) < 2:
        return None
    durations = np.diff(np.asarray(crossings, dtype=np.int64)).astype(np.float64) * 1e-9
    plausible = durations[
        (durations >= min_lap_duration_s) & (durations <= max_lap_duration_s)
    ]
    if not len(plausible):
        return None

    median_duration = float(np.median(plausible))
    deviations = np.abs(plausible - median_duration)
    tolerance = max(10.0, 0.30 * median_duration)
    regular_count = int(np.count_nonzero(deviations <= tolerance))
    cv = float(np.std(plausible) / median_duration) if len(plausible) > 1 else 0.0
    coverage_s = (crossings[-1] - crossings[0]) * 1e-9
    # Primary objective is a long chain of regular laps. A candidate that cuts
    # another nearby track segment tends to add irregular crossings and loses
    # on regular_count/CV even if its raw crossing count is larger.
    score = (
        float(regular_count),
        float(len(plausible)),
        -cv,
        coverage_s,
    )
    return score, cv, tuple(float(value) for value in durations)


def detect_laps(
    ins: Sequence[PosePoint],
    *,
    min_lap_duration_s: float = 60.0,
    max_lap_duration_s: float = 600.0,
    line_half_width_m: float = 25.0,
    min_speed_mps: float = 3.0,
    start_xy: tuple[float, float] | None = None,
) -> tuple[list[LapInterval], LapDetection]:
    """Infer complete lap intervals from same-direction INS line crossings."""
    if len(ins) < 2:
        raise ValueError('at least two INS samples are required')
    stamps_ns, xy = _point_arrays(ins)
    _times_s, speed, _distance = _kinematics(stamps_ns, xy)

    if start_xy is None:
        candidate_indices = _candidate_indices(speed, min_speed_mps)
    else:
        target = np.asarray(start_xy, dtype=np.float64)
        candidate_indices = np.asarray(
            [int(np.argmin(np.linalg.norm(xy - target, axis=1)))], dtype=np.int64
        )

    best: tuple[
        tuple[float, ...],
        np.ndarray,
        np.ndarray,
        list[int],
        float | None,
        tuple[float, ...],
    ] | None = None

    for candidate_index in candidate_indices:
        tangent = _tangent_at(xy, int(candidate_index))
        if tangent is None:
            continue
        anchor = xy[int(candidate_index)]
        crossings = _line_crossings(
            stamps_ns,
            xy,
            anchor,
            tangent,
            line_half_width_m,
            min_speed_mps,
            min_lap_duration_s,
        )
        scored = _crossing_score(
            crossings, min_lap_duration_s, max_lap_duration_s
        )
        if scored is None:
            continue
        score, cv, durations = scored
        if best is None or score > best[0]:
            best = (score, anchor.copy(), tangent.copy(), crossings, cv, durations)

    if best is None:
        interval = LapInterval(
            1,
            int(stamps_ns[0]),
            int(stamps_ns[-1]),
            False,
            'partial_run',
        )
        detection = LapDetection(
            method='whole-run fallback',
            confidence='partial',
            anchor_x=None,
            anchor_y=None,
            tangent_x=None,
            tangent_y=None,
            crossing_times_ns=(),
            lap_durations_s=(interval.duration_s,),
            interval_cv=None,
            partial_before_s=0.0,
            partial_after_s=0.0,
            note=(
                'No repeated same-direction INS crossing was found; the run is '
                'rendered once and labelled partial.'
            ),
        )
        return [interval], detection

    _score, anchor, tangent, crossings, cv, durations = best
    intervals = [
        LapInterval(index + 1, int(start), int(end), True)
        for index, (start, end) in enumerate(zip(crossings[:-1], crossings[1:]))
        if min_lap_duration_s <= (end - start) * 1e-9 <= max_lap_duration_s
    ]
    if not intervals:
        interval = LapInterval(
            1,
            int(stamps_ns[0]),
            int(stamps_ns[-1]),
            False,
            'partial_run',
        )
        return [interval], LapDetection(
            method='whole-run fallback',
            confidence='partial',
            anchor_x=float(anchor[0]),
            anchor_y=float(anchor[1]),
            tangent_x=float(tangent[0]),
            tangent_y=float(tangent[1]),
            crossing_times_ns=tuple(crossings),
            lap_durations_s=(interval.duration_s,),
            interval_cv=cv,
            partial_before_s=0.0,
            partial_after_s=0.0,
            note='Crossings were found, but none formed a plausible complete lap.',
        )

    plausible_durations = tuple(interval.duration_s for interval in intervals)
    confidence = 'high' if len(intervals) >= 2 and (cv or 0.0) <= 0.20 else 'medium'
    detection = LapDetection(
        method=(
            'explicit INS start-line crossing'
            if start_xy is not None
            else 'automatic repeated same-direction INS crossing'
        ),
        confidence=confidence,
        anchor_x=float(anchor[0]),
        anchor_y=float(anchor[1]),
        tangent_x=float(tangent[0]),
        tangent_y=float(tangent[1]),
        crossing_times_ns=tuple(int(value) for value in crossings),
        lap_durations_s=plausible_durations,
        interval_cv=cv,
        partial_before_s=max(0.0, (crossings[0] - int(stamps_ns[0])) * 1e-9),
        partial_after_s=max(0.0, (int(stamps_ns[-1]) - crossings[-1]) * 1e-9),
        note=(
            'Lap boundaries are inferred from Atlas INS geometry, not an official '
            'timing transponder line. Pre-first and post-last partial laps are not '
            'numbered as complete laps.'
        ),
    )
    return intervals, detection


def _slice_points(
    points: Sequence[PosePoint], start_ns: int, end_ns: int
) -> list[PosePoint]:
    return [point for point in points if start_ns <= point.stamp_ns <= end_ns]


def _slice_snaps(
    events: Sequence[SnapEvent], start_ns: int, end_ns: int, include_end: bool
) -> list[SnapEvent]:
    if include_end:
        return [event for event in events if start_ns <= event.stamp_ns <= end_ns]
    return [event for event in events if start_ns <= event.stamp_ns < end_ns]


def _format_ros_time(stamp_ns: int) -> str:
    seconds, nanoseconds = divmod(int(stamp_ns), 1_000_000_000)
    return f'{seconds}.{nanoseconds:09d}'


def _build_plot_intervals(
    complete_intervals: Sequence[LapInterval],
    ins: Sequence[PosePoint],
    snaps: Sequence[SnapEvent],
    *,
    min_partial_duration_s: float = 5.0,
) -> list[LapInterval]:
    """Add partial segments so every snap event is assigned to an image."""
    if not complete_intervals:
        return []
    if not any(interval.complete for interval in complete_intervals):
        return list(complete_intervals)

    start_ns = ins[0].stamp_ns
    end_ns = ins[-1].stamp_ns
    ordered = sorted(complete_intervals, key=lambda interval: interval.start_ns)
    result: list[LapInterval] = []
    cursor_ns = start_ns
    partial_index = 1

    def append_partial(partial_start_ns: int, partial_end_ns: int) -> None:
        nonlocal partial_index
        if partial_end_ns <= partial_start_ns:
            return
        duration_s = (partial_end_ns - partial_start_ns) * 1e-9
        has_snap = any(
            partial_start_ns <= event.stamp_ns < partial_end_ns for event in snaps
        )
        if duration_s < min_partial_duration_s and not has_snap:
            return
        result.append(
            LapInterval(
                partial_index,
                partial_start_ns,
                partial_end_ns,
                False,
                'partial_segment',
            )
        )
        partial_index += 1

    for interval in ordered:
        append_partial(cursor_ns, interval.start_ns)
        result.append(interval)
        cursor_ns = max(cursor_ns, interval.end_ns)
    append_partial(cursor_ns, end_ns)
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def render_lap(
    interval: LapInterval,
    trajectory: Sequence[PosePoint],
    ins: Sequence[PosePoint],
    snaps: Sequence[SnapEvent],
    output_path: Path,
    *,
    max_annotation_rows: int = 34,
    include_end: bool = False,
) -> dict[str, object]:
    trajectory_lap = _slice_points(trajectory, interval.start_ns, interval.end_ns)
    ins_lap = _slice_points(ins, interval.start_ns, interval.end_ns)
    # Lap intervals use [start, end) so an event exactly on a boundary cannot
    # be duplicated in two adjacent images.
    snap_lap = _slice_snaps(
        snaps, interval.start_ns, interval.end_ns, include_end
    )
    if len(trajectory_lap) < 2:
        raise RuntimeError(
            f'lap {interval.index} has fewer than two localization samples'
        )
    if len(ins_lap) < 2:
        raise RuntimeError(f'lap {interval.index} has fewer than two INS samples')

    annotation_columns = max(1, math.ceil(len(snap_lap) / max_annotation_rows))
    figure_width = 15.0 + 1.8 * annotation_columns
    figure = plt.figure(figsize=(figure_width, 10.0), constrained_layout=True)
    grid = figure.add_gridspec(
        1, 2, width_ratios=[4.8, max(1.4, 0.9 * annotation_columns)]
    )
    axis = figure.add_subplot(grid[0, 0])
    info_axis = figure.add_subplot(grid[0, 1])

    ins_x = [point.x for point in ins_lap]
    ins_y = [point.y for point in ins_lap]
    trajectory_x = [point.x for point in trajectory_lap]
    trajectory_y = [point.y for point in trajectory_lap]

    axis.plot(
        ins_x,
        ins_y,
        color=INS_COLOR,
        linewidth=2.0,
        label='Atlas INS reference',
        zorder=1,
    )
    axis.plot(
        trajectory_x,
        trajectory_y,
        color=TRAJECTORY_COLOR,
        linewidth=1.6,
        label='Applied GICP trajectory',
        zorder=2,
    )
    axis.scatter(
        [ins_x[0]],
        [ins_y[0]],
        marker='o',
        s=38,
        facecolors='white',
        edgecolors='black',
        linewidths=0.8,
        label='Segment boundary',
        zorder=5,
    )

    snap_manifest: list[dict[str, object]] = []
    for local_index, event in enumerate(snap_lap, start=1):
        axis.plot(
            [event.from_x, event.to_x],
            [event.from_y, event.to_y],
            color=SNAP_COLOR,
            linewidth=2.6,
            solid_capstyle='round',
            zorder=4,
            label='Snap correction to INS' if local_index == 1 else None,
        )
        axis.scatter(
            [event.to_x],
            [event.to_y],
            marker='*',
            s=90,
            facecolors=SNAP_COLOR,
            edgecolors='black',
            linewidths=0.55,
            zorder=6,
        )
        axis.annotate(
            f'#{local_index}',
            xy=(event.to_x, event.to_y),
            xytext=(4, 4),
            textcoords='offset points',
            fontsize=6.5,
            color='#6f5b00',
            weight='bold',
            zorder=7,
        )
        snap_manifest.append(
            {
                'lap_snap_index': local_index,
                'ros_time_ns': event.stamp_ns,
                'ros_time_s': _format_ros_time(event.stamp_ns),
                'from_xy_m': [event.from_x, event.from_y],
                'to_ins_xy_m': [event.to_x, event.to_y],
                'correction_m': event.correction_m,
            }
        )

    if interval.complete:
        segment_title = f'Lap {interval.index:02d} (complete)'
    elif interval.segment_kind == 'partial_run':
        segment_title = 'Partial run (no complete lap detected)'
    else:
        segment_title = f'Partial segment {interval.index:02d}'
    axis.set_title(
        f'{segment_title} — top-down local ENU\n'
        f'ROS start {_format_ros_time(interval.start_ns)}\n'
        f'ROS end   {_format_ros_time(interval.end_ns)}  |  '
        f'duration {interval.duration_s:.2f} s  |  snaps {len(snap_lap)}',
        fontsize=10.5,
    )
    axis.set_xlabel('East / map x [m]')
    axis.set_ylabel('North / map y [m]')
    axis.set_aspect('equal', adjustable='datalim')
    axis.grid(True, alpha=0.25)
    axis.legend(loc='best', fontsize=8)

    info_axis.axis('off')
    info_axis.set_title('Snap locations and ROS times', fontsize=10, loc='left')
    if snap_lap:
        rows = [
            f'#{index:03d}  ROS {_format_ros_time(event.stamp_ns)}\n'
            f'       |correction|={event.correction_m:.3f} m'
            for index, event in enumerate(snap_lap, start=1)
        ]
        rows_per_column = math.ceil(len(rows) / annotation_columns)
        for column in range(annotation_columns):
            column_rows = rows[
                column * rows_per_column: (column + 1) * rows_per_column
            ]
            info_axis.text(
                column / annotation_columns,
                0.98,
                '\n'.join(column_rows),
                transform=info_axis.transAxes,
                va='top',
                ha='left',
                family='monospace',
                fontsize=6.5,
                linespacing=1.18,
            )
    else:
        info_axis.text(
            0.0,
            0.95,
            'No snap events in this lap.',
            transform=info_axis.transAxes,
            va='top',
            fontsize=9,
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)
    return {
        'lap_index': interval.index,
        'complete': interval.complete,
        'segment_kind': interval.segment_kind,
        'start_ros_time_ns': interval.start_ns,
        'start_ros_time_s': _format_ros_time(interval.start_ns),
        'end_ros_time_ns': interval.end_ns,
        'end_ros_time_s': _format_ros_time(interval.end_ns),
        'duration_s': interval.duration_s,
        'trajectory_samples': len(trajectory_lap),
        'ins_samples': len(ins_lap),
        'snap_count': len(snap_lap),
        'snaps': snap_manifest,
        'image': output_path.name,
        'image_sha256': _sha256(output_path),
    }


def generate_plots(
    trajectory: Sequence[PosePoint],
    ins: Sequence[PosePoint],
    snaps: Sequence[SnapEvent],
    output_dir: Path,
    *,
    source_metadata: dict[str, object] | None = None,
    min_lap_duration_s: float = 60.0,
    max_lap_duration_s: float = 600.0,
    line_half_width_m: float = 25.0,
    min_speed_mps: float = 3.0,
    start_xy: tuple[float, float] | None = None,
) -> dict[str, object]:
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    for stale in output_dir.glob('lap_*.png'):
        stale.unlink()
    for stale in output_dir.glob('partial_*.png'):
        stale.unlink()

    complete_intervals, detection = detect_laps(
        ins,
        min_lap_duration_s=min_lap_duration_s,
        max_lap_duration_s=max_lap_duration_s,
        line_half_width_m=line_half_width_m,
        min_speed_mps=min_speed_mps,
        start_xy=start_xy,
    )
    intervals = _build_plot_intervals(complete_intervals, ins, snaps)
    lap_outputs: list[dict[str, object]] = []
    for plot_index, interval in enumerate(intervals):
        if interval.complete:
            image_name = f'lap_{interval.index:03d}.png'
        elif interval.segment_kind == 'partial_run':
            image_name = 'lap_001_partial.png'
        else:
            image_name = f'partial_{interval.index:03d}.png'
        image_path = output_dir / image_name
        lap_outputs.append(
            render_lap(
                interval,
                trajectory,
                ins,
                snaps,
                image_path,
                include_end=plot_index == len(intervals) - 1,
            )
        )

    plotted_snap_count = sum(int(item['snap_count']) for item in lap_outputs)
    if plotted_snap_count != len(snaps):
        raise RuntimeError(
            f'snap assignment is incomplete: plotted {plotted_snap_count} of '
            f'{len(snaps)} events'
        )

    detection_manifest = asdict(detection)
    detection_manifest['crossing_times_ns'] = list(detection.crossing_times_ns)
    detection_manifest['lap_durations_s'] = list(detection.lap_durations_s)
    manifest: dict[str, object] = {
        'schema_version': 2,
        'output_contract': (
            'one top-down image per completed INS-derived lap, plus labelled '
            'partial segments needed to cover every snap event'
        ),
        'output_directory_name': 'trajtory',
        'colors': {
            'applied_gicp_trajectory': TRAJECTORY_COLOR,
            'atlas_ins_reference': INS_COLOR,
            'snap_correction_to_ins': SNAP_COLOR,
        },
        'topics': {
            'applied_gicp_trajectory': TRAJECTORY_TOPIC,
            'atlas_ins_reference': INS_TOPIC,
            'snap_correction': SNAP_TOPIC,
        },
        'source': source_metadata or {},
        'lap_detection': detection_manifest,
        'complete_lap_count': sum(bool(item['complete']) for item in lap_outputs),
        'image_count': len(lap_outputs),
        'total_snap_count': len(snaps),
        'plotted_snap_count': plotted_snap_count,
        'unassigned_snap_count': len(snaps) - plotted_snap_count,
        'laps': lap_outputs,
    }
    manifest_path = output_dir / 'trajtory_manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    return manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--bag', type=Path, required=True, help='GICP debug rosbag2 directory')
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=None,
        help='Output directory (default: <debug-bag-parent>/trajtory)',
    )
    parser.add_argument('--min-lap-duration-s', type=float, default=60.0)
    parser.add_argument('--max-lap-duration-s', type=float, default=600.0)
    parser.add_argument('--line-half-width-m', type=float, default=25.0)
    parser.add_argument('--min-speed-mps', type=float, default=3.0)
    parser.add_argument('--start-x', type=float, default=None)
    parser.add_argument('--start-y', type=float, default=None)
    args = parser.parse_args()
    if (args.start_x is None) != (args.start_y is None):
        parser.error('--start-x and --start-y must be provided together')
    if args.min_lap_duration_s <= 0.0:
        parser.error('--min-lap-duration-s must be positive')
    if args.max_lap_duration_s <= args.min_lap_duration_s:
        parser.error('--max-lap-duration-s must exceed --min-lap-duration-s')
    if args.line_half_width_m <= 0.0:
        parser.error('--line-half-width-m must be positive')
    if args.min_speed_mps < 0.0:
        parser.error('--min-speed-mps must be non-negative')
    return args


def main() -> int:
    args = parse_args()
    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir is not None
        else args.bag.expanduser().resolve().parent / 'trajtory'
    )
    start_xy = (
        None
        if args.start_x is None
        else (float(args.start_x), float(args.start_y))
    )
    try:
        trajectory, ins, snaps, metadata = load_debug_bag(args.bag)
        manifest = generate_plots(
            trajectory,
            ins,
            snaps,
            output_dir,
            source_metadata=metadata,
            min_lap_duration_s=args.min_lap_duration_s,
            max_lap_duration_s=args.max_lap_duration_s,
            line_half_width_m=args.line_half_width_m,
            min_speed_mps=args.min_speed_mps,
            start_xy=start_xy,
        )
    except Exception as exc:
        print(f'trajectory plot generation failed: {exc}', file=sys.stderr)
        return 1

    summary = {
        'output_dir': str(output_dir),
        'manifest': str(output_dir / 'trajtory_manifest.json'),
        'image_count': manifest['image_count'],
        'complete_lap_count': manifest['complete_lap_count'],
        'total_snap_count': manifest['total_snap_count'],
        'plotted_snap_count': manifest['plotted_snap_count'],
        'unassigned_snap_count': manifest['unassigned_snap_count'],
        'lap_detection_confidence': manifest['lap_detection']['confidence'],
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
