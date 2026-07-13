import json
import math
from pathlib import Path
import sys


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

import generate_lap_trajectory_plots as plots  # noqa: E402


def synthetic_circuit(laps: float, lap_duration_s: float = 100.0):
    base_ns = 1_700_000_000_000_000_000
    count = int(laps * lap_duration_s * 10) + 1
    ins = []
    trajectory = []
    for index in range(count):
        elapsed_s = index * 0.1
        angle = -2.0 * math.pi * elapsed_s / lap_duration_s
        x = 120.0 * math.cos(angle)
        y = 80.0 * math.sin(angle)
        stamp_ns = base_ns + int(round(elapsed_s * 1e9))
        ins.append(plots.PosePoint(stamp_ns, x, y, 0.0))
        trajectory.append(
            plots.PosePoint(
                stamp_ns,
                x + 0.8 * math.sin(3.0 * angle),
                y - 0.5,
                0.0,
            )
        )
    return base_ns, trajectory, ins


def snap_at(base_ns: int, elapsed_s: float, lap_duration_s: float = 100.0):
    angle = -2.0 * math.pi * elapsed_s / lap_duration_s
    target_x = 120.0 * math.cos(angle)
    target_y = 80.0 * math.sin(angle)
    return plots.SnapEvent(
        base_ns + int(round(elapsed_s * 1e9)),
        target_x + 5.0,
        target_y - 3.0,
        target_x,
        target_y,
    )


def test_generates_one_image_per_completed_lap_with_snap_times(tmp_path):
    base_ns, trajectory, ins = synthetic_circuit(4.25)
    snaps = [
        snap_at(base_ns, elapsed_s)
        for elapsed_s in (35.0, 145.0, 248.0, 349.0, 420.0)
    ]

    manifest = plots.generate_plots(
        trajectory,
        ins,
        snaps,
        tmp_path / 'trajtory',
        min_lap_duration_s=60.0,
        max_lap_duration_s=140.0,
        line_half_width_m=10.0,
        min_speed_mps=2.0,
    )

    assert manifest['complete_lap_count'] == 4
    assert manifest['image_count'] == 5
    assert manifest['total_snap_count'] == 5
    assert manifest['plotted_snap_count'] == 5
    assert manifest['unassigned_snap_count'] == 0
    assert manifest['lap_detection']['confidence'] == 'high'
    assert [lap['snap_count'] for lap in manifest['laps']] == [1, 1, 1, 1, 1]
    assert [lap['complete'] for lap in manifest['laps']] == [
        True,
        True,
        True,
        True,
        False,
    ]
    assert manifest['colors'] == {
        'applied_gicp_trajectory': plots.TRAJECTORY_COLOR,
        'atlas_ins_reference': plots.INS_COLOR,
        'snap_correction_to_ins': plots.SNAP_COLOR,
    }

    output_dir = tmp_path / 'trajtory'
    images = sorted(output_dir.glob('lap_*.png'))
    assert [image.name for image in images] == [
        'lap_001.png',
        'lap_002.png',
        'lap_003.png',
        'lap_004.png',
    ]
    assert all(image.stat().st_size > 10_000 for image in images)
    assert (output_dir / 'partial_001.png').stat().st_size > 10_000
    disk_manifest = json.loads((output_dir / 'trajtory_manifest.json').read_text())
    assert disk_manifest == manifest
    assert disk_manifest['laps'][0]['snaps'][0]['ros_time_ns'] == snaps[0].stamp_ns
    assert disk_manifest['laps'][0]['snaps'][0]['ros_time_s'].endswith(
        '.000000000'
    )
    assert disk_manifest['laps'][0]['snaps'][0]['correction_m'] == math.hypot(
        5.0, 3.0
    )


def test_short_run_generates_one_labelled_partial_image(tmp_path):
    _base_ns, trajectory, ins = synthetic_circuit(0.25)
    manifest = plots.generate_plots(
        trajectory,
        ins,
        [],
        tmp_path / 'trajtory',
        min_lap_duration_s=60.0,
        max_lap_duration_s=140.0,
        line_half_width_m=10.0,
        min_speed_mps=2.0,
    )

    assert manifest['complete_lap_count'] == 0
    assert manifest['image_count'] == 1
    assert manifest['plotted_snap_count'] == 0
    assert manifest['unassigned_snap_count'] == 0
    assert manifest['lap_detection']['confidence'] == 'partial'
    assert manifest['laps'][0]['complete'] is False
    assert manifest['laps'][0]['snap_count'] == 0
    assert (tmp_path / 'trajtory' / 'lap_001_partial.png').is_file()
