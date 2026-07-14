from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "check_luminar_clock_hypothesis.py"
SPEC = importlib.util.spec_from_file_location("check_luminar_clock_hypothesis", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def frame(speed: float, along: float) -> object:
    return MODULE.FrameResult(
        aux_topic="/luminar_left/points",
        front_header_s=0.0,
        speed_mps=speed,
        velocity_flu=(speed, 0.0, 0.0),
        omega_flu=(0.0, 0.0, 0.0),
        front_span_ms=49.0,
        aux_span_ms=49.0,
        selected_header_delta_ms=0.0,
        selected_point_mid_delta_ms=0.0,
        header_nearest_point_mid_delta_ms=0.0,
        along_track_correction_m=along,
        lateral_correction_m=0.0,
        vertical_correction_m=0.0,
        icp_rmse_m=0.1,
        icp_inliers=500,
        icp_inlier_ratio=0.5,
        icp_rotation_deg=0.0,
        front_points_used=1000,
        aux_points_used=1000,
    )


def test_regression_recovers_80_ms_clock_correction() -> None:
    rng = np.random.default_rng(7)
    speeds = np.linspace(0.0, 20.0, 50)
    frames = [
        frame(float(speed), float(0.15 + 0.080 * speed + rng.normal(0.0, 0.01)))
        for speed in speeds
    ]
    result = MODULE.regress_clock_offset(
        frames,
        bootstrap_samples=400,
        seed=11,
        min_frames=15,
        min_speed_span_mps=5.0,
        effect_threshold_ms=10.0,
        min_abs_correlation=0.35,
    )
    assert result["decision"] == "YES"
    assert abs(result["offset_ms"] - 80.0) < 2.0
    assert MODULE.classify_large_offset(result, 50.0)["decision"] == "YES"


def test_regression_accepts_two_ms_as_equivalent_to_zero() -> None:
    rng = np.random.default_rng(9)
    speeds = np.linspace(0.0, 20.0, 50)
    frames = [
        frame(float(speed), float(0.12 + 0.002 * speed + rng.normal(0.0, 0.005)))
        for speed in speeds
    ]
    result = MODULE.regress_clock_offset(
        frames,
        bootstrap_samples=400,
        seed=13,
        min_frames=15,
        min_speed_span_mps=5.0,
        effect_threshold_ms=10.0,
        min_abs_correlation=0.35,
    )
    assert result["decision"] == "NO"
    assert abs(result["offset_ms"] - 2.0) < 2.0
    assert MODULE.classify_large_offset(result, 50.0)["decision"] == "NO"


def test_icp_recovers_known_aux_to_front_transform() -> None:
    rng = np.random.default_rng(4)
    source = rng.uniform(-2.0, 2.0, size=(1500, 3))
    angle = np.deg2rad(1.5)
    transform = np.eye(4)
    transform[:3, :3] = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    transform[:3, 3] = [0.12, -0.08, 0.03]
    target = MODULE.transform_points(source, transform)
    result = MODULE.run_icp(
        source,
        target,
        max_iterations=30,
        initial_correspondence_m=0.5,
        final_correspondence_m=0.05,
        trim_fraction=0.9,
        min_inliers=200,
    )
    assert result.converged
    assert np.allclose(np.asarray(result.transform), transform, atol=1e-3)


def test_urdf_extrinsic_direction_is_base_from_sensor() -> None:
    urdf = SCRIPT.parents[1] / "av24.urdf"
    transforms = MODULE.load_urdf_extrinsics(
        urdf, "rear_axle_middle", ["luminar_front", "luminar_left", "luminar_right"]
    )
    assert np.allclose(transforms["luminar_front"][:3, 3], [2.222, 0.005, 0.448])
    assert np.allclose(transforms["luminar_left"][:3, 3], [1.564, 0.149, 0.535])
    assert np.allclose(transforms["luminar_right"][:3, 3], [1.574, -0.153, 0.535])
