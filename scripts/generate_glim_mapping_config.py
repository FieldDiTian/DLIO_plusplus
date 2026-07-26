#!/usr/bin/env python3
"""Generate a self-contained high-quality GLIM mapping configuration.

The profile is dataset-independent. Dataset-specific values are injected only
through CLI arguments:

* IMU/GNSS/point topics describe the prepared bag contract.
* ``--t-lidar-imu`` supplies the calibration for that bag's body-frame IMU.
* ``--imu-input-rotation`` optionally corrects incoming IMU vectors that still
  contain a measured receiver-mount tilt.
* ``--gnss-min-baseline`` controls when the one-shot world/GNSS alignment is
  initialized; the default is the 10 m Laguna value validated in perception-ws.
* ``--gnss-fit-max-rms`` is the quality gate for that alignment; the generated
  high-quality profile fits the newest segment that still spans the baseline.
* ``--keyframes-per-submap`` controls how many locally optimized scans are
  grouped before one global pose-graph update. The one-scan perception-ws
  setting remains the default; longer runs can inject a bounded group size to
  avoid a global iSAM2 update for every LiDAR frame.
* ``--urdf-path`` and repeated ``--aux-lidar TOPIC:FRAME`` entries describe
  multi-LiDAR concatenation.
* ``--offload-dir`` is a unique, absolute per-run scratch path. Dense submap
  points are kept there while the bounded registration sample stays in RAM.

The numerical defaults reproduce the successful perception-ws Laguna profile:
0.25 m / 40k preprocessing, CPU LIO, one scan per submap, 5 m loop search
after 50 m travel, 0.1 loop sigma, global gauge damping 1.0, covariance-aware
GNSS anchors initialized after a 10 m baseline and bounded to honest floor/cap
values, and a 0.5 m post-optimization divergence gate.
The generated directory is passed without installing or mutating shared config:

  ros2 run glim_ros glim_rosbag BAG --ros-args \
    -p config_path:=/absolute/generated/config \
    -p dump_path:=/absolute/output/dump
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


GENERATED_FILES = {
    "config.json",
    "config_logging.json",
    "config_viewer.json",
    "config_sensors.json",
    "config_preprocess_quality.json",
    "config_odometry_cpu_quality.json",
    "config_sub_mapping_quality.json",
    "config_global_mapping_quality.json",
    "config_ros.json",
    "config_gnss_global.json",
    "config_pcap.json",
    "quality_profile.json",
}


def parse_aux_lidar(value: str) -> tuple[str, str]:
    try:
        topic, frame = value.split(":", 1)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "expected TOPIC:FRAME, for example /luminar_left/points:luminar_left"
        ) from exc
    if not topic.startswith("/") or not frame:
        raise argparse.ArgumentTypeError("aux lidar topic must be absolute and frame must be non-empty")
    return topic, frame


def finite_vector(values: list[float], name: str) -> list[float]:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain only finite numbers")
    return values


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=False) + "\n", encoding="utf-8")


def build_configs(args: argparse.Namespace) -> dict[str, Any]:
    aux_topics = [topic for topic, _ in args.aux_lidar]
    aux_frames = [frame for _, frame in args.aux_lidar]
    concat_enabled = bool(args.aux_lidar)
    if concat_enabled and args.urdf_path is None:
        raise ValueError("--urdf-path is required when --aux-lidar is used")
    if not math.isfinite(args.gnss_min_baseline) or args.gnss_min_baseline <= 0.0:
        raise ValueError("--gnss-min-baseline must be a finite positive value")
    if not math.isfinite(args.gnss_fit_max_rms):
        raise ValueError("--gnss-fit-max-rms must be finite")
    if args.keyframes_per_submap <= 0:
        raise ValueError("--keyframes-per-submap must be a positive integer")

    t_lidar_imu = finite_vector(args.t_lidar_imu, "--t-lidar-imu")
    imu_input_rotation = finite_vector(
        args.imu_input_rotation, "--imu-input-rotation"
    )
    imu_quat_norm = math.sqrt(sum(value * value for value in imu_input_rotation))
    if imu_quat_norm < 1.0e-9:
        raise ValueError("--imu-input-rotation must be a non-zero quaternion")
    imu_input_rotation = [value / imu_quat_norm for value in imu_input_rotation]
    expanded_offload_dir = args.offload_dir.expanduser()
    if not expanded_offload_dir.is_absolute():
        raise ValueError("--offload-dir must be an absolute path")
    offload_dir = expanded_offload_dir.resolve()

    config = {
        "global": {
            "config_path": "",
            "config_ros": "config_ros.json",
            "config_logging": "config_logging.json",
            "config_viewer": "config_viewer.json",
            "config_sensors": "config_sensors.json",
            "config_preprocess": "config_preprocess_quality.json",
            "config_odometry": "config_odometry_cpu_quality.json",
            "config_sub_mapping": "config_sub_mapping_quality.json",
            "config_global_mapping": "config_global_mapping_quality.json",
            # GlobalConfigExt checks the main config first. Keeping this here
            # avoids copying a run-specific GNSS file into the installed
            # glim_ext share directory (the old cross-run contamination trap).
            "config_gnss_global": "config_gnss_global.json",
            "config_pcap": "config_pcap.json",
        }
    }

    sensors = {
        "lidar_concat": {
            "enabled": concat_enabled,
            "primary_frame": args.primary_frame,
            "aux_topics": aux_topics,
            "aux_frames": aux_frames,
            "buffer_size": args.concat_buffer_size,
            "time_threshold": args.header_time_threshold,
            "luminar_time_threshold": args.point_time_threshold,
            "future_sweep_wait_timeout": args.future_sweep_wait_timeout,
            # The two-pass planner requires absolute per-point epochs. Decoded
            # FLOAT32/FLOAT64 sweep-relative seconds must use the future-header
            # watermark join; indexing them would read the bag once only to
            # discover that no absolute ranges are available and fall back.
            "two_pass_point_time_join": args.float64_time_is_epoch_ns,
            "aux_match_time_offsets": [0.0] * len(aux_topics),
            "aux_point_time_offsets": [0.0] * len(aux_topics),
            "aux_time_offsets": [],
            "urdf_path": str(args.urdf_path.expanduser().resolve()) if args.urdf_path else "",
            "require_all_aux": args.require_all_aux,
            "abort_on_merge_failure": args.require_all_aux,
            "max_consecutive_aux_merge_failures": 10,
            "frame_diag_log": True,
        },
        "sensors": {
            "T_lidar_imu": t_lidar_imu,
            "imu_input_rotation": imu_input_rotation,
            "intensity_field": args.intensity_field,
            "ring_field": args.ring_field,
            "autoconf_perpoint_times": True,
            "autoconf_prefer_frame_time": False,
            "float64_time_is_epoch_ns": args.float64_time_is_epoch_ns,
            # Decoded FLOAT32/FLOAT64 point times are seconds relative to each
            # sweep unless the explicit raw epoch-ns switch above is enabled.
            # TimeKeeper autoconfiguration reaches the same conclusion, but
            # keeping the declared contract accurate makes this safe if an
            # operator later disables autoconfiguration.
            "perpoint_relative_time": not args.float64_time_is_epoch_ns,
            "perpoint_time_scale": 1.0,
            "global_shutter_lidar": False,
            "flip_points_y": False,
            "imu_acc_noise": args.imu_acc_noise,
            "imu_gyro_noise": args.imu_gyro_noise,
            "imu_bias_noise": 1.0e-5,
            "imu_int_noise": 1.0e-3,
            # Leave these blank: T_lidar_imu above is the explicit calibration
            # for the injected IMU topic. The URDF is used only for aux LiDAR
            # extrinsics, so GlimROS must not overwrite this transform.
            "urdf_path": "",
            "urdf_lidar_frame": "",
            "urdf_imu_frame": "",
        },
    }

    preprocess = {
        "preprocess": {
            "distance_near_thresh": args.distance_near,
            "distance_far_thresh": args.distance_far,
            "use_random_grid_downsampling": True,
            "downsample_resolution": args.downsample,
            "random_downsample_target": args.points_per_scan,
            "random_downsample_rate": 0.1,
            "enable_outlier_removal": False,
            "outlier_removal_k": 10,
            "outlier_std_mul_factor": 1.0,
            "enable_cropbox_filter": False,
            "crop_bbox_frame": "lidar",
            "crop_bbox_min": [-1.0, -1.0, -1.0],
            "crop_bbox_max": [1.0, 1.0, 1.0],
            "k_correspondences": 10,
            "num_threads": args.preprocess_threads,
        }
    }

    odometry = {
        "odometry_estimation": {
            "so_name": "libodometry_estimation_cpu.so",
            "initialization_mode": "LOOSE",
            "initialization_window_size": 3.0,
            # This is the local fixed-lag estimator's initial gauge, not the
            # global pose-graph damping corrected below.
            "init_pose_damping_scale": 1.0e10,
            "smoother_lag": 5.0,
            "use_isam2_dogleg": False,
            "isam2_relinearize_skip": 1,
            "isam2_relinearize_thresh": 0.1,
            "fix_imu_bias": False,
            "registration_type": "GICP",
            "max_iterations": 8,
            "lru_thresh": 100,
            "target_downsampling_rate": 0.1,
            "ivox_resolution": 1.0,
            "ivox_min_dist": 0.1,
            "vgicp_resolution": 0.5,
            "vgicp_voxelmap_levels": 1,
            "vgicp_voxelmap_scaling_factor": 2.0,
            "validate_imu": True,
            "save_imu_rate_trajectory": True,
            "num_threads": args.odom_threads,
        }
    }

    sub_mapping = {
        "sub_mapping": {
            "so_name": "libsub_mapping_passthrough.so",
            "keyframe_update_interval_rot": 0.0,
            "keyframe_update_interval_trans": 0.0,
            "max_num_keyframes": args.keyframes_per_submap,
            "max_num_voxels": -1,
            "adaptive_max_num_voxels": -1,
            "submap_voxel_resolution": args.submap_voxel,
            "min_dist_in_voxel": args.submap_min_point_distance,
            "max_num_points_in_voxel": 200,
            "submap_target_num_points": args.submap_points,
        }
    }

    global_mapping = {
        "global_mapping": {
            "so_name": "libglobal_mapping_pose_graph.so",
            "enable_optimization": True,
            "init_pose_damping_scale": 1.0,
            "registration_type": "GICP",
            "min_travel_dist": 50.0,
            "max_neighbor_dist": 5.0,
            "min_inliear_fraction": 0.5,
            "subsample_target": args.loop_registration_points,
            "subsample_rate": 0.1,
            "gicp_max_correspondence_dist": 2.0,
            "vgicp_voxel_resolution": 2.0,
            "odom_factor_stddev": 1.0e-3,
            "loop_factor_stddev": 0.1,
            "loop_factor_robust_width": 1.0,
            "loop_candidate_buffer_size": 100,
            "loop_candidate_eval_per_thread": 2,
            "use_isam2_dogleg": False,
            "isam2_relinearize_skip": 1,
            "isam2_relinearize_thresh": 0.1,
            "offload_points_dir": str(offload_dir),
            "num_threads": args.loop_threads,
        }
    }

    ros = {
        "glim_ros": {
            "enable_online_mapping": False,
            "enable_local_mapping": True,
            "enable_global_mapping": True,
            "keep_raw_points": False,
            "imu_time_offset": args.imu_time_offset,
            "points_time_offset": args.points_time_offset,
            "acc_scale": 0.0,
            "imu_frame_id": "",
            "lidar_frame_id": "",
            "base_frame_id": "",
            "odom_frame_id": "odom",
            "map_frame_id": "map",
            "publish_imu2lidar": True,
            "tf_time_offset": 1.0e-6,
            # Headless mapping: visualization extensions retain extra clouds
            # and are deliberately excluded from the production profile.
            "extension_modules": ["libgnss_global.so"],
            "imu_topic": args.imu_topic,
            "points_topic": args.points_topic,
            "image_topic": "/image",
            "external_odom_topic": "",
            "playback_speed": 100.0,
            "imu_qos": {"profile": "sensor_data", "depth": 1000},
            "points_qos": {
                "profile": "sensor_data",
                "reliability": "best_effort",
            },
            "image_qos": {"profile": "sensor_data"},
        }
    }

    gnss = {
        "gnss": {
            "gnss_topic": args.gnss_topic,
            "gnss_msg_type": args.gnss_msg_type,
            "min_baseline": args.gnss_min_baseline,
            "fit_recent_baseline_window": args.gnss_recent_fit_window,
            # Missing/invalid covariance falls back to this honest floor.
            "prior_inf_scale": [100.0, 100.0, 25.0],
            "prior_inf_floor": [100.0, 100.0, 25.0],
            "prior_inf_cap": [2500.0, 2500.0, 1000.0],
            "position_prior_robust_width": 1.5,
            "enable_orientation_prior": False,
            "orientation_prior_inf_scale": [0.0, 0.0, 0.0],
            "orientation_prior_max_yaw_sigma_deg": 3.0,
            "max_interp_gap_sec": 1.0,
            "fit_max_rms": args.gnss_fit_max_rms,
            "anchor_abort_median_m": 0.5,
            "anchor_residual_window": 100,
            "anchor_residual_min_samples": 20,
            # A new GNSS factor batch can move the rolling median across the
            # threshold for one optimizer update before iSAM2 settles. Keep the
            # 0.5 m gate, but require sustained divergence for five updates.
            "anchor_abort_consecutive_updates": 5,
            "enable_lever_arm": False,
            "urdf_gnss_frame": "",
        }
    }

    pcap = {
        "pcap": {
            "iris_data_src_port": 4371,
            "inactivity_sec": 0.05,
            "scan_reorder_lookahead_ns": 200_000_000,
            "lidar_ips": [],
            "lidar_topics": [args.points_topic, *aux_topics],
            "lidar_frame_ids": [args.primary_frame, *aux_frames],
            "lidar_dst_ports": [],
        }
    }

    profile = {
        "profile": f"perception-ws-high-quality-kf{args.keyframes_per_submap}",
        "generated_config_path": str(args.output_dir.expanduser().resolve()),
        "offload_points_dir": str(offload_dir),
        "expected_export_voxel_m": 0.15,
        "injected": {
            "imu_topic": args.imu_topic,
            "gnss_topic": args.gnss_topic,
            "gnss_msg_type": args.gnss_msg_type,
            "gnss_min_baseline_m": args.gnss_min_baseline,
            "gnss_recent_fit_window": args.gnss_recent_fit_window,
            "gnss_fit_max_rms_m": args.gnss_fit_max_rms,
            "keyframes_per_submap": args.keyframes_per_submap,
            "preprocess_downsample_m": args.downsample,
            "preprocess_points_per_scan": args.points_per_scan,
            "submap_voxel_m": args.submap_voxel,
            "submap_target_points": args.submap_points,
            "loop_registration_points": args.loop_registration_points,
            "points_topic": args.points_topic,
            "T_lidar_imu": t_lidar_imu,
            "imu_input_rotation": imu_input_rotation,
            "urdf_path": sensors["lidar_concat"]["urdf_path"],
            "aux_lidars": [
                {"topic": topic, "frame": frame} for topic, frame in args.aux_lidar
            ],
        },
    }

    return {
        "config.json": config,
        "config_logging.json": {
            "logging": {
                "log_dir": "/tmp",
                "save_logs": True,
                "rotate_logs": True,
                "max_file_size_kb": 8192,
                "max_files": 10,
            }
        },
        "config_viewer.json": {"viewer": {"auto_close": True}},
        "config_sensors.json": sensors,
        "config_preprocess_quality.json": preprocess,
        "config_odometry_cpu_quality.json": odometry,
        "config_sub_mapping_quality.json": sub_mapping,
        "config_global_mapping_quality.json": global_mapping,
        "config_ros.json": ros,
        "config_gnss_global.json": gnss,
        "config_pcap.json": pcap,
        "quality_profile.json": profile,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--offload-dir", required=True, type=Path)
    parser.add_argument("--imu-topic", required=True)
    parser.add_argument("--gnss-topic", default="/gnss")
    parser.add_argument(
        "--gnss-msg-type",
        choices=[
            "geometry_msgs/msg/PoseWithCovarianceStamped",
            "nav_msgs/msg/Odometry",
        ],
        default="geometry_msgs/msg/PoseWithCovarianceStamped",
    )
    parser.add_argument(
        "--gnss-min-baseline",
        type=float,
        default=10.0,
        help="Minimum mapped travel in metres before the one-shot world/GNSS "
        "alignment is initialized (perception-ws Laguna default: 10.0).",
    )
    parser.add_argument(
        "--gnss-recent-fit-window",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Fit the newest segment that still spans --gnss-min-baseline on "
        "both trajectories; disable to reproduce the legacy all-history fit.",
    )
    parser.add_argument(
        "--gnss-fit-max-rms",
        type=float,
        default=0.25,
        help="Maximum RMS residual in metres for latching the world/GNSS fit; "
        "<= 0 disables this gate (high-quality default: 0.25).",
    )
    parser.add_argument("--points-topic", default="/luminar_front/points")
    parser.add_argument("--primary-frame", default="luminar_front")
    parser.add_argument("--t-lidar-imu", nargs=7, required=True, type=float)
    parser.add_argument(
        "--imu-input-rotation",
        nargs=4,
        type=float,
        default=[0.0, 0.0, 0.0, 1.0],
        metavar=("QX", "QY", "QZ", "QW"),
        help="Fixed quaternion rotating incoming acceleration/gyro vectors into the "
        "calibrated IMU frame used by --t-lidar-imu. Keep identity when the "
        "prepared topic is already fully body-frame calibrated.",
    )
    parser.add_argument("--urdf-path", type=Path)
    parser.add_argument(
        "--aux-lidar",
        action="append",
        default=[],
        type=parse_aux_lidar,
        metavar="TOPIC:FRAME",
    )
    parser.add_argument(
        "--require-all-aux",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--float64-time-is-epoch-ns", action="store_true")
    parser.add_argument("--intensity-field", default="reflectance")
    parser.add_argument("--ring-field", default="line_index")
    parser.add_argument("--imu-time-offset", type=float, default=0.0)
    parser.add_argument("--points-time-offset", type=float, default=0.0)
    parser.add_argument("--imu-acc-noise", type=float, default=0.05)
    parser.add_argument("--imu-gyro-noise", type=float, default=0.01)
    parser.add_argument("--distance-near", type=float, default=0.5)
    parser.add_argument("--distance-far", type=float, default=100.0)
    parser.add_argument("--downsample", type=float, default=0.25)
    parser.add_argument("--points-per-scan", type=int, default=40_000)
    parser.add_argument("--submap-voxel", type=float, default=0.05)
    parser.add_argument("--submap-min-point-distance", type=float, default=0.02)
    parser.add_argument("--submap-points", type=int, default=60_000)
    parser.add_argument(
        "--keyframes-per-submap",
        type=int,
        default=1,
        help="Locally optimized LiDAR keyframes grouped into each global "
        "pose-graph submap. The perception-ws quality default is 1; inject a "
        "bounded value such as 10 for long full-run maps so global iSAM2 does "
        "not update once per scan.",
    )
    parser.add_argument("--loop-registration-points", type=int, default=10_000)
    parser.add_argument("--preprocess-threads", type=int, default=1)
    parser.add_argument("--odom-threads", type=int, default=2)
    parser.add_argument("--loop-threads", type=int, default=4)
    parser.add_argument("--concat-buffer-size", type=int, default=200)
    parser.add_argument("--header-time-threshold", type=float, default=0.1)
    parser.add_argument("--point-time-threshold", type=float, default=0.01)
    parser.add_argument("--future-sweep-wait-timeout", type=float, default=0.15)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    expanded_output_dir = args.output_dir.expanduser()
    if not expanded_output_dir.is_absolute():
        raise SystemExit("--output-dir must be an absolute path")
    output_dir = expanded_output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    existing = [name for name in GENERATED_FILES if (output_dir / name).exists()]
    if existing and not args.force:
        raise SystemExit(
            f"refusing to overwrite generated config files in {output_dir}: "
            f"{', '.join(sorted(existing))}; pass --force for this exact directory"
        )

    configs = build_configs(args)
    for filename, value in configs.items():
        write_json(output_dir / filename, value)

    print(output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
