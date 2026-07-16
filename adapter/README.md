# Atlas Adapter

The Atlas adapter is the **normalization boundary** (stage 1) of the DLIO++
pipeline:

```
adapter → scripts/prep_bag.py → GLIM (map) → gicp_localization (localize)
```

It keeps sensor-specific WGS84 parsing, RTK covariance gating, and Point One P1
time mapping at the edge so GLIM/GICP consume a stable, vendor-neutral contract:
`/gps_p1/*` (and optional `/gnss*`) already in the **local ENU** `map` frame.
The raw Luminar LiDAR topics are **not** touched by the adapter — they are copied
through unchanged by `prep_bag.py`, and multi-LiDAR merge / timestamp handling
stays owned by GLIM/GICP `lidar_concat`. See the [root README](../README.md) for
the full pipeline and the local-ENU coordinate contract.

This package normalizes Point One Atlas inputs for the default DLIO
mapping/localization contract:

```text
Atlas pose:  WGS84 LLA
adapter:     LLA -> local ENU using one configured origin
/gnss:       PoseWithCovarianceStamped in map/local ENU
GLIM/GICP:   consume local ENU directly
```

The adapter no longer publishes a map-frame bridge topic from a transform
sidecar. `map` is the local ENU frame.

## AV-24 LiDAR quality preflight

Before launching GLIM or GICP, verify that the front, left, and right Luminar
topics each provide at least 30 degrees of vertical scan coverage:

```bash
ros2 launch adapter av24_lidar_quality.launch.py
```

The check uses the Luminar `elevation` field for every commanded ray and
fails closed if a topic is missing, the field is invalid, or any checked frame
is below 30 degrees. For an offline bag audit:

```bash
ros2 run adapter lidar_fov_quality_check.py \
  --bag <bag-or-mcap> --samples 30 --windows 10 \
  --report-json lidar_fov_report.json
```

To start another command only after the live preflight passes:

```bash
ros2 run adapter lidar_fov_quality_check.py \
  --exec -- ros2 launch gicp_plusplus localization_with_tf.launch.py \
  map_path:=<local-enu-map.pcd>
```

Exit codes are 0 for pass, 2 for sensor-quality rejection, and 3 for
topic/bag/access errors. See the
[Putnam/Laguna audit](../docs/putnam_laguna_lidar_fov_audit.md) for measured
results.

## Outputs

- `/gnss` (`geometry_msgs/msg/PoseWithCovarianceStamped`): continuous Atlas INS
  pose in local ENU, `header.frame_id="map"`.
- `/gnss_rtk_fixed` (`geometry_msgs/msg/PoseWithCovarianceStamped`): same pose
  type, only when the configured covariance/RTK gate passes.
- `/gps_p1/filtered_odom` (`nav_msgs/msg/Odometry`): compatibility odom with
  the same local ENU pose and `child_frame_id="gps_antenna_top"`.
- `/gps_p1/filtered_odom_rtk_fixed` (`nav_msgs/msg/Odometry`): gated
  compatibility odom.
- `/gps_p1/imu` (`sensor_msgs/msg/Imu`): retimed Atlas IMU.

Set `publish_gnss_pose=false` when a prep/recording pipeline only needs the
`/gps_p1/*` compatibility streams and should not create `/gnss*` publishers.

## IMU frame

`/gps_p1/imu` is stamped with `imu_frame_id` (default `gps_antenna_top`). The
adapter overwrites the incoming `frame_id`, so the PCAP replay node's placeholder
(`p1_imu_pcap_frame_id`, default `cg`) is relabeled to `imu_frame_id` on
republish. This relabel is **correct and safe** — verified against the Point One
FusionEngine Message Specification v0.21, §3.4.1 IMUOutput (11000):

> "corrected for estimated accelerometer and gyro errors, including biases and
> scale factors, and has been **rotated into the vehicle body frame** from the
> original IMU orientation."

Key point: IMUOutput is **rotated** into body axes but **not** lever-arm-projected
to the antenna (the spec keeps Device 0x10, GNSS 0x12, and Output 0x13 lever arms
as distinct config items; the Output Lever Arm re-points the *pose/position*
output, not the IMU accel stream). The live `/atlas/imu_calibrated` topic carries
this **same** IMUOutput(11000) message, so the live and PCAP IMU streams are the
identical physical quantity (device-located, body-axis-rotated). The relabel is
therefore consistent live-vs-replay — not a mismatch.

Residual modeling note (pre-existing, shared by GLIM + GICP, **not** adapter-
specific): tagging the device-located IMU as `gps_antenna_top` with an identity
IMU→base transform drops the ~0.8 m device→antenna accelerometer lever-arm term
(`ω×(ω×r)+α×r`). This is negligible at slow mapping speeds (~0.2 m/s² at
0.5 rad/s) and is absorbed by the accel-bias estimator plus scan matching; the
gyro is unaffected (rigid body). Model it only if pushing high-yaw-rate segments.

## Origin

Exactly one origin source must be configured:

- `local_enu_origin: "lat,lon,alt"`
- `local_enu_origin_ttl_path: "/path/to/ttl.csv"`

The TTL parser reads the first non-empty CSV row and uses its last three fields
as `(lat, lon, alt)`, matching the race metadata convention. The checked-in
default is the shared Putnam map/local ENU origin from
`race_metadata/ttls/PUTNAM_ENU_TTL_CSV`:

```text
39.58227391,-86.74232215,260.4
```

## Example

```bash
ros2 launch adapter adapter.launch.py \
  p1_imu_pcap_path:=/path/to/ins_*.pcap \
  local_enu_origin_ttl_path:=/path/to/race_metadata/ttls/PUTNAM_ENU_TTL_CSV/ttl_2.csv
```

The launch file overrides the YAML default origin when
`local_enu_origin_ttl_path` is passed. The node fails at startup if both origin
sources are set or both are empty.
