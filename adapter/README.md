# Atlas Adapter

The Atlas adapter is the **normalization boundary** (stage 1) of the DLIO++
pipeline:

```
adapter → prep_bag.py (normalize + map by default) → GLIM dump → ENU PCD → localizer
```

It keeps sensor-specific WGS84 parsing, RTK covariance gating, and Point One P1
time mapping at the edge so GLIM/GICP consume a stable, vendor-neutral contract:
`/gps_p1/*` (and optional `/gnss*`) already in the **local ENU** `map` frame.
The raw Luminar LiDAR topics are **not** touched by the adapter — they are copied
through unchanged by `prep_bag.py`, and multi-LiDAR merge / timestamp handling
stays owned by GLIM/GICP `lidar_concat`. The adapter's output contract is one
local-ENU `map` datum shared by mapping and either online localizer
(`gicp_localization` or `gicp_plusplus`).

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

Configure **at most one** origin source:

- `local_enu_origin: "lat,lon,alt"`
- `local_enu_origin_ttl_path: "/path/to/ttl.csv"`

The TTL parser reads the first non-empty CSV row and uses its last three fields
as `(lat, lon, alt)`, matching the race metadata convention. The checked-in
default is the shared Putnam map/local ENU origin from
`race_metadata/ttls/PUTNAM_ENU_TTL_CSV`:

```text
39.58227391,-86.74232215,260.4
```

If neither parameter is supplied, the node warns and uses that built-in Putnam
origin. The checked-in launch configuration also supplies the same inline value.
Supplying both sources is a startup error.

## Example

```bash
ros2 launch adapter adapter.launch.py \
  p1_imu_pcap_path:=/path/to/ins_*.pcap \
  local_enu_origin_ttl_path:=/path/to/race_metadata/ttls/PUTNAM_ENU_TTL_CSV/ttl_2.csv
```

The launch file overrides the YAML inline origin when
`local_enu_origin_ttl_path` is passed. It rejects both sources being set; with
neither source set, the node uses the documented built-in origin above.

## Run summary and audit counters

The adapter logs (and optionally writes via `summary_output_path`) a one-line
summary designed so a run report can **prove** zero data loss instead of
inferring it from matching in/out totals:

```text
pose_in=… gnss_out=… gnss_rtk_out=… odom_out=… rtk_out=… imu_in=… imu_out=…
pose_dropped_invalid=… imu_dropped_invalid_stamp=… imu_sidecar_miss_drop=… imu_dropped_clock_not_ready=…
p1_clock_ready=… p1_clock_drift_ms=…
```

- `pose_dropped_invalid` — NaN / invalid-solution FusionEngine poses rejected
  before publication (cold-start samples land here); also counts poses dropped
  for an invalid or quarantined-forward-spike P1 stamp.
- `imu_dropped_invalid_stamp` — IMU samples dropped at ingest for a non-finite /
  ≤0 / ≥4e9 (sentinel) header stamp, before they can reach a consumer buffer.
- `imu_sidecar_miss_drop` — IMU samples dropped because no sidecar P1 stamp
  matched within tolerance (sidecar replay mode only).
- `imu_dropped_clock_not_ready` — IMU samples dropped from the bounded
  not-ready queue before the P1→ROS clock mapper initialized.
- `p1_clock_ready` / `p1_clock_drift_ms` — end-to-end P1→ROS retiming
  evidence: the mapper reached readiness, and the measured offset drift over
  the run. Together with GLIM's `gnss_global summary` line (bracket widths,
  gap/non-monotonic rejections, factor counts) these close the RTK timing
  audit chain from PCAP to map factors.

All four drop counters (`pose_dropped_invalid`, `imu_dropped_invalid_stamp`,
`imu_sidecar_miss_drop`, `imu_dropped_clock_not_ready`) are expected to be **0**
on a healthy run; the sidecar match/miss/skip triple is additionally printed in
sidecar mode.

## Startup validation (fail loud, not degraded)

Bad parameter overrides refuse to start rather than silently degrading
retiming: `nominal_imu_period_sec`, `imu_flush_timeout_sec`,
`p1_like_threshold_sec`, and `imu_p1_sidecar_match_tolerance_sec` must be
finite and positive (a zero flush timeout strands the arrival-retime queue; a
nonpositive nominal period breaks synthesized spacing). The RTK gate
covariance thresholds require finite, nonnegative values per sample — the
same contract the downstream GICP gate and the legacy
`rtk_fixed_odom_filter.py` now enforce. The PCAP replay node also rejects IMU
samples whose `fraction_ns >= 1e9` (a corrupt fraction would otherwise raise
on ROS timestamp assignment or alias into a wrong stamp), alongside the
existing `0xFFFFFFFF` sentinel rejection.
