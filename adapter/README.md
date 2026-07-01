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
