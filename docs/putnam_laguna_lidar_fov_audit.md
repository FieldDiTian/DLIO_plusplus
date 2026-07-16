# Putnam Park and Laguna Seca Luminar vertical-FOV audit

Date: 2026-07-16

## Acceptance rule and measurement

GLIM and GICP require every primary or auxiliary AV-24 Luminar cloud to cover
at least **30.0 degrees vertically**. The audit measures
`max(elevation) - min(elevation)` from the PointCloud2 `elevation` field.
Luminar publishes this field in radians for every commanded ray, including
zero-return rays, so it measures the configured scan pattern. Inferring FOV
from nonzero XYZ returns is not equivalent and is disabled by default.

Each result below samples 30 frames per LiDAR across 10 uniformly distributed
time windows. A bag is usable only when every checked frame on every required
topic passes.

## Putnam Park result

The only mounted Putnam raw bags were:

- `may_26/run_3/filtered/all`
- `may_26/run_5/filtered/all`

| Bag | LiDAR | Minimum | Median | Maximum | Passing frames |
| --- | --- | ---: | ---: | ---: | ---: |
| Run 3 | front | 27.256° | 27.277° | 27.291° | 0/30 |
| Run 3 | left | 26.361° | 27.398° | 27.448° | 0/30 |
| Run 3 | right | 27.354° | 27.394° | 27.427° | 0/30 |
| Run 5 | front | 26.217° | 27.261° | 27.277° | 0/30 |
| Run 5 | left | 20.346° | **27.410°** | **27.469°** | 0/30 |
| Run 5 | right | 26.273° | 27.399° | 27.420° | 0/30 |

**Conclusion:** none of the mounted Putnam Park bags is usable under the
30-degree requirement. Run 5 is the widest available candidate by
time-distributed median and observed maximum (left Luminar), but it still
fails. Run 5's 20.346-degree minimum is an additional intermittent-quality
warning. Putnam should be re-recorded after the AV-24 preflight passes.

Machine-readable reports:

- `putnam_run3_lidar_fov.json`
- `putnam_run5_lidar_fov.json`

## Laguna Seca result and blocker

No Laguna Seca rosbag, MCAP, or raw LiDAR capture is mounted under the
accessible `/media` data roots. A connected Google Drive search for
`Laguna Seca rosbag`, `Laguna rosbag`, and `laguna mcap` returned no
rosbag. The only Laguna hit, `0617.md`, explicitly says the author did not
have Laguna Seca data and could not build the map. The source email/thread
contains no attached Laguna bag and asks for new track data.

Therefore no Laguna FOV can be claimed from current evidence. A Laguna rosbag
path or ART NAS/Drive access to the raw three-Luminar topics is required.
When supplied, run the same command below before GLIM or GICP.

## Audit and preflight commands

Offline bag audit:

```bash
ros2 run adapter lidar_fov_quality_check.py \
  --bag <bag-or-mcap> \
  --samples 30 --windows 10 \
  --min-fov-deg 30 \
  --report-json lidar_fov_report.json
```

Live AV-24 preflight:

```bash
ros2 launch adapter av24_lidar_quality.launch.py \
  min_vertical_fov_deg:=30.0 \
  samples_per_lidar:=3
```

To guarantee that a downstream launch is not started until all three LiDARs
pass, keep the sensor drivers running and use the preflight's exec handoff:

```bash
ros2 run adapter lidar_fov_quality_check.py \
  --samples 3 --timeout-sec 15 --min-fov-deg 30 \
  --exec -- ros2 launch gicp_plusplus localization_with_tf.launch.py \
  map_path:=<local-enu-map.pcd>
```

Exit code 0 means every required sample passed. Exit code 2 is a sensor-quality
rejection; exit code 3 is a missing topic, unreadable bag, or other access
failure. GLIM and both GICP implementations also enforce the same rule at
runtime and exit 2 on the first invalid or sub-30-degree primary/auxiliary
scan. GLIM does not save a partial map after a quality failure.
