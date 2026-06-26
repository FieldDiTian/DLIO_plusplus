# DLIO++

ROS 2 perception stack for the AV-24 Cybertruck autonomous race car. Pairs a GPU-accelerated LiDAR-inertial SLAM front end with a map-based localizer so the vehicle can build a map offline, then localize against it online at IMU rate.

## Packages

| Package | Upstream | Purpose in this fork |
|---|---|---|
| [`GLIM/`](GLIM/) | [`koide3/GLIM`](https://github.com/koide3/glim) (+ `glim_ext`, `glim_ros2`) | LiDAR-inertial SLAM. Builds a 3D map from IMU + multi-LiDAR + GNSS. |
| [`gicp_localization/`](gicp_localization/) | Vendored from the `vectr-ucla` DLIO line (uses `nano_gicp`) | GICP scan-to-map localization against a PCD map produced by GLIM. |
| [`dlio/`](dlio/) | new in this repo | Convenience metapackage that pulls both packages into a single colcon build. |

Each subpackage has its own README (`GLIM/README.md`, `gicp_localization/README.md`) covering installation, configuration, and per-knob tuning. **This top-level README focuses on what we changed versus upstream and why.**

## Sensor / Vehicle Target

The configs target an AV-24 Cybertruck instrumented with:

- **3× Luminar Iris LiDAR** — `luminar_front` is the primary sensor; `luminar_left` and `luminar_right` are concatenated into the primary cloud via URDF transforms.
- **Point One Atlas (LG69T) INS** publishing IMU on `/gps_p1/imu` (`imu_calibrated`: sensor-level bias/scale/misalignment removed by FusionEngine firmware, gravity present, no fused orientation) and odometry on `/gps_p1/filtered_odom`. Atlas firmware projects both the IMU and the INS pose to the primary antenna phase centre, so the URDF link `gps_antenna_top` is used as both `base_frame` and `imu_frame` in the localization config. RTK quality is gated on the Atlas-reported pose covariance.
- **RTK GPS** — the FusionEngine INS itself; no separate raw RTK topic is needed for localization.
- Optional camera (used only by extension modules).

All sensor extrinsics are resolved at runtime from [`av24.urdf`](av24.urdf); the `*_frame` strings in the configs are URDF link names, not free-form labels.

Current localization scope is intentionally single-source Point One Atlas. Earlier
project notes mention NovAtel and VectorNav GNSS integration, but
`gicp_localization` no longer subscribes to either; adding them back is future
work and needs a fresh source-selection and fix-status design rather than a
topic remap.

### GNSS lever-arm policy

Atlas firmware compensates the IMU-to-antenna lever arm internally, so the software side stays **off** to avoid double-compensation. The disable is explicit in three independent places — any one is sufficient:

1. **Config flag** — `GLIM/glim_ext/config/config_gnss_global.json` sets `"enable_lever_arm": false`. This is the grep-able single source of truth.
2. **Empty antenna frame** — same file sets `"urdf_gnss_frame": ""`. With this empty, the URDF lookup is skipped and `t_imu_gnss` stays zero even if the flag check were bypassed.
3. **Module not loaded** — `GLIM/glim/config/config_ros.json` keeps `libgnss_global.so` commented out of `extension_modules`, so the code path does not execute in the current INS-based mapping pipeline.

To verify the disable in one command:

```bash
grep enable_lever_arm GLIM/glim_ext/config/config_gnss_global.json
# expected: "enable_lever_arm": false,
```

If the module ever loads with this config, it logs `lever-arm compensation explicitly disabled via gnss.enable_lever_arm=false; t_imu_gnss=0` at startup. If the GNSS extension is ever turned back on, verify the receiver's `LEVERARMCONFIG` state first and flip the flag accordingly — only one side should be doing the correction.

### Recovery during GICP failures

In low-feature stretches the localizer first falls back to IMU dead-reckoning. If GICP keeps rejecting, the node snaps pose and velocity to the latest Atlas INS sample that passed the pose-covariance quality gate. If the gate rejects (Atlas covariance above the configured thresholds), GT samples are dropped and the node stays on IMU dead-reckoning until either LiDAR geometry or RTK quality recovers.

### Initialization: RTK-driven IMU calibration

By default the localizer uses the post-gate Atlas GT odom stream to calibrate gyro/accel biases while the vehicle is moving, and seeds pose+velocity from the first high-quality sample rather than assuming the vehicle is stationary. Falls back to the legacy stationary calibration if no gated GT odom is received within a configurable timeout. With `localization/rtk_gate/enable=true`, the localizer inspects `pose.covariance` on every `/gps_p1/filtered_odom` sample and drops anything that exceeds `max_pose_var_xy` / `max_pose_var_z`. Knobs live under `localization/rtk_init/*` and `localization/rtk_gate/*` in the localization yaml.

### GLIM mapping init — two sequenced conditions, both satisfied in the park position

> The two conditions are **sequenced, not conflicting**. Point One Atlas's
> dual-antenna LG69T resolves RTK FIXED + INS attitude *at standstill* —
> heading comes from the antenna baseline, no motion required for either —
> so a single parked phase satisfies both.

**🅐 PHASE 1 — Stationary calibration of the GLIM odometry estimator.**

> **PARK THE VEHICLE LEVEL. KEEP IT STATIONARY FOR AT LEAST 5 SECONDS AFTER LAUNCHING GLIM.**

GLIM's `LOOSE` init (`config_odometry_gpu.json`: `initialization_mode=LOOSE`, `initialization_window_size=5.0`) collects 5 s of IMU + LiDAR, then runs a batch optimization that estimates the **gravity direction** by averaging the normalized IMU specific-force vector across the window. The math assumes mean acceleration ≈ gravity, which is exact at standstill. The result is locked: `fix_imu_bias: true` freezes the IMU bias at whatever the init optimizer landed on. Aggressive accel during this phase tilts the gravity estimate and rotates the map for the rest of the session — a restart is the only fix.

**🅑 PHASE 2 — RTK-FIXED anchor on the first map frame (no map data is committed before this).**

> **DO NOT BEGIN DRIVING UNTIL `rtk_fixed_odom_filter.py` HAS LOGGED `RTK transition: … -> FIXED` AT LEAST ONCE.**

The pre-filter (`gicp_localization/scripts/rtk_fixed_odom_filter.py`) gates `/gps_p1/filtered_odom` on Atlas's pose covariance and forwards only RTK-FIXED-quality samples to `libgnss_global.so`. The first forwarded message becomes the first GNSS-position prior factor on the iSAM2 graph. **The map's first geo-referenced frame must therefore be anchored to a FIXED Atlas pose — not to a degraded RTK-FLOAT or GPS-only fallback.** Begin driving before that, and the early map segment grows in a local odom frame and only retroactively aligns to global when RTK eventually lands — which iSAM2 will smooth, but the map is no longer guaranteed to start from cm-level absolute coordinates.

**Why the conditions sequence cleanly (and why they don't conflict):**

A single-antenna INS receiver that aligns its heading from motion would create a real conflict with Phase 1's stationary requirement. The dual-antenna Atlas does not: heading is observable from the antenna baseline at standstill, and RTK position fixing depends only on satellite geometry + base-station correction, also fine at standstill with clear sky. In practice the operator parks once, Phase 1 completes during the LOOSE init window, and Phase 2 satisfies itself within 30 s – 2 min as Atlas reaches RTK FIXED.

**Operator sequence:**

1. Park the vehicle level at the intended map origin, with clear sky view.
2. Power Atlas; wait for its status display to read RTK FIXED + INS aligned.
3. Launch `rtk_fixed_odom_filter.py`; verify `First INS sample received … -> FIXED` in its log.
4. Launch GLIM; wait for the 5 s LOOSE init to complete — look for `estimate initial IMU state` and the first sub-map appearing in the viewer.
5. Confirm `gnss_global` has logged its first prior-factor insertion.
6. **Only now begin driving.**

Steps 3–5 happen concurrently inside the parked 30 s – 2 min RTK acquisition window; no extra wait is added by Phase 2 in normal operation. If Atlas never reaches FIXED while parked, that's a hardware/sky-view problem to resolve before driving — it should not be papered over by starting GLIM and "hoping" RTK lands later.

Neither phase applies to `gicp_localization` — that pipeline does RTK-driven IMU calibration while the vehicle is moving and snaps from a single RTK-FIXED GT sample. Only GLIM mapping needs the two-phase sequenced startup.

### Remaining tuning work

- **LiDAR-specific hyperparameter tuning.** Motion-model tuning is in place for race-car dynamics, but the lidar density/range parameters (preprocessing downsample target, voxel-resolution fade horizons) are still close to GLIM's defaults, which were chosen for a lower-density rotary lidar at indoor-to-short-outdoor ranges. Retuning these for Luminar is on the TODO list; current values are workable but not optimal.

### Diagnostic: silent IMU subscription failures

When the `imu_topic:=` launch arg points at a non-existent topic, the subscription is created but no callback fires and historically there was nothing in the log to explain it. The localizer now runs a periodic health check that warns (every 3 s, until the first IMU arrives) with the resolved topic name and whether 0 publishers exist — surfacing the typo case immediately. The timer self-cancels on first IMU receipt.

### Per-point timestamp formats and motion deskewing

Both GLIM and `gicp_localization` deskew each LiDAR scan to compensate for vehicle motion across the scan duration. Deskewing reads the scan's per-point timestamp field, interpolates the IMU-propagated pose for each point's capture instant, and projects every point into a single common time. At race speeds (30 m/s) this can be the difference between a 30 cm scan-end smear and a clean point.

**Deskewing is ON in both stacks** (`dlio/deskew: true` in `gicp_localization/cfg/localization.yaml`; `autoconf_perpoint_times: true` + `autoconf_prefer_frame_time: false` in `GLIM/glim/config/config_sensors.json`).

#### Luminar Iris per-point timestamps — the definitive account

This is the authoritative description; if other comments disagree, this section and the validated code win.

**On-the-wire format (Luminar Iris Data Output Specification v1.3.0).** The Iris does *not* emit a single 64-bit timestamp on the wire. It splits the PTP time across two places: **48-bit integer epoch seconds in the packet header** (§2.1, `UQ48.0`) and a **32-bit sub-second nanosecond count per ray** (§2.2 / §2.6.3, `UQ32.0`) that wraps every 1 s; all fields little-endian. The ROS2 driver reconstructs these into one **little-endian `uint64` of full epoch nanoseconds** per point and publishes it as PointCloud2 field `timestamp` (`datatype=UINT8`, `count=8`, `offset=0`, `point_step=56`). *(Note: the previously cited "Iris PIG R2.0.7 §7.8.4" is PTP Troubleshooting, not the data layout — the Data Output Spec above is the real source.)*

**Validated (May-26 `run_5`/`run_3` bags).** All three Luminar topics expose that exact schema; the bytes decode as `uint64` epoch ns (e.g. `1779827344001041615` → 2026-05-26T20:29:04Z), intra-scan span ≈ **48.997 ms**, second rollovers safe, no collapse. See `GLIM_GICP_Luminar_Timestamp_Validation.pdf` (procedure) and the validation report for evidence.

**GICP deskew — robust by construction.** `copyPointTimeFromCloud` (LUMINAR case in `localization.cc`) stores the raw `uint64` ns; `deskewPointcloud` then computes each point's capture time as

```
t_point = scan_stamp.seconds()  +  (ts - anchor_ts) * 1e-9
          └── header anchor ──┘     └── intra-sweep relative offset (signed) ──┘
```

It uses **only the relative offset within the sweep, anchored at the header stamp** — it never trusts the absolute epoch of `ts`. `anchor_ts` is the **earliest timestamp of the PRIMARY scan**, captured in `mergeAuxClouds()` before any aux cloud is appended (for a single-sensor scan this is just that scan's own minimum). Anchoring on the primary — rather than the global merged minimum — keeps a multi-LiDAR sweep correctly timed when an aux scan started *before* the primary: such aux points get correctly **negative** offsets (hence the **signed** `int64` subtraction), instead of being collapsed onto the header stamp and shifting the whole sweep late. That makes GICP deskew **correct regardless of whether the per-point clock is on the Unix/INS epoch or a sensor-local/PTP axis**. The only way it could break is a driver emitting a bare 32-bit sub-second field that wraps mid-scan; the one-second-boundary check (Procedure C) confirms that does not happen. This is why GICP needs no timestamp repair and is the more trustworthy pipeline for deskew.

**GLIM deskew — correct, but requires epoch alignment.** `ros_cloud_converter.hpp` reads `UINT8[8]` as little-endian `uint64` and divides by `1e9` → epoch *seconds* (~1.78e9). `TimeKeeper::replace_points_stamp` then sees `max_time ≥ 1.0`, takes the *absolute → relative* branch, and (with `prefer_frame_time=false`) **overwrites the frame stamp with the first point time** while making per-point times relative; `point_time_scale` stays `1.0`. Because GLIM *trusts the absolute point-time epoch*, that epoch must match the IMU/header epoch. Raw bags were observed with point times on the sensor/PTP axis (~2e13 ns) while the header/IMU were on the ROS/INS epoch — GLIM then overwrote the frame stamp with a sensor-clock value and dropped every scan as unsynchronized. Two complementary repairs close this:

- **Offline:** `scripts/prep_bag.py` rebases each Luminar cloud by `header.stamp − min(point_time)` (span preserved) when building a prepped bag.
- **Live:** `ros_cloud_converter.hpp` applies the same rebase in-pipeline — only when the times are absolute *and* `|header − min| > 1 s` (a no-op on already-aligned/prepped data and on scan-relative sensors).

GICP requires neither because of the header-anchored relative-offset design above.

**Status.** Deskew is validated correct for all bagged/offline data in both stacks. The only open item is a **live-hardware PTP-lock repeat** (no live publishers were available during the final check). That item concerns absolute-epoch / GT time association, **not** GICP deskew geometry, which depends only on the (validated) intra-scan span. Previously both stacks ran with deskew effectively off because this encoding was ambiguous; it no longer is.

**`gicp_localization` supports five sensor-type-driven decoders** (`copyPointTimeFromCloud` in `localization.cc`), selected by `localization/sensor_type` in the yaml:

| `localization/sensor_type` | Field encodings handled | Notes |
|---|---|---|
| `luminar` | `UINT8[8]` (uint64 epoch ns; validated default — field `timestamp`, offset 0, point_step 56), `FLOAT64` (raw uint64 bits in a mislabelled FLOAT64 wrapper) | Iris PTP-synced output, reconstructed to full epoch ns by the driver (see the definitive account above). Only these two **8-byte absolute-epoch** carriers are accepted; `UINT32` is **intentionally rejected** — 32 bits cannot hold an absolute epoch (it wraps every ~4.29 s), so it would be a scan-relative counter the absolute path would misread. A `UINT32` Luminar therefore degrades to no per-point time (rigid transform) rather than corrupting deskew. |
| `ouster` | `UINT32`, `FLOAT32`, `FLOAT64` (all scan-relative ns or s) | Standard Ouster ROS driver layouts. |
| `velodyne` | `FLOAT32`, `UINT32` (scan-relative s or ns) | VLP-16/32 and similar. |
| `hesai` | `FLOAT64`, `FLOAT32` (absolute or relative seconds) | Pandar / XT line. |
| `livox` | `UINT8[8]`, `UINT32`, `FLOAT64`, `FLOAT32` | MID/HAP/Avia. Both packed uint64 ns and scaled-double conventions. |

That's **5 sensor types × multiple PointField datatypes** per family. Adding a new vendor means extending the `case dlio::SensorType::*` switch with the right `memcpy` and unit conversion — about 10 lines.

**GLIM supports three auto-detected timestamp buckets** (`TimeKeeper::replace_points_stamp` in `glim/src/glim/util/time_keeper.cpp`), one per encoding family:

| Bucket detected from `min/max` per-point time | Source encodings that fall here | What GLIM does |
|---|---|---|
| `max_time < 1.0` | Scan-relative FLOAT seconds (Ouster, Velodyne, Hesai, Livox in their FLOAT modes) | Use as-is. |
| `1.0 ≤ max_time < 1e16` | Absolute epoch **seconds** — Hesai FLOAT64 absolute, **and Luminar Iris** (its `UINT8[8]` ns are divided by `1e9` in `ros_cloud_converter.hpp` *before* TimeKeeper, landing here at ~1.78e9) | Overwrite frame stamp with first point time; treat per-point times as relative seconds (`point_time_scale = 1.0`). |
| `min_time ≥ 1e16` | Raw, *unconverted* 64-bit nanoseconds (e.g. Livox FLOAT64 ns forwarded without scaling) | Apply `1e-9` scale; rebase to first-point time. |

Note: Luminar lands in the **middle** bucket, not the `≥1e16` one, precisely because `ros_cloud_converter.hpp` already applies the `1e-9` divide. The `≥1e16` branch only fires for pipelines that forward raw nanoseconds — which this one never does for Iris.

The combination of `autoconf_perpoint_times: true` and `autoconf_prefer_frame_time: false` makes GLIM use the *per-point* times for deskew. The earlier "Luminar timestamps look collapsed" symptom was the `autoconf_prefer_frame_time: true` default collapsing each scan to its single header stamp — that has been turned off.

Concatenation note: when multiple LiDARs are merged in `lidar_concat`, both stacks now leave Iris's `UINT8[8]` per-point times **unshifted** (they're already absolute capture times). Other encodings (FLOAT32/FLOAT64 scan-relative seconds, UINT32 scan-relative ns) still get shifted by `dt = T_aux − T_primary` so they rebase onto the primary's header. See the comment block on `shiftCloudTimestamps` in `gicp_localization/src/localization.cc` and `shift_cloud_timestamps` in `GLIM/glim_ros2/include/glim_ros/lidar_concat.hpp`.

If you ever switch sensors and the deskew looks wrong, run the one-shot diagnostic in `gicp_localization` (always-on; emits a `[LUMINAR_TS_DIAG] BEGIN ... END` block on the first PointCloud2 message of each session, see `gicp_localization/docs/luminar_timestamp_diagnostic_guide.pdf`) — it dumps the per-point time field metadata + raw bytes interpreted four ways so you can decide which decoder branch to take.

## Workflow

1. **Record** a bag containing IMU + LiDAR + GNSS topics during a driving session.
2. **Map** offline with GLIM:
   ```bash
   ros2 run glim_ros glim_rosbag <bag_path> --ros-args -p dump_path:=/tmp/dump
   ```
   Outputs `graph.bin`, `traj_lidar.txt`, `odom_lidar.txt`, numbered submap point clouds, and `T_world_utm.txt` (GNSS-to-map SE(3)) into `dump_path`.
3. **Convert** submaps into a single PCD map by opening the dump in `glim_ros offline_viewer` and exporting to PLY (then to PCD via `gicp_localization/scripts/convert_ply_to_pcd.py`). The GUI step is **intentional, not a gap** — see "Why the offline_viewer step is manual" below.
4. **Localize** online against that PCD map with `gicp_localization`. Point the launch file at the PCD and (optionally) the matching `T_world_utm.txt`.

### Why the offline_viewer step is manual

A reviewer reasonably asks: why not auto-merge the per-submap directories into a single PCD with a script? Because the viewer pass is the QA stage for the mapping output, and skipping it would silently push bad maps into the localizer:

- **Visual inspection** of the assembled map before it's frozen as the localization reference catches drift, ghosting, and bad submaps that would otherwise propagate into GICP at runtime.
- **Post-hoc global optimization** — the viewer prompts "Do optimization?" on load (see `offline_viewer.cpp:191`) and re-runs the iSAM2 backend over the full graph, which can improve the dump beyond what the online pass produced.
- **Manual loop closure** — `manual_loop_close_modal` lets the operator add constraints when the automatic detector misses a loop (common on long highway runs with weak geometry).

A blind `merge_glim_submaps.py` would skip all three and bake any unresolved drift into the PCD. Adding such a script as a dev-only "quick-look" mode is reasonable, but it must not become the default mapping→localization handoff.

## Build

ROS 2 Humble + colcon. Built and tested inside an Ubuntu 22.04 distrobox (`distrobox enter ros2-humble`).

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Headline dependencies (per-package READMEs go deeper):

- GTSAM 4.2, gtsam_points (GPU factors), Eigen3, PCL, OpenMP, nlohmann::json, spdlog
- Optional: CUDA 11.8+ (GPU acceleration), Iridescence (viewer), OpenCV

If `ros2 pkg prefix glim` does not point inside this workspace's `install/`, an apt-installed `ros-humble-glim-*` package is being picked up instead of this fork — re-source `install/setup.bash` **after** `/opt/ros/humble/setup.bash`. The same caveat applies to `gicp_localization` if a sibling workspace is also sourced.

## Quick Reference

```bash
# Live SLAM with real sensors
ros2 launch glim_ros glim_ros.launch.py config_path:=config

# Offline bag → map (ROS 2 mcap input)
ros2 run glim_ros glim_rosbag <bag_path> --ros-args -p dump_path:=<out_dir>

# Offline pcap → map (raw Luminar pcap + IMU/GNSS from a sibling mcap)
ros2 run glim_ros glim_pcap_rosbag <pcap_dir> <mcap_bag> --ros-args -p dump_path:=<out_dir>

# Inspect a saved map
ros2 run glim_ros offline_viewer

# GICP localization against a pre-built PCD map
# (single-source P1 design: IMU + GT odom both from Atlas, at gps_antenna_top)
ros2 launch gicp_localization localization_with_tf.launch.py rviz:=true \
    pointcloud_topic:=/luminar_front/points \
    imu_topic:=/gps_p1/imu \
    gt_odom_topic:=/gps_p1/filtered_odom
```

---

## Key Changes vs. Upstream

The two packages started from different upstream codebases and diverged for different reasons. This section summarizes the substantive deltas — small config tweaks, log-level changes, and routine refactors aren't enumerated here; consult `git log` for the exhaustive list.

### GLIM (vs. `koide3/glim`, `glim_ext`, `glim_ros2`)

Upstream GLIM publishes `glim`, `glim_ext`, and `glim_ros2` as three sibling repos. This fork keeps them together inside `DLIO_plusplus/GLIM/` and adds:

**Sensor / preprocessing**

- **Multi-LiDAR concatenation (`lidar_concat`).** New module in `glim_ros2` (`include/glim_ros/lidar_concat.hpp`) that subscribes to N aux LiDAR topics, time-aligns each scan to the primary clock, transforms aux points into the primary frame via URDF, **rebases per-point timestamps** so the concatenated cloud has a single monotonic time base, and emits a single merged cloud to the rest of the pipeline. Includes a validation step that **rolls back the aux-merge append** if the merged cloud fails sanity checks, instead of letting a malformed cloud poison odometry (commit `52f88cb`).
- **URDF-based extrinsic resolution.** Sensor extrinsics (`T_lidar_imu`, inter-LiDAR transforms, IMU↔GNSS) are read from a runtime URDF instead of hand-edited JSON. The relevant configs (`config_sensors.json`) reference *URDF link names*; the loader walks the URDF at startup. Removes the previous hard-coded URDF path.
- **`flip_points_y` preprocessing** flag (`config_sensors.json` → `glim_ros.cpp`) for mirrored-installed LiDARs.
- **Per-point timestamp rebasing fix** when merging multi-LiDAR clouds (commit `7f5a6d9`). Without this the merged cloud had a non-monotonic stamp field that broke deskewing.

**Mapping / odometry**

- **INS-driven odometry mode** for sparse-feature stretches (commit `c26c8b0`). Lets the optimizer lean on INS odometry when LiDAR geometry is degenerate (e.g. open sky and runway-like surfaces).
- **Race-car drift tuning** in `glim/config/` (commit `20ba88d`). Defaults relaxed to admit higher angular rates and lateral slip than the road-car defaults assume.

**GNSS extension (`glim_ext/modules/mapping/gnss_global`)**

- **`T_world_utm.txt` export** of the recovered GNSS-to-map SE(3) once GNSS alignment initializes. Downstream localizers (including `gicp_localization` here) consume this file to publish poses in a `utm` frame in addition to `map`.
- **URDF lever-arm support** (commit `50ae6ae`/`50ae...50a...50aa50a` — see `git log`): the IMU→GNSS lever-arm is taken from the URDF rather than from a manual offset in the config.
- **Orientation prior** mode: optionally constrain map yaw directly from GNSS heading.
- **Strip stale GNSS rotation priors on graph reload** (commit `6a50632`) so a re-opened graph doesn't double-apply an orientation constraint that no longer matches the live frame.
- **Warn when URDF IMU↔GNSS rotation breaks the lever-arm assumption** (commit `622271f`). The lever-arm math assumes IMU and GNSS share orientation; if the URDF says otherwise the user is told instead of silently getting biased corrections.
- Switched the noise model expression from `Isotropic::Information(diagonal)` (which silently dispatched to `Gaussian::Information(Matrix)` through inheritance) to `Diagonal::Precisions(vector)` (commit `d8b2809`). Same numerical result, more honest signature — see `AGENTS.md §1` for the reasoning trail.

**Offline tooling**

- **`glim_pcap_rosbag`** (`glim_ros2/src/glim_pcap_rosbag.cpp` + `iris_pcap_reader.cpp`). Reads raw Luminar `.pcap` files alongside a sibling mcap (for IMU and GNSS) and runs offline mapping directly, skipping the intermediate "decode pcap into a bag" step. Useful when the live-recorded mcap is missing LiDAR or had a decode hiccup.

### gicp_localization (vs. the vectr-ucla DLIO line)

`gicp_localization` is built around a vendored `nano_gicp`. Compared to a stock DLIO odometry node turned into a localizer, this fork adds:

**Scan-to-map vs. scan-to-submap**

- **Single pre-built PCD map.** No submap stitching at runtime — the map is loaded once and never grows. Trades adaptability for a small, predictable working set.
- **Multi-LiDAR concatenation** (mirrors the GLIM-side feature). Subscribes to N aux LiDARs, transforms via URDF, concatenates onto the primary cloud's clock.

**Robustness against degenerate geometry**

- **Layered rejection gates** on every GICP solve:
  1. Hard fitness reject (`gicp/fitnessRejectThreshold`).
  2. **Combined hessian-degeneracy gate.** Fires only when the hessian condition number is high *and* one of `fitness`/`trans`/`rot` warn floors is crossed — high hessian alone is fine if GICP barely moved, but high hessian combined with a large correction is the slide-along-unconstrained-axis signature (commit `4a594a9`).
  3. Large-jump reject vs. the IMU-predicted prior.
- **IMU dead-reckoning fallback.** Rejected scans propagate from the IMU-integrated prior, not by freezing at the last accepted pose — transient corner failures don't cascade into a stuck pose.
- **GT-driven pose recovery** (optional, off by default). When GICP rejects N scans in a row, optionally snap pose + velocity to a time-matched GT odom sample (composed through TF into `base_frame`) so GICP can re-acquire from a known-good state (commit `83b48a4`).
- **`getFitnessScore` correctness fixes.** Cleared `sq_distances_` per align so stale distances couldn't leak into the score (commit `d949e64`); cached `sq_distances_` reused inside `NanoGICP::getFitnessScore` to avoid recomputing nearest neighbors (commit `1e27b84`).

**Geometric observer**

- **Observer + IMU pipeline aligned to upstream DLIO design** (commit `db5cda8`). The original lift-and-shift had subtle differences in how the geometric observer was driven; this commit brings the data path back in line with DLIO's reference implementation so IMU dead-reckoning is mathematically consistent with scan corrections.

**Initialization**

- **GT-bootstrapped initial pose** (commit `add7a54`). The first message on the GT odom topic seeds the localizer; works for any bag start-offset without hand-tuning numerics or clicking in RViz.
- Three init paths in priority order: GT bootstrap → numeric pose from YAML (with `frame: "lidar"` mode that auto-applies `inv(T_base_lidar)` for direct pasting from GLIM's `traj_lidar.txt`) → RViz "2D Pose Estimate".

**Output frames**

- **UTM-frame publishing.** If `T_world_utm.txt` (from the GLIM run that built the map) is configured, the node publishes pose/odom/path in a `utm` frame alongside `map`. Lets downstream consumers consume world-referenced poses without re-deriving the transform.
- **TF policy:** the `map → base_link` TF broadcast is disabled by default (commit `9cc18d7`) to avoid fighting other publishers; downstream nodes consume the published `nav_msgs/msg/Odometry` instead.

**Operational defaults**

- Verbose logging, debug topic publication, per-scan jump/scan logs, and outgoing point-cloud topics are all **off by default** (commits `d492a69`, `b5116d3`, `100011c`). The pipeline is quiet and lean unless you explicitly enable diagnostics.

---

## Repo Layout

```
DLIO_plusplus/
├── GLIM/                # SLAM workspace (glim, glim_ext, glim_ros2)
├── gicp_localization/   # Map-based localization package
├── dlio/                # Convenience metapackage
├── av24.urdf            # Vehicle URDF (drives all sensor extrinsics)
├── scripts/             # Bag-prep, map-merge, and analysis helpers
├── profiling_logs/      # Resource-profile CSVs + comparison plots
├── CLAUDE.md            # Developer-facing project summary
├── AGENTS.md            # Notes for AI reviewers (false positives, watch-conditions)
└── README.md            # This file
```

## License

GLIM and `gtsam_points` are MIT-licensed; GTSAM is BSD. See the upstream repositories and `GLIM/README.md` for full attributions.
