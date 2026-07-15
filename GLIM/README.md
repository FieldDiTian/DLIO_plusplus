# GLIM ROS2 Workspace

ROS2 workspace for **GLIM** (Graph-based LiDAR-Inertial Mapping) maintained as an `airacingtech` fork of the upstream `koide3/GLIM` project family.

## Overview

This directory is the GLIM workspace inside the [`augcog/DLIO_plusplus`](https://github.com/augcog/DLIO_plusplus) monorepo and contains:
- `glim` for the core SLAM framework
- `glim_ext` for extension modules
- `glim_ros2` for ROS2 integration

**Where GLIM sits in the pipeline.** GLIM is the offline-map stage: **adapter** converts Atlas WGS84 to local ENU `/gps_p1/*`; **`scripts/prep_bag.py`** copies raw Luminar clouds, writes the normalized bag, and by default invokes GLIM to create a dump; the dump is exported to an ENU PCD for either online localizer (`gicp_localization` or `gicp_plusplus`). GLIM consumes `/gps_p1/*` already in local ENU and performs no WGS84/UTM projection itself. Pass `--skip-glim` to `prep_bag.py` only if you will invoke `glim_rosbag` manually.

**Target sensor stack:** AV-24 Cybertruck with three Luminar Iris LiDAR (front + left + right concatenated) and the **Point One Nav Atlas (LG69T) dual-antenna RTK-INS**. All GNSS, RTK, and IMU input comes from Atlas. Note the two streams are referenced differently: the INS **pose/position** solution is projected to the primary GNSS antenna phase centre (URDF link `gps_antenna_top`), but the **IMU** is *not* — per FusionEngine, `IMUOutput` is bias/scale-corrected and **rotated into vehicle body axes** yet stays at the physical device link (`pointonenav`), not the antenna. Tagging the IMU as `gps_antenna_top` is a deliberate **approximation** that drops the small (~0.63 m) device→antenna accelerometer lever-arm term (`ω×(ω×r)`, negligible at mapping speeds; gyro unaffected) — it is not a firmware projection. IMU rate is 99 Hz, RTK is delivered at cm-level horizontal / sub-cm vertical when FIXED.

### Differences From Upstream GLIM

Reviewer summary of every functional delta from upstream. Base: **koide3 GLIM ~v1.2.1** (already carries upstream fixes #221, #234, #266, #275, #293 — no porting needed). The checked-in configs/source are authoritative; do not assume upstream defaults. Paths are relative to `GLIM/`.

**Packaging**

- `glim`, `glim_ext`, `glim_ros2` are vendored together in the `DLIO_plusplus` monorepo (not sibling repos, not git submodules). Upstream `glim_ext`'s ScanContext / DBoW / ORB-SLAM3 third-party submodules are intentionally absent (none enabled).

**Sensor & platform adaptation — Point One Atlas LG69T + Luminar Iris, AV-24**

- `glim/config/config_sensors.json`, `config_ros.json`: LiDAR-IMU extrinsic, topics (`/gps_p1/imu`, `/luminar_front/points`), `intensity_field=reflectance`, Atlas-tuned IMU noise (deliberately conservative vs measured stationary noise — see *Key Parameters*). Frame IDs / `acc_scale` auto-detected.
- `glim_ros2/src/glim_ros/glim_ros.cpp`: optional `flip_points_y` for mirrored clouds (config key in `config_sensors.json`).

**Luminar per-point timestamps & deskew** — `ros_cloud_converter.hpp`, `config_sensors.json`

- Decode the Luminar `UINT8[8]` little-endian uint64 **PTP epoch-nanosecond** per-point time field (`/1e9` → epoch seconds); deskew is ON. Upstream handles only `UINT32`/`FLOAT32`/`FLOAT64` and would drop these scans as "unsupported time type". Timestamp-format authority: **Luminar Iris Data Output Specification v1.3.0**.
- **Epoch-axis safeguard**: when absolute per-point times sit on a different epoch than `header.stamp` (sensor clock not PTP-locked to the ROS/INS epoch), `ros_cloud_converter` rebases them onto the header epoch, **anchored on the primary scan** (intra-scan span preserved; threaded `epoch_anchor_count`), so `TimeKeeper`'s absolute-time branch can't overwrite the frame stamp with a wrong-epoch value and drop the scan. This live safeguard is the **sole** timestamp repair — `scripts/prep_bag.py` deliberately copies the raw Luminar clouds byte-for-byte (no offline restamp), so runtime owns epoch alignment. No-op on already-aligned or scan-relative data.
- The converter is the sole timestamp-repair point: `prep_bag.py` copies raw
  Luminar payloads byte-for-byte, while GLIM preserves intra-sweep span and
  rebases only an absolute point-time epoch that disagrees with the header.

**Multi-LiDAR concatenation** — `glim_ros2/include/glim_ros/lidar_concat.hpp`, `glim_ros2/src/glim_ros/glim_ros.cpp`

- Merge 3× Luminar Iris — `luminar_front` **primary** plus `luminar_left` / `luminar_right` aux — into the primary `luminar_front` frame. For Iris `UINT8[8]`, selection is by the decoded absolute per-point interval, not by `header.stamp`: a header delta is acquisition phase and is only a tie-break hint. Offline `glim_rosbag` queues primary scans until every aux can match by point time or its watermark proves that no match can arrive, so a future-arriving sweep cannot be replaced by the previous header-nearest sweep. Absolute times are left unshifted unless an explicit residual clock correction is configured; scan-relative encodings are shifted by inter-scan `dt`.
- **Aux extrinsics are resolved OFFLINE** (no live `/tf_static` needed), in priority order: (1) **URDF** — `av24.urdf`, path from `lidar_concat/urdf_path`, resolved CWD-independently by walking up from the config directory (`av24.urdf` is installed into `share/glim/config`); (2) a **static per-aux 4×4 matrix** in config. There is **no live-TF fallback** (corrected 2026-07-10 — an earlier claim of one did not match any executable path): an unresolvable aux is dropped at startup with an error log, or aborts under the strict merge guard.
- **Strict merge guard** (config in `glim/config/config_sensors.json` under `lidar_concat`; **identical semantics and defaults to GICP**):
  - `require_all_aux` (default **false**) — false = build/localize on whatever LiDARs merged this scan; true = an incomplete merge **skips** the scan entirely rather than emitting a degraded cloud.
  - `abort_on_merge_failure` (default **true**, only relevant when `require_all_aux=true`) — abort the node once past the failure budget vs. keep skipping non-fatally.
  - `max_consecutive_aux_merge_failures: 10`.
  - `time_threshold: 0.1` is the legacy/header window for non-Iris encodings.
  - `luminar_time_threshold: 0.01` gates the difference between decoded Iris point-time interval endpoints.
  - `aux_match_time_offsets` affects only header matching/tie-breaking; `aux_point_time_offsets` is the only setting that changes authoritative absolute point clocks. `aux_time_offsets` remains a deprecated fallback for old configs. **Header phase is not point-clock evidence**: PTP-synchronized Iris units hold a stable 66–92 ms acquisition phase while their absolute point clocks agree to <1 ms — never copy header deltas into the point offsets (the H1 geometric regression measured |offset| < 11 ms; keep `[0.0, 0.0]`).
  - Big-endian PointCloud2 payloads are rejected before matching (the decoder is little-endian; a garbage-decoded range could pass the 10 ms gate by chance), and an aux whose clouds carry no decodable absolute point time cannot merge under a Luminar primary at all — header matching is not a usable fallback because the byte-append merge requires an identical schema (capped warning + `no_absolute_time` plan reason).
  - Startup guards also gate on `require_all_aux && abort_on_merge_failure`.
- **Offline two-pass point-time join** (`two_pass_point_time_join`, default **true**; `glim_rosbag`): pass 1 indexes every LiDAR scan's absolute point-time range and bag location (filtered read, Ctrl-C-able), then plans each primary's right/left selection by **minimum endpoint-range error within `luminar_time_threshold`** — header time only as tie-break, each aux sweep reserved by **at most one** primary (`candidate_reserved` otherwise), plan identity = per-topic **bag-record ordinal** (robust to duplicate/zero header stamps, which are warned loudly). The streaming pass merges each primary exactly when its planned sweeps have arrived (~90 ms read-ahead, well inside IMU coverage); primaries with no in-gate candidate map **front-only immediately** with the miss reason recorded at plan time — **never deferred to EOF**. A pre-flight `two-pass join plan` log summarizes matched / no_candidate / exceeds_gate / candidate_reserved / no_absolute_time per aux before mapping starts. Automatically falls back to the bounded streaming wait (release on match/watermark or after `future_sweep_wait_timeout` = 0.15 s of bag time) under `start_offset`, on index failure, or when <90 % of primaries carry absolute point times. `glim_pcap_rosbag` uses the same bounded queued-primary release policy (its LiDAR originates from PCAP assembly, so no pass-1 bag index is possible).
- **Never-drop-front accounting**: both offline readers log `lidar_concat primary accounting: received=… forwarded=… strict_skipped=… imu_skipped=… …` at end of input and mark the run as a **hard error (nonzero exit)** if the counts fail to reconcile — a front sweep can be skipped only by the explicit `require_all_aux` policy or GLIM ingestion validation, never silently.
- **Primary-anchored epoch handling**: merged-cloud timing anchors on the **primary** scan's earliest timestamp, not the global merged minimum.
- **Full PointCloud2 schema-equality gate** before byte-appending an aux scan (name/offset/datatype/count + point_step + endianness), not just `point_step` — a same-step-but-different-layout aux cloud is now skipped with a diagnostic instead of being silently misread.
- **Per-frame merge diagnostics** (P4, 2026-07; `lidar_concat.frame_diag_log`, default **true**): one parseable `CONCAT DEBUG | stamp=… merged=n/N dt<i>=…s pts<i>=… span=…s total_pts=…` INFO line per primary scan — the map-side merge evidence, mirroring GICP's per-frame debug topics (GLIM's offline tools have no node to publish from). The reported `dt<i>` values are raw header acquisition phase, not clock estimates. Point-time endpoint mismatches and rejected candidates are logged separately; only those decoded point times are authoritative for Iris.
- Live wiring: only the **offline** readers (`glim_rosbag` / `glim_pcap_rosbag`) support concat, because merging correctly requires buffering **future** aux sweeps (the point-coherent right sweep arrives ~66–92 ms *after* the primary) — which only the offline future-aware release queue can do. The live path (`points_callback_live`) has no such queue, so as of the 2026-07 guard the live node **throws at startup** when `enable_online_mapping=true` **and** `lidar_concat` is enabled, rather than silently merging front+left only (or the wrong sweep). Build concat maps offline; see the startup guard in `glim_ros2/src/glim_ros/glim_ros.cpp`.

**GNSS / RTK global anchoring** — `glim_ext/modules/mapping/gnss_global`, `config_gnss_global.json`, `config.json`

- INS-tolerant design: LiDAR+IMU (`libodometry_estimation_gpu.so`) is the *primary* trajectory; `libgnss_global.so` adds RTK-FIXED **position-prior** factors to the iSAM2 graph. The INS-driven `config_odometry_ins.json` path (which pauses on RTK loss) is left in tree but not selected.
- RTK-FIXED-only gating via the `gicp_localization/scripts/rtk_fixed_odom_filter.py` pre-filter; factors go silent during dropouts and re-anchor on reacquisition (iSAM2 retroactively smooths the gap).
- **Atlas dual-antenna heading prior**: `enable_orientation_prior=true`, `orientation_prior_inf_scale=[1e-6,1e-6,1e2]` → a **yaw-only** `PoseRotationPrior` per submap (roll/pitch left free), pinning heading the position prior can't. Hardened (P5, 2026-07) with a **per-sample yaw-quality gate** (`orientation_prior_max_yaw_sigma_deg: 3.0`): the upstream RTK filter qualifies *position* quality only, and the dual-antenna heading can be degraded while position stays FIXED (secondary-antenna outage, baseline multipath) — such samples now skip the heading prior (position prior still applied; skips logged with a running count). Unpopulated yaw covariance passes, so covariance-less publishers keep the old behavior. Lever-arm compensation disabled (the Atlas INS pose is already at the antenna phase centre and the graph body runs there too). *Validate the yaw convention before tightening.*
- `T_world_utm.txt` export of the odom→GNSS-input-frame SE(3) transform for downstream GICP localization / post-processing. `gnss_global` still aligns the map to the GNSS input frame via a 2D Umeyama fit and exports that SE(3) (the code still names the variable `T_world_utm` internally). The operational **contract is now local ENU** supplied by the adapter — since the adapter feeds `/gps_p1/*` already in ENU, the exported transform is effectively world↔ENU. UTM is not used.

**Mapping / optimization — offline whole-track refinement** — `config_global_mapping_gpu.json`

- Loop closure **enabled** (`max_implicit_loop_distance: 200`, was `0` = off), `min_implicit_loop_overlap: 0.1`, `create_between_factors: true`, tighter `isam2_relinearize_thresh: 0.01`; all submaps retained at full density. Tuned for offline full-graph forward/backward refinement over the entire track (lap-over-lap closure on a closed circuit).

**Offline-only operation** — `config_ros.json`, `glim_ros.cpp`, `glim_rosnode.cpp`

- `enable_online_mapping: false` (default): the constructor creates **no** live subscriptions or wall timer, and `glim_rosnode` refuses to run with a message pointing at the offline tools. Maps are built only via `glim_rosbag` / `glim_pcap_rosbag` (which feed the callbacks directly). Flip the flag to restore the legacy live path.

**Robustness** — `glim_ros.cpp`

- Multi-stage bag-playback throttle: `points_callback` returns `max(odom, sub_mapping, global_mapping)` workload so playback waits on the slowest stage, preventing sub/global-mapping input queues from growing unbounded and OOMing.
- `cv_bridge::toCvCopy` wrapped in try/catch — malformed image frames are dropped, not fatal.

**Tooling**

- `glim_ros2/src/iris_pcap_reader.cpp` + `merge_luminar_pcap.py` (repo root): raw Luminar PCAP → merged `PointCloud2` ingestion (`glim_pcap_rosbag`), byte-identical layout between the C++ and Python paths.
- `scripts/export_glim_dump_to_pcd.py` (repo root): GLIM dump → composed ENU PCD map + provenance manifest (the supported map handoff to GICP).

### Key Features

- **LiDAR+IMU tight fusion as primary odometry** — runs every scan via VGICP + GTSAM `CombinedImuFactor` preintegration. No external pose required; mapping cannot stall on GNSS loss.
- **RTK-FIXED-only GNSS anchoring** — a small ROS2 pre-filter (`gicp_localization/scripts/rtk_fixed_odom_filter.py`) admits only Atlas samples whose pose covariance indicates a FIXED-integer solution. `libgnss_global.so` then turns each forwarded message into a position-prior factor on the iSAM2 graph, plus a yaw-only heading prior when the sample's reported heading quality passes the yaw gate (see the GNSS anchoring section above).
- **Seamless GNSS-denied continuity** — when RTK quality degrades, the filter stops forwarding and the GNSS factor stream goes silent. LiDAR+IMU odometry continues to extend the map perimeter; on RTK reacquisition the next factor anchors the post-dropout trajectory back to the global frame and iSAM2 retroactively smooths the dropout.
- **Geo-referenced output** — `T_world_utm.txt` saves the SE(3) transform from the local odom frame to the GNSS input frame for downstream use (GICP localization, post-processing, etc.). Because the adapter feeds `/gps_p1/*` already in **local ENU**, that input frame *is* ENU and the exported transform is effectively world↔ENU. The variable keeps its historical `T_world_utm` name in code, but no UTM projection is performed.

The exact behavior of this fork should be taken from the checked-in config and source files in this repository, not assumed to match upstream defaults.

## Repository Structure

```
.
├── glim/          # Core SLAM framework
│   ├── config/    # Configuration files (optimized for cybertruck)
│   ├── include/   # Header files
│   └── src/       # Source code
├── glim_ext/      # Extension modules
│   ├── modules/
│   │   └── mapping/
│   │       └── gnss_global/  # RTK-GPS constraint module
│   └── config/    # Extension configs
└── glim_ros2/     # ROS2 interface
    ├── include/   # lidar_concat + ros compatibility headers
    ├── src/       # ROS2 nodes (glim_rosbag, glim_pcap_rosbag, glim_rosnode, tools)
    └── test/      # concat point-time gtests
```

## Dependencies

### System Requirements
- Ubuntu 24.04 (recommended)
- ROS 2 Jazzy
- CUDA 11.8+ (optional, for GPU acceleration)

### Core Dependencies
```bash
sudo apt update
sudo apt install -y \
  libeigen3-dev \
  libboost-all-dev \
  libfmt-dev \
  libomp-dev \
  libmetis-dev \
  ros-jazzy-tf2-eigen \
  ros-jazzy-pcl-ros
```

### GTSAM (Required)
```bash
# Install GTSAM
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake .. -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF
make -j$(nproc)
sudo make install
```

### gtsam_points (Required)
```bash
# Install gtsam_points
git clone https://github.com/koide3/gtsam_points.git
cd gtsam_points
mkdir build && cd build
cmake .. -DBUILD_WITH_CUDA=ON  # Set OFF if no GPU
make -j$(nproc)
sudo make install
```

### iridescence (Optional, for visualization)
```bash
git clone https://github.com/koide3/iridescence.git
cd iridescence
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## Building

### Clone and Build
```bash
# Clone the parent monorepo
cd ~/ros2_ws/src
git clone https://github.com/augcog/DLIO_plusplus.git

# Build the GLIM packages (use --packages-up-to to limit scope, or omit to build everything)
cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-up-to glim_ros

# Source the workspace
source install/setup.bash
```

### Build Options
- **CPU-only build**: Remove `-DBUILD_WITH_CUDA=ON` from gtsam_points build
- **Debug build**: Use `-DCMAKE_BUILD_TYPE=Debug` instead of Release

## Usage

### Mapping pipeline overview

Run commands beginning with `scripts/` from the DLIO++ repository root (the
directory containing `GLIM/`, `adapter/`, and `scripts/`).

> **Inputs.** GLIM consumes a normalized bag whose Atlas-derived `/gps_p1/*`
> streams are already local ENU and whose Luminar payloads are raw. The default
> `prep_bag.py` route creates that bag **and maps it with GLIM**; use
> `--skip-glim` only for a manual `glim_rosbag` run. GLIM itself does no WGS84
> or UTM projection.

```
                              ┌─────────────────────────────┐
       /gps_p1/imu (99 Hz) ──▶│  GLIM odometry estimator    │
  /luminar_*/points (10 Hz) ─▶│  libodometry_estimation_gpu │── per-scan ─┐
                              │  (VGICP + CombinedImuFactor) │             │
                              └─────────────────────────────┘             │
                                                                          ▼
                                                            ┌──────────────────────┐
                                                            │  Global mapping     │
                                                            │  iSAM2 pose graph   │
                                                            │  (sub-maps + loop   │
                                                            │   closures + GNSS)  │
                                                            └──────────────────────┘
                                                                          ▲
                                                                          │ GNSS prior factor
                                                                          │ (only when arriving)
                                                            ┌──────────────────────┐
  /gps_p1/filtered_odom ──▶ rtk_fixed_odom_filter.py ──▶   │ libgnss_global.so   │
                            (drops every sample with        │ subscribes to       │
                             cov > FIXED thresholds)        │ /gps_p1/filtered_   │
                                                            │ odom_rtk_fixed      │
                                                            └──────────────────────┘
```

The odometry estimator and the GNSS extension are deliberately decoupled. Odometry runs every scan regardless of GNSS state. Once its world↔ENU alignment is initialized, the GNSS extension turns each eligible associated sample into position (and, when quality allows, yaw) prior factors on the global graph; if no qualified messages arrive, no factors are added — but the trajectory still gets scan-to-scan factors from VGICP and IMU preintegration, so the map perimeter keeps extending.

### Startup procedure (RTK FIXED required at session start)

The mapping session must start with Atlas in RTK FIXED. The odometry estimator can technically run earlier (it does not require any GNSS), but you want the *first* GNSS prior factor to land while RTK is FIXED so the global map frame is anchored to centimetre-level absolute position.

**Step 1 — Park with sky view, wait for Atlas FIXED:**
- Stop the vehicle at the intended map origin with a clear sky view.
- Watch Atlas's status display or `ros2 topic echo /gps_p1/filtered_odom` and look for `pose.covariance[0]` dropping under ~1×10⁻³ m² (≈ 3 cm σ). Typical FIXED acquisition under open sky is 30 s — 2 min.

**Step 2 — Launch the RTK-FIXED pre-filter:**
```bash
python3 gicp_localization/scripts/rtk_fixed_odom_filter.py
```
Expect a log line:
```
RTK-FIXED odometry pre-filter ready: '/gps_p1/filtered_odom' -> '/gps_p1/filtered_odom_rtk_fixed'
First INS sample received at stamp=… cov=[…] -> FIXED
```
If `-> NOT FIXED` instead, wait. The filter will log the transition the moment Atlas reaches FIXED.

**Step 3 — Record the session, then normalize and map it offline:**

GLIM maps **offline only**, so the parked-init + drive sequence below is performed
while **recording a bag** (the standstill and RTK-FIXED conditions govern the
recorded data quality). The recommended command normalizes the bag and runs GLIM
in one step:
```bash
python3 scripts/prep_bag.py --input <raw_bag> --output <normalized_bag> \
    --p1-imu-pcap <ins.pcap> --dump-dir <out_dir>
```
During the offline run you should see `estimate initial IMU state` from the
LiDAR+IMU loose-init within ~5 s of the bag's parked segment, followed by
`T_world_utm=…` and GNSS-prior insertion after replay reaches the first ≥5 m of
FIXED-quality travel. To invoke `glim_rosbag` manually instead, first run the
same command with `--skip-glim`, then use the manual bag command in **Running
modes** below; otherwise you would map the bag twice.
Map points appear in the viewer. The two-phase init conditions below apply to the
**recording**; the same standstill/RTK ordering is what the offline run replays.

> ### Two sequenced init conditions — parked IMU init, then a short FIXED-quality drive
>
> GLIM init is two-staged. The conditions are **sequenced, not in conflict**:
> Atlas LG69T's dual-antenna design resolves RTK FIXED and INS attitude at
> standstill, so the parked phase establishes IMU initialization and RTK
> quality. The world↔ENU alignment then requires the first ≥5 m of travel.
>
> **🅐 Phase 1 — Stationary calibration completes the GLIM init step.**
>
> > **THE VEHICLE MUST REMAIN STATIONARY FOR AT LEAST 5 SECONDS AFTER LAUNCHING GLIM.**
>
> The `LOOSE` init in `config_odometry_gpu.json` (`initialization_mode: "LOOSE"`, `initialization_window_size: 5.0`) runs a 5-second batch optimization that **estimates the gravity direction by averaging the IMU specific-force vector** (see `loose_initial_state_estimation.cpp:142-165`). The math assumes `mean(acc_local) ≈ gravity` — exact at standstill. Aggressive accel/braking/cornering during this window tilts the gravity estimate and rotates the resulting map. `fix_imu_bias: true` then **locks** the IMU bias at the init value, so a bad init cannot self-correct — a restart from a stationary state is the only fix.
>
> **🅑 Phase 2 — RTK-FIXED world↔ENU alignment after the first ≥5 m.**
>
> > **DO NOT BEGIN DRIVING UNTIL `rtk_fixed_odom_filter.py` HAS LOGGED `RTK transition: ... -> FIXED`.** Then drive the first **≥ 5 m carefully** and verify `gnss_global` logs `T_world_utm=...` followed by prior-factor insertions.
>
> The pre-filter forwards Atlas samples to `libgnss_global.so` only while pose covariance indicates RTK-FIXED. Note the actual initialization sequence: `gnss_global` **cannot** emit any prior factor while the vehicle is parked — it waits until the trajectory baseline exceeds `min_baseline: 5.0 m` (`config_gnss_global.json`) before fitting the one-shot world↔ENU transform, and only then emits GNSS factors. So "wait for the first prior factor before driving" is unsatisfiable; the correct contract is: (1) RTK-FIXED while parked, (2) drive the first ≥5 m gently (this segment seeds the alignment fit), (3) verify `T_world_utm` initialization and factor insertion in the log. The pre-baseline segment is **backfilled** with factors once the baseline is reached, so no early submap is left unanchored. **The map's first geo-referenced frames must come from FIXED-quality Atlas poses, not a degraded RTK-FLOAT or GPS-only fallback.** Driving before RTK-FIXED means the early trajectory grows in a local odom frame and only retroactively aligns to global when RTK reacquires — iSAM2 will smooth it, but the map no longer starts from cm-level absolute coordinates.
>
> Atlas normally reaches FIXED during the parked 30 s – 2 min acquisition window; the transform fit and first factors then arrive during the first few metres of the run. If Atlas never reaches FIXED while parked, fix the hardware/sky-view condition before driving — don't paper over it by starting GLIM and hoping RTK lands en route.
>
> **Safe sequence:** park level with clear sky view → wait for Atlas display to show RTK FIXED + INS aligned → launch `rtk_fixed_odom_filter.py` and GLIM → confirm `estimate initial IMU state` (the 5 s LOOSE init) in the log → **then begin driving gently for the first ≥ 5 m** → confirm `gnss_global` logs `T_world_utm=…` (INFO level) during that stretch. (The per-insertion `insert ... GNSS prior factors` line is DEBUG level and invisible at the default log level; the `T_world_utm=` line is the reliable INFO-level confirmation. Prior factors CANNOT appear while parked — the transform fit needs the ≥5 m baseline; the parked FIXED samples are backfilled once it's reached.)

**Step 4 — Drive the track:**
Watch the viewer; the map should grow continuously. The filter will print FIXED↔NOT_FIXED transitions whenever Atlas's RTK quality crosses the covariance gate — these are diagnostic, not errors, and the map keeps extending through them.

### RTK-denied terrain strategy

GLIM is designed for tracks that include GNSS-denied passages (tunnels, dense foliage, urban canyons, mountain switchbacks where multipath kills FIXED quality temporarily). The behaviour is:

**During the dropout:**
- `rtk_fixed_odom_filter.py` stops forwarding samples. It logs `RTK transition: FIXED -> NOT_FIXED at stamp=… cov=[…]`.
- `libgnss_global.so` receives no new messages → no new GNSS factors added to the iSAM2 graph.
- **LiDAR+IMU odometry keeps running every scan.** VGICP between-factors + ImuFactor preintegration drive the trajectory forward. The map perimeter keeps extending — every new scan's points get inserted at the LiDAR+IMU-estimated pose, with no holes.
- The trajectory in the GNSS-denied section gradually drifts (sub-metre over hundreds of metres on a calibrated MEMS-grade IMU + multi-LiDAR Luminar; multi-metre on kilometre-scale dropouts).

**On RTK FIXED reacquisition (exit the tunnel):**
- The filter resumes forwarding. Logs `RTK transition: NOT_FIXED -> FIXED at stamp=… cov=[…]`.
- The next forwarded message becomes a fresh GNSS prior factor on the current pose.
- iSAM2 detects the disagreement between the drifted current pose and the GNSS anchor. The incremental smoother **redistributes the error retroactively across all the poses inside the dropout**, satisfying both the LiDAR/IMU consistency constraints and the GNSS anchor at exit.
- Map points in the dropout segment shift with their poses. The result is a continuous, drift-corrected map.

This is why mapping **never** falls back to pure IMU dead-reckoning (would drift visibly within seconds) or to LiDAR-only odometry (would create a discontinuity at rejoin). The native LiDAR+IMU fusion already handles GNSS loss as a first-class scenario.

If the dropout is long enough or feature-poor enough that residual error matters, two follow-ups help:

- **Re-traverse the dropout area** on a later lap. Loop closure factors get added, further refining the dropout trajectory.
- Note: raising `smoother_lag` does **not** help here. GNSS priors are inserted into the **global** submap graph (via `gnss_global` → global mapping), not the local fixed-lag smoother — the old 30 s lag was removed for exactly this reason. Dropout behavior is governed by `max_interp_gap_sec` (submaps inside a dropout are left un-anchored by design) plus loop closures on re-traversal.

### Running modes

**GLIM builds maps OFFLINE only.** `enable_online_mapping: false` (default), and
the live node (`glim_rosnode`) exits by design when it is off — there is no
`glim_ros.launch.py` shipped in this fork. Use one of the offline entry points.

**Offline — mcap bag:**
```bash
ros2 run glim_ros glim_rosbag <rosbag_path> --ros-args -p dump_path:=<output_directory>
```
`glim_rosbag` plays the bag and processes it in one step — it reads bag messages
and invokes GLIM callbacks **directly, without publishing them to ROS topics**.
An externally-running `rtk_fixed_odom_filter.py` therefore receives **nothing**
from this replay and cannot gate anything. If your bag carries only the raw
`/gps_p1/filtered_odom`, you must pre-normalize it so it contains
`/gps_p1/filtered_odom_rtk_fixed` before mapping — the intended route is
`prep_bag.py`, which **by default normalizes AND runs GLIM in one shot** (into
`--dump-dir`, writing `enu_origin.txt` for the exporter):
```bash
python3 scripts/prep_bag.py --input <raw_bag> --output <normalized_bag> --dump-dir <dump_dir>
```
Do **not** then run `glim_rosbag` again on the normalized bag — that double-maps.
Only if you want to drive `glim_rosbag` yourself, pass `--skip-glim` to make
`prep_bag.py` normalization-only, then run the `glim_rosbag` command above on
`<normalized_bag>` (that hand-run dump will have no `enu_origin.txt`, so pass
`--enu-origin` to the exporter explicitly). Either way, `prep_bag.py` replays
through the adapter's RTK covariance gate, records the qualified topic, and
enforces the RTK-anchor acceptance gate on the resulting map unless
`--lidar-imu-only` is passed.

### Exporting the localization map

A GLIM dump is in GLIM's internal WORLD frame, not automatically the adapter's
local ENU frame. Export it before giving it to either localizer:

```bash
python3 scripts/export_glim_dump_to_pcd.py <dump_dir> /path/to/track_map.pcd \
    --voxel-size 0.1
```

The exporter composes compact submaps, applies `inverse(T_world_utm)` to produce
an ENU PCD, and writes `/path/to/track_map.pcd.manifest.yaml`. The all-in-one
`prep_bag.py` route records the chosen datum in `<dump_dir>/enu_origin.txt`, which
the exporter reads automatically. For a hand-run dump, append
`--enu-origin "<lat,lon,alt>"` using the exact adapter origin. Do not use the
viewer point export or a hand-converted individual submap: those are WORLD-frame
and lack the manifest needed to verify the ENU contract.

Note: with `two_pass_point_time_join` (default on), `glim_rosbag` first
re-reads the bag's LiDAR topics once to index absolute point-time ranges and
plan every front/aux merge deterministically — expect an extra disk-speed pass
before mapping starts, a `two-pass join plan` summary up front, and a
`lidar_concat primary accounting` line at the end whose counts must reconcile
(hard error otherwise).

**Offline — raw Luminar pcap (+ sibling mcap for IMU/GNSS):**
```bash
ros2 run glim_ros glim_pcap_rosbag <pcap_dir> <mcap_bag> --ros-args -p dump_path:=<output_directory>
```

**With logging:** append `| tee /tmp/glim_offline.log` to either command above.

**Legacy live mode** (opt-in, unsupported here): set `glim_ros/enable_online_mapping=true`
in `config_ros.json` and run the live node directly with `ros2 run glim_ros glim_rosnode`.
This fork ships no live launch file and does not exercise this path; prefer recording
a bag and mapping offline.

### Monitoring RTK and GNSS Alignment

**Map acceptance — RTK anchoring is enforced, not assumed.** A clean
`glim_rosbag` exit only proves local consistency: a map can be completely
unanchored (zero GNSS factors, no `T_world_utm.txt`) and still exit 0. Two
mechanisms close this:

1. `gnss_global` emits a machine-parseable audit line at save time —
   `gnss_global summary: transformation_initialized=… fit_rms_m=…
   position_factors=… orientation_factors=… factors_delivered=…
   factors_undelivered=… yaw_gate_skips=… gap_unanchored=… submaps_seen=…
   submaps_dropped_pre_gnss=… submaps_dropped_no_bracket=…
   submap_anchor_coverage=… nonmonotonic_drops=… bracket_count=…
   bracket_max_s=… bracket_mean_s=…` — covering the latched one-shot-fit RMS
   residual (`fit_rms_m`), factors EMITTED vs actually DELIVERED to the graph
   (`factors_delivered` / `factors_undelivered` — a nonzero undelivered count
   means `save()` flushed before delivery, so the serialized map has fewer
   anchors than emitted), submap anchoring coverage (`submaps_seen` and the
   per-cause drop counters vs the factored fraction, so a run anchored only in
   its last minute is distinguishable from a fully anchored one), GNSS-to-submap
   bracket widths, and every rejection class (dropout gaps, non-monotonic
   stamps).
2. `scripts/prep_bag.py` applies a **default-on RTK-anchor acceptance gate**
   after mapping: the run fails unless `T_world_utm.txt` exists and parses as
   a finite 4×4 SE(3) AND the summary reports an initialized transform with
   `position_factors > 0` **and** `factors_undelivered == 0`. Fields are parsed
   by name (order-independent). LiDAR/IMU-only mapping must opt out explicitly
   with `--lidar-imu-only`.

Pre-filter messages (live tracking of RTK quality):
```
[rtk_fixed_odom_filter] RTK-FIXED odometry pre-filter ready: ...
[rtk_fixed_odom_filter] First INS sample received at stamp=… cov=[…] -> FIXED
[rtk_fixed_odom_filter] RTK transition: FIXED -> NOT_FIXED at stamp=…   # entering dropout
[rtk_fixed_odom_filter] RTK transition: NOT_FIXED -> FIXED at stamp=…   # exiting dropout
```

GLIM `gnss_global` messages (global anchoring):
```
[gnss_global] initializing GNSS global constraints
[gnss_global] gnss_global_config_path=<path>
[gnss_global] T_world_utm=<transformation>           # first anchor
[gnss_global] insert <N> GNSS prior factors          # debug level
[gnss_global] saved T_world_utm (4x4 SE(3)) to: <dump_path>/T_world_utm.txt
```

GLIM odometry messages (LiDAR+IMU pipeline health):
```
[odometry_estimation] estimate initial IMU state          # ~5 s after start
IMU validation results: / num_validations=…               # IMU sanity summary (debug log level)
```

### Map Output

When using `glim_rosbag`, maps are saved to the specified `dump_path`:
```bash
ros2 run glim_ros glim_rosbag <rosbag> --ros-args -p dump_path:=<output_directory>
```

Each directory contains:
- `graph.txt` / `graph.bin` - Pose graph structure
- `000000/`, `000001/`, ... - Submap directories with point clouds
- `odom_lidar.txt` / `odom_imu.txt` - Odometry trajectories
- `traj_lidar.txt` / `traj_imu.txt` - Optimized trajectories
- `T_world_utm.txt` - **SE(3) transformation between odom frame and the GNSS input frame (local ENU)** (if GNSS enabled; historical filename, no UTM projection)
- `config/` - Configuration files used for this map

## Configuration

### Main Configuration Files

**GLIM Core (`glim/config/`):**
- `config.json` — Main config (selects which odometry estimator to load)
- `config_ros.json` — ROS topics and extension modules
- `config_sensors.json` — Sensor noise + IMU/LiDAR extrinsics (`T_lidar_imu`)
- `config_odometry_gpu.json` — **Currently selected** odometry estimator (VGICP + IMU)
- `config_odometry_ins.json` — Alternative INS-driven estimator (NOT selected; pauses on RTK loss)
- `config_odometry_{cpu,ct}.json` — Other alternatives (CPU-only VGICP, continuous-time)
- `config_preprocess.json` — Point cloud preprocessing (legacy sparse; see dense profile below)
- `config_preprocess_dense_map.json` / `config_sub_mapping_dense_map.json` — **Dense localization-map profile, the ACTIVE DEFAULT in `config.json`** (P4, 2026-07): preprocess random-grid 1.0 → 0.4 m / target 30k → 80k, submap voxel 0.5 → 0.25 m / 50k → 150k points. Every map this pipeline builds is consumed by `gicp_localization`, and the cross-run GICP fitness floor (~0.27) was dominated by map sparsity from the old 1.0 m downsample. Swap back to the legacy sparse configs (commented in `config.json`) only for odometry-only smoke runs. After the first dense rebuild, **re-baseline the GICP fitness floor** with `gicp_localization/scripts/analyze_scan_debug_log.py` — the GICP P1 ratio thresholds depend on it.
- `config_global_mapping_gpu.json` — Loop closure and global optimization (iSAM2 backend)

**GNSS Extension (`glim_ext/config/`):**
- `config_gnss_global.json` — RTK prior factor topic and precision

### Key Parameters (Atlas-tuned values, AV-24 deployment)

**Atlas-derived noise envelope** — measured on a known-RTK-fixed AV-24 bag (`run_2`, 17 min):

| Field | Median | p95 | Equivalent σ |
|---|---|---|---|
| `pose.covariance[0]` (x) | 2.8×10⁻⁵ m² | 4.1×10⁻⁵ m² | ~5–6 mm |
| `pose.covariance[7]` (y) | 4.2×10⁻⁵ m² | 5.7×10⁻⁵ m² | ~6–8 mm |
| `pose.covariance[14]` (z) | 1.0×10⁻⁴ m² | 1.3×10⁻⁴ m² | ~1.0–1.1 cm |
| IMU stationary accel σ | 3 mm/s² @ 99 Hz | — | density ~3×10⁻⁴ m/s²/√Hz |
| IMU stationary gyro σ | 7 mrad/s @ 99 Hz | — | density ~7×10⁻⁴ rad/s/√Hz |

The IMU and GNSS noise parameters below are deliberately conservative — set much looser than these measured stationary values to leave headroom for transients (vibration spikes, multipath bursts) that the per-message covariance doesn't capture. Concretely, `imu_acc_noise = 0.05` is ~170× the measured ~3×10⁻⁴ accel density and `imu_gyro_noise = 0.01` is ~14× the measured ~7×10⁻⁴ gyro density. (Framed against the upstream GLIM defaults of 0.2 / 0.05 instead, these same values are ~4× / ~5× *tighter* — see `config_sensors.json`; the two framings just use different baselines.)

**RTK-FIXED pre-filter** (`gicp_localization/scripts/rtk_fixed_odom_filter.py` params):
```yaml
input_topic:    /gps_p1/filtered_odom         # raw Atlas INS pose
output_topic:   /gps_p1/filtered_odom_rtk_fixed
max_pose_var_xy: 0.001    # m^2 — admit only Atlas FIXED quality (~3 cm σ allowed)
max_pose_var_z:  0.005    # m^2 — Z naturally looser (~7 cm σ allowed)
```
Loosen these to admit RTK-FLOAT if your sky view is poor; tighten to reject Atlas's occasional bias-walk during long FIXED stretches.

**GNSS prior factor precision** (`config_gnss_global.json`):
```json
{
  "gnss": {
    "gnss_topic": "/gps_p1/filtered_odom_rtk_fixed",
    "gnss_msg_type": "nav_msgs/msg/Odometry",
    "min_baseline": 5.0,
    "enable_orientation_prior": true,
    "orientation_prior_inf_scale": [1e-6, 1e-6, 1e2],
    "orientation_prior_max_yaw_sigma_deg": 3.0,
    "prior_inf_scale": [1e4, 1e4, 1e3],
    "enable_lever_arm": false
  }
}
```
- `prior_inf_scale` is **precision** (1/variance), not sigma. Equivalent sigmas: σ_x = σ_y ≈ 1 cm, σ_z ≈ 3 cm — about 2× looser than Atlas's reported precision.
- `enable_orientation_prior: true` adds a **yaw-only** heading prior per submap from the Atlas dual-antenna heading carried in the GNSS `Odometry.pose.orientation` (this is the INS pose orientation, *not* the `sensor_msgs/Imu.orientation` field, which Atlas leaves unpopulated). `orientation_prior_inf_scale: [1e-6, 1e-6, 1e2]` constrains only yaw (σ ≈ 5.7°) and leaves roll/pitch to gravity/LiDAR. It pins heading against the slow LiDAR-IMU yaw drift that the position prior alone cannot fix. Fires only on RTK-FIXED samples; validate the yaw convention on a bag before tightening the yaw precision.
- `orientation_prior_max_yaw_sigma_deg: 3.0` (P5 yaw-quality gate) — skip the heading prior when the message's reported yaw sigma (√`pose.covariance[35]`, populated by the adapter from Atlas `rpy_covariance`) exceeds this. The RTK filter above qualifies *position* quality only; dual-antenna heading can be degraded while position is FIXED, and a garbage heading at 5.7° precision would twist the map. Healthy Atlas heading is 0.1–0.3° sigma. Position priors are unaffected; skips are logged with a running count; `<= 0` disables; unpopulated covariance passes.
- `enable_lever_arm: false` because the Atlas INS **pose/position** is already output at `gps_antenna_top` and the graph body runs there too, so a software **position** lever-arm would double-compensate. (The IMU stream is body-axis-rotated but device-located per FusionEngine Spec §3.4.1 — a separate, small accel lever-arm, deliberately not modelled.)

**IMU noise** (`config_sensors.json`, tuned for Atlas `imu_calibrated`):
```json
{
  "sensors": {
    "imu_acc_noise":  0.05,   // m/s^2/sqrt(Hz) — ~4x tighter than uncalibrated MEMS
    "imu_gyro_noise": 0.01,   // rad/s/sqrt(Hz) — ~5x tighter
    "imu_bias_noise": 1e-5,   // Atlas firmware bias is firmware-stable
    "imu_int_noise":  0.001,
    "urdf_imu_frame": "gps_antenna_top"
  }
}
```
Revert to 0.2 / 0.05 if you ever re-point GLIM at a raw MEMS IMU stream.

**Odometry estimator** (`config_odometry_gpu.json`):
```json
{
  "odometry_estimation": {
    "so_name": "libodometry_estimation_gpu.so",
    "initialization_mode": "LOOSE",
    "initialization_window_size": 5.0,    // seconds of IMU+LiDAR before init optimization runs
    "init_pose_damping_scale": 1e10,      // tight origin anchor while initializing
    "fix_imu_bias": true,                 // Atlas bias is pre-calibrated; freezing is safer through long GNSS dropouts
    "smoother_lag": 5.0,                  // seconds; local odometry window only (GNSS priors live in the GLOBAL graph)
    "num_threads": 2
  }
}
```

**Threading (adjust based on your CPU):**
```json
"odometry_estimation": { "num_threads": 2 },
"preprocess":          { "num_threads": 1 }
```
Sub/global mapping use library defaults; tune up if you have spare cores.

### When to retune

| Symptom | Likely fix |
|---|---|
| Loop closures show >10 cm Z error through dropouts | Tighten `prior_inf_scale[2]` from 1e3 to 1e4 |
| GNSS factors visibly tug the trajectory each scan | Loosen `prior_inf_scale` to `[5e3, 5e3, 5e2]` |
| Dropout segment shows visible kink after iSAM2 finishes | Re-traverse the segment (loop closures refine it); check `max_interp_gap_sec` didn't anchor a chord across the dropout. `smoother_lag` does not help — GNSS priors live in the global graph, not the local smoother |
| Map-viewer points jitter on still vehicle | Tighten `imu_acc_noise` further (e.g. 0.02) or check vibration coupling |
| Sub-mm jitter in IMU bias estimates | Already at `imu_bias_noise: 1e-5`; if still problematic, set `fix_imu_bias: true` (already is in current config) |
| Pre-filter never reaches FIXED | Loosen `max_pose_var_xy` / `max_pose_var_z` to admit RTK-FLOAT for that session |

## Coordinate Transformation

### Frame contract: local ENU (not UTM)

The operational coordinate contract is a **local ENU** tangent frame. The `map` frame is a local ENU frame anchored at a fixed datum (the **Putnam origin** from the `race_metadata` TTL). The **adapter is the single authority** that converts Atlas WGS84 → local ENU and feeds GLIM (and GICP) `/gps_p1/*` **already in ENU** — GLIM performs no WGS84/UTM projection of its own.

`gnss_global` still aligns the map to the GNSS input frame via a **2D Umeyama fit** and can export that SE(3). The code still names the exported variable `T_world_utm` internally, but because the adapter supplies ENU input, the exported transform is effectively **world↔ENU**. UTM is not used.

**Transformation variable:** `T_world_utm` (historical name; contract is world↔ENU)

This transformation is:
- Computed once per session after achieving `min_baseline` travel distance (currently `5.0 m` in `config_gnss_global.json`)
- Remains static throughout the mapping run
- **Automatically saved to `T_world_utm.txt` in the map directory**

**Convert map point to ENU (GNSS input frame):**
```cpp
Eigen::Vector3d enu_position = T_world_utm.inverse() * map_position;
```

**Convert ENU to map:**
```cpp
Eigen::Vector3d map_position = T_world_utm * enu_position;
```

The transformation is logged when alignment initializes:
```
[gnss_global] T_world_utm=<transformation>
```

And saved to the map directory when mapping completes:
```
[gnss_global] saved T_world_utm (4x4 SE(3)) to: <dump_path>/T_world_utm.txt
```

## Troubleshooting

### Whole map is tilted / rotates the wrong way
- Almost certainly Phase 1 (stationary calibration) was violated. The `LOOSE` init estimates gravity direction over the first 5 s of IMU samples and assumes mean acceleration ≈ gravity vector; any sustained accel/cornering during those 5 s tilts the estimate. Combined with `fix_imu_bias: true`, the bias is also locked at a wrong value. Stop GLIM, return to a level stationary park, re-launch, wait the 5 s for the first sub-map to appear in the viewer, then drive.

### Map's first frame isn't geo-referenced (jumps when RTK lands)
- Phase 2 (RTK anchoring) was violated — driving started before the pre-filter reported FIXED. The early trajectory grew in a local odom frame and only got rigidly transformed to the global frame when the first GNSS factor finally landed. iSAM2 will smooth this and the final map is still correct, but the first map frame is no longer cm-aligned. Future sessions: wait for `estimate initial IMU state` **and** a FIXED transition before moving, then drive the first ≥5 m gently and confirm `T_world_utm=…` plus GNSS-factor insertion.

### Pre-filter never reports FIXED
- Atlas itself hasn't reached FIXED. Check `ros2 topic echo /gps_p1/filtered_odom --once` and inspect `pose.covariance[0]`; it should drop to ~1×10⁻⁴ m² or below.
- For poor sky-view sessions, raise the pre-filter thresholds to admit FLOAT: launch with `-p max_pose_var_xy:=0.05 -p max_pose_var_z:=0.1`.

### Map not anchoring to global frame (no `T_world_utm` log line)
- The pre-filter is running but `libgnss_global.so` isn't subscribing. Check `extension_modules` in `config_ros.json` includes `libgnss_global.so`.
- Verify `gnss_topic` in `config_gnss_global.json` matches the filter's output (`/gps_p1/filtered_odom_rtk_fixed` by default).
- Vehicle has not yet accumulated `min_baseline` (5.0 m by default) in both the estimated trajectory and GNSS samples — `libgnss_global.so` needs two well-separated samples before it can initialize the SE(3) anchor and emit factors.

### Map shows discontinuity / kink after a GNSS-denied passage
- Plan re-traversal on a later lap so loop closure factors refine the dropout trajectory — this, not `smoother_lag`, is the lever: GNSS priors are inserted into the **global** submap graph, so the local fixed-lag smoother's window cannot redistribute dropout error.
- Verify `max_interp_gap_sec` (default 1.0 s) is active so no chord was anchored across the dropout; the `gnss_global summary:` line reports `gap_unanchored` counts.

### Trajectory drifts visibly during long FIXED stretch
- IMU bias may not be locked. Verify `fix_imu_bias: true` in `config_odometry_gpu.json` (current default).
- Check Atlas's pose covariance is actually still FIXED (`pose.covariance[0]` < 1e-3 m²); transient bias-walk during good FIXED can briefly degrade.

### LiDAR points appear shifted by a fixed offset everywhere
- The IMU/LiDAR extrinsic (`T_lidar_imu` in `config_sensors.json`) is wrong. With Atlas, the IMU is *treated as* located at `gps_antenna_top` — a deliberate approximation (the FusionEngine IMU is body-axis-rotated but device-located; see the sensor-stack note at the top) — so the translation is `luminar_front` (URDF) → `gps_antenna_top` (URDF) = `(−0.95065, −0.005, 0.47194)`. Verify against `av24.urdf`.

### Low performance
- Reduce thread counts if CPU usage is 100%
- Increase downsampling: lower `random_downsample_target` from the default `80000` (dense-map profile; the legacy sparse profile uses `30000`)
- Disable viewers if running headless

### CUDA errors
- Build gtsam_points with `-DBUILD_WITH_CUDA=OFF`
- System falls back to CPU automatically (use `config_odometry_cpu.json` instead of `_gpu.json`)

### Map not saving
- Use `tee` for logging instead of piping through `grep` so the SIGINT shutdown
  sequence reaches GLIM directly:
  `ros2 run glim_ros glim_rosbag <bag> --ros-args -p dump_path:=<dir> | tee output.log`
- Default dump path is `/tmp/dump`; override with `-p dump_path:=<dir>` and
  check write permissions on the chosen directory

## Credits

This workspace is based on:

- **GLIM** by Kenji Koide
  - Repository: https://github.com/koide3/glim
  - Paper: [Graph-based LiDAR-Inertial Mapping](https://staff.aist.go.jp/k.koide/assets/pdf/koide2024ral.pdf)

- **gtsam_points** by Kenji Koide
  - Repository: https://github.com/koide3/gtsam_points

- **GTSAM** by Georgia Tech
  - Repository: https://github.com/borglab/gtsam

## License

This workspace inherits licenses from its constituent packages:
- GLIM: MIT License
- gtsam_points: MIT License
- GTSAM: BSD License

See individual package directories for full license texts.

## Modifications

The complete, reviewer-oriented list of every functional change from upstream koide3 GLIM is in **[Differences From Upstream GLIM](#differences-from-upstream-glim)** above (grouped by area, with file pointers). In brief, this fork adds: Atlas dual-antenna RTK-INS integration, Luminar Iris `UINT8[8]` timestamp decode + epoch-axis safeguard, multi-LiDAR concatenation with schema-equality validation, RTK-FIXED position + dual-antenna heading anchoring (INS-tolerant, never stalls on GNSS loss), offline-only operation, whole-track loop-closure/optimization tuning, playback-throttle and image-decode hardening, and the PCAP / dump-to-PCD tooling. `T_world_utm.txt` SE(3) export and Atlas-derived precision tuning are preserved.

## Citation

If you use this work, please cite the original GLIM paper:

```bibtex
@article{koide2024glim,
  title={GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors},
  author={Koide, Kenji and Yokozuka, Masashi and Oishi, Shuji and Banno, Atsuhiko},
  journal={IEEE Robotics and Automation Letters},
  year={2024}
}
```
