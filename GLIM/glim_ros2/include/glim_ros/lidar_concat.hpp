#pragma once

#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>

#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <glim/util/config.hpp>
#include <glim/util/urdf_transforms.hpp>

namespace glim_ros {

struct AuxLidarSensor {
  std::string topic;
  Eigen::Isometry3d T_primary_sensor;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> buffer;
  size_t buffer_size;
  // P4#3 (GLIM parity with gicp_localization): signed header-time offset stats
  // vs the primary (aux - primary), over MERGED scans only. A stable nonzero
  // mean is the constant-per-aux-clock-offset signature vs the P1 timebase.
  double dt_sum = 0.0;
  double dt_min = std::numeric_limits<double>::infinity();
  double dt_max = -std::numeric_limits<double>::infinity();
  uint64_t dt_count = 0;
};

inline double stamp_to_sec(const builtin_interfaces::msg::Time& stamp) {
  return stamp.sec + stamp.nanosec * 1e-9;
}

inline bool find_xyz_offsets(const sensor_msgs::msg::PointCloud2& msg, int& x_off, int& y_off, int& z_off) {
  x_off = y_off = z_off = -1;
  for (const auto& f : msg.fields) {
    if (f.name == "x") x_off = f.offset;
    else if (f.name == "y") y_off = f.offset;
    else if (f.name == "z") z_off = f.offset;
  }
  return x_off >= 0 && y_off >= 0 && z_off >= 0;
}

// Full point-field schema comparison: decides whether an auxiliary cloud can be
// byte-appended onto the primary. merge_clouds() transforms aux points using the
// AUX cloud's own field offsets, but the MERGED cloud keeps the PRIMARY's
// `fields`, so every downstream reader interprets the appended aux bytes with the
// primary layout. Appending is only safe when the aux layout is byte-identical to
// the primary: same point_step, same endianness, and the same ordered set of
// field {name, offset, datatype, count}. A same-point_step cloud with different
// offsets/datatypes would otherwise be silently misread. O(#fields) (~10-20 per
// Luminar scan) -- negligible next to deskew / voxelisation / registration.
// Returns true on match; on mismatch returns false and sets `reason` for logging.
inline bool schema_matches_primary(const sensor_msgs::msg::PointCloud2& aux,
                                   const sensor_msgs::msg::PointCloud2& primary,
                                   std::string& reason) {
  if (aux.point_step != primary.point_step) {
    reason = "point_step " + std::to_string(aux.point_step) + " vs primary " + std::to_string(primary.point_step);
    return false;
  }
  if (aux.is_bigendian != primary.is_bigendian) {
    reason = "endianness differs (aux is_bigendian=" + std::to_string(aux.is_bigendian) + ")";
    return false;
  }
  if (aux.fields.size() != primary.fields.size()) {
    reason = "field count " + std::to_string(aux.fields.size()) + " vs primary " + std::to_string(primary.fields.size());
    return false;
  }
  for (size_t i = 0; i < primary.fields.size(); i++) {
    const auto& a = aux.fields[i];
    const auto& p = primary.fields[i];
    if (a.name != p.name || a.offset != p.offset || a.datatype != p.datatype || a.count != p.count) {
      reason = "field[" + std::to_string(i) + "] '" + a.name + "' (offset=" + std::to_string(a.offset) +
               ",datatype=" + std::to_string(a.datatype) + ",count=" + std::to_string(a.count) +
               ") differs from primary '" + p.name + "' (offset=" + std::to_string(p.offset) +
               ",datatype=" + std::to_string(p.datatype) + ",count=" + std::to_string(p.count) + ")";
      return false;
    }
  }
  return true;
}

inline void transform_cloud_data(
  std::vector<uint8_t>& data,
  uint32_t point_step,
  int x_off,
  int y_off,
  int z_off,
  const Eigen::Isometry3d& T) {
  const Eigen::Matrix3f R = T.linear().cast<float>();
  const Eigen::Vector3f t = T.translation().cast<float>();
  const size_t num_points = data.size() / point_step;

  for (size_t i = 0; i < num_points; i++) {
    const size_t base = i * point_step;
    float x, y, z;
    std::memcpy(&x, &data[base + x_off], sizeof(float));
    std::memcpy(&y, &data[base + y_off], sizeof(float));
    std::memcpy(&z, &data[base + z_off], sizeof(float));

    Eigen::Vector3f p = R * Eigen::Vector3f(x, y, z) + t;
    std::memcpy(&data[base + x_off], &p.x(), sizeof(float));
    std::memcpy(&data[base + y_off], &p.y(), sizeof(float));
    std::memcpy(&data[base + z_off], &p.z(), sizeof(float));
  }
}

inline sensor_msgs::msg::PointCloud2::SharedPtr find_nearest(
  const std::deque<sensor_msgs::msg::PointCloud2::SharedPtr>& buffer,
  double target_sec,
  double threshold) {
  sensor_msgs::msg::PointCloud2::SharedPtr best;
  double best_dt = std::numeric_limits<double>::max();
  for (const auto& msg : buffer) {
    double dt = std::abs(stamp_to_sec(msg->header.stamp) - target_sec);
    if (dt < best_dt) {
      best_dt = dt;
      best = msg;
    }
  }
  return (best && best_dt <= threshold) ? best : nullptr;
}

inline bool find_time_field(const sensor_msgs::msg::PointCloud2& msg, int& time_off, uint8_t& time_datatype, int& time_count) {
  time_off = -1;
  time_datatype = 0;
  time_count = 0;
  for (const auto& f : msg.fields) {
    if (f.name == "t" || f.name == "time" || f.name == "time_stamp" || f.name == "timestamp") {
      time_off = f.offset;
      time_datatype = f.datatype;
      time_count = f.count;
      return true;
    }
  }
  return false;
}

// Shift per-point timestamps by `dt` seconds to rebase an aux scan from its
// own header.stamp onto the merged cloud's primary header.stamp.
//
// SCAN-RELATIVE encodings (FLOAT32/FLOAT64 seconds-since-scan-start, UINT32
// nanoseconds-since-scan-start): add dt so the value reads as "offset since
// primary scan start" and deskew works.
//
// CAVEAT: the FLOAT64 branch ALWAYS adds dt, i.e. it assumes scan-relative
// seconds. This is correct for every aux LiDAR wired up today, but FLOAT64
// is also a valid carrier for ABSOLUTE epoch seconds (and GLIM's converter
// + TimeKeeper interpret large FLOAT64 values as absolute). A future aux
// sensor emitting FLOAT64 epoch seconds would therefore be double-shifted
// here, exactly like an unguarded UINT8[8] sensor would be. If such a
// sensor is added, gate the FLOAT64 shift the same way UINT8[8] is left
// untouched below (e.g. skip the shift when values look epoch-scaled).
//
// ABSOLUTE-EPOCH encodings (Luminar Iris UINT8[8] = uint64 PTP epoch ns):
// must NOT be shifted. Each point already carries its absolute capture
// time; the deskewer computes (t_i - merged_header.stamp) and naturally
// produces the correct (T_aux - T_primary + intra-aux-offset). Adding dt
// here would double-count the inter-scan offset.
//
// Luminar timestamp format (Luminar Iris Data Output Specification v1.3.0):
// the sensor does NOT emit a single uint64 epoch-ns field -- it carries
// 48-bit integer epoch SECONDS once per packet header (§2.1, UQ48.0) and a
// 32-bit SUB-SECOND NANOSECOND count per ray (§2.2/§2.6.3, UQ32.0) that
// wraps every 1 s; all fields little-endian (§2). The uint64 epoch-ns used
// here is the upstream ROS driver's reconstruction (seconds*1e9 + ns), so
// this depends on the driver, not the datasheet -- verify against the
// actual Luminar driver. (The "epoch time" guidance lives in the PTP
// sections of the Product Information Guide, not the data layout.)
inline void shift_cloud_timestamps(
  std::vector<uint8_t>& data,
  uint32_t point_step,
  int time_off,
  uint8_t time_datatype,
  int time_count,
  double dt) {
  if (time_off < 0) return;

  // UINT8[8] (Luminar Iris uint64 PTP epoch nanoseconds -- driver reconstruction of
  // header seconds + per-ray nanoseconds) is ABSOLUTE and must never be shifted, so
  // skip the whole per-point loop for it. The count check is done ONCE here (not per
  // point): any UINT8 count != 8 is not a recognised timestamp encoding, so warn once
  // -- mirroring extract_raw_points()'s `count != 8` rejection -- and leave untouched
  // (there is no correct shift for an unknown layout).
  if (time_datatype == sensor_msgs::msg::PointField::UINT8) {
    if (time_count != 8) {
      static bool warned_uint8_count = false;
      if (!warned_uint8_count) {
        spdlog::warn("shift_cloud_timestamps: UINT8 time field with count={} (expected 8 for Luminar epoch-ns); leaving unshifted", time_count);
        warned_uint8_count = true;
      }
    }
    (void)dt;
    return;
  }

  const size_t num_points = data.size() / point_step;
  for (size_t i = 0; i < num_points; i++) {
    uint8_t* time_ptr = &data[i * point_step + time_off];
    switch (time_datatype) {
      case sensor_msgs::msg::PointField::UINT32: {
        uint32_t val;
        std::memcpy(&val, time_ptr, sizeof(uint32_t));
        int64_t shifted = static_cast<int64_t>(val) + static_cast<int64_t>(dt * 1e9);
        val = static_cast<uint32_t>(std::max<int64_t>(0, shifted));
        std::memcpy(time_ptr, &val, sizeof(uint32_t));
        break;
      }
      case sensor_msgs::msg::PointField::FLOAT32: {
        float val;
        std::memcpy(&val, time_ptr, sizeof(float));
        val += static_cast<float>(dt);
        std::memcpy(time_ptr, &val, sizeof(float));
        break;
      }
      case sensor_msgs::msg::PointField::FLOAT64: {
        double val;
        std::memcpy(&val, time_ptr, sizeof(double));
        val += dt;
        std::memcpy(time_ptr, &val, sizeof(double));
        break;
      }
      default:
        break;
    }
  }
}

// `primary` is taken as a ConstSharedPtr so both the offline tools (which hold
// a mutable SharedPtr) and the live GlimROS points_callback (which receives a
// P4 review follow-up: per-point time span (seconds) of a cloud, for the
// CONCAT DEBUG evidence line (parity with GICP's scan_time_span_s debug
// topic). A healthy 3-LiDAR merge spans ~1 sweep period; a much larger span
// means a badly-offset aux was rebased far from the primary and will be
// deskewed across a long arc. Returns NaN when no usable time field exists.
// Units by encoding: UINT8[8] = uint64 epoch ns; UINT32 = ns;
// FLOAT32/FLOAT64 = seconds (absolute or scan-relative — the SPAN is
// epoch-invariant either way). One O(N) pass; only run when diag is enabled.
inline double cloud_time_span_seconds(const sensor_msgs::msg::PointCloud2& cloud) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  int off = -1;
  uint8_t datatype = 0;
  int count = 0;
  if (!find_time_field(cloud, off, datatype, count)) return nan;
  const uint32_t step = cloud.point_step;
  if (step == 0 || off < 0 || static_cast<uint32_t>(off) >= step) return nan;
  const size_t n = cloud.data.size() / step;
  if (n == 0) return nan;
  const size_t avail = step - static_cast<uint32_t>(off);

  switch (datatype) {
    case sensor_msgs::msg::PointField::UINT8: {  // Luminar uint64 epoch ns
      if (count != 8 || avail < 8) return nan;
      uint64_t mn = std::numeric_limits<uint64_t>::max(), mx = 0;
      for (size_t i = 0; i < n; i++) {
        uint64_t v;
        std::memcpy(&v, &cloud.data[i * step + off], sizeof(v));
        mn = std::min(mn, v);
        mx = std::max(mx, v);
      }
      return (mx >= mn) ? (mx - mn) * 1e-9 : nan;
    }
    case sensor_msgs::msg::PointField::UINT32: {  // scan-relative ns
      if (avail < 4) return nan;
      uint32_t mn = std::numeric_limits<uint32_t>::max(), mx = 0;
      for (size_t i = 0; i < n; i++) {
        uint32_t v;
        std::memcpy(&v, &cloud.data[i * step + off], sizeof(v));
        mn = std::min(mn, v);
        mx = std::max(mx, v);
      }
      return (mx >= mn) ? (mx - mn) * 1e-9 : nan;
    }
    case sensor_msgs::msg::PointField::FLOAT32: {
      if (avail < 4) return nan;
      float mn = std::numeric_limits<float>::infinity(), mx = -mn;
      for (size_t i = 0; i < n; i++) {
        float v;
        std::memcpy(&v, &cloud.data[i * step + off], sizeof(v));
        if (std::isfinite(v)) { mn = std::min(mn, v); mx = std::max(mx, v); }
      }
      return (mx >= mn) ? static_cast<double>(mx - mn) : nan;
    }
    case sensor_msgs::msg::PointField::FLOAT64: {
      if (avail < 8) return nan;
      double mn = std::numeric_limits<double>::infinity(), mx = -mn;
      for (size_t i = 0; i < n; i++) {
        double v;
        std::memcpy(&v, &cloud.data[i * step + off], sizeof(v));
        if (std::isfinite(v)) { mn = std::min(mn, v); mx = std::max(mx, v); }
      }
      return (mx >= mn) ? (mx - mn) : nan;
    }
    default:
      return nan;
  }
}

// ConstSharedPtr) can call this directly. The primary cloud is only read here;
// the merged output is a fresh copy.
// Strict merge guard (optional): when require_all_aux is set, a scan that fails to
// merge every configured aux must not be silently localized on fewer LiDARs. The
// caller passes a persistent counter (consec_fail); brief transient misses are
// tolerated up to max_consec_fail, after which merge_clouds throws std::runtime_error
// (stopping the node) rather than returning a degraded single-/partial-LiDAR cloud.
// Returns nullptr (not the primary) when require_all_aux is set and the merge is
// incomplete, so the caller SKIPS the scan rather than localizing a degraded cloud.
inline sensor_msgs::msg::PointCloud2::ConstSharedPtr merge_clouds(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& primary,
  std::vector<AuxLidarSensor>& aux_sensors,
  double time_threshold,
  bool require_all_aux = false,
  int max_consec_fail = 0,
  int* consec_fail = nullptr,
  bool abort_on_merge_failure = true,
  bool frame_diag_log = false) {
  const double t_primary = stamp_to_sec(primary->header.stamp);
  const uint32_t point_step = primary->point_step;
  size_t merged_aux_count = 0;

  // P4#3 (GLIM parity): per-frame merge record. GLIM's offline mapping tools
  // have no ROS node to publish debug topics from, so the structured evidence
  // channel is one parseable INFO line per frame ("CONCAT DEBUG | ...") in the
  // mapping log, mirroring gicp_localization's per-frame debug topics.
  std::vector<double> diag_aux_dt(aux_sensors.size(), std::numeric_limits<double>::quiet_NaN());
  std::vector<size_t> diag_aux_pts(aux_sensors.size(), 0);
  const auto emit_frame_diag = [&](size_t total_pts, const sensor_msgs::msg::PointCloud2& cloud) {
    if (!frame_diag_log) return;
    std::ostringstream oss;
    oss << "CONCAT DEBUG | stamp=" << std::fixed << std::setprecision(3) << t_primary
        << " merged=" << merged_aux_count << "/" << aux_sensors.size();
    oss << std::setprecision(4);
    for (size_t i = 0; i < aux_sensors.size(); ++i) {
      oss << " dt" << i << "=" << diag_aux_dt[i] << "s pts" << i << "=" << diag_aux_pts[i];
    }
    // Merged-sweep per-point time span (GICP scan_time_span_s parity); NaN =
    // no usable time field. Computed only when the diag line is enabled.
    oss << " span=" << cloud_time_span_seconds(cloud) << "s total_pts=" << total_pts;
    // "{}" wrapper: never pass a runtime string as the fmt format string
    // (stray braces would throw fmt::format_error mid-mapping).
    spdlog::info("{}", oss.str());
  };

  // Strict-merge failure handler. Single routing point for every "required merge
  // can't complete" path (incomplete aux merge AND primary-precondition failures).
  //   - require_all_aux=false -> returns false: degraded merging allowed.
  //   - require_all_aux=true  -> returns true: the caller returns nullptr so the
  //     degraded cloud is never localized (scan skipped). Past max_consec_fail the
  //     node either aborts (abort_on_merge_failure=true) or keeps skipping with a
  //     louder warning (abort_on_merge_failure=false). Reset on a fully merged scan.
  auto on_required_failure = [&](size_t got, const char* reason) -> bool {
    if (!require_all_aux || !consec_fail) return false;  // degraded merging allowed
    ++(*consec_fail);
    const bool over_budget = *consec_fail > max_consec_fail;
    if (over_budget && abort_on_merge_failure) {
      const std::string msg =
        std::string("lidar_concat: multi-LiDAR merge REQUIRED but could not complete (") + reason + ") for " +
        std::to_string(*consec_fail) + " consecutive scans; abort_on_merge_failure=true -> stopping "
        "(set require_all_aux=false to localize on available LiDARs, or abort_on_merge_failure=false to keep skipping)";
      spdlog::critical(msg);
      throw std::runtime_error(msg);
    }
    spdlog::error(
      "lidar_concat: REQUIRED merge incomplete ({}/{} aux): {} for {} consecutive scan(s) (budget {}){} "
      "-- skipping scan (degraded cloud NOT localized).",
      got, aux_sensors.size(), reason, *consec_fail, max_consec_fail,
      over_budget ? ", budget exceeded (non-fatal)" : "");
    return true;  // skip this scan
  };

  int x_off, y_off, z_off;
  if (!find_xyz_offsets(*primary, x_off, y_off, z_off)) {
    spdlog::warn("lidar_concat: cannot find xyz fields in primary cloud");
    emit_frame_diag(primary->width * primary->height, *primary);
    if (on_required_failure(0, "primary cloud missing xyz fields")) return nullptr;
    return primary;
  }

  // Concatenation treats each cloud as a TIGHT array of point_step-sized points
  // (it byte-appends aux data and re-counts by point_step). Reject a row-padded
  // primary (data.size() != width*height*point_step) loudly rather than silently
  // counting padding as points. Organized-but-tight (height>1, no padding) is
  // fine to flatten; only padding is rejected. Luminar clouds are unorganized and
  // tight (PCAP reader emits height=1, row_step=point_step*width).
  if (point_step == 0 || primary->data.size() != static_cast<size_t>(primary->width) * primary->height * point_step) {
    spdlog::warn("lidar_concat: primary cloud is organized/padded (data={}, width={}, height={}, step={}); skipping concat",
                 primary->data.size(), primary->width, primary->height, point_step);
    emit_frame_diag(primary->width * primary->height, *primary);
    if (on_required_failure(0, "primary cloud organized/padded (non-tight)")) return nullptr;
    return primary;
  }

  auto merged = std::make_shared<sensor_msgs::msg::PointCloud2>(*primary);
  // Count points from the byte buffer (equals width*height for the tight cloud
  // validated above) so merged width/row_step always match the appended bytes.
  size_t total_points = primary->data.size() / point_step;

  for (size_t aux_i = 0; aux_i < aux_sensors.size(); ++aux_i) {
    auto& aux = aux_sensors[aux_i];
    auto match = find_nearest(aux.buffer, t_primary, time_threshold);
    if (!match) {
      spdlog::debug("lidar_concat: no match for {} (t={:.3f})", aux.topic, t_primary);
      continue;
    }
    // Validate the FULL field schema, not just point_step: the merged cloud
    // keeps the primary's `fields`, so an aux scan with the same point_step but
    // different field offsets/datatypes would be silently misread downstream.
    std::string schema_reason;
    if (!schema_matches_primary(*match, *primary, schema_reason)) {
      spdlog::warn(
        "lidar_concat: skipping {} — PointCloud2 schema mismatch vs primary: {} "
        "(merged cloud uses the primary field layout; appending mismatched aux bytes "
        "would misread them — normalize the aux layout upstream to enable concatenation)",
        aux.topic, schema_reason);
      continue;
    }

    // Reject an organized/padded or otherwise non-tight aux: byte-appending it
    // (or counting by point_step) would desync points from the field layout.
    // Requires data.size() == width*height*point_step. Organized-but-tight is OK.
    if ((match->data.size() % point_step) != 0 ||
        match->data.size() != static_cast<size_t>(match->width) * match->height * point_step) {
      spdlog::warn("lidar_concat: skipping {} — non-tight cloud (data={}, width={}, height={}, step={})",
                   aux.topic, match->data.size(), match->width, match->height, point_step);
      continue;
    }

    std::vector<uint8_t> data(match->data.begin(), match->data.end());
    int ax, ay, az;
    if (find_xyz_offsets(*match, ax, ay, az)) {
      transform_cloud_data(data, point_step, ax, ay, az, aux.T_primary_sensor);
    }

    int time_off;
    uint8_t time_datatype;
    int time_count;
    const double dt = stamp_to_sec(match->header.stamp) - t_primary;
    if (find_time_field(*match, time_off, time_datatype, time_count)) {
      shift_cloud_timestamps(data, point_step, time_off, time_datatype, time_count, dt);
      spdlog::debug("lidar_concat: shifted timestamps for {} by {:.6f}s", aux.topic, dt);
    }

    const size_t aux_pts = data.size() / point_step;
    merged->data.insert(merged->data.end(), data.begin(), data.end());
    // Accumulate the byte-derived count (matches the bytes actually appended),
    // not width*height, so merged->width/row_step stay consistent with data.
    total_points += aux_pts;
    ++merged_aux_count;

    spdlog::debug("lidar_concat: merged {} (dt={:.4f}s, {} pts)", aux.topic, std::abs(dt), aux_pts);

    // P4#3: per-frame + running merge-timing diagnostics (parity with
    // gicp_localization). dt here is SIGNED (aux header - primary header): a
    // stable nonzero mean across the run is the constant per-aux clock-offset
    // signature vs the P1 timebase, and it distorts the map's deskew at high
    // yaw rates exactly like it does localization's.
    diag_aux_dt[aux_i] = dt;
    diag_aux_pts[aux_i] = aux_pts;
    aux.dt_sum += dt;
    aux.dt_min = std::min(aux.dt_min, dt);
    aux.dt_max = std::max(aux.dt_max, dt);
    if (++aux.dt_count % 512 == 0) {  // ~every 50 s at 10 Hz
      const double mean = aux.dt_sum / static_cast<double>(aux.dt_count);
      spdlog::info(
        "lidar_concat: '{}' header offset vs primary over {} merges: mean={:+.1f} ms, min={:+.1f} ms, max={:+.1f} ms{}",
        aux.topic, aux.dt_count, 1e3 * mean, 1e3 * aux.dt_min, 1e3 * aux.dt_max,
        std::abs(mean) > 0.02 ? " — mean >20 ms: likely constant clock offset, consider a per-aux time correction" : "");
    }
  }

  emit_frame_diag(total_points, *merged);

  merged->width = total_points;
  merged->height = 1;
  merged->row_step = point_step * total_points;

  // Strict guard: a REQUIRED multi-LiDAR merge that stays incomplete must not be
  // silently localized on fewer LiDARs. The primary-precondition bail-outs above
  // route through the same handler. When require_all_aux is set, an incomplete
  // merge SKIPS the scan (returns nullptr); otherwise the degraded cloud is
  // returned and localized. A fully merged scan resets the budget.
  if (merged_aux_count < aux_sensors.size()) {
    if (on_required_failure(merged_aux_count, "incomplete aux merge")) return nullptr;
  } else if (consec_fail) {
    *consec_fail = 0;
  }

  return merged;
}

struct AuxConcatConfig {
  bool enabled = false;
  double time_threshold = 0.05;
  int buffer_size = 200;
  std::vector<AuxLidarSensor> aux_sensors;
  // Strict merge guard (see merge_clouds). consecutive_merge_failures is mutable
  // running state that the caller passes to merge_clouds each scan.
  bool require_all_aux = false;               // false = build/localize on available LiDARs
  bool abort_on_merge_failure = true;         // true = stop past budget; false = keep skipping non-fatally
  int max_consecutive_aux_merge_failures = 10;
  int consecutive_merge_failures = 0;
  // P4#3: one parseable "CONCAT DEBUG | ..." INFO line per primary scan
  // (merged count, per-aux signed dt + points, total points). Default ON for
  // the offline mapping tools — this is the map-side merge evidence the run
  // reports need; ~1 line / 100 ms costs a few MB per mapping run.
  bool frame_diag_log = true;
};

// Resolve a (possibly relative) urdf_path CWD-independently. parse_urdf_transforms
// feeds the string straight to xmlReadFile() (CWD-relative), which silently fails
// from a different working directory. Try, in order: as-given (absolute or CWD),
// then relative to the GLIM config directory, then a walk UP from the config dir
// looking for the file (av24.urdf is copied into the GLIM stack root, above the
// config dir). Returns the original string if nothing is found, so the caller's
// parse failure + startup guard still fire with a clear error.
inline std::string resolve_urdf_path(const std::string& urdf_path) {
  namespace fs = std::filesystem;
  if (urdf_path.empty() || fs::exists(urdf_path)) {
    return urdf_path;
  }
  const std::string config_dir = glim::GlobalConfig::instance()->param<std::string>("global", "config_path", ".");
  const std::string basename = fs::path(urdf_path).filename().string();
  fs::path d = config_dir;
  for (int i = 0; i < 10; ++i) {
    if (fs::exists(d / urdf_path)) return (d / urdf_path).string();
    if (fs::exists(d / basename)) return (d / basename).string();
    if (!d.has_parent_path() || d.parent_path() == d) break;
    d = d.parent_path();
  }
  return urdf_path;
}

inline AuxConcatConfig load_aux_sensors_from_config(const glim::Config& config_sensors) {
  AuxConcatConfig out;
  out.enabled = config_sensors.param<bool>("lidar_concat", "enabled", false);
  out.time_threshold = config_sensors.param<double>("lidar_concat", "time_threshold", 0.05);
  out.buffer_size = config_sensors.param<int>("lidar_concat", "buffer_size", 200);
  out.require_all_aux = config_sensors.param<bool>("lidar_concat", "require_all_aux", false);
  out.abort_on_merge_failure = config_sensors.param<bool>("lidar_concat", "abort_on_merge_failure", true);
  out.max_consecutive_aux_merge_failures = config_sensors.param<int>("lidar_concat", "max_consecutive_aux_merge_failures", 10);
  out.frame_diag_log = config_sensors.param<bool>("lidar_concat", "frame_diag_log", true);

  if (!out.enabled) {
    return out;
  }

  const auto aux_topics = config_sensors.param<std::vector<std::string>>("lidar_concat", "aux_topics", {});

  // A REQUIRED merge that is enabled with no aux topics is a config error. Hard-fail
  // only when the strict path is also set to abort; otherwise it is handled at
  // runtime (the merge simply never completes and scans are skipped non-fatally).
  if (out.require_all_aux && out.abort_on_merge_failure && aux_topics.empty()) {
    const std::string msg = "lidar_concat: enabled with require_all_aux=true, abort_on_merge_failure=true and no aux_topics; refusing to start";
    spdlog::critical(msg);
    throw std::runtime_error(msg);
  }

  const std::string urdf_path = config_sensors.param<std::string>("lidar_concat", "urdf_path", "");
  const std::string primary_frame = config_sensors.param<std::string>("lidar_concat", "primary_frame", "");
  std::unordered_map<std::string, std::pair<std::string, Eigen::Isometry3d>> urdf_transforms;
  bool use_urdf = !urdf_path.empty() && !primary_frame.empty();

  if (use_urdf) {
    const std::string resolved_urdf = resolve_urdf_path(urdf_path);
    try {
      urdf_transforms = glim::parse_urdf_transforms(resolved_urdf);
      spdlog::info("lidar_concat: loaded URDF from '{}' (config urdf_path='{}', primary_frame={})",
                   resolved_urdf, urdf_path, primary_frame);
    } catch (const std::exception& e) {
      spdlog::error("lidar_concat: failed to parse URDF '{}' (from urdf_path='{}'): {}", resolved_urdf, urdf_path, e.what());
      use_urdf = false;
    }
  }

  const auto aux_frames = config_sensors.param<std::vector<std::string>>("lidar_concat", "aux_frames", {});
  if (use_urdf && aux_frames.size() != aux_topics.size()) {
    spdlog::error("lidar_concat: aux_frames size ({}) must match aux_topics size ({})", aux_frames.size(), aux_topics.size());
    use_urdf = false;
  }

  for (size_t i = 0; i < aux_topics.size(); i++) {
    const auto& topic = aux_topics[i];
    AuxLidarSensor sensor;
    sensor.topic = topic;
    sensor.buffer_size = out.buffer_size;

    if (use_urdf) {
      const std::string& aux_frame = aux_frames[i];
      try {
        sensor.T_primary_sensor = glim::compute_transform(urdf_transforms, primary_frame, aux_frame);
        std::stringstream ss;
        ss << sensor.T_primary_sensor.matrix();
        spdlog::info("lidar_concat: T_{}_{}:\n{}", primary_frame, aux_frame, ss.str());
      } catch (const std::exception& e) {
        spdlog::error("lidar_concat: failed to compute transform {} -> {}: {}", primary_frame, aux_frame, e.what());
        continue;
      }
    } else {
      std::string key = topic;
      for (auto& c : key) {
        if (c == '/') c = '_';
      }
      if (!key.empty() && key[0] == '_') key = key.substr(1);
      key = "T_primary_" + key;

      auto flat = config_sensors.param<std::vector<double>>("lidar_concat", key);
      if (!flat || flat->size() != 16) {
        spdlog::error("lidar_concat: missing or invalid transform '{}' for topic '{}'", key, topic);
        continue;
      }

      Eigen::Matrix4d mat;
      for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
          mat(r, c) = (*flat)[r * 4 + c];
      sensor.T_primary_sensor = Eigen::Isometry3d(mat);
    }

    spdlog::info("lidar_concat: auxiliary sensor {} enabled", sensor.topic);
    out.aux_sensors.push_back(std::move(sensor));
  }
  spdlog::info("lidar_concat: {} auxiliary sensors, threshold={:.3f}s", out.aux_sensors.size(), out.time_threshold);

  // Startup strict guard: if a complete multi-LiDAR merge is REQUIRED and set to
  // abort, but some aux sensors could not even be set up (missing/invalid extrinsic,
  // bad URDF chain), fail loudly at load. With abort_on_merge_failure=false the merge
  // just never completes at runtime and scans are skipped non-fatally instead.
  if (out.require_all_aux && out.abort_on_merge_failure && out.aux_sensors.size() < aux_topics.size()) {
    const std::string msg =
      "lidar_concat: require_all_aux=true and abort_on_merge_failure=true but only " + std::to_string(out.aux_sensors.size()) + "/" +
      std::to_string(aux_topics.size()) + " aux sensors resolved at startup (check URDF/static extrinsics and aux_frames). Refusing to start.";
    spdlog::critical(msg);
    throw std::runtime_error(msg);
  }

  return out;
}

}  // namespace glim_ros
