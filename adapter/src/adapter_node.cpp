#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>

#include "adapter/adapter_utils.hpp"
#include "fusion_engine_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kDegToRadSq = kDegToRad * kDegToRad;

}  // namespace

using namespace adapter;

class Adapter : public rclcpp::Node {
public:
  Adapter()
  : Node("adapter"),
    clock_mapper_(declare_parameter("p1_clock_bin_seconds", 60.0))
  {
    pose_input_topic_ = declare_parameter("pose_input_topic", "/atlas/pose_filtered");
    imu_input_topic_ = declare_parameter("imu_input_topic", "/atlas/imu_calibrated");
    imu_stamp_mode_ = declare_parameter("imu_stamp_mode", "auto");
    p1_like_threshold_sec_ = declare_parameter("p1_like_threshold_sec", 100000000.0);
    // [P2 FIX 2026-07-09] clamp: a negative value wrapped to SIZE_MAX and the
    // arrival-retime queue never published during continuous streaming.
    imu_lookahead_ = static_cast<size_t>(
        std::clamp<int64_t>(declare_parameter("imu_arrival_retime_lookahead", 128), 1, 4096));
    imu_flush_timeout_sec_ = declare_parameter("imu_flush_timeout_sec", 0.5);
    nominal_imu_period_sec_ = declare_parameter("nominal_imu_period_sec", 0.01);
    imu_period_sec_ = nominal_imu_period_sec_;
    pose_input_reliability_ = declare_parameter("pose_input_reliability", "reliable");
    pose_input_qos_depth_ = declare_parameter("pose_input_qos_depth", 100);
    imu_input_reliability_ = declare_parameter("imu_input_reliability", "best_effort");
    imu_input_qos_depth_ = declare_parameter("imu_input_qos_depth", 100);
    imu_frame_id_ = declare_parameter("imu_frame_id", "gps_antenna_top");
    odom_frame_id_ = declare_parameter("odom_frame_id", "map");
    body_frame_id_ = declare_parameter("body_frame_id", "gps_antenna_top");
    rtk_max_var_xy_ = declare_parameter("rtk_max_var_xy", 1e-3);
    rtk_max_var_z_ = declare_parameter("rtk_max_var_z", 5e-3);
    publish_gnss_pose_ = declare_parameter("publish_gnss_pose", true);
    summary_output_path_ = declare_parameter("summary_output_path", "");
    imu_p1_sidecar_path_ = declare_parameter("imu_p1_sidecar_path", "");
    imu_p1_sidecar_match_tolerance_sec_ =
      declare_parameter("imu_p1_sidecar_match_tolerance_sec", 0.02);

    // [P3 HARDENING 2026-07-14] Timing parameters must be finite and positive.
    // The defaults are safe, but a bad override (NaN, 0, negative) silently
    // degrades retiming: a nonpositive nominal period breaks synthesized
    // spacing, a zero flush timeout strands the arrival-retime queue, a
    // nonpositive P1 threshold reclassifies every stamp, and a nonpositive
    // sidecar tolerance drops every sidecar match. Fail loud at startup.
    const auto require_positive_finite = [this](const char* name, double v) {
      if (!std::isfinite(v) || v <= 0.0) {
        RCLCPP_FATAL(get_logger(), "%s = %g is invalid (must be finite and > 0); refusing to start",
                     name, v);
        throw std::runtime_error(std::string(name) + ": invalid timing parameter");
      }
    };
    require_positive_finite("nominal_imu_period_sec", nominal_imu_period_sec_);
    require_positive_finite("imu_flush_timeout_sec", imu_flush_timeout_sec_);
    require_positive_finite("p1_like_threshold_sec", p1_like_threshold_sec_);
    require_positive_finite("imu_p1_sidecar_match_tolerance_sec",
                            imu_p1_sidecar_match_tolerance_sec_);

    if (!imu_p1_sidecar_path_.empty()) {
      loadImuP1Sidecar(imu_p1_sidecar_path_);
    }

    const std::string origin_text = declare_parameter(
      "local_enu_origin", "39.58227391,-86.74232215,260.4");
    const std::string origin_ttl_path = declare_parameter("local_enu_origin_ttl_path", "");
    if (origin_text.empty() == origin_ttl_path.empty()) {
      throw std::runtime_error(
        "exactly one of local_enu_origin or local_enu_origin_ttl_path must be set");
    }
    if (!origin_text.empty()) {
      if (!parseLocalEnuOrigin(origin_text, local_origin_lat_, local_origin_lon_, local_origin_alt_)) {
        throw std::runtime_error(
          "local_enu_origin must be formatted as 'lat_deg,lon_deg,alt_m'");
      }
    } else if (!parseLocalEnuOriginTtl(origin_ttl_path, local_origin_lat_, local_origin_lon_, local_origin_alt_)) {
      throw std::runtime_error("failed to read local_enu_origin_ttl_path: " + origin_ttl_path);
    }
    local_cartesian_.Reset(local_origin_lat_, local_origin_lon_, local_origin_alt_);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/gps_p1/imu", rclcpp::SensorDataQoS());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/gps_p1/filtered_odom", 50);
    odom_rtk_pub_ = create_publisher<nav_msgs::msg::Odometry>("/gps_p1/filtered_odom_rtk_fixed", 50);
    if (publish_gnss_pose_) {
      gnss_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gnss", 50);
      gnss_rtk_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gnss_rtk_fixed", 50);
    }

    pose_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions pose_options;
    pose_options.callback_group = pose_group_;
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_group_;
    pose_sub_ = create_subscription<fusion_engine_msgs::msg::Pose>(
      pose_input_topic_, inputQos(pose_input_reliability_, pose_input_qos_depth_),
      std::bind(&Adapter::poseCallback, this, std::placeholders::_1), pose_options);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input_topic_, inputQos(imu_input_reliability_, imu_input_qos_depth_),
      std::bind(&Adapter::imuCallback, this, std::placeholders::_1), imu_options);

    flush_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Adapter::flushTimerCallback, this), timer_group_);
    if (!summary_output_path_.empty()) {
      summary_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&Adapter::writeSummaryFile, this), timer_group_);
    }

    RCLCPP_INFO(
      get_logger(),
      "Adapter ready: pose=%s imu=%s imu_stamp_mode=%s local_enu_origin=[%.8f, %.8f, %.3f]",
      pose_input_topic_.c_str(), imu_input_topic_.c_str(), imu_stamp_mode_.c_str(),
      local_origin_lat_, local_origin_lon_, local_origin_alt_);
  }

  ~Adapter() override
  {
    writeSummaryFile();
    const std::string summary = summaryText();
    RCLCPP_INFO(get_logger(), "Adapter summary:\n%s", summary.c_str());
  }

  std::string summaryText() const
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    std::ostringstream out;
    out << "pose_in=" << pose_in_count_
        << " gnss_out=" << gnss_out_count_
        << " gnss_rtk_out=" << gnss_rtk_out_count_
        << " odom_out=" << odom_out_count_
        << " rtk_out=" << rtk_out_count_
        << " imu_in=" << imu_in_count_
        << " imu_out=" << imu_out_count_
        // Drop counters are ALWAYS reported (P3 hardening): equal in/out
        // counts alone can hide loss — a run report must be able to prove
        // pose_dropped_invalid == sidecar_miss_drop == imu_dropped_not_ready
        // == 0 rather than infer it.
        << " pose_dropped_invalid=" << pose_dropped_invalid_count_
        << " imu_sidecar_miss_drop=" << sidecar_miss_drop_count_
        << " imu_dropped_clock_not_ready=" << p1_imu_dropped_not_ready_count_
        // [P3 AUDIT 2026-07-14] P1->ROS clock mapping evidence: run reports
        // must be able to verify the retiming contract (mapper became ready,
        // and the offset drift over the run stayed sane) end-to-end.
        << " p1_clock_ready=" << (clock_mapper_.ready() ? 1 : 0)
        << " p1_clock_drift_ms=" << clock_mapper_.driftMs();
    if (!imu_p1_sidecar_.empty()) {
      out << " imu_p1_sidecar_match=" << imu_p1_sidecar_match_count_
          << " imu_p1_sidecar_miss=" << imu_p1_sidecar_miss_count_
          << " imu_p1_sidecar_skip=" << imu_p1_sidecar_skip_count_;
    }
    out << "\n";
    return out.str();
  }

  void writeSummaryFile() const
  {
    if (summary_output_path_.empty()) {
      return;
    }
    std::ofstream out(summary_output_path_);
    out << summaryText();
  }

private:
  struct QueuedImu {
    sensor_msgs::msg::Imu msg;
    double arrival = 0.0;
    double p1 = 0.0;
  };

  struct ImuP1SidecarSample {
    double capture_ros = 0.0;
    double p1_time = 0.0;
  };

  static std::vector<std::string> splitCsvLine(const std::string& line)
  {
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      cells.push_back(cell);
    }
    return cells;
  }

  void loadImuP1Sidecar(const std::string& path)
  {
    std::ifstream in(path);
    if (!in) {
      throw std::runtime_error("failed to open imu_p1_sidecar_path: " + path);
    }

    std::string line;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      const auto cells = splitCsvLine(line);
      if (cells.size() < 2) {
        continue;
      }
      try {
        const double capture_ros = std::stod(cells[0]);
        const double p1_time = std::stod(cells[1]);
        if (std::isfinite(capture_ros) && std::isfinite(p1_time) &&
            capture_ros > 0.0 && p1_time > 0.0) {
          imu_p1_sidecar_.push_back({capture_ros, p1_time});
        }
      } catch (const std::exception&) {
        // Header or malformed line.
      }
    }

    std::sort(
      imu_p1_sidecar_.begin(), imu_p1_sidecar_.end(),
      [](const auto& a, const auto& b) { return a.capture_ros < b.capture_ros; });
    if (imu_p1_sidecar_.empty()) {
      throw std::runtime_error("imu_p1_sidecar_path contained no usable samples: " + path);
    }
    RCLCPP_INFO(
      get_logger(),
      "Loaded IMU P1 sidecar: %zu samples from %s (match_tolerance=%.3f ms)",
      imu_p1_sidecar_.size(), path.c_str(), imu_p1_sidecar_match_tolerance_sec_ * 1e3);
  }

  bool lookupSidecarP1(double capture_ros, double& p1_time)
  {
    if (imu_p1_sidecar_.empty()) {
      return false;
    }

    const double tol = std::max(0.0, imu_p1_sidecar_match_tolerance_sec_);
    while (imu_p1_sidecar_index_ < imu_p1_sidecar_.size() &&
           imu_p1_sidecar_[imu_p1_sidecar_index_].capture_ros < capture_ros - tol) {
      ++imu_p1_sidecar_index_;
      ++imu_p1_sidecar_skip_count_;
    }

    size_t best = imu_p1_sidecar_.size();
    double best_abs_dt = std::numeric_limits<double>::infinity();
    const size_t begin = imu_p1_sidecar_index_ > 0 ? imu_p1_sidecar_index_ - 1 : 0;
    const size_t end = std::min(imu_p1_sidecar_.size(), imu_p1_sidecar_index_ + 3);
    for (size_t i = begin; i < end; ++i) {
      const double abs_dt = std::abs(imu_p1_sidecar_[i].capture_ros - capture_ros);
      if (abs_dt <= tol && abs_dt < best_abs_dt) {
        best = i;
        best_abs_dt = abs_dt;
      }
    }

    if (best == imu_p1_sidecar_.size()) {
      ++imu_p1_sidecar_miss_count_;
      return false;
    }

    p1_time = imu_p1_sidecar_[best].p1_time;
    if (best >= imu_p1_sidecar_index_) {
      imu_p1_sidecar_skip_count_ += best - imu_p1_sidecar_index_;
    }
    imu_p1_sidecar_index_ = best + 1;
    ++imu_p1_sidecar_match_count_;
    return true;
  }

  rclcpp::QoS inputQos(const std::string& reliability_text, int depth) const
  {
    rclcpp::QoS qos = depth <= 0
      ? rclcpp::QoS(rclcpp::KeepAll())
      : rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(depth)));
    std::string reliability = reliability_text;
    std::transform(reliability.begin(), reliability.end(), reliability.begin(), ::tolower);
    if (reliability == "reliable") {
      qos.reliable();
    } else {
      if (reliability != "best_effort") {
        RCLCPP_WARN(get_logger(),
                    "Unrecognized reliability '%s'; using best_effort (check for a typo in YAML)",
                    reliability.c_str());
      }
      qos.best_effort();
    }
    qos.durability_volatile();
    return qos;
  }

  void poseCallback(const fusion_engine_msgs::msg::Pose::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    ++pose_in_count_;
    const double arrival = stampToSec(msg->header.stamp);
    const double p1_time = p1ToSec(msg->p1_time);

    // [P2 FIX 2026-07-09] P1 epoch-reset detection (device power-cycle or
    // bag loop): the clock mapper re-learns internally (see addPosePair);
    // the node-side stamp map and sidecar cursor must reset with it or every
    // subsequent output stays frozen at last+1us / the sidecar never matches.
    if (std::isfinite(last_p1_time_) && p1_time < last_p1_time_ - 5.0) {
      RCLCPP_WARN(get_logger(),
                  "P1 time regressed %.3f -> %.3f (device power-cycle or bag loop); "
                  "resetting stamp map and sidecar cursor",
                  last_p1_time_, p1_time);
      last_stamp_by_topic_.clear();
      imu_p1_sidecar_index_ = 0;
    }
    if (std::isfinite(p1_time) && p1_time > 0.0 && p1_time < 4.0e9) {
      last_p1_time_ = p1_time;
    }
    clock_mapper_.addPosePair(arrival, p1_time);

    // [P2 FIX 2026-07-09] Fail closed on invalid solutions. FusionEngine
    // emits solution_type=Invalid(0) with NaN lla/rpy on every cold start
    // and during full outages; LocalCartesian/rpyToQuat propagate the NaNs
    // into /gps_p1/filtered_odom and /gnss, which downstream uses for INS
    // heading priors and GT-snap recovery. The clock mapper above is still
    // fed (it validates its own inputs).
    const bool pose_finite =
        std::isfinite(msg->latitude) && std::isfinite(msg->longitude) &&
        std::isfinite(msg->altitude) && std::isfinite(msg->rpy.roll) &&
        std::isfinite(msg->rpy.pitch) && std::isfinite(msg->rpy.yaw) &&
        std::isfinite(msg->velflu.x) && std::isfinite(msg->velflu.y) &&
        std::isfinite(msg->velflu.z);
    if (msg->solution_type == 0 || !pose_finite) {
      ++pose_dropped_invalid_count_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Dropping invalid FusionEngine pose (solution_type=%u, finite=%d) — "
                           "%lu dropped so far",
                           static_cast<unsigned>(msg->solution_type),
                           pose_finite ? 1 : 0,
                           static_cast<unsigned long>(pose_dropped_invalid_count_));
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = monotonicStamp("/gps_p1/filtered_odom", clock_mapper_.toRos(p1_time));
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = body_frame_id_;

    double east = 0.0;
    double north = 0.0;
    double up = 0.0;
    local_cartesian_.Forward(msg->latitude, msg->longitude, msg->altitude, east, north, up);

    odom.pose.pose.position.x = east;
    odom.pose.pose.position.y = north;
    odom.pose.pose.position.z = up;
    const Eigen::Quaterniond q = rpyToQuat(msg->rpy.roll, msg->rpy.pitch, msg->rpy.yaw);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // [P2 FIX 2026-07-10] Fail closed on non-finite covariance. The pose
    // VALUES are validated above, but a solution can be value-finite with a
    // NaN covariance (degraded heading with an unsolved variance). Downstream
    // gates split by comparison direction: position gates use `<= max`
    // (NaN already fails closed), but GICP's INS yaw-quality gate keys on
    // `cov_yaw > 0` — false for NaN — so a NaN yaw variance used to pass as
    // "unpopulated" (fail-OPEN). Map non-finite variances to +inf = KNOWN-BAD,
    // matching GLIM gnss_global's sanitize_yaw_var policy.
    const auto fail_closed = [](double v) {
      return std::isfinite(v) ? v : std::numeric_limits<double>::infinity();
    };
    const auto& pc = msg->position_covariance;
    const auto& rc = msg->rpy_covariance;
    odom.pose.covariance[0] = fail_closed(static_cast<double>(pc[0]));
    odom.pose.covariance[7] = fail_closed(static_cast<double>(pc[4]));
    odom.pose.covariance[14] = fail_closed(static_cast<double>(pc[8]));
    odom.pose.covariance[21] = fail_closed(static_cast<double>(rc[0]) * kDegToRadSq);
    odom.pose.covariance[28] = fail_closed(static_cast<double>(rc[4]) * kDegToRadSq);
    odom.pose.covariance[35] = fail_closed(static_cast<double>(rc[8]) * kDegToRadSq);

    odom.twist.twist.linear.x = msg->velflu.x;
    odom.twist.twist.linear.y = msg->velflu.y;
    odom.twist.twist.linear.z = msg->velflu.z;
    const auto& vc = msg->velflu_covariance;
    odom.twist.covariance[0] = static_cast<double>(vc[0]);
    odom.twist.covariance[7] = static_cast<double>(vc[4]);
    odom.twist.covariance[14] = static_cast<double>(vc[8]);
    // P2#2: angular twist from the latest Atlas gyro (see publishImu note).
    // Only when reasonably fresh; a stale gyro (IMU stream stalled) is worse
    // than the documented "no angular twist" default of zero, which consumers
    // (gicp_localization snap) now detect and back-fill from their own IMU.
    // Both stamps are in the ROS-mapped time domain (odom.header.stamp was set
    // via clock_mapper_.toRos above; the gyro freshness check below is now
    // stamp), so the age comparison is domain-consistent.
    // [P3 FIX 2026-07-10] WALL-CLOCK freshness, cached at INGEST. The old
    // publish-time stamp-domain age check was permanently dead in
    // arrival_retime mode: the 128-sample queue delays publish by ~1.28 s at
    // 100 Hz, always exceeding the 0.5 s window. Angular rate is a physical
    // quantity — its freshness is how long ago it was MEASURED (wall time),
    // independent of any stamp domain; this also removes the last stamp-
    // domain mixing from the backfill.
    const double gyro_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - latest_gyro_wall_).count();
    if (has_gyro_ && gyro_age < kMaxGyroAgeSec) {
      odom.twist.twist.angular = latest_gyro_;
      // Conservative fixed rate variance (Atlas imu_calibrated gyro noise is
      // ~1e-2 rad/s class at 100 Hz bandwidth).
      odom.twist.covariance[21] = 1.0e-4;
      odom.twist.covariance[28] = 1.0e-4;
      odom.twist.covariance[35] = 1.0e-4;
    }

    if (publish_gnss_pose_) {
      auto gnss = makeGnssMsg(odom, "/gnss");
      gnss_pub_->publish(gnss);
      ++gnss_out_count_;
    }

    odom_pub_->publish(odom);
    ++odom_out_count_;
    if (isRtkFixed(*msg)) {
      auto rtk = odom;
      rtk.header.stamp = monotonicStamp("/gps_p1/filtered_odom_rtk_fixed", stampToSec(odom.header.stamp));
      if (publish_gnss_pose_) {
        auto gnss_rtk = makeGnssMsg(rtk, "/gnss_rtk_fixed");
        gnss_rtk_pub_->publish(gnss_rtk);
        ++gnss_rtk_out_count_;
      }
      odom_rtk_pub_->publish(rtk);
      ++rtk_out_count_;
    }

    flushP1ImuQueue();
  }

  bool isRtkFixed(const fusion_engine_msgs::msg::Pose& msg) const
  {
    return adapter::posePassesRtkGate(msg, rtk_max_var_xy_, rtk_max_var_z_);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped makeGnssMsg(
      const nav_msgs::msg::Odometry& odom, const std::string& topic)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header = odom.header;
    out.header.stamp = monotonicStamp(topic, stampToSec(odom.header.stamp));
    out.pose = odom.pose;
    return out;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    ++imu_in_count_;
    last_imu_wall_time_ = std::chrono::steady_clock::now();
    // [P3 FIX 2026-07-10] Cache the gyro at INGEST (see poseCallback): the
    // measurement is fresh NOW regardless of when the retime queue publishes.
    latest_gyro_ = msg->angular_velocity;
    latest_gyro_wall_ = last_imu_wall_time_;
    has_gyro_ = true;
    const double stamp = stampToSec(msg->header.stamp);
    if (!imu_p1_sidecar_.empty()) {
      double sidecar_p1 = 0.0;
      if (lookupSidecarP1(stamp, sidecar_p1)) {
        QueuedImu queued;
        queued.msg = *msg;
        queued.p1 = sidecar_p1;
        p1_imu_queue_.push_back(std::move(queued));
        flushP1ImuQueue();
        return;
      }
      // [P2 FIX 2026-07-09] With a sidecar configured, a tolerance miss must
      // DROP the sample, not fall through to arrival-retime: the fallback
      // stranded the sample in the arrival queue (which only drains at
      // 128-depth or a full-stream stall), then injected it minutes later,
      // stale and with a fabricated last+1us stamp. Losing ~1 sample per
      // burst at 100+ Hz is negligible; mixing stamp domains is not.
      ++sidecar_miss_drop_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No IMU P1 sidecar match for capture stamp %.6f; sample dropped (%lu so far).",
        stamp, static_cast<unsigned long>(sidecar_miss_drop_count_));
      return;
    }

    const bool p1_like = stamp > 0.0 && stamp < p1_like_threshold_sec_;
    const bool use_p1 = imu_stamp_mode_ == "p1" || (imu_stamp_mode_ == "auto" && p1_like);

    if (use_p1) {
      QueuedImu queued;
      queued.msg = *msg;
      queued.p1 = stamp;
      p1_imu_queue_.push_back(std::move(queued));
      flushP1ImuQueue();
      return;
    }

    if (imu_stamp_mode_ != "auto" && imu_stamp_mode_ != "arrival_retime") {
      RCLCPP_WARN_ONCE(get_logger(), "Unsupported imu_stamp_mode='%s'; using arrival_retime.",
                       imu_stamp_mode_.c_str());
    }

    updateImuPeriod(stamp);
    QueuedImu queued;
    queued.msg = *msg;
    queued.arrival = stamp;
    arrival_imu_queue_.push_back(std::move(queued));
    while (arrival_imu_queue_.size() > imu_lookahead_) {
      publishFrontArrivalRetimedImu();
    }
  }

  void updateImuPeriod(double arrival)
  {
    if (last_imu_arrival_ > 0.0) {
      const double dt = arrival - last_imu_arrival_;
      if (dt > 0.5 * nominal_imu_period_sec_ && dt < 1.5 * nominal_imu_period_sec_) {
        imu_period_samples_.push_back(dt);
        if (imu_period_samples_.size() > 2048) {
          imu_period_samples_.erase(imu_period_samples_.begin());
        }
        auto tmp = imu_period_samples_;
        const size_t mid = tmp.size() / 2;
        std::nth_element(tmp.begin(), tmp.begin() + static_cast<long>(mid), tmp.end());
        imu_period_sec_ = tmp[mid];
      }
    }
    last_imu_arrival_ = arrival;
  }

  std::vector<double> retimeArrivalQueue() const
  {
    std::vector<double> stamps;
    stamps.reserve(arrival_imu_queue_.size());
    for (const auto& q : arrival_imu_queue_) {
      stamps.push_back(q.arrival);
    }
    return adapter::retimeArrivalStamps(stamps, imu_period_sec_);
  }

  void publishFrontArrivalRetimedImu()
  {
    if (arrival_imu_queue_.empty()) {
      return;
    }
    const auto stamps = retimeArrivalQueue();
    auto queued = arrival_imu_queue_.front();
    arrival_imu_queue_.pop_front();
    publishImu(queued.msg, stamps.front());
  }

  void flushP1ImuQueue()
  {
    if (!clock_mapper_.ready()) {
      // [P2 FIX 2026-07-09] Previously this returned silently forever: with a
      // wrong pose topic or GNSS down, every IMU sample queued unbounded and
      // /gps_p1/imu published NOTHING with no diagnostic. Bound the queue
      // (~10 s at 200 Hz) and say why the output is silent.
      constexpr size_t kMaxP1ImuQueue = 2000;
      if (p1_imu_queue_.size() > kMaxP1ImuQueue) {
        while (p1_imu_queue_.size() > kMaxP1ImuQueue) {
          p1_imu_queue_.pop_front();
          ++p1_imu_dropped_not_ready_count_;
        }
      }
      if (!p1_imu_queue_.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "/gps_p1/imu silent: %zu IMU samples queued but no pose pair yet on '%s' "
                             "(clock mapper not ready; %lu oldest dropped)",
                             p1_imu_queue_.size(), pose_input_topic_.c_str(),
                             static_cast<unsigned long>(p1_imu_dropped_not_ready_count_));
      }
      return;
    }
    while (!p1_imu_queue_.empty()) {
      auto queued = p1_imu_queue_.front();
      p1_imu_queue_.pop_front();
      publishImu(queued.msg, clock_mapper_.toRos(queued.p1));
    }
  }

  void flushTimerCallback()
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    flushP1ImuQueue();
    if (arrival_imu_queue_.empty()) {
      return;
    }
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_imu_wall_time_).count();
    if (elapsed < imu_flush_timeout_sec_) {
      return;
    }
    while (!arrival_imu_queue_.empty()) {
      publishFrontArrivalRetimedImu();
    }
  }

  void publishImu(sensor_msgs::msg::Imu msg, double stamp)
  {
    msg.header.stamp = monotonicStamp("/gps_p1/imu", stamp);
    msg.header.frame_id = imu_frame_id_;
    // Cache the latest gyro so poseCallback can populate the odometry's
    // angular twist (P2#2: FusionEngine Pose carries no body rates, and a
    // zero twist.angular made every GT snap in gicp_localization reset the
    // published angular velocity to zero mid-turn). Angular rate is
    // location-independent on a rigid body, and IMUOutput is already rotated
    // into vehicle body axes == odom child_frame axes, so the gyro is exactly
    // the odometry twist.angular. Guarded by core_mutex_ (all callers hold it).
    // (gyro cache moved to imuCallback ingest — see P3 FIX 2026-07-10)
    imu_pub_->publish(msg);
    ++imu_out_count_;
  }

  builtin_interfaces::msg::Time monotonicStamp(const std::string& topic, double sec)
  {
    double& last = last_stamp_by_topic_[topic];
    if (last > 0.0 && sec <= last) {
      // [P2 FIX 2026-07-09] The +1us nudge is for us-scale jitter only. A
      // regression larger than 1 s is a legitimate time reset (bag loop,
      // device power-cycle): pass it through and re-anchor, instead of
      // freezing the topic at last+1us forever.
      if (last - sec > 1.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Stamp on '%s' regressed by %.3f s — re-anchoring (time reset)",
                             topic.c_str(), last - sec);
        last = sec;
        return secToStamp(sec);
      }
      sec = last + 1e-6;
    }
    last = sec;
    return secToStamp(sec);
  }

  std::string pose_input_topic_;
  std::string imu_input_topic_;
  std::string imu_stamp_mode_;
  std::string imu_frame_id_;
  std::string odom_frame_id_;
  std::string body_frame_id_;
  std::string summary_output_path_;
  std::string pose_input_reliability_;
  std::string imu_input_reliability_;
  std::string imu_p1_sidecar_path_;

  P1ClockMapper clock_mapper_;
  double p1_like_threshold_sec_ = 100000000.0;
  double imu_p1_sidecar_match_tolerance_sec_ = 0.02;
  size_t imu_lookahead_ = 128;
  double imu_flush_timeout_sec_ = 0.5;
  double nominal_imu_period_sec_ = 0.01;
  double imu_period_sec_ = 0.01;
  double last_imu_arrival_ = 0.0;
  double rtk_max_var_xy_ = 1e-3;
  double rtk_max_var_z_ = 5e-3;
  int pose_input_qos_depth_ = 100;
  int imu_input_qos_depth_ = 100;
  bool publish_gnss_pose_ = true;
  double local_origin_lat_ = 0.0;
  double local_origin_lon_ = 0.0;
  double local_origin_alt_ = 0.0;
  GeographicLib::LocalCartesian local_cartesian_;
  std::chrono::steady_clock::time_point last_imu_wall_time_ = std::chrono::steady_clock::now();

  std::vector<double> imu_period_samples_;
  std::deque<QueuedImu> arrival_imu_queue_;
  std::deque<QueuedImu> p1_imu_queue_;
  std::vector<ImuP1SidecarSample> imu_p1_sidecar_;
  size_t imu_p1_sidecar_index_ = 0;
  double last_p1_time_ = std::numeric_limits<double>::quiet_NaN();  // P2 fix: epoch-reset detection
  uint64_t pose_dropped_invalid_count_ = 0;       // P2 fix: NaN/Invalid solution drops
  uint64_t sidecar_miss_drop_count_ = 0;          // P2 fix: sidecar tolerance misses
  uint64_t p1_imu_dropped_not_ready_count_ = 0;   // P2 fix: bounded not-ready queue drops
  std::map<std::string, double> last_stamp_by_topic_;

  rclcpp::CallbackGroup::SharedPtr pose_group_;
  rclcpp::CallbackGroup::SharedPtr imu_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_rtk_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_rtk_pub_;

  rclcpp::Subscription<fusion_engine_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
  rclcpp::TimerBase::SharedPtr summary_timer_;

  uint64_t pose_in_count_ = 0;
  uint64_t gnss_out_count_ = 0;
  uint64_t gnss_rtk_out_count_ = 0;
  uint64_t odom_out_count_ = 0;
  uint64_t rtk_out_count_ = 0;
  uint64_t imu_in_count_ = 0;
  uint64_t imu_out_count_ = 0;
  uint64_t imu_p1_sidecar_match_count_ = 0;
  uint64_t imu_p1_sidecar_miss_count_ = 0;
  uint64_t imu_p1_sidecar_skip_count_ = 0;

  // P2#2: latest Atlas gyro (vehicle body axes), used to populate
  // /gps_p1/filtered_odom twist.angular. Guarded by core_mutex_.
  static constexpr double kMaxGyroAgeSec = 0.5;
  geometry_msgs::msg::Vector3 latest_gyro_;
  std::chrono::steady_clock::time_point latest_gyro_wall_{};  // P3: wall-clock freshness
  bool has_gyro_ = false;

  mutable std::mutex core_mutex_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Adapter>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
