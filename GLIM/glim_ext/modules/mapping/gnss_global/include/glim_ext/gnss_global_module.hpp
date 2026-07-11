#include <deque>
#include <cmath>
#include <limits>
#include <atomic>
#include <mutex>
#include <thread>
#include <numeric>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define GLIM_ROS2

#include <boost/format.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/concurrent_vector.hpp>

#ifdef GLIM_ROS2
#include <glim/util/extension_module_ros2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

using ExtensionModuleBase = glim::ExtensionModuleROS2;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using PoseWithCovarianceStampedConstPtr = geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr;
using Odometry = nav_msgs::msg::Odometry;
using OdometryConstPtr = nav_msgs::msg::Odometry::ConstSharedPtr;

template <typename Stamp>
double to_sec(const Stamp& stamp) {
  return stamp.sec + stamp.nanosec / 1e9;
}
#else
#include <glim/util/extension_module_ros.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.hpp>

using ExtensionModuleBase = glim::ExtensionModuleROS;
#endif

#include <spdlog/spdlog.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PoseRotationPrior.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <glim/util/logging.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim_ext/util/config_ext.hpp>
#include <glim/util/urdf_transforms.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;

struct GNSSData {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double stamp = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  bool has_orientation = false;
  // P5#1: reported yaw variance (rad^2) from pose.covariance[35].
  // < 0 means "not populated by the publisher" (passes the yaw-quality gate
  // for backwards compatibility with covariance-less GNSS sources).
  double yaw_var = -1.0;
};

/**
 * @brief Naive implementation of GNSS constraints for the global optimization.
 * @note  This implementation is very naive and ignores the IMU-GNSS transformation and GNSS observation covariance.
 *        If you use a precise GNSS (e.g., RTK), consider asking for a closed-source extension module with better GNSS handling.
 */
class GNSSGlobal : public ExtensionModuleBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GNSSGlobal() : logger(create_module_logger("gnss_global")) {
    logger->info("initializing GNSS global constraints");
    const std::string config_path = glim::GlobalConfigExt::get_config_path("config_gnss_global");
    logger->info("gnss_global_config_path={}", config_path);

    glim::Config config(config_path);
    gnss_topic = config.param<std::string>("gnss", "gnss_topic", "/pose_with_cov");
    gnss_msg_type = config.param<std::string>("gnss", "gnss_msg_type", "geometry_msgs/msg/PoseWithCovarianceStamped");
    prior_inf_scale = config.param<Eigen::Vector3d>("gnss", "prior_inf_scale", Eigen::Vector3d(1e3, 1e3, 0.0));
    enable_orientation_prior = config.param<bool>("gnss", "enable_orientation_prior", false);
    orientation_prior_inf_scale = config.param<Eigen::Vector3d>("gnss", "orientation_prior_inf_scale", Eigen::Vector3d(1e2, 1e2, 1e2));
    // P5#1: yaw-quality gate for the orientation prior. The RTK gate upstream
    // (adapter/rtk_fixed filter) qualifies POSITION quality only; a
    // position-FIXED sample can still carry a degraded/unsolved dual-antenna
    // heading (secondary-antenna outage, short baseline multipath). Feeding
    // that heading as a stiff PoseRotationPrior would twist the map exactly
    // where we are trying to pin it. Skip the orientation prior when the
    // publisher's reported yaw sigma (sqrt of pose.covariance[35]) exceeds
    // this threshold. <= 0 disables the gate; samples WITHOUT a populated
    // yaw covariance pass (backwards compatible with covariance-less sources).
    orientation_prior_max_yaw_sigma_deg = config.param<double>("gnss", "orientation_prior_max_yaw_sigma_deg", 3.0);
    min_baseline = config.param<double>("gnss", "min_baseline", 5.0);
    // [P1 FIX 2026-07-09] Maximum GNSS bracket width for submap association.
    // Without this bound, an RTK dropout leaves the queue with (last sample
    // before the gap, first sample after it) and every submap inside the gap
    // is anchored to a STRAIGHT-LINE chord between dropout entry and exit —
    // at prior_inf_scale stiffness (~1 cm) that warps the map by the chord
    // sagitta on any curved segment, exactly where the documented contract
    // says the factor stream must go silent. Submaps whose bracket exceeds
    // this width are left un-anchored (LiDAR+IMU only). <= 0 disables the
    // bound (legacy behavior).
    max_interp_gap_sec = config.param<double>("gnss", "max_interp_gap_sec", 1.0);

    if (enable_orientation_prior && orientation_prior_inf_scale.minCoeff() < 0.0) {
      logger->warn("orientation prior enabled but orientation_prior_inf_scale has negative values; disabling orientation prior");
      enable_orientation_prior = false;
    }

    // Resolve IMU -> GNSS antenna lever arm from URDF (if configured).
    // urdf_path / urdf_imu_frame come from config_sensors.json (shared with the lidar/IMU calibration).
    // urdf_gnss_frame is gnss_global-specific (e.g., "gps_antenna_top").
    t_imu_gnss.setZero();
    warned_missing_orientation_for_lever_arm = false;

    // Explicit master switch. Tightly-coupled INS receivers (Novatel SPAN,
    // Septentrio AsteRx-i, Atlas Duo, ...) compensate the antenna->IMU lever
    // arm in firmware via LEVERARMCONFIG; doing it here as well would
    // double-compensate. Default true to preserve upstream raw-GNSS behavior,
    // but the shipped config sets it false for this vehicle.
    const bool enable_lever_arm = config.param<bool>("gnss", "enable_lever_arm", true);
    if (!enable_lever_arm) {
      logger->info("lever-arm compensation explicitly disabled via gnss.enable_lever_arm=false; t_imu_gnss=0");
    }

    try {
      glim::Config config_sensors(glim::GlobalConfig::get_config_path("config_sensors"));
      const std::string urdf_path = config_sensors.param<std::string>("sensors", "urdf_path", "");
      const std::string urdf_imu_frame = config_sensors.param<std::string>("sensors", "urdf_imu_frame", "");
      const std::string urdf_gnss_frame = config.param<std::string>("gnss", "urdf_gnss_frame", "");

      if (enable_lever_arm && !urdf_path.empty() && !urdf_imu_frame.empty() && !urdf_gnss_frame.empty()) {
        const auto urdf_transforms = glim::parse_urdf_transforms(urdf_path);
        const Eigen::Isometry3d T_imu_gnss = glim::compute_transform(urdf_transforms, urdf_imu_frame, urdf_gnss_frame);
        t_imu_gnss = T_imu_gnss.translation();
        logger->info("URDF lever arm t_imu_gnss ({} -> {}): [{:.4f}, {:.4f}, {:.4f}]", urdf_imu_frame, urdf_gnss_frame, t_imu_gnss.x(), t_imu_gnss.y(), t_imu_gnss.z());

        // The lever-arm correction below assumes the GNSS message's orientation
        // is the IMU body's rotation in world. That's true when the antenna
        // frame is axis-aligned with the IMU frame (URDF rotation = identity)
        // OR when the publisher fuses GNSS+IMU and reports IMU-body orientation
        // directly (typical INS like Novatel SPAN, Septentrio AsteRx-i). When
        // the URDF rotation is non-identity AND the publisher reports
        // antenna-frame orientation, the lever arm is applied in the wrong
        // frame. Warn so a future URDF tweak doesn't silently produce a bias.
        const Eigen::Matrix3d R_imu_gnss = T_imu_gnss.linear();
        if (!R_imu_gnss.isApprox(Eigen::Matrix3d::Identity(), 1e-3)) {
          const double off_deg = Eigen::AngleAxisd(R_imu_gnss).angle() * 180.0 / M_PI;
          logger->warn(
            "URDF rotation between {} and {} is {:.2f} deg off identity; lever-arm "
            "correction is correct only if the GNSS publisher reports IMU-body "
            "orientation in world (e.g., an INS). Antenna-frame orientation will "
            "be biased.",
            urdf_imu_frame, urdf_gnss_frame, off_deg);
        }
      } else if (enable_lever_arm) {
        logger->info("URDF lever arm not configured (urdf_path/urdf_imu_frame/urdf_gnss_frame); GNSS positions used as-is");
      }
    } catch (const std::exception& e) {
      logger->error("failed to compute t_imu_gnss from URDF: {}; lever arm compensation disabled", e.what());
      t_imu_gnss.setZero();
    }

    transformation_initialized = false;
    T_world_utm.setIdentity();
    warned_missing_orientation = false;

    kill_switch = false;
    thread = std::thread([this] { backend_task(); });

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&GNSSGlobal::on_insert_submap, this, _1));
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&GNSSGlobal::on_smoother_update, this, _1, _2, _3));
  }
  ~GNSSGlobal() {
    kill_switch = true;
    thread.join();
  }

  virtual void at_exit(const std::string& dump_path) override {
    // [P3 FIX 2026-07-10] Guarded: after a flush TIMEOUT GlimROS::save() can
    // reach here while the backend thread is mid-write in the initialization
    // block — a torn T_world_utm.txt (consumed by the map exporter) and a UB
    // data race. Cold path on both sides; a plain mutex suffices.
    std::lock_guard<std::mutex> lock(T_world_utm_mtx_);
    if (transformation_initialized) {
      save_transformation_to_file(dump_path);
    }
  }

  // Report pending work so GlimROS::save() drains us before the final global
  // optimize. The backend produces position/heading factors on its own thread
  // and delivers them only through on_smoother_update(); if save() ran while we
  // still had undelivered factors they would never reach the serialized graph.
  // We are NOT done while: a batch is mid-process (processing_); submaps are
  // queued for us; a submap in our local queue is still bracketable
  // (pending_associable_); or GNSS is queued WHILE a submap is waiting for it
  // (closes the bag-EOF race where the bracketing GNSS hasn't been drained).
  // [P2 FIX 2026-07-09] A GNSS backlog with NO submap waiting is deliberately
  // NOT pending work: the old predicate counted every queued GNSS message,
  // and since glim_rosbag/glim_pcap_rosbag poll needs_wait() after EVERY bag
  // message while this queue drains only at the backend's 100 ms cadence,
  // offline replay was throttled to roughly the GNSS message rate regardless
  // of playback_speed — with the 1 s throttle timeout spamming "extension
  // module may be hanged" warnings.
  virtual bool needs_wait() const override {
    return processing_ || !input_submap_queue.empty() || pending_associable_ ||
           (!input_gnss_queue.empty() && submaps_waiting_);
  }

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions() override {
    if (gnss_msg_type == "nav_msgs/msg/Odometry") {
      const auto sub = std::make_shared<TopicSubscription<Odometry>>(gnss_topic, gnss_msg_type, [this](const OdometryConstPtr msg) { gnss_callback(msg); });
      return {sub};
    }

    const auto sub = std::make_shared<TopicSubscription<PoseWithCovarianceStamped>>(
      gnss_topic,
      gnss_msg_type,
      [this](const PoseWithCovarianceStampedConstPtr msg) { gnss_callback(msg); });
    return {sub};
  }

  void gnss_callback(const PoseWithCovarianceStampedConstPtr& gnss_msg) {
    const auto& pos = gnss_msg->pose.pose.position;
    const auto& ori = gnss_msg->pose.pose.orientation;
    const double yaw_var = sanitize_yaw_var(gnss_msg->pose.covariance[35]);
    push_gnss_data(to_sec(gnss_msg->header.stamp), pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w, yaw_var);
  }

  void gnss_callback(const OdometryConstPtr& gnss_msg) {
    const auto& pos = gnss_msg->pose.pose.position;
    const auto& ori = gnss_msg->pose.pose.orientation;
    const double yaw_var = sanitize_yaw_var(gnss_msg->pose.covariance[35]);
    push_gnss_data(to_sec(gnss_msg->header.stamp), pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w, yaw_var);
  }

  // [P3 FIX 2026-07-10] Snapshot the submap ORIGIN TRANSLATION at insert
  // time: on_insert_submap runs synchronously on the mapping thread, but the
  // backend thread previously dereferenced the live submap->T_world_origin
  // later, racing GlobalMapping's post-optimization rewrites (torn/mixed
  // reads feeding the one-shot T_world_utm fit). Only the translation is
  // consumed (Umeyama fit, baseline check, debug logs) and the baseline norm
  // is rotation-invariant, so a Vector3d snapshot suffices.
  struct QueuedSubmap {
    SubMap::ConstPtr submap;
    Eigen::Vector3d t_world_origin_snap;
  };
  void on_insert_submap(const SubMap::ConstPtr& submap) {
    input_submap_queue.push_back({submap, submap->T_world_origin.translation()});
  }

  void on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    const auto factors = output_factors.get_all_and_clear();
    if (!factors.empty()) {
      logger->debug("insert {} GNSS prior factors", factors.size());
      new_factors.add(factors);
    }
  }

  void backend_task() {
    logger->info("starting GNSS global thread");
    std::deque<GNSSData, Eigen::aligned_allocator<GNSSData>> utm_queue;
    std::deque<QueuedSubmap> submap_queue;
    std::vector<Eigen::Vector3d> submap_t_snap;  // parallel to `submaps` (P3 fix)

    while (!kill_switch) {
      // Bound the loop rate so re-attempting association on every GNSS arrival
      // (below) cannot busy-spin while a submap waits to be bracketed.
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // [P2 FIX 2026-07-09] Raise processing_ BEFORE draining the input
      // queues. Previously the queues were cleared first and processing_ was
      // raised only after the early-continue check: in that window ALL
      // needs_wait() observables read false while this thread held an
      // undelivered batch — a save() polling at 50 ms could flush past it
      // and the final submaps' GNSS factors silently never reached the
      // serialized graph (TOCTOU). With the flag raised first, at least one
      // observable is true whenever work exists; the early-continue branch
      // lowers it again immediately when there is nothing to do.
      processing_ = true;

      // Convert GeoPoint(lat/lon) to UTM
      const auto gnss_data = input_gnss_queue.get_all_and_clear();
      // [P2 FIX 2026-07-09] Enforce stamp monotonicity on insert. Duplicate
      // stamps make interpolate_gnss_data divide by zero (NaN position ->
      // NaN factor); out-of-order stamps break std::lower_bound's
      // partitioning precondition below (potential begin()-1 dereference).
      // Realistic producers: overlapping multi-bag globs, filter-node
      // restarts re-emitting samples.
      for (const auto& g : gnss_data) {
        if (!utm_queue.empty() && g.stamp <= utm_queue.back().stamp) {
          if (!warned_nonmonotonic_gnss) {
            logger->warn("dropping non-monotonic GNSS sample (stamp={:.6f} <= newest {:.6f}) — further drops silent", g.stamp, utm_queue.back().stamp);
            warned_nonmonotonic_gnss = true;
          }
          continue;
        }
        utm_queue.push_back(g);
      }

      // Add new submaps to the local queue.
      const auto new_submaps = input_submap_queue.get_all_and_clear();
      submap_queue.insert(submap_queue.end(), new_submaps.begin(), new_submaps.end());
      submaps_waiting_ = !submap_queue.empty();  // [P2 FIX 2026-07-09] see needs_wait()

      // Attempt association whenever there is a pending submap AND something new
      // arrived this cycle: new submaps to place, OR new GNSS that may have just
      // bracketed a submap already waiting in submap_queue. The old code skipped
      // the pass whenever new_submaps was empty, which stranded the last submap
      // at bag EOF -- its bracketing GNSS arrives with no accompanying submap.
      if (submap_queue.empty() || (gnss_data.empty() && new_submaps.empty())) {
        pending_associable_ = !submap_queue.empty() && !utm_queue.empty() &&
                              submap_queue.front().submap->frames.back()->stamp < utm_queue.back().stamp;
        processing_ = false;  // nothing to do this cycle
        continue;
      }
      // (processing_ already true — raised before the queue drain above)

      // Remove submaps that are created earlier than the oldest GNSS data
      while (!utm_queue.empty() && !submap_queue.empty() && submap_queue.front().submap->frames.front()->stamp < utm_queue.front().stamp) {
        submap_queue.pop_front();
      }

      // Interpolate UTM coords and associate with submaps
      while (!utm_queue.empty() && !submap_queue.empty() && submap_queue.front().submap->frames.front()->stamp > utm_queue.front().stamp &&
             submap_queue.front().submap->frames.back()->stamp < utm_queue.back().stamp) {
        const auto& submap = submap_queue.front().submap;
        const Eigen::Vector3d t_snap = submap_queue.front().t_world_origin_snap;
        const double stamp = submap->frames[submap->frames.size() / 2]->stamp;

        const auto right = std::lower_bound(utm_queue.begin(), utm_queue.end(), stamp, [](const GNSSData& utm, const double t) { return utm.stamp < t; });
        // [P3 FIX 2026-07-09] right == LAST sample is a perfectly valid
        // bracket (interpolation needs only left/right). The old extra
        // refusal of (right + 1) == end demanded a SECOND sample after the
        // submap while pending_associable_ counted the submap as bracketable
        // with just one — on a dropout-at-EOF the two criteria disagreed
        // forever: the loop broke every cycle, needs_wait() never cleared,
        // and save() burned its full flush timeout before dropping the
        // submap's factors.
        if (right == utm_queue.end()) {
          logger->warn("invalid condition in GNSS global module!!");
          break;
        }
        // [P2 FIX 2026-07-09] Belt-and-braces for the lower_bound
        // precondition: with the monotonic insert above this cannot fire,
        // but right == begin() would make (right - 1) UB.
        if (right == utm_queue.begin()) {
          logger->warn("GNSS association: bracket left edge missing (right == begin); skipping submap");
          submap_queue.pop_front();
          continue;
        }
        const auto left = right - 1;
        logger->debug("submap={:.6f} utm_left={:.6f} utm_right={:.6f}", stamp, left->stamp, right->stamp);

        // [P1 FIX 2026-07-09] Do NOT interpolate across a GNSS dropout. When
        // the bracket spans more than max_interp_gap_sec, the submap sits
        // inside a gap where the RTK filter went silent — a chord between
        // dropout entry/exit is NOT a measurement. Leave the submap
        // un-anchored (LiDAR+IMU odometry + loop closures carry it), exactly
        // as the documented GNSS-denied contract promises.
        if (max_interp_gap_sec > 0.0 && (right->stamp - left->stamp) > max_interp_gap_sec) {
          logger->warn(
            "GNSS association: bracket gap {:.2f}s > max_interp_gap_sec {:.2f}s (dropout) — submap at {:.3f} left un-anchored",
            right->stamp - left->stamp, max_interp_gap_sec, stamp);
          submap_queue.pop_front();
          continue;
        }

        const GNSSData interpolated = interpolate_gnss_data(*left, *right, stamp);

        submaps.push_back(submap);
        submap_t_snap.push_back(t_snap);
        submap_coords.push_back(interpolated);

        submap_queue.pop_front();
        utm_queue.erase(utm_queue.begin(), left);
      }

      // Initialize T_world_utm
      if (!transformation_initialized && !submaps.empty() &&
          (submap_t_snap.back() - submap_t_snap.front()).norm() > min_baseline) {
        Eigen::Vector3d mean_est = Eigen::Vector3d::Zero();
        Eigen::Vector3d mean_gnss = Eigen::Vector3d::Zero();
        for (int i = 0; i < submaps.size(); i++) {
          mean_est += submap_t_snap[i];
          mean_gnss += submap_coords[i].position;
        }
        mean_est /= submaps.size();
        mean_gnss /= submaps.size();

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (int i = 0; i < submaps.size(); i++) {
          const Eigen::Vector3d centered_est = submap_t_snap[i] - mean_est;
          const Eigen::Vector3d centered_gnss = submap_coords[i].position - mean_gnss;
          cov += centered_gnss * centered_est.transpose();
        }
        cov /= submaps.size();

        const Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov.block<2, 2>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Matrix2d U = svd.matrixU();
        const Eigen::Matrix2d V = svd.matrixV();
        const Eigen::Matrix2d D = svd.singularValues().asDiagonal();
        Eigen::Matrix2d S = Eigen::Matrix2d::Identity();

        const double det = U.determinant() * V.determinant();
        if (det < 0.0) {
          S(1, 1) = -1;
        }

        Eigen::Isometry3d T_utm_world = Eigen::Isometry3d::Identity();
        T_utm_world.linear().block<2, 2>(0, 0) = U * S * V.transpose();
        T_utm_world.translation() = mean_gnss - T_utm_world.linear() * mean_est;

        {
          std::lock_guard<std::mutex> lock(T_world_utm_mtx_);
          T_world_utm = T_utm_world.inverse();
        }

        for (int i = 0; i < submaps.size(); i++) {
          const Eigen::Vector3d gnss = T_world_utm * submap_coords[i].position;
          logger->debug("submap={} gnss={}", convert_to_string(submap_t_snap[i]), convert_to_string(gnss));
        }

        logger->info("T_world_utm={}", convert_to_string(T_world_utm));
        {
          std::lock_guard<std::mutex> lock(T_world_utm_mtx_);
          transformation_initialized = true;  // published under the same lock as the matrix
        }
      }

      // Add GNSS prior factors for EVERY associated submap that doesn't have
      // them yet, not just submaps.back(). The association loop above banks all
      // eligible submaps (offline replay associates many per cycle), and the
      // whole backlog accumulated before T_world_utm initialized is still
      // unfactored at the moment it does. Emitting only for .back() silently
      // skips position + yaw priors for all but the newest submap in a batch
      // (including most pre-baseline submaps). Backfill from the cursor instead.
      if (transformation_initialized) {
        for (size_t i = factored_submap_count; i < submaps.size(); i++) {
          const GNSSData& gnss = submap_coords[i];
          const auto& submap = submaps[i];
          const Eigen::Vector3d xyz = T_world_utm * gnss.position;
          logger->debug("submap={} gnss={}", convert_to_string(submap_t_snap[i]), convert_to_string(xyz));

          // note: should use a more accurate information matrix
          const auto model = gtsam::noiseModel::Diagonal::Precisions(prior_inf_scale);
          output_factors.push_back(
            gtsam::NonlinearFactor::shared_ptr(new gtsam::PoseTranslationPrior<gtsam::Pose3>(X(submap->id), xyz, model)));

          // P5#1 yaw-quality gate: skip the heading prior when the publisher
          // reports a degraded yaw solution (dual-antenna heading can be bad
          // while position is RTK-FIXED — the upstream RTK filter qualifies
          // position only). Unpopulated covariance (yaw_var < 0) passes.
          const double max_yaw_sigma_rad = orientation_prior_max_yaw_sigma_deg * M_PI / 180.0;
          const bool yaw_quality_ok =
            orientation_prior_max_yaw_sigma_deg <= 0.0 || gnss.yaw_var < 0.0 ||
            gnss.yaw_var <= max_yaw_sigma_rad * max_yaw_sigma_rad;

          if (enable_orientation_prior && gnss.has_orientation && yaw_quality_ok) {
            const Eigen::Matrix3d R_world_gnss = T_world_utm.linear() * gnss.orientation.toRotationMatrix();
            const auto rotation_model = gtsam::noiseModel::Diagonal::Precisions(orientation_prior_inf_scale);
            output_factors.push_back(
              gtsam::NonlinearFactor::shared_ptr(new gtsam::PoseRotationPrior<gtsam::Pose3>(X(submap->id), gtsam::Rot3(R_world_gnss), rotation_model)));
          } else if (enable_orientation_prior && gnss.has_orientation && !yaw_quality_ok) {
            ++yaw_gate_skip_count;
            if (yaw_gate_skip_count == 1 || yaw_gate_skip_count % 50 == 0) {
              logger->warn(
                "orientation prior skipped for submap {}: reported yaw sigma {:.2f} deg > max {:.2f} deg "
                "({} skipped so far) — position prior still applied",
                submap->id, std::sqrt(gnss.yaw_var) * 180.0 / M_PI,
                orientation_prior_max_yaw_sigma_deg, yaw_gate_skip_count);
            }
          } else if (enable_orientation_prior && !warned_missing_orientation) {
            logger->warn("orientation prior enabled but GNSS messages contain invalid quaternions; skipping orientation priors");
            warned_missing_orientation = true;
          }
        }
        factored_submap_count = submaps.size();
      }

      // Pending state for needs_wait(): an associable submap still remains while
      // the front of submap_queue has a GNSS sample after it (so it can be
      // bracketed/interpolated). Trailing submaps with NO GNSS after them are
      // genuinely un-factorable, so they are NOT counted -- save() must not block
      // waiting on them.
      pending_associable_ = !submap_queue.empty() && !utm_queue.empty() &&
                            submap_queue.front().submap->frames.back()->stamp < utm_queue.back().stamp;
      submaps_waiting_ = !submap_queue.empty();
      processing_ = false;
    }
  }

private:
  void save_transformation_to_file(const std::string& dump_path) {
    const std::string filename = dump_path + "/T_world_utm.txt";
    std::ofstream ofs(filename);

    if (!ofs.is_open()) {
      logger->error("failed to open file for writing: {}", filename);
      return;
    }

    ofs << "# SE(3) Transformation from GNSS/UTM to Odom (World) Frame\n";
    ofs << "# This transformation aligns GNSS coordinates with GLIM's world frame\n";
    ofs << "# Format: 4x4 homogeneous transformation matrix\n";
    ofs << "T_world_utm:\n";

    const Eigen::Matrix4d mat = T_world_utm.matrix();
    ofs << std::fixed << std::setprecision(10);
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        ofs << std::setw(15) << mat(i, j);
        if (j < 3) {
          ofs << " ";
        }
      }
      ofs << "\n";
    }

    logger->info("saved T_world_utm (4x4 SE(3)) to: {}", filename);
  }

  // [P3 FIX 2026-07-09] NaN yaw covariance is KNOWN-BAD (invalid heading
  // solution propagated through the adapter's deg^2->rad^2 conversion), not
  // "unpopulated": map it to +inf so the yaw-quality gate rejects it.
  // 0/negative keep the documented legacy "unpopulated passes" semantics.
  static double sanitize_yaw_var(double c35) {
    if (std::isnan(c35)) return std::numeric_limits<double>::infinity();
    return c35 > 0.0 ? c35 : -1.0;
  }

  void push_gnss_data(double stamp, double x, double y, double z, double qx, double qy, double qz, double qw, double yaw_var = -1.0) {
    // [P2 FIX 2026-07-09] Fail closed on non-finite input. A single NaN
    // position either poisons the one-shot T_world_utm fit (latched true
    // forever) or reaches iSAM2 as a NaN factor and destroys the graph.
    // FusionEngine emits NaN lla/rpy for SolutionType::Invalid (cold start,
    // full outage); the RTK filter gates on covariance, not finiteness.
    if (!std::isfinite(stamp) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      if (!warned_nonfinite_gnss) {
        logger->warn("dropping GNSS sample with non-finite stamp/position (stamp={}, p=[{}, {}, {}]) — further drops silent", stamp, x, y, z);
        warned_nonfinite_gnss = true;
      }
      return;
    }
    GNSSData gnss_data;
    gnss_data.stamp = stamp;
    gnss_data.position << x, y, z;
    gnss_data.yaw_var = yaw_var;

    Eigen::Quaterniond orientation(qw, qx, qy, qz);
    if (orientation.coeffs().allFinite() && orientation.norm() > 1e-6) {
      orientation.normalize();
      gnss_data.orientation = orientation;
      gnss_data.has_orientation = true;
    }

    // Lever-arm compensation: convert reported antenna position to IMU-origin position in UTM.
    //   p_imu_utm = p_antenna_utm - R_world_imu * t_imu_gnss
    // where t_imu_gnss is the URDF translation (antenna origin in IMU body coords).
    // ASSUMPTION: gnss_data.orientation == R_world_imu, i.e., the publisher reports
    // the IMU body's rotation in world (true for INS-fused outputs like Novatel SPAN
    // and Septentrio AsteRx-i, OR when the antenna is axis-aligned with the IMU per
    // the URDF — see the rotation check at startup). If neither holds, the rotated
    // lever arm is wrong and the prior pulls toward a biased position.
    if (t_imu_gnss.squaredNorm() > 0.0) {
      if (gnss_data.has_orientation) {
        gnss_data.position -= gnss_data.orientation * t_imu_gnss;
      } else if (!warned_missing_orientation_for_lever_arm) {
        logger->warn("t_imu_gnss is non-zero but GNSS messages have no orientation; lever arm cannot be compensated");
        warned_missing_orientation_for_lever_arm = true;
      }
    }

    input_gnss_queue.push_back(gnss_data);
  }

  GNSSData interpolate_gnss_data(const GNSSData& left, const GNSSData& right, double stamp) const {
    const double p = (stamp - left.stamp) / (right.stamp - left.stamp);

    GNSSData interpolated;
    interpolated.stamp = stamp;
    interpolated.position = (1.0 - p) * left.position + p * right.position;
    interpolated.has_orientation = left.has_orientation && right.has_orientation;
    if (interpolated.has_orientation) {
      interpolated.orientation = left.orientation.slerp(p, right.orientation).normalized();
    }
    // Yaw variance: conservative max of the bracketing samples. If EITHER is
    // unpopulated (<0), the interpolated variance is unknown too — the gate
    // then passes it (unknown != known-bad), matching the per-sample contract.
    interpolated.yaw_var = (left.yaw_var > 0.0 && right.yaw_var > 0.0)
        ? std::max(left.yaw_var, right.yaw_var) : -1.0;

    return interpolated;
  }

  std::atomic_bool kill_switch;
  // True while the backend is actively associating/factoring a batch of submaps.
  std::atomic_bool processing_{false};
  // True while a submap waiting in the backend's local submap_queue can still be
  // bracketed by available GNSS (i.e. its prior factors are not yet produced).
  // Lets needs_wait() block save() until that submap is factored, without
  // blocking on un-bracketable trailing submaps.
  std::atomic_bool pending_associable_{false};
  // [P2 FIX 2026-07-09] mirrors "local submap_queue non-empty" for
  // needs_wait(): queued GNSS blocks save()/replay only while a submap is
  // actually waiting to be bracketed by it.
  std::atomic_bool submaps_waiting_{false};
  // [P3 FIX 2026-07-10] guards T_world_utm/transformation_initialized between
  // the backend writer and at_exit (main thread) — see at_exit.
  std::mutex T_world_utm_mtx_;
  std::thread thread;

  ConcurrentVector<GNSSData, Eigen::aligned_allocator<GNSSData>> input_gnss_queue;
  ConcurrentVector<QueuedSubmap> input_submap_queue;
  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors;

  std::vector<SubMap::ConstPtr> submaps;
  std::vector<GNSSData, Eigen::aligned_allocator<GNSSData>> submap_coords;
  // Number of associated submaps that have already had GNSS prior factors
  // emitted. Everything in [factored_submap_count, submaps.size()) still needs
  // factors -- this backfills the pre-T_world_utm backlog and every submap in a
  // multi-submap offline batch, not just submaps.back().
  size_t factored_submap_count = 0;

  std::string gnss_topic;
  std::string gnss_msg_type;
  Eigen::Vector3d prior_inf_scale;
  bool enable_orientation_prior;
  Eigen::Vector3d orientation_prior_inf_scale;
  double orientation_prior_max_yaw_sigma_deg;  // P5#1 yaw-quality gate (<=0 disables)
  size_t yaw_gate_skip_count = 0;              // heading priors skipped by the gate
  double min_baseline;
  double max_interp_gap_sec;  // P1 fix: max GNSS bracket width for association (<=0 disables)

  Eigen::Vector3d t_imu_gnss;
  bool warned_missing_orientation_for_lever_arm;
  bool warned_nonfinite_gnss = false;      // P2 fix: non-finite input drop warn-once
  bool warned_nonmonotonic_gnss = false;   // P2 fix: non-monotonic stamp drop warn-once

  bool transformation_initialized;
  Eigen::Isometry3d T_world_utm;
  bool warned_missing_orientation;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GNSSGlobal();
}
