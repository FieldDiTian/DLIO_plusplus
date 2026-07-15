#include <glim/odometry/odometry_estimation_ins.hpp>

#include <chrono>
#include <thread>
#include <stdexcept>

#include <spdlog/spdlog.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/util/config.hpp>
#include <glim/util/urdf_transforms.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/odometry/callbacks.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;
using Callbacks = OdometryEstimationCallbacks;

namespace {

Eigen::Isometry3d interpolate_pose(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b, double t) {
  Eigen::Quaterniond qa(a.linear());
  Eigen::Quaterniond qb(b.linear());
  Eigen::Quaterniond q = qa.slerp(t, qb).normalized();
  Eigen::Vector3d p = (1.0 - t) * a.translation() + t * b.translation();
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.linear() = q.toRotationMatrix();
  out.translation() = p;
  return out;
}

Eigen::Isometry3d resolve_T_imu_ins(const Config& config_sensors, const Config& config_odom) {
  const std::string urdf_path_cfg = config_odom.param<std::string>("odometry_estimation", "urdf_path", "");
  const std::string urdf_path = urdf_path_cfg.empty() ? config_sensors.param<std::string>("sensors", "urdf_path", "") : urdf_path_cfg;
  const std::string urdf_imu_frame = config_sensors.param<std::string>("sensors", "urdf_imu_frame", "");
  std::string urdf_ins_frame = config_odom.param<std::string>("odometry_estimation", "urdf_ins_frame", "");
  if (urdf_ins_frame.empty()) {
    urdf_ins_frame = config_sensors.param<std::string>("sensors", "urdf_ins_frame", "");
  }

  if (urdf_path.empty() || urdf_imu_frame.empty() || urdf_ins_frame.empty()) {
    spdlog::warn("OdometryEstimationINS: urdf_path / urdf_imu_frame / urdf_ins_frame not configured; using T_imu_ins = identity");
    return Eigen::Isometry3d::Identity();
  }

  try {
    const auto transforms = parse_urdf_transforms(urdf_path);
    const Eigen::Isometry3d T_imu_ins = compute_transform(transforms, urdf_imu_frame, urdf_ins_frame);
    std::stringstream ss;
    ss << T_imu_ins.matrix();
    spdlog::info("OdometryEstimationINS: T_imu_ins from URDF ({} -> {}):\n{}", urdf_imu_frame, urdf_ins_frame, ss.str());
    return T_imu_ins;
  } catch (const std::exception& e) {
    spdlog::error("OdometryEstimationINS: failed to compute T_imu_ins from URDF: {}", e.what());
    return Eigen::Isometry3d::Identity();
  }
}

}  // namespace

OdometryEstimationINSParams::OdometryEstimationINSParams() {
  Config config_sensors(GlobalConfig::get_config_path("config_sensors"));
  Config config(GlobalConfig::get_config_path("config_odometry"));

  T_lidar_imu = config_sensors.param<Eigen::Isometry3d>("sensors", "T_lidar_imu", Eigen::Isometry3d::Identity());
  T_imu_ins = resolve_T_imu_ins(config_sensors, config);

  enable_lidar_refinement = config.param<bool>("odometry_estimation", "enable_lidar_refinement", true);
  max_refinement_translation = config.param<double>("odometry_estimation", "max_refinement_translation", 0.30);
  max_refinement_rotation = config.param<double>("odometry_estimation", "max_refinement_rotation", 0.05);
  refinement_max_iterations = config.param<int>("odometry_estimation", "refinement_max_iterations", 5);
  ins_prior_sigma_trans = config.param<double>("odometry_estimation", "ins_prior_sigma_trans", 0.05);
  ins_prior_sigma_rot = config.param<double>("odometry_estimation", "ins_prior_sigma_rot", 0.0087);
  target_downsampling_rate = config.param<double>("odometry_estimation", "target_downsampling_rate", 0.1);

  ivox_resolution = config.param<double>("odometry_estimation", "ivox_resolution", 0.5);
  ivox_min_dist = config.param<double>("odometry_estimation", "ivox_min_dist", 0.1);
  ivox_lru_thresh = config.param<int>("odometry_estimation", "ivox_lru_thresh", 200);

  max_ins_wait_seconds = config.param<double>("odometry_estimation", "max_ins_wait_seconds", 0.05);
  ins_buffer_seconds = config.param<double>("odometry_estimation", "ins_buffer_seconds", 2.0);

  save_imu_rate_trajectory = config.param<bool>("odometry_estimation", "save_imu_rate_trajectory", true);
  num_threads = config.param<int>("odometry_estimation", "num_threads", 2);
}

OdometryEstimationINSParams::~OdometryEstimationINSParams() = default;

OdometryEstimationINS::OdometryEstimationINS(const OdometryEstimationINSParams& params)
: params(params), frame_id(0), mt(12345) {
  deskewer = std::make_unique<CloudDeskewing>();
  covariance_estimation = std::make_unique<CloudCovarianceEstimation>(this->params.num_threads);

  if (this->params.enable_lidar_refinement) {
    target_ivox = std::make_shared<gtsam_points::iVox>(this->params.ivox_resolution);
    target_ivox->voxel_insertion_setting().set_min_dist_in_cell(this->params.ivox_min_dist);
    target_ivox->set_lru_horizon(this->params.ivox_lru_thresh);
    target_ivox->set_neighbor_voxel_mode(1);
  }
}

OdometryEstimationINS::~OdometryEstimationINS() = default;

void OdometryEstimationINS::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);

  imu_buffer.emplace_back(stamp, linear_acc, angular_vel);
  while (!imu_buffer.empty() && std::get<0>(imu_buffer.front()) < stamp - params.ins_buffer_seconds) {
    imu_buffer.pop_front();
  }
}

void OdometryEstimationINS::insert_external_pose(const double stamp, const Eigen::Isometry3d& T_world_ins) {
  if (!ins_buffer.empty() && stamp <= ins_buffer.back().first) {
    spdlog::trace("OdometryEstimationINS: dropping out-of-order INS sample (stamp={}, last={})", stamp, ins_buffer.back().first);
    return;
  }
  ins_buffer.emplace_back(stamp, T_world_ins);
  while (!ins_buffer.empty() && ins_buffer.front().first < stamp - params.ins_buffer_seconds) {
    ins_buffer.pop_front();
  }
}

bool OdometryEstimationINS::wait_for_ins_coverage(double t_start, double t_end) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(params.max_ins_wait_seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (!ins_buffer.empty() && ins_buffer.front().first <= t_start && ins_buffer.back().first >= t_end) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return !ins_buffer.empty() && ins_buffer.front().first <= t_start && ins_buffer.back().first >= t_end;
}

bool OdometryEstimationINS::interpolate_ins(double stamp, Eigen::Isometry3d& T_world_ins) const {
  if (ins_buffer.empty()) return false;
  if (stamp <= ins_buffer.front().first) {
    T_world_ins = ins_buffer.front().second;
    return true;
  }
  if (stamp >= ins_buffer.back().first) {
    T_world_ins = ins_buffer.back().second;
    return true;
  }
  for (size_t i = 1; i < ins_buffer.size(); i++) {
    if (ins_buffer[i].first >= stamp) {
      const auto& a = ins_buffer[i - 1];
      const auto& b = ins_buffer[i];
      const double dt = b.first - a.first;
      const double t = dt > 1e-9 ? (stamp - a.first) / dt : 0.0;
      T_world_ins = interpolate_pose(a.second, b.second, t);
      return true;
    }
  }
  return false;
}

void OdometryEstimationINS::sample_ins_trajectory(
  double t_start,
  double t_end,
  double step,
  std::vector<double>& out_times,
  std::vector<Eigen::Isometry3d>& out_poses) const {
  out_times.clear();
  out_poses.clear();
  if (t_end <= t_start) {
    Eigen::Isometry3d T;
    if (interpolate_ins(t_start, T)) {
      out_times.push_back(t_start);
      out_poses.push_back(T);
    }
    return;
  }
  const int n = std::max(2, static_cast<int>(std::ceil((t_end - t_start) / std::max(step, 1e-3))) + 1);
  out_times.reserve(n);
  out_poses.reserve(n);
  for (int i = 0; i < n; i++) {
    const double t = t_start + (t_end - t_start) * static_cast<double>(i) / (n - 1);
    Eigen::Isometry3d T;
    if (!interpolate_ins(t, T)) continue;
    out_times.push_back(t);
    out_poses.push_back(T);
  }
}

EstimationFrame::ConstPtr OdometryEstimationINS::insert_frame(const PreprocessedFrame::Ptr& raw_frame, std::vector<EstimationFrame::ConstPtr>& marginalized_states) {
  Callbacks::on_insert_frame(raw_frame);

  const double t_start = raw_frame->stamp;
  const double t_end = std::max(raw_frame->scan_end_time, raw_frame->stamp);

  if (!wait_for_ins_coverage(t_start, t_end)) {
    ++ins_coverage_skips;
    spdlog::warn("OdometryEstimationINS: INS data does not cover frame [{:.6f}, {:.6f}] (buffer: {} samples); skipping", t_start, t_end, ins_buffer.size());
    return nullptr;
  }

  const Eigen::Isometry3d T_ins_imu = params.T_imu_ins.inverse();
  const Eigen::Isometry3d T_lidar_imu = params.T_lidar_imu;
  const Eigen::Isometry3d T_imu_lidar = T_lidar_imu.inverse();

  // Sample INS over the scan window for deskewing. For instantaneous frames (t_end == t_start)
  // there is no motion to compensate — fall back to a single pose and skip deskewing.
  std::vector<double> traj_times;
  std::vector<Eigen::Isometry3d> traj_imu_poses;
  std::vector<Eigen::Isometry3d> traj_ins_poses;
  const bool has_scan_window = (t_end - t_start) > 1e-4;
  if (has_scan_window) {
    sample_ins_trajectory(t_start, t_end, 0.01, traj_times, traj_ins_poses);
  }
  if (traj_times.size() < 2) {
    Eigen::Isometry3d T_world_ins_single;
    if (!interpolate_ins(t_start, T_world_ins_single)) {
      ++ins_coverage_skips;
      spdlog::warn("OdometryEstimationINS: failed to interpolate INS at frame stamp {:.6f}; skipping", t_start);
      return nullptr;
    }
    traj_times = {t_start, t_end > t_start ? t_end : t_start + 1e-3};
    traj_ins_poses = {T_world_ins_single, T_world_ins_single};
  }

  traj_imu_poses.reserve(traj_ins_poses.size());
  for (const auto& T_world_ins : traj_ins_poses) {
    traj_imu_poses.push_back(T_world_ins * T_ins_imu);
  }

  Eigen::Isometry3d T_world_imu_init = traj_imu_poses.front();

  // Deskew points into the IMU frame at scan-start. With a degenerate trajectory (two equal
  // poses) the deskew is a no-op, which is the right behavior for instantaneous frames.
  std::vector<Eigen::Vector4d> deskewed;
  if (has_scan_window) {
    deskewed = deskewer->deskew(T_imu_lidar, traj_times, traj_imu_poses, raw_frame->stamp, raw_frame->times, raw_frame->points);
  } else {
    deskewed = raw_frame->points;
  }
  for (auto& pt : deskewed) {
    pt = T_imu_lidar * pt;
  }

  std::vector<Eigen::Vector4d> deskewed_normals;
  std::vector<Eigen::Matrix4d> deskewed_covs;
  covariance_estimation->estimate(deskewed, raw_frame->neighbors, deskewed_normals, deskewed_covs);

  auto pc = std::make_shared<gtsam_points::PointCloudCPU>(deskewed);
  if (!raw_frame->intensities.empty()) {
    pc->add_intensities(raw_frame->intensities);
  }
  pc->add_covs(deskewed_covs);
  pc->add_normals(deskewed_normals);

  // Optional single-step GICP refinement.
  Eigen::Isometry3d T_world_imu = T_world_imu_init;
  bool refinement_accepted = false;
  if (params.enable_lidar_refinement && target_ivox && frame_id >= 3) {
    gtsam::Values values;
    values.insert(X(0), gtsam::Pose3(T_world_imu_init.matrix()));

    gtsam::NonlinearFactorGraph graph;

    auto gicp = gtsam::make_shared<gtsam_points::IntegratedGICPFactor_<gtsam_points::iVox, gtsam_points::PointCloud>>(
      gtsam::Pose3(),
      X(0),
      target_ivox,
      pc,
      target_ivox);
    gicp->set_max_correspondence_distance(params.ivox_resolution * 2.0);
    gicp->set_num_threads(params.num_threads);
    graph.add(gicp);

    Eigen::VectorXd prior_sigmas(6);
    prior_sigmas << params.ins_prior_sigma_rot, params.ins_prior_sigma_rot, params.ins_prior_sigma_rot,
      params.ins_prior_sigma_trans, params.ins_prior_sigma_trans, params.ins_prior_sigma_trans;
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3(T_world_imu_init.matrix()), prior_noise);

    gtsam_points::LevenbergMarquardtExtParams lm_params;
    lm_params.setMaxIterations(params.refinement_max_iterations);
    lm_params.setAbsoluteErrorTol(0.1);
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
    const gtsam::Values result = optimizer.optimize();

    const Eigen::Isometry3d T_world_imu_refined(result.at<gtsam::Pose3>(X(0)).matrix());
    const Eigen::Isometry3d delta = T_world_imu_init.inverse() * T_world_imu_refined;
    const double delta_t = delta.translation().norm();
    const double delta_r = Eigen::AngleAxisd(delta.linear()).angle();

    if (delta_t < params.max_refinement_translation && delta_r < params.max_refinement_rotation) {
      T_world_imu = T_world_imu_refined;
      refinement_accepted = true;
      spdlog::trace("OdometryEstimationINS: refinement accepted (dt={:.4f}m dr={:.4f}rad)", delta_t, delta_r);
    } else {
      spdlog::warn("OdometryEstimationINS: refinement rejected (dt={:.4f}m dr={:.4f}rad), using INS pose", delta_t, delta_r);
    }
  }

  // Update target with the chosen pose.
  if (target_ivox) {
    auto downsampled = pc;
    if (frame_id >= 5 && params.target_downsampling_rate < 0.999) {
      downsampled = gtsam_points::random_sampling(pc, params.target_downsampling_rate, mt);
    }
    auto transformed = gtsam_points::transform(downsampled, T_world_imu);
    target_ivox->insert(*transformed);
  }

  // Build output EstimationFrame.
  EstimationFrame::Ptr new_frame(new EstimationFrame);
  new_frame->id = frame_id++;
  new_frame->stamp = raw_frame->stamp;
  new_frame->T_lidar_imu = T_lidar_imu;
  new_frame->T_world_imu = T_world_imu;
  new_frame->T_world_lidar = T_world_imu * T_imu_lidar;

  if (last_frame) {
    const double dt = new_frame->stamp - last_frame->stamp;
    if (dt > 1e-3) {
      new_frame->v_world_imu = (T_world_imu.translation() - last_frame->T_world_imu.translation()) / dt;
    } else {
      new_frame->v_world_imu.setZero();
    }
  } else {
    new_frame->v_world_imu.setZero();
  }
  new_frame->imu_bias.setZero();
  new_frame->raw_frame = raw_frame;
  new_frame->frame_id = FrameID::IMU;
  new_frame->frame = pc;

  if (params.save_imu_rate_trajectory) {
    new_frame->imu_rate_trajectory.resize(8, traj_times.size());
    for (size_t i = 0; i < traj_times.size(); i++) {
      const Eigen::Vector3d trans = traj_imu_poses[i].translation();
      const Eigen::Quaterniond quat(traj_imu_poses[i].linear());
      new_frame->imu_rate_trajectory.col(i) << traj_times[i], trans, quat.x(), quat.y(), quat.z(), quat.w();
    }
  }

  Callbacks::on_new_frame(new_frame);

  // Marginalize immediately — no fixed-lag smoother.
  marginalized_states.push_back(new_frame);
  std::vector<EstimationFrame::ConstPtr> marg_singleton{new_frame};
  Callbacks::on_marginalized_frames(marg_singleton);
  Callbacks::on_update_new_frame(new_frame);

  std::vector<EstimationFrame::ConstPtr> active_view{new_frame};
  Callbacks::on_update_frames(active_view);

  last_frame = new_frame;
  (void)refinement_accepted;
  return new_frame;
}

}  // namespace glim
