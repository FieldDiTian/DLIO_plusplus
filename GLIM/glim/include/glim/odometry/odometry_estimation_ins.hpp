#pragma once

#include <deque>
#include <memory>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/odometry/odometry_estimation_base.hpp>

namespace gtsam_points {
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;
}  // namespace gtsam_points

namespace glim {

class CloudDeskewing;
class CloudCovarianceEstimation;

/**
 * @brief Parameters for OdometryEstimationINS
 */
struct OdometryEstimationINSParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationINSParams();
  ~OdometryEstimationINSParams();

  // Frames
  Eigen::Isometry3d T_lidar_imu;
  Eigen::Isometry3d T_imu_ins;

  // Refinement
  bool enable_lidar_refinement;
  double max_refinement_translation;
  double max_refinement_rotation;
  int refinement_max_iterations;
  double ins_prior_sigma_trans;
  double ins_prior_sigma_rot;
  double target_downsampling_rate;

  // iVox
  double ivox_resolution;
  double ivox_min_dist;
  int ivox_lru_thresh;

  // Sync
  double max_ins_wait_seconds;
  double ins_buffer_seconds;

  // Misc
  bool save_imu_rate_trajectory;
  int num_threads;
};

/**
 * @brief Odometry estimator driven by an external INS / GNSS-aided pose source,
 *        with an optional single-step LiDAR refinement against a rolling iVox target.
 *
 * Each call to insert_frame:
 *   1) interpolates an INS pose at the frame timestamp
 *   2) deskews the cloud using INS-interpolated poses across the scan window
 *   3) optionally refines via single-step GICP (rejected if delta exceeds thresholds)
 *   4) emits a fully-populated EstimationFrame and marginalizes immediately
 */
class OdometryEstimationINS : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationINS(const OdometryEstimationINSParams& params = OdometryEstimationINSParams());
  ~OdometryEstimationINS() override;

  bool requires_imu() const override { return true; }
  bool requires_external_pose() const override { return true; }
  uint64_t ins_coverage_skip_count() const override { return ins_coverage_skips; }

  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  void insert_external_pose(const double stamp, const Eigen::Isometry3d& T_world_ins) override;
  EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_states) override;

private:
  bool wait_for_ins_coverage(double t_start, double t_end);
  bool interpolate_ins(double stamp, Eigen::Isometry3d& T_world_ins) const;
  void sample_ins_trajectory(double t_start, double t_end, double step, std::vector<double>& out_times, std::vector<Eigen::Isometry3d>& out_poses) const;

private:
  OdometryEstimationINSParams params;

  std::deque<std::pair<double, Eigen::Isometry3d>> ins_buffer;
  std::deque<std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>> imu_buffer;

  long frame_id;
  uint64_t ins_coverage_skips = 0;
  EstimationFrame::ConstPtr last_frame;

  std::shared_ptr<gtsam_points::iVox> target_ivox;
  std::unique_ptr<CloudDeskewing> deskewer;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  std::mt19937 mt;
};

}  // namespace glim
