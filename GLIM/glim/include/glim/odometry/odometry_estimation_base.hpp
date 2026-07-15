#pragma once

#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef GLIM_USE_OPENCV
#include <opencv2/core.hpp>
#endif
#include <glim/odometry/estimation_frame.hpp>
#include <glim/preprocess/preprocessed_frame.hpp>

namespace spdlog {
class logger;
}

namespace glim {

/**
 * @brief Odometry estimation base class
 *
 */
class OdometryEstimationBase {
public:
  OdometryEstimationBase();
  virtual ~OdometryEstimationBase() {}

  /**
   * @brief Returns true if the odometry estimation module requires IMU data
   */
  virtual bool requires_imu() const { return true; }

  /**
   * @brief Returns true if the odometry estimation module requires external pose data
   *        (e.g. from an INS / GNSS-aided IMU). When true, AsyncOdometryEstimation gates
   *        frames on the latest external pose timestamp, just like it does for IMU.
   */
  virtual bool requires_external_pose() const { return false; }

  // Number of frames rejected because the configured INS/external-pose buffer
  // did not cover their scan window. Non-INS estimators return zero.
  virtual uint64_t ins_coverage_skip_count() const { return 0; }

#ifdef GLIM_USE_OPENCV
  /**
   * @brief Insert an image
   * @param stamp   Timestamp
   * @param image   Image
   */
  virtual void insert_image(const double stamp, const cv::Mat& image);
#endif

  /**
   * @brief Insert an IMU data
   * @param stamp        Timestamp
   * @param linear_acc   Linear acceleration
   * @param angular_vel  Angular velocity
   */
  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  /**
   * @brief Insert an external pose (e.g. from an INS / GNSS-aided IMU). Default no-op.
   * @param stamp         Timestamp
   * @param T_world_ins   Pose of the INS body frame in the world frame
   */
  virtual void insert_external_pose(const double stamp, const Eigen::Isometry3d& T_world_ins) {}

  /**
   * @brief Insert a point cloud
   * @param frame                       Preprocessed point cloud
   * @param marginalized_states         [out] Marginalized estimation frames
   * @return EstimationFrame::ConstPtr  Estimation result for the latest frame
   */
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_states);

  /**
   * @brief Pop out the remaining non-marginalized frames (called at the end of the sequence)
   * @return std::vector<EstimationFrame::ConstPtr>
   */
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() { return std::vector<EstimationFrame::ConstPtr>(); }

  /**
   * @brief Load an odometry estimation module from a dynamic library
   * @param so_name  Dynamic library name
   * @return         Loaded odometry estimation module
   */
  static std::shared_ptr<OdometryEstimationBase> load_module(const std::string& so_name);

protected:
  // Logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim
