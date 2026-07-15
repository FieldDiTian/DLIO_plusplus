#pragma once

#include <mutex>
#include <thread>
#include <atomic>

#include <glim/util/concurrent_vector.hpp>
#include <glim/odometry/odometry_estimation_base.hpp>

namespace spdlog {
class logger;
}

namespace glim {

/**
 * @brief Odometry estimation executor to wrap and asynchronously run OdometryEstimationBase
 * @note  All the exposed public methods are thread-safe
 *
 */
class AsyncOdometryEstimation {
public:
  /**
   * @brief Construct a new Async Odometry Estimation object
   * @param odometry_estimation  Odometry estimation to be wrapped
   */
  AsyncOdometryEstimation(const std::shared_ptr<OdometryEstimationBase>& odometry_estimation, bool enable_imu);

  /**
   * @brief Destroy the Async Odometry Estimation object
   */
  ~AsyncOdometryEstimation();

#ifdef GLIM_USE_OPENCV
  /**
   * @brief Insert an image into the odometry estimation
   * @param stamp   Timestamp
   * @param image   Image
   */
  void insert_image(const double stamp, const cv::Mat& image);
#endif

  /**
   * @brief Insert an IMU data into the odometry estimation
   * @param stamp         Timestamp
   * @param linear_acc    Linear acceleration
   * @param angular_vel   Angular velocity
   */
  void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel);

  /**
   * @brief Insert a preprocessed point cloud into odometry estimation
   * @param frame  Preprocessed point cloud
   */
  void insert_frame(const PreprocessedFrame::Ptr& frame);

  /**
   * @brief Insert an external pose (forwarded to the wrapped odometry estimator)
   * @param stamp        Timestamp
   * @param T_world_ins  Pose of the INS body frame in the world frame
   */
  void insert_external_pose(const double stamp, const Eigen::Isometry3d& T_world_ins);

  /**
   * @brief Wait for the odometry estimation thread
   */
  void join();

  /**
   * @brief   Get the size of the input queue
   */
  int workload() const;

  // Read after join(); delegated to the wrapped estimator.
  uint64_t ins_coverage_skip_count() const;

  /**
   * @brief Get the estimation results
   * @param estimation_results    Estimation results
   * @param marginalized_frames   Marginalized frames
   */
  void get_results(std::vector<EstimationFrame::ConstPtr>& estimation_results, std::vector<EstimationFrame::ConstPtr>& marginalized_frames);

private:
  void run();

private:
  std::atomic_bool kill_switch;      // Flag to stop the thread immediately (Hard kill switch)
  std::atomic_bool end_of_sequence;  // Flag to stop the thread when the input queues become empty (Soft kill switch)
  std::thread thread;

// Input queues
#ifdef GLIM_USE_OPENCV
  ConcurrentVector<std::pair<double, cv::Mat>> input_image_queue;
#endif
  ConcurrentVector<Eigen::Matrix<double, 7, 1>> input_imu_queue;
  ConcurrentVector<PreprocessedFrame::Ptr> input_frame_queue;
  ConcurrentVector<std::pair<double, Eigen::Isometry3d>> input_external_pose_queue;

  // Output queues
  ConcurrentVector<EstimationFrame::ConstPtr> output_estimation_results;
  ConcurrentVector<EstimationFrame::ConstPtr> output_marginalized_frames;

  bool enable_imu;
  bool enable_external_pose;
  std::atomic_int internal_frame_queue_size;
  std::shared_ptr<OdometryEstimationBase> odometry_estimation;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim
