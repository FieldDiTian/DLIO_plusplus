#include <cstring>
#include <memory>

#include <gtest/gtest.h>

#include <glim_ros/lidar_concat.hpp>

namespace {

sensor_msgs::msg::PointCloud2::SharedPtr cloud(
  uint64_t min_ns, uint64_t max_ns, double header_s) {
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp.sec = static_cast<int32_t>(header_s);
  msg->header.stamp.nanosec = static_cast<uint32_t>(
    (header_s - static_cast<double>(msg->header.stamp.sec)) * 1.0e9);
  sensor_msgs::msg::PointField time;
  time.name = "timestamp";
  time.offset = 0;
  time.datatype = sensor_msgs::msg::PointField::UINT8;
  time.count = 8;
  msg->fields = {time};
  msg->height = 1;
  msg->width = 2;
  msg->point_step = 8;
  msg->row_step = 16;
  msg->data.resize(16);
  std::memcpy(msg->data.data(), &min_ns, sizeof(min_ns));
  std::memcpy(msg->data.data() + 8, &max_ns, sizeof(max_ns));
  return msg;
}

TEST(LidarConcatPointTime, DecodesAbsoluteRangeOnceWhenBuffered) {
  const auto buffered = glim_ros::buffer_aux_cloud(
    cloud(1'000'000'000ULL, 1'049'000'000ULL, 1.0));
  ASSERT_TRUE(buffered.luminar_range.valid);
  EXPECT_EQ(buffered.luminar_range.min_ns, 1'000'000'000ULL);
  EXPECT_EQ(buffered.luminar_range.max_ns, 1'049'000'000ULL);
  EXPECT_EQ(buffered.luminar_range.count, 2U);
}

TEST(LidarConcatPointTime, PointTimesOverrideMisleadingHeaderProximity) {
  std::deque<glim_ros::BufferedAuxCloud> candidates;
  candidates.push_back(glim_ros::buffer_aux_cloud(
    cloud(900'000'000ULL, 949'000'000ULL, 0.992)));
  candidates.push_back(glim_ros::buffer_aux_cloud(
    cloud(1'000'000'000ULL, 1'049'000'000ULL, 1.092)));
  const glim_ros::LuminarTimestampRangeNs primary{
    true, 1'000'000'000ULL, 1'049'000'000ULL, 2};

  const auto match = glim_ros::find_closest_luminar_sweep(
    candidates, primary, 0.0, 1.0, 0.0);
  ASSERT_TRUE(match.has_value());
  EXPECT_EQ(match->msg, candidates[1].msg);
  EXPECT_DOUBLE_EQ(match->range_delta_s, 0.0);
  EXPECT_NEAR(match->header_abs_delta_s, 0.092, 1.0e-9);
}

TEST(LidarConcatPointTime, PointClockCorrectionIsSeparateFromHeaderPhase) {
  const glim_ros::LuminarTimestampRangeNs uncorrected{
    true, 1'010'000'000ULL, 1'059'000'000ULL, 2};
  const auto corrected = glim_ros::shifted_range(uncorrected, -0.010);
  EXPECT_EQ(corrected.min_ns, 1'000'000'000ULL);
  EXPECT_EQ(corrected.max_ns, 1'049'000'000ULL);
}

TEST(LidarConcatPointTime, OfflineWatermarkWaitsForFutureSweep) {
  auto primary = cloud(1'000'000'000ULL, 1'049'000'000ULL, 1.0);
  glim_ros::AuxLidarSensor aux;
  aux.buffer_size = 10;
  aux.buffer.push_back(glim_ros::buffer_aux_cloud(
    cloud(900'000'000ULL, 949'000'000ULL, 0.992)));
  std::vector<glim_ros::AuxLidarSensor> sensors{aux};

  EXPECT_FALSE(glim_ros::aux_buffers_ready_for_primary(
    *primary, sensors, 0.010));

  sensors[0].buffer.push_back(glim_ros::buffer_aux_cloud(
    cloud(1'010'000'001ULL, 1'059'000'001ULL, 1.192)));
  EXPECT_TRUE(glim_ros::aux_buffers_ready_for_primary(
    *primary, sensors, 0.010));
}

}  // namespace
