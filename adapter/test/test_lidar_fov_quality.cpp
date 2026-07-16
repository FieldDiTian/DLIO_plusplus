#include <cmath>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_field.hpp>

#include <adapter/lidar_fov_quality.hpp>

namespace {

sensor_msgs::msg::PointCloud2 make_elevation_cloud(const std::vector<float>& angles_deg) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(angles_deg.size());
  cloud.is_bigendian = false;
  cloud.point_step = sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  sensor_msgs::msg::PointField elevation;
  elevation.name = "elevation";
  elevation.offset = 0;
  elevation.datatype = sensor_msgs::msg::PointField::FLOAT32;
  elevation.count = 1;
  cloud.fields.push_back(elevation);
  cloud.data.resize(cloud.row_step);
  for (std::size_t i = 0; i < angles_deg.size(); ++i) {
    const float radians = angles_deg[i] * static_cast<float>(M_PI / 180.0);
    std::memcpy(cloud.data.data() + i * cloud.point_step, &radians, sizeof(radians));
  }
  return cloud;
}

TEST(LidarFovQuality, AcceptsAtLeastThirtyDegrees) {
  const auto cloud = make_elevation_cloud({-15.0F, 0.0F, 15.0F});
  const auto result = adapter::measure_vertical_fov(cloud, "elevation", true, 3, true);
  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(result.vertical_fov_deg, 30.0, 1e-4);
  EXPECT_TRUE(adapter::meets_vertical_fov(result, 30.0));
}

TEST(LidarFovQuality, RejectsBelowThirtyDegrees) {
  const auto cloud = make_elevation_cloud({-16.6F, 0.0F, 10.8F});
  const auto result = adapter::measure_vertical_fov(cloud, "elevation", true, 3, true);
  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_NEAR(result.vertical_fov_deg, 27.4, 1e-4);
  EXPECT_FALSE(adapter::meets_vertical_fov(result, 30.0));
}

TEST(LidarFovQuality, FailsClosedWhenElevationFieldIsMissing) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = 1000;
  cloud.point_step = 4;
  cloud.row_step = 4000;
  cloud.data.resize(4000);
  const auto result = adapter::measure_vertical_fov(cloud, "elevation", true, 1000, true);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error.find("elevation"), std::string::npos);
}

TEST(LidarFovQuality, FailsClosedWhenElevationFieldExceedsPointStride) {
  auto cloud = make_elevation_cloud({-15.0F, 0.0F, 15.0F});
  cloud.fields.front().offset = cloud.point_step;
  const auto result = adapter::measure_vertical_fov(cloud, "elevation", true, 3, true);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.error.find("point_step"), std::string::npos);
}

}  // namespace
