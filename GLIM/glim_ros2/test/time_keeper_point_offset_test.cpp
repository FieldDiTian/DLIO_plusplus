#include <memory>

#include <gtest/gtest.h>

#include <glim/util/config.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/util/time_keeper.hpp>

namespace {

TEST(TimeKeeperPointOffset, ConfiguredOffsetSurvivesAbsoluteStampReplacement) {
  glim::GlobalConfig::instance(GLIM_TEST_CONFIG_DIR, true);

  glim::TimeKeeper time_keeper;
  time_keeper.set_point_time_offset(0.25);

  auto points = std::make_shared<glim::RawPoints>();
  points->stamp = 7.0;
  points->points = {Eigen::Vector4d::Zero(), Eigen::Vector4d::Zero()};
  points->times = {1000.0, 1000.05};

  ASSERT_TRUE(time_keeper.process(points));
  EXPECT_DOUBLE_EQ(points->stamp, 1000.25);
  ASSERT_EQ(points->times.size(), 2U);
  EXPECT_DOUBLE_EQ(points->times.front(), 0.0);
  EXPECT_NEAR(points->times.back(), 0.05, 1.0e-12);
}

}  // namespace
