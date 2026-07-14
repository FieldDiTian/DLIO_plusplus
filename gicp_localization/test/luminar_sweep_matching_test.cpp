#include <gtest/gtest.h>

#include "gicp_localization/luminar_sweep_matching.hpp"

namespace {

using gicp_localization::LuminarSweepCandidate;
using gicp_localization::LuminarTimestampRangeNs;

LuminarTimestampRangeNs range(uint64_t min_ns, uint64_t max_ns) {
  return LuminarTimestampRangeNs{true, min_ns, max_ns, 2};
}

TEST(LuminarSweepMatching, PointTimesOverrideMisleadingHeaderProximity) {
  const auto primary = range(1'000'000'000ULL, 1'049'000'000ULL);
  const std::vector<LuminarSweepCandidate> candidates{
      {0, range(900'000'000ULL, 949'000'000ULL), 0.008},
      {1, range(1'000'000'000ULL, 1'049'000'000ULL), 0.092}};

  const auto selected =
      gicp_localization::selectClosestLuminarSweep(primary, candidates);

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->index, 1U);
  EXPECT_DOUBLE_EQ(selected->range_delta_s, 0.0);
  EXPECT_DOUBLE_EQ(selected->header_abs_delta_s, 0.092);
}

TEST(LuminarSweepMatching, ClockCorrectionAppliesToAbsolutePointRange) {
  const auto uncorrected = range(1'010'000'000ULL, 1'059'000'000ULL);
  const auto corrected = gicp_localization::shiftedRange(uncorrected, -0.010);

  EXPECT_EQ(corrected.min_ns, 1'000'000'000ULL);
  EXPECT_EQ(corrected.max_ns, 1'049'000'000ULL);
}

TEST(LuminarSweepMatching, WatermarkWaitsForFutureAlignedSweep) {
  const auto primary = range(1'000'000'000ULL, 1'049'000'000ULL);
  const auto previous = range(900'000'000ULL, 949'000'000ULL);
  const auto definitely_later = range(1'010'000'001ULL, 1'059'000'001ULL);

  EXPECT_FALSE(gicp_localization::luminarWatermarkPassed(
      primary, previous, 0.010));
  EXPECT_TRUE(gicp_localization::luminarWatermarkPassed(
      primary, definitely_later, 0.010));
}

}  // namespace
