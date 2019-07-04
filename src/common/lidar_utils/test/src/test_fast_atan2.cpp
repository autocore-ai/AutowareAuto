#include "lidar_utils/lidar_utils.hpp"
#include <gtest/gtest.h>

constexpr float FAST_ATAN2_MAX_ERROR = 0.00469f;

TEST(fast_atan2, corner_cases) {
  ASSERT_TRUE(fabsf(autoware::common::lidar_utils::fast_atan2(0.0f, 0.0f) -
                    atan2f(0.0f, 0.0f)) < FAST_ATAN2_MAX_ERROR);
  ASSERT_TRUE(fabsf(autoware::common::lidar_utils::fast_atan2(1.0f, 0.0f) -
                    atan2f(1.0f, 0.0f)) < FAST_ATAN2_MAX_ERROR);
  ASSERT_TRUE(fabsf(autoware::common::lidar_utils::fast_atan2(-1.0f, 0.0f) -
                    atan2f(-1.0f, 0.0f)) < FAST_ATAN2_MAX_ERROR);
  ASSERT_TRUE(fabsf(autoware::common::lidar_utils::fast_atan2(0.0f, 1.0f) -
                    atan2f(0.0f, 1.0f)) < FAST_ATAN2_MAX_ERROR);
  ASSERT_TRUE(fabsf(autoware::common::lidar_utils::fast_atan2(0.0f, -1.0f) -
                    atan2f(0.0f, -1.0f)) < FAST_ATAN2_MAX_ERROR);
}

TEST(fast_atan2, max_error) {
  float max_error = 0;
  for (float f = 0; f < autoware::common::lidar_utils::TAU; f += 0.00001f) {
    float x = cos(f);
    float y = sin(f);
    max_error = ::std::max(
        max_error,
        fabsf(atan2f(y, x) - autoware::common::lidar_utils::fast_atan2(y, x)));
  }
  ASSERT_TRUE(max_error < FAST_ATAN2_MAX_ERROR);
}