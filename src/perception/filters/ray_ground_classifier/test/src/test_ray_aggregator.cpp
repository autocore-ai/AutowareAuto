// Copyright 2017-2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <ray_ground_classifier/ray_aggregator.hpp>
#include <ray_ground_classifier/ray_ground_point_classifier.hpp>
#include <common/types.hpp>

namespace
{

using autoware::common::types::PointXYZIF;
using autoware::common::types::float32_t;
using autoware::perception::filters::ray_ground_classifier::PointBlock;
using autoware::perception::filters::ray_ground_classifier::PointXYZIFR;
using autoware::perception::filters::ray_ground_classifier::Ray;
using autoware::perception::filters::ray_ground_classifier::RayAggregator;

void check_ray(const Ray & ray, const float32_t th)
{
  float32_t last_x = -1.0F;
  float32_t last_y = -1.0F;
  float32_t last_r = -1.0F;
  for (const auto & pt : ray) {
    const float32_t x = pt.get_point_pointer()->x;
    const float32_t y = pt.get_point_pointer()->y;
    // (0, 0) is a special case
    if ((fabsf(x) > 0.0001F) || (fabsf(y) > 0.0001F)) {
      EXPECT_LT(autoware::perception::filters::ray_ground_classifier::angle_distance_rad(th,
        atan2f(y, x)), 0.1F) <<
        x << ", " << y;
    }
    const float32_t r = (x * x) + (y * y);
    EXPECT_GE(fabsf(x), last_x);
    EXPECT_GE(fabsf(y), last_y);
    EXPECT_GE(r, last_r);
    last_x = fabsf(x);
    last_y = fabsf(y);
    last_r = r;
  }
}

// basic usage, stateful behavior
TEST(ray_aggregator, basic) {
  const std::size_t min_ray_points = 10U;
  RayAggregator::Config cfg{-3.14159F, 3.14159F, 0.1F, min_ray_points};
  RayAggregator agg{cfg};
  EXPECT_EQ(min_ray_points, cfg.get_min_ray_points());
  // Do this twice, expect same result to exercise internal reset logic
  for (uint32_t jdx = 0U; jdx < 3U; ++jdx) {
    // insert points along one ray
    for (uint32_t idx = 0U; idx < min_ray_points - 1U; ++idx) {
      PointXYZIF pt;
      pt.x = static_cast<float32_t>(idx % 3U) + (0.1F * static_cast<float32_t>(idx)) + 0.1F;
      pt.y = static_cast<float32_t>(idx % 3U) + (0.1F * static_cast<float32_t>(idx)) + 0.1F;
      agg.insert(pt);
      EXPECT_FALSE(agg.is_ray_ready()) << idx;
    }
    // insert one more
    PointXYZIF pt;
    pt.x = static_cast<float32_t>(min_ray_points - 1U);
    pt.y = static_cast<float32_t>(min_ray_points - 1U);
    agg.insert(pt);
    // ready
    EXPECT_TRUE(agg.is_ray_ready());
    // ray should be in sorted order
    const auto & ray = agg.get_next_ray();
    check_ray(ray, atan2f(1.0F, 1.0F));
    EXPECT_EQ(ray.size(), min_ray_points);
    EXPECT_FALSE(agg.is_ray_ready());
  }
  EXPECT_THROW(agg.get_next_ray(), std::runtime_error);
}

// excluded bounds rays, multiple rays, end of scan id
TEST(ray_aggregator, multi_flipped)
{
  const std::size_t min_ray_points = 50U;
  RayAggregator::Config cfg{1.14159F, -1.14159F, 0.2F, min_ray_points};
  RayAggregator agg{cfg};
  PointBlock blk;
  const std::size_t num_points = 32U;
  // fill out a couple of rays in the point block
  for (uint32_t idx = 0U; idx < num_points; ++idx) {
    PointXYZIF pt;
    // .. generate points ..
    float32_t u, v;
    switch (idx % 3U) {
      case 0U:
        u = -1.0F;
        v = 1.0F;
        break;
      case 1U:
        u = -1.0F;
        v = -1.0F;
        break;
      case 2U:
        u = -1.0F;
        v = 0.0F;
        break;
    }
    pt.x = u * (static_cast<float32_t>(idx % 3U) + (0.1F * static_cast<float32_t>(idx)) + 0.1F);
    pt.y = v * (static_cast<float32_t>(idx % 3U) + (0.1F * static_cast<float32_t>(idx)) + 0.1F);
    blk.push_back(pt);
  }
  agg.insert(blk);
  EXPECT_FALSE(agg.is_ray_ready());
  // End of scan
  blk.resize(1U);
  blk[0U].id = static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID);
  agg.insert(blk);
  EXPECT_TRUE(agg.is_ray_ready());
  // .. check rays ..
  std::size_t total_points = 0U;
  // ray 1
  const auto & ray1 = agg.get_next_ray();
  total_points += ray1.size();
  EXPECT_EQ(ray1.size(), 11U);
  check_ray(ray1, atan2f(1.0F, -1.0F));
  // ray 2
  const auto & ray2 = agg.get_next_ray();
  total_points += ray2.size();
  EXPECT_EQ(ray2.size(), 10U);
  check_ray(ray2, atan2f(0.0F, -1.0F));
  // ray 3
  const auto & ray3 = agg.get_next_ray();
  total_points += ray3.size();
  EXPECT_EQ(ray3.size(), 11U);
  check_ray(ray3, atan2f(-1.0F, -1.0F));
  EXPECT_FALSE(agg.is_ray_ready());
  EXPECT_EQ(total_points, num_points);
}

// test various forms of templated insert, multiple times, (0, y)
TEST(ray_aggregator, multi_insert)
{
  std::array<PointXYZIF, 3U> pts;
  pts[0U].x = 0.0F; pts[0U].y = 1.0F;
  pts[1U].x = 0.0F; pts[1U].y = 5.0F;
  pts[2U].x = 0.0F; pts[2U].y = 2.0F;
  std::array<PointXYZIFR, 3U> pts_r{PointXYZIFR{pts[0U]}, PointXYZIFR{pts[1U]},
    PointXYZIFR{pts[2U]}};
  std::array<PointBlock, 2U> blks;
  blks[0U].resize(2U);
  blks[0U][0U].x = 0.0F; blks[0U][0U].y = -1.0F;
  blks[0U][1U].x = 0.0F; blks[0U][1U].y = -2.0F;
  blks[1U].resize(2U);
  blks[1U][0U].x = 1.0F; blks[1U][0U].y = 0.0F;
  blks[1U][1U].x = 2.0F; blks[1U][1U].y = 0.0F;
  const std::size_t min_ray_points = 4U;
  RayAggregator::Config cfg{-3.14159F, 3.14159F, 0.2F, min_ray_points};
  RayAggregator agg{cfg};
  // const insertion
  agg.insert(pts.cbegin(), pts.cend());
  EXPECT_FALSE(agg.is_ray_ready());
  agg.insert(blks.cbegin(), blks.cend());
  EXPECT_FALSE(agg.is_ray_ready());
  agg.insert(pts_r.cbegin(), pts_r.cend());
  EXPECT_TRUE(agg.is_ray_ready());
  // nonconst
  agg.insert(pts.begin(), pts.end());
  EXPECT_TRUE(agg.is_ray_ready());
  agg.insert(pts_r.begin(), pts_r.end());
  EXPECT_TRUE(agg.is_ray_ready());
  agg.insert(blks.begin(), blks.end());
  EXPECT_TRUE(agg.is_ray_ready());
  // check result: (0, +y) was init'd first
  std::size_t total_points = 0U;
  const auto & ray1 = agg.get_next_ray();
  total_points += ray1.size();
  EXPECT_EQ(ray1.size(), 12U);
  check_ray(ray1, atan2f(-1.0F, 0.0F));
  const auto & ray2 = agg.get_next_ray();
  total_points += ray2.size();
  EXPECT_EQ(ray2.size(), 4U);
  check_ray(ray2, atan2f(-1.0F, 0.0F));
  const auto & ray3 = agg.get_next_ray();
  total_points += ray3.size();
  EXPECT_EQ(ray3.size(), 4U);
  check_ray(ray3, atan2f(0.0F, 1.0F));
  EXPECT_EQ(total_points, 2 * (3 + 3 + 2 + 2));
}

// bad cases: fill out MC/DC for error handling
TEST(ray_aggregator, bad_cases)
{
  const uint32_t capacity =
    static_cast<uint32_t>(autoware::common::types::POINT_BLOCK_CAPACITY);
  // small width
  EXPECT_THROW(RayAggregator::Config cfg({-3.14159F, 3.14159F, 0.00000000001F, 10U}),
    std::runtime_error);
  // too big ray limit
  EXPECT_THROW(RayAggregator::Config cfg({-3.14159F, 3.14159F, 0.1F, capacity + 1U}),
    std::runtime_error);
  // insert past capacity
  RayAggregator::Config cfg{-3.14159F, 3.14159F, 0.2F, 10U};
  RayAggregator agg{cfg};
  for (uint32_t idx = 0U; idx < capacity; ++idx) {
    PointXYZIF pt;
    pt.x = 0.1F + static_cast<float32_t>(idx);
    pt.y = 0.1F + static_cast<float32_t>(idx);
    agg.insert(pt);
  }
  PointXYZIF pt2;
  pt2.x = 1.5;
  pt2.y = 1.5;
  EXPECT_THROW(agg.insert(pt2), std::runtime_error);
}

TEST(ray_aggregator, segfault)
{
  RayAggregator::Config cfg{-3.14159F, 3.14159F, 0.005F, 512};
  RayAggregator agg{cfg};
  // Ensure the segfault conditions are reproduced.
  //
  // {m_point = {x = -16.376606, y = -3.12589109e-05, z = 6.11320305, intensity = 13, id = 0, static END_OF_SCAN_ID = 65535}, m_r_xy = 16.376606}
  // $4 = {m_min_ray_points = 512, m_num_rays = 1257, m_ray_width_rad = 0.00499999989, m_min_angle_rad = -3.14159012, m_domain_crosses_180 = false}

  EXPECT_EQ(cfg.get_num_rays(), 1257);
  EXPECT_EQ(cfg.get_min_ray_points(), 512);
  EXPECT_FLOAT_EQ(cfg.get_ray_width(), 0.00499999989);
  EXPECT_FLOAT_EQ(cfg.get_min_angle(), -3.14159012);
  EXPECT_FALSE(cfg.domain_crosses_180());

  // Reproduce point
  PointXYZIF pt;
  pt.x = -16.376606;
  pt.y = -3.12589109E-05;
  pt.z = 6.1132035;
  pt.intensity = 13;
  pt.id = 0;

  EXPECT_NO_THROW(agg.insert(pt));
}

// https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/issues/217
// Make sure rays with "NOT_READY" state does not get included as ready rays when we reach end of
// scan.
TEST(ray_aggregator, reset_bug) {
  RayAggregator::Config cfg{-3.14159F, 3.14159F, 0.005F, 10};
  RayAggregator agg{cfg};

  const size_t kNumPoints = 3;
  // Call aggregator with the given points and check that there is only one ready ray and it
  // contains the given points.
  const auto check_one_ray_fn = [&agg, kNumPoints](const std::array<PointXYZIF, kNumPoints> & pts)
    -> void {
      agg.insert(pts.cbegin(), pts.cend());
      EXPECT_TRUE(agg.is_ray_ready());

      while (agg.is_ray_ready()) {
        const auto & ray = agg.get_next_ray();
        // Last point is just to mark end of scan. So it wont be included in any ray.
        EXPECT_EQ(ray.size(), kNumPoints - 1);
        for (size_t i = 0; i < kNumPoints - 1; ++i) {
          EXPECT_EQ(ray[i].get_point_pointer()->x, pts[i].x);
          EXPECT_EQ(ray[i].get_point_pointer()->y, pts[i].y);
        }
      }
    };

  {
    // 2 points that fall in the same ray
    std::array<PointXYZIF, kNumPoints> pts;
    pts[0U].x = 1.0F; pts[0U].y = 5.0F;
    pts[1U].x = 2.0F; pts[1U].y = 10.0F;
    pts[2U].id = static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID);
    check_one_ray_fn(pts);
  }

  {
    // 2 points in the same ray but this ray is in different bin than the ray in the previous
    // scan. This should verify that the ray from the previous scan is not included as ready in this
    // scan.
    std::array<PointXYZIF, kNumPoints> pts;
    pts[0U].x = 1.0F; pts[0U].y = 2.0F;
    pts[1U].x = 2.0F; pts[1U].y = 4.0F;
    pts[2U].id = static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID);
    check_one_ray_fn(pts);
  }
}

}  // namespace
