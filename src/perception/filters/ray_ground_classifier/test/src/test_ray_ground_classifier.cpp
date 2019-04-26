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

#include <vector>
#include <tuple>
#include <functional>
#include <algorithm>
#include <chrono>
#include <random>

#include <lidar_utils/lidar_types.hpp>

#include "gtest/gtest.h"
#include "ray_ground_classifier/ray_ground_point_classifier.hpp"
#include "test_ray_ground_classifier_aux.hpp"

///////////////////////////////////////////////////////////////
namespace test_ray_ground_classifier
{

using autoware::common::lidar_utils::POINT_BLOCK_CAPACITY;

// do basic sanity checks
TEST_F(ray_ground_classifier, point_classification)
{
  using autoware::perception::filters::ray_ground_classifier::RayGroundPointClassifier;
  RayGroundPointClassifier cls{cfg};
  PointXYZIF pt;
  // close to ground: GROUND
  pt.x = 1.0F;
  pt.z = cfg.get_ground_z();
  EXPECT_EQ(cls.is_ground(PointXYZIFR{pt}), RayGroundPointClassifier::PointLabel::GROUND);
  // exhibit vertical structure: RETRO_NONGROUND
  pt.x += 0.1F;
  pt.z = cfg.get_ground_z() + 1.0F;
  EXPECT_EQ(cls.is_ground(PointXYZIFR{pt}), RayGroundPointClassifier::PointLabel::RETRO_NONGROUND);
  // near last nonground point: NONGROUND
  pt.x += 1.0F;
  EXPECT_EQ(cls.is_ground(PointXYZIFR{pt}), RayGroundPointClassifier::PointLabel::NONGROUND);
  // back to ground: PROVISIONAL_GROUND
  pt.x += 0.1F;
  pt.z = cfg.get_ground_z();
  EXPECT_EQ(cls.is_ground(PointXYZIFR{pt}),
    RayGroundPointClassifier::PointLabel::PROVISIONAL_GROUND);
  // distant nonground: NONLOCAL_NONGROUND
  pt.x += 10.1F;
  pt.z = cfg.get_ground_z() + 2.0F;
  EXPECT_EQ(cls.is_ground(PointXYZIFR{pt}),
    RayGroundPointClassifier::PointLabel::NONLOCAL_NONGROUND);
  // bad case: go backwards
  pt.x -= 1.0F;
  EXPECT_THROW(cls.is_ground(PointXYZIFR{pt}), std::runtime_error);
}

/*
Legend:
O = origin of sensor frame
r = r-axis
z = z-axis
g = ground point
n = not ground point
 */

/////
/*
 O-->r
 ^
 |
 z
 |
             g g g g .... g
 */
TEST_F(ray_ground_classifier, flat_ground)
{
  generate_groundspace(5.0F, 35.0F, 32U,
    [](float r) {return 0.0F;}, dat);

  generate_points(cfg, dat, pts, labels);

  label_and_check(cfg, pts, labels);
}

/////
/*
 O-->r
 ^
 |                        n
 z                        n
 |                        n
             g g g g .... g
 */
TEST_F(ray_ground_classifier, wall)
{
  const float ro = 5.0F, rf = 20.0F;
  generate_groundspace(ro, rf, 16U,
    [](float r) {return 0.0F;}, dat);

  // build a wall
  float height = 0.0F, dh = 0.5F;
  for (size_t idx = 0U; idx < 16U; ++idx) {
    height += dh;
    dat.push_back(std::make_tuple(rf, height, 1));
  }

  generate_points(cfg, dat, pts, labels);

  label_and_check(cfg, pts, labels, false);
}


/////
/*
 O-->r
 ^
 |                         n
 z                         n
 |                         n
             g g g g .... g
 */
TEST_F(ray_ground_classifier, wall2)
{
  const float ro = 5.0F, rf = 20.0F;
  generate_groundspace(ro, rf, 16U,
    [](float r) {return 0.0F;}, dat);

  // build a wall
  const float dr = 1.0F;
  float height = 0.0F, dh = 0.5F;
  for (size_t idx = 0U; idx < 16U; ++idx) {
    height += dh;
    dat.push_back(std::make_tuple(rf + dr, height, 1));
  }

  generate_points(cfg, dat, pts, labels);

  label_and_check(cfg, pts, labels, false);
}


/////
/*
 O-->r
 ^
 |                        n
 z                        n
 |        g g .... g      n
                          n
 */
TEST_F(ray_ground_classifier, ditch)
{
  const float ro = 5.0F, rf = 20.0F;
  generate_groundspace(ro, rf, 16U,
    [](float r) {return 0.0F;}, dat);

  // build a wall
  const float w = 5.0F;
  const float rd = w + rf;
  float height = -2.0F, dh = 0.5F;
  for (size_t idx = 0U; idx < 16U; ++idx) {
    height += dh;
    dat.push_back(std::make_tuple(rd, height, 1));
  }

  generate_points(cfg, dat, pts, labels);

  label_and_check(cfg, pts, labels, false);
}

/////
/*
 O-->r
 ^                   n
 |                   n
 z                   n
 |        g g .... g   g

 */
TEST_F(ray_ground_classifier, gap)
{
  const float ro = 5.0F, rf = 20.0F;
  generate_groundspace(ro, rf, 16U,
    [](float r) {return 0.0F;}, dat);

  // build a wall
  const float w = 1.0F;
  const float rd = w + rf;
  float height = 0.0F, dh = 0.5F;
  for (size_t idx = 0U; idx < 16U; ++idx) {
    height += dh;
    dat.push_back(std::make_tuple(rd, height, 1));
  }

  // gap point
  dat.push_back(std::make_tuple(rd + w, 0.0, 0));

  generate_points(cfg, dat, pts, labels);

  label_and_check(cfg, pts, labels, false);
}


/////
/*
 O-->r
 ^                          g
 |                      g g
 z                  g g
 |            g g g
       g g g
 */
TEST_F(ray_ground_classifier, valley)
{
  // linearly varying slope until max slope
  const float max_slope = 1.0F / 5.145F;
  // from http://www.ugpti.org/dotsc/engcenter/downloads/2010-07_VerticalCurves.pdf page 10
  // 25 mph/40 kmh => K = L/A ~4; A = max_grade
  // ==> L = 80m => dmdr = max_slope / L
  const float dmdr = max_slope / 80.0F;

  auto zfn = [ = ](const float r) {
      const float bp = max_slope / dmdr;
      if (r < bp) {
        return 0.5 * dmdr * r * r;
      } else {
        return 0.5 * dmdr * bp * bp + max_slope * (r - bp);
      }
    };

  const float ro = 5.0F, rf = 35.0F;
  generate_groundspace(ro, rf, 32U, zfn, dat);

  generate_points(cfg, dat, pts, labels);

  label_and_check(cfg, pts, labels);
}


/////
/*
 O-->r
 ^     g g g
 |            g g g
 z                  g g
 |                      g g
                            g
 */
TEST_F(ray_ground_classifier, crest)
{
  // linearly varying slope until max slope
  const float max_slope = -1.0F / 5.145F;
  // from http://www.ugpti.org/dotsc/engcenter/downloads/2010-07_VerticalCurves.pdf page 10
  // 25 mph/40 kmh => K = L/A ~4; A = max_grade
  // ==> L = 80m => dmdr = max_slope / L
  const float dmdr = max_slope / 80.0F;

  auto zfn = [ = ](const float r) {
      const float bp = max_slope / dmdr;
      if (r < bp) {
        return 0.5 * dmdr * r * r;
      } else {
        return 0.5 * dmdr * bp * bp + max_slope * (r - bp);
      }
    };

  const float ro = 5.0F, rf = 35.0F;
  generate_groundspace(ro, rf, 32U, zfn, dat);

  generate_points(cfg, dat, pts, labels);

  label_and_check(cfg, pts, labels);
}

/////
/*
 O-->r
 ^
 |
 z
 |        g      g g     g g
      g g   g  g     g g     g
 */
TEST_F(ray_ground_classifier, rough)
{
  // up to 10 cm variation in height
  auto zfn = [](const float r) {
      return 0.1F * sinf(r);
    };

  const float ro = 5.0F, rf = 40.0F;
  generate_groundspace(ro, rf, 32U, zfn, dat);

  generate_points(cfg, dat, pts, labels);

  label_and_check(cfg, pts, labels);
}


/////
/*
 O-->r
 ^
 |
 z   g g g g       g g g
 |           g   g
               g
 */
TEST_F(ray_ground_classifier, driveway)
{
  // driveway itself
  const float ro = 5.0F, rf = 10.0F;
  generate_groundspace(ro, rf, 12U, [](float r) {return 0.0F;}, dat);
  const float W = 1.5F;
  const float dr = 1.0;
  const float H = 0.25F;  // 10-20 cm curb + dip
  // dip
  float r1 = rf + dr, r2 = rf + dr + W;
  generate_groundspace(r1, r2, 3U,
    [ = ](float r) {
      return (r - r1) * (-H / W);
    }, dat);

  // undip
  r1 = rf + 2 * dr + W, r2 = rf + 2 * dr + 2 * W;
  generate_groundspace(r1, r2, 3U,
    [ = ](float r) {
      return (r - r1) * (H / W);
    }, dat);
  // road
  generate_groundspace(r2 + dr, r2 + dr + 10, 12U, [](float r) {return 0.0F;}, dat);

  generate_points(cfg, dat, pts, labels);

  label_and_check(cfg, pts, labels);
}

/////
/*
          n n n n n
                    g g g g g
g g g g g
*/
TEST_F(ray_ground_classifier, plateau_ground)
{
  Config cfg2{
    0.4F,          // sensor_height_m,
    10.0F,         // max_local_slope_deg,
    3.0F,          // max_global_slope_deg,
    20.0F,         // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    1.8F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m,
    -1.0F,         // min_height_m,
    2.5F           // max_height_m
  };
  ASSERT_LT(fabsf(cfg2.get_max_local_slope() - tanf(10.0F * 3.14159F / 180.0F)), 0.001F);
  dat =
  {
    std::tuple<float, float, int>(1.0F, 0.0F, 0),
    std::tuple<float, float, int>(2.0F, 0.0F, 0),
    std::tuple<float, float, int>(3.0F, 0.0F, 0),
    std::tuple<float, float, int>(4.0F, 0.0F, 0),
    std::tuple<float, float, int>(5.0F, 0.0F, 0),
    std::tuple<float, float, int>(5.0F, 2.0F, 1),
    std::tuple<float, float, int>(6.0F, 2.0F, 1),
    std::tuple<float, float, int>(7.0F, 2.0F, 1),
    std::tuple<float, float, int>(8.0F, 2.0F, 1),
    std::tuple<float, float, int>(8.1F, 0.5F, 0),
    std::tuple<float, float, int>(9.0F, 0.5F, 0),
    std::tuple<float, float, int>(10.0F, 0.5F, 0),
    std::tuple<float, float, int>(11.0F, 0.5F, 0),
    std::tuple<float, float, int>(12.0F, 0.5F, 0)
  };
  generate_points(cfg2, dat, pts, labels);
  label_and_check(cfg2, pts, labels, false, false);
}

/////
// exercise provisional ground
/*
                n n
                      n               n n
g g g g g g g g     n   g
*/
TEST_F(ray_ground_classifier, provisional_ground)
{
  Config cfg2{
    0.4F,          // sensor_height_m,
    5.0F,          // max_local_slope_deg,
    3.0F,          // max_global_slope_deg,
    70.0F,         // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    5.0F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m,
    -1.0F,         // min_height_m,
    1.5F           // max_height_m
  };
  ASSERT_LT(fabsf(cfg2.get_nonground_retro_thresh() - tanf(70.0F * 3.14159F / 180.0F)), 0.001F);
  ASSERT_LT(fabsf(cfg2.get_max_last_local_ground_thresh() - 5.0F), 0.001F);
  dat =
  {
    std::tuple<float, float, int>(1.0F, 0.0F, 0),
    std::tuple<float, float, int>(2.0F, 0.0F, 0),
    std::tuple<float, float, int>(3.0F, 0.0F, 0),
    std::tuple<float, float, int>(4.0F, 0.0F, 0),
    std::tuple<float, float, int>(5.0F, 0.0F, 0),
    std::tuple<float, float, int>(6.0F, 2.0F, 1),
    std::tuple<float, float, int>(7.0F, 2.0F, 1),
    std::tuple<float, float, int>(8.0F, 0.0F, 1),
    std::tuple<float, float, int>(9.0F, 1.0F, 1),
    std::tuple<float, float, int>(10.0F, 0.0F, 0),
    std::tuple<float, float, int>(20.0F, 2.0F, 1),
    std::tuple<float, float, int>(21.0F, 2.0F, 1),
  };
  generate_points(cfg2, dat, pts, labels);
  label_and_check(cfg2, pts, labels, false);
}

// Test height filtering
TEST_F(ray_ground_classifier, height_filter)
{
  EXPECT_FLOAT_EQ(cfg.get_min_height(), -1.0F);
  EXPECT_FLOAT_EQ(cfg.get_max_height(), 1.5F);
  autoware::perception::filters::ray_ground_classifier::RayGroundClassifier filter(cfg);
  autoware::common::lidar_utils::PointXYZIF pt1, pt2;
  pt1.x = 1.0F;
  pt1.z = cfg.get_max_height() + 0.00001F;
  filter.insert(pt1);
  pt2.x = 2.0F;
  pt2.z = cfg.get_min_height() - 0.00001F;
  filter.insert(pt2);
  autoware::perception::filters::ray_ground_classifier::PointBlock blk1, blk2;
  blk1.clear();
  blk2.clear();
  filter.partition(blk1, blk2, false);
  EXPECT_EQ(blk1.size(), 0U);
  EXPECT_EQ(blk2.size(), 0U);
}

// same as wall, but just a different path to exercise the logic
TEST_F(ray_ground_classifier, structured_partition_and_other)
{
  const float ro = 5.0F, rf = 20.0F;
  generate_groundspace(ro, rf, 16U,
    [](float r) {return 0.0F;}, dat);

  // build a wall
  float height = 0.0F, dh = 0.1F;
  for (size_t idx = 0U; idx < 16U; ++idx) {
    height += dh;
    dat.push_back(std::make_tuple(rf, height, 1));
  }
  reverse(std::begin(dat), std::end(dat));
  generate_points(cfg, dat, pts, labels);
  PointBlock raw_points, ground_points, nonground_points;
  raw_points.clear();
  // initialize weird numbers
  ground_points.resize(3U);
  nonground_points.resize(5U);
  for (PointXYZIF & pt : pts) {
    pt.id = 0U;
    raw_points.push_back(pt);
  }
  // add second ray
  for (PointXYZIF & pt : pts) {
    pt.id = 1U;
    raw_points.push_back(pt);
  }
  autoware::perception::filters::ray_ground_classifier::RayGroundClassifier cls{cfg};
  cls.structured_partition(raw_points, ground_points, nonground_points);
  // check size: tolerance is 2 for 2 rays
  EXPECT_LE(fabs(static_cast<int32_t>(ground_points.size()) - (2 * 16)), 2) << ground_points.size();
  EXPECT_LE(fabs(static_cast<int32_t>(nonground_points.size()) - (2 * 16)),
    2) << nonground_points.size();
  // check individual points
  for (uint32_t idx = 0U; idx < ground_points.size(); ++idx) {
    const auto & ground_pt = ground_points[idx];
    EXPECT_LT(ground_pt.x, 20.0F);
    // hack due to knowledge that they should be roughly ordered in radius, then height
    if ((idx != 15U) && (idx != 30U)) {
      EXPECT_LT(ground_pt.z,
        cfg.get_ground_z() + 0.1F) << idx << ", " << ground_pt.x << ", " << ground_pt.id;
    }
  }
  for (uint32_t idx = 0U; idx < nonground_points.size(); ++idx) {
    const auto & nonground_pt = nonground_points[idx];
    EXPECT_GT(nonground_pt.x, 19.0F);
    if ((idx != 0U) && (idx != 17U)) {
      EXPECT_GT(nonground_pt.z, cfg.get_ground_z()) << idx;
    }
  }
  // test bad insert
  PointXYZIF pt;
  for (uint32_t idx = 0U;
    idx <
    static_cast<uint32_t>(POINT_BLOCK_CAPACITY);
    ++idx)
  {
    cls.insert(pt);
  }
  EXPECT_THROW(cls.insert(pt), std::runtime_error);
  // test bad partition due to not being able to fit result
  ground_points.resize(1U);
  nonground_points.clear();
  EXPECT_FALSE(cls.can_fit_result(ground_points, nonground_points));
  EXPECT_THROW(cls.partition(ground_points, nonground_points, false), std::runtime_error);
  ground_points.clear();
  nonground_points.resize(1U);
  EXPECT_FALSE(cls.can_fit_result(ground_points, nonground_points));
  EXPECT_THROW(cls.partition(ground_points, nonground_points, false), std::runtime_error);
  ground_points.clear();
  nonground_points.clear();
  EXPECT_TRUE(cls.can_fit_result(ground_points, nonground_points));
  cls.partition(ground_points, nonground_points, false);
  EXPECT_EQ(ground_points.size() + nonground_points.size(),
    static_cast<uint32_t>(POINT_BLOCK_CAPACITY));
  // check empty
  raw_points.resize(0);
  ground_points.resize(1U);
  nonground_points.resize(1U);
  cls.structured_partition(raw_points, ground_points, nonground_points);
  EXPECT_EQ(ground_points.size(), 0);
  EXPECT_EQ(nonground_points.size(), 0);
  // add end of scan
  raw_points.resize(1);
  raw_points[0U].id = static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID);
  cls.structured_partition(raw_points, ground_points, nonground_points);
  EXPECT_EQ(ground_points.size(), 1);
  EXPECT_EQ(nonground_points.size(), 1);
  EXPECT_EQ(ground_points[0U].id, static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID));
  EXPECT_EQ(nonground_points[0U].id, static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID));
}
/////
TEST_F(ray_ground_classifier, bad_cases)
{
  // check negative slopes
  EXPECT_THROW(Config cfg2({
      0.4F,        // sensor_height_m,
      -5.0F,       // max_local_slope_deg,
      3.0F,        // max_global_slope_deg,
      70.0F,       // nonground_retro_thresh_deg,
      0.05F,       // min_height_thresh_m,
      1.5F,        // max_global_height_thresh_m,
      5.0F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m,
      -1.0F,       // min_height_m,
      1.5F         // max_height_m
    }), std::runtime_error);
  EXPECT_THROW(Config cfg2({
      0.4F,        // sensor_height_m,
      5.0F,       // max_local_slope_deg,
      -3.0F,        // max_global_slope_deg,
      70.0F,       // nonground_retro_thresh_deg,
      0.05F,       // min_height_thresh_m,
      1.5F,        // max_global_height_thresh_m,
      5.0F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m,
      -1.0F,       // min_height_m,
      1.5F         // max_height_m
    }), std::runtime_error);
  EXPECT_THROW(Config cfg2({
      0.4F,        // sensor_height_m,
      5.0F,       // max_local_slope_deg,
      3.0F,        // max_global_slope_deg,
      -70.0F,       // nonground_retro_thresh_deg,
      0.05F,       // min_height_thresh_m,
      1.5F,        // max_global_height_thresh_m,
      5.0F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m,
      -1.0F,       // min_height_m,
      1.5F         // max_height_m
    }), std::runtime_error);

  // check >90 deg slopes
  EXPECT_THROW(Config cfg2({
      0.4F,        // sensor_height_m,
      95.0F,       // max_local_slope_deg,
      3.0F,        // max_global_slope_deg,
      70.0F,       // nonground_retro_thresh_deg,
      0.05F,       // min_height_thresh_m,
      1.5F,        // max_global_height_thresh_m,
      5.0F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m,
      -1.0F,       // min_height_m,
      1.5F         // max_height_m
    }), std::runtime_error);
  EXPECT_THROW(Config cfg2({
      0.4F,        // sensor_height_m,
      5.0F,       // max_local_slope_deg,
      93.0F,        // max_global_slope_deg,
      70.0F,       // nonground_retro_thresh_deg,
      0.05F,       // min_height_thresh_m,
      1.5F,        // max_global_height_thresh_m,
      5.0F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m,
      -1.0F,       // min_height_m,
      1.5F         // max_height_m
    }), std::runtime_error);
  EXPECT_THROW(Config cfg2({
      0.4F,        // sensor_height_m,
      5.0F,       // max_local_slope_deg,
      3.0F,        // max_global_slope_deg,
      170.0F,       // nonground_retro_thresh_deg,
      0.05F,       // min_height_thresh_m,
      1.5F,        // max_global_height_thresh_m,
      5.0F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m,
      -1.0F,       // min_height_m,
      1.5F         // max_height_m
    }), std::runtime_error);
  // check negative height thresholds
  EXPECT_THROW(Config cfg2({
      0.4F,        // sensor_height_m,
      5.0F,       // max_local_slope_deg,
      3.0F,        // max_global_slope_deg,
      170.0F,       // nonground_retro_thresh_deg,
      -0.05F,       // min_height_thresh_m,
      1.5F,        // max_global_height_thresh_m,
      5.0F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m,
      -1.0F,       // min_height_m,
      1.5F         // max_height_m
    }), std::runtime_error);
  EXPECT_THROW(Config cfg2({
      0.4F,        // sensor_height_m,
      5.0F,       // max_local_slope_deg,
      3.0F,        // max_global_slope_deg,
      70.0F,       // nonground_retro_thresh_deg,
      0.05F,       // min_height_thresh_m,
      -1.5F,        // max_global_height_thresh_m,
      5.0F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m,
      -1.0F,       // min_height_m,
      1.5F         // max_height_m
    }), std::runtime_error);
  // check for min/max height consistency
  EXPECT_THROW(Config cfg2({
      0.4F,        // sensor_height_m,
      5.0F,       // max_local_slope_deg,
      3.0F,        // max_global_slope_deg,
      70.0F,       // nonground_retro_thresh_deg,
      0.05F,       // min_height_thresh_m,
      1.5F,        // max_global_height_thresh_m,
      5.0F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m,
      1.0F,       // min_height_m,
      0.5F         // max_height_m
    }), std::runtime_error);
  // check for consistency wrt vertical structure threshold
  EXPECT_THROW(Config cfg2({
      0.4F,        // sensor_height_m,
      75.0F,       // max_local_slope_deg,
      3.0F,        // max_global_slope_deg,
      70.0F,       // nonground_retro_thresh_deg,
      0.05F,       // min_height_thresh_m,
      1.5F,        // max_global_height_thresh_m,
      5.0F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m,
      -1.0F,       // min_height_m,
      1.5F         // max_height_m
    }), std::runtime_error);
  EXPECT_THROW(Config cfg2({
      0.4F,        // sensor_height_m,
      5.0F,       // max_local_slope_deg,
      73.0F,        // max_global_slope_deg,
      70.0F,       // nonground_retro_thresh_deg,
      0.05F,       // min_height_thresh_m,
      1.5F,        // max_global_height_thresh_m,
      5.0F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m,
      -1.0F,       // min_height_m,
      1.5F         // max_height_m
    }), std::runtime_error);
  // check for local vs global consistency
  EXPECT_THROW(Config cfg2({
      0.4F,        // sensor_height_m,
      5.0F,       // max_local_slope_deg,
      73.0F,        // max_global_slope_deg,
      70.0F,       // nonground_retro_thresh_deg,
      0.05F,       // min_height_thresh_m,
      1.5F,        // max_global_height_thresh_m,
      1.0F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m,
      -1.0F,       // min_height_m,
      1.5F         // max_height_m
    }), std::runtime_error);
}

/////
// Run ray ground filter on simulated data a hundred times or so to estimate the runtime
// This can be turned into a real test when we get SIL/HIL going on
TEST_F(ray_ground_classifier, benchmark)
{
  // Generate simulated data for rays of size 8, 16, 32, 64, 128, 256, and 512
  PointBlock raw_points, ground_points, nonground_points;
  autoware::perception::filters::ray_ground_classifier::RayGroundClassifier cls{cfg};
  std::mt19937 gen(1338U);  // Standard mersenne_twister_engine seeded with value
  using unif_distr = std::uniform_real_distribution<float>;
  unif_distr mode_samp{0.0F, 1.0F};
  unif_distr dr_inc_samp{0.1F, 5.0F};
  unif_distr dr_unif_samp{0.0F, 0.2F};
  unif_distr dh_unif_samp{0.1F, 1.0F};
  std::normal_distribution<float> dh_gauss_samp{0.0F, 0.1F};
  auto fn = [&](const uint32_t ray_size) {
      raw_points.clear();
      // generate points using a simple markov chain
      constexpr float ground2vert_prob = 0.1F;
      constexpr float nonground2vert_prob = 0.2F;
      constexpr float vert2nonground_prob = 0.7F;
      constexpr float vert2ground_prob = 0.8F;
      // initialize state variables
      uint32_t mode = 0;  // 0 = ground, 1 = nonground, 2 = vertical structure ground -> nonground
      //                     3 = veritcal structure nonground->ground
      float last_dr = 1.0F;
      PointXYZIF pt;
      for (uint32_t idx = 0U;
        idx <
        static_cast<uint32_t>(POINT_BLOCK_CAPACITY);
        ++idx)
      {
        if (idx % ray_size == 0U) {
          // start new ray: reinitialize state variables
          mode = 0;
          last_dr = 1.0F;
          pt.x = 0.0F;
          pt.z = cfg.get_ground_z();
        }
        // check state transition
        const float r = mode_samp(gen);
        switch (mode) {
          case 0:
            if (r < ground2vert_prob) {
              mode = 2;
            }
            break;
          case 1:
            if (r < nonground2vert_prob) {
              mode = 3;
            }
            break;
          case 2:
            if (r < vert2nonground_prob) {
              mode = 1;
            }
            break;
          case 3:
            if (r < vert2ground_prob) {
              mode = 0;
            }
            break;
          default:
            throw std::runtime_error("Test failure: Mode transition");
        }
        // generate point
        float dr, dh = 0.0F;
        switch (mode) {
          case 0:
          case 1:
            // fuzzy (gaussian) height delta
            dh = dh_gauss_samp(gen);
            // increasing radial delta (uniform)
            dr = last_dr + dr_inc_samp(gen);
            last_dr = dr;
            break;
          case 2:
          case 3:
            // fuzzy (uniform) radial delta
            dr = dr_unif_samp(gen);
            // positive (uniform) height delta
            dh = dh_unif_samp(gen);
            break;
          default:
            throw std::runtime_error("Test failure: Point generation");
        }
        if (3 == mode) {
          dh = -dh;
        }
        pt.x += dr;
        pt.z += dh;
        // insert point
        raw_points.push_back(pt);
      }
      // start timer
      const auto start = std::chrono::steady_clock::now();
      constexpr uint32_t num_iter = 100U;
      for (uint32_t idx = 0U; idx < num_iter; ++idx) {
        cls.structured_partition(raw_points, ground_points, nonground_points);
      }
      // end timer, print result
      const auto diff = std::chrono::steady_clock::now() - start;
      std::cout << "Ray size " << ray_size << ",\truntime = " <<
        std::chrono::duration_cast<std::chrono::microseconds>(diff).count() /
      (1000.0F * num_iter /** (static_cast<uint32_t>(POINT_BLOCK_CAPACITY) / ray_size)*/) <<
        "ms\n";
      // We can return average runtime for a full block later
      // Using the VLP16 as an example, we expect 750 packets/ms, so we should expect the filter to
      // run on a full block in no more than 1 ms
    };
  fn(8U);
  fn(16U);
  fn(32U);
  fn(64U);
  fn(128U);
  fn(256U);
  fn(512U);
}

}  // namespace test_ray_ground_classifier
