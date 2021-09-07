// Copyright 2017-2020 the Autoware Foundation, Arm Limited
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <common/types.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <random>
#include <tuple>
#include <vector>

#include "gtest/gtest.h"
#include "ray_ground_classifier/ray_ground_point_classifier.hpp"
#include "test_ray_ground_classifier_aux.hpp"

///////////////////////////////////////////////////////////////
using autoware::common::types::POINT_BLOCK_CAPACITY;
using autoware::common::types::float32_t;

// do basic sanity checks
TEST_F(RayGroundClassifier, PointClassification)
{
  using autoware::perception::filters::ray_ground_classifier::RayGroundPointClassifier;
  RayGroundPointClassifier cls{cfg};
  PointXYZIF pt_g, pt_rng, pt_ng, pt_pg, pt_nlng, bad_pt;
  // close to ground: GROUND
  pt_g.x = 1.0F;
  pt_g.z = cfg.m_ground_z_m;
  EXPECT_EQ(
    cls.is_ground(PointXYZIFR{&pt_g}),
    RayGroundPointClassifier::PointLabel::GROUND);
  // exhibit vertical structure: RETRO_NONGROUND
  pt_rng.x = pt_g.x + 0.1F;
  pt_rng.z = cfg.m_ground_z_m + 1.0F;
  EXPECT_EQ(
    cls.is_ground(PointXYZIFR{&pt_rng}),
    RayGroundPointClassifier::PointLabel::RETRO_NONGROUND);
  // near last nonground point: NONGROUND
  pt_ng.x += pt_rng.x + 1.0F;
  pt_ng.z = pt_rng.z;
  EXPECT_EQ(
    cls.is_ground(PointXYZIFR{&pt_ng}),
    RayGroundPointClassifier::PointLabel::NONGROUND);
  // back to ground: PROVISIONAL_GROUND
  pt_pg.x = pt_ng.x + 0.1F;
  pt_pg.z = cfg.m_ground_z_m;
  EXPECT_EQ(
    cls.is_ground(PointXYZIFR{&pt_pg}),
    RayGroundPointClassifier::PointLabel::PROVISIONAL_GROUND);
  // distant nonground: NONLOCAL_NONGROUND
  pt_nlng.x += pt_pg.x + 10.1F;
  pt_nlng.z = cfg.m_ground_z_m + 2.0F;
  EXPECT_EQ(
    cls.is_ground(PointXYZIFR{&pt_nlng}),
    RayGroundPointClassifier::PointLabel::NONLOCAL_NONGROUND);
  // bad case: go backwards
  bad_pt.x = pt_nlng.x - 1.0F;
  bad_pt.z = pt_nlng.z;
  EXPECT_THROW(cls.is_ground(PointXYZIFR{&bad_pt}), std::runtime_error);
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
TEST_F(RayGroundClassifier, FlatGround)
{
  generate_groundspace(
    5.0F, 35.0F, 32U,
    [](float32_t r) {(void)r; return 0.0F;}, dat);

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
TEST_F(RayGroundClassifier, Wall)
{
  const float32_t ro = 5.0F, rf = 20.0F;
  generate_groundspace(
    ro, rf, 16U,
    [](float32_t r) {(void)r; return 0.0F;}, dat);

  // build a wall
  float32_t height = 0.0F, dh = 0.5F;
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
TEST_F(RayGroundClassifier, Wall2)
{
  const float32_t ro = 5.0F, rf = 20.0F;
  generate_groundspace(
    ro, rf, 16U,
    [](float32_t r) {(void)r; return 0.0F;}, dat);

  // build a wall
  const float32_t dr = 1.0F;
  float32_t height = 0.0F, dh = 0.5F;
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
TEST_F(RayGroundClassifier, Ditch)
{
  const float32_t ro = 5.0F, rf = 20.0F;
  generate_groundspace(
    ro, rf, 16U,
    [](float32_t r) {(void)r; return 0.0F;}, dat);

  // build a wall
  const float32_t w = 5.0F;
  const float32_t rd = w + rf;
  float32_t height = -2.0F, dh = 0.5F;
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
TEST_F(RayGroundClassifier, Gap)
{
  const float32_t ro = 5.0F, rf = 20.0F;
  generate_groundspace(
    ro, rf, 16U,
    [](float32_t r) {(void)r; return 0.0F;}, dat);

  // build a wall
  const float32_t w = 1.0F;
  const float32_t rd = w + rf;
  float32_t height = 0.0F, dh = 0.5F;
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
TEST_F(RayGroundClassifier, Valley)
{
  // linearly varying slope until max slope
  const float32_t max_slope = 1.0F / 5.145F;
  // from http://www.ugpti.org/dotsc/engcenter/downloads/2010-07_VerticalCurves.pdf page 10
  // 25 mph/40 kmh => K = L/A ~4; A = max_grade
  // ==> L = 80m => dmdr = max_slope / L
  const float32_t dmdr = max_slope / 80.0F;

  auto zfn = [ = ](const float32_t r) {
      const float32_t bp = max_slope / dmdr;
      if (r < bp) {
        return 0.5 * dmdr * r * r;
      } else {
        return 0.5 * dmdr * bp * bp + max_slope * (r - bp);
      }
    };

  const float32_t ro = 5.0F, rf = 35.0F;
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
TEST_F(RayGroundClassifier, Crest)
{
  // linearly varying slope until max slope
  const float32_t max_slope = -1.0F / 5.145F;
  // from http://www.ugpti.org/dotsc/engcenter/downloads/2010-07_VerticalCurves.pdf page 10
  // 25 mph/40 kmh => K = L/A ~4; A = max_grade
  // ==> L = 80m => dmdr = max_slope / L
  const float32_t dmdr = max_slope / 80.0F;

  auto zfn = [ = ](const float32_t r) {
      const float32_t bp = max_slope / dmdr;
      if (r < bp) {
        return 0.5 * dmdr * r * r;
      } else {
        return 0.5 * dmdr * bp * bp + max_slope * (r - bp);
      }
    };

  const float32_t ro = 5.0F, rf = 35.0F;
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
TEST_F(RayGroundClassifier, Rough)
{
  // up to 10 cm variation in height
  auto zfn = [](const float32_t r) {
      return 0.1F * sinf(r);
    };

  const float32_t ro = 5.0F, rf = 40.0F;
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
TEST_F(RayGroundClassifier, Driveway)
{
  // driveway itself
  const float32_t ro = 5.0F, rf = 10.0F;
  generate_groundspace(ro, rf, 12U, [](float32_t r) {(void)r; return 0.0F;}, dat);
  const float32_t W = 1.5F;
  const float32_t dr = 1.0F;
  const float32_t H = 0.25F;  // 10-20 cm curb + dip
  // dip
  float32_t r1 = rf + dr, r2 = rf + dr + W;
  generate_groundspace(
    r1, r2, 3U,
    [ = ](float32_t r) {
      return (r - r1) * (-H / W);
    }, dat);

  // undip
  r1 = rf + 2 * dr + W, r2 = rf + 2 * dr + 2 * W;
  generate_groundspace(
    r1, r2, 3U,
    [ = ](float32_t r) {
      return (r - r1) * (H / W);
    }, dat);
  // road
  generate_groundspace(r2 + dr, r2 + dr + 10, 12U, [](float32_t r) {(void)r; return 0.0F;}, dat);

  generate_points(cfg, dat, pts, labels);

  label_and_check(cfg, pts, labels);
}

/////
/*
          n n n n n
                    g g g g g
g g g g g
*/
TEST_F(RayGroundClassifier, PlateauGround)
{
  Config cfg2{
    0.4F,          // sensor_height_m,
    10.0F,         // max_local_slope_deg,
    3.0F,          // max_global_slope_deg,
    20.0F,         // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    1.8F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m
  };
  ASSERT_LT(fabsf(cfg2.m_max_local_slope - tanf(10.0F * 3.14159F / 180.0F)), 0.001F);
  dat =
  {
    std::tuple<float32_t, float32_t, uint8_t>(1.0F, 0.0F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(2.0F, 0.0F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(3.0F, 0.0F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(4.0F, 0.0F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(5.0F, 0.0F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(5.0F, 2.0F, 1),
    std::tuple<float32_t, float32_t, uint8_t>(6.0F, 2.0F, 1),
    std::tuple<float32_t, float32_t, uint8_t>(7.0F, 2.0F, 1),
    std::tuple<float32_t, float32_t, uint8_t>(8.0F, 2.0F, 1),
    std::tuple<float32_t, float32_t, uint8_t>(8.1F, 0.5F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(9.0F, 0.5F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(10.0F, 0.5F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(11.0F, 0.5F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(12.0F, 0.5F, 0)
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
TEST_F(RayGroundClassifier, ProvisionalGround)
{
  Config cfg2{
    0.4F,          // sensor_height_m,
    5.0F,          // max_local_slope_deg,
    3.0F,          // max_global_slope_deg,
    70.0F,         // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    5.0F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m
  };
  ASSERT_LT(fabsf(cfg2.m_nonground_retro_thresh - tanf(70.0F * 3.14159F / 180.0F)), 0.001F);
  ASSERT_LT(fabsf(cfg2.m_max_last_local_ground_thresh_m - 5.0F), 0.001F);
  dat =
  {
    std::tuple<float32_t, float32_t, uint8_t>(1.0F, 0.0F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(2.0F, 0.0F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(3.0F, 0.0F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(4.0F, 0.0F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(5.0F, 0.0F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(6.0F, 2.0F, 1),
    std::tuple<float32_t, float32_t, uint8_t>(7.0F, 2.0F, 1),
    std::tuple<float32_t, float32_t, uint8_t>(8.0F, 0.0F, 1),
    std::tuple<float32_t, float32_t, uint8_t>(9.0F, 1.0F, 1),
    std::tuple<float32_t, float32_t, uint8_t>(10.0F, 0.0F, 0),
    std::tuple<float32_t, float32_t, uint8_t>(20.0F, 2.0F, 1),
    std::tuple<float32_t, float32_t, uint8_t>(21.0F, 2.0F, 1),
  };
  generate_points(cfg2, dat, pts, labels);
  label_and_check(cfg2, pts, labels, false);
}

// same as wall, but just a different path to exercise the logic
TEST_F(RayGroundClassifier, StructuredPartitionAndOther)
{
  const float32_t ro = 5.0F, rf = 20.0F;
  generate_groundspace(
    ro, rf, 16U,
    [](float32_t r) {(void)r; return 0.0F;}, dat);

  // build a wall
  float32_t height = 0.0F, dh = 0.1F;
  for (size_t idx = 0U; idx < 16U; ++idx) {
    height += dh;
    dat.push_back(std::make_tuple(rf, height, 1));
  }
  reverse(std::begin(dat), std::end(dat));
  generate_points(cfg, dat, pts, labels);
  PointPtrBlock raw_points, ground_points, nonground_points;
  raw_points.clear();
  // initialize weird numbers
  ground_points.resize(3U);
  nonground_points.resize(5U);
  std::vector<PointXYZIF> all_points;
  all_points.reserve(2 * pts.size());
  for (PointXYZIF & pt : pts) {
    pt.id = 0U;
    all_points.push_back(pt);
    raw_points.push_back(&all_points.back());
  }
  // add second ray
  for (PointXYZIF & pt : pts) {
    pt.id = 1U;
    all_points.push_back(pt);
    raw_points.push_back(&all_points.back());
  }
  autoware::perception::filters::ray_ground_classifier::RayGroundClassifier cls{cfg};
  cls.structured_partition(raw_points, ground_points, nonground_points);
  // check size: tolerance is 2 for 2 rays
  EXPECT_LE(fabs(static_cast<int32_t>(ground_points.size()) - (2 * 16)), 2) << ground_points.size();
  EXPECT_LE(
    fabs(static_cast<int32_t>(nonground_points.size()) - (2 * 16)),
    2) << nonground_points.size();
  // check individual points
  for (uint32_t idx = 0U; idx < ground_points.size(); ++idx) {
    const auto & ground_pt = ground_points[idx];
    EXPECT_LT(ground_pt->x, 20.0F);
    // hack due to knowledge that they should be roughly ordered in radius, then height
    if ((idx != 15U) && (idx != 30U)) {
      EXPECT_LT(
        ground_pt->z,
        cfg.m_ground_z_m + 0.1F) << idx << ", " << ground_pt->x << ", " << ground_pt->id;
    }
  }
  for (uint32_t idx = 0U; idx < nonground_points.size(); ++idx) {
    const auto & nonground_pt = nonground_points[idx];
    EXPECT_GT(nonground_pt->x, 19.0F);
    if ((idx != 0U) && (idx != 17U)) {
      EXPECT_GT(nonground_pt->z, cfg.m_ground_z_m) << idx;
    }
  }
  // test bad insert
  PointXYZIF pt;
  for (uint32_t idx = 0U;
    idx <
    static_cast<uint32_t>(POINT_BLOCK_CAPACITY);
    ++idx)
  {
    // we don't care about the data in those points,so we can use the same pointer all the while
    cls.insert(&pt);
  }
  EXPECT_THROW(cls.insert(&pt), std::runtime_error);
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
  EXPECT_EQ(
    ground_points.size() + nonground_points.size(),
    static_cast<uint32_t>(POINT_BLOCK_CAPACITY));
  // check empty
  raw_points.resize(0);
  ground_points.resize(1U);
  nonground_points.resize(1U);
  cls.structured_partition(raw_points, ground_points, nonground_points);
  EXPECT_EQ(ground_points.size(), 0UL);
  EXPECT_EQ(nonground_points.size(), 0UL);
  // add end of scan
  raw_points.resize(1);
  PointXYZIF eos_pt;
  eos_pt.id = static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID);
  raw_points[0U] = &eos_pt;
  cls.structured_partition(raw_points, ground_points, nonground_points);
  EXPECT_EQ(ground_points.size(), 1UL);
  EXPECT_EQ(nonground_points.size(), 1UL);
  EXPECT_EQ(ground_points[0U]->id, static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID));
  EXPECT_EQ(nonground_points[0U]->id, static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID));
}
/////
TEST_F(RayGroundClassifier, BadCases)
{
  // check negative slopes
  EXPECT_THROW(
    Config cfg2(
  {
    0.4F,          // sensor_height_m,
    -5.0F,         // max_local_slope_deg,
    3.0F,          // max_global_slope_deg,
    70.0F,         // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    5.0F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m
  }), std::runtime_error);
  EXPECT_THROW(
    Config cfg2(
  {
    0.4F,          // sensor_height_m,
    5.0F,          // max_local_slope_deg,
    -3.0F,         // max_global_slope_deg,
    70.0F,         // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    5.0F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m
  }), std::runtime_error);
  EXPECT_THROW(
    Config cfg2(
  {
    0.4F,          // sensor_height_m,
    5.0F,          // max_local_slope_deg,
    3.0F,          // max_global_slope_deg,
    -70.0F,        // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    5.0F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m
  }), std::runtime_error);

  // check >90 deg slopes
  EXPECT_THROW(
    Config cfg2(
  {
    0.4F,          // sensor_height_m,
    95.0F,         // max_local_slope_deg,
    3.0F,          // max_global_slope_deg,
    70.0F,         // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    5.0F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m
  }), std::runtime_error);
  EXPECT_THROW(
    Config cfg2(
  {
    0.4F,          // sensor_height_m,
    5.0F,          // max_local_slope_deg,
    93.0F,         // max_global_slope_deg,
    70.0F,         // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    5.0F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m
  }), std::runtime_error);
  EXPECT_THROW(
    Config cfg2(
  {
    0.4F,          // sensor_height_m,
    5.0F,          // max_local_slope_deg,
    3.0F,          // max_global_slope_deg,
    170.0F,        // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    5.0F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m
  }), std::runtime_error);
  // check negative height thresholds
  EXPECT_THROW(
    Config cfg2(
  {
    0.4F,          // sensor_height_m,
    5.0F,          // max_local_slope_deg,
    3.0F,          // max_global_slope_deg,
    170.0F,        // nonground_retro_thresh_deg,
    -0.05F,        // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    5.0F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m
  }), std::runtime_error);
  EXPECT_THROW(
    Config cfg2(
  {
    0.4F,          // sensor_height_m,
    5.0F,          // max_local_slope_deg,
    3.0F,          // max_global_slope_deg,
    70.0F,         // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    -1.5F,         // max_global_height_thresh_m,
    5.0F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m
  }), std::runtime_error);
  // check for consistency wrt vertical structure threshold
  EXPECT_THROW(
    Config cfg2(
  {
    0.4F,          // sensor_height_m,
    75.0F,         // max_local_slope_deg,
    3.0F,          // max_global_slope_deg,
    70.0F,         // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    5.0F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m
  }), std::runtime_error);
  EXPECT_THROW(
    Config cfg2(
  {
    0.4F,          // sensor_height_m,
    5.0F,          // max_local_slope_deg,
    73.0F,         // max_global_slope_deg,
    70.0F,         // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    5.0F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m
  }), std::runtime_error);
  // check for local vs global consistency
  EXPECT_THROW(
    Config cfg2(
  {
    0.4F,          // sensor_height_m,
    5.0F,          // max_local_slope_deg,
    73.0F,         // max_global_slope_deg,
    70.0F,         // nonground_retro_thresh_deg,
    0.05F,         // min_height_thresh_m,
    1.5F,          // max_global_height_thresh_m,
    1.0F,          // max_last_local_ground_thresh_m,
    2.0,           // max_provisional_ground_distance_m
  }), std::runtime_error);
}

/////
// Run ray ground filter on simulated data a hundred times or so to estimate the runtime
// This can be turned into a real test when we get SIL/HIL going on
TEST_F(RayGroundClassifier, Benchmark)
{
  // Generate simulated data for rays of size 8, 16, 32, 64, 128, 256, and 512
  PointPtrBlock raw_points, ground_points, nonground_points;
  autoware::perception::filters::ray_ground_classifier::RayGroundClassifier cls{cfg};
  std::mt19937 gen(1338U);  // Standard mersenne_twister_engine seeded with value
  using unif_distr = std::uniform_real_distribution<float32_t>;
  unif_distr mode_samp{0.0F, 1.0F};
  unif_distr dr_inc_samp{0.1F, 5.0F};
  unif_distr dr_unif_samp{0.0F, 0.2F};
  unif_distr dh_unif_samp{0.1F, 1.0F};
  std::normal_distribution<float32_t> dh_gauss_samp{0.0F, 0.1F};
  auto fn = [&](const uint32_t ray_size) {
      raw_points.clear();
      // generate points using a simple markov chain
      constexpr float32_t ground2vert_prob = 0.1F;
      constexpr float32_t nonground2vert_prob = 0.2F;
      constexpr float32_t vert2nonground_prob = 0.7F;
      constexpr float32_t vert2ground_prob = 0.8F;
      // initialize state variables
      uint32_t mode = 0;  // 0 = ground, 1 = nonground, 2 = vertical structure ground -> nonground
      //                     3 = veritcal structure nonground->ground
      float32_t last_dr = 1.0F;
      PointXYZIF pt;
      std::vector<PointXYZIF> all_points;
      all_points.reserve(static_cast<uint32_t>(POINT_BLOCK_CAPACITY));
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
          pt.z = cfg.m_ground_z_m;
        }
        // check state transition
        const float32_t r = mode_samp(gen);
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
        float32_t dr, dh = 0.0F;
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
        all_points.push_back(pt);
        raw_points.push_back(&all_points.back());
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
