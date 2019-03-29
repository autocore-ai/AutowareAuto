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

#ifndef TEST_RAY_GROUND_CLASSIFIER_AUX_HPP_
#define TEST_RAY_GROUND_CLASSIFIER_AUX_HPP_

#include <functional>
#include <vector>
#include <tuple>

#include "ray_ground_classifier/ray_ground_point_classifier.hpp"
#include "ray_ground_classifier/ray_ground_classifier.hpp"

namespace test_ray_ground_classifier
{

using autoware::perception::filters::ray_ground_classifier::Config;
using autoware::perception::filters::ray_ground_classifier::PointBlock;
using autoware::perception::filters::ray_ground_classifier::PointXYZIF;
using autoware::perception::filters::ray_ground_classifier::PointXYZIFR;

class ray_ground_classifier : public ::testing::Test
{
public:
  ray_ground_classifier()
  : cfg{
      0.4F,        // sensor_height_m,
      5.0F,        // max_local_slope_deg,
      3.0F,        // max_global_slope_deg,
      20.0F,       // nonground_retro_thresh_deg,
      0.05F,       // min_height_thresh_m,
      1.5F,        // max_global_height_thresh_m,
      1.8F,        // max_last_local_ground_thresh_m,
      2.0,         // max_provisional_ground_distance_m,
      -1.0F,       // min_height_m,
      1.5F         // max_height_m
  }
  {
  }

protected:
  std::vector<std::tuple<float, float, int>> dat;
  std::vector<PointXYZIF> pts;
  std::vector<bool> labels;
  Config cfg;
};

/// adds num_points ground points to out of form (r, f(r), ground)
void generate_groundspace(
  const float ro,
  const float rf,
  const size_t num_points,
  const std::function<float(float)> & f,
  std::vector<std::tuple<float, float, int>> & out)
{
  const float den = (num_points < 1U) ? 1.0F : static_cast<float>(num_points - 1U);
  const float dr = (rf - ro) / den;
  for (size_t idx = 0U; idx < num_points; ++idx) {
    float r = ro + static_cast<float>(idx) * dr;
    float h = f(r);
    out.push_back(std::make_tuple(r, h, 0));
  }
}

/// convert data format into actual points and labels for processing
void generate_points(
  const Config & cfg,
  const std::vector<std::tuple<float, float, int>> & in,
  std::vector<PointXYZIF> & pts_out,
  std::vector<bool> & label_out)
{
  for (auto & x : in) {
    // height is relative to ground
    PointXYZIF pt;
    pt.x = std::get<0>(x);
    pt.y = 0.0F;
    pt.z = -cfg.get_sensor_height() + std::get<1>(x);
    pt.intensity = 0.0F;
    pt.id = 0U;
    pts_out.push_back(pt);
    // posix: 0  =  ground
    label_out.push_back(std::get<2>(x) == 0);
  }
}

/// do ground filtering and check label
void label_and_check(
  const Config & cfg,
  const std::vector<PointXYZIF> & pts,
  const std::vector<bool> & label,
  const bool ground_only = true,
  const bool do_reverse = true)
{
  ASSERT_EQ(pts.size(), label.size());

  using autoware::perception::filters::ray_ground_classifier::RayGroundClassifier;
  RayGroundClassifier filter(cfg);

  // insert points in reverse order to exercise sorting
  if (do_reverse) {
    for (size_t idx = pts.size() - 1U; idx < pts.size(); --idx) {
      PointXYZIF pt = pts[idx];
      pt.id = idx;
      filter.insert(pt);
    }
  } else {
    for (size_t idx = 0U; idx < pts.size(); ++idx) {
      PointXYZIF pt = pts[idx];
      pt.id = idx;
      filter.insert(pt);
    }
  }

  // filter
  PointBlock ground_blk;
  PointBlock nonground_blk;
  EXPECT_TRUE(filter.can_fit_result(ground_blk, nonground_blk));
  filter.partition(ground_blk, nonground_blk, !do_reverse);

  // check result
  size_t num_wrong = 0U;
  for (const PointXYZIF & pt : ground_blk) {
    if (!label[pt.id]) {  // true <--> ground
      std::cerr << pt.id << " expected " << (label[pt.id] ? "ground" : "nonground") << "\n";
      ++num_wrong;
    }
  }
  for(const PointXYZIF & pt : nonground_blk) {
    if (label[pt.id]) {  // true <--> ground
      std::cerr << pt.id << " expected " << (label[pt.id] ? "ground" : "nonground") << "\n";
      ++num_wrong;
    }
  }
  // if you have ground and nonground, getting one wrong is ok
  const size_t TOL = ground_only ? 0U : 1U;
  ASSERT_LE(num_wrong, TOL);
}

}  // namespace test_ray_ground_classifier

#endif  // TEST_RAY_GROUND_CLASSIFIER_AUX_HPP_
