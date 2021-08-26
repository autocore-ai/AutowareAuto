// Copyright 2021 Apex.AI, Inc.
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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <autoware_auto_msgs/msg/shape.hpp>
#include <geometry/common_2d.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <gtest/gtest.h>
#include <tracking/test_utils.hpp>
#include <vector>

using autoware::common::geometry::point_adapter::x_;
using autoware::common::geometry::point_adapter::y_;
using autoware::common::geometry::point_adapter::z_;
using CameraIntrinsics = autoware::perception::tracking::CameraIntrinsics;

std::vector<geometry_msgs::msg::Point32> expand_shape_to_vector(
  const autoware_auto_msgs::msg::Shape & shape)
{
  std::vector<geometry_msgs::msg::Point32> result{shape.polygon.points};
  const auto num_corners = shape.polygon.points.size();
  for (auto i = 0U; i < num_corners; ++i) {
    auto pt = shape.polygon.points[i];
    result.push_back(pt.set__z(pt.z + shape.height));
  }
  return result;
}

void compare_shapes(
  const geometry_msgs::msg::Polygon & prism_face,
  const autoware::perception::tracking::Projection & projection,
  const CameraIntrinsics & intrinsics)
{
  for (auto i = 0U; i < prism_face.points.size(); ++i) {
    const auto & pt_3d = prism_face.points[i];
    const auto expected_projected_x =
      (pt_3d.x * intrinsics.fx + pt_3d.y * intrinsics.skew + pt_3d.z * intrinsics.ox) / pt_3d.z;
    const auto expected_projected_y =
      (pt_3d.y * intrinsics.fy + pt_3d.z * intrinsics.oy) / pt_3d.z;

    const auto projected_pt_it = std::find_if(
      projection.shape.begin(), projection.shape.end(),
      [expected_projected_x, expected_projected_y](const auto & pt) {
        // TODO(#1241): Investigate the numerical instability that causes large
        //  floating point errors
        constexpr auto eps = 1e-5F;
        return autoware::common::helper_functions::comparisons::abs_eq(
          (x_(pt) - expected_projected_x), 0.0F, eps) &&
        autoware::common::helper_functions::comparisons::abs_eq(
          (y_(pt) - expected_projected_y), 0.0F, eps);
      });
    EXPECT_NE(projected_pt_it, projection.shape.end());
  }
}
