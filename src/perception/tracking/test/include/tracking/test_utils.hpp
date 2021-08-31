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

#ifndef TRACKING__TEST_UTILS_HPP_
#define TRACKING__TEST_UTILS_HPP_

#include <tracking/projection.hpp>
#include <autoware_auto_msgs/msg/classified_roi_array.hpp>
#include <autoware_auto_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <vector>

std::vector<geometry_msgs::msg::Point32> expand_shape_to_vector(
  const autoware_auto_msgs::msg::Shape & shape);

void compare_shapes(
  const geometry_msgs::msg::Polygon & prism_face,
  const autoware::perception::tracking::Projection & projection,
  const autoware::perception::tracking::CameraIntrinsics & intrinsics);

geometry_msgs::msg::Point32 make_pt(float32_t x, float32_t y, float32_t z);

autoware_auto_msgs::msg::Shape make_rectangular_shape(
  const geometry_msgs::msg::Point32 & base_face_origin,
  float32_t half_width, float32_t half_length, float32_t shape_height);

autoware_auto_msgs::msg::ClassifiedRoi projection_to_roi(
  const autoware::perception::tracking::Projection & projection);

geometry_msgs::msg::Transform make_identity();

#endif  // TRACKING__TEST_UTILS_HPP_
