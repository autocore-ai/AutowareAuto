// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the ground_truth_visualizer_node class.

#ifndef DETECTION_2D_VISUALIZER__UTILS_HPP_
#define DETECTION_2D_VISUALIZER__UTILS_HPP_

#include <cv_bridge/cv_bridge.h>
#include <detection_2d_visualizer/visibility_control.hpp>
#include <geometry_msgs/msg/polygon.hpp>

namespace autoware
{
namespace detection_2d_visualizer
{

void DETECTION_2D_VISUALIZER_PUBLIC draw_shape(
  cv_bridge::CvImagePtr & image_ptr, const geometry_msgs::msg::Polygon & polygon,
  const cv::Scalar & color, std::int32_t thickness);
}  // namespace detection_2d_visualizer
}  // namespace autoware

#endif  // DETECTION_2D_VISUALIZER__UTILS_HPP_
