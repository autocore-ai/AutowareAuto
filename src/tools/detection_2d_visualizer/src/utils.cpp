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

#include <detection_2d_visualizer/utils.hpp>
#include <vector>
#include <utility>

namespace autoware
{
namespace detection_2d_visualizer
{
void  draw_shape(
  cv_bridge::CvImagePtr & image_ptr, const geometry_msgs::msg::Polygon & polygon,
  const cv::Scalar & color, const std::int32_t thickness)
{
  std::vector<cv::Point> pts;
  constexpr auto is_polyline_closed = true;
  for (const auto & pt : polygon.points) {
    pts.emplace_back(static_cast<std::int32_t>(pt.x), static_cast<std::int32_t>(pt.y));
  }
  cv::polylines(image_ptr->image, pts, is_polyline_closed, color, thickness);
}
}  // namespace detection_2d_visualizer
}  // namespace autoware
