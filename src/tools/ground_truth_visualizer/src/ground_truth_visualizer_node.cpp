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

#include "ground_truth_visualizer/ground_truth_visualizer_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <utility>
#include <vector>

namespace autoware
{
namespace ground_truth_visualizer
{

GroundTruthVisualizerNode::GroundTruthVisualizerNode(const rclcpp::NodeOptions & options)
:  Node("ground_truth_visualizer", options),
  m_image_sub{this, "/simulator/main_camera"},
  m_roi_sub(this, "/perception/ground_truth_detections_2d"),
  m_sync_ptr(std::make_unique<message_filters::Synchronizer<Policy>>(
      Policy(50), m_image_sub,
      m_roi_sub)),
  m_image_pub{create_publisher<sensor_msgs::msg::Image>("/image_with_detections", rclcpp::QoS{10})}
{
  m_sync_ptr->registerCallback(
    std::bind(
      &GroundTruthVisualizerNode::process, this,
      std::placeholders::_1, std::placeholders::_2));
}

void GroundTruthVisualizerNode::process(
  sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg,
  autoware_auto_msgs::msg::ClassifiedRoiArray::ConstSharedPtr roi_msg)
{
  const bool is_polyline_closed = true;
  const std::int32_t thickness = 5;
  const cv::Scalar color{0, 255, 0};
  cv_bridge::CvImagePtr cv_img_ptr{};
  try {
    cv_img_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  if (!cv_img_ptr) {return;}
  for (const auto & rect : roi_msg->rois) {
    std::vector<cv::Point> pts;
    for (const auto & pt : rect.polygon.points) {
      pts.emplace_back(static_cast<std::int32_t>(pt.x), static_cast<std::int32_t>(pt.y));
    }
    cv::polylines(cv_img_ptr->image, pts, is_polyline_closed, color, thickness);
  }
  m_image_pub->publish(*(cv_img_ptr->toImageMsg()));
}

}  // namespace ground_truth_visualizer
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ground_truth_visualizer::GroundTruthVisualizerNode)
