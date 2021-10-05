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

#include <detection_2d_visualizer/detection_2d_visualizer_node.hpp>
#include <detection_2d_visualizer/utils.hpp>
#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <utility>
#include <vector>

namespace autoware
{
namespace detection_2d_visualizer
{
Detection2dVisualizerNode::Detection2dVisualizerNode(const rclcpp::NodeOptions & options)
:  Node("ground_truth_visualizer", options),
  m_image_sub{this, "/simulator/main_camera"},
  m_roi_sub(this, "/perception/ground_truth_detections_2d"),
  m_projection_sub(this, "/projections"),
  m_image_pub{create_publisher<sensor_msgs::msg::Image>("/image_with_detections", rclcpp::QoS{20})}
{
  auto initialize = [this](auto & sync_ptr) {
      sync_ptr->registerCallback(
        std::bind(
          &Detection2dVisualizerNode::process, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    };
  if (declare_parameter("sync_approximately", true)) {
    m_approximate_sync_ptr = std::make_unique<message_filters::Synchronizer<ApproximatePolicy>>(
      ApproximatePolicy(50), m_image_sub, m_roi_sub, m_projection_sub);
    initialize(m_approximate_sync_ptr);
  } else {
    m_exact_sync_ptr = std::make_unique<message_filters::Synchronizer<ExactPolicy>>(
      ExactPolicy(50), m_image_sub, m_roi_sub, m_projection_sub);
    initialize(m_exact_sync_ptr);
  }
}

void Detection2dVisualizerNode::process(
  sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg,
  autoware_auto_msgs::msg::ClassifiedRoiArray::ConstSharedPtr roi_msg,
  autoware_auto_msgs::msg::ClassifiedRoiArray::ConstSharedPtr projection_msg)
{
  static const cv::Scalar ground_truth_color{0, 255, 0};
  static const cv::Scalar projection_color{0, 0, 255};
  constexpr std::int32_t thickness = 5;
  cv_bridge::CvImagePtr cv_img_ptr;
  try {
    cv_img_ptr = cv_bridge::toCvCopy(img_msg);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  for (const auto & rect : roi_msg->rois) {
    draw_shape(cv_img_ptr, rect.polygon, ground_truth_color, thickness);
  }

  for (const auto & rect : projection_msg->rois) {
    draw_shape(cv_img_ptr, rect.polygon, projection_color, thickness);
  }

  m_image_pub->publish(*(cv_img_ptr->toImageMsg()));
}

}  // namespace detection_2d_visualizer
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::detection_2d_visualizer::Detection2dVisualizerNode)
