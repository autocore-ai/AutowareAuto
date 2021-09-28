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

#ifndef DETECTION_2D_VISUALIZER__DETECTION_2D_VISUALIZER_NODE_HPP_
#define DETECTION_2D_VISUALIZER__DETECTION_2D_VISUALIZER_NODE_HPP_

#include <autoware_auto_msgs/msg/classified_roi_array.hpp>
#include <detection_2d_visualizer/visibility_control.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>


namespace autoware
{
namespace detection_2d_visualizer
{

/// Class subscribes to CompressedImage and ClassifiedROIArray boxes,
/// then converts it into Image and draws the boxes over the image
class DETECTION_2D_VISUALIZER_PUBLIC Detection2dVisualizerNode : public rclcpp::Node
{
public:
  explicit Detection2dVisualizerNode(const rclcpp::NodeOptions & options);

  /// Convert compressed image to raw image and draw the boxes over the image
  /// \param img_msg CompressedImage
  /// \param roi_msg boxes to draw over the image
  /// \param projection_msg Projections of clusters that correspond with the captured image
  void process(
    sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg,
    autoware_auto_msgs::msg::ClassifiedRoiArray::ConstSharedPtr roi_msg,
    autoware_auto_msgs::msg::ClassifiedRoiArray::ConstSharedPtr projection_msg);

private:
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg
      ::CompressedImage,
      autoware_auto_msgs::msg::ClassifiedRoiArray,
      autoware_auto_msgs::msg::ClassifiedRoiArray>;

  using ExactPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg
      ::CompressedImage,
      autoware_auto_msgs::msg::ClassifiedRoiArray,
      autoware_auto_msgs::msg::ClassifiedRoiArray>;

  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> m_image_sub;
  message_filters::Subscriber<autoware_auto_msgs::msg::ClassifiedRoiArray> m_roi_sub;
  message_filters::Subscriber<autoware_auto_msgs::msg::ClassifiedRoiArray> m_projection_sub;
  std::unique_ptr<message_filters::Synchronizer<ApproximatePolicy>> m_approximate_sync_ptr{};
  std::unique_ptr<message_filters::Synchronizer<ExactPolicy>> m_exact_sync_ptr{};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_pub;
};
}  // namespace detection_2d_visualizer
}  // namespace autoware

#endif  // DETECTION_2D_VISUALIZER__DETECTION_2D_VISUALIZER_NODE_HPP_
