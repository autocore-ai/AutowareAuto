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
/// \brief This file defines the ground_truth_detections_node class.

#ifndef GROUND_TRUTH_DETECTIONS__GROUND_TRUTH_DETECTIONS_NODE_HPP_
#define GROUND_TRUTH_DETECTIONS__GROUND_TRUTH_DETECTIONS_NODE_HPP_

#include <ground_truth_detections/ground_truth_detections.hpp>

#include <autoware_auto_msgs/msg/classified_roi_array.hpp>
#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <lgsvl_msgs/msg/detection2_d_array.hpp>
#include <lgsvl_msgs/msg/detection3_d_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace autoware
{
namespace ground_truth_detections
{

/// \brief ROS 2 Node for to convert ground-truth detections from the simulator format to the format
/// needed in Autoware.Auto
class GROUND_TRUTH_DETECTIONS_PUBLIC GroundTruthDetectionsNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit GroundTruthDetectionsNode(const rclcpp::NodeOptions & options);

private:
  /// \brief Callback for input message. Publish output message.
  /// @param msg Input message
  void on_detection(const lgsvl_msgs::msg::Detection2DArray & msg);

  /// \brief Callback for input message. Publish output message.
  /// @param msg Input message
  void on_detection(const lgsvl_msgs::msg::Detection3DArray & msg);

  rclcpp::Publisher<autoware_auto_msgs::msg::ClassifiedRoiArray>::SharedPtr m_detection2d_pub{};
  rclcpp::Subscription<lgsvl_msgs::msg::Detection2DArray>::SharedPtr m_detection2d_sub{};
  std::string m_vision_frame_id;

  rclcpp::Publisher<autoware_auto_msgs::msg::DetectedObjects>::SharedPtr m_detection3d_pub{};
  rclcpp::Subscription<lgsvl_msgs::msg::Detection3DArray>::SharedPtr m_detection3d_sub{};
  static constexpr char kFrameId3d[] = "base_link";
};
}  // namespace ground_truth_detections
}  // namespace autoware

#endif  // GROUND_TRUTH_DETECTIONS__GROUND_TRUTH_DETECTIONS_NODE_HPP_
