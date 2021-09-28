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

#include "ground_truth_detections/ground_truth_detections_node.hpp"
#include <autoware_auto_msgs/msg/classified_roi.hpp>
#include <autoware_auto_msgs/msg/detected_object.hpp>
#include <algorithm>

namespace autoware
{
namespace ground_truth_detections
{

constexpr char GroundTruthDetectionsNode::kFrameId2d[];
constexpr char GroundTruthDetectionsNode::kFrameId3d[];

GroundTruthDetectionsNode::GroundTruthDetectionsNode(const rclcpp::NodeOptions & options)
:  Node("ground_truth_detections", options),
  m_detection2d_pub{create_publisher<autoware_auto_msgs::msg::ClassifiedRoiArray>(
      "/perception/ground_truth_detections_2d", rclcpp::QoS{10})},
  m_detection2d_sub{create_subscription<lgsvl_msgs::msg::Detection2DArray>(
      "/simulator/ground_truth/detections2D", rclcpp::QoS{10},
      [this](lgsvl_msgs::msg::Detection2DArray::SharedPtr msg) {on_detection(*msg);}
    )},
  m_detection3d_pub{create_publisher<autoware_auto_msgs::msg::DetectedObjects>(
    "/perception/ground_truth_detections_3d", rclcpp::QoS{10})},
m_detection3d_sub{create_subscription<lgsvl_msgs::msg::Detection3DArray>(
    "/simulator/ground_truth/detections3D", rclcpp::QoS{10},
    [this](lgsvl_msgs::msg::Detection3DArray::SharedPtr msg) {on_detection(*msg);}
  )}
{
}

void GroundTruthDetectionsNode::on_detection(const lgsvl_msgs::msg::Detection2DArray & msg)
{
  autoware_auto_msgs::msg::ClassifiedRoiArray roi_array;
  roi_array.header = msg.header;
  roi_array.header.frame_id = kFrameId2d;

  roi_array.rois.resize(msg.detections.size());
  std::transform(
    msg.detections.begin(), msg.detections.end(), roi_array.rois.begin(),
    [](const lgsvl_msgs::msg::Detection2D & detection) {
      return autoware_auto_msgs::build<autoware_auto_msgs::msg::ClassifiedRoi>()
      .classifications({make_classification(detection.label)})
      .polygon(make_polygon(detection));
    });
  m_detection2d_pub->publish(roi_array);
}

void GroundTruthDetectionsNode::on_detection(const lgsvl_msgs::msg::Detection3DArray & msg)
{
  autoware_auto_msgs::msg::DetectedObjects detected_objects;
  detected_objects.header = msg.header;
  detected_objects.header.frame_id = kFrameId3d;

  detected_objects.objects.resize(msg.detections.size());
  const auto create_detected_object =
    [](const lgsvl_msgs::msg::Detection3D & detection) {
      return autoware_auto_msgs::build<autoware_auto_msgs::msg::DetectedObject>()
             .existence_probability(1.0F)
             .classification({make_classification(detection.label)})
             .kinematics(make_kinematics(detection))
             .shape(make_shape(detection));
    };

  std::transform(
    msg.detections.begin(), msg.detections.end(),
    detected_objects.objects.begin(), create_detected_object);
  m_detection3d_pub->publish(detected_objects);
}
}  // namespace ground_truth_detections
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ground_truth_detections::GroundTruthDetectionsNode)
