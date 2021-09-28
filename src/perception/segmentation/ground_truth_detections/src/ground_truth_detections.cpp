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

#include "ground_truth_detections/ground_truth_detections.hpp"
#include <autoware_auto_msgs/msg/detected_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <algorithm>
#include <cmath>
#include <string>

namespace
{
template<typename T1, typename T2>
T1 convert(const T2 & p);

template<>
geometry_msgs::msg::Point32 convert(const geometry_msgs::msg::Vector3 & p_in)
{
  geometry_msgs::msg::Point32 p_out;
  p_out.x = static_cast<float_t>(p_in.x);
  p_out.y = static_cast<float_t>(p_in.y);
  p_out.z = static_cast<float_t>(p_in.z);
  return p_out;
}

geometry_msgs::msg::Polygon make_polygon_around_origin(
  const lgsvl_msgs::msg::Detection3D & detection)
{
  // Polygon is 2D rectangle
  geometry_msgs::msg::Polygon polygon;
  auto & points = polygon.points;
  points.resize(4);

  // origin of reference system: centroid of bbox
  const auto size = ::convert<geometry_msgs::msg::Point32>(detection.bbox.size);
  points[0].x = -0.5F * size.x;
  points[0].y = -0.5F * size.y;
  points[0].z = 0.0F;

  points[1] = points[0];
  points[1].y += size.y;

  points[2] = points[1];
  points[2].x += size.x;

  points[3] = points[2];
  points[3].y -= size.y;

  return polygon;
}

}  // namespace

namespace autoware
{

namespace ground_truth_detections
{

autoware_auto_msgs::msg::ObjectClassification make_classification(const std::string & label)
{
  autoware_auto_msgs::msg::ObjectClassification obj_classification;
  obj_classification.probability = 1.0;
  auto & classification = obj_classification.classification;

  // default classification
  classification = autoware_auto_msgs::msg::ObjectClassification::UNKNOWN;

  static constexpr const char * car_labels[] = {"Hatchback", "Jeep", "Sedan", "SUV"};
  auto found_car_label = std::find(std::begin(car_labels), std::end(car_labels), label);
  if (found_car_label != std::end(car_labels)) {
    classification = autoware_auto_msgs::msg::ObjectClassification::CAR;
  } else if (label == "BoxTruck") {
    classification = autoware_auto_msgs::msg::ObjectClassification::TRUCK;
  } else if (label == "Pedestrian") {
    classification = autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN;
  }
  return obj_classification;
}

geometry_msgs::msg::Polygon make_polygon(const lgsvl_msgs::msg::Detection2D & detection)
{
  geometry_msgs::msg::Polygon polygon;
  auto & points = polygon.points;
  // implicitly assign 0 to z coordinate
  points.resize(4);
  const float_t width = detection.bbox.width;
  const float_t height = detection.bbox.height;

  // clip coordinates to avoid negative values due to rounding.
  // Can't clip upper bound because image size not known.

  // bbox coordinates (x, y) given at center

  // lower left corner
  points[0].x = std::max(detection.bbox.x - 0.5F * width, 0.0F);
  points[0].y = std::max(detection.bbox.y - 0.5F * height, 0.0F);

  // lower right corner
  points[1] = points[0];
  points[1].x += width;

  // upper right corner
  points[2] = points[1];
  points[2].y += height;

  // upper left corner
  points[3] = points[2];
  points[3].x = std::max(points[3].x - width, 0.0F);

  return polygon;
}

autoware_auto_msgs::msg::DetectedObjectKinematics make_kinematics(
  const lgsvl_msgs::msg::Detection3D & detection)
{
  geometry_msgs::msg::TwistWithCovariance twist;
  twist.twist = detection.velocity;

  return autoware_auto_msgs::build<autoware_auto_msgs::msg::DetectedObjectKinematics>()
         .centroid_position(detection.bbox.position.position)
         .position_covariance({})
         .has_position_covariance(false)
         .orientation(detection.bbox.position.orientation)
         .orientation_availability(autoware_auto_msgs::msg::DetectedObjectKinematics::AVAILABLE)
         .twist(twist)
         .has_twist(true)
         .has_twist_covariance(false);
}

autoware_auto_msgs::msg::Shape make_shape(const lgsvl_msgs::msg::Detection3D & detection)
{
  return autoware_auto_msgs::build<autoware_auto_msgs::msg::Shape>()
         .polygon(make_polygon_around_origin(detection))
         .height(static_cast<float_t>(detection.bbox.size.z));
}

}  // namespace ground_truth_detections
}  // namespace autoware
