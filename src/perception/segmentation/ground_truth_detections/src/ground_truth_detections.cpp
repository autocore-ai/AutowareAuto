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

#include <algorithm>

namespace autoware
{

namespace ground_truth_detections
{

autoware_auto_msgs::msg::ObjectClassification make_classification(
  const lgsvl_msgs::msg::Detection2D & detection)
{
  autoware_auto_msgs::msg::ObjectClassification obj_classification;
  obj_classification.probability = 1.0;

  static constexpr const char * car_labels[] = {"Hatchback", "Jeep", "Sedan", "SUV"};
  auto & classification = obj_classification.classification;
  if (std::find(
      std::begin(car_labels), std::end(car_labels),
      detection.label) != std::end(car_labels))
  {
    classification = autoware_auto_msgs::msg::ObjectClassification::CAR;
  } else if (detection.label == "BoxTruck") {
    classification = autoware_auto_msgs::msg::ObjectClassification::TRUCK;
  } else if (detection.label == "Pedestrian") {
    classification = autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN;
  } else {
    classification = autoware_auto_msgs::msg::ObjectClassification::UNKNOWN;
  }
  return obj_classification;
}

geometry_msgs::msg::Polygon make_polygon(const lgsvl_msgs::msg::Detection2D & detection)
{
  geometry_msgs::msg::Polygon polygon;
  auto & points = polygon.points;
  // implicitly assign 0 to z coordinate
  points.resize(4);
  const float width = detection.bbox.width;
  const float height = detection.bbox.height;

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

}  // namespace ground_truth_detections
}  // namespace autoware
