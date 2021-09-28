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
/// \brief This file defines the ground_truth_detections class.

#ifndef GROUND_TRUTH_DETECTIONS__GROUND_TRUTH_DETECTIONS_HPP_
#define GROUND_TRUTH_DETECTIONS__GROUND_TRUTH_DETECTIONS_HPP_

#include <ground_truth_detections/visibility_control.hpp>

#include <autoware_auto_msgs/msg/classified_roi_array.hpp>
#include <autoware_auto_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_auto_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <lgsvl_msgs/msg/detection2_d_array.hpp>
#include <lgsvl_msgs/msg/detection3_d_array.hpp>
#include <string>

namespace autoware
{
namespace ground_truth_detections
{

/**
 * @brief Make classification with class known with certainty.
 *
 * The `label` is mapped to an `ObjectClassification` value if known, else mapped to `UNKNOWN`. The
 * full mapping is given in the design doc.
 *
 * @param label The label of an detection assumed to come from SVL
 * @return the classification
 */
autoware_auto_msgs::msg::ObjectClassification GROUND_TRUTH_DETECTIONS_PUBLIC make_classification(
  const std::string & label);

/**
 * @brief Convert 2D bounding box from LGSVL format of center point, width, and height to a
 * polygon with four points in image coordinates.
 *
 * @param detection The 2D input detection
 * @return the output polygon. The first point is the lower left corner, the second point is the lower right corner etc.
 */
geometry_msgs::msg::Polygon GROUND_TRUTH_DETECTIONS_PUBLIC make_polygon(
  const lgsvl_msgs::msg::Detection2D & detection);

/**
 * @brief Make kinematics from a 3D detection from SVL
 *
 * @param detection The 3D input detection
 */
autoware_auto_msgs::msg::DetectedObjectKinematics GROUND_TRUTH_DETECTIONS_PUBLIC make_kinematics(
  const lgsvl_msgs::msg::Detection3D & detection);

/**
 * @brief Make a (2D + height) shape from a 3D detection from SVL
 *
 * The four points of the shape's polygon represent the bottom of the detection. The first point is
 * the rear left corner, the second point is the rear right corner, the third point is the front
 * right corner, and the fourth point is the front left corner.
 *
 * The z coordinate of all four points is identical and set to the minimum value of all points after
 * rotating and translating in 3D. This assumes the detection is made in a frame whose z axis is
 * gravity aligned.
 *
 * @param detection The 3D input detection
 */
autoware_auto_msgs::msg::Shape GROUND_TRUTH_DETECTIONS_PUBLIC make_shape(
  const lgsvl_msgs::msg::Detection3D & detection);

}  // namespace ground_truth_detections
}  // namespace autoware

#endif  // GROUND_TRUTH_DETECTIONS__GROUND_TRUTH_DETECTIONS_HPP_
