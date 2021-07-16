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
/// \brief This file defines the class which represents the scene to put the TrackedObjects and
/// the sensors

#ifndef TRACKING_TEST_FRAMEWORK__SCENE_HPP_
#define TRACKING_TEST_FRAMEWORK__SCENE_HPP_

#include <tracking_test_framework/lidar.hpp>

#include <autoware_auto_msgs/msg/detected_objects.hpp>

#include <chrono>
#include <memory>
#include <vector>

namespace autoware
{
namespace tracking_test_framework
{
/// \brief This is the class which has the APIs to create a representation of Scene
class TRACKING_TEST_FRAMEWORK_PUBLIC Scene
{
public:
  /// \brief constructor
  /// \param[in] lidar object representing the liDAR sensor
  /// \param[in] objects std::vector of TrackedObjects put in the Scene
  Scene(const Lidar & lidar, std::vector<std::unique_ptr<TrackedObject>> && objects);

  /// \brief Method to move objects in the scene
  /// \param[in] dt_in_ms time interval in milliseconds
  void move_all_objects(const std::chrono::milliseconds dt_in_ms);

  /// \brief Method to get DetectedObjects of the TrackedObjects in the Scene with LiDAR
  /// \param[in] closest_only the boolean to determine if closest intersection to be
  /// returned or all
  /// \return returns the DetectedObjects filled with the TrackedObjects information in
  /// the scene
  autoware_auto_msgs::msg::DetectedObjects get_detected_objects_array(
    const bool closest_only) const;

private:
  /// \brief Method to get intersection points of the TrackedObject put in the Scene with LiDAR
  /// \param[in] closest_only the boolean to determine if closest intersection to be
  /// returned or all
  /// \return returns the intersection points of the TrackedObjects in Scene with LiDAR
  std::vector<ObjIntersections> get_intersections_with_lidar(
    const bool closest_only) const;

  /// Object representing the LiDAR sensor
  Lidar m_lidar;
  /// Vector of TrackedObjects put in the Scene
  std::vector<std::unique_ptr<TrackedObject>> m_objects;
};
}  // namespace tracking_test_framework
}  // namespace autoware

#endif  // TRACKING_TEST_FRAMEWORK__SCENE_HPP_
