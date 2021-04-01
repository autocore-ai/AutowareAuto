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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the class which represents the liDAR sensor

#ifndef TRACKING_TEST_FRAMEWORK__LIDAR_HPP_
#define TRACKING_TEST_FRAMEWORK__LIDAR_HPP_

#include <tracking_test_framework/tracked_object.hpp>

#include <memory>
#include <vector>

namespace autoware
{
namespace tracking_test_framework
{
/// \brief This is the class which has the APIs to create a representation of LiDAR sensor to put
/// on the scene
class TRACKING_TEST_FRAMEWORK_PUBLIC Lidar
{
public:
  /// \brief constructor
  /// \param[in] position current position of the LiDAR
  /// \param[in] num_azimuth_ticks number of ticks in LiDAR scan space between 0 to 2*PI
  /// \param[in] max_range max range of the LiDAR
  Lidar(
    const Eigen::Vector2f & position, const uint32_t num_azimuth_ticks,
    const autoware::common::types::float32_t max_range);

  /// \brief Method to get intersection points with the object clusters and lidar beams
  /// \param[in] objects std::vector holding unique_ptr to the TrackedObjects : Car, Pedestrian
  /// \param[in] closest_only the boolean to determine if closest intersection to be
  /// returned or all
  /// \return returns the intersection points
  std::vector<EigenStlVector<Eigen::Vector2f>> get_intersections_per_object(
    const std::vector<std::unique_ptr<TrackedObject>> & objects,
    const bool closest_only) const;

private:
  /// current position of the LiDAR
  Eigen::Vector2f m_position;
  /// number of ticks in LiDAR scan space between 0 to 2*PI
  uint32_t m_num_azimuth_ticks;
  /// max range of the LiDAR
  autoware::common::types::float32_t m_max_range;
  /// vector of the LiDAR beams
  std::vector<Line> m_beams;

  /// \brief Helper method to get end point of the LiDAR
  /// \param[in] angle angle of the LiDAR beam
  /// \return returns the end point of the LiDAR beam
  Eigen::Vector2f get_beam_end(const autoware::common::types::float32_t angle) const;
};
}  // namespace tracking_test_framework
}  // namespace autoware

#endif  // TRACKING_TEST_FRAMEWORK__LIDAR_HPP_
