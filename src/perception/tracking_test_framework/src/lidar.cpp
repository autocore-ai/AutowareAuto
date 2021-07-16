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

#include <tracking_test_framework/lidar.hpp>

#include <algorithm>
#include <memory>
#include <vector>

namespace autoware
{
namespace tracking_test_framework
{

Lidar::Lidar(
  const Eigen::Vector2f & position, const uint32_t num_azimuth_ticks,
  const autoware::common::types::float32_t max_range)
: m_position(position),
  m_num_azimuth_ticks(num_azimuth_ticks), m_max_range(max_range)
{
  std::vector<autoware::common::types::float32_t> angles =
    utils::linspace<autoware::common::types::float32_t>(
    0.0F, 2.0F * autoware::common::types::PI,
    num_azimuth_ticks);

  for (const auto angle : angles) {
    m_beams.emplace_back(Line{m_position, get_beam_end(angle)});
  }
}

std::vector<ObjIntersections> Lidar::get_intersections_per_object(
  const std::vector<std::unique_ptr<TrackedObject>> & objects, const bool closest_only) const
{
  std::vector<ObjIntersections> intersections{};
  for (const auto & beam : m_beams) {
    std::vector<ObjIntersections> intersections_all_objects{};
    for (const auto & object : objects) {
      ObjIntersections intersections_current;
      intersections_current.points = object->intersect_with_line(beam, closest_only);
      intersections_current.obj_type = object->object_type();
      if (!intersections_current.points.empty()) {
        intersections_all_objects.push_back(intersections_current);
      }
    }

    if (!intersections_all_objects.empty()) {
      if (closest_only) {
        intersections.resize(1);
        // prune all but nearest point
        sort(
          intersections_all_objects.begin(), intersections_all_objects.end(),
          [&](const ObjIntersections & isec_a, const ObjIntersections & isec_b) {
            return (isec_a.points[0] - m_position).norm() < (isec_b.points[0] - m_position).norm();
          });
        intersections[0].obj_type = intersections_all_objects[0].obj_type;
        intersections[0].points.push_back(intersections_all_objects[0].points[0]);
      } else {
        intersections.resize(intersections_all_objects.size());
        for (size_t isec_idx = 0; isec_idx < intersections_all_objects.size(); ++isec_idx) {
          intersections[isec_idx].obj_type = intersections_all_objects[isec_idx].obj_type;
          intersections[isec_idx].points.insert(
            intersections[isec_idx].points.end(), intersections_all_objects[isec_idx].points
            .begin(), intersections_all_objects[isec_idx].points.end());
        }
      }
    }
  }
  return intersections;
}

Eigen::Vector2f Lidar::get_beam_end(const autoware::common::types::float32_t angle) const
{
  const Eigen::Rotation2D<autoware::common::types::float32_t> rotation(angle);
  const Eigen::Matrix2f rotation_matrix = rotation.toRotationMatrix();
  return this->m_position + rotation_matrix * Eigen::Vector2f{this->m_max_range, 0.0F};
}


}  // namespace tracking_test_framework
}  // namespace autoware
