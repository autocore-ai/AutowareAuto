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

#include <tracking_test_framework/scene.hpp>

#include <time_utils/time_utils.hpp>

#include <memory>
#include <vector>

namespace autoware
{
namespace tracking_test_framework
{

Scene::Scene(const Lidar & lidar, const std::vector<std::unique_ptr<TrackedObject>> & objects)
: m_lidar(lidar)
{
  m_objects.reserve(objects.size());
  for (const auto & object : objects) {
    m_objects.emplace_back(object->clone());
  }
}

void Scene::move_all_objects(const std::chrono::milliseconds dt_in_ms)
{
  for (auto & object : m_objects) {
    object->move_object(dt_in_ms);
  }
}

std::vector<EigenStlVector<Eigen::Vector2f>> Scene::get_intersections_with_lidar(
  const bool
  closest_only) const
{
  return m_lidar.get_intersections_per_object(m_objects, closest_only);
}

autoware_auto_msgs::msg::DetectedDynamicObjectArray Scene::get_detected_objects_array(
  const bool closest_only) const
{
  std::vector<EigenStlVector<Eigen::Vector2f>> intersections_all_objects =
    this->get_intersections_with_lidar(closest_only);

  autoware_auto_msgs::msg::DetectedDynamicObjectArray detected_object_msg_array{};
  for (const auto & intersection_per_object : intersections_all_objects) {
    /// Fill Shape with all intersections of LiDAR and each object
    autoware_auto_msgs::msg::Shape shape_msg{};
    geometry_msgs::msg::Polygon polygon{};
    polygon.points.resize(intersection_per_object.size());
    for (size_t isec_idx = 0; isec_idx < intersection_per_object.size(); ++isec_idx) {
      const auto & intersection = intersection_per_object[isec_idx];
      polygon.points[isec_idx].x = intersection.x();
      polygon.points[isec_idx].y = intersection.y();
      polygon.points[isec_idx].z = 0.0F;
    }
    shape_msg.polygon = polygon;
    shape_msg.height = 0.0;

    /// Detected Object
    autoware_auto_msgs::msg::DetectedDynamicObject detected_object_msg{};
    detected_object_msg.existence_probability = 1.0;
    detected_object_msg.classification.resize(1);
    detected_object_msg.classification.front().classification =
      autoware_auto_msgs::msg::ObjectClassification::UNKNOWN;
    detected_object_msg.classification.front().probability = 1.0;
    detected_object_msg.shape = shape_msg;
    detected_object_msg.kinematics.has_pose = false;
    detected_object_msg.kinematics.has_pose_covariance = false;
    detected_object_msg.kinematics.has_twist = false;
    detected_object_msg.kinematics.has_twist_covariance = false;

    detected_object_msg_array.header.stamp =
      time_utils::to_message(std::chrono::system_clock::now());
    detected_object_msg_array.objects.emplace_back(detected_object_msg);
  }
  return detected_object_msg_array;
}

}  // namespace tracking_test_framework
}  // namespace autoware
