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

#include <tracking_test_framework/scene.hpp>

#include <autoware_auto_msgs/msg/bounding_box.hpp>
#include <geometry/bounding_box_2d.hpp>
#include <time_utils/time_utils.hpp>

#include <memory>
#include <vector>
#include <utility>

namespace autoware
{
namespace tracking_test_framework
{

Scene::Scene(const Lidar & lidar, std::vector<std::unique_ptr<TrackedObject>> && objects)
: m_lidar(lidar), m_objects(std::move(objects))
{}

void Scene::move_all_objects(const std::chrono::milliseconds dt_in_ms)
{
  for (auto & object : m_objects) {
    object->move_object(dt_in_ms);
  }
}

std::vector<ObjIntersections> Scene::get_intersections_with_lidar(
  const bool closest_only) const
{
  return m_lidar.get_intersections_per_object(m_objects, closest_only);
}

autoware_auto_msgs::msg::DetectedObjects Scene::get_detected_objects_array(
  const bool closest_only) const
{
  std::vector<ObjIntersections> intersections_all_objects =
    this->get_intersections_with_lidar(closest_only);

  autoware_auto_msgs::msg::DetectedObjects detected_object_msg_array{};
  for (const auto & intersection_per_object : intersections_all_objects) {
    /// Fill Shape with all intersections of LiDAR and each object
    geometry_msgs::msg::Polygon polygon{};
    autoware_auto_msgs::msg::BoundingBox bounding_box{};
    std::vector<autoware::common::types::PointXYZIF> points_vec{};
    /// Loop to convert point represented as Eigen::Vector2f to autoware::common::types::PointXYZIF
    for (const auto & point : intersection_per_object.points) {
      points_vec.push_back(
        autoware::tracking_test_framework::utils::
        get_point_from_vector_2d(point));
    }
    if (intersection_per_object.obj_type == ObjectType::Car) {
      bounding_box = autoware::common::geometry::bounding_box::lfit_bounding_box_2d(
        points_vec.begin(), points_vec.end());
    } else {
      bounding_box = autoware::common::geometry::bounding_box::minimum_area_bounding_box(
        points_vec.begin(), points_vec.end());
    }
    for (size_t isec_idx = 0; isec_idx < bounding_box.corners.size(); ++isec_idx) {
      const auto corner_point = bounding_box.corners[isec_idx];
      geometry_msgs::msg::Point32 pt;
      pt.x = corner_point.x;
      pt.y = corner_point.y;
      pt.z = 0.0F;
      polygon.points.push_back(pt);
    }

    /// Detected Object
    autoware_auto_msgs::msg::DetectedObject detected_object_msg{};
    detected_object_msg.existence_probability = 1.0;
    autoware_auto_msgs::msg::ObjectClassification classification;
    if (intersection_per_object.obj_type == ObjectType::Pedestrian) {
      classification.classification =
        autoware_auto_msgs::msg::ObjectClassification::PEDESTRIAN;
    } else if (intersection_per_object.obj_type == ObjectType::Car) {
      classification.classification =
        autoware_auto_msgs::msg::ObjectClassification::CAR;
    } else {
      classification.classification =
        autoware_auto_msgs::msg::ObjectClassification::UNKNOWN;
    }

    classification.probability = 1.0;
    detected_object_msg.classification.push_back(classification);
    detected_object_msg.shape.polygon = polygon;
    detected_object_msg.shape.height = 1.5;
    detected_object_msg.kinematics.centroid_position.x = bounding_box.centroid.x;
    detected_object_msg.kinematics.centroid_position.y = bounding_box.centroid.y;
    detected_object_msg.kinematics.centroid_position.z = bounding_box.centroid.z;
    detected_object_msg.kinematics.has_position_covariance = false;
    detected_object_msg.kinematics.has_twist = false;
    detected_object_msg.kinematics.has_twist_covariance = false;

    detected_object_msg_array.header.stamp =
      time_utils::to_message(std::chrono::system_clock::now());
    detected_object_msg_array.objects.push_back(detected_object_msg);
  }
  return detected_object_msg_array;
}

}  // namespace tracking_test_framework
}  // namespace autoware
