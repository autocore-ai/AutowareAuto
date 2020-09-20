// Copyright 2020 Arm Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <autoware_auto_msgs/srv/modify_trajectory.hpp>
#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <motion_common/config.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <common/types.hpp>
#include <string>
#include <memory>

#include "object_collision_estimator_node/object_collision_estimator_node.hpp"
#include "object_collision_estimator/object_collision_estimator.hpp"

namespace motion
{
namespace planning
{
namespace object_collision_estimator_node
{

using motion::planning::object_collision_estimator::ObjectCollisionEstimatorConfig;
using motion::planning::object_collision_estimator::ObjectCollisionEstimator;
using motion::planning::object_collision_estimator::TrajectorySmootherConfig;
using motion::planning::object_collision_estimator::TrajectorySmoother;
using motion::motion_common::VehicleConfig;
using motion::motion_common::Real;
using autoware::common::types::float64_t;
using autoware::common::types::float32_t;
using rclcpp::QoS;

ObjectCollisionEstimatorNode::ObjectCollisionEstimatorNode(const rclcpp::NodeOptions & node_options)
: Node{OBJECT_COLLISION_ESTIMATOR_NODE_NAME, node_options}
{
  // Declare node parameters. See ObjectCollisionEstimator Class for details of the functions of
  // these parameters.
  const VehicleConfig vehicle_param{
    static_cast<Real>(declare_parameter(
      "vehicle.cg_to_front_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.cg_to_rear_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.front_corner_stiffness"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.rear_corner_stiffness"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.mass_kg"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.yaw_inertia_kgm2"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.width_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.front_overhang_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.rear_overhang_m"
    ).get<float32_t>())
  };

  const auto safety_factor =
    static_cast<float32_t>(declare_parameter(
      "safety_factor"
    ).get<float32_t>());
  const TrajectorySmootherConfig smoother_config {
    static_cast<float32_t>(declare_parameter(
      "trajectory_smoother.kernel_std"
    ).get<float32_t>()),
    static_cast<uint32_t>(declare_parameter(
      "trajectory_smoother.kernel_size"
    ).get<uint32_t>())
  };

  // the tf frame in which planned local trajectories are published
  m_target_frame_id =
    static_cast<std::string>(declare_parameter(
      "target_frame_id"
    ).get<std::string>());

  // Create an object collision estimator
  const ObjectCollisionEstimatorConfig config {vehicle_param, safety_factor};
  const TrajectorySmoother smoother{smoother_config};
  m_estimator = std::make_unique<ObjectCollisionEstimator>(config, smoother);

  // Set up service interface for collision_detection
  m_service_interface = create_service<autoware_auto_msgs::srv::ModifyTrajectory>(
    "estimate_collision",
    [this](const std::shared_ptr<autoware_auto_msgs::srv::ModifyTrajectory::Request> request,
    std::shared_ptr<autoware_auto_msgs::srv::ModifyTrajectory::Response> response) {
      estimate_collision(request, response);
    });

  // Create subscriber and subscribe to the obstacles topic
  m_obstacles_sub = Node::create_subscription<BoundingBoxArray>(
    OBSTACLE_TOPIC, QoS{10},
    [this](const BoundingBoxArray::SharedPtr msg) {this->on_bounding_box(msg);});

  // Create a tf interface to perform transforms on obstacle bounding boxes
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
}

void ObjectCollisionEstimatorNode::on_bounding_box(const BoundingBoxArray::SharedPtr & msg)
{
  // Update most recent bounding boxes internally
  if (msg->header.frame_id == m_target_frame_id) {
    // No transform needed, update bounding boxes directly
    m_estimator->updateObstacles(*msg);
  } else {
    // Transform the coordinates into map frame
    tf2::Duration timeout = tf2::durationFromSec(0.2);
    if (m_tf_buffer->canTransform(
        m_target_frame_id, msg->header.frame_id,
        tf2_ros::fromMsg(msg->header.stamp), timeout) )
    {
      auto msg_tansformed = m_tf_buffer->transform(*msg, m_target_frame_id, timeout);
      m_estimator->updateObstacles(msg_tansformed);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "on_bounding_box cannot transform %s to %s",
        msg->header.frame_id.c_str(), m_target_frame_id.c_str());
    }
  }
}

void ObjectCollisionEstimatorNode::estimate_collision(
  const std::shared_ptr<autoware_auto_msgs::srv::ModifyTrajectory::Request> request,
  std::shared_ptr<autoware_auto_msgs::srv::ModifyTrajectory::Response> response)
{
  // copy the input trajectory into the output variable
  response->modified_trajectory = request->original_trajectory;

  // m_estimator performs the collision estimation and the trajectory will get updated inside
  m_estimator->updatePlan(response->modified_trajectory);
}

}  // namespace object_collision_estimator_node
}  // namespace planning
}  // namespace motion
