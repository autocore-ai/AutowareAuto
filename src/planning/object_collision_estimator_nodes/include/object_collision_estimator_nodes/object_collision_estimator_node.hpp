// Copyright 2020-2021 Arm Limited
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
#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

#include "object_collision_estimator/object_collision_estimator.hpp"
#include "object_collision_estimator_nodes/visibility_control.hpp"

#ifndef OBJECT_COLLISION_ESTIMATOR_NODES__OBJECT_COLLISION_ESTIMATOR_NODE_HPP_
#define OBJECT_COLLISION_ESTIMATOR_NODES__OBJECT_COLLISION_ESTIMATOR_NODE_HPP_

namespace motion
{
namespace planning
{
namespace object_collision_estimator_nodes
{

using motion::planning::object_collision_estimator::ObjectCollisionEstimator;
using autoware_auto_msgs::msg::BoundingBoxArray;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;


/// \brief ROS2 interface to Object Collision Estimator Library. It has 2 main interfaces:
///        1. Subscribe to a topic to obtain obstacle bounding boxes from perception stack.
///        2. Present as a service to other nodes. This service takes a trajectory as an input and
///           outputs a modified trajectory avoiding collisions.
class OBJECT_COLLISION_ESTIMATOR_NODES_PUBLIC ObjectCollisionEstimatorNode : public rclcpp::Node
{
public:
  /// \brief Construct a new Object Collision Estimator Node object
  /// \param[in] node_options Node options for this node.
  explicit ObjectCollisionEstimatorNode(const rclcpp::NodeOptions & node_options);

private:
  /// \brief Callback function for the service interface
  /// \param[in] request The input to the service. Contains the planned trajectory from the
  ///                    behaviour planner.
  /// \param[out] response The output of the service. Contains the trajectory modified by the
  ///                      collision estimator to avoid any collisions.
  void estimate_collision(
    const std::shared_ptr<autoware_auto_msgs::srv::ModifyTrajectory::Request> request,
    std::shared_ptr<autoware_auto_msgs::srv::ModifyTrajectory::Response> response);

  /// \brief Pointer to the subscriber listening for a list of obstacles
  rclcpp::Subscription<BoundingBoxArray>::SharedPtr m_obstacles_sub{nullptr};

  /// \brief Pointer to the publisher for bounding boxes of the target trajectory
  rclcpp::Publisher<MarkerArray>::SharedPtr m_trajectory_bbox_pub{nullptr};

  /// \brief Helper function to handle modified bounding boxes when updating the obstacles.
  /// \param[in] bbox_array An array of bounding boxes representing a list of obstacles
  void update_obstacles(const BoundingBoxArray & bbox_array);

  /// \brief Callback function for the obstacles topic
  /// \param[in] msg ROS2 message from the obstacle topic containing an array of bounding boxes
  ///                representing obstacles found by the perception pipeline.
  void on_bounding_box(const BoundingBoxArray::SharedPtr & msg);

  /// \brief Pointer to an instance of object collision estimator. It performs the main task of
  ///        estimating collisions and modifying the trajectory.
  std::unique_ptr<ObjectCollisionEstimator> m_estimator{nullptr};

  /// \brief The frame id of the map frame. Trajectories are assumed in this frame. Obstacles are
  ///        transformed into this frame before storing. Configured through the `target_frame` node
  ///        parameter.
  std::string m_target_frame_id{};

  /// \brief Pointer to the service interface of the service that estimates collisions.
  rclcpp::Service<autoware_auto_msgs::srv::ModifyTrajectory>::SharedPtr
    m_service_interface{nullptr};

  /// \brief Pointer to the tf interface used to transform obstacle bounding box coordinates.
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;

  /// \brief Pointer to the tf listener which listens for transforms published in the system and
  ///        stores them in the node for use.
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

  /// \brief Pointer to the wall timer used to periodically check if transforms have become
  ///        available.
  rclcpp::TimerBase::SharedPtr m_wall_timer{nullptr};

  /// \brief The time stamp of the last obstacle message received by the node
  rclcpp::Time m_last_obstacle_msg_time {0, 0, RCL_ROS_TIME};

  /// \brief The staleness threshold for objects in milliseconds
  std::chrono::milliseconds m_staleness_threshold_ms{};

  /// \brief Hard coded node name
  static constexpr const char * OBJECT_COLLISION_ESTIMATOR_NODE_NAME =
    "object_collision_estimator_node";

  /// \brief Hard coded topic name on which obstacle bounding boxes are received.
  static constexpr const char * OBSTACLE_TOPIC = "obstacle_topic";
};

}  // namespace object_collision_estimator_nodes
}  // namespace planning
}  // namespace motion

#endif  // OBJECT_COLLISION_ESTIMATOR_NODES__OBJECT_COLLISION_ESTIMATOR_NODE_HPP_
