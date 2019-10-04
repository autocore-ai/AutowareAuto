/// \copyright Copyright 2017-2018 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief This file defines the pure pursuit node

#ifndef PURE_PURSUIT_NODES__PURE_PURSUIT_NODE_HPP_
#define PURE_PURSUIT_NODES__PURE_PURSUIT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/control_diagnostic.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/buffer_core.h>
#include <pure_pursuit/pure_pursuit.hpp>
#include <string>
#include <vector>
#include "pure_pursuit_nodes/visibility_control.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Boilerplate Apex.OS nodes around pure_pursuit
namespace pure_pursuit_nodes
{
/// \brief Boilerplate node that subscribes to the current pose and
/// publishes a vehicle control command
class PURE_PURSUIT_NODES_PUBLIC PurePursuitNode : public rclcpp::Node
{
public:
  /// \brief Parameter constructor
  /// \param[in] node_name Name of the node, controls which parameter set from the file is matched
  /// \param[in] node_namespace Name of the node's namespace, controls which parameters are used
  PurePursuitNode(
    const std::string & node_name,
    const std::string & node_namespace = "");

  /// \brief Explicit constructor
  /// \param[in] node_name Name of the node
  /// \param[in] pose_topic Name of input pose topic
  /// \param[in] trajectory_topic Name of input trajectory topic
  /// \param[in] command_topic Name of output control command topic
  /// \param[in] diagnosis_topic Name of output diagnosis topic
  /// \param[in] cfg Configuration object for PurePursuit
  /// \param[in] node_namespace Namespace of this node
  PurePursuitNode(
    const std::string & node_name,
    const std::string & pose_topic,
    const std::string & trajectory_topic,
    const std::string & command_topic,
    const std::string & diagnosis_topic,
    const pure_pursuit::Config & cfg,
    const std::string & node_namespace = "");

private:
  using Trajectory = autoware_auto_msgs::msg::Trajectory;
  using TrajectoryPointStamped = autoware_auto_msgs::msg::VehicleKinematicState;
  using ControllerDiagnostic = autoware_auto_msgs::msg::ControlDiagnostic;
  using VehicleControlCommand = autoware_auto_msgs::msg::VehicleControlCommand;
  using Transform = geometry_msgs::msg::Transform;
  using Transforms = tf2_msgs::msg::TFMessage;

  PURE_PURSUIT_NODES_LOCAL void init(
    const std::string & pose_topic,
    const std::string & trajectory_topic,
    const std::string & diagnostic_topic,
    const std::string & command_topic);
  PURE_PURSUIT_NODES_LOCAL void on_trajectory(const Trajectory::SharedPtr msg);
  PURE_PURSUIT_NODES_LOCAL void on_pose(const TrajectoryPointStamped::SharedPtr msg);
  PURE_PURSUIT_NODES_LOCAL void on_tf(const Transforms::SharedPtr msg);

  /// \brief Core run loop
  void function(TrajectoryPointStamped pose);
  /// \brief Transform the current pose by the given transformation
  /// \param[inout] pose The current vehicle pose
  /// \param[in] transform The transformation from the source frame to the target frame
  PURE_PURSUIT_NODES_LOCAL
  void transform_pose(
    TrajectoryPointStamped & pose, const Transform & transform) const;

  pure_pursuit::PurePursuit m_controller;
  tf2::BufferCore m_tf{};

  rclcpp::Subscription<TrajectoryPointStamped>::SharedPtr m_pose_sub_ptr{};
  rclcpp::Subscription<Trajectory>::SharedPtr m_traj_sub_ptr{};
  rclcpp::Subscription<Transforms>::SharedPtr m_tf_sub_ptr{};
  typename rclcpp::Publisher<VehicleControlCommand>::SharedPtr m_cmd_pub_ptr{};
  typename rclcpp::Publisher<ControllerDiagnostic>::SharedPtr m_diag_pub_ptr{};

  bool m_traj_initialized{false};
};
}  // namespace pure_pursuit_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // PURE_PURSUIT_NODES__PURE_PURSUIT_NODE_HPP_
