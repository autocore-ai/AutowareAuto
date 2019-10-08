// Copyright 2017-2018 Apex.AI, Inc.
// All rights reserved.

#include <motion_common/motion_common.hpp>
#include <time_utils/time_utils.hpp>

#include <cmath>
#include <chrono>
#include <utility>
#include <string>
#include <vector>
#include "pure_pursuit_nodes/pure_pursuit_node.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Resources relating to the pure pursuit node package
namespace pure_pursuit_nodes
{
using float32_t = float;
////////////////////////////////////////////////////////////////////////////////
PurePursuitNode::PurePursuitNode(
  const std::string & node_name,
  const std::string & node_namespace)
: Node{node_name, node_namespace},
  m_controller(pure_pursuit::Config{
          static_cast<float32_t>(declare_parameter(
            "controller.minimum_lookahead_distance").get<double>()),
          static_cast<float32_t>(declare_parameter(
            "controller.maximum_lookahead_distance").get<double>()),
          static_cast<float32_t>(
            declare_parameter("controller.speed_to_lookahead_ratio").get<double>()),
          declare_parameter("controller.is_interpolate_lookahead_point").get<bool>(),
          declare_parameter("controller.is_delay_compensation").get<bool>(),
          static_cast<float32_t>(declare_parameter(
            "controller.emergency_stop_distance").get<double>()),
          static_cast<float32_t>(declare_parameter(
            "controller.speed_thres_traveling_direction").get<double>()),
          static_cast<float32_t>(declare_parameter(
            "controller.dist_front_rear_wheels").get<double>())})
{
  init(
    declare_parameter("pose_topic").get<std::string>(),
    declare_parameter("trajectory_topic").get<std::string>(),
    declare_parameter("diagnosis_topic").get<std::string>(),
    declare_parameter("command_topic").get<std::string>());
}
////////////////////////////////////////////////////////////////////////////////
PurePursuitNode::PurePursuitNode(
  const std::string & node_name,
  const std::string & pose_topic,
  const std::string & trajectory_topic,
  const std::string & command_topic,
  const std::string & diagnosis_topic,
  const pure_pursuit::Config & cfg,
  const std::string & node_namespace)
: Node(node_name, node_namespace),
  m_controller(cfg)
{
  init(pose_topic, trajectory_topic, diagnosis_topic, command_topic);
}
////////////////////////////////////////////////////////////////////////////////
void PurePursuitNode::init(
  const std::string & pose_topic,
  const std::string & trajectory_topic,
  const std::string & diagnostic_topic,
  const std::string & command_topic)
{
  // Subscribers
  m_pose_sub_ptr = create_subscription<TrajectoryPointStamped>(
    pose_topic,
    rclcpp::QoS{10},
    [this](const TrajectoryPointStamped::SharedPtr msg) -> void {on_pose(msg);});
  m_traj_sub_ptr = create_subscription<Trajectory>(
    trajectory_topic,
    rclcpp::QoS{10},
    [this](const Trajectory::SharedPtr msg) -> void {on_trajectory(msg);});
  m_tf_sub_ptr = create_subscription<Transforms>(
    "tf",
    rclcpp::QoS{10},
    [this](const Transforms::SharedPtr msg) -> void {on_tf(msg);});
  // Publishers
  m_cmd_pub_ptr = create_publisher<VehicleControlCommand>(command_topic, rclcpp::QoS{10});
  m_diag_pub_ptr = create_publisher<ControllerDiagnostic>(diagnostic_topic, rclcpp::QoS{10});
}
////////////////////////////////////////////////////////////////////////////////
void PurePursuitNode::on_trajectory(const Trajectory::SharedPtr msg)
{
  m_controller.set_trajectory(*msg);
  m_traj_initialized = true;
}
////////////////////////////////////////////////////////////////////////////////
void PurePursuitNode::on_pose(const TrajectoryPointStamped::SharedPtr msg)
{
  function(*msg);
}
////////////////////////////////////////////////////////////////////////////////
void PurePursuitNode::on_tf(const Transforms::SharedPtr msg)
{
  for (const auto & tf : msg->transforms) {
    m_tf.setTransform(tf, "pure_pursuit_node");
  }
}
////////////////////////////////////////////////////////////////////////////////
void PurePursuitNode::function(TrajectoryPointStamped pose)
{
  // Take a trajectory
  const auto now = std::chrono::system_clock::now();
  const auto t_trajectory = time_utils::from_message(m_controller.get_trajectory().header.stamp);
  const auto can_transform = m_tf.canTransform(
    pose.header.frame_id.c_str(),
    m_controller.get_trajectory().header.frame_id.c_str(),
    t_trajectory);
  if (can_transform) {
    const auto pose_to_traj = m_tf.lookupTransform(
      pose.header.frame_id.c_str(),
      m_controller.get_trajectory().header.frame_id.c_str(),
      t_trajectory);
    auto pose_tf = pose;
    ::motion::motion_common::doTransform(pose, pose_tf, pose_to_traj);
    const auto & cmd = m_controller.compute_command(pose_tf);
    m_cmd_pub_ptr->publish(cmd);
    const auto & diagnostic = m_controller.get_diagnostic();
    m_diag_pub_ptr->publish(diagnostic);
  } else {
    // Since the trajectory is taken successfully, TF graph or the trajectory's timestamp
    // has problems. Fatal error.
    RCLCPP_WARN(get_logger(), "PurePursuitNode: could not subscribe the tf topic");
  }
}
}  // namespace pure_pursuit_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware
