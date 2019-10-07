// Copyright 2017-2018 Apex.AI, Inc.
// All rights reserved.

#include <time_utils/time_utils.hpp>

#include <cmath>
#include <chrono>
#include <utility>
#include <string>
#include <vector>
#include "pure_pursuit_nodes/pure_pursuit_node.hpp"
#include "pure_pursuit/heading.hpp"

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
    transform_pose(pose, pose_to_traj.transform);
    const auto & cmd = m_controller.update(pose);
    m_cmd_pub_ptr->publish(cmd);
    const auto & diagnostic = m_controller.get_diagnostic();
    m_diag_pub_ptr->publish(diagnostic);
  } else {
    // Since the trajectory is taken successfully, TF graph or the trajectory's timestamp
    // has problems. Fatal error.
    RCLCPP_WARN(get_logger(), "PurePursuitNode: could not subscribe the tf topic");
  }
}
////////////////////////////////////////////////////////////////////////////////
void PurePursuitNode::transform_pose(
  TrajectoryPointStamped & pose, const Transform & transform) const
{
  // Compute transform TODO(y.tsuji) Ported from the KinematicTracker.
  // Current ApexOS does not support tf based transformation.
  const float32_t a2 = static_cast<float32_t>(transform.rotation.w * transform.rotation.w);
  const float32_t b2 = static_cast<float32_t>(transform.rotation.x * transform.rotation.x);
  const float32_t c2 = static_cast<float32_t>(transform.rotation.y * transform.rotation.y);
  const float32_t d2 = static_cast<float32_t>(transform.rotation.z * transform.rotation.z);
  const float32_t ad = static_cast<float32_t>(transform.rotation.w * transform.rotation.z);
  const float32_t bc = static_cast<float32_t>(transform.rotation.x * transform.rotation.y);
  const float32_t rot_xx = (a2 + b2) - (c2 + d2);
  const float32_t rot_xy = 2.0F * (bc - ad);
  const float32_t rot_x = static_cast<float32_t>(transform.translation.x);
  const float32_t rot_yx = 2.0F * (bc + ad);
  const float32_t rot_yy = (a2 + c2) - (b2 + d2);
  const float32_t rot_y = static_cast<float32_t>(transform.translation.y);
  const float32_t pos_x = pose.state.x;
  pose.state.x = (rot_xx * pos_x) + (rot_xy * pose.state.y) + rot_x;
  pose.state.y = (rot_yx * pos_x) + (rot_yy * pose.state.y) + rot_y;
  {
    // Ensure you're using a normalized 2D quaternion
    autoware_auto_msgs::msg::Complex32 dth;
    const auto s = 1.0F / (std::sqrt(a2 + b2 + c2 + d2));
    dth.real = static_cast<decltype(dth.real)>(transform.rotation.w) * s;
    dth.imag = static_cast<decltype(dth.imag)>(transform.rotation.z) * s;
    pose.state.heading = pure_pursuit::angle_addition(pose.state.heading, dth);
  }
}
}  // namespace pure_pursuit_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware
