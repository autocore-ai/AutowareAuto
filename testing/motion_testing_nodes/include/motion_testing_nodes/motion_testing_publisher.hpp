// Copyright 2019 Christopher Ho
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
#ifndef MOTION_TESTING_NODES__MOTION_TESTING_PUBLISHER_HPP_
#define MOTION_TESTING_NODES__MOTION_TESTING_PUBLISHER_HPP_

#include <motion_testing_nodes/visibility_control.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <list>
#include <string>
#include <vector>

namespace motion
{
namespace motion_testing_nodes
{
using State = autoware_auto_msgs::msg::VehicleKinematicState;
using TFMessage = tf2_msgs::msg::TFMessage;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using TransformStamped = geometry_msgs::msg::TransformStamped;

/// Characterizes one part/profile of the test sequence
/// Only the position of the first start_state is respected, other ones are
/// inferred from the dynamics
struct MOTION_TESTING_NODES_PUBLIC TrajectoryProfile
{
  State start_state;
  std::chrono::nanoseconds active_time;
  std::chrono::nanoseconds trajectory_sample_period;
  std::chrono::nanoseconds state_update_period;
  std::string trajectory_frame;
  std::string ego_frame;
};  // struct TrajectoryProfile

using TrajectoryProfilesVec = std::vector<TrajectoryProfile>;

/// Generate a sequence of constant trajectories and states assuming
/// the vehicle perfectly follows the trajectory. States are in the
/// ego frame, trajectories are in an inertial frame, and transforms
/// between the two are provided
class MOTION_TESTING_NODES_PUBLIC MotionTestingPublisher : public rclcpp::Node
{
public:
  MotionTestingPublisher(
    const std::string & node_name,
    const std::string & traj_topic,
    const std::string & state_topic,
    const std::string & tf_topic,
    const TrajectoryProfilesVec & profiles);

  /// Check if all messages have been published
  bool done() const noexcept;

  /// Blocks until all match conditions are satisfied, then published initial trajectory (semihack)
  void match(
    const std::size_t tf_subs = 1U,
    const std::size_t state_subs = 1U,
    const std::size_t traj_subs = 1U) const;

private:
  using Header = std_msgs::msg::Header;
  /// Main worker function
  MOTION_TESTING_NODES_LOCAL void on_timer();
  // Initialization helper functions
  MOTION_TESTING_NODES_LOCAL
  std::chrono::nanoseconds min_period(const TrajectoryProfilesVec & profiles) const noexcept;
  MOTION_TESTING_NODES_LOCAL
  std::size_t end_index(const TrajectoryProfile & prof, const Trajectory & traj) const noexcept;
  MOTION_TESTING_NODES_LOCAL void add_state_and_tf(
    const Header & header,
    const TrajectoryProfile & prof,
    const TrajectoryPoint & pt) noexcept;

  rclcpp::Publisher<TFMessage>::SharedPtr m_tf_pub;
  rclcpp::Publisher<State>::SharedPtr m_state_pub;
  rclcpp::Publisher<Trajectory>::SharedPtr m_traj_pub;
  rclcpp::TimerBase::SharedPtr m_timer;
  std::list<State> m_states;
  std::list<Trajectory> m_trajectories;
  std::list<TFMessage> m_tfs;
  std::chrono::system_clock::time_point m_last_traj_time;
  std::chrono::system_clock::time_point m_last_state_time;
  std::list<std::chrono::nanoseconds> m_state_period;
  std::list<std::chrono::nanoseconds> m_traj_period;
};  // class MotionTestingPublisher
}  // namespace motion_testing_nodes
}  // namespace motion

#endif  // MOTION_TESTING_NODES__MOTION_TESTING_PUBLISHER_HPP_
