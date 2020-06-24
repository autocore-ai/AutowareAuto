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
#ifndef PLANNING_COMMON_NODES__PLANNER_BASE_NODE_HPP_
#define PLANNING_COMMON_NODES__PLANNER_BASE_NODE_HPP_

#include <planning_common_nodes/visibility_control.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <planning_common/planner_base.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>
#include <string>
#include <vector>


namespace motion
{
namespace planning
{
namespace planning_common_nodes
{
using planning_common::State;
using planning_common::Trajectory;
using tf2_msgs::msg::TFMessage;
using PlannerPtr = std::unique_ptr<planning_common::PlannerBase>;

/// Denotes which topic is used to trigger computation and/or be the frame/stamp for the context
/// lookup
enum class ContextSource
{
  EGO,
  TARGET,
  OBJECT,
  BOUNDARY
};  // enum class ContextSource

class PLANNING_COMMON_NODES_PUBLIC PlannerBaseNode : public rclcpp::Node
{
public:
  /// Parameter file constructor
  PlannerBaseNode(const std::string & name, const std::string & ns);
  /// Explicit constructor
  PlannerBaseNode(
    const std::string & name,
    const std::string & ns,
    const std::string & trajectory_topic,
    const std::string & ego_topic,
    const std::string & target_topic,
    const std::string & object_topic,
    const std::string & boundary_topic,
    const std::string & tf_topic,
    const std::string & diagnostic_topic,
    ContextSource source,
    const planning_common::EnvironmentConfig & cfg);

  virtual ~PlannerBaseNode() noexcept = default;

protected:
  /// Child class should call this to set the planning
  void set_planner(PlannerPtr && planner) noexcept;
  /// Handles errors thrown by compute_command_impl(), or std::domain_error due to
  /// empty trajectories
  virtual void on_bad_compute(std::exception_ptr eptr);
  /// Expose publishing in case a child class wants to do something during error handling
  void publish(const Trajectory & msg);

private:
  using Header = std_msgs::msg::Header;
  // Common initialization
  PLANNING_COMMON_NODES_LOCAL void init(
    const std::string & ego_topic,
    const std::string & target_topic,
    const std::string & object_topic,
    const std::string & boundary_topic,
    const std::string & tf_topic,
    const std::string & trajectory_topic,
    const std::string & diagnostic_topic,
    ContextSource source);
  // Callbacks, note passing smart pointers by ref is fine if you're not using ownership
  // semantics:
  // stackoverflow.com/questions/3310737/
  // should-we-pass-a-shared-ptr-by-reference-or-by-value/8741626
  PLANNING_COMMON_NODES_LOCAL void on_tf(const TFMessage::SharedPtr & msg);
  PLANNING_COMMON_NODES_LOCAL void on_ego(const State::SharedPtr & msg);
  PLANNING_COMMON_NODES_LOCAL void on_target(const State::SharedPtr & msg);
  // Main computation, false if failure (due to incomplete context?)
  PLANNING_COMMON_NODES_LOCAL bool try_compute(const Header & header);
  // Check if there's a lingering context that hasn't been computed and compute
  PLANNING_COMMON_NODES_LOCAL void try_compute();
  // Try to directly compute if set source matches given source, otherwise try to compute with
  // cached frame
  PLANNING_COMMON_NODES_LOCAL void try_compute(ContextSource source, const Header & header);

  rclcpp::Subscription<State>::SharedPtr m_ego_sub{};
  rclcpp::Subscription<State>::SharedPtr m_target_sub{};
  rclcpp::Subscription<TFMessage>::SharedPtr m_tf_sub{};
  rclcpp::Publisher<Trajectory>::SharedPtr m_trajectory_pub{};
  planning_common::PlanningEnvironment m_environment;
  PlannerPtr m_planner{nullptr};
  ContextSource m_source{};
  // Would prefer std::optional, but I have to use C++14; unique_ptr is semantically more correct,
  // but allocates
  std::vector<Header> m_contexts{};
};  // class PlannerBaseNode
}  // namespace planning_common_nodes
}  // namespace planning
}  // namespace motion

#endif  // PLANNING_COMMON_NODES__PLANNER_BASE_NODE_HPP_
