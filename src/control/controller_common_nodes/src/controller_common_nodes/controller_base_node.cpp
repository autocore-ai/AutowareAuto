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

#include "controller_common_nodes/controller_base_node.hpp"
#include <motion_common/motion_common.hpp>
#include <time_utils/time_utils.hpp>

#include <exception>
#include <memory>
#include <string>
#include <utility>

namespace motion
{
namespace control
{
namespace controller_common_nodes
{
////////////////////////////////////////////////////////////////////////////////
ControllerBaseNode::ControllerBaseNode(const std::string & name, const std::string & ns)
: Node{name, ns, rclcpp::NodeOptions{rcl_get_default_allocator()}}
{
  using rcl_interfaces::msg::ParameterDescriptor;
  using rclcpp::ParameterValue;
  using std::string;
  init(
    declare_parameter("command_topic", ParameterValue{""}, ParameterDescriptor{}).get<string>(),
    declare_parameter("state_topic", ParameterValue{""}, ParameterDescriptor{}).get<string>(),
    declare_parameter("tf_topic", ParameterValue{""}, ParameterDescriptor{}).get<string>(),
    declare_parameter("static_tf_topic", ParameterValue{""}, ParameterDescriptor{}).get<string>(),
    declare_parameter("trajectory_topic", ParameterValue{""}, ParameterDescriptor{}).get<string>(),
    declare_parameter("diagnostic_topic", ParameterValue{""}, ParameterDescriptor{}).get<string>());
}

////////////////////////////////////////////////////////////////////////////////
ControllerBaseNode::ControllerBaseNode(
  const std::string & name,
  const std::string & ns,
  const std::string & command_topic,
  const std::string & state_topic,
  const std::string & tf_topic,
  const std::string & trajectory_topic,
  const std::string & diagnostic_topic,
  const std::string & static_tf_topic)
: Node{name, ns, rclcpp::NodeOptions{rcl_get_default_allocator()}}
{
  init(command_topic, state_topic, tf_topic, static_tf_topic, trajectory_topic, diagnostic_topic);
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::init(
  const std::string & command_topic,
  const std::string & state_topic,
  const std::string & tf_topic,
  const std::string & static_tf_topic,
  const std::string & trajectory_topic,
  const std::string & diagnostic_topic)
{
  // Common error checking
  if (command_topic.empty()) {
    throw std::domain_error{"Command topic not set"};
  }
  if (state_topic.empty()) {
    throw std::domain_error{"State topic not set"};
  }
  if (tf_topic.empty()) {
    throw std::domain_error{"TF topic not set"};
  }
  if (static_tf_topic.empty()) {
    throw std::domain_error{"Static TF topic not set"};
  }
  if (trajectory_topic.empty()) {
    throw std::domain_error{"Trajectory topic not set"};
  }
  using rclcpp::QoS;
  // Subs
  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  // TODO(#825): Remove ifdef
  // Since Foxy, static transforms are not published periodically but instead with
  // a StaticBroadcasterQoS which has transient_local durability. If the subscriber
  // isn't also transient_local, it will often miss the static transform message.
  const QoS static_tf_qos = QoS{10}.transient_local();
  m_state_sub = create_subscription<State>(
    state_topic, QoS{10},
    [this](const State::SharedPtr msg) {on_state(msg);}, SubAllocT{});
  m_trajectory_sub = create_subscription<Trajectory>(
    trajectory_topic, QoS{10},
    [this](const Trajectory::SharedPtr msg) {on_trajectory(msg);}, SubAllocT{});
  m_tf_sub = create_subscription<TFMessage>(
    tf_topic, QoS{10},
    [this](const TFMessage::SharedPtr msg) {on_tf(msg);}, SubAllocT{});
  m_static_tf_sub = create_subscription<TFMessage>(
    static_tf_topic, static_tf_qos,
    [this](const TFMessage::SharedPtr msg) {on_static_tf(msg);}, SubAllocT{});
  // Pubs
  using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
  m_command_pub = create_publisher<Command>(command_topic, QoS{10}, PubAllocT{});
  // Diagnostics are not strictly needed for proper running
  if (!diagnostic_topic.empty()) {
    m_diagnostic_pub = create_publisher<Diagnostic>(diagnostic_topic, QoS{10}, PubAllocT{});
  }
}
////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::retry_compute()
{
  // Really inelegant mechanism, but that's callbacks, need to maintain throughput
  while (!m_uncomputed_states.empty()) {
    const auto & state = m_uncomputed_states.front();
    if (!try_compute(state)) {
      break;
    }
    m_uncomputed_states.pop_front();
  }
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::on_tf(const TFMessage::SharedPtr & msg)
{
  for (const auto & tf : msg->transforms) {
    if (!m_tf_buffer.setTransform(tf, "external", false)) {
      RCLCPP_WARN(get_logger(), "Warning: tf2::BufferCore::setTransform failed");
    }
  }
  retry_compute();
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::on_static_tf(const TFMessage::SharedPtr & msg)
{
  for (const auto & tf : msg->transforms) {
    if (!m_tf_buffer.setTransform(tf, "external", true)) {
      RCLCPP_WARN(get_logger(), "Warning: tf2::BufferCore::setTransform failed (static)");
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::on_trajectory(const Trajectory::SharedPtr & msg)
{
  try {
    m_controller->set_trajectory(*msg);
    // Only retry computation if new trajectory was successfully set
    retry_compute();
  } catch (...) {
    on_bad_trajectory(std::current_exception());
  }
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::on_state(const State::SharedPtr & msg)
{
  if (!try_compute(*msg)) {
    m_uncomputed_states.push_back(*msg);
  }
}
////////////////////////////////////////////////////////////////////////////////
bool ControllerBaseNode::try_compute(const State & state)
{
  if (state.header.frame_id.empty()) {
    RCLCPP_WARN(get_logger(), "try_compute: empty state frame, ignoring");
    return true;
  }
  if (m_controller->get_reference_trajectory().header.frame_id.empty()) {
    // TODO(Takamasa Horibe): Enable with RCLCPP_WARN_THROTTLE after Foxy
    // RCLCPP_WARN(
    //   get_logger(),
    //   "try_compute: empty trajectory frame, possibly uninitialized, deferring");
    return false;
  }
  // TODO(c.ho) these should honestly be two functions
  // Transform state into same frame as trajectory
  const auto traj_frame = m_controller->get_reference_trajectory().header.frame_id;
  const auto state_frame = state.header.frame_id;
  const auto stamp = time_utils::from_message(state.header.stamp);

  geometry_msgs::msg::TransformStamped tf;
  if (m_tf_buffer.canTransform(traj_frame, state_frame, stamp)) {
    tf = m_tf_buffer.lookupTransform(traj_frame, state_frame, stamp);
  } else if (m_tf_buffer.canTransform(traj_frame, state_frame, tf2::TimePointZero)) {
    tf = m_tf_buffer.lookupTransform(traj_frame, state_frame, tf2::TimePointZero);
  } else {
    return false;
  }

  auto state_tf = state;
  motion_common::doTransform(state, state_tf, tf);
  // Diagnostic stuff: should maybe be different functions
  const auto start = std::chrono::system_clock::now();
  Diagnostic diag;
  if (m_diagnostic_pub) {
    diag.diag_header.computation_start = time_utils::to_message(start);
    if (m_controller->get_reference_trajectory().points.empty()) {
      diag.new_trajectory = false;
    } else {
      const auto ref_idx = m_controller->get_base_config().is_temporal_reference() ?
        m_controller->get_current_state_temporal_index() :
        m_controller->get_current_state_spatial_index();
      diag.new_trajectory = ref_idx == decltype(ref_idx) {};
    }
  }
  const auto diagnostic_fn = [this, start, &state, &diag]() -> void {
      if (m_diagnostic_pub) {
        diag.diag_header.runtime = time_utils::to_message(std::chrono::system_clock::now() - start);
        compute_diagnostic(
          *m_controller,
          state,
          m_controller->get_base_config().is_temporal_reference(),
          diag);
        m_diagnostic_pub->publish(diag);
      }
    };
  // Compute result
  try {
    const auto cmd{m_controller->compute_command(state_tf)};
    publish(cmd);
    diagnostic_fn();
  } catch (...) {
    diagnostic_fn();
    on_bad_compute(std::current_exception());
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::publish(const Command & msg)
{
  m_command_pub->publish(msg);
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::set_controller(ControllerPtr && controller) noexcept
{
  m_controller = std::forward<ControllerPtr &&>(controller);
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::on_bad_trajectory(std::exception_ptr eptr)  // NOLINT
{
  // Pass-by-reference is generally inappropriate for pointer types
  try {
    if (eptr) {
      std::rethrow_exception(eptr);
    } else {
      RCLCPP_WARN(get_logger(), "on_bad_trajectory: nullptr");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), e.what());
  }
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBaseNode::on_bad_compute(std::exception_ptr eptr)  // NOLINT
{
  // Pass-by-reference is generally inappropriate for pointer types
  try {
    if (eptr) {
      std::rethrow_exception(eptr);
    } else {
      RCLCPP_WARN(get_logger(), "on_bad_compute: nullptr");
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), e.what());
  }
}

}  // namespace controller_common_nodes
}  // namespace control
}  // namespace motion
