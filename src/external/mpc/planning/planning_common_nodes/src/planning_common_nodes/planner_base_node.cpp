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

#include "planning_common_nodes/planner_base_node.hpp"

#include <time_utils/time_utils.hpp>

#include <memory>
#include <string>
#include <utility>

namespace motion
{
namespace planning
{
namespace planning_common_nodes
{
PlannerBaseNode::PlannerBaseNode(const std::string & name, const std::string & ns)
: Node{name, ns},
  m_environment{planning_common::EnvironmentConfig{
          std::chrono::milliseconds{declare_parameter("max_context_time_error_ms").get<int64_t>()}}}
{
  const auto ego_topic = declare_parameter("ego_topic").get<std::string>();
  const auto target_topic = declare_parameter("target_topic").get<std::string>();
  const auto object_topic = declare_parameter("object_topic").get<std::string>();
  const auto boundary_topic = declare_parameter("boundary_topic").get<std::string>();
  const auto tf_topic = declare_parameter("tf_topic").get<std::string>();
  const auto trajectory_topic = declare_parameter("trajectory_topic").get<std::string>();
  const auto diagnostic_topic = declare_parameter("diagnostic_topic").get<std::string>();
  ContextSource source{};
  const auto source_name = declare_parameter("source").get<std::string>();
  if ("ego" == source_name) {
    source = ContextSource::EGO;
  } else if ("target" == source_name) {
    source = ContextSource::TARGET;
  } else if ("object" == source_name) {
    source = ContextSource::OBJECT;
  } else if ("boundary" == source_name) {
    source = ContextSource::BOUNDARY;
  } else {
    throw std::runtime_error{"Planner source type must be one of: EGO, BOUNDARY, TARGET, OBJECT"};
  }
  init(ego_topic, target_topic, object_topic, boundary_topic, tf_topic,
    trajectory_topic, diagnostic_topic, source);
}

////////////////////////////////////////////////////////////////////////////////
PlannerBaseNode::PlannerBaseNode(
  const std::string & name,
  const std::string & ns,
  const std::string & trajectory_topic,
  const std::string & ego_topic,
  const std::string & target_topic,
  const std::string & object_topic,
  const std::string & boundary_topic,
  const std::string & tf_topic,
  const std::string & diagnostic_topic,
  const ContextSource source,
  const planning_common::EnvironmentConfig & cfg)
: Node{name, ns},
  m_environment{cfg}
{
  init(ego_topic, target_topic, object_topic, boundary_topic, tf_topic,
    trajectory_topic, diagnostic_topic, source);
}

////////////////////////////////////////////////////////////////////////////////
void PlannerBaseNode::init(
  const std::string & ego_topic,
  const std::string & target_topic,
  const std::string & object_topic,
  const std::string & boundary_topic,
  const std::string & tf_topic,
  const std::string & trajectory_topic,
  const std::string & diagnostic_topic,
  ContextSource source)
{
  m_contexts.reserve(2U);
  // Validate source
  switch (source) {
    case ContextSource::EGO:
    case ContextSource::TARGET:
      m_source = source;
      break;
    case ContextSource::OBJECT:
    case ContextSource::BOUNDARY:
      throw std::domain_error{"PlannerBaseNode: Source type not yet implemented!"};
      break;
    default:
      throw std::domain_error{"PlannerBaseNode: Unknown source type!"};
  }
  if (ego_topic.empty()) {
    throw std::domain_error{"Ego state topic not set"};
  }
  if (target_topic.empty()) {
    throw std::domain_error{"Target state topic not set"};
  }
  using rclcpp::QoS;
  // Set up subscribers
  (void)object_topic;
  (void)boundary_topic;
  using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
  m_ego_sub = create_subscription<State>(ego_topic, QoS{10},
      [this](const State::SharedPtr msg) {on_ego(msg);}, SubAllocT{});
  m_target_sub = create_subscription<State>(target_topic, QoS{10},
      [this](const State::SharedPtr msg) {on_target(msg);}, SubAllocT{});
  m_tf_sub = create_subscription<TFMessage>(tf_topic, QoS{10},
      [this](const TFMessage::SharedPtr msg) {on_tf(msg);}, SubAllocT{});
  // Set up publishers
  (void)diagnostic_topic;
  using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
  m_trajectory_pub =
    create_publisher<Trajectory>(trajectory_topic, QoS{10}, PubAllocT{});
}

////////////////////////////////////////////////////////////////////////////////
void PlannerBaseNode::set_planner(PlannerPtr && planner) noexcept
{
  m_planner = std::forward<PlannerPtr &&>(planner);
}

////////////////////////////////////////////////////////////////////////////////
void PlannerBaseNode::on_bad_compute(std::exception_ptr eptr)
{
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

////////////////////////////////////////////////////////////////////////////////
void PlannerBaseNode::publish(const Trajectory & msg)
{
  m_trajectory_pub->publish(msg);
}

////////////////////////////////////////////////////////////////////////////////
void PlannerBaseNode::on_tf(const TFMessage::SharedPtr & msg)
{
  m_environment.add_transform(*msg);
  try_compute();
}

////////////////////////////////////////////////////////////////////////////////
void PlannerBaseNode::on_ego(const State::SharedPtr & msg)
{
  m_environment.add_ego_state(*msg);
  try_compute(ContextSource::EGO, msg->header);
}

////////////////////////////////////////////////////////////////////////////////
void PlannerBaseNode::on_target(const State::SharedPtr & msg)
{
  m_environment.add_target_state(*msg);
  try_compute(ContextSource::TARGET, msg->header);
}

////////////////////////////////////////////////////////////////////////////////
void PlannerBaseNode::try_compute(const ContextSource source, const Header & header)
{
  if (source == m_source) {
    (void)try_compute(header);
  } else {
    try_compute();
  }
}

////////////////////////////////////////////////////////////////////////////////
void PlannerBaseNode::try_compute()
{
  if (!m_contexts.empty()) {
    const auto & header = m_contexts.front();
    if (try_compute(header)) {
      m_environment.clear_before(time_utils::from_message(header.stamp));
      m_contexts.clear();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
bool PlannerBaseNode::try_compute(const std_msgs::msg::Header & header)
{
  if (!m_environment.has_valid_context(header.frame_id, time_utils::from_message(header.stamp))) {
    // Only the latest requested context is valid
    if (!m_contexts.empty()) {
      m_contexts.clear();
    }
    m_contexts.emplace_back(header);
    return false;
  }
  try {
    const auto ctx = m_environment.context(header.frame_id, time_utils::from_message(header.stamp));
    // TODO(c.ho) diagnostics...
    const auto & traj = m_planner->plan(ctx);
    publish(traj);
  } catch (...) {
    on_bad_compute(std::current_exception());
  }
  return true;
}

}  // namespace planning_common_nodes
}  // namespace planning
}  // namespace motion
