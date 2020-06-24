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
#include "planning_common/planning_context.hpp"
#include <motion_common/motion_common.hpp>
#include <time_utils/time_utils.hpp>

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>

namespace motion
{
namespace planning
{
namespace planning_common
{
EnvironmentConfig::EnvironmentConfig(std::chrono::nanoseconds max_allowable_time_error)
: m_max_allowable_time_error{max_allowable_time_error}
{
  if (max_allowable_time_error < std::chrono::nanoseconds::zero()) {
    throw std::domain_error{"EnvironmentConfig: max error cannot be negative"};
  }
}

std::chrono::nanoseconds EnvironmentConfig::max_allowable_time_error() const noexcept
{
  return m_max_allowable_time_error;
}
////////////////////////////////////////////////////////////////////////////////
PlanningContext::Environment::Environment(const EnvironmentConfig & cfg)
: m_config{cfg}
{
}

const EnvironmentConfig & PlanningContext::Environment::config() const noexcept
{
  return m_config;
}

bool PlanningContext::Environment::check_duration(const std::chrono::nanoseconds duration) const
{
  return duration <= config().max_allowable_time_error();
}

////////////////////////////////////////////////////////////////////////////////
void PlanningContext::Environment::add_ego_state(const State & state)
{
  const auto t = time_utils::from_message(state.header.stamp);
  const auto it = find_newer_than(m_ego_states, t);
  m_ego_states.insert(it, state);
}
void PlanningContext::Environment::add_ego_state(State && state)
{
  const auto t = time_utils::from_message(state.header.stamp);
  const auto it = find_newer_than(m_ego_states, t);
  m_ego_states.emplace(it, state);
}
////////////////////////////////////////////////////////////////////////////////
void PlanningContext::Environment::add_target_state(const State & state)
{
  const auto t = time_utils::from_message(state.header.stamp);
  const auto it = find_newer_than(m_target_states, t);
  m_target_states.insert(it, state);
}
void PlanningContext::Environment::add_target_state(State && state)
{
  const auto t = time_utils::from_message(state.header.stamp);
  const auto it = find_newer_than(m_target_states, t);
  m_target_states.emplace(it, state);
}
////////////////////////////////////////////////////////////////////////////////
void PlanningContext::Environment::add_transform(const Transforms & tfs)
{
  for (const auto & tf : tfs.transforms) {
    add_transform(tf);
  }
}
void PlanningContext::Environment::add_transform(Transform && tf)
{
  // Kick time point precision down: for compatibility with clang
  tf.header.stamp.nanosec -= (tf.header.stamp.nanosec % 1000U);
  m_tf_buffer.setTransform(tf, "foo");
}
void PlanningContext::Environment::add_transform(const Transform & tf)
{
  // Kick time point precision down: for compatibility with clang
  // Horrible hack...
  auto tf_copy = tf;
  tf_copy.header.stamp.nanosec -= (tf.header.stamp.nanosec % 1000U);
  m_tf_buffer.setTransform(tf_copy, "foo");
}

////////////////////////////////////////////////////////////////////////////////
void PlanningContext::Environment::clear_before(std::chrono::system_clock::time_point stamp)
{
  const auto clear_fn = [stamp](auto & buff) {
      const auto it = find_newer_than(buff, stamp);
      buff.erase(buff.begin(), it);
    };
  clear_fn(m_ego_states);
  clear_fn(m_target_states);
}

////////////////////////////////////////////////////////////////////////////////
bool PlanningContext::Environment::frame_ok(
  const std::string & target_frame,
  const std::string & current_frame,
  Clock::time_point stamp) const
{
  if (target_frame == current_frame) {
    return true;
  }
  return m_tf_buffer.canTransform(target_frame, current_frame, stamp);
}

////////////////////////////////////////////////////////////////////////////////
bool PlanningContext::Environment::has_valid_context(
  const std::string & frame_id,
  Clock::time_point stamp) const
{
  // Kick time point precision down: for compatibility with clang
  stamp = std::chrono::time_point_cast<std::chrono::microseconds>(stamp);
  // Something is empty--no context possible
  if (m_target_states.empty()) {
    return false;
  }
  // No target match
  const auto target_state = get_target_state(stamp);
  if (nullptr == target_state) {
    return false;
  }
  // Can't transform target
  if (!frame_ok(target_state->header.frame_id, frame_id, stamp)) {
    return false;
  }
  // Fall back to ego check
  return can_interpolate_ego(frame_id, stamp);
}

////////////////////////////////////////////////////////////////////////////////
PlanningContext PlanningContext::Environment::context(
  const std::string & frame_id,
  Clock::time_point stamp) const
{
  // Kick time point precision down: for compatibility with clang
  stamp = std::chrono::time_point_cast<std::chrono::microseconds>(stamp);
  const auto target_state = get_target_state(stamp);
  if (nullptr == target_state) {
    throw std::domain_error{"PlanningEnvironment: Nearest time stamp for discrete environmental"
            "object is out of range"};
  }
  const auto target_state_tf = transform_state(*target_state, frame_id, stamp);
  // Get ego states for interpolation
  const auto ego_interp = interpolated_ego(frame_id, stamp);

  return PlanningContext{ego_interp, target_state_tf};
}

////////////////////////////////////////////////////////////////////////////////
const State * PlanningContext::Environment::get_target_state(const Clock::time_point stamp) const
{
  if (m_target_states.empty()) {
    throw std::domain_error{"PlanningEnvironment: Cannot get context; no targets!"};
  }
  // Find iterator (right-hand) adjacent to given time stamp
  const auto target_it = find_newer_than(m_target_states, stamp);
  // If closest is last
  if (m_target_states.end() == target_it) {
    const auto last_it = std::prev(target_it);
    const auto dt_last = stamp - time_utils::from_message(last_it->header.stamp);
    return check_duration(dt_last) ? &(*last_it) : nullptr;
  }
  const auto dt_next = time_utils::from_message(target_it->header.stamp) - stamp;
  // If oldest is newer than requested time
  if (m_target_states.begin() == target_it) {
    return check_duration(dt_next) ? &(*target_it) : nullptr;
  }
  // In between: Try to get the nearest target state
  const auto last_it = std::prev(target_it);
  const auto dt_last = stamp - time_utils::from_message(last_it->header.stamp);
  if (dt_next < dt_last) {
    return check_duration(dt_next) ? &(*target_it) : nullptr;
  }
  return check_duration(dt_last) ? &(*last_it) : nullptr;
}

////////////////////////////////////////////////////////////////////////////////
State PlanningContext::Environment::transform_state(
  const State & in,
  const std::string & frame,
  const Clock::time_point stamp) const
{
  if (in.header.frame_id == frame) {
    return in;
  }
  if (!m_tf_buffer.canTransform(in.header.frame_id, frame, stamp)) {
    throw std::domain_error{"PlanningEnvironment: Can't transform state"};
  }
  const auto target_tf = m_tf_buffer.lookupTransform(frame, in.header.frame_id, stamp);
  State ret{};
  motion_common::doTransform(in, ret, target_tf);
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
bool PlanningContext::Environment::can_interpolate_ego(
  const std::string & frame,
  const Clock::time_point stamp) const
{
  if (m_ego_states.empty()) {
    return false;
  }
  const auto ego_it = find_newer_than(m_ego_states, stamp);
  if (ego_it == m_ego_states.end()) {
    return false;
  }
  const auto stamp_next = time_utils::from_message(ego_it->header.stamp);
  // If exact match, no need for interpolation
  if (stamp_next == stamp) {
    return frame_ok(ego_it->header.frame_id, frame, stamp_next);
  }
  // Nothing to interpolate with
  if (m_ego_states.begin() == ego_it) {
    if (stamp_next > stamp) {
      return false;
    }
    return frame_ok(ego_it->header.frame_id, frame, stamp_next);
  }
  // Get previous, put in same frame
  const auto last_it = std::prev(ego_it);
  const auto stamp_last = time_utils::from_message(last_it->header.stamp);
  // check last transform
  return frame_ok(ego_it->header.frame_id, frame, stamp_next) &&
         frame_ok(last_it->header.frame_id, frame, stamp_last);
}
////////////////////////////////////////////////////////////////////////////////
State PlanningContext::Environment::interpolated_ego(
  const std::string & frame,
  const Clock::time_point stamp) const
{
  if (m_ego_states.empty()) {
    throw std::domain_error{"PlanningEnvironment: Cannot get context; no ego states!"};
  }
  const auto ego_it = find_newer_than(m_ego_states, stamp);
  if (ego_it == m_ego_states.end()) {
    throw std::domain_error{"PlanningEnvironment: Requested stamp newer than ego history"};
  }
  const auto stamp_next = time_utils::from_message(ego_it->header.stamp);
  // If exact match, no need for interpolation
  if (stamp_next == stamp) {
    return transform_state(*ego_it, frame, stamp);
  }
  // Nothing to interpolate with
  if (m_ego_states.begin() == ego_it) {
    if (stamp_next > stamp) {
      throw std::domain_error{"PlanningEnvironment: Requested stamp older than ego history"};
    }
    // TODO(c.ho) Warn? Throw?
    return transform_state(*ego_it, frame, stamp);
  }
  // Get previous, put in same frame
  const auto last_it = std::prev(ego_it);
  const auto stamp_last = time_utils::from_message(last_it->header.stamp);
  // Interpolate
  const auto dt = stamp - stamp_last;
  const auto dt_ = stamp_next - stamp_last;
  using Duration = std::chrono::duration<float>;
  const auto del = std::chrono::duration_cast<Duration>(dt).count() /
    std::chrono::duration_cast<Duration>(dt_).count();
  const auto ego_last_tf = transform_state(*last_it, frame, stamp);
  const auto ego_next_tf = transform_state(*ego_it, frame, stamp);
  auto ret = ego_last_tf;
  ret.state = motion_common::interpolate(ego_last_tf.state, ego_next_tf.state, del);
  ret.header.stamp = time_utils::to_message(stamp);
  return ret;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
PlanningContext::PlanningContext(State ego_state, State target_state)
: m_ego_state{std::move(ego_state)},
  m_target_state{std::move(target_state)}
{
}
////////////////////////////////////////////////////////////////////////////////
State PlanningContext::ego_state() const noexcept
{
  return m_ego_state;
}
State PlanningContext::target_state() const noexcept
{
  return m_target_state;
}
}  // namespace planning_common
}  // namespace planning
}  // namespace motion
