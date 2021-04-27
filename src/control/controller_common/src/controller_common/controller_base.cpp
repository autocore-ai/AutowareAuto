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

#include "controller_common/controller_base.hpp"

#include <time_utils/time_utils.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace motion
{
namespace control
{
namespace controller_common
{
////////////////////////////////////////////////////////////////////////////////
BehaviorConfig::BehaviorConfig(
  const Real safe_deceleration_rate_mps2,
  const std::chrono::nanoseconds time_step,
  const ControlReference type)
: m_safe_deceleration_rate_mps2{safe_deceleration_rate_mps2},
  m_time_step{time_step},
  m_is_spatial_reference{ControlReference::SPATIAL == type}
{
  if (safe_deceleration_rate_mps2 <= 0.0F) {
    throw std::domain_error{"Safe deceleration rate must be positive"};
  }
}

Real BehaviorConfig::safe_deceleration_rate() const noexcept
{
  return m_safe_deceleration_rate_mps2;
}
std::chrono::nanoseconds BehaviorConfig::time_step() const noexcept
{
  return m_time_step;
}
bool BehaviorConfig::is_spatial_reference() const noexcept
{
  return m_is_spatial_reference;
}
bool BehaviorConfig::is_temporal_reference() const noexcept
{
  return !m_is_spatial_reference;
}

////////////////////////////////////////////////////////////////////////////////
ControllerBase::ControllerBase(const BehaviorConfig & config)
: m_config{config},
  m_reference_trajectory{rosidl_runtime_cpp::MessageInitialization::ALL},
  m_latest_reference{decltype(m_latest_reference)::min()},
  m_reference_spatial_index{},  // zero initialization
  m_reference_temporal_index{}  // zero initialization
{
  m_reference_trajectory.points.reserve(100LL);  // TODO(c.ho) +1?
  m_reference_trajectory.points.clear();
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBase::set_trajectory_impl()
{
  m_latest_reference = time_utils::from_message(m_reference_trajectory.header.stamp);
  if (!m_reference_trajectory.points.empty()) {
    m_latest_reference +=
      time_utils::from_message(m_reference_trajectory.points.back().time_from_start);
  }
  m_reference_spatial_index = 0U;
  m_reference_temporal_index = 0U;
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBase::set_trajectory(const Trajectory & trajectory)
{
  if (!check_new_trajectory(trajectory)) {
    throw std::domain_error{"ControllerBase: Trajectory not as expected"};
  }
  if (trajectory.points.empty()) {
    throw std::domain_error{"ControllerBase: Zero length trajectory is not expected"};
  }
  // Return value elision should hit for at most a single copy (a warning told me not to move)
  m_reference_trajectory = handle_new_trajectory(trajectory);
  // This happens _after_ because of dependence on state; handle_new_trajectory might mutate
  // trajectory
  set_trajectory_impl();
}

////////////////////////////////////////////////////////////////////////////////
const Trajectory & ControllerBase::get_reference_trajectory() const noexcept
{
  return m_reference_trajectory;
}

////////////////////////////////////////////////////////////////////////////////
Command ControllerBase::compute_command(const State & state)
{
  if (m_reference_trajectory.points.empty()) {
    return compute_stop_command(state);
  }
  if (state.header.frame_id != m_reference_trajectory.header.frame_id) {
    throw std::domain_error{"Vehicle state is not in same frame as reference trajectory"};
  }
  update_reference_indices(state);
  if (!is_state_ok(state)) {
    return compute_stop_command(state);
  }
  auto ret = compute_command_impl(state);
  // Ensure you're properly populating the stamp
  ret.stamp = state.header.stamp;
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
bool ControllerBase::check_new_trajectory(const Trajectory & trajectory) const
{
  // noop
  (void)trajectory;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
const Trajectory & ControllerBase::handle_new_trajectory(const Trajectory & trajectory)
{
  return trajectory;
}

////////////////////////////////////////////////////////////////////////////////
Index ControllerBase::get_current_state_spatial_index() const
{
  if (m_reference_trajectory.points.empty()) {
    throw std::domain_error{"Cannot get reference state index: no or zero length trajectory"};
  }
  return m_reference_spatial_index;
}

////////////////////////////////////////////////////////////////////////////////
Index ControllerBase::get_current_state_temporal_index() const
{
  if (m_reference_trajectory.points.empty()) {
    throw std::domain_error{"Cannot get reference state index: no or zero length trajectory"};
  }
  return m_reference_temporal_index;
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBase::set_base_config(const BehaviorConfig & config) noexcept
{
  m_config = config;
}

////////////////////////////////////////////////////////////////////////////////
const BehaviorConfig & ControllerBase::get_base_config() const noexcept
{
  return m_config;
}

////////////////////////////////////////////////////////////////////////////////
bool ControllerBase::is_state_ok(const State & state) const noexcept
{
  const auto pose_ok = !is_past_trajectory(state);
  const auto t = time_utils::from_message(state.header.stamp);
  const auto time_ok = t < m_latest_reference;
  return pose_ok && time_ok;
}

////////////////////////////////////////////////////////////////////////////////
void ControllerBase::update_reference_indices(const State & new_state) noexcept
{
  // Update spatial index
  const auto space_f =
    [](const State & state, const Point & pt1, const Point & pt2, const Trajectory &) -> bool {
      return motion_common::is_past_point(state.state, pt1, pt2);
    };
  using motion_common::update_reference_index;
  m_reference_spatial_index =
    update_reference_index(m_reference_trajectory, new_state, m_reference_spatial_index, space_f);
  // Update temporal index
  const auto time_f =
    [](const State & state, const Point &, const Point & pt, const Trajectory & traj) -> bool {
      using time_utils::from_message;
      const auto t_s = from_message(state.header.stamp);
      const auto t_t = from_message(traj.header.stamp);
      const auto dt = from_message(pt.time_from_start);
      return t_s >= (t_t + dt);
    };
  m_reference_temporal_index =
    update_reference_index(m_reference_trajectory, new_state, m_reference_temporal_index, time_f);
}

////////////////////////////////////////////////////////////////////////////////
bool ControllerBase::is_past_trajectory(const State & state) const noexcept
{
  // You are always past an empty trajectory
  if (m_reference_trajectory.points.empty()) {
    return true;
  }
  // If the current reference point is not the last one, then I can't be past the end
  if (m_reference_trajectory.points.size() > (m_reference_spatial_index + 1U)) {
    return false;
  }
  // Otherwise, check if I'm past the last point
  const auto & last_pt = m_reference_trajectory.points.back();
  // ^ back() shouldn't throw because the empty check happens before this function
  return motion_common::is_past_point(state.state, last_pt);
}

////////////////////////////////////////////////////////////////////////////////
Command ControllerBase::compute_stop_command(const State & state) const noexcept
{
  Command ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  ret.stamp = state.header.stamp;
  // Steering angle "uses up" stopping power/grip capacity
  ret.front_wheel_angle_rad = Real{};  // zero initialization etc.
  ret.rear_wheel_angle_rad = Real{};
  // Compute stopping acceleration
  // validate input
  const auto velocity = std::fabs(state.state.longitudinal_velocity_mps);
  const auto dt = std::chrono::duration_cast<std::chrono::duration<Real>>(m_config.time_step());
  const auto decel = std::min(
    velocity / dt.count(),
    m_config.safe_deceleration_rate());   // positive
  ret.long_accel_mps2 = state.state.longitudinal_velocity_mps >= 0.0F ? -decel : decel;

  return ret;
}

////////////////////////////////////////////////////////////////////////////////
Point ControllerBase::predict(const Point & point, std::chrono::nanoseconds dt) noexcept
{
  using autoware::common::state_vector::variable::X;
  using autoware::common::state_vector::variable::Y;
  using autoware::common::state_vector::variable::XY_VELOCITY;
  using autoware::common::state_vector::variable::XY_ACCELERATION;
  using autoware::common::state_vector::variable::YAW;
  using autoware::common::state_vector::variable::YAW_CHANGE_RATE;
  // Set up state
  autoware::common::motion_model::CatrMotionModel::State state{};
  state.at<X>() = point.x;
  state.at<Y>() = point.y;
  state.at<XY_VELOCITY>() = point.longitudinal_velocity_mps;
  state.at<XY_ACCELERATION>() = point.acceleration_mps2;
  state.at<YAW>() = motion_common::to_angle(point.heading);
  state.at<YAW_CHANGE_RATE>() = point.heading_rate_rps;
  state = m_model.predict(state, dt);
  // Get result
  auto ret = point;  // to fill the gaps of the model
  {
    ret.time_from_start =
      time_utils::to_message(time_utils::from_message(point.time_from_start) + dt);
    ret.x = state.at<X>();
    ret.y = state.at<Y>();
    ret.longitudinal_velocity_mps = state.at<XY_VELOCITY>();
    ret.acceleration_mps2 = state.at<XY_ACCELERATION>();
    ret.heading = motion_common::from_angle(state.at<YAW>());
    ret.heading_rate_rps = state.at<YAW_CHANGE_RATE>();
  }
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
std::string ControllerBase::name() const
{
  return std::string{"controller_base", std::allocator<char>{}};
}

////////////////////////////////////////////////////////////////////////////////
Index ControllerBase::get_compute_iterations() const
{
  return Index{};
}

////////////////////////////////////////////////////////////////////////////////
void compute_diagnostic(
  const ControllerBase & ctrl,
  const State & state,
  bool use_temporal_reference,
  Diagnostic & out)
{
  Index ref_idx = {};
  try {
    if (use_temporal_reference) {
      ref_idx = ctrl.get_current_state_temporal_index();
    } else {
      ref_idx = ctrl.get_current_state_spatial_index();
    }
  } catch (const std::domain_error &) {
    // nothing to do: no reference trajectory
    return;
  }
  out.diag_header.name = ctrl.name();
  out.diag_header.data_stamp = state.header.stamp;
  out.diag_header.iterations =
    static_cast<decltype(out.diag_header.iterations)>(ctrl.get_compute_iterations());
  const auto & ref = ctrl.get_reference_trajectory().points[ref_idx];
  motion_common::error(state.state, ref, out);
}

}  // namespace controller_common
}  // namespace control
}  // namespace motion
