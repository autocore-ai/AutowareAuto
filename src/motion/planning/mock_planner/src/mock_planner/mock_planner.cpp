// Copyright 2019 Christopher Ho, changes by Sandro Merkli
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
#include "mock_planner/mock_planner.hpp"


#include <motion_common/motion_common.hpp>
#include <time_utils/time_utils.hpp>

#include <string>
#include <utility>

namespace motion
{
namespace planning
{
namespace mock_planner
{
using motion_common::Real;
using motion_common::Index;

constexpr auto HORIZON = static_cast<Index>(10);
constexpr auto NX = static_cast<Index>(4);
static_assert(NX == 4U, "Unexpected value for number of states");
constexpr auto IDX_X = 0U;
constexpr auto IDX_Y = 1U;
constexpr auto IDX_HEADING = 2U;
constexpr auto IDX_VEL_LONG = 3U;
constexpr auto NU = static_cast<Index>(2);
static_assert(NU == 2U, "Unexpected value for number of controls");
constexpr auto IDU_ACCEL = 0U;
constexpr auto IDU_WHEEL_ANGLE = 1U;
// constexpr auto NYN = static_cast<Index>(4);
// static_assert(NYN == 4U, "Unexpected value for number of terminal references");
constexpr auto IDYN_X = 0U;
constexpr auto IDYN_Y = 1U;
constexpr auto IDYN_HEADING = 2U;
constexpr auto IDYN_VEL_LONG = 3U;
////////////////////////////////////////////////////////////////////////////////
MockPlanner::MockPlanner(const PlannerConfig & config)
: m_config{config}
{
  set_zero_references();
  set_config_impl();
  static_assert(HORIZON <= Trajectory::CAPACITY, "Mock Solution cannot fit in Trajectory");
  m_trajectory.points.resize(HORIZON);
  for (auto idx = Index{}; idx < m_trajectory.points.size(); ++idx) {
    auto & pt = m_trajectory.points[idx];
    constexpr auto ms100 = std::chrono::milliseconds{100LL};
    pt.time_from_start = time_utils::to_message(idx * ms100);
  }
}

////////////////////////////////////////////////////////////////////////////////
const PlannerConfig & MockPlanner::get_config() const noexcept
{
  return m_config;
}

void MockPlanner::set_config(const PlannerConfig & config) noexcept
{
  m_config = config;
  set_config_impl();
}

void MockPlanner::set_config(PlannerConfig && config) noexcept
{
  m_config = std::forward<PlannerConfig &&>(config);
  set_config_impl();
}

////////////////////////////////////////////////////////////////////////////////
const Trajectory & MockPlanner::plan_impl(const planning_common::PlanningContext & context)
{
  const auto & target = context.target_state();
  const auto & ego = context.ego_state();
  set_target(target.state, ego.state.heading);
  set_reference_velocity(target.state.longitudinal_velocity_mps);
  set_x0(ego.state);
  return compute_trajectory(target.header);
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_config_impl() noexcept
{
  set_parameters(get_config().vehicle_param());
  set_limits(get_config().limits());
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_zero_references() noexcept
{
  // TODO(s.me) mock implementation
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_parameters(const motion_common::VehicleConfig & cfg) noexcept
{
  // TODO(s.me) mock implementation
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_limits(const motion_common::LimitsConfig & cfg) noexcept
{
  // TODO(s.me) mock implementation
  // Use stuff like cfg.acceleration().min() and so on
  (void)cfg;
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_reference_velocity(Real velocity) noexcept
{
  // TODO(s.me) mock implementation
  (void)velocity;
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_target(
  const Point & target,
  const motion::motion_common::Heading ref_heading) noexcept
{
  // TODO(s.me) mock implementation
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_x0(const Point & state)
{
  // TODO(s.me) mock implementation
}

////////////////////////////////////////////////////////////////////////////////
const Trajectory & MockPlanner::compute_trajectory(const std_msgs::msg::Header & header)
{
  // TODO(s.me) mock implementation
  auto & traj = m_trajectory;
  traj.header = header;
  for (std::size_t i = {}; i < HORIZON; ++i) {
    // time from start set in constructor
    auto & pt = traj.points[i];
    const auto idx = NX * i;

    // Set state values
    pt.x = Real{};
    pt.y = Real{};
    pt.lateral_velocity_mps = Real{};
    pt.longitudinal_velocity_mps = Real{};
    pt.heading = motion_common::from_angle(Real{});
    pt.acceleration_mps2 = Real{};
    {
      // Kinematic bicycle: omega = v tan(delta) / L
      // TODO(s.me) get this back to in higher precision if needed
      const auto L = static_cast<Real>(get_config().vehicle_param().length_cg_front_axel() +
        get_config().vehicle_param().length_cg_rear_axel());
      const auto v = Real{};
      const auto delta = Real{};
      const auto tan_del = std::tan(delta);
      pt.heading_rate_rps = static_cast<Real>((v * tan_del) / L);
    }
  }
  return traj;
}

}  // namespace mock_planner
}  // namespace planning
}  // namespace motion
