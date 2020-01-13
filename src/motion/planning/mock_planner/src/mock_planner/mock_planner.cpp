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
constexpr auto NYN = static_cast<Index>(4);
static_assert(NYN == 4U, "Unexpected value for number of terminal references");
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
  static_assert(HORIZON <= Trajectory::CAPACITY, "MOCK Solution cannot fit in Trajectory");
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
  solve();
  return from_solution(target.header);
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_config_impl() noexcept
{
  set_parameters(get_config().vehicle_param());
  set_limits(get_config().limits());
  set_nominal_weights(get_config().weights().nominal());
  set_terminal_weights(get_config().weights().terminal());
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_zero_references() noexcept
{
  // TODO mock implementation
  //static_assert(ACADO_NY == 1, "Unexpected number of reference variables");
  //constexpr auto NY = static_cast<Index>(ACADO_NY);
  //std::fill(&acadoVariables.y[0U], &acadoVariables.y[HORIZON * NY], AcadoReal{});
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_parameters(const motion_common::VehicleConfig & cfg) noexcept
{
  // TODO mock implementation
  //constexpr auto NOD = static_cast<Index>(ACADO_NOD);
  //static_assert(NOD == 2U, "Unexpected value for number of parameters");
  //for (std::size_t i = {}; i < HORIZON; ++i) {
    //const auto idx = i * ACADO_NOD;
    //constexpr auto idx_Lf = 0U;
    //constexpr auto idx_Lr = 1U;
    //acadoVariables.od[idx + idx_Lf] = static_cast<AcadoReal>(cfg.length_cg_front_axel());
    //acadoVariables.od[idx + idx_Lr] = static_cast<AcadoReal>(cfg.length_cg_rear_axel());
  //}
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_limits(const motion_common::LimitsConfig & cfg) noexcept
{
  // TODO mock implementation
  //static_assert(ACADO_HARDCODED_CONSTRAINT_VALUES == 0, "Constraints not hard coded");
  //constexpr auto NUM_CTRL_CONSTRAINTS = 2U;
  //constexpr auto NUM_STATE_CONSTRAINTS = 1U;
  //for (std::size_t i = {}; i < HORIZON; ++i) {
    //{
      //const auto idx = i * NUM_CTRL_CONSTRAINTS;
      //constexpr auto idx_ax = 0U;
      //constexpr auto idx_delta = 1U;
      //acadoVariables.lbValues[idx + idx_ax] = static_cast<AcadoReal>(cfg.acceleration().min());
      //acadoVariables.ubValues[idx + idx_ax] = static_cast<AcadoReal>(cfg.acceleration().max());
      //acadoVariables.lbValues[idx + idx_delta] = static_cast<AcadoReal>(cfg.steer_angle().min());
      //acadoVariables.ubValues[idx + idx_delta] = static_cast<AcadoReal>(cfg.steer_angle().max());
    //}
    //{
      //// DifferentialState or Affine constraints are sometimes put into different structs
      //// i.e. when using qpOASES
      //const auto idx = i * NUM_STATE_CONSTRAINTS;
      //// These are different from the general state constraints because not all states
      //// have constraints
      //// If you're changing this, check the order in the code generation script
      //constexpr auto idx_u = 0U;
      //acadoVariables.lbAValues[idx + idx_u] =
        //static_cast<AcadoReal>(cfg.longitudinal_velocity().min());
      //acadoVariables.ubAValues[idx + idx_u] =
        //static_cast<AcadoReal>(cfg.longitudinal_velocity().max());
    //}
  //}
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_nominal_weights(const motion_common::StateWeight & cfg) noexcept
{
  // TODO mock implementation
  //static_assert(ACADO_WEIGHTING_MATRICES_TYPE == 1, "Weighting matrix should be fixed");
  //static_assert(ACADO_NY == 1, "Unexpected number of reference variables");
  //acadoVariables.W[0U] = static_cast<AcadoReal>(cfg.longitudinal_velocity());
}
////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_terminal_weights(const motion_common::StateWeight & cfg) noexcept
{
  // TODO mock implementation
  //static_assert(ACADO_NYN == 4, "Unexpected number of terminal reference variables");
  //acadoVariables.WN[(IDYN_X * NYN) + IDYN_X] = static_cast<AcadoReal>(cfg.pose());
  //acadoVariables.WN[(IDYN_Y * NYN) + IDYN_Y] = static_cast<AcadoReal>(cfg.pose());
  //acadoVariables.WN[(IDYN_HEADING * NYN) + IDYN_HEADING] =
    //static_cast<AcadoReal>(cfg.heading());
  //acadoVariables.WN[(IDYN_VEL_LONG * NYN) + IDYN_VEL_LONG] =
    //static_cast<AcadoReal>(cfg.longitudinal_velocity());
}
////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_reference_velocity(Real velocity) noexcept
{
  // TODO mock implementation
  //constexpr auto NY = static_cast<Index>(ACADO_NY);
  //static_assert(NY == 1U, "Unexpected value for number of nominal references");
  //const auto vel = static_cast<AcadoReal>(velocity);
  //std::fill(&acadoVariables.y[0U], &acadoVariables.y[HORIZON * NY], vel);
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_target(
  const Point & target,
  const motion::motion_common::Heading ref_heading) noexcept
{
  // TODO mock implementation
  //acadoVariables.yN[IDYN_X] = static_cast<AcadoReal>(target.x);
  //acadoVariables.yN[IDYN_Y] = static_cast<AcadoReal>(target.y);
  //const auto dth = motion_common::to_angle(target.heading - ref_heading);
  //acadoVariables.yN[IDYN_HEADING] =
    //static_cast<AcadoReal>(motion_common::to_angle(ref_heading)) + dth;
  //acadoVariables.yN[IDYN_VEL_LONG] = static_cast<AcadoReal>(target.longitudinal_velocity_mps);
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::set_x0(const Point & state)
{
  // TODO mock implementation
  //// Set x0
  //acadoVariables.x0[IDX_X] = static_cast<AcadoReal>(state.x);
  //acadoVariables.x0[IDX_Y] = static_cast<AcadoReal>(state.y);
  //acadoVariables.x0[IDX_HEADING] =
    //static_cast<AcadoReal>(motion_common::to_angle(state.heading));
  //acadoVariables.x0[IDX_VEL_LONG] = static_cast<AcadoReal>(state.longitudinal_velocity_mps);
  //// Initialization stuff
  //acadoVariables.x[IDX_X] = acadoVariables.x0[IDX_X];
  //acadoVariables.x[IDX_Y] = acadoVariables.x0[IDX_Y];
  //acadoVariables.x[IDX_HEADING] = acadoVariables.x0[IDX_HEADING];
  //acadoVariables.x[IDX_VEL_LONG] = acadoVariables.x0[IDX_VEL_LONG];
}

////////////////////////////////////////////////////////////////////////////////
void MockPlanner::solve()
{
  // TODO mock implementation
  //// Cold start
  //{
    //std::fill(&acadoVariables.u[0U], &acadoVariables.u[HORIZON * NU], AcadoReal{});
    //acado_initializeNodesByForwardSimulation();
  //}
  //const auto prep_ret = acado_preparationStep();
  //if (0 != prep_ret) {
    //std::string err_str{"Solver preparation error: ", std::string::allocator_type{}};
    //err_str += std::to_string(prep_ret);
    //throw std::runtime_error{err_str};
  //}
  //const auto solve_ret = acado_feedbackStep();
  //if (0 != solve_ret) {
    //std::string err_str{"Solver error: ", std::string::allocator_type{}};
    //err_str += std::to_string(solve_ret);
    //throw std::runtime_error{err_str};
  //}
}

////////////////////////////////////////////////////////////////////////////////
const Trajectory & MockPlanner::from_solution(const std_msgs::msg::Header & header)
{
  // TODO mock implementation
  //auto & traj = m_trajectory;
  //traj.header = header;
  //for (std::size_t i = {}; i < HORIZON; ++i) {
    //// time from start set in constructor
    //auto & pt = traj.points[i];
    //const auto idx = NX * i;
    //pt.x = static_cast<Real>(acadoVariables.x[idx + IDX_X]);
    //pt.y = static_cast<Real>(acadoVariables.x[idx + IDX_Y]);
    //pt.lateral_velocity_mps = Real{};
    //const auto heading = static_cast<Real>(acadoVariables.x[idx + IDX_HEADING]);
    //pt.longitudinal_velocity_mps = static_cast<Real>(acadoVariables.x[idx + IDX_VEL_LONG]);
    //pt.heading = motion_common::from_angle(heading);
    //const auto jdx = NU * i;
    //pt.acceleration_mps2 = static_cast<Real>(acadoVariables.u[jdx + IDU_ACCEL]);
    //{
      //// Kinematic bicycle: omega = v tan(delta) / L
      //// Make sure this is in higher precision
      //const auto L = static_cast<AcadoReal>(get_config().vehicle_param().length_cg_front_axel() +
        //get_config().vehicle_param().length_cg_rear_axel());
      //const auto v = acadoVariables.x[idx + IDX_VEL_LONG];
      //const auto tan_del = std::tan(acadoVariables.u[jdx + IDU_WHEEL_ANGLE]);
      //pt.heading_rate_rps = static_cast<Real>((v * tan_del) / L);
    //}
  //}
  //return traj;
}

}  // namespace mock_planner
}  // namespace planning
}  // namespace motion
