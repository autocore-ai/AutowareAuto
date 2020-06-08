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

#ifndef MPC_PLANNER__MPC_PLANNER_HPP_
#define MPC_PLANNER__MPC_PLANNER_HPP_

#include <mpc_planner/visibility_control.hpp>
#include <motion_common/config.hpp>
#include <planning_common/planner_base.hpp>

#include <ostream>

namespace motion
{
namespace planning
{
namespace mpc_planner
{
using planning_common::Trajectory;

/// Configuration class for MPC Planner
class MPC_PLANNER_PUBLIC PlannerConfig
{
public:
  PlannerConfig(
    const motion_common::LimitsConfig & limits,
    const motion_common::VehicleConfig & vehicle_param,
    const motion_common::OptimizationConfig & weights);

  const motion_common::LimitsConfig & limits() const noexcept;
  const motion_common::VehicleConfig & vehicle_param() const noexcept;
  const motion_common::OptimizationConfig & weights() const noexcept;

private:
  motion_common::LimitsConfig m_limits;
  motion_common::VehicleConfig m_vehicle_param;
  motion_common::OptimizationConfig m_weights;
};  // class PlannerConfig

/// \brief A bare base class for planning algorithms
class MPC_PLANNER_PUBLIC MpcPlanner : public planning_common::PlannerBase
{
public:
  /// Constructor
  explicit MpcPlanner(const PlannerConfig & config);

  /// Getters and setters for configuration
  const PlannerConfig & get_config() const noexcept;
  void set_config(const PlannerConfig & config) noexcept;
  void set_config(PlannerConfig && config) noexcept;

  /// Printout of current MPC state; for debugging purposes
  void debug_print(std::ostream & out) const;

protected:
  /// Solve mpc
  const Trajectory & plan_impl(const planning_common::PlanningContext & context) override;

private:
  using Point = motion_common::Point;
  using Real = motion_common::Real;
  /// Applies configuration parametes
  MPC_PLANNER_LOCAL void set_config_impl() noexcept;
  /// Sets vehicle parameters to online data
  MPC_PLANNER_LOCAL void set_parameters(const motion_common::VehicleConfig & cfg) noexcept;
  /// Sets upper and lower bounds for state variables
  MPC_PLANNER_LOCAL void set_limits(const motion_common::LimitsConfig & cfg) noexcept;
  /// Sets nominal weights in optimization problem
  MPC_PLANNER_LOCAL void set_nominal_weights(const motion_common::StateWeight & cfg) noexcept;
  /// Sets terminal weights in optimization problem
  MPC_PLANNER_LOCAL void set_terminal_weights(const motion_common::StateWeight & cfg) noexcept;
  /// Zeros all terms in reference vector
  MPC_PLANNER_LOCAL void set_zero_references() noexcept;
  /// Sets terminal reference
  MPC_PLANNER_LOCAL
  void set_target(const Point & target, motion::motion_common::Heading ref_heading) noexcept;
  /// Sets reference velocity; in general the references for all other variables will be zero
  /// or not available nominally
  MPC_PLANNER_LOCAL void set_reference_velocity(Real velocity) noexcept;
  /// Sets current state
  MPC_PLANNER_LOCAL void set_x0(const Point & state);
  /// Solves MPC
  MPC_PLANNER_LOCAL void solve();
  /// Deserializes a vector version of a solution into a trajectory message
  MPC_PLANNER_LOCAL const Trajectory & from_solution(const std_msgs::msg::Header & header);

  PlannerConfig m_config;
  Trajectory m_trajectory{};
};  // class PlannerBase
}  // namespace mpc_planner
}  // namespace planning
}  // namespace motion
#endif  // MPC_PLANNER__MPC_PLANNER_HPP_
