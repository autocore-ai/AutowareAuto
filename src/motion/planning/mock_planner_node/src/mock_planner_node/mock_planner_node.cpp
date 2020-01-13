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
#include "mock_planner_node/mock_planner_node.hpp"

#include <motion_common/config.hpp>

#include <memory>
#include <string>

namespace motion
{
namespace planning
{
namespace mock_planner_node
{
MockPlannerNode::MockPlannerNode(const std::string & name, const std::string & ns)
: PlannerBaseNode{name, ns}
{
  using motion_common::Real;
  using motion_common::LimitsConfig;
  const LimitsConfig limits{
    {
      static_cast<Real>(
        declare_parameter("planner.limits.min_longitudinal_velocity_mps").get<double>()),
      static_cast<Real>(
        declare_parameter("planner.limits.max_longitudinal_velocity_mps").get<double>())
    },
    {
      static_cast<Real>(
        declare_parameter("planner.limits.min_lateral_velocity_mps").get<double>()),
      static_cast<Real>(
        declare_parameter("planner.limits.max_lateral_velocity_mps").get<double>())
    },
    {
      static_cast<Real>(declare_parameter("planner.limits.min_acceleration_mps2").get<double>()),
      static_cast<Real>(declare_parameter("planner.limits.max_acceleration_mps2").get<double>())
    },
    {
      static_cast<Real>(declare_parameter("planner.limits.min_yaw_rate_rps").get<double>()),
      static_cast<Real>(declare_parameter("planner.limits.max_yaw_rate_rps").get<double>())
    },
    {
      static_cast<Real>(declare_parameter("planner.limits.min_jerk_mps3").get<double>()),
      static_cast<Real>(declare_parameter("planner.limits.max_jerk_mps3").get<double>())
    },
    {
      static_cast<Real>(declare_parameter("planner.limits.min_steer_angle_rad").get<double>()),
      static_cast<Real>(declare_parameter("planner.limits.max_steer_angle_rad").get<double>())
    },
    {
      static_cast<Real>(
        declare_parameter("planner.limits.min_steer_angle_rate_rps").get<double>()),
      static_cast<Real>(
        declare_parameter("planner.limits.max_steer_angle_rate_rps").get<double>())
    },
  };
  using motion_common::VehicleConfig;
  const VehicleConfig vehicle_param{
    static_cast<Real>(declare_parameter("planner.vehicle.cg_to_front_m").get<double>()),
    static_cast<Real>(declare_parameter("planner.vehicle.cg_to_rear_m").get<double>()),
    static_cast<Real>(declare_parameter("planner.vehicle.front_corner_stiffness").get<double>()),
    static_cast<Real>(declare_parameter("planner.vehicle.rear_corner_stiffness").get<double>()),
    static_cast<Real>(declare_parameter("planner.vehicle.mass_kg").get<double>()),
    static_cast<Real>(declare_parameter("planner.vehicle.yaw_inertia_kgm2").get<double>())
  };

  using motion_common::OptimizationConfig;
  using motion_common::StateWeight;
  const OptimizationConfig weights{
    StateWeight{
      static_cast<Real>(declare_parameter("planner.weights.nominal.pose").get<double>()),
      static_cast<Real>(declare_parameter("planner.weights.nominal.heading").get<double>()),
      static_cast<Real>(
        declare_parameter("planner.weights.nominal.longitudinal_velocity").get<double>()),
      static_cast<Real>(
        declare_parameter("planner.weights.nominal.lateral_velocity").get<double>()),
      static_cast<Real>(declare_parameter("planner.weights.nominal.yaw_rate").get<double>()),
      static_cast<Real>(declare_parameter("planner.weights.nominal.acceleration").get<double>()),
      static_cast<Real>(declare_parameter("planner.weights.nominal.jerk").get<double>()),
      static_cast<Real>(declare_parameter("planner.weights.nominal.steer_angle").get<double>()),
      static_cast<Real>(
        declare_parameter("planner.weights.nominal.steer_angle_rate").get<double>()),
    },
    StateWeight{
      static_cast<Real>(declare_parameter("planner.weights.terminal.pose").get<double>()),
      static_cast<Real>(declare_parameter("planner.weights.terminal.heading").get<double>()),
      static_cast<Real>(
        declare_parameter("planner.weights.terminal.longitudinal_velocity").get<double>()),
      static_cast<Real>(
        declare_parameter("planner.weights.terminal.lateral_velocity").get<double>()),
      static_cast<Real>(declare_parameter("planner.weights.terminal.yaw_rate").get<double>()),
      static_cast<Real>(
        declare_parameter("planner.weights.terminal.acceleration").get<double>()),
      static_cast<Real>(declare_parameter("planner.weights.terminal.jerk").get<double>()),
      static_cast<Real>(declare_parameter("planner.weights.terminal.steer_angle").get<double>()),
      static_cast<Real>(
        declare_parameter("planner.weights.terminal.steer_angle_rate").get<double>()),
    }
  };

  set_planner(std::make_unique<mock_planner::MockPlanner>(
      mock_planner::PlannerConfig{limits, vehicle_param, weights}));
}
////////////////////////////////////////////////////////////////////////////////
MockPlannerNode::MockPlannerNode(
  const std::string & name,
  const std::string & ns,
  const std::string & trajectory_topic,
  const std::string & ego_topic,
  const std::string & target_topic,
  const std::string & object_topic,
  const std::string & boundary_topic,
  const std::string & tf_topic,
  const std::string & diagnostic_topic,
  const planning_common_nodes::ContextSource source,
  const planning_common::EnvironmentConfig & cfg,
  const mock_planner::PlannerConfig & planner_cfg)
: PlannerBaseNode{
    name,
    ns,
    trajectory_topic,
    ego_topic,
    target_topic,
    object_topic,
    boundary_topic,
    tf_topic,
    diagnostic_topic,
    source,
    cfg
}
{
  set_planner(std::make_unique<mock_planner::MockPlanner>(planner_cfg));
}
}  // namespace mock_planner_node
}  // namespace planning
}  // namespace motion
