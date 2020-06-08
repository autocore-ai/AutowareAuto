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
#include "mpc_planner/mpc_planner.hpp"

namespace motion
{
namespace planning
{
namespace mpc_planner
{
PlannerConfig::PlannerConfig(
  const motion_common::LimitsConfig & limits,
  const motion_common::VehicleConfig & vehicle_param,
  const motion_common::OptimizationConfig & weights)
: m_limits{limits},
  m_vehicle_param{vehicle_param},
  m_weights{weights}
{
}

const motion_common::LimitsConfig & PlannerConfig::limits() const noexcept
{
  return m_limits;
}

const motion_common::VehicleConfig & PlannerConfig::vehicle_param() const noexcept
{
  return m_vehicle_param;
}

const motion_common::OptimizationConfig & PlannerConfig::weights() const noexcept
{
  return m_weights;
}
}  // namespace mpc_planner
}  // namespace planning
}  // namespace motion
