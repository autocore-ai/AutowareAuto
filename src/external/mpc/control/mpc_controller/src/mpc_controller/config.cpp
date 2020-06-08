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

#include "mpc_controller/config.hpp"

#include <limits>
#include <stdexcept>

namespace motion
{
namespace control
{
namespace mpc_controller
{
Config::Config(
  const LimitsConfig & limits,
  const VehicleConfig & vehicle_param,
  const BehaviorConfig & behavior,
  const OptimizationConfig & optimization_param,
  const std::chrono::nanoseconds sample_period_tolerance,
  const std::chrono::nanoseconds control_lookahead_duration,
  const Interpolation interpolation_option)
: m_limits{limits},
  m_vehicle_param{vehicle_param},
  m_behavior_param{behavior},
  m_optimization_param{optimization_param},
  m_sample_period_tolerance{sample_period_tolerance},
  m_control_lookahead_duration{control_lookahead_duration},
  m_do_interpolate{interpolation_option == Interpolation::YES}
{
  if (sample_period_tolerance < decltype(sample_period_tolerance)::zero()) {
    throw std::domain_error{"Sample period tolerance must be positive"};
  }
  // Does it actually _have_ to be positive?
  if (control_lookahead_duration < decltype(control_lookahead_duration)::zero()) {
    throw std::domain_error{"Control lookahead duration must be positive"};
  }
}

const LimitsConfig & Config::limits() const noexcept
{
  return m_limits;
}
const VehicleConfig & Config::vehicle_param() const noexcept
{
  return m_vehicle_param;
}
const BehaviorConfig & Config::behavior() const noexcept
{
  return m_behavior_param;
}
const OptimizationConfig & Config::optimization_param() const noexcept
{
  return m_optimization_param;
}
std::chrono::nanoseconds Config::sample_period_tolerance() const noexcept
{
  return m_sample_period_tolerance;
}
std::chrono::nanoseconds Config::control_lookahead_duration() const noexcept
{
  return m_control_lookahead_duration;
}
bool Config::do_interpolate() const noexcept
{
  return m_do_interpolate;
}
}  // namespace mpc_controller
}  // namespace control
}  // namespace motion
