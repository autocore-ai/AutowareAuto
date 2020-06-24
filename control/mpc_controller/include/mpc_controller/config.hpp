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
#ifndef MPC_CONTROLLER__CONFIG_HPP_
#define MPC_CONTROLLER__CONFIG_HPP_

#include <mpc_controller/visibility_control.hpp>
#include <controller_common/controller_base.hpp>
#include <motion_common/config.hpp>

#define MPC_CONTROLLER_COPY_MOVE_ASSIGNABLE(Class) \
  Class(const Class &) = default; \
  Class(Class &&) = default; \
  Class & operator=(const Class &) = default; \
  Class & operator=(Class &&) = default; \
  ~Class() = default;

namespace motion
{
namespace control
{
namespace mpc_controller
{
using motion_common::Real;
using motion_common::Command;
using motion_common::State;
using motion_common::Trajectory;
using motion_common::Point;
using motion_common::Heading;
using motion_common::Index;

using controller_common::BehaviorConfig;
using motion_common::StateWeight;
using motion_common::LimitsConfig;
using motion_common::VehicleConfig;
using motion_common::OptimizationConfig;
using motion_common::LimitsConfig;

enum class Interpolation : uint8_t
{
  YES = 0U,
  NO = 1U
};

/// \brief A configuration class for the MpcController
class MPC_CONTROLLER_PUBLIC Config
{
public:
  Config(
    const LimitsConfig & limits,
    const VehicleConfig & vehicle_param,
    const BehaviorConfig & behavior,
    const OptimizationConfig & optimization_param,
    std::chrono::nanoseconds sample_period_tolerance,
    std::chrono::nanoseconds control_lookahead_duration,
    Interpolation interpolation_option);
  MPC_CONTROLLER_COPY_MOVE_ASSIGNABLE(Config)

  const LimitsConfig & limits() const noexcept;
  const VehicleConfig & vehicle_param() const noexcept;
  const BehaviorConfig & behavior() const noexcept;
  const OptimizationConfig & optimization_param() const noexcept;
  std::chrono::nanoseconds sample_period_tolerance() const noexcept;
  std::chrono::nanoseconds control_lookahead_duration() const noexcept;
  bool do_interpolate() const noexcept;

private:
  LimitsConfig m_limits;
  VehicleConfig m_vehicle_param;
  BehaviorConfig m_behavior_param;
  OptimizationConfig m_optimization_param;
  std::chrono::nanoseconds m_sample_period_tolerance;
  std::chrono::nanoseconds m_control_lookahead_duration;
  bool m_do_interpolate;
};  // class Config

struct MPC_CONTROLLER_PUBLIC ControlDerivatives
{
  Real jerk_mps3;
  Real steer_angle_rate_rps;
};  // struct ControlDerivatives
}  // namespace mpc_controller
}  // namespace control
}  // namespace motion
#endif  // MPC_CONTROLLER__CONFIG_HPP_
