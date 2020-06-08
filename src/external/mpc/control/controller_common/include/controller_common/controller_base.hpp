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
#ifndef CONTROLLER_COMMON__CONTROLLER_BASE_HPP_
#define CONTROLLER_COMMON__CONTROLLER_BASE_HPP_

#include <controller_common/visibility_control.hpp>
#include <motion_common/motion_common.hpp>

#include <chrono>
#include <string>

#define CONTROLLER_COMMON_COPY_MOVE_ASSIGNABLE(Class) \
  Class(const Class &) = default; \
  Class(Class &&) = default; \
  Class & operator=(const Class &) = default; \
  Class & operator=(Class &&) = default; \
  ~Class() = default;

namespace motion
{
namespace control
{
namespace controller_common
{

using motion_common::Index;
using motion_common::Real;
using motion_common::Command;
using motion_common::Diagnostic;
using motion_common::Heading;
using motion_common::Point;
using motion_common::State;
using motion_common::Trajectory;

enum class ControlReference
{
  TEMPORAL,
  SPATIAL
};  // enum class ControlReference

/// \brief Specifies the behavior of the controller
class CONTROLLER_COMMON_PUBLIC BehaviorConfig
{
public:
  /// Positive rate of deceleration and characteristic time step of overall system
  BehaviorConfig(
    Real safe_deceleration_rate_mps2,
    std::chrono::nanoseconds time_step,
    ControlReference type);
  CONTROLLER_COMMON_COPY_MOVE_ASSIGNABLE(BehaviorConfig)

  Real safe_deceleration_rate() const noexcept;
  std::chrono::nanoseconds time_step() const noexcept;
  bool is_spatial_reference() const noexcept;
  bool is_temporal_reference() const noexcept;

private:
  Real m_safe_deceleration_rate_mps2;
  std::chrono::nanoseconds m_time_step;
  bool m_is_spatial_reference;
};  // class BehaviorConfig

/// A base class which exposes the basic API and implements the basic behaviors
/// of a controller in the absence of a trajectory, and other basic bookkeeping
class CONTROLLER_COMMON_PUBLIC ControllerBase
{
public:
  /// Constructors
  explicit ControllerBase(const BehaviorConfig & config);
  ControllerBase(const ControllerBase &) = default;
  ControllerBase(ControllerBase &&) = default;
  ControllerBase & operator=(const ControllerBase &) = default;
  ControllerBase & operator=(ControllerBase &&) = default;
  virtual ~ControllerBase() noexcept = default;

  /// Setter for reference trajectory, throws if trajectory is inappropriate
  /// without mutating state (assuming handle_new_trajectory also does not)
  void set_trajectory(const Trajectory & trajectory);
  /// Getter for reference trajectory
  const Trajectory & get_reference_trajectory() const noexcept;

  /// Main API: Given current state (and reference trajectory), compute next command.
  /// Updates the current reference point of the trajectory
  /// \throw std::domain_error If state is not in the same frame as reference trajectory
  Command compute_command(const State & state);

  /// Computes stopping control command
  Command compute_stop_command(const State & state) const noexcept;

  /// Returns the index of the current reference point in the trajectory, i.e.
  /// the first point on the trajectory just after the current point spatially.
  /// If a point exactly matches, return that index.
  /// \throw std::runtime_error if an empty tajectory or no trajectory is provided
  Index get_current_state_spatial_index() const;
  /// Returns the index of the current reference point in the trajectory, i.e.
  /// the first point on the trajectory just after the current point in time.
  /// If a point exactly matches, return that index.
  /// \throw std::runtime_error if an empty tajectory or no trajectory is provided
  Index get_current_state_temporal_index() const;

  /// Get name of algorithm, for debugging or diagnostic purposes
  virtual std::string name() const;

  /// Get name of algorithm, for debugging or diagnostic purposes
  virtual Index get_compute_iterations() const;

  const BehaviorConfig & get_base_config() const noexcept;

protected:
  /// Given a new trajectory, do some input validation: If the trajectory
  /// is fine for the algorithm, return true; otherwise return false.
  virtual bool check_new_trajectory(const Trajectory & trajectory) const;
  /// Update internal bookkeeping or the algorithm-specific representation of the trajectory
  /// associated with updating to a new trajectory, return preferred form of trajectory
  virtual const Trajectory & handle_new_trajectory(const Trajectory & trajectory);
  /// Actually compute the control command
  virtual Command compute_command_impl(const State & state) = 0;
  /// Get a predicted state based on given state
  Point predict(const Point & point, std::chrono::nanoseconds dt) noexcept;
  /// Config getters and setters
  void set_base_config(const BehaviorConfig & config) noexcept;

private:
  /// Check if state should require a safe stop rather than normal control logic
  CONTROLLER_COMMON_LOCAL bool is_state_ok(const State & state) const noexcept;
  /// Do an O(N) scan to find the next reference point State is just past
  CONTROLLER_COMMON_LOCAL void update_reference_indices(const State & new_state) noexcept;
  /// Determines if a given state is past the reference trajectory
  CONTROLLER_COMMON_LOCAL bool is_past_trajectory(const State & state) const noexcept;
  /// Just some cleanup
  CONTROLLER_COMMON_LOCAL void set_trajectory_impl();

  BehaviorConfig m_config;
  Trajectory m_reference_trajectory;
  std::chrono::system_clock::time_point m_latest_reference;
  Index m_reference_spatial_index;
  Index m_reference_temporal_index;
};  // class ControllerBase

/// Fill out a controller diagnostic message
CONTROLLER_COMMON_PUBLIC void compute_diagnostic(
  const ControllerBase & ctrl,
  const State & state,
  bool use_temporal_reference,  // TODO(c.ho) kind of a bad interface
  Diagnostic & out);

}  // namespace controller_common
}  // namespace control
}  // namespace motion
#endif  // CONTROLLER_COMMON__CONTROLLER_BASE_HPP_
