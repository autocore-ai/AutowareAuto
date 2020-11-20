// Copyright 2020 Embotech AG, Zurich, Switzerland. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef PARKING_PLANNER__PARKING_PLANNER_TYPES_HPP_
#define PARKING_PLANNER__PARKING_PLANNER_TYPES_HPP_

#include <stdint.h>
#include <vector>
#include <string>

#include "geometry.hpp"

namespace autoware
{

namespace motion
{

namespace planning
{

namespace parking_planner
{
// -- VehicleState ------------------------------------------------------------
/// \brief Class to represent the state of a vehicle
template<typename T>
class VehicleState
{
public:
  /// \brief No-input constructor sets everything to the zero of the template parameter.
  //         This is used for example in constructs such as VehicleState{}.
  VehicleState() noexcept
  : VehicleState({}, {}, {}, {}, {}) {}

  VehicleState(T x, T y, T velocity, T heading, T steering) noexcept
  {
    m_position_x = x;
    m_position_y = y;
    m_velocity = velocity;
    m_heading = heading;
    m_steering = steering;
  }

  /// \brief Turn the object into flat vector of doubles (for use with numerical code)
  std::vector<T> serialize(void) const
  {
    return std::vector<T>{m_position_x, m_position_y, m_velocity, m_heading, m_steering};
  }

  // \brief Turn a previously serialized state vector back into a VehicleState object
  static VehicleState deserialize(std::vector<T> serialized_states)
  {
    if (serialized_states.size() != VehicleState::internal_states_number) {
      throw std::length_error {"Need a vector of length " +
              std::string{VehicleState::internal_states_number}
      };
    }

    return VehicleState(
      serialized_states[0], serialized_states[1], serialized_states[2],
      serialized_states[3],
      serialized_states[4]);
  }

  // Getters and setters, nothing fancy now but may get checks later
  T get_x() const noexcept {return m_position_x;}
  T get_y() const noexcept {return m_position_y;}
  T get_velocity() const noexcept {return m_velocity;}
  T get_heading() const noexcept {return m_heading;}
  T get_steering() const noexcept {return m_steering;}
  void set_x(T x) noexcept {m_position_x = x;}
  void set_y(T y) noexcept {m_position_y = y;}
  void set_velocity(T velocity) noexcept {m_velocity = velocity;}
  void set_heading(T heading) noexcept {m_heading = heading;}
  void set_steering(T steering) noexcept {m_steering = steering;}

  /// \brief Get number of scalars a serialized version of this object has
  static constexpr std::size_t get_serialized_length() noexcept
  {
    return internal_states_number;
  }

  /// \brief Equality comparison operator
  bool operator==(const VehicleState<T> & other) const noexcept
  {
    return (this->m_position_x == other.m_position_x) &&
           (this->m_position_y == other.m_position_y) &&
           (this->m_velocity == other.m_velocity) &&
           (this->m_heading == other.m_heading) &&
           (this->m_steering == other.m_steering);
  }

private:
  /// Number of internal states, update this if that number changes
  static constexpr std::size_t internal_states_number = 5;

  /// X position in meters
  T m_position_x;

  /// Y position in meters
  T m_position_y;

  /// Velocity in meters per second
  T m_velocity;

  /// Heading in radians (positive x direction is 0), counter-clockwise
  T m_heading;

  /// Steering angle of the front wheels in radians
  T m_steering;
};


// -- VehicleCommand ----------------------------------------------------------
/// \brief Class to represent the inputs of a vehicle
template<typename T>
class VehicleCommand
{
public:
  // No-input constructor sets everything to the zero of the template parameter.
  // This is used when one creates empty std::vectors of those types.
  VehicleCommand() noexcept
  : VehicleCommand({}, {}) {}

  VehicleCommand(T steering_rate, T throttle) noexcept
  {
    m_steering_rate = steering_rate;
    m_throttle = throttle;
  }

  /// \brief Turn the object into flat vector of doubles (for use with numerical code)
  std::vector<T> serialize(void) const
  {return std::vector<T>{m_steering_rate, m_throttle};}

  /// \brief Turn a previously serialized command vector back into a VehicleCommand object
  static VehicleCommand deserialize(std::vector<T> serialized_commands)
  {
    if (serialized_commands.size() != VehicleCommand::internal_commands_number) {
      throw std::length_error {"Need a vector of length " +
              std::string{VehicleCommand::internal_commands_number}
      };
    }

    return VehicleCommand(serialized_commands[0], serialized_commands[1]);
  }

  T get_steering_rate() const noexcept {return m_steering_rate;}
  T get_throttle() const noexcept {return m_throttle;}
  void set_steering_rate(T steering_rate) noexcept {m_steering_rate = steering_rate;}
  void set_throttle(T throttle) noexcept {m_throttle = throttle;}

  /// \brief Get number of scalars a serialized version of this object has
  static constexpr std::size_t get_serialized_length() noexcept
  {return internal_commands_number;}

private:
  /// Number of internal commands, update this if that number changes
  static constexpr std::size_t internal_commands_number = 2;

  /// Steering rate of the vehicle, in radians per second
  T m_steering_rate;

  /// Throttle to be applied
  T m_throttle;
};


// -- PlanningResult ----------------------------------------------------------
/// \brief Class to represent one timestep in a dynamic trajectory
template<typename T>
class TrajectoryStep
{
public:
  TrajectoryStep(const VehicleCommand<T> command, const VehicleState<T> state) noexcept
  {
    m_command = command;
    m_state = state;
  }

  VehicleState<T> get_state() const noexcept {return this->m_state;}
  VehicleCommand<T> get_command() const noexcept {return this->m_command;}

private:
  VehicleCommand<T> m_command;
  VehicleState<T> m_state;
};

/// \brief Class to represent a dynamic trajectory
template<typename T>
using Trajectory = std::vector<TrajectoryStep<T>>;

}  // namespace parking_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // PARKING_PLANNER__PARKING_PLANNER_TYPES_HPP_
