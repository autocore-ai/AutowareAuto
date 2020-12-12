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
#ifndef PARKING_PLANNER__BICYCLE_MODEL_HPP_
#define PARKING_PLANNER__BICYCLE_MODEL_HPP_

#include <common/types.hpp>
#include <vector>
#include <string>

#include "parking_planner_types.hpp"
#include "geometry.hpp"
#include "rungekutta.hpp"

namespace autoware
{

namespace motion
{

namespace planning
{

namespace parking_planner
{
using autoware::common::types::float64_t;

// We could introduce a separate class for this, but it really has the same fields
// and the same order, so we just alias
template<typename T>
using VehicleStateDerivative = parking_planner::VehicleState<T>;

/// \brief Class for storing physical parameters of a bicycle model bicycle model
template<typename T>
class BicycleModelParameters
{
public:
  BicycleModelParameters(
    T length_front, T length_rear, T vehicle_width,
    T front_overhang, T rear_overhang) noexcept
  {
    m_length_front = length_front;
    m_length_rear = length_rear;
    m_vehicle_width = vehicle_width;
    m_front_overhang = front_overhang;
    m_rear_overhang = rear_overhang;
  }

  std::vector<T> serialize(void) const
  {
    return std::vector<T>{m_length_front, m_length_rear, m_vehicle_width, m_front_overhang,
      m_rear_overhang};
  }
  //
  // Turn a previsouly serialized state vector back into a VehicleState object
  static BicycleModelParameters deserialize(std::vector<T> serialized_states)
  {
    if (serialized_states.size() != BicycleModelParameters::internal_parameters_number) {
      throw std::length_error {"Need a vector of length " +
              std::string{BicycleModelParameters::internal_parameters_number}
      };
    }

    return BicycleModelParameters(
      serialized_states[0], serialized_states[1], serialized_states[2],
      serialized_states[3],
      serialized_states[4]);
  }

  // Getters and setters, nothing fancy now but may get checks later
  // TODO(enhancement,s.me) add nonnegativity checks
  T get_length_front() const noexcept {return m_length_front;}
  T get_length_rear() const noexcept {return m_length_rear;}
  T get_vehicle_width() const noexcept {return m_vehicle_width;}
  T get_front_overhang() const noexcept {return m_front_overhang;}
  T get_rear_overhang() const noexcept {return m_rear_overhang;}
  void set_length_front(T length_front) noexcept {m_length_front = length_front;}
  void set_length_rear(T length_rear) noexcept {m_length_rear = length_rear;}
  void set_vehicle_width(T vehicle_width) noexcept {m_vehicle_width = vehicle_width;}
  void set_front_overhang(T front_overhang) noexcept {m_front_overhang = front_overhang;}
  void set_rear_overhang(T rear_overhang) noexcept {m_rear_overhang = rear_overhang;}
  static constexpr std::size_t get_serialized_length() noexcept
  {
    return internal_parameters_number;
  }

private:
  static constexpr std::size_t internal_parameters_number = 5;

  /// \brief Length (in meters) between center of the vehicle as indicated by the
  ///        coordinate states and the front axle of the bicycle model.
  T m_length_front;

  /// \brief Length (in meters) between center of the vehicle as indicated by the
  ///        coordinate states and the rear axle of the bicycle model.
  T m_length_rear;

  /// \brief Width of the vehicle (meters)
  T m_vehicle_width;

  /// \brief Length (in meters) of the front overhang, that is how far the vehicle
  ///        extends beyond the front axle
  T m_front_overhang;

  /// \brief Length (in meters) of the rear overhang, that is how far the vehicle
  ///        extends behind the rear axle
  T m_rear_overhang;
};

/// \brief Class implementing a kinematic bicycle model
template<typename T, typename V>
class BicycleModel
{
public:
  explicit BicycleModel(BicycleModelParameters<V> parameters)
  : m_parameters(parameters)
  {
    // nothing other than the explicit initializer right now
  }

  /// \brief Evaluate the dynamics for given states and commands.
  /// \param[in] states The current state of the vehicle
  /// \param[in] commands The commands to be applied to the vehicle
  /// \return The (continous-time) derivative of the dynamics.
  VehicleStateDerivative<T> dynamics(VehicleState<T> states, VehicleCommand<T> commands) const
  {
    const auto lr = this->m_parameters.get_length_rear();
    const auto lf = this->m_parameters.get_length_front();
    const auto beta = atan(lr * tan(states.get_steering()) / (lf + lr));
    return VehicleStateDerivative<T>(
      states.get_velocity() * cos(states.get_heading() + beta),
      states.get_velocity() * sin(states.get_heading() + beta),
      commands.get_throttle(),
      states.get_velocity() / lr * sin(beta),
      commands.get_steering_rate()
    );
  }


  /// \brief Evaluate the integrated dynamics for given states and commands. A runge-kutta
  ///        integration scheme is used.
  /// \param[in] states The current state of the vehicle
  /// \param[in] commands The commands to be applied to the vehicle
  /// \param[in] stepsize Integration step size in seconds
  /// \param[in] number_of_steps Number of steps to integrate for
  /// \return The integrated state
  VehicleStateDerivative<T> integrated_dynamics(
    VehicleState<T> states,
    VehicleCommand<T> commands,
    const float64_t stepsize,
    const std::size_t number_of_steps
  ) const
  {
    const auto states_vec = states.serialize();
    const auto commands_vec = commands.serialize();

    const std::function<std::vector<T>(std::vector<T>, std::vector<T>)> dynfun =
      [this](const auto states, const auto commands) {
        return this->dynamics_serialized(states, commands);
      };

    const auto results_vec = RK4(states_vec, commands_vec, dynfun, stepsize, number_of_steps);
    return VehicleStateDerivative<T>::deserialize(results_vec);
  }

  /// \brief Serialized version of the continuous-time dynamics evaluation. See
  ///        the help of the dynamics function for more info.
  /// \param[in] serialized_states Serialized states
  /// \param[in] serialized_commands Serialized commands
  /// \return The serialized integrated states
  std::vector<T> dynamics_serialized(
    std::vector<T> serialized_states,
    std::vector<T> serialized_commands) const
  {
    return this->dynamics(
      VehicleState<T>::deserialize(serialized_states),
      VehicleCommand<T>::deserialize(serialized_commands) ).serialize();
  }


  /// \brief compute a bounding box of a given vehicle model for given states.
  /// \param[in] states States to compute a bounding box for
  /// \return The computed bounding box
  Polytope2D<T> compute_bounding_box(VehicleState<T> states) const
  {
    std::vector<Point2D<T>> corners{};
    const auto xfront = m_parameters.get_length_front() + m_parameters.get_front_overhang();
    const auto xrear = m_parameters.get_length_rear() + m_parameters.get_rear_overhang();
    const auto yhalf = 0.5 * m_parameters.get_vehicle_width();
    corners.push_back(Point2D<T>(-xrear, yhalf) );
    corners.push_back(Point2D<T>(-xrear, -yhalf) );
    corners.push_back(Point2D<T>(xfront, -yhalf) );
    corners.push_back(Point2D<T>(xfront, yhalf) );
    auto polytope = Polytope2D<T>(corners);
    polytope.rotate_and_shift(
      states.get_heading(),
      Point2D<T>({}, {}), Point2D<T>(states.get_x(), states.get_y()));
    return polytope;
  }

private:
  /// Physical parameters of this model
  BicycleModelParameters<V> m_parameters;
};


}  // namespace parking_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // PARKING_PLANNER__BICYCLE_MODEL_HPP_
