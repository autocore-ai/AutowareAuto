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

// This file contains some templated classes for interacting with the NLP, both
// at the time of problem formulation as well as for calling the solver after it
// has been generated. All the templates can be used with "symbolic" variables such as
// CasADi variable objects, but also with concrete numbers such as float and double.
//
// In fact, the adapters assemble_variable_vector and assemble_parameter_vector are
// used both when formulating the problem for code generation as well as afterwards
// for calling the generated solver. This ensures consistency of variable and parameter
// ordering because those orderings are defined just in one place - here.

#ifndef PARKING_PLANNER__NLP_ADAPTERS_HPP_
#define PARKING_PLANNER__NLP_ADAPTERS_HPP_

#include <common/types.hpp>
#include <vector>
#include <string>

#include "configuration.hpp"
#include "parking_planner_types.hpp"
#include "bicycle_model.hpp"
#include "geometry.hpp"

namespace autoware
{

namespace motion
{

namespace planning
{

/// \brief Classes and functionality for planning parking maneuvers
namespace parking_planner
{

using autoware::common::types::float64_t;
using autoware::motion::planning::parking_planner::VehicleState;
using autoware::motion::planning::parking_planner::VehicleCommand;
using autoware::motion::planning::parking_planner::BicycleModelParameters;
using autoware::motion::planning::parking_planner::Trajectory;

/// \brief Since we need upper and lower bounds in the constructs used here,
///        but some variables do not have clearly defined upper bounds, we introduce
///        an artificial upper bound number that is "large enough to never matter".
///        The value has been determined experimentally to work - picking this
///        value too small will lead to the problem no longer being mathematically
///        correct, making it too large will lead to numerical difficulties.
constexpr auto ARTIFICIAL_DUAL_MULTIPLIER_BOUND = 1.0e4;

// \brief Class to represent cost function weights used in the nonlinear
//        optimization problem solve
template<typename T>
class NLPCostWeights
{
public:
  NLPCostWeights(
    T steering,
    T throttle,
    T goal
  )
  {
    m_steering = steering;
    m_throttle = throttle;
    m_goal = goal;
  }

  std::vector<T> serialize(void) const
  {
    return std::vector<T>({m_steering, m_throttle, m_goal});
  }

  static NLPCostWeights deserialize(std::vector<T> serialized)
  {
    constexpr std::size_t expected_length = NLPCostWeights::get_serialized_length();
    if (serialized.size() != expected_length) {
      throw std::length_error {"Need a vector of length " + std::string{expected_length}};
    }
    NLPCostWeights weights(serialized[0], serialized[1], serialized[2]);
    return weights;
  }

  static constexpr std::size_t get_serialized_length() noexcept
  {return internal_weights_number;}

  T get_steering_weight() const noexcept {return m_steering;}
  T get_throttle_weight() const noexcept  {return m_throttle;}
  T get_goal_weight() const noexcept {return m_goal;}

private:
  static constexpr std::size_t internal_weights_number = 3;
  /// Weight for steering input
  T m_steering;

  /// Weight for throttle input
  T m_throttle;

  /// \brief Weight for closeness to the goal at the end of the trajectory. This cost
  ///        is in the cost function, but somewhat irrelevant if the terminal state
  ///        is set to a hard constraint.
  T m_goal;
};

/// \brief Class used to describe the additional dual variables required for each
//         stage for describing obstacle constraints
template<typename T>
class NLPObstacleStageVariables
{
public:
  NLPObstacleStageVariables(
    std::vector<T> lambda,
    std::vector<T> mu
  )
  {
    m_lambda = lambda;
    m_mu = mu;
  }

  /// \brief Serialize the object into a vector of scalars
  std::vector<T> serialize(void) const
  {
    std::vector<T> serialized{};
    serialized.reserve(internal_lambda_number + internal_mu_number);
    serialized.insert(serialized.end(), m_lambda.begin(), m_lambda.end());
    serialized.insert(serialized.end(), m_mu.begin(), m_mu.end());
    return serialized;
  }

  static constexpr std::size_t get_lambda_length() noexcept
  {return internal_lambda_number;}

  static constexpr std::size_t get_mu_length() noexcept
  {return internal_mu_number;}

  static constexpr std::size_t get_serialized_length() noexcept
  {return internal_mu_number + internal_lambda_number;}

  static NLPObstacleStageVariables deserialize(std::vector<T> serialized)
  {
    static constexpr auto lambda_length = NLPObstacleStageVariables<float64_t>::get_lambda_length();
    static constexpr auto mu_length = NLPObstacleStageVariables<float64_t>::get_mu_length();
    static constexpr auto expected_length =
      NLPObstacleStageVariables<float64_t>::get_serialized_length();
    if (serialized.size() != expected_length) {
      throw std::length_error {"Need a vector of length " + std::string{expected_length}};
    }

    const auto start = serialized.begin();
    return NLPObstacleStageVariables(
      std::vector<T>(start, start + lambda_length),
      std::vector<T>(start + lambda_length, start + lambda_length + mu_length) );
  }

  std::vector<T> get_lambda() const noexcept {return m_lambda;}
  std::vector<T> get_mu() const noexcept {return m_mu;}

private:
  /// Number of internal lambda variables, update this if that number changes
  static constexpr std::size_t internal_lambda_number = MAX_HYPERPLANES_PER_OBSTACLE;

  /// Number of internal mu variables, update this if that number changes
  static constexpr std::size_t internal_mu_number = MAX_EGO_HYPERPLANES;

  /// Lambda variables of the dual obstacle formulation (see paper)
  std::vector<T> m_lambda;

  /// Mu variables of the dual obstacle formulation (see paper)
  std::vector<T> m_mu;
};


/// \brief Class used to represent an obstacle with the relevant data and conversions
///        for the NLP solver. The idea is that this can be instantiated by the solver
///        using MX as a template parameter to build constraints, then again using
///        a concrete scalar type to call the solver with.
template<typename T>
class NLPObstacle
{
public:
  NLPObstacle(
    std::vector<NLPObstacleStageVariables<T>> stage_variables,
    std::vector<Halfplane2D<T>> halfplanes) noexcept
  {
    m_stage_variables = stage_variables;
    m_halfplanes = halfplanes;
  }

  /// \brief Turn the obstacle variables into a vector of scalars, for use by
  //         a CasADi-interfaced solver
  std::vector<T> serialize_variables(void) const
  {
    std::vector<T> serialized{};
    serialized.reserve(m_stage_variables.size() * m_stage_variables[0].get_serialized_length() );
    for (const auto & stage : m_stage_variables) {
      const auto serialized_stage = stage.serialize();
      serialized.insert(serialized.end(), serialized_stage.begin(), serialized_stage.end());
    }
    return serialized;
  }

  /// \brief Turn the obstacle parameter variables into a vector of scalars,
  //         for use by a CasADi-interfaced solver
  std::vector<T> serialize_parameters(void) const
  {
    std::vector<T> serialized{};
    for (const auto & halfplane : m_halfplanes) {
      const auto serialized_halfplane = halfplane.serialize();
      serialized.insert(
        serialized.end(), serialized_halfplane.begin(),
        serialized_halfplane.end() );
    }
    return serialized;
  }

  /// \brief Getter for the halfplanes
  std::vector<Halfplane2D<T>> get_halfplanes() const noexcept
  {return m_halfplanes;}

  /// \brief Getter for the stage variables
  std::vector<NLPObstacleStageVariables<T>> get_stage_variables() const noexcept
  {return m_stage_variables;}

  /// \brief Assemble an A matrix from the halfplanes in form of a vector of vectors
  //         where the outer vector is the column and the inner vectors each contain
  //         a row.
  std::vector<std::vector<T>> build_A() const
  {
    std::vector<std::vector<T>> A{};
    for (const auto & halfplane : this->m_halfplanes) {
      auto coordinates = halfplane.get_coefficients().get_coord();
      A.push_back(std::vector<T>({coordinates.first, coordinates.second}) );
    }
    return A;
  }

  /// \brief Build the b matrix from the halfplanes of the obstacles
  std::vector<T> build_b() const
  {
    std::vector<T> b{};
    for (const auto & halfplane : this->m_halfplanes) {
      auto rhs = halfplane.get_right_hand_side();
      b.push_back(rhs);
    }
    return b;
  }

private:
  /// Halfplanes of this obstacle
  std::vector<Halfplane2D<T>> m_halfplanes;

  /// Stage variables of this obstacle
  std::vector<NLPObstacleStageVariables<T>> m_stage_variables;
};


template<typename T>
struct SerializedVariables
{
  std::vector<T> variables;
  std::vector<float64_t> lower_bounds;
  std::vector<float64_t> upper_bounds;
};


/// \brief Function to assemble the variable vector for the NLP from given states and commands.
///        This is used in both the solver generation for specifying how the variables are
///        ordered as well as when calling the solver because it will expect the initial guess
///        in the same format. The adapter here serves as a single source of truth for how
///        the serialization is to be done.
///
///        The parameters for the bounds are only used when the solver is actually called, hence
///        they are not templated - for assembly of the solver vectors, they are ignored and can
///        be safely fed in as just VehicleState<float64_t>{} and VehicleCommand<float64_t>{}.
template<typename T>
SerializedVariables<T> assemble_variable_vector_and_bounds(
  const Trajectory<T> & trajectory,
  const std::vector<NLPObstacle<T>> & obstacles,
  const VehicleState<float64_t> & lower_state_bounds,
  const VehicleState<float64_t> & upper_state_bounds,
  const VehicleCommand<float64_t> & lower_command_bounds,
  const VehicleCommand<float64_t> & upper_command_bounds
)
{
  std::vector<T> assembled_variables{};
  std::vector<float64_t> lower_bounds{};
  std::vector<float64_t> upper_bounds{};
  const auto serialized_state_lower = lower_state_bounds.serialize();
  const auto serialized_state_upper = upper_state_bounds.serialize();
  const auto serialized_command_lower = lower_command_bounds.serialize();
  const auto serialized_command_upper = upper_command_bounds.serialize();
  for (const auto & step : trajectory) {
    // State and its bounds
    const auto serialized_state = step.get_state().serialize();
    assembled_variables.insert(
      assembled_variables.end(),
      serialized_state.begin(), serialized_state.end() );
    lower_bounds.insert(
      lower_bounds.end(),
      serialized_state_lower.begin(), serialized_state_lower.end() );
    upper_bounds.insert(
      upper_bounds.end(),
      serialized_state_upper.begin(), serialized_state_upper.end() );

    // Commands and their bounds
    const auto serialized_command = step.get_command().serialize();
    assembled_variables.insert(
      assembled_variables.end(),
      serialized_command.begin(), serialized_command.end() );
    lower_bounds.insert(
      lower_bounds.end(),
      serialized_command_lower.begin(), serialized_command_lower.end() );
    upper_bounds.insert(
      upper_bounds.end(),
      serialized_command_upper.begin(), serialized_command_upper.end() );
  }

  // Obstacle variables and their bounds
  for (const auto & obstacle : obstacles) {
    const auto serialized = obstacle.serialize_variables();
    assembled_variables.insert(
      assembled_variables.end(),
      serialized.begin(), serialized.end()
    );
    for (std::size_t k = {}; k < serialized.size(); k++) {
      lower_bounds.push_back(0.0);
      upper_bounds.push_back(ARTIFICIAL_DUAL_MULTIPLIER_BOUND);
    }
  }

  return SerializedVariables<T>{assembled_variables, lower_bounds, upper_bounds};
}


/// \brief Helper function to turn a serialized variable vector into a structured
///        Trajectory object
template<typename T>
Trajectory<float64_t> disassemble_variable_vector(
  const std::vector<T> & variable_vector,
  const std::size_t horizon_length)
{
  constexpr auto Nstates = VehicleState<float64_t>::get_serialized_length();
  constexpr auto Ncommands = VehicleCommand<float64_t>::get_serialized_length();

  Trajectory<float64_t> disassembled{};
  disassembled.reserve(variable_vector.size());
  auto vstart = variable_vector.begin();
  for (std::size_t k = {}; k < horizon_length; k++) {
    const auto state_k = VehicleState<T>::deserialize(std::vector<T>(vstart, vstart + Nstates));
    vstart += Nstates;
    const auto command_k = VehicleCommand<T>::deserialize(
      std::vector<T>(vstart, vstart + Ncommands)
    );
    vstart += Ncommands;
    disassembled.push_back(TrajectoryStep<T>(command_k, state_k));
  }

  return disassembled;
}


/// \brief Helper function to turn the inputs to an NLP solve into a flat vector
///        of scalars for use by the CasADi-interfaced solver
template<typename T>
std::vector<T> assemble_parameter_vector(
  const VehicleState<T> & current_state,
  const VehicleState<T> & goal_state,
  const BicycleModelParameters<T> & model_parameters,
  const std::vector<NLPObstacle<T>> & obstacles,
  const NLPCostWeights<T> & cost_weights
)
{
  std::vector<T> assembled{};
  {
    const auto serialized = current_state.serialize();
    assembled.insert(assembled.end(), serialized.begin(), serialized.end() );
  }
  {
    const auto serialized = goal_state.serialize();
    assembled.insert(assembled.end(), serialized.begin(), serialized.end() );
  }
  {
    const auto serialized = model_parameters.serialize();
    assembled.insert(assembled.end(), serialized.begin(), serialized.end() );
  }

  {
    const auto serialized = cost_weights.serialize();
    assembled.insert(assembled.end(), serialized.begin(), serialized.end() );
  }

  for (const auto & obstacle : obstacles) {
    const auto serialized = obstacle.serialize_parameters();
    assembled.insert(assembled.end(), serialized.begin(), serialized.end() );
  }

  return assembled;
}

}  // namespace parking_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // PARKING_PLANNER__NLP_ADAPTERS_HPP_
