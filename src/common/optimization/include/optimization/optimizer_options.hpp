// Copyright 2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#ifndef OPTIMIZATION__OPTIMIZER_OPTIONS_HPP_
#define OPTIMIZATION__OPTIMIZER_OPTIONS_HPP_

#include <optimization/visibility_control.hpp>
#include <limits>
#include <cstdint>

namespace autoware
{
namespace common
{
namespace optimization
{
/// \brief Type of termination at the end of an optimization loop.
enum class TerminationType
{
  // Optimizer could converge according to the criteria specified in the used
  CONVERGENCE = 0U,
  // Optimizer finished without reaching convergence.
  NO_CONVERGENCE = 1U,
  // An error occurred during optimization and the result is not usable.
  // Usually indicates a numerical issue.
  FAILURE = 2U
};

// Optimization options base class.
class OPTIMIZATION_PUBLIC OptimizationOptionsBase
{
// TODO(yunus.caliskan, zoza): Add a switch for MINIMIZATION/MAXIMIZATION. Gather common params.
};

// Optimization options class for newton's method.
class OPTIMIZATION_PUBLIC NewtonOptimizationOptions : public OptimizationOptionsBase
{
public:
  /// Constructor to initialize const members
  /// \param max_num_iterations maximum number of iterations
  /// \param function_tolerance minimum relative change in the cost function.
  /// \param parameter_tolerance minimum step size relative to the parameter's norm.
  /// \param gradient_tolerance minimum absolute change in the gradient.
  /// \throws std::domain_error on negative or NaN tolerance values.
  NewtonOptimizationOptions(
    uint64_t max_num_iterations = std::numeric_limits<int64_t>::max(),
    double function_tolerance = 0.0, double parameter_tolerance = 0.0,
    double gradient_tolerance = 0.0);

  /// Get maximum number of iterations
  uint64_t max_num_iterations() const noexcept;
  /// Get minimum relative change in the cost function
  double function_tolerance() const noexcept;
  /// Get minimum relative change in the parameter
  double parameter_tolerance() const noexcept;
  /// Get minimum relative change in the gradient
  double gradient_tolerance() const noexcept;

private:
  uint64_t m_max_num_iterations;
  double m_function_tolerance;
  double m_parameter_tolerance;
  double m_gradient_tolerance;
};

// Optimization summary class.
class OPTIMIZATION_PUBLIC OptimizationSummary
{
public:
  /// Constructor to initialize const members
  /// \param dist estimated distance to the optimum
  /// \param termination_type Type of termination. Check the enum definition for possible outcomes.
  /// \param iter number of iterations that were made
  OptimizationSummary(double dist, TerminationType termination_type, uint64_t iter);

  /// Get the estimated distance to the optimum
  double estimated_distance_to_optimum() const noexcept;
  /// Get termination type.
  TerminationType termination_type() const noexcept;
  /// Get the number of iterations that were made
  uint64_t number_of_iterations_made() const noexcept;

private:
  double m_estimated_distance_to_optimum;
  uint64_t m_number_of_iterations_made;
  TerminationType m_termination_type;
};

}  // namespace optimization
}  // namespace common
}  // namespace autoware

#endif  // OPTIMIZATION__OPTIMIZER_OPTIONS_HPP_
