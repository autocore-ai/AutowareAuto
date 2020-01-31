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

#ifndef OPTIMIZATION__OPTIMIZER_HPP_
#define OPTIMIZATION__OPTIMIZER_HPP_

#include <optimization/optimization_problem.hpp>
#include <Eigen/SVD>
#include <limits>
#include <memory>

namespace autoware
{
namespace common
{
namespace optimization
{

enum class OptimizationDirection
{
  TOWARDS_ZERO,
  AWAY_FROM_ZERO
};

// Optimization options base class.
class OptimizationOptionsBase
{
public:
  /// Constructor to initialize const members
  /// \param dir towards or away from 0
  explicit OptimizationOptionsBase(OptimizationDirection dir = OptimizationDirection::TOWARDS_ZERO)
  : m_optimization_direction(dir)
  {}

  /// Getter for the optimization direction
  /// \return The direction of the optimization
  OptimizationDirection optimization_direction() const
  {
    return m_optimization_direction;
  }

private:
  // Can be set to go towards or away from 0
  OptimizationDirection m_optimization_direction;
  // TODO(zozen): maybe add functor for custom convergece criteria
};

// Optimization options class for newton's method.
class NewtonOptimizationOptions : public OptimizationOptionsBase
{
public:
  /// Constructor to initialize const members
  /// \param dir towards or away from 0
  /// \param max_num_iterations maximum number of iterations
  /// \param function_tolerance minimum relative change in the cost function, disabled if < 0
  /// \param parameter_tolerance minimum step size relative to the parameter's norm, disabled if < 0
  /// \param gradient_tolerance minimum absolute change in the gradient, disabled if < 0
  NewtonOptimizationOptions(
    OptimizationDirection dir = OptimizationDirection::TOWARDS_ZERO,
    uint64_t max_num_iterations = std::numeric_limits<int64_t>::max(),
    double function_tolerance = 0.0, double parameter_tolerance = 0.0,
    double gradient_tolerance = 0.0)
  : OptimizationOptionsBase(dir), m_max_num_iterations(max_num_iterations),
    m_function_tolerance(function_tolerance), m_parameter_tolerance(parameter_tolerance),
    m_gradient_tolerance(gradient_tolerance)
  {
    if ((m_function_tolerance < 0.0) ||
      (m_parameter_tolerance < 0.0) ||
      (m_gradient_tolerance < 0.0))
    {
      throw std::domain_error("Tolerance values must be positive.");
    }
  }
  uint64_t max_num_iterations() const {return m_max_num_iterations;}
  double function_tolerance() const {return m_function_tolerance;}
  double parameter_tolerance() const {return m_parameter_tolerance;}
  double gradient_tolerance() const {return m_gradient_tolerance;}

private:
  // Maximum number of iterations
  uint64_t m_max_num_iterations;
  // Minimum relative change in the cost function, disabled if <= 0
  double m_function_tolerance;
  // Minimum relative change in the parameter, disabled if <= 0
  double m_parameter_tolerance;
  // Minimum relative change in the gradient, disabled if <= 0
  double m_gradient_tolerance;
};

// Optimization summary class.
class OptimizationSummary
{
public:
  /// Constructor to initialize const members
  /// \param dist estimated distance to the optimum
  /// \param conv convergence tolerance criteria passed as options parameters were met
  /// \param iter number of iterations that were made
  OptimizationSummary(double dist, bool conv, uint64_t iter)
  : m_estimated_distance_to_optimum(dist),
    m_convergence_tolerance_criteria_met(conv),
    m_number_of_iterations_made(iter)
  {}

  double estimated_distance_to_optimum() const
  {
    return m_estimated_distance_to_optimum;
  }
  bool convergence_tolerance_criteria_met() const
  {
    return m_convergence_tolerance_criteria_met;
  }
  uint64_t number_of_iterations_made() const
  {
    return m_number_of_iterations_made;
  }

private:
  // The estimated distance to the optimum
  double m_estimated_distance_to_optimum;
  // The convergence tolerance criteria passed as options parameters were met
  bool m_convergence_tolerance_criteria_met;
  // The number of iterations that were made
  uint64_t m_number_of_iterations_made;
};

// Optimization solver base class(CRTP) for a given optimization problem.
template<typename Derived>
class OPTIMIZATION_PUBLIC Optimizer
{
public:
  /// Solves `x_out` for an objective `optimziation_problem` and an initial value `x0`
  /// \param optimization_problem optimization_problem optimization objective
  /// \param x0 initial value
  /// \param x_out optimized value
  /// \param options optimization options
  /// \return summary object
  template<typename OptimizationProblemT, typename DomainValueT>
  OptimizationSummary solve(
    OptimizationProblemT & optimization_problem,
    const DomainValueT & x0, DomainValueT & x_out,
    const OptimizationOptionsBase * const options)
  {
    return impl().solve_(optimization_problem, x0, x_out, options);
  }

private:
  Derived & impl()
  {
    return *static_cast<Derived *>(this);
  }
};


/// Optimizer using the Newton method with line search
template<typename LineSearchT>
class NewtonsMethod : public Optimizer<NewtonsMethod<LineSearchT>>
{
public:
  using StepT = float_t;

  /// Constructor to initialize the line search method
  explicit NewtonsMethod(LineSearchT & line_searcher)
  : m_line_searcher(line_searcher) {}

  /// Solves `x_out` for an objective `optimization_problem` and an initial value `x0`
  /// \param optimization_problem optimization_problem optimization objective
  /// \param x0 initial value
  /// \param x_out optimized value
  /// \param options optimization options
  /// \return summary object
  template<typename OptimizationProblemT, typename DomainValueT>
  OptimizationSummary solve_(
    OptimizationProblemT & optimization_problem,
    const DomainValueT & x0, DomainValueT & x_out,
    const OptimizationOptionsBase * const options)
  {
    // Get types from the method's templated parameter
    using Value = typename OptimizationProblemT::Value;
    using Jacobian = typename OptimizationProblemT::Jacobian;
    using Hessian = typename OptimizationProblemT::Hessian;

    // Get options
    const auto opt =
      static_cast<const NewtonOptimizationOptions * const>(options);

    // Initialize
    const auto sign = opt->optimization_direction() == OptimizationDirection::TOWARDS_ZERO ? -1 : 1;
    auto converged = false;
    Value x_delta_norm;
    x_out = x0;

    // Get value, Jacobian and Hessian (pre-computed using evaluate)
    optimization_problem.evaluate(x0, ComputeMode{}.set_score().set_jacobian().set_hessian());
    auto score_previous = optimization_problem(x_out);
    Jacobian jacobian;
    optimization_problem.jacobian(x_out, jacobian);
    Hessian hessian;
    optimization_problem.hessian(x_out, hessian);

    // Iterate until convergence, error, or maximum number of iterations
    auto nr_iterations = 0UL;
    for (; nr_iterations < opt->max_num_iterations(); ++nr_iterations) {
      // Find decent direction using Newton's method
      Eigen::JacobiSVD<typename OptimizationProblemT::Hessian>
      sv(hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
      DomainValueT x_delta = sv.solve(sign * jacobian);

      // Check if there was a problem during Eigen's solve()
      x_delta_norm = x_delta.norm();
      if (std::isnan(x_delta_norm)) {break;}

      // Check if converged
      if (x_delta_norm <= opt->gradient_tolerance()) {
        converged = true;
        break;
      }

      // Calculate and apply step length
      x_delta.normalize();
      // TODO(zozen): with guarnteed sufficient decrease as in [More, Thuente 1994]
      // would need partial results passed to optimization_problem before call, as in:
      // computeStepLengthMT (x0, x_delta, x_delta_norm, transformation_epsilon_/2, ...
      // and would pre-compute score/jacobian/hessian as during init in evaluate!
      // also needs the sign to know the direction of optimization?
      const auto step = m_line_searcher.compute_step_length(optimization_problem);
      x_delta *= step;  // TODO(zozen): fabs(step)?
      x_out += x_delta;

      // Check change in parameter
      if (step / x_out.norm() <= opt->parameter_tolerance()) {
        converged = true;
        break;
      }

      // Update value, Jacobian and Hessian (pre-computed using evaluate)
      optimization_problem.evaluate(x0, ComputeMode{}.set_score().set_jacobian().set_hessian());
      auto score = optimization_problem(x_out);
      optimization_problem.jacobian(x_out, jacobian);
      optimization_problem.hessian(x_out, hessian);

      // Check change in cost function
      if (fabs(score - score_previous) / std::fabs(score_previous) <= opt->function_tolerance()) {
        converged = true;
        break;
      }
      score_previous = score;
    }

    // Returning summary consisting of the following three values:
    // estimated_distance_to_optimum, convergence_tolerance_criteria_met, number_of_iterations_made
    return OptimizationSummary(x_delta_norm, converged, nr_iterations);
  }

private:
  // initialize on construction
  LineSearchT m_line_searcher;
};


/// Base class (CRTP) to mange the step length during optimization.
template<typename Derived>
class OPTIMIZATION_PUBLIC LineSearch
{
public:
  // TODO(zozen): should this be forced to be positive?
  using StepT = float_t;

  /// Default constructor initializes maximum step length to the numerically minimal value
  explicit LineSearch(const StepT step_max = std::numeric_limits<StepT>::min())
  {
    m_step_max = step_max;
  }

  /// Computes the optimal step length for the optimization problem
  /// \param optimization_problem optimization objective
  /// \return The resulting step length.
  template<typename OptimizationProblemT>
  StepT compute_step_length(OptimizationProblemT & optimization_problem)
  {
    return impl().compute_step_length_(optimization_problem);
  }

  /// Getter for the maximum step length
  /// \return The maximum step length.
  StepT get_step_max() const noexcept
  {
    return m_step_max;
  }

  /// Setter for the maximum step length
  /// \param step_max the new maximal step length
  void set_step_max(const StepT step_max) noexcept
  {
    m_step_max = step_max;
  }

private:
  StepT m_step_max;

  const Derived & impl() const
  {
    return *static_cast<const Derived *>(this);
  }
};

/// Class to use a fixed step length during optimization.
class FixedLineSearch : public LineSearch<FixedLineSearch>
{
public:
  /// Returns directly the pre-set (maximum) step length
  /// \return The fixed step length.
  template<typename OptimizationProblemT>
  StepT compute_step_length_(OptimizationProblemT &) const
  {
    return get_step_max();
  }
};

}  // namespace optimization
}  // namespace common
}  // namespace autoware

#endif  // OPTIMIZATION__OPTIMIZER_HPP_
