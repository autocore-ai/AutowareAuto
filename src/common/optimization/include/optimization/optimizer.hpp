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
#include <optimization/optimizer_options.hpp>
#include <optimization/line_search.hpp>
#include <Eigen/SVD>
#include <limits>
#include <memory>
#include <cmath>

namespace autoware
{
namespace common
{
namespace optimization
{
// Optimization solver base class(CRTP) for a given optimization problem.
template<typename Derived>
class OPTIMIZATION_PUBLIC Optimizer : public common::helper_functions::crtp<Derived>
{
public:
  /// Solves `x_out` for an objective `optimization_problem` and an initial value `x0`
  /// \tparam OptimizationProblemT Optimization problem type. Must be an
  /// implementation of `common::optimization::OptimizationProblem`.
  /// \tparam DomainValueT Type of the parameter
  /// \tparam EigenSolverT Type of eigen solver to be used internallt for solving the
  /// necessary linear equations. By default set to `Eigen::LDLT`.
  /// \param optimization_problem optimization_problem optimization objective
  /// \param x0 initial value
  /// \param x_out optimized value
  /// \param options optimization options
  /// \return summary object
  template<typename OptimizationProblemT, typename DomainValueT, typename OptionsT,
    typename EigenSolverT = Eigen::LDLT<typename OptimizationProblemT::Hessian>>
  OptimizationSummary solve(
    OptimizationProblemT & optimization_problem,
    const DomainValueT & x0, DomainValueT & x_out,
    const OptionsT & options)
  {
    return this->impl().template solve_<OptimizationProblemT, DomainValueT, EigenSolverT>(
      optimization_problem, x0, x_out, options);
  }
};


/// Optimizer using the Newton method with line search
template<typename LineSearchT>
class NewtonsMethod : public Optimizer<NewtonsMethod<LineSearchT>>
{
public:
  using StepT = float_t;

  /// Constructor to initialize the line search method
  explicit NewtonsMethod(const LineSearchT & line_searcher)
  : m_line_searcher(line_searcher) {}

  /// Solves `x_out` for an objective `optimization_problem` and an initial value `x0`
  /// \tparam OptimizationProblemT Optimization problem type. Must be an
  /// implementation of `common::optimization::OptimizationProblem`.
  /// \tparam DomainValueT Type of the parameter
  /// \tparam EigenSolverT Type of eigen solver to be used internallt for solving the
  /// necessary linear equations. By default set to `Eigen::LDLT`.
  /// \param optimization_problem optimization_problem optimization objective
  /// \param x0 initial value
  /// \param x_out optimized value
  /// \param options optimization options
  /// \return summary object
  template<typename OptimizationProblemT, typename DomainValueT, typename EigenSolverT>
  OptimizationSummary solve_(
    OptimizationProblemT & optimization_problem,
    const DomainValueT & x0, DomainValueT & x_out,
    const NewtonOptimizationOptions & options)
  {
    // Get types from the method's templated parameter
    using Value = typename OptimizationProblemT::Value;
    using Jacobian = typename OptimizationProblemT::Jacobian;
    using Hessian = typename OptimizationProblemT::Hessian;
    TerminationType termination_type{TerminationType::NO_CONVERGENCE};

    // Initialize
    Value x_delta_norm{0.0};
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
    for (; nr_iterations < options.max_num_iterations(); ++nr_iterations) {
      if (!x_out.allFinite() || !jacobian.allFinite() || !hessian.allFinite()) {
        termination_type = TerminationType::FAILURE;
        break;
      }
      // Find decent direction using Newton's method
      EigenSolverT solver(hessian);
      DomainValueT x_delta = solver.solve(-jacobian);

      // Check if there was a problem during Eigen's solve()
      x_delta_norm = x_delta.norm();
      if (!std::isfinite(x_delta_norm)) {
        termination_type = TerminationType::FAILURE;
        break;
      }

      // TODO(yunus.caliskan): Probably copy CERES gradient check.
      // // Check if converged
      // if (x_delta_norm <= options.gradient_tolerance()) {
      //   termination_type = TerminationType::CONVERGENCE;
      //   break;
      //   }

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

      // Check change in parameter relative to the parameter value
      // tolerance added to the norm for stability when the norm is close to 0
      // (Inspired from https://github.com/ceres-solver/ceres-solver/blob/4362a2169966e08394252098
      // c80d1f26764becd0/include/ceres/tiny_solver.h#L244)
      const auto parameter_tolerance =
        options.parameter_tolerance() * (x_out.norm() + options.parameter_tolerance());
      if (x_delta.norm() < parameter_tolerance) {
        termination_type = TerminationType::CONVERGENCE;
        break;
      }

      // Update value, Jacobian and Hessian (pre-computed using evaluate)
      optimization_problem.evaluate(x0, ComputeMode{}.set_score().set_jacobian().set_hessian());
      const auto score = optimization_problem(x_out);
      optimization_problem.jacobian(x_out, jacobian);
      optimization_problem.hessian(x_out, hessian);

      // Check change in cost function
      if (fabs(score - score_previous) / std::fabs(score_previous) <=
        options.function_tolerance())
      {
        termination_type = TerminationType::CONVERGENCE;
        break;
      }

      score_previous = score;
    }

    // Returning summary consisting of the following three values:
    // estimated_distance_to_optimum, convergence_tolerance_criteria_met, number_of_iterations_made
    return OptimizationSummary{x_delta_norm, termination_type, nr_iterations};
  }

private:
  // initialize on construction
  LineSearchT m_line_searcher;
};
}  // namespace optimization
}  // namespace common
}  // namespace autoware

#endif  // OPTIMIZATION__OPTIMIZER_HPP_
