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

#include <common/types.hpp>
#include <gtest/gtest.h>
#include <limits>
#include "test_newton_optimization.hpp"

using autoware::common::types::float64_t;

namespace autoware
{
namespace common
{
namespace optimization
{

// Init
int DummyObjective::sign = -1;
using DummyOptimizationProblem =
  UnconstrainedOptimizationProblem<DummyObjective, Eigen::Matrix<float64_t, 1U, 1U>, 1U>;

TEST_F(TestNewtonOptimization, newton_optimization_validation) {
  // set up varaibles
  uint16_t max_iter = 21;
  constexpr auto step = 0.1F;
  FixedLineSearch fls;
  fls.set_step_max(step);

  // set up optimization
  NewtonsMethod<FixedLineSearch> no(fls);
  DummyOptimizationProblem dummy_optimization_problem;
  DummyOptimizationProblem::DomainValue dummy_x0 = Eigen::Matrix<double, 1U, 1U>::Ones();
  DummyOptimizationProblem::DomainValue dummy_x_out;

  // using function_tolerance and parameter_tolerance does not make sense with fixed step length
  const auto options = NewtonOptimizationOptions(max_iter, 1e-2, 1e-2, 1e-2);
  EXPECT_EQ(options.max_num_iterations(), max_iter);

  // test optimization with both positive and negative objective function
  for (DummyObjective::sign = -1; DummyObjective::sign <= 1; DummyObjective::sign += 2) {
    OptimizationSummary summary = no.solve(dummy_optimization_problem, dummy_x0, dummy_x_out,
        options);
    EXPECT_LE(summary.number_of_iterations_made(), max_iter);
    EXPECT_EQ(summary.termination_type(), TerminationType::CONVERGENCE);
    EXPECT_NEAR(dummy_x_out(0, 0), -1.0, step);
    EXPECT_FLOAT_EQ(dummy_optimization_problem(dummy_x_out), DummyObjective::sign * 1.0);
    EXPECT_LE(summary.estimated_distance_to_optimum(), step);
  }
}

TEST_F(TestFixedLineSearch, fixed_line_search_validation) {
  // set up varaibles
  constexpr auto step = 0.01F;
  DummyOptimizationProblem dummy_optimization_problem;

  // test derived class
  FixedLineSearch fls;
  EXPECT_FLOAT_EQ(fls.get_step_max(), std::numeric_limits<float_t>::min());
  fls.set_step_max(step);
  EXPECT_FLOAT_EQ(fls.compute_step_length(dummy_optimization_problem), step);
}

}  // namespace optimization
}  // namespace common
}  // namespace autoware
