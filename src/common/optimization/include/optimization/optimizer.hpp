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

namespace autoware
{
namespace common
{
namespace optimization
{

// Optimization solver base class(CRTP) for a given optimization problem.
template<typename Derived, typename LineSearchT>
class OPTIMIZATION_PUBLIC Optimizer
{
public:
  using StepT = float_t;

  // Solves x for an objective `optimziation_problem` and an initial value `x0`
  template <typename OptimizationProblemT, typename DomainValueT>
  void solve(OptimizationProblemT & optimization_problem, const DomainValueT & x0, DomainValueT & x_out)
  {
    impl().solve(optimization_problem, x0, x_out);
  }

private:
  const Derived & impl() const
  {
    return *static_cast<const Derived *>(this);
  }

  StepT compute_step_length()
  {
    return impl().compute_step_length();
  }

  // initialzie on constructor
  LineSearchT m_line_searcher;
};

template<typename LineSearchT>
class NewtonsMethod : public Optimizer<NewtonsMethod<LineSearchT>, LineSearchT>
{
  template <typename OptimizationProblemT, typename DomainValueT>
  void solve(OptimizationProblemT & optimization_problem, const DomainValueT & x0, DomainValueT & x_out);
};


// Class to mange the step length during optimization.
template <typename Derived>
class OPTIMIZATION_PUBLIC LineSearch{
    using value = double;
    template <typename OptimizationProblemT>
    value compute_step_length(OptimizationProblemT &);
};


}          // namespace autoware
}      // namespace common
}  // namespace optimization

#endif