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
#ifndef OPTIMIZATION__OPTIMIZATION_PROBLEM_HPP_
#define OPTIMIZATION__OPTIMIZATION_PROBLEM_HPP_

#include <optimization/visibility_control.hpp>
#include <Eigen/Core>
#include <vector>
#include <cstddef>
#include <memory>
#include <tuple>

namespace autoware
{
namespace common
{
namespace optimization
{
// Mathematical expression base class (CRTP for performance)
template<typename Derived, typename DomainValueT, int NumJacobianRowsT, int NumVarsT>
class OPTIMIZATION_PUBLIC Expression
{
protected:
public:
  static constexpr auto NumJacobianRows = NumJacobianRowsT;
  static constexpr auto NumVars = NumVarsT;
  using DomainValue = DomainValueT;
  using Value = double;
  using Jacobian = Eigen::Matrix<Value, NumJacobianRows, NumVars>;
  using Hessian = Eigen::Matrix<Value, NumVars, NumVars>;


  Value operator()(const DomainValue & x)
  {
    return impl()(x);
  }
  Jacobian & jacobian(const DomainValue & x)
  {
    return impl().jacobian(x);
  }

  Hessian & hessian(const DomainValue & x)
  {
    return impl().hessian(x);
  }

private:
  const Derived & impl() const
  {
    return *static_cast<const Derived *>(this);
  }
};

// This is a generalized representation of an optimization problem. (CRTP base class)
template<typename Derived, typename DomainValueT, int NumJacobianRowsT, int NumVarsT,
  typename ObjectiveT, typename EqualityConstraintsT, typename InequalityConstraintsT>
class OPTIMIZATION_PUBLIC OptimizationProblem;


// Definition of OptimizationProblem.
//1. Since Optimization problem is just a collection of expressions, it's an expression itself as well and hence inherits
//      and implements the `Expression` class. (Uses identical parameters to its objective function.)
//2. Template specialization is used for type deduction and forwarding constraint parameter packs to the tuples.
// This way "std::tuple" is enforced to be used on the constrained OptimizationProblem implementations.

template<typename Derived,
  typename ... EqualityConstraints,
  typename ... InequalityConstraints,
  typename ObjectiveT, typename DomainValueT, int NumJacobianRowsT, int NumVarsT>
class OPTIMIZATION_PUBLIC OptimizationProblem<Derived, DomainValueT, NumJacobianRowsT, NumVarsT, ObjectiveT,
    std::tuple<EqualityConstraints...>,
    std::tuple<InequalityConstraints...>>
  : public Expression<
    OptimizationProblem<Derived, DomainValueT, NumJacobianRowsT, NumVarsT, ObjectiveT,
    std::tuple<EqualityConstraints...>,
    std::tuple<InequalityConstraints...>>,
    DomainValueT, NumJacobianRowsT, NumVarsT>
{
public:
  // Same types and dimensions used as the objective function.
  using DomainValue = typename ObjectiveT::DomainValue;
  using Value = typename ObjectiveT::Value;
  using Jacobian = typename ObjectiveT::Jacobian;
  using Hessian = typename ObjectiveT::Hessian;

  Value operator()(const DomainValue & x);
  Jacobian & jacobian(const DomainValue & x);
  Hessian & hessian(const DomainValue & x);

  // getters, setters.

private:
  ObjectiveT m_objective;
  std::tuple<EqualityConstraints...> m_equality_constraints;
  std::tuple<InequalityConstraints...> m_inequality_constraints;
};


// convenience alias for defining unconstrained optimization problems.
template<typename Derived, typename DomainValueT, int NumVarsT, typename ObjectiveT>
using UnconstrainedOptimizationProblem = OptimizationProblem<Derived, DomainValueT, 1U, NumVarsT, ObjectiveT, std::tuple<>,
    std::tuple<>>;

}          // namespace autoware
}      // namespace common
}  // namespace optimization

#endif