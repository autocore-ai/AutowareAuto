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

#ifndef TEST_NEWTON_OPTIMIZATION_HPP_
#define TEST_NEWTON_OPTIMIZATION_HPP_

#include <optimization/optimizer.hpp>

namespace autoware
{
namespace common
{
namespace optimization
{

class TestNewtonOptimization : public ::testing::Test
{
};

class TestFixedLineSearch : public ::testing::Test
{
};

/// This is the expression for `y = (x + 1)^2 +1`
class DummyObjective : public Expression<DummyObjective,
    Eigen::Matrix<double, 1U, 1U>, 1U, 1U>
{
  // getting aliases from the base class.
  using ExpressionT = Expression<DummyObjective, Eigen::Matrix<double, 1U, 1U>, 1U, 1U>;
  using DomainValue = typename ExpressionT::DomainValue;
  using Value = typename ExpressionT::Value;
  using Jacobian = typename ExpressionT::Jacobian;
  using Hessian = typename ExpressionT::Hessian;
  using JacobianRef = Eigen::Ref<Jacobian>;
  using HessianRef = Eigen::Ref<Hessian>;

public:
  static int sign;

  /// Get the result of an expression for a given parameter value.
  /// \param x Parameter value
  /// \return Evaluated score
  Value operator()(const DomainValue & x)
  {
    return sign * (std::pow(x(0, 0) + 1.0, 2.0) + 1.0);
  }

  /// Get the jacobian at a given parameter value.
  /// \param x Parameter value.
  /// \param out Evaluated jacobian matrix.
  void jacobian(const DomainValue & x, JacobianRef out)
  {
    out(0, 0) = sign * 2.0 * (x(0, 0) + 1.0);
  }

  /// Get the hessian at a given parameter value.
  /// \param x Parameter value.
  /// \param out Evaluated hessian matrix.
  void hessian(const DomainValue & x, HessianRef out)
  {
    out(0, 0) = sign * 2.0;
  }

};

}  // namespace optimization
}  // namespace common
}  // namespace autoware

#endif  // TEST_NEWTON_OPTIMIZATION_HPP_
