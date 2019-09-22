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

#include <ndt/ndt_representations.hpp>
#include <optimization/optimization_problem.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <Eigen/Core>
#include <tuple>

namespace autoware{
namespace localization{
namespace ndt{

  class P2DNDTObjective : public common::optimization::Expression<P2DNDTObjective, geometry_msgs::msg::Transform, 1U, 6U>{
  // getting aliases from the base class.
  using ExpressionT = Expression<P2DNDTObjective, geometry_msgs::msg::Transform, 1U, 6U>;
  using DomainValue = typename ExpressionT::DomainValue;
  using Value = typename ExpressionT::Value;
  using Jacobian = typename ExpressionT::Jacobian;
  using Hessian = typename ExpressionT::Hessian;


  public:
      P2DNDTObjective(const DomainValue & init_guess, const P2DNDTScan & scan, const NDTMap & map):
      m_init_guess_ref(init_guess),
      m_scan_ref(scan),
      m_map_ref(map){}

  Value operator()(const DomainValue & x);
  Jacobian & jacobian(const DomainValue & x);
  Hessian & hessian(const DomainValue & x);

  private:
  // references as class members to be initialized at constructor.
  const DomainValue & m_init_guess_ref;
  const P2DNDTScan & m_scan_ref;
  const NDTMap & m_map_ref;
};

// Here the class P2DNDTObjective
class P2DNDTOptimizationProblem : public common::optimization::UnconstrainedOptimizationProblem<
        P2DNDTOptimizationProblem, geometry_msgs::msg::Transform, 6U, P2DNDTObjective>{
// Forward everything to the P2DNDTObjective? Possibly unnecessary.
};

}
}
}
