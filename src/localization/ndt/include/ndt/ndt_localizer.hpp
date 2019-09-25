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

#ifndef NDT__NDT_LOCALIZER_HPP_
#define NDT__NDT_LOCALIZER_HPP_

#include <localization_common/localizer_base.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <ndt/ndt_representations.hpp>
#include <ndt/ndt_optimization_problem.hpp>

namespace autoware{
namespace localization{
namespace ndt{

    using CloudT = sensor_msgs::msg::PointCloud2;
    // Base class for NDT based localizers.
template <typename derived, typename ScanT, typename MapT, typename NDTOptimizationProblemT, typename OptimizerT, typename PoseInitializationT>
class NDTLocalizerBase : localization_common::RelativeLocalizerBase<CloudT, CloudT, PoseInitializationT>{
public:
    using PoseT = geometry_msgs::msg::Transform;

  PoseT register_measurement(const CloudT &msg, const PoseT & initial_guess) override{
//    auto scan = msg_to_scan(msg);
//    auto problem = NDTOptimizationProblemT(initial_guess, scan, m_map);
//    PoseT res;
//    res = m_optimizer.solve(problem, initial_guess, res);
  }

  void set_map(const CloudT & msg) override;

protected:
    ScanT msg_to_scan(const CloudT &);
    MapT msg_to_map(const CloudT &);
private:
    OptimizerT m_optimizer;
    MapT m_map;
};

// this can use dynamic polymorphism as well
template <typename OptimizerT, typename PoseInitializationT>
class P2DNDTLocalizer : public NDTLocalizerBase<P2DNDTLocalizer<OptimizerT, PoseInitializationT>,
        P2DNDTScan, NDTMap, P2DNDTOptimizationProblem, OptimizerT, PoseInitializationT>{
    using CloudT = sensor_msgs::msg::PointCloud2;

    P2DNDTScan msg_to_scan(const CloudT &);
    NDTMap msg_to_map(const CloudT &);
};

}
}
}

#endif