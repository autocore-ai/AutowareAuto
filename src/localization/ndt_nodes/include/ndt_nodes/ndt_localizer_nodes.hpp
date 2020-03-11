// Copyright 2020 Apex.AI, Inc.
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

#ifndef NDT_NODES__NDT_LOCALIZER_NODES_HPP_
#define NDT_NODES__NDT_LOCALIZER_NODES_HPP_

#include <ndt_nodes/visibility_control.hpp>
#include <ndt/ndt_localizer.hpp>
#include <localization_nodes/localization_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <optimization/optimizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <string>
#include <memory>

namespace autoware
{
namespace localization
{
namespace ndt_nodes
{
// Alias common types.
using CloudMsg = sensor_msgs::msg::PointCloud2;
template<typename ... Types>
using P2DNDTLocalizer = ndt::P2DNDTLocalizer<Types...>;
template<typename ... Types>
using P2DNDTConfig = ndt::P2DNDTLocalizerConfig<Types...>;

// TODO(yunus.caliskan) remove the hard-coded optimizer set up and make it fully configurable
using Optimizer_ = common::optimization::NewtonsMethod<common::optimization::FixedLineSearch>;
using OptimizerOptions_ = common::optimization::NewtonOptimizationOptions;
using PoseInitializer_ = localization_common::BestEffortInitializer;

/// P2D NDT localizer node. Currently uses the hard coded optimizer and pose initializers.
/// \tparam OptimizerT Hard coded for Newton optimizer. TODO(yunus.caliskan): Make Configurable
/// \tparam OptimizerOptionsT Hard coded for Newton optimizer. TODO(yunus.caliskan): Make
/// Configurable
/// \tparam PoseInitializerT Hard coded for Best effort. TODO(yunus.caliskan): Make Configurable
template<typename OptimizerT = Optimizer_,
  typename OptimizerOptionsT = OptimizerOptions_,
  typename PoseInitializerT = PoseInitializer_>
class NDT_NODES_PUBLIC P2DNDTLocalizerNode
  : public localization_nodes::RelativeLocalizerNode<CloudMsg, CloudMsg,
    P2DNDTLocalizer<OptimizerT, OptimizerOptionsT>,
    P2DNDTConfig<OptimizerOptionsT>,
    PoseInitializerT>
{
public:
  using Localizer = P2DNDTLocalizer<OptimizerT, OptimizerOptionsT>;
  using LocalizerBasePtr = std::unique_ptr<
    localization_common::RelativeLocalizerBase<CloudMsg, CloudMsg>>;
  using ParentT = localization_nodes::RelativeLocalizerNode<CloudMsg, CloudMsg,
      Localizer, P2DNDTConfig<OptimizerOptionsT>, PoseInitializerT>;

  /// Constructor
  /// \param node_name node name
  /// \param name_space node namespace
  /// \param pose_initializer pose initializer to use.
  P2DNDTLocalizerNode(
    const std::string & node_name,
    const std::string & name_space,
    const PoseInitializerT & pose_initializer)
  : ParentT(node_name, name_space, pose_initializer)
  {
    auto get_point_param = [this](const std::string & config_name_prefix) {
        perception::filters::voxel_grid::PointXYZ point;
        point.x = static_cast<float>(this->declare_parameter(config_name_prefix + ".x").
          template get<float>());
        point.y = static_cast<float>(this->declare_parameter(config_name_prefix + ".y").
          template get<float>());
        point.z = static_cast<float>(this->declare_parameter(config_name_prefix + ".z").
          template get<float>());
        return point;
      };
    // Fetch map configuration
    const auto capacity = static_cast<std::size_t>(
      this->declare_parameter("localizer.map.capacity").template get<std::size_t>());
    const perception::filters::voxel_grid::Config map_config{get_point_param(
        "localizer.map.min_point"), get_point_param("localizer.map.max_point"),
      get_point_param("localizer.map.voxel_size"), capacity};

    // Fetch localizer configuration
    P2DNDTConfig<OptimizerOptionsT> localizer_config{
      ndt::P2DNDTOptimizationConfig{this->declare_parameter(
          "localizer.optimization.outlier_ratio").template get<double>()},
      OptimizerOptionsT{
        static_cast<uint64_t>(
          this->declare_parameter("localizer.optimizer.max_iterations").template get<uint64_t>()),
        this->declare_parameter("localizer.optimizer.score_tolerance").template get<double>(),
        this->declare_parameter("localizer.optimizer.parameter_tolerance").template get<double>(),
        this->declare_parameter("localizer.optimizer.gradient_tolerance").template get<double>()
      },
      map_config,
      static_cast<uint32_t>(this->declare_parameter("localizer.scan.capacity").
      template get<uint32_t>()),
      std::chrono::milliseconds(static_cast<uint64_t>(
          this->declare_parameter("localizer.guess_time_tolerance_ms").template get<uint64_t>()))
    };

    // Construct and set the localizer.
    LocalizerBasePtr localizer_ptr = std::make_unique<Localizer>(
      localizer_config, OptimizerT{common::optimization::FixedLineSearch{
          static_cast<float>(this->declare_parameter("localizer.optimizer.line_search.step_size").
          template get<float>())}});
    this->set_localizer(std::move(localizer_ptr));
  }
};

}  // namespace ndt_nodes
}  // namespace localization
}  // namespace autoware

#endif  // NDT_NODES__NDT_LOCALIZER_NODES_HPP_
