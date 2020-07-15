// Copyright 2020 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef NDT_NODES__NDT_LOCALIZER_NODES_HPP_
#define NDT_NODES__NDT_LOCALIZER_NODES_HPP_

#include <common/types.hpp>
#include <ndt_nodes/visibility_control.hpp>
#include <ndt/ndt_localizer.hpp>
#include <localization_nodes/localization_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <optimization/newtons_method_optimizer.hpp>
#include <optimization/line_search/more_thuente_line_search.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <string>
#include <memory>
#include <limits>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

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
using Optimizer_ =
  common::optimization::NewtonsMethodOptimizer<common::optimization::MoreThuenteLineSearch>;
using OptimizerOptions_ = common::optimization::OptimizationOptions;
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
    PoseInitializerT>
{
public:
  using Localizer = P2DNDTLocalizer<OptimizerT, OptimizerOptionsT>;
  using RegistrationSummary = typename Localizer::RegistrationSummary;
  using LocalizerBasePtr = std::unique_ptr<localization_common::RelativeLocalizerBase<CloudMsg,
      CloudMsg, RegistrationSummary>>;
  using ParentT = localization_nodes::RelativeLocalizerNode<CloudMsg, CloudMsg,
      Localizer, PoseInitializerT>;
  using PoseWithCovarianceStamped = typename Localizer::PoseWithCovarianceStamped;
  using Transform = typename Localizer::Transform;

  using EigTranslation = Eigen::Vector3d;
  using EigRotation = Eigen::Quaterniond;
  static constexpr auto EPS = std::numeric_limits<ndt::Real>::epsilon();

  /// Constructor
  /// \param node_name node name
  /// \param name_space node namespace
  /// \param pose_initializer pose initializer to use.
  P2DNDTLocalizerNode(
    const std::string & node_name,
    const std::string & name_space,
    const PoseInitializerT & pose_initializer)
  : ParentT(node_name, name_space, pose_initializer),
    m_predict_translation_threshold{
      this->declare_parameter("predict_pose_threshold.translation").template get<double>()},
    m_predict_rotation_threshold{
      this->declare_parameter("predict_pose_threshold.rotation").template get<double>()}
  {
    auto get_point_param = [this](const std::string & config_name_prefix) {
        perception::filters::voxel_grid::PointXYZ point;
        point.x = static_cast<float32_t>(this->declare_parameter(config_name_prefix + ".x").
          template get<float32_t>());
        point.y = static_cast<float32_t>(this->declare_parameter(config_name_prefix + ".y").
          template get<float32_t>());
        point.z = static_cast<float32_t>(this->declare_parameter(config_name_prefix + ".z").
          template get<float32_t>());
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
          "localizer.optimization.outlier_ratio").template get<float64_t>()},
      OptimizerOptionsT{
        static_cast<uint64_t>(
          this->declare_parameter("localizer.optimizer.max_iterations").template get<uint64_t>()),
        this->declare_parameter("localizer.optimizer.score_tolerance").template get<float64_t>(),
        this->declare_parameter(
          "localizer.optimizer.parameter_tolerance").template get<float64_t>(),
        this->declare_parameter("localizer.optimizer.gradient_tolerance").template get<float64_t>()
      },
      map_config,
      static_cast<uint32_t>(this->declare_parameter("localizer.scan.capacity").
      template get<uint32_t>()),
      std::chrono::milliseconds(static_cast<uint64_t>(
          this->declare_parameter("localizer.guess_time_tolerance_ms").template get<uint64_t>()))
    };

    // Construct and set the localizer.
    const float32_t step_max{static_cast<float32_t>(this->declare_parameter(
        "localizer.optimizer.line_search.step_max").
      template get<float64_t>())};
    const float32_t step_min{static_cast<float32_t>(this->declare_parameter(
        "localizer.optimizer.line_search.step_min").
      template get<float64_t>())};
    // TODO(igor): make the line search configurable.
    LocalizerBasePtr localizer_ptr = std::make_unique<Localizer>(
      localizer_config, OptimizerT{
            common::optimization::MoreThuenteLineSearch{
              step_max, step_min,
              common::optimization::MoreThuenteLineSearch::OptimizationDirection::kMaximization},
            localizer_config.optimizer_options()
          });
    this->set_localizer(std::move(localizer_ptr));
  }

protected:
  bool validate_output(
    const RegistrationSummary & summary,
    const PoseWithCovarianceStamped & pose, const Transform & guess) override
  {
    bool ret = true;
    switch (summary.optimization_summary().termination_type()) {
      case common::optimization::TerminationType::FAILURE:
        // Numerical failure, result is unusable.
        ret = false;
        break;
      case common::optimization::TerminationType::NO_CONVERGENCE:
        ret = on_non_convergence(summary, pose, guess);
        break;
      default:
        break;
    }
    if (ret) {
      // Check if translation is valid
      ret = translation_valid(pose, guess);
    }
    if (ret) {
      // Check if rotation is valid
      ret = rotation_valid(pose, guess);
    }
    return ret;
  }

private:
  virtual bool on_non_convergence(
    const RegistrationSummary &,
    const PoseWithCovarianceStamped &, const Transform &)
  {
    // In practice, it's hard to come up with a perfect termination criterion for ndt
    // optimization and even non-convergence may be a decent effort in localizing the
    // vehicle. Hence the result is not discarded on non-convergence.
    RCLCPP_DEBUG(this->get_logger(), "Localizer optimizer failed to converge.");
    return true;
  }

  /// Check if translation of pose estimate is within the allowed range from the initial guess.
  /// \param pose NDT pose estimate.
  /// \param guess Initial guess for the localizer.
  /// \return True if translation estimate is valid.
  virtual bool translation_valid(const PoseWithCovarianceStamped & pose, const Transform guess)
  {
    EigTranslation pose_translation{pose.pose.pose.position.x,
      pose.pose.pose.position.y,
      pose.pose.pose.position.z};
    EigTranslation guess_translation{guess.transform.translation.x,
      guess.transform.translation.y,
      guess.transform.translation.z};
    EigTranslation diff = pose_translation - guess_translation;
    return diff.norm() <= (m_predict_translation_threshold + EPS);
  }

  /// Check if rotation of pose estimate is within the allowed range from the initial guess.
  /// \param pose NDT pose estimate.
  /// \param guess Initial guess for the localizer.
  /// \return True if rotation estimate is valid.
  virtual bool rotation_valid(const PoseWithCovarianceStamped & pose, const Transform guess)
  {
    EigRotation pose_rotation{pose.pose.pose.orientation.w,
      pose.pose.pose.orientation.x,
      pose.pose.pose.orientation.y,
      pose.pose.pose.orientation.z
    };
    EigRotation guess_rotation{guess.transform.rotation.x,
      guess.transform.rotation.x,
      guess.transform.rotation.y,
      guess.transform.rotation.z
    };
    return std::fabs(pose_rotation.angularDistance(guess_rotation)) <=
           (m_predict_rotation_threshold + EPS);
  }

  ndt::Real m_predict_translation_threshold;
  ndt::Real m_predict_rotation_threshold;
};
}  // namespace ndt_nodes
}  // namespace localization
}  // namespace autoware

#endif  // NDT_NODES__NDT_LOCALIZER_NODES_HPP_
