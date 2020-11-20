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

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#ifndef NDT_MAPPING_NODES__NDT_MAPPING_NODES_HPP_
#define NDT_MAPPING_NODES__NDT_MAPPING_NODES_HPP_

#include <ndt_mapping_nodes/visibility_control.hpp>
#include <localization_common/initialization.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <point_cloud_mapping/point_cloud_mapper.hpp>
#include <point_cloud_mapping/policies.hpp>
#include <ndt/ndt_localizer.hpp>
#include <optimization/newtons_method_optimizer.hpp>
#include <localization_nodes/localization_node.hpp>
#include <optimization/line_search/more_thuente_line_search.hpp>
#include <helper_functions/float_comparisons.hpp>

#include <string>
#include <limits>
#include <memory>
#include <utility>

namespace autoware
{
namespace mapping
{
namespace ndt_mapping_nodes
{

// TODO(yunus.caliskan) remove the hard-coded optimizer set up and make it fully configurable
using Optimizer = common::optimization::NewtonsMethodOptimizer<
  common::optimization::MoreThuenteLineSearch>;
using Localizer = localization::ndt::P2DNDTLocalizer<Optimizer, localization::ndt::DynamicNDTMap>;
using P2DNDTConfig = localization::ndt::P2DNDTLocalizerConfig;
using PoseInitializer = localization::localization_common::BestEffortInitializer;
using VoxelMap = point_cloud_mapping::VoxelMap;
using Mapper = point_cloud_mapping::PointCloudMapper<Localizer, VoxelMap,
    point_cloud_mapping::CapacityTrigger, point_cloud_mapping::TimeStampPrefixGenerator>;
using RelativeLocalizerNode = localization::localization_nodes::RelativeLocalizerNode<
  sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, Mapper, PoseInitializer>;


class NDT_MAPPING_NODES_PUBLIC P2DNDTVoxelMapperNode : public RelativeLocalizerNode
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using MapperSummary = typename Mapper::RegistrationSummary;
  using Transform = typename Mapper::Base::TransformStamped;

  // TODO(yunus.caliskan): Probably set the pose initializer explicitly.
  explicit P2DNDTVoxelMapperNode(const rclcpp::NodeOptions & options)
  : RelativeLocalizerNode{"ndt_mapper_node", options, PoseInitializer{}} {init();}

private:
  void init()
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


    const auto parse_grid_config = [this, get_point_param](const std::string & prefix) {
        // Fetch map configuration
        const auto capacity = static_cast<std::size_t>(
          this->declare_parameter(prefix + ".capacity").template get<std::size_t>());

        return perception::filters::voxel_grid::Config{get_point_param(
          prefix + ".min_point"), get_point_param(prefix + ".max_point"),
        get_point_param(prefix + ".voxel_size"), capacity};
      };

    m_predict_translation_threshold =
      this->declare_parameter("predict_pose_threshold.translation")
      .template get<autoware::common::types::float64_t>();
    m_predict_rotation_threshold =
      this->declare_parameter("predict_pose_threshold.rotation")
      .template get<autoware::common::types::float64_t>();

    // Fetch localizer configuration
    P2DNDTConfig localizer_config{
      parse_grid_config("localizer.map"),
      static_cast<uint32_t>(this->declare_parameter("localizer.scan.capacity").
      template get<uint32_t>()),
      std::chrono::milliseconds(
        static_cast<uint64_t>(
          this->declare_parameter("localizer.guess_time_tolerance_ms").template get<uint64_t>()))
    };

    const auto outlier_ratio{this->declare_parameter(
        "localizer.optimization.outlier_ratio").template get<float64_t>()};

    const common::optimization::OptimizationOptions optimization_options{
      static_cast<uint64_t>(
        this->declare_parameter("localizer.optimizer.max_iterations").template get<uint64_t>()),
      this->declare_parameter("localizer.optimizer.score_tolerance").template get<float64_t>(),
      this->declare_parameter(
        "localizer.optimizer.parameter_tolerance").template get<float64_t>(),
      this->declare_parameter("localizer.optimizer.gradient_tolerance").template get<float64_t>()
    };

    auto localizer_ptr = std::make_unique<Localizer>(
      localizer_config,
      Optimizer{
            common::optimization::MoreThuenteLineSearch{
              static_cast<float32_t>(this->declare_parameter(
                "localizer.optimizer.line_search.step_max")
              .template get<float32_t>()),
              static_cast<float32_t>(this->declare_parameter(
                "localizer.optimizer.line_search.step_min")
              .template get<float32_t>()),
              common::optimization::MoreThuenteLineSearch::OptimizationDirection::kMaximization
            },
            optimization_options},
      outlier_ratio);
    const auto & map_frame_id = this->declare_parameter("map.frame_id").template get<std::string>();
    VoxelMap map{parse_grid_config("map"), map_frame_id};

    this->set_localizer(
      std::make_unique<Mapper>(
        this->declare_parameter("file_name_prefix").template get<std::string>(),
        std::move(map), std::move(localizer_ptr), map_frame_id
    ));

    if (this->declare_parameter("publish_map_increment").template get<bool8_t>()) {
      m_increment_publisher = this->template create_publisher<sensor_msgs::msg::PointCloud2>(
        "points_registered",
        rclcpp::QoS{rclcpp::KeepLast{
            static_cast<size_t>(this->declare_parameter("map_increment_pub.history_depth").template
            get<size_t>())}});
    }
  }

  bool8_t validate_output(
    const MapperSummary & summary,
    const PoseWithCovarianceStamped & pose,
    const Transform & guess) override
  {
    bool8_t ret = true;
    if (summary.localizer_summary) {
      switch (summary.localizer_summary->optimization_summary().termination_type()) {
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
      ret = translation_valid(pose, guess) && rotation_valid(pose, guess);
    }

    return ret;
  }

  bool8_t on_non_convergence(
    const MapperSummary &,
    const PoseWithCovarianceStamped &, const Transform &)
  {
    // In practice, it's hard to come up with a perfect termination criterion for ndt
    // optimization and even non-convergence may be a decent effort in localizing the
    // vehicle. Hence the result is not discarded on non-convergence.
    RCLCPP_DEBUG(this->get_logger(), "NDT mapper optimizer failed to converge.");
    return true;
  }

  /// Check if translation of pose estimate is within the allowed range from the initial guess.
  /// \param pose NDT pose estimate.
  /// \param guess Initial guess for the localizer.
  /// \return True if translation estimate is valid.
  bool8_t translation_valid(
    const PoseWithCovarianceStamped & pose,
    const Transform & guess)
  {
    Eigen::Vector3d pose_translation{pose.pose.pose.position.x,
      pose.pose.pose.position.y,
      pose.pose.pose.position.z};
    Eigen::Vector3d guess_translation{guess.transform.translation.x,
      guess.transform.translation.y,
      guess.transform.translation.z};
    Eigen::Vector3d diff = pose_translation - guess_translation;
    return common::comp::abs_lte(
      diff.norm(), m_predict_translation_threshold,
      std::numeric_limits<autoware::common::types::float64_t>::epsilon());
  }

  /// Check if rotation of pose estimate is within the allowed range from the initial guess.
  /// \param pose NDT pose estimate.
  /// \param guess Initial guess for the localizer.
  /// \return True if rotation estimate is valid.
  bool8_t rotation_valid(
    const PoseWithCovarianceStamped & pose,
    const Transform & guess)
  {
    Eigen::Quaterniond pose_rotation{
      pose.pose.pose.orientation.w,
      pose.pose.pose.orientation.x,
      pose.pose.pose.orientation.y,
      pose.pose.pose.orientation.z
    };
    Eigen::Quaterniond guess_rotation{
      guess.transform.rotation.x,
      guess.transform.rotation.x,
      guess.transform.rotation.y,
      guess.transform.rotation.z
    };
    return common::comp::abs_lte(
      pose_rotation.angularDistance(guess_rotation), m_predict_rotation_threshold,
      std::numeric_limits<autoware::common::types::float64_t>::epsilon());
  }

  void handle_registration_summary(const MapperSummary & summary) override
  {
    if (m_increment_publisher && summary.map_increment) {
      m_increment_publisher->publish(*summary.map_increment);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_increment_publisher{nullptr};
  autoware::common::types::float64_t m_predict_translation_threshold{};
  autoware::common::types::float64_t m_predict_rotation_threshold{};
};

}  // namespace ndt_mapping_nodes
}  // namespace mapping
}  // namespace autoware

#endif  // NDT_MAPPING_NODES__NDT_MAPPING_NODES_HPP_
