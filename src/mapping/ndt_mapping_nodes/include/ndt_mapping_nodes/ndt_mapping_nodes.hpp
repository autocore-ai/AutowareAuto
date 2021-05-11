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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <point_cloud_mapping/point_cloud_map.hpp>
#include <point_cloud_mapping/policies.hpp>
#include <ndt/ndt_localizer.hpp>
#include <optimization/newtons_method_optimizer.hpp>
#include <optimization/line_search/more_thuente_line_search.hpp>
#include <helper_functions/float_comparisons.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

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
using Optimizer = common::optimization::NewtonsMethodOptimizer<
  common::optimization::MoreThuenteLineSearch>;
using NDTMap = localization::ndt::DynamicNDTMap;
using VoxelMap = point_cloud_mapping::DualVoxelMap<NDTMap>;
using Localizer = localization::ndt::P2DNDTLocalizer<Optimizer, NDTMap>;
using P2DNDTConfig = localization::ndt::P2DNDTLocalizerConfig;
using WritePolicy = mapping::point_cloud_mapping::CapacityTrigger;
using ClearPolicy = mapping::point_cloud_mapping::CapacityTrigger;
using PrefixPolicy = mapping::point_cloud_mapping::TimeStampPrefixGenerator;

/// \brief Mapper node implementation that localizes using a `P2DNDTLocalizer` and accumulates
/// the registered scans into a `DualVoxelMap`.
/// \tparam WriteTriggerPolicyT Policy specifying when to write the map into a file
/// \tparam ClearTriggerPolicyT Policy specifying when to clear the map.
/// \tparam PrefixGeneratorT Functor that generates the full filename prefix given a base prefix.
template<class WriteTriggerPolicyT = WritePolicy,
  class ClearTriggerPolicyT = ClearPolicy,
  class PrefixGeneratorT = PrefixPolicy>
class NDT_MAPPING_NODES_PUBLIC P2DNDTVoxelMapperNode : public rclcpp::Node
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Cloud = sensor_msgs::msg::PointCloud2;
  using RegistrationSummary = localization::localization_common::OptimizedRegistrationSummary;
  using PointXYZI = common::types::PointXYZI;
  // Static asserts to make sure the policies are valid
  static_assert(
    std::is_base_of<mapping::point_cloud_mapping::TriggerPolicyBase<WriteTriggerPolicyT>,
    WriteTriggerPolicyT>::value,
    "Write trigger policy must implement the `TriggerPolicyBase` interface.");

  static_assert(
    std::is_base_of<mapping::point_cloud_mapping::TriggerPolicyBase<ClearTriggerPolicyT>,
    ClearTriggerPolicyT>::value,
    "Clear trigger policy must implement the `TriggerPolicyBase` interface.");

  static_assert(
    std::is_base_of<mapping::point_cloud_mapping::PrefixGeneratorBase<PrefixGeneratorT>,
    PrefixGeneratorT>::value,
    "Prefix generator policy must implement the `PrefixGeneratorBase` interface.");


  explicit P2DNDTVoxelMapperNode(const rclcpp::NodeOptions & options)
  : Node{"ndt_mapper_node", options},
    m_observation_sub{create_subscription<Cloud>(
        "points_in",
        rclcpp::QoS{rclcpp::KeepLast{
            static_cast<size_t>(declare_parameter("observation_sub.history_depth").template
            get<size_t>())}},
        [this](typename Cloud::ConstSharedPtr msg) {observation_callback(msg);})},
    m_pose_publisher(
      create_publisher<PoseWithCovarianceStamped>(
        "ndt_pose",
        rclcpp::QoS{rclcpp::KeepLast{
            static_cast<size_t>(declare_parameter(
              "pose_pub.history_depth").template get<size_t>())}})),
    m_base_fn_prefix{this->declare_parameter("file_name_prefix").template get<std::string>()}
  {
    init();
  }

  ~P2DNDTVoxelMapperNode()
  {
    if (m_map_ptr->size() > 0U) {
      const auto & file_name_prefix = m_prefix_generator.get(m_base_fn_prefix);
      RCLCPP_DEBUG(get_logger(), "The map is written to" + file_name_prefix + ".pcd");
      m_map_ptr->write(file_name_prefix);
    }
  }

private:
  void init()
  {
    auto get_point_param = [this](const std::string & config_name_prefix) {
        perception::filters::voxel_grid::PointXYZ point;
        point.x = static_cast<float32_t>(declare_parameter(config_name_prefix + ".x").
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

    // Fetch localizer configuration
    P2DNDTConfig localizer_config{
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

    m_localizer_ptr = std::make_unique<Localizer>(
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
    m_map_ptr = std::make_unique<VoxelMap>(
      parse_grid_config("map"), map_frame_id,
      NDTMap{parse_grid_config("localizer.map")});

    if (this->declare_parameter("publish_map_increment").template get<bool8_t>()) {
      m_increment_publisher = this->template create_publisher<sensor_msgs::msg::PointCloud2>(
        "points_registered",
        rclcpp::QoS{rclcpp::KeepLast{
            static_cast<size_t>(this->declare_parameter("map_increment_pub.history_depth").template
            get<size_t>())}});
    }

    if (declare_parameter("publish_tf").template get<bool>()) {
      m_tf_publisher = create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf",
        rclcpp::QoS{rclcpp::KeepLast{m_pose_publisher->get_queue_size()}});
    }

    m_previous_transform.transform.rotation.set__w(1.0);
    m_previous_transform.header.frame_id = m_map_ptr->frame_id();
    point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> msg_initializer{m_cached_increment,
      map_frame_id};
  }

  void observation_callback(Cloud::ConstSharedPtr msg_ptr)
  {
    try {
      RegistrationSummary summary{};
      m_previous_transform.header.stamp = msg_ptr->header.stamp;

      geometry_msgs::msg::PoseWithCovarianceStamped pose_out;
      if (m_map_ptr->empty()) {
        // If the map is empty, get the initial pose for inserting the increment into the map.
        // If the map is empty in further iterations, then throw as we don't know where to place
        // the scan anymore.
        pose_out = get_initial_pose_once();
      } else {
        // Register the measurement only if there is a valid map.
        pose_out = m_localizer_ptr->register_measurement(
          *msg_ptr, m_previous_transform, m_map_ptr->localizer_map(), &summary);
      }

      if (!validate_output(summary)) {
        RCLCPP_WARN(get_logger(), "Invalid pose estimate. The result is ignored.");
        return;
      }
      // Transform the measurement into the map frame and insert it into the map.
      const auto & increment = get_map_increment(*msg_ptr, pose_out);
      m_pose_publisher->publish(pose_out);

      if (m_increment_publisher) {
        m_increment_publisher->publish(increment);
      }
      if (m_tf_publisher) {
        publish_tf(pose_to_transform(pose_out, msg_ptr->header.frame_id));
      }
      if (m_write_trigger.ready(*m_map_ptr)) {
        const auto & file_name_prefix = m_prefix_generator.get(m_base_fn_prefix);
        m_map_ptr->write(file_name_prefix);
        RCLCPP_DEBUG(get_logger(), "The map is written to" + file_name_prefix + ".pcd");
      }
      if (m_clear_trigger.ready(*m_map_ptr)) {
        RCLCPP_DEBUG(get_logger(), "The map is cleared.");
        m_map_ptr->clear();
      }

      // Update the map after a possible clearance and not before so that the map is never fully
      // empty.
      m_map_ptr->update(increment);
      m_previous_transform = pose_to_transform(pose_out, msg_ptr->header.frame_id);
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(get_logger(), "Failed to register the measurement: ", e.what());
    }
  }

  bool8_t validate_output(
    const RegistrationSummary & summary)
  {
    switch (summary.optimization_summary().termination_type()) {
      case common::optimization::TerminationType::FAILURE:
        // Numerical failure, result is unusable.
        return false;
      case common::optimization::TerminationType::NO_CONVERGENCE:
        // In practice, it's hard to come up with a perfect termination criterion for ndt
        // optimization and even non-convergence may be a decent effort in localizing the
        // vehicle. Hence the result is not discarded on non-convergence.
        RCLCPP_DEBUG(this->get_logger(), "NDT mapper optimizer failed to converge.");
        return true;
      default:
        return true;
    }
  }

  /// Transform the observed point cloud into the map frame using the registered
  /// pose.
  /// \param observation Point cloud observation.
  /// \param registered_pose Registered pose of the observation.
  /// \return Pointer to the transformed point cloud;
  const Cloud & get_map_increment(
    const Cloud & observation,
    const PoseWithCovarianceStamped & registered_pose)
  {
    point_cloud_msg_wrapper::PointCloud2View<PointXYZI> obs_view{observation};
    reset_cached_msg(obs_view.size());
    // Convert pose to transform for `doTransform()`
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = registered_pose.header.stamp;
    tf.header.frame_id = registered_pose.header.frame_id;
    tf.child_frame_id = observation.header.frame_id;
    const auto & trans = registered_pose.pose.pose.position;
    const auto & rot = registered_pose.pose.pose.orientation;
    tf.transform.translation.set__x(trans.x).set__y(trans.y).set__z(trans.z);
    tf.transform.rotation.set__x(rot.x).set__y(rot.y).set__z(rot.z).set__w(rot.w);

    tf2::doTransform(observation, m_cached_increment, tf);
    return m_cached_increment;
  }

  /// Publish the given transform
  void publish_tf(const geometry_msgs::msg::TransformStamped & transform)
  {
    tf2_msgs::msg::TFMessage tf_message;
    tf_message.transforms.emplace_back(transform);
    m_tf_publisher->publish(tf_message);
  }

  /// Clear the cached pc2 message used for storing the transformed point clouds
  void reset_cached_msg(std::size_t size)
  {
    point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> inc_modifier{m_cached_increment};
    inc_modifier.clear();
    inc_modifier.resize(size);
  }

  /// Convert a `PoseWithCovarianceStamped` into a `TransformStamped`
  geometry_msgs::msg::TransformStamped pose_to_transform(
    const PoseWithCovarianceStamped & pose_msg,
    const std::string & child_frame_id = "base_link")
  {
    geometry_msgs::msg::TransformStamped tf_stamped{};
    tf_stamped.header = pose_msg.header;
    tf_stamped.child_frame_id = child_frame_id;
    const auto & tf_trans = pose_msg.pose.pose.position;
    const auto & tf_rot = pose_msg.pose.pose.orientation;
    tf_stamped.transform.translation.set__x(tf_trans.x).set__y(tf_trans.y).
    set__z(tf_trans.z);
    tf_stamped.transform.rotation.set__x(tf_rot.x).set__y(tf_rot.y).set__z(tf_rot.z).
    set__w(tf_rot.w);
    return tf_stamped;
  }

  /// Convert a `TransformStamped` into a `PoseWithCovarianceStamped`
  PoseWithCovarianceStamped transform_to_pose(
    const geometry_msgs::msg::TransformStamped & transform_msg)
  {
    PoseWithCovarianceStamped pose;
    pose.header = transform_msg.header;
    const auto & pose_trans = transform_msg.transform.translation;
    const auto & pose_rot = transform_msg.transform.rotation;
    pose.pose.pose.position.set__x(pose_trans.x).set__y(pose_trans.y).
    set__z(pose_trans.z);
    pose.pose.pose.orientation.set__x(pose_rot.x).set__y(pose_rot.y).set__z(pose_rot.z).
    set__w(pose_rot.w);
    return pose;
  }

  /// Get the initial pose (identity). If the map is already initialized once, throw.
  geometry_msgs::msg::PoseWithCovarianceStamped get_initial_pose_once()
  {
    if (!m_map_initialized) {
      m_map_initialized = true;
      return transform_to_pose(m_previous_transform);
    } else {
      throw std::runtime_error(
              "Current map is initialized yet empty, the scan cannot be "
              "registered.");
    }
  }

  std::unique_ptr<Localizer> m_localizer_ptr;
  std::unique_ptr<VoxelMap> m_map_ptr;
  typename rclcpp::Subscription<Cloud>::SharedPtr m_observation_sub;
  typename rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr m_pose_publisher;
  typename rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr m_tf_publisher{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_increment_publisher{nullptr};
  geometry_msgs::msg::TransformStamped m_previous_transform;
  Cloud m_cached_increment;
  WriteTriggerPolicyT m_write_trigger{};
  ClearTriggerPolicyT m_clear_trigger{};
  PrefixGeneratorT m_prefix_generator{};
  bool8_t m_map_initialized{false};
  std::string m_base_fn_prefix;
};

}  // namespace ndt_mapping_nodes
}  // namespace mapping
}  // namespace autoware

#endif  // NDT_MAPPING_NODES__NDT_MAPPING_NODES_HPP_
