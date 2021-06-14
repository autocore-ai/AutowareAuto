// Copyright 2021 the Autoware Foundation
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
/// \file
/// \brief Implementation of the localization evaluator
#ifndef LOCALIZATION_SYSTEM_TESTS__LOCALIZATION_EVALUATION_NODE_HPP_
#define LOCALIZATION_SYSTEM_TESTS__LOCALIZATION_EVALUATION_NODE_HPP_

#include <localization_system_tests/visibility_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>
#include <time_utils/time_utils.hpp>
#include <common/types.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>

namespace localization_system_tests
{
/// \brief A node that is ran beside a localization application and a source of ground truth and
// estimates the error with respect to the ground truth.
class LOCALIZATION_SYSTEM_TESTS_PUBLIC LocalizationEvaluationNode
  : public rclcpp::Node
{
public:
  using float64_t = autoware::common::types::float64_t;
  using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
  using EigTransform = Eigen::Transform<float64_t, 3, Eigen::Affine, Eigen::ColMajor>;
  using EigTranslationPart = decltype(std::declval<const EigTransform>().translation());
  using EigRotationPart = decltype(std::declval<const EigTransform>().rotation());
  using SyncPolicyT = message_filters::sync_policies::ApproximateTime<Pose, Pose>;
  /// \brief Basic node constructor.
  explicit LocalizationEvaluationNode(const rclcpp::NodeOptions & options);

  /// \brief Finalize and report the metrics in the destructor.
  ~LocalizationEvaluationNode() noexcept;

private:
  /// \brief Compute the translation and rotation error metrics for the given estimate - ground
  /// truth pair.
  void evaluation_callback(
    const Pose::ConstSharedPtr & estimate, const Pose::ConstSharedPtr & ground_truth);

  /// \brief Given the current average and the increment of a single sample, compute the moving
  /// average using the number of points stored in the class
  /// \param current_avg Current average
  /// \param addition New sample's contribution
  /// \return The new average
  float64_t get_new_average(float64_t current_avg, float64_t addition);

  /// \brief Compute metrics and update the values
  void compute_and_update_metrics(
    const EigTransform & ground_truth,
    const EigTransform & estimate);

  /// Compute the euclidean distance between two translation values.
  float64_t translation_error(const EigTranslationPart & tf1, const EigTranslationPart & tf2);

  /// Compute the angular distance in radian between two rotation matrices
  float64_t rotation_error(const EigRotationPart & tf1, const EigRotationPart & tf2);

  std::array<std::unique_ptr<message_filters::Subscriber<Pose>>, 2U> m_pose_subscribers;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicyT>> m_pose_synchronizer;
  float64_t m_average_translation_err;
  float64_t m_average_rotation_err;
  std::uint32_t m_num_computed{0U};
  EigTransform m_ground_truth_to_estimate_tf;
};  // class LocalizationEvaluationNode

}  // namespace localization_system_tests

#endif  // LOCALIZATION_SYSTEM_TESTS__LOCALIZATION_EVALUATION_NODE_HPP_
