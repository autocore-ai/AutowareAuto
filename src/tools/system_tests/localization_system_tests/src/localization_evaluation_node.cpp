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

#include <localization_system_tests/localization_evaluation_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <memory>
#include <string>

namespace localization_system_tests
{

LocalizationEvaluationNode::LocalizationEvaluationNode(
  const rclcpp::NodeOptions & options)
: Node("localization_evaluator", options), m_pose_subscribers{
    std::make_unique<message_filters::Subscriber<Pose>>(this, "localization/ndt_pose"),
    std::make_unique<message_filters::Subscriber<Pose>>(this, "/vehicle/odom_pose")
}
{
  EigTransform estimate_offset;
  estimate_offset.setIdentity();
  estimate_offset.translate(
    Eigen::Vector3d{
      declare_parameter("estimate_offset.translation.x").get<double>(),
      declare_parameter("estimate_offset.translation.y").get<double>(),
      declare_parameter("estimate_offset.translation.z").get<double>()
    });
  estimate_offset.rotate(
    Eigen::Quaterniond {
      declare_parameter("estimate_offset.rotation.w").get<double>(),
      declare_parameter("estimate_offset.rotation.x").get<double>(),
      declare_parameter("estimate_offset.rotation.y").get<double>(),
      declare_parameter("estimate_offset.rotation.z").get<double>()
    });

  EigTransform gt_offset;
  gt_offset.setIdentity();
  gt_offset.translate(
    Eigen::Vector3d{
      declare_parameter("ground_truth_offset.translation.x").get<double>(),
      declare_parameter("ground_truth_offset.translation.y").get<double>(),
      declare_parameter("ground_truth_offset.translation.z").get<double>()
    });

  estimate_offset.rotate(
    Eigen::Quaterniond {
      declare_parameter("ground_truth_offset.rotation.w").get<double>(),
      declare_parameter("ground_truth_offset.rotation.x").get<double>(),
      declare_parameter("ground_truth_offset.rotation.y").get<double>(),
      declare_parameter("ground_truth_offset.rotation.z").get<double>()
    });

  m_ground_truth_to_estimate_tf = estimate_offset * gt_offset.inverse();

  m_pose_synchronizer = std::make_unique<message_filters::Synchronizer<SyncPolicyT>>(
    SyncPolicyT(50), *m_pose_subscribers[0], *m_pose_subscribers[1]);

  m_pose_synchronizer->registerCallback(
    std::bind(
      &LocalizationEvaluationNode::evaluation_callback, this,
      std::placeholders::_1, std::placeholders::_2));
}

void LocalizationEvaluationNode::evaluation_callback(
  const Pose::ConstSharedPtr & estimate, const Pose::ConstSharedPtr & ground_truth)
{
  EigTransform estimate_eig;
  EigTransform ground_truth_eig;
  EigTransform ground_truth_in_map_frame_eig;

  tf2::fromMsg(estimate->pose.pose, estimate_eig);
  tf2::fromMsg(ground_truth->pose.pose, ground_truth_eig);

  ground_truth_in_map_frame_eig = m_ground_truth_to_estimate_tf * ground_truth_eig;

  compute_and_update_metrics(ground_truth_in_map_frame_eig, estimate_eig);
}

LocalizationEvaluationNode::~LocalizationEvaluationNode() noexcept
{
  std::stringstream report;

  report <<
    "Processed " << m_num_computed << " samples." << std::endl <<
    "Average translation error: " << m_average_translation_err << " meters." << std::endl <<
    "Average rotation error: " << m_average_rotation_err << " radians." << std::endl;
  RCLCPP_INFO(get_logger(), report.str());
}

LocalizationEvaluationNode::float64_t
LocalizationEvaluationNode::get_new_average(float64_t current_avg, float64_t addition)
{
  const auto current_num_samples = static_cast<float64_t>(m_num_computed);
  const auto total_err = (current_avg * current_num_samples) + addition;
  return total_err / (current_num_samples + 1.0);
}

LocalizationEvaluationNode::float64_t LocalizationEvaluationNode::translation_error(
  const LocalizationEvaluationNode::EigTranslationPart & tf1,
  const LocalizationEvaluationNode::EigTranslationPart & tf2)
{
  return (tf1 - tf2).norm();
}

LocalizationEvaluationNode::float64_t LocalizationEvaluationNode::rotation_error(
  const LocalizationEvaluationNode::EigRotationPart & tf1,
  const LocalizationEvaluationNode::EigRotationPart & tf2)
{
  const Eigen::Quaterniond rot1{tf1};
  const Eigen::Quaterniond rot2{tf2};
  return std::fabs(rot1.angularDistance(rot2));
}

void LocalizationEvaluationNode::compute_and_update_metrics(
  const EigTransform & ground_truth, const EigTransform & estimate)
{
  const auto translation_err =
    translation_error(ground_truth.translation(), estimate.translation());
  const auto rotation_err = rotation_error(ground_truth.rotation(), estimate.rotation());

  m_average_translation_err = get_new_average(m_average_translation_err, translation_err);
  m_average_rotation_err = get_new_average(m_average_rotation_err, rotation_err);

  ++m_num_computed;
}
}  // namespace localization_system_tests

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(localization_system_tests::LocalizationEvaluationNode)
