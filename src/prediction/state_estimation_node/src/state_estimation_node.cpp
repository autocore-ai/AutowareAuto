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

#include <state_estimation_node/state_estimation_node.hpp>

#include <state_estimation_node/time.hpp>
#include <state_estimation_node/measurement_conversion.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.h>

#include <string>
#include <vector>
#include <memory>
#include <limits>
#include <functional>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;

namespace
{
constexpr int kDefaultHistory{10};  // TODO(igor): remove this.
constexpr float64_t kInvalidFrequency{-1.0};  // Frames per second.
const std::chrono::milliseconds kDefaultTimeBetweenUpdates{100LL};
const char kDefaultOutputTopic[]{"filtered_state"};
constexpr auto kCovarianceMatrixRows{6U};
constexpr auto kIndexX{0U};
constexpr auto kIndexY{kCovarianceMatrixRows + 1U};
constexpr auto kCovarianceMatrixRowsSquared{kCovarianceMatrixRows * kCovarianceMatrixRows};
static_assert(std::tuple_size<
    geometry_msgs::msg::PoseWithCovariance::_covariance_type>::value ==
  kCovarianceMatrixRowsSquared, "We expect the covariance matrix to have 36 entries.");

/// Convert the ROS timestamp to chrono time point.
autoware::prediction::GlobalTime
to_time_point(const rclcpp::Time & time)
{
  return autoware::prediction::GlobalTime{
    std::chrono::system_clock::time_point{std::chrono::nanoseconds{time.nanoseconds()}}};
}

void assert_all_entries_positive(const std::vector<float64_t> & entries, const std::string & tag)
{
  for (const auto entry : entries) {
    if (std::isnan(entry) || std::isinf(entry) || entry <= 0) {
      throw std::runtime_error(tag + ": entries must all be positive.");
    }
  }
}

std::chrono::nanoseconds validate_publish_frequency(
  float64_t publish_frequency,
  bool8_t publish_data_driven)
{
  std::chrono::nanoseconds time_between_publish_requests;
  if ((publish_frequency > 0.0) && publish_data_driven) {
    throw std::logic_error(
            "Please provide either 'output_frequency' setting or 'data_driven' one, not both.");
  } else if (publish_data_driven) {
    time_between_publish_requests = kDefaultTimeBetweenUpdates;
  } else if (publish_frequency > 0.0) {
    const float64_t nanoseconds_in_second{
      std::chrono::nanoseconds{std::chrono::seconds{1}}.count()};
    time_between_publish_requests = std::chrono::nanoseconds{
      static_cast<std::uint64_t>(std::floor(nanoseconds_in_second / publish_frequency))};
  } else {
    throw std::logic_error(
            "Please provide either 'output_frequency' setting or 'data_driven' one.");
  }
  return time_between_publish_requests;
}

template<std::int32_t kStateDim, std::int32_t kNoiseDim>
Eigen::Matrix<float32_t, kStateDim, kNoiseDim> create_process_noise_variances(
  const std::vector<float64_t> & position_variance,
  const std::vector<float64_t> & velocity_variance,
  const std::vector<float64_t> & acceleration_variance)
{
  // TODO(igor): this is a placeholder for a more generic implementation that would allow
  // configuring the process noise covariances with more flexibility.
  using autoware::motion::motion_model::ConstantAcceleration;

  if (acceleration_variance.size() != static_cast<size_t>(kNoiseDim)) {
    throw std::logic_error("For now we require a 2D acceleration variance.");
  }
  Eigen::Matrix<float32_t, kStateDim, kNoiseDim> process_noise_variances{
    Eigen::Matrix<float32_t, kStateDim, kNoiseDim>::Zero()};
  if (position_variance.size() == static_cast<size_t>(kNoiseDim)) {
    assert_all_entries_positive(position_variance, "position_variance");
    process_noise_variances(ConstantAcceleration::States::POSE_X, 0) =
      static_cast<float32_t>(position_variance[0]);
    process_noise_variances(ConstantAcceleration::States::POSE_Y, 1) =
      static_cast<float32_t>(position_variance[1]);
  }
  if (velocity_variance.size() == static_cast<size_t>(kNoiseDim)) {
    assert_all_entries_positive(velocity_variance, "velocity_variance");
    process_noise_variances(ConstantAcceleration::States::VELOCITY_X, 0) =
      static_cast<float32_t>(velocity_variance[0]);
    process_noise_variances(ConstantAcceleration::States::VELOCITY_Y, 1) =
      static_cast<float32_t>(velocity_variance[1]);
  }
  assert_all_entries_positive(acceleration_variance, "acceleration_variance");
  process_noise_variances(ConstantAcceleration::States::ACCELERATION_X, 0) =
    static_cast<float32_t>(acceleration_variance[0]);
  process_noise_variances(ConstantAcceleration::States::ACCELERATION_Y, 1) =
    static_cast<float32_t>(acceleration_variance[1]);
  return process_noise_variances;
}

template<std::int32_t kStateDim>
Eigen::Matrix<float32_t, kStateDim, kStateDim> create_state_variances(
  const std::vector<float64_t> & state_variances)
{
  if (state_variances.size() != static_cast<size_t>(kStateDim)) {
    throw std::logic_error("State variances are of wrong size.");
  }
  assert_all_entries_positive(state_variances, "state_variances");
  Eigen::Matrix<float32_t, kStateDim, 1> diagonal;
  diagonal <<
    static_cast<float32_t>(state_variances[0]),
    static_cast<float32_t>(state_variances[1]),
    static_cast<float32_t>(state_variances[2]),
    static_cast<float32_t>(state_variances[3]),
    static_cast<float32_t>(state_variances[4]),
    static_cast<float32_t>(state_variances[5]);
  return diagonal.asDiagonal();
}


}  // namespace

namespace autoware
{
namespace prediction
{
namespace state_estimation_node
{

StateEstimationNode::StateEstimationNode(const rclcpp::NodeOptions & node_options)
: StateEstimationNode{"state_estimation_node", "", node_options} {}

StateEstimationNode::StateEstimationNode(
  const std::string & node_name,
  const std::string & node_namespace,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node{node_name, node_namespace, node_options},
  m_tf_listener(m_tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false)
{
  m_frame_id = declare_parameter("frame_id").get<std::string>();
  m_publish_frequency = declare_parameter("output_frequency", kInvalidFrequency);
  m_publish_data_driven = declare_parameter("data_driven", false);
  const auto time_between_publish_requests{
    validate_publish_frequency(m_publish_frequency, m_publish_data_driven)};
  if (!m_publish_data_driven) {
    m_wall_timer = create_wall_timer(
      time_between_publish_requests,
      std::bind(&StateEstimationNode::predict_and_publish_current_state, this));
  }

  const auto position_variance{
    declare_parameter("process_noise_variances.position", std::vector<float64_t>{})};
  const auto velocity_variance{
    declare_parameter("process_noise_variances.velocity", std::vector<float64_t>{})};
  const auto acceleration_variance{
    declare_parameter("process_noise_variances.acceleration", std::vector<float64_t>{})};
  const auto state_variances{
    declare_parameter("state_variances", std::vector<float64_t>{})};
  const auto mahalanobis_threshold{
    declare_parameter("mahalanobis_threshold", std::numeric_limits<float32_t>::max())};

  m_ekf = std::make_unique<ConstantAccelerationFilter>(
    create_state_variances<6>(state_variances),
    create_process_noise_variances<6, 2>(
      position_variance, velocity_variance, acceleration_variance),
    time_between_publish_requests,
    m_frame_id,
    mahalanobis_threshold);

  const auto input_odom_topics{
    declare_parameter("topics.input_odom").get<std::vector<std::string>>()};
  const auto input_pose_topics{
    declare_parameter("topics.input_pose").get<std::vector<std::string>>()};
  const auto input_twist_topics{
    declare_parameter("topics.input_twist").get<std::vector<std::string>>()};

  create_subscriptions<OdomMsgT>(
    input_odom_topics, &m_odom_subscribers, &StateEstimationNode::odom_callback);
  create_subscriptions<PoseMsgT>(
    input_pose_topics, &m_pose_subscribers, &StateEstimationNode::pose_callback);
  create_subscriptions<TwistMsgT>(
    input_twist_topics, &m_twist_subscribers, &StateEstimationNode::twist_callback);

  m_publisher = create_publisher<nav_msgs::msg::Odometry>(kDefaultOutputTopic, kDefaultHistory);
}

void StateEstimationNode::odom_callback(const OdomMsgT::SharedPtr msg)
{
  const auto time_observation_received = to_time_point(now());
  m_ekf->observation_update(
    time_observation_received, message_to_measurement<MeasurementPoseAndSpeed>(
      *msg, get_transform(msg->header)));
  if (m_publish_data_driven) {
    m_publisher->publish(m_ekf->get_state());
  }
}

void StateEstimationNode::pose_callback(const PoseMsgT::SharedPtr msg)
{
  const auto time_observation_received = to_time_point(now());
  m_ekf->observation_update(
    time_observation_received, message_to_measurement<MeasurementPose>(
      *msg, get_transform(msg->header)));
  if (m_publish_data_driven) {
    m_publisher->publish(m_ekf->get_state());
  }
}

void StateEstimationNode::twist_callback(const TwistMsgT::SharedPtr msg)
{
  if (!m_ekf->is_initialized()) {
    RCLCPP_WARN(
      get_logger(),
      "Received twist update, but ekf has not been initialized with any state yet. Skipping.");
    return;
  }
  const auto time_observation_received = to_time_point(now());
  m_ekf->observation_update(
    time_observation_received, message_to_measurement<MeasurementSpeed>(
      *msg, get_transform(msg->header)));
  if (m_publish_data_driven) {
    m_publisher->publish(m_ekf->get_state());
  }
}

Eigen::Isometry3f StateEstimationNode::get_transform(const std_msgs::msg::Header & header)
{
  // Get the transform between the msg and the output frame. We treat the
  // possible exceptions as unrecoverable and let them bubble up.
  auto transform = m_tf_buffer.lookupTransform(
    m_frame_id, header.frame_id, to_time_point(header.stamp));
  Eigen::Isometry3f transform_eigen{tf2::transformToEigen(transform).cast<float32_t>()};
  return transform_eigen;
}

void StateEstimationNode::predict_and_publish_current_state()
{
  if (m_ekf->is_initialized()) {
    m_ekf->temporal_update(to_time_point(now()));
    m_publisher->publish(m_ekf->get_state());
  }
}

template<typename MessageT>
void StateEstimationNode::create_subscriptions(
  const std::vector<std::string> & input_topics,
  std::vector<typename rclcpp::Subscription<MessageT>::SharedPtr> * subscribers,
  CallbackFnT<MessageT> callback)
{
  for (const auto & input_topic : input_topics) {
    subscribers->emplace_back(
      create_subscription<MessageT>(
        input_topic, kDefaultHistory,
        std::bind(callback, this, std::placeholders::_1)));
  }
}

template void StateEstimationNode::create_subscriptions<StateEstimationNode::OdomMsgT>(
  const std::vector<std::string> &,
  std::vector<rclcpp::Subscription<OdomMsgT>::SharedPtr> *,
  CallbackFnT<StateEstimationNode::OdomMsgT>);
template void StateEstimationNode::create_subscriptions<StateEstimationNode::PoseMsgT>(
  const std::vector<std::string> &,
  std::vector<rclcpp::Subscription<PoseMsgT>::SharedPtr> *,
  CallbackFnT<StateEstimationNode::PoseMsgT>);
template void StateEstimationNode::create_subscriptions<StateEstimationNode::TwistMsgT>(
  const std::vector<std::string> &,
  std::vector<rclcpp::Subscription<TwistMsgT>::SharedPtr> *,
  CallbackFnT<StateEstimationNode::TwistMsgT>);


}  // namespace state_estimation_node
}  // namespace prediction
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::prediction::state_estimation_node::StateEstimationNode)
