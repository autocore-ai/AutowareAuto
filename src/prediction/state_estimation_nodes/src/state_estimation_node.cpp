// Copyright 2021 Apex.AI, Inc.
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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <state_estimation_nodes/state_estimation_node.hpp>

#include <kalman_filter/kalman_filter.hpp>
#include <motion_model/linear_motion_model.hpp>
#include <motion_model/wiener_noise.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <state_estimation_nodes/measurement_conversion.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;

namespace
{
constexpr int kDefaultHistory{10};  // TODO(igor): remove this.
constexpr float64_t kInvalidFrequency{-1.0};  // Frames per second.
constexpr std::chrono::milliseconds kDefaultTimeBetweenUpdates{100LL};
constexpr std::chrono::milliseconds kDefaultHistoryLength{5000LL};
const char kDefaultOutputTopic[]{"filtered_state"};
constexpr auto kCovarianceMatrixRows{6U};
constexpr auto kIndexX{0U};
constexpr auto kIndexY{kCovarianceMatrixRows + 1U};
constexpr auto kCovarianceMatrixRowsSquared{kCovarianceMatrixRows * kCovarianceMatrixRows};
static_assert(
  std::tuple_size<
    geometry_msgs::msg::PoseWithCovariance::_covariance_type>::value ==
  kCovarianceMatrixRowsSquared, "We expect the covariance matrix to have 36 entries.");

/// Convert the ROS timestamp to chrono time point.
std::chrono::system_clock::time_point
to_time_point(const rclcpp::Time & time)
{
  return std::chrono::system_clock::time_point{std::chrono::nanoseconds{time.nanoseconds()}};
}

void assert_all_entries_positive(const std::vector<float64_t> & entries, const std::string & tag)
{
  for (const auto entry : entries) {
    if (std::isnan(entry) || std::isinf(entry) || entry <= 0.0) {
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
namespace state_estimation_nodes
{

StateEstimationNode::StateEstimationNode(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node{"state_estimation_node", node_options},
  m_tf_listener(m_tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false)
{
  m_frame_id = declare_parameter("frame_id").get<std::string>();
  m_child_frame_id = declare_parameter("child_frame_id").get<std::string>();
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
  const auto acceleration_variances{
    declare_parameter("process_noise_variances.acceleration", std::vector<float64_t>{})};
  const auto state_variances{
    declare_parameter("state_variances", std::vector<float64_t>{})};
  const auto mahalanobis_threshold{
    declare_parameter("mahalanobis_threshold", std::numeric_limits<float32_t>::max())};

  using State = ConstantAccelerationFilterWrapper::State;
  m_ekf = std::make_unique<ConstantAccelerationFilterWrapper>(
    LinearMotionModel<State>{},
    make_wiener_noise<State>(acceleration_variances),
    create_state_variances<6>(state_variances),
    time_between_publish_requests,
    m_frame_id,
    kDefaultHistoryLength,
    mahalanobis_threshold);


  const std::vector<std::string> empty_vector{};
  const auto input_odom_topics{declare_parameter("topics.input_odom", empty_vector)};
  const auto input_pose_topics{declare_parameter("topics.input_pose", empty_vector)};
  const auto input_twist_topics{declare_parameter("topics.input_twist", empty_vector)};
  if (input_odom_topics.empty() && input_pose_topics.empty() && input_twist_topics.empty()) {
    throw std::runtime_error("No input topics provided. Make sure to set these in the param file.");
  }
  create_subscriptions<OdomMsgT>(
    input_odom_topics, &m_odom_subscribers, &StateEstimationNode::odom_callback);
  create_subscriptions<PoseMsgT>(
    input_pose_topics, &m_pose_subscribers, &StateEstimationNode::pose_callback);
  create_subscriptions<TwistMsgT>(
    input_twist_topics, &m_twist_subscribers, &StateEstimationNode::twist_callback);

  m_publisher = create_publisher<nav_msgs::msg::Odometry>(kDefaultOutputTopic, kDefaultHistory);

  const auto publish_ft = declare_parameter("publish_tf", false);
  if (publish_ft) {
    m_tf_publisher = create_publisher<tf2_msgs::msg::TFMessage>("/tf", kDefaultHistory);
  }

  m_min_speed_to_use_speed_orientation = declare_parameter(
    "min_speed_to_use_speed_orientation", 0.0);
}

void StateEstimationNode::odom_callback(const OdomMsgT::SharedPtr msg)
{
  const auto tf__m_frame_id__msg_frame_id =
    get_transform(m_frame_id, msg->header.frame_id, msg->header.stamp);
  const auto tf__m_frame_id__msg_child_frame_id =
    get_transform(m_frame_id, msg->child_frame_id, msg->header.stamp);

  const auto measurement = message_to_measurement<StampedMeasurementPoseAndSpeed>(
    *msg,
    tf2::transformToEigen(tf__m_frame_id__msg_frame_id).cast<float32_t>(),
    tf2::transformToEigen(tf__m_frame_id__msg_child_frame_id).cast<float32_t>());
  if (m_ekf->is_initialized()) {
    if (!m_ekf->add_observation_to_history(measurement)) {
      throw std::runtime_error("Cannot add an odometry observation to history.");
    }
  } else {
    m_ekf->add_reset_event_to_history(measurement);
  }
  geometry_msgs::msg::QuaternionStamped orientation_in_expected_frame;
  tf2::doTransform(
    geometry_msgs::msg::QuaternionStamped{}.
    set__quaternion(msg->pose.pose.orientation).
    set__header(msg->header),
    orientation_in_expected_frame, tf__m_frame_id__msg_frame_id);
  update_latest_orientation_if_needed(orientation_in_expected_frame);
  if (m_publish_data_driven && m_ekf->is_initialized()) {
    publish_current_state();
  }
}

void StateEstimationNode::pose_callback(const PoseMsgT::SharedPtr msg)
{
  const auto tf__m_frame_id__msg_frame_id =
    get_transform(m_frame_id, msg->header.frame_id, msg->header.stamp);
  const auto measurement = message_to_measurement<StampedMeasurementPose>(
    *msg, tf2::transformToEigen(tf__m_frame_id__msg_frame_id).cast<float32_t>());
  if (m_ekf->is_initialized()) {
    if (!m_ekf->add_observation_to_history(measurement)) {
      throw std::runtime_error("Cannot add an odometry observation to history.");
    }
  } else {
    m_ekf->add_reset_event_to_history(measurement);
  }
  geometry_msgs::msg::QuaternionStamped orientation_in_expected_frame;
  tf2::doTransform(
    geometry_msgs::msg::QuaternionStamped{}.
    set__quaternion(msg->pose.pose.orientation).
    set__header(msg->header),
    orientation_in_expected_frame, tf__m_frame_id__msg_frame_id);
  update_latest_orientation_if_needed(orientation_in_expected_frame);
  if (m_publish_data_driven && m_ekf->is_initialized()) {
    publish_current_state();
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
  const auto tf__m_frame_id__msg_frame_id =
    get_transform(m_frame_id, msg->header.frame_id, msg->header.stamp);
  if (!m_ekf->add_observation_to_history(
      message_to_measurement<StampedMeasurementSpeed>(
        *msg, tf2::transformToEigen(tf__m_frame_id__msg_frame_id).cast<float32_t>())))
  {
    throw std::runtime_error("Cannot add a twist observation to history.");
  }
  if (m_publish_data_driven && m_ekf->is_initialized()) {
    publish_current_state();
  }
}

geometry_msgs::msg::TransformStamped StateEstimationNode::get_transform(
  const std_msgs::msg::Header::_frame_id_type & target_frame_id,
  const std_msgs::msg::Header::_frame_id_type & source_frame_id,
  const std_msgs::msg::Header::_stamp_type & timestamp)
{
  // Get the transform between the msg and the output frame. We treat the
  // possible exceptions as unrecoverable and let them bubble up.
  return m_tf_buffer.lookupTransform(
    target_frame_id, source_frame_id, to_time_point(timestamp));
}

void StateEstimationNode::predict_and_publish_current_state()
{
  if (!m_ekf->is_initialized()) {return;}
  if (!m_ekf->add_next_temporal_update_to_history()) {
    throw std::runtime_error("Could not perform a temporal update.");
  }
  publish_current_state();
}

void StateEstimationNode::publish_current_state()
{
  if (m_ekf->is_initialized() && m_publisher) {
    auto state = m_ekf->get_state();
    if (state.twist.twist.linear.x < m_min_speed_to_use_speed_orientation) {
      state.pose.pose.orientation = m_latest_orientation.quaternion;
    }
    state.child_frame_id = m_child_frame_id;
    m_publisher->publish(state);
    if (m_tf_publisher) {
      TfMsgT tf_msg{};
      tf_msg.transforms.emplace_back();
      auto & tf = tf_msg.transforms.back();
      tf.header = state.header;
      tf.child_frame_id = m_child_frame_id;
      tf.transform.translation.x = state.pose.pose.position.x;
      tf.transform.translation.y = state.pose.pose.position.y;
      tf.transform.translation.z = state.pose.pose.position.z;
      tf.transform.rotation = state.pose.pose.orientation;
      m_tf_publisher->publish(tf_msg);
    }
  }
}

void StateEstimationNode::update_latest_orientation_if_needed(
  const geometry_msgs::msg::QuaternionStamped & rotation)
{
  if (m_latest_orientation.header.stamp.sec > rotation.header.stamp.sec) {return;}
  if (m_latest_orientation.header.stamp.sec == rotation.header.stamp.sec) {
    if (m_latest_orientation.header.stamp.nanosec > rotation.header.stamp.nanosec) {return;}
  }
  m_latest_orientation = rotation;
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


}  // namespace state_estimation_nodes
}  // namespace prediction
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::prediction::state_estimation_nodes::StateEstimationNode)
