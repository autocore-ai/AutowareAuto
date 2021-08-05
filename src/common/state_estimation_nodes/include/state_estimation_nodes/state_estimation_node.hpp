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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.


#ifndef STATE_ESTIMATION_NODES__STATE_ESTIMATION_NODE_HPP_
#define STATE_ESTIMATION_NODES__STATE_ESTIMATION_NODE_HPP_


#include <autoware_auto_msgs/msg/relative_position_with_covariance_stamped.hpp>
#include <common/types.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <measurement_conversion/measurement_transformation.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <state_estimation_nodes/kalman_filter_wrapper.hpp>
#include <state_estimation_nodes/visibility_control.hpp>

#include <tf2/buffer_core.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// A node that uses EKF to estimate the state.
///
class STATE_ESTIMATION_NODES_PUBLIC StateEstimationNode : public rclcpp::Node
{
public:
  ///
  /// ROS 2 parameter constructor for node composition.
  ///
  /// @param[in]  node_options    Node options for this node.
  ///
  explicit StateEstimationNode(
    const rclcpp::NodeOptions & node_options);

  /// @brief      Get the tf2::buffer used by the node.
  tf2::BufferCore & buffer() {return m_tf_buffer;}

private:
  using OdomMsgT = nav_msgs::msg::Odometry;
  using PoseMsgT = geometry_msgs::msg::PoseWithCovarianceStamped;
  using TwistMsgT = geometry_msgs::msg::TwistWithCovarianceStamped;
  using RelativePosMsgT = autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped;
  using TfMsgT = tf2_msgs::msg::TFMessage;
  using FilterWrapperT = ConstantAccelerationFilterWrapperXYZRPY;

  template<std::int32_t kDim>
  using VectorT = Eigen::Matrix<autoware::common::types::float32_t, kDim, 1>;

  /// Gets called when a new pose message arrives.
  ///
  /// @param[in]  msg   Pose message.
  ///
  void STATE_ESTIMATION_NODES_LOCAL pose_callback(const PoseMsgT::SharedPtr msg);

  /// Gets called when a new relative position measurement arrives.
  ///
  /// @param[in]  msg   The relative position message that holds a relative position.
  ///
  void STATE_ESTIMATION_NODES_LOCAL relative_pos_callback(const RelativePosMsgT::SharedPtr msg);

  /// Predict the state and publish the current estimate.
  void STATE_ESTIMATION_NODES_LOCAL predict_and_publish_current_state();

  /// Publish the currently estimated state.
  void STATE_ESTIMATION_NODES_LOCAL publish_current_state();

  template<typename MessageT>
  using CallbackFnT = void (StateEstimationNode::*)(const typename MessageT::SharedPtr);

  template<typename MessageT>
  void create_subscriptions(
    const std::vector<std::string> & input_topics,
    std::vector<typename rclcpp::Subscription<MessageT>::SharedPtr> * subscribers,
    CallbackFnT<MessageT> callback);

  std::vector<rclcpp::Subscription<PoseMsgT>::SharedPtr> m_pose_subscribers;
  std::vector<rclcpp::Subscription<RelativePosMsgT>::SharedPtr> m_relative_pos_subscribers;

  std::shared_ptr<rclcpp::Publisher<OdomMsgT>> m_publisher{};
  std::shared_ptr<rclcpp::Publisher<TfMsgT>> m_tf_publisher{};

  std::chrono::system_clock::duration m_init_timeout{};
  std::chrono::system_clock::duration m_expected_time_between_publish_requests{};
  std::chrono::system_clock::duration m_history_length{};
  std::chrono::system_clock::time_point m_time_of_last_publish{};

  rclcpp::TimerBase::SharedPtr m_wall_timer{};

  common::types::bool8_t m_filter_initialized{};
  common::types::bool8_t m_publish_data_driven{};
  common::types::float64_t m_publish_frequency{};

  std::string m_frame_id{};
  std::string m_child_frame_id{};

  // TODO(igor): we can replace the unique_ptr here with std::variant or alike at a later time to
  // allow configuring which filter to use at runtime.
  std::unique_ptr<FilterWrapperT> m_ekf{};

  tf2::BufferCore m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;
};

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware

#endif  // STATE_ESTIMATION_NODES__STATE_ESTIMATION_NODE_HPP_
