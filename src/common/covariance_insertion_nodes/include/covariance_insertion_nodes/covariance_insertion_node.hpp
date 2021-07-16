// Copyright 2020-2021 The Autoware Foundation
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

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#ifndef COVARIANCE_INSERTION_NODES__COVARIANCE_INSERTION_NODE_HPP_
#define COVARIANCE_INSERTION_NODES__COVARIANCE_INSERTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <common/types.hpp>
#include <mpark_variant_vendor/variant.hpp>
#include <covariance_insertion/covariance_insertion.hpp>
#include <covariance_insertion_nodes/visibility_control.hpp>
#include <covariance_insertion_nodes/convert.hpp>

#include <string>
#include <map>
#include <memory>
#include <vector>

namespace autoware
{
namespace covariance_insertion_nodes
{

///
/// @class      CovarianceInsertionNode
///
/// @brief      ROS 2 Node for the covariance_insertion_nodes.
///
class COVARIANCE_INSERTION_NODES_PUBLIC CovarianceInsertionNode : public rclcpp::Node
{
public:
  /// @brief      A variant that holds all possible message types that this node can work with.
  using MsgVariant = mpark::variant<
    nav_msgs::msg::Odometry,
    geometry_msgs::msg::Pose,
    geometry_msgs::msg::PoseStamped,
    geometry_msgs::msg::PoseWithCovariance,
    geometry_msgs::msg::PoseWithCovarianceStamped,
    geometry_msgs::msg::Twist,
    geometry_msgs::msg::TwistStamped,
    geometry_msgs::msg::TwistWithCovariance,
    geometry_msgs::msg::TwistWithCovarianceStamped>;

  ///
  /// @brief      default constructor, starts the subscription and publisher.
  ///
  /// @param[in]  options  The node options
  /// @throws     domain_error  if the parameters are specified in a wrong way.
  ///
  explicit CovarianceInsertionNode(const rclcpp::NodeOptions & options);

private:
  // Validate that we can change the covariances of the entries from parameters.
  void validate(const MsgVariant & msg_variant);

  // Create a publisher and a subscriber that add covariance.
  void create_pub_sub(const MsgVariant & msg_variant);

  // Create a message variant from the input message.
  MsgVariant fill_message_variant(const std::string & input_msg_type_name);

  std::unique_ptr<covariance_insertion::CovarianceInsertion> m_core;
  rclcpp::SubscriptionBase::SharedPtr m_subscription{};
  rclcpp::PublisherBase::SharedPtr m_publisher{};
  std::size_t m_history_size{};
  std::string m_input_topic{};
  std::string m_output_topic{};
  std::string m_input_msg_type_name{};
};
}  // namespace covariance_insertion_nodes
}  // namespace autoware

#endif  // COVARIANCE_INSERTION_NODES__COVARIANCE_INSERTION_NODE_HPP_
