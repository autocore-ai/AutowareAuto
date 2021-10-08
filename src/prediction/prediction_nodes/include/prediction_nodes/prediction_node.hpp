// Copyright 2021 The Autoware Foundation
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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the prediction node

#ifndef PREDICTION_NODES__PREDICTION_NODE_HPP_
#define PREDICTION_NODES__PREDICTION_NODE_HPP_


#include <chrono>

#include "lonely_world_prediction/parameters.hpp"
#include "prediction_nodes/visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_auto_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_msgs/msg/route.hpp"
#include "autoware_auto_msgs/msg/tracked_objects.hpp"

namespace autoware
{
namespace prediction
{
/// Main node for prediction of dynamic objects
class PREDICTION_NODES_PUBLIC PredictionNode : public rclcpp::Node
{
public:
  using PredictedMsgT = autoware_auto_msgs::msg::PredictedObjects;
  using TrackedMsgT = autoware_auto_msgs::msg::TrackedObjects;

  /// \brief default constructor, starts node
  /// \throw runtime error if failed to start threads or configure driver
  explicit PredictionNode(const rclcpp::NodeOptions & options);

private:
  /**
   * Predict tracked objects and publish message on the output topic
   *
   * @param msg Message with tracked objects
   */
  void PREDICTION_NODES_LOCAL on_tracked_objects(TrackedMsgT::ConstSharedPtr msg);

  Parameters m_parameters;
  rclcpp::Publisher<PredictedMsgT>::SharedPtr m_predicted_objects_pub{};
  rclcpp::Subscription<TrackedMsgT>::SharedPtr m_tracked_dynamic_objects_sub{};

  tf2::BufferCore m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;
};
}  // namespace prediction
}  // namespace autoware

#endif  // PREDICTION_NODES__PREDICTION_NODE_HPP_
