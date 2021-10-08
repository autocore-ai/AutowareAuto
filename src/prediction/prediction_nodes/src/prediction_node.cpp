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

#include "prediction_nodes/prediction_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>

#include "lonely_world_prediction/init_from_tracked.hpp"
#include "lonely_world_prediction/lonely_world_prediction.hpp"

#include "autoware_auto_msgs/msg/predicted_object.hpp"
#include "autoware_auto_msgs/msg/predicted_object_kinematics.hpp"

namespace
{
constexpr int32_t kDefaultTimeStep_ms{50};
constexpr int32_t kDefaultTimeHorizon_ms{3000};

using std::placeholders::_1;

rclcpp::QoS default_qos()
{
  return rclcpp::QoS{10}.reliable().transient_local();
}

}  // namespace

namespace autoware
{
namespace prediction
{
PredictionNode::PredictionNode(const rclcpp::NodeOptions & options)
: Node("prediction_nodes", options),
  m_parameters(
    std::chrono::milliseconds{declare_parameter("time_step_ms", kDefaultTimeStep_ms)},
    std::chrono::milliseconds{declare_parameter("time_horizon_ms", kDefaultTimeHorizon_ms)}),
  m_predicted_objects_pub{
    create_publisher<PredictedMsgT>("/prediction/predicted_objects", default_qos())},
  m_tracked_dynamic_objects_sub{create_subscription<TrackedMsgT>(
      "tracked_objects", rclcpp::QoS{10},
      std::bind(&PredictionNode::on_tracked_objects, this, _1))},
  m_tf_buffer{},
  m_tf_listener(m_tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false)
{
}

void PredictionNode::on_tracked_objects(TrackedMsgT::ConstSharedPtr msg)
{
  PredictedMsgT predicted_objects = from_tracked(*msg);
  autoware::prediction::predict_stationary(predicted_objects, m_parameters);
  m_predicted_objects_pub->publish(predicted_objects);
}


}  // namespace prediction
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::prediction::PredictionNode)
