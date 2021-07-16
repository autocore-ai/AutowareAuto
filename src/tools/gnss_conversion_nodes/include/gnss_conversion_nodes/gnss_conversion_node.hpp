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


#ifndef GNSS_CONVERSION_NODES__GNSS_CONVERSION_NODE_HPP_
#define GNSS_CONVERSION_NODES__GNSS_CONVERSION_NODE_HPP_

#include <autoware_auto_msgs/msg/relative_position_with_covariance_stamped.hpp>
#include <common/types.hpp>
#include <gnss_conversion_nodes/visibility_control.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <GeographicLib/Geocentric.hpp>

#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware
{
namespace gnss_conversion_nodes
{

///
/// @brief      A node used for conversion between GNSS messages and ECEF coordinates.
///
class GNSS_CONVERSION_NODE_PUBLIC GnssConversionNode : public rclcpp::Node
{
public:
  ///
  /// @brief      Constructor from the node options.
  ///
  /// @param[in]  options  The Node options to load the parameters from.
  ///
  explicit GnssConversionNode(const rclcpp::NodeOptions & options);

  /// Expose the TF buffer for testing purposes.
  tf2::BufferCore & tf_buffer() {return m_tf_buffer;}

private:
  /// @brief      Callback for the NavSatFix message.
  void GNSS_CONVERSION_NODE_LOCAL nav_sat_fix_callback(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) const;

  /// Frame id to be set to the output messages.
  std::string m_frame_id{};
  /// Child frame id to be set to the output messages.
  std::string m_child_frame_id{};
  /// Output message publisher.
  rclcpp::Publisher<
    autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped>::SharedPtr m_publisher{};
  /// Input message subscription.
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_gnss_nav_fix_subscription{};
  /// Covariances to set in the output message as a diagonal.
  std::vector<common::types::float64_t> m_override_variances_diagonal{};

  /// A converter used for performing the actual conversions.
  GeographicLib::Geocentric m_wgs84_to_ecef_convertor{};

  /// A TF buffer.
  tf2::BufferCore m_tf_buffer;
  /// A TF listener.
  tf2_ros::TransformListener m_tf_listener;

  /// A clock used for logging.
  mutable rclcpp::Clock m_steady_clock{RCL_STEADY_TIME};
};

}  // namespace gnss_conversion_nodes
}  // namespace autoware

#endif  // GNSS_CONVERSION_NODES__GNSS_CONVERSION_NODE_HPP_
