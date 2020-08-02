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

#include <spinnaker_camera_node/spinnaker_camera_node.hpp>

#include <rcutils/logging_macros.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

/// Create topic name from the camera index.
std::string create_camera_topic_name(std::uint32_t camera_index)
{
  return "/pointgrey/camera_" + std::to_string(camera_index) + "/image_raw";
}

}  // namespace

using autoware::common::types::float64_t;

namespace
{
static constexpr const char kDefaultCameraFrame[] = "camera";
static constexpr std::int64_t kDefaultDeviceThroughputLimit = 100000000L;
}  // namespace

namespace autoware
{
namespace drivers
{
namespace camera
{

SpinnakerCameraNode::SpinnakerCameraNode(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node{"spinnaker_camera_node", node_options}
{
  m_spinnaker_wrapper =
    std::make_unique<spinnaker::SystemWrapper>();
  auto & cameras{create_cameras_from_params(m_spinnaker_wrapper.get())};
  const auto number_of_cameras{cameras.get_number_of_cameras()};
  if (number_of_cameras < 1) {
    // TODO(igor): this should really be a terminate. It is a post-condition violation.
    throw std::runtime_error("No cameras were found. Cannot start node.");
  }
  const std::string one_publisher_per_camera_param{"one_publisher_per_camera"};
  m_use_publisher_per_camera = declare_parameter(one_publisher_per_camera_param, false);
  m_publishers = create_publishers(this, number_of_cameras, m_use_publisher_per_camera);
  if (m_publishers.empty()) {
    // TODO(igor): this should really be a terminate. It is a post-condition violation.
    throw std::runtime_error("No publishers created. Cannot start node.");
  }
  cameras.set_image_callback(std::bind(
      &SpinnakerCameraNode::publish_image, this, std::placeholders::_1, std::placeholders::_2));
  cameras.start_capturing();
}

spinnaker::CameraListWrapper & SpinnakerCameraNode::create_cameras_from_params(
  spinnaker::SystemWrapper * spinnaker_wrapper)
{
  if (!spinnaker_wrapper) {
    // TODO(igor): this should really be a terminate. It is a pre-condition violation.
    throw std::runtime_error("The Spinnaker Wrapper is not initialized.");
  }
  const auto settings_from_params = [this](const std::string & prefix) {
      const auto prefix_dot = prefix + '.';
      // Declare and init optional params.
      const auto camera_frame_id_param{
        declare_parameter(prefix_dot + "frame_id", std::string{kDefaultCameraFrame})};
      const auto device_link_throughput_limit_param{
        declare_parameter(
          prefix_dot + "device_link_throughput_limit", kDefaultDeviceThroughputLimit)};
      return spinnaker::CameraSettings{
      static_cast<std::uint32_t>(
        declare_parameter(prefix_dot + "window_width").get<std::uint64_t>()),
      static_cast<std::uint32_t>(
        declare_parameter(prefix_dot + "window_height").get<std::uint64_t>()),
      declare_parameter(prefix_dot + "fps").template get<float64_t>(),
      declare_parameter(prefix_dot + "pixel_format").template get<std::string>(),
      camera_frame_id_param,
      device_link_throughput_limit_param};
    };

  const std::string camera_settings_param_name{"camera_settings"};
  const std::string cameras_param_name{"cameras"};
  const auto camera_names{declare_parameter(cameras_param_name, std::vector<std::string>{})};
  if (camera_names.empty()) {
    RCLCPP_INFO(
      get_logger(), "No '%s' param provided. Looking for a single camera setting.",
      cameras_param_name.c_str());
    spinnaker_wrapper->create_cameras(
      settings_from_params(camera_settings_param_name));
  } else {
    std::vector<spinnaker::CameraSettings> settings;
    std::transform(camera_names.begin(), camera_names.end(), std::back_inserter(settings),
      [&camera_settings_param_name, &settings_from_params](
        const std::string & name) -> spinnaker::CameraSettings {
        return settings_from_params(camera_settings_param_name + "." + name);
      });
    RCLCPP_INFO(get_logger(), "Configuring %lu cameras.", settings.size());
    spinnaker_wrapper->create_cameras(settings);
  }
  return spinnaker_wrapper->get_cameras();
}

std::vector<SpinnakerCameraNode::ProtectedPublisher>
SpinnakerCameraNode::create_publishers(
  ::rclcpp::Node * node,
  const size_t number_of_cameras,
  const bool use_publisher_per_camera)
{
  if (!node) {
    // TODO(igor): this should really be a terminate. It is a pre-condition violation.
    throw std::runtime_error("The node is not initialized. Cannot create publishers.");
  }
  const auto number_of_publishers = use_publisher_per_camera ? number_of_cameras : 1UL;
  // We must create a new vector here as mutexes are not copyable.
  std::vector<ProtectedPublisher> publishers(number_of_publishers);
  for (auto i = 0U; i < number_of_publishers; ++i) {
    publishers[i].set_publisher(
      node->create_publisher<sensor_msgs::msg::Image>(create_camera_topic_name(i), 10));
  }
  return publishers;
}


void SpinnakerCameraNode::publish_image(
  std::uint32_t camera_index,
  std::unique_ptr<sensor_msgs::msg::Image> image)
{
  const auto publisher_index = m_use_publisher_per_camera ? camera_index : 0UL;
  if (image) {
    m_publishers.at(publisher_index).publish(std::move(image));
  }
}

void SpinnakerCameraNode::ProtectedPublisher::set_publisher(PublisherT::SharedPtr publisher)
{
  m_publisher = publisher;
}

void SpinnakerCameraNode::ProtectedPublisher::publish(
  std::unique_ptr<sensor_msgs::msg::Image> image)
{
  if (m_publisher) {
    const std::lock_guard<std::mutex> lock{m_publish_mutex};
    if (image) {
      m_publisher->publish(std::move(image));
    }
  } else {
    throw std::runtime_error("Publisher is nullptr, cannot publish.");
  }
}

}  // namespace camera
}  // namespace drivers
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::drivers::camera::SpinnakerCameraNode)
