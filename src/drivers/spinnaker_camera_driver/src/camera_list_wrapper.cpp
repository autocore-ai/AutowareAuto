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

#include <spinnaker_camera_driver/camera_list_wrapper.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstdint>
#include <memory>
#include <vector>
#include <functional>
#include <iostream>
#include <utility>

namespace autoware
{
namespace drivers
{
namespace camera
{
namespace spinnaker
{

CameraListWrapper::CameraListWrapper(
  Spinnaker::CameraList camera_list,
  const CameraSettings & camera_settings)
: m_camera_list{camera_list}
{
  for (std::uint32_t camera_index = 0; camera_index < m_camera_list.GetSize(); ++camera_index) {
    m_cameras.emplace_back(camera_index, m_camera_list.GetByIndex(camera_index), camera_settings);
  }
}

CameraListWrapper::CameraListWrapper(
  Spinnaker::CameraList camera_list,
  const std::vector<CameraSettings> & camera_settings)
: m_camera_list{camera_list}
{
  if (camera_settings.size() > m_camera_list.GetSize()) {
    throw std::logic_error("The number of settings is greater than the number of available cameras.");
  }

  m_cameras.reserve(m_camera_list.GetSize());
  std::uint32_t current_index = 0;
  for (std::uint32_t camera_index = 0; camera_index < m_camera_list.GetSize(); ++camera_index) {
    std::string device_serial_number = get_camera_serial_number(m_camera_list[camera_index]);
    for (std::uint32_t setting_index = 0; setting_index < camera_settings.size(); ++setting_index) {
      if (camera_settings[setting_index].get_serial_number().empty() ||
        (device_serial_number == camera_settings[setting_index].get_serial_number())) {
          m_cameras.emplace_back(current_index,
            m_camera_list.GetByIndex(current_index),
            camera_settings[setting_index]);
          current_index++;
       }
    }
  }
}

CameraListWrapper::~CameraListWrapper()
{
  // First destroy all the cameras using their destructors.
  m_cameras.clear();
  // Now clear the camera list that actually destoys the last instances of the cameras.
  m_camera_list.Clear();
}

void CameraListWrapper::start_capturing()
{
  for (auto & camera : m_cameras) {
    camera.start_capturing();
  }
}

void CameraListWrapper::stop_capturing()
{
  for (auto & camera : m_cameras) {
    camera.stop_capturing();
  }
}

void CameraListWrapper::set_image_callback(CameraWrapper::ImageCallbackFunction callback)
{
  for (auto & camera : m_cameras) {
    camera.set_on_image_callback(callback);
  }
}

std::unique_ptr<sensor_msgs::msg::Image> CameraListWrapper::retreive_image_from_camera(
  const std::uint32_t camera_index) const
{
  return m_cameras.at(camera_index).retreive_image();
}

std::string CameraListWrapper::get_camera_serial_number(const Spinnaker::CameraPtr camera)
{
  std::string serial_number;
  Spinnaker::GenApi::CStringPtr ptr_serial_number = camera->GetTLDeviceNodeMap().GetNode(
    "DeviceSerialNumber");
  if (Spinnaker::GenApi::IsReadable(ptr_serial_number)) {
    serial_number = std::string(ptr_serial_number->GetValue());
  } else {
    serial_number  = "";
  }

  return serial_number;
}

}  // namespace spinnaker
}  // namespace camera
}  // namespace drivers
}  // namespace autoware
