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
  if (camera_settings.size() != m_camera_list.GetSize()) {
    throw std::logic_error("Number of settings does not match the number of available cameras.");
  }
  for (std::uint32_t camera_index = 0; camera_index < m_camera_list.GetSize(); ++camera_index) {
    m_cameras.emplace_back(camera_index,
      m_camera_list.GetByIndex(camera_index),
      camera_settings[camera_index]);
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

}  //  namespace spinnaker
}  // namespace camera
}  // namespace drivers
}  // namespace autoware
