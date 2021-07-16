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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <spinnaker_camera_driver/system_wrapper.hpp>

#include <vector>
#include <memory>

namespace autoware
{
namespace drivers
{
namespace camera
{
namespace spinnaker
{

SystemWrapper::SystemWrapper()
: m_system{Spinnaker::System::GetInstance()} {}

SystemWrapper::~SystemWrapper()
{
  // We need to cleanup the camera list *before* we release the system instance.
  m_camera_list.reset();
  m_system->ReleaseInstance();
}

CameraListWrapper & SystemWrapper::create_cameras(
  const CameraSettings & camera_settings)
{
  if (m_camera_list) {
    throw std::logic_error("Camera list already initialized. Cannot create cameras again.");
  }
  m_camera_list = std::make_unique<CameraListWrapper>(m_system->GetCameras(), camera_settings);
  return *m_camera_list;
}

CameraListWrapper & SystemWrapper::create_cameras(
  const std::vector<CameraSettings> & camera_settings)
{
  if (m_camera_list) {
    throw std::logic_error("Camera list already initialized. Cannot create cameras again.");
  }
  m_camera_list = std::make_unique<CameraListWrapper>(m_system->GetCameras(), camera_settings);
  return *m_camera_list;
}

CameraListWrapper & SystemWrapper::get_cameras()
{
  if (!m_camera_list) {
    throw std::logic_error("Camera list has not been initialized.");
  }
  return *m_camera_list;
}


}  //  namespace spinnaker
}  //  namespace camera
}  //  namespace drivers
}  //  namespace autoware
