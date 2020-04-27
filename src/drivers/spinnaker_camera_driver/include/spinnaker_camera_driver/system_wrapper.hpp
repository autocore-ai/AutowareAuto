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

#ifndef SPINNAKER_CAMERA_DRIVER__SYSTEM_WRAPPER_HPP_
#define SPINNAKER_CAMERA_DRIVER__SYSTEM_WRAPPER_HPP_

#ifndef DOXYGEN_SKIP
#include <spinnaker/Spinnaker.h>
#endif

#include <spinnaker_camera_driver/camera_list_wrapper.hpp>
#include <spinnaker_camera_driver/visibility_control.hpp>

#include <memory>
#include <vector>

namespace autoware
{
namespace drivers
{
namespace camera
{
namespace spinnaker
{

/// A class that wraps the Spinnaker SDK system pointer and handles its correct release.
class SPINNAKER_CAMERA_PUBLIC SystemWrapper
{
public:
  /// Create a system wrapper.
  SystemWrapper();

  /// Release the system instance.
  ~SystemWrapper();

  /// Get a list of all connected cameras and configure them with the same settings.
  CameraListWrapper & create_cameras(
    const CameraSettings & camera_settings);

  /// Get a list of all connected cameras with settings instance per camera.
  CameraListWrapper & create_cameras(
    const std::vector<CameraSettings> & camera_settings);

  /// Get available camera list.
  CameraListWrapper & get_cameras();

private:
  Spinnaker::SystemPtr m_system{};
  std::unique_ptr<CameraListWrapper> m_camera_list{};
};

}  //  namespace spinnaker
}  //  namespace camera
}  //  namespace drivers
}  //  namespace autoware

#endif  // SPINNAKER_CAMERA_DRIVER__SYSTEM_WRAPPER_HPP_
