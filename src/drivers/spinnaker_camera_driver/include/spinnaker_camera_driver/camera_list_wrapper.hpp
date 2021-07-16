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

#ifndef SPINNAKER_CAMERA_DRIVER__CAMERA_LIST_WRAPPER_HPP_
#define SPINNAKER_CAMERA_DRIVER__CAMERA_LIST_WRAPPER_HPP_

#ifndef DOXYGEN_SKIP
#include <Spinnaker.h>
#endif

#include <sensor_msgs/msg/image.hpp>
#include <spinnaker_camera_driver/camera_wrapper.hpp>
#include <spinnaker_camera_driver/visibility_control.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <functional>

namespace autoware
{
/// \brief Libraries, ROS nodes, and other functionality relating to
///         sensor drivers or actuation.
namespace drivers
{
/// \brief Contains everything related to camera drivers.
namespace camera
{
/// \brief Contains everything related to Spinnaker SDK.
namespace spinnaker
{

/// A class that wraps a Spnnaker SDK list of cameras.
///
/// It handles the lifetimes of the list of cameras, making sure the cameras are released
/// and that the list is cleared in a proper way.
class SPINNAKER_CAMERA_PUBLIC CameraListWrapper
{
public:
  /// Create the list wrapper and assign the same camera_settings to each camera.
  explicit CameraListWrapper(
    Spinnaker::CameraList camera_list,
    const CameraSettings & camera_settings);

  /// Create the list wrapper with a camera_settings instance for each camera.
  explicit CameraListWrapper(
    Spinnaker::CameraList camera_list,
    const std::vector<CameraSettings> & camera_settings);

  /// We need a custom destructor as the camera list need to be cleared.
  virtual ~CameraListWrapper();

  // Satisfy the rule of all or nothing: we need a custom destructor,
  // so we need to define all constructors.
  // We don't expect this class to be coppied over, so all of these are deleted.
  CameraListWrapper(const CameraListWrapper &) = delete;
  CameraListWrapper & operator=(const CameraListWrapper &) = delete;

  // It's still ok to move the camera list.
  CameraListWrapper(CameraListWrapper &&) = default;
  CameraListWrapper & operator=(CameraListWrapper &&) = default;

  /// Start capturing on all cameras.
  void start_capturing();

  /// Stop capturing on all cameras.
  void stop_capturing();

  /// Retreive latest available image from a camera.
  std::unique_ptr<sensor_msgs::msg::Image> retreive_image_from_camera(
    const std::uint32_t camera_index) const;

  /// Get number of cameras.
  inline std::size_t get_number_of_cameras() const {return m_cameras.size();}

  /// Set a function that is going to be called when an image arrives.
  void set_image_callback(CameraWrapper::ImageCallbackFunction callback);

private:
  /// Get the serial number of camera.
  static std::string get_camera_serial_number(const Spinnaker::CameraPtr camera);

  /// A handle to the camera list pointer.
  Spinnaker::CameraList m_camera_list{};
  /// We also store the cameras along with the camera list.
  std::vector<CameraWrapper> m_cameras{};
};

}  //  namespace spinnaker
}  //  namespace camera
}  //  namespace drivers
}  //  namespace autoware

#endif  // SPINNAKER_CAMERA_DRIVER__CAMERA_LIST_WRAPPER_HPP_
