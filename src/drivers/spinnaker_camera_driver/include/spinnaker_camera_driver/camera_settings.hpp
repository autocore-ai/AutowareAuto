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

#ifndef SPINNAKER_CAMERA_DRIVER__CAMERA_SETTINGS_HPP_
#define SPINNAKER_CAMERA_DRIVER__CAMERA_SETTINGS_HPP_

#include <common/types.hpp>
#include <spinnaker_camera_driver/visibility_control.hpp>

#include <string>
#include <set>
#include <stdexcept>

namespace autoware
{
namespace drivers
{
namespace camera
{
namespace spinnaker
{

/// Encapsulate settings that make sense to pass to a camera.
class SPINNAKER_CAMERA_PUBLIC CameraSettings
{
public:
  static const char * kPixelFormatStr_RGGB8;
  static const char * kPixelFormatStr_GRBG8;
  static const char * kPixelFormatStr_GBRG8;
  static const char * kPixelFormatStr_BGGR8;
  static const char * kPixelFormatStr_RGB8;
  static const char * kPixelFormatStr_BGR8;
  static const char * kPixelFormatStr_MONO8;
  static const char * kPixelFormatStr_UNKNOWN;

  /// Instantiate settings and throw if they are not valid.
  explicit CameraSettings(
    std::uint32_t window_width,
    std::uint32_t window_height,
    common::types::float64_t fps,
    const std::string & pixel_format,
    const std::string & frame_id = "camera",
    std::int64_t device_link_throughput_limit = 100000000L);

  inline std::uint32_t get_window_width() const noexcept {return m_window_width;}
  inline std::uint32_t get_window_height() const noexcept {return m_window_height;}
  inline const std::string & get_pixel_format() const noexcept {return m_pixel_format;}
  inline const std::string & get_frame_id() const noexcept {return m_frame_id;}
  inline common::types::float64_t get_fps() const noexcept {return m_fps;}
  inline std::int64_t get_device_link_throughput_limit() const noexcept
  {
    return m_device_link_throughput_limit;
  }

private:
  /// A set of all valid pixel formats.
  static const std::set<std::string> kValidPixelFormats;

  /// Width of the returned image.
  std::uint32_t m_window_width;
  /// Height of the returned image.
  std::uint32_t m_window_height;

  /// Format of pixels.
  std::string m_pixel_format;

  /// Camera frame id.
  std::string m_frame_id;

  /// Wanted fps.
  common::types::float64_t m_fps;

  /// Desired device link throuput limit.
  std::int64_t m_device_link_throughput_limit;
};

}  //  namespace spinnaker
}  //  namespace camera
}  //  namespace drivers
}  //  namespace autoware


#endif  // SPINNAKER_CAMERA_DRIVER__CAMERA_SETTINGS_HPP_
