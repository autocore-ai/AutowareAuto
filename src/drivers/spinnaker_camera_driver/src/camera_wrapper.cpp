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


#include <spinnaker_camera_driver/camera_wrapper.hpp>
#include <spinnaker_camera_driver/camera_settings.hpp>

#include <spinnaker/Spinnaker.h>

#include <memory>
#include <string>

namespace
{
constexpr std::uint64_t kNanoSecondsInSecond = 1000000000U;
}  // namespace


namespace autoware
{
namespace drivers
{
namespace camera
{
namespace spinnaker
{

CameraWrapper::CameraWrapper(
  std::uint32_t camera_index,
  const Spinnaker::CameraPtr & camera)
: m_camera_index{camera_index},
  m_camera{camera}
{
  m_camera->Init();
}

CameraWrapper::CameraWrapper(
  std::uint32_t camera_index,
  const Spinnaker::CameraPtr & camera,
  const CameraSettings & camera_settings)
: CameraWrapper{camera_index, camera}
{
  configure_camera(camera_settings);
}

CameraWrapper::~CameraWrapper()
{
  m_camera->UnregisterEvent(*this);
  if (m_camera->IsStreaming()) {
    m_camera->AcquisitionStop();
  }
  m_camera->DeInit();
}

void CameraWrapper::OnImageEvent(Spinnaker::ImagePtr image)
{
  if (m_on_image_callback) {
    m_on_image_callback(m_camera_index, convert_to_image_msg(image, m_frame_id));
    image->Release();
  }
}

std::unique_ptr<sensor_msgs::msg::Image> CameraWrapper::retreive_image() const
{
  if (m_on_image_callback) {
    throw std::logic_error("A callback is set, please use it to retreive images.");
  }
  auto image = m_camera->GetNextImage();
  auto image_msg = convert_to_image_msg(image, m_frame_id);
  image->Release();
  return image_msg;
}

void CameraWrapper::start_capturing()
{
  if (!m_is_camera_configured) {
    throw std::logic_error("Please configure the camera before capturing images.");
  }
  m_camera->AcquisitionStart();
  m_camera_is_capturing = true;
}

void CameraWrapper::stop_capturing()
{
  m_camera->AcquisitionStop();
  m_camera_is_capturing = false;
}

void CameraWrapper::set_on_image_callback(ImageCallbackFunction callback)
{
  m_on_image_callback = callback;
}

std::unique_ptr<sensor_msgs::msg::Image> CameraWrapper::convert_to_image_msg(
  const Spinnaker::ImagePtr & image, const std::string & frame_id)
{
  if (image->IsIncomplete()) {
    std::cerr << "Received an incomplete image. Skipping." << std::endl;
    return nullptr;
  }
  auto acquisition_time = image->GetTimeStamp();

  const std::string encoding_pattern = convert_to_pixel_format_string(image->GetPixelFormat());
  const auto seconds = acquisition_time / kNanoSecondsInSecond;
  std_msgs::msg::Header header;
  header.stamp.sec = static_cast<std::int32_t>(seconds);
  header.stamp.nanosec = static_cast<std::uint32_t>(acquisition_time - seconds);
  header.frame_id = frame_id;

  auto msg{std::make_unique<sensor_msgs::msg::Image>()};
  msg->header = header;
  msg->height = static_cast<std::uint32_t>(image->GetHeight());
  msg->width = static_cast<std::uint32_t>(image->GetWidth());
  msg->encoding = encoding_pattern;
  msg->step = static_cast<std::uint32_t>(image->GetStride());

  const size_t image_size = image->GetImageSize();
  msg->data.resize(image_size);
  std::copy_n(static_cast<std::uint8_t *>(
      image->GetData()), image_size, msg->data.data());
  return msg;
}

Spinnaker::PixelFormatEnums CameraWrapper::convert_to_pixel_format_enum(
  const std::string & pixel_format)
{
  if (pixel_format == CameraSettings::kPixelFormatStr_RGGB8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerRG8;
  }
  if (pixel_format == CameraSettings::kPixelFormatStr_GRBG8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerGR8;
  }
  if (pixel_format == CameraSettings::kPixelFormatStr_GBRG8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerGB8;
  }
  if (pixel_format == CameraSettings::kPixelFormatStr_BGGR8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerBG8;
  }
  if (pixel_format == CameraSettings::kPixelFormatStr_RGB8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_RGB8;
  }
  if (pixel_format == CameraSettings::kPixelFormatStr_BGR8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_BGR8;
  }
  if (pixel_format == CameraSettings::kPixelFormatStr_MONO8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_Mono8;
  }
  throw std::invalid_argument("Unknown pixel format.");
}

std::string CameraWrapper::convert_to_pixel_format_string(
  Spinnaker::PixelFormatEnums pixel_format)
{
  switch (pixel_format) {
    case Spinnaker::PixelFormatEnums::PixelFormat_BayerRG8:
      return CameraSettings::kPixelFormatStr_RGGB8;
      break;
    case Spinnaker::PixelFormatEnums::PixelFormat_BayerGR8:
      return CameraSettings::kPixelFormatStr_GRBG8;
      break;
    case Spinnaker::PixelFormatEnums::PixelFormat_BayerGB8:
      return CameraSettings::kPixelFormatStr_GBRG8;
      break;
    case Spinnaker::PixelFormatEnums::PixelFormat_BayerBG8:
      return CameraSettings::kPixelFormatStr_BGGR8;
      break;
    case Spinnaker::PixelFormatEnums::PixelFormat_RGB8:
      return CameraSettings::kPixelFormatStr_RGB8;
      break;
    case Spinnaker::PixelFormatEnums::PixelFormat_BGR8:
      return CameraSettings::kPixelFormatStr_BGR8;
      break;
    case Spinnaker::PixelFormatEnums::PixelFormat_Mono8:
      return CameraSettings::kPixelFormatStr_MONO8;
      break;
    default:
      throw std::invalid_argument("Unknown pixel format.");
  }
  return {};
}

/// Configure a Spinnaker camera.
void CameraWrapper::configure_camera(
  const CameraSettings & camera_settings)
{
  // Make sure this camera does not send callbacks while it is being configured.
  bool camera_should_capture{m_camera_is_capturing};
  if (m_camera_is_capturing) {
    stop_capturing();
  }

  using Spinnaker::GenApi::IsAvailable;
  using Spinnaker::GenApi::IsWritable;

  m_frame_id = camera_settings.get_frame_id();

  m_camera->Width.SetValue(camera_settings.get_window_width());
  m_camera->Height.SetValue(camera_settings.get_window_height());

  auto IsAvailableAndWritable = [](const Spinnaker::GenApi::IBase & value) {
      return IsAvailable(value) && IsWritable(value);
    };

  if (IsAvailable(m_camera->DeviceType)) {
    if (m_camera->DeviceType.GetCurrentEntry()->GetSymbolic() == "GEV") {
      if (IsAvailableAndWritable(m_camera->DeviceLinkThroughputLimit)) {
        m_camera->DeviceLinkThroughputLimit.SetValue(
          camera_settings.get_device_link_throughput_limit());
      } else {
        throw std::invalid_argument("Cannot set throuput limit on the m_camera.");
      }
    }
  }
  if (IsAvailableAndWritable(m_camera->AcquisitionFrameRateEnable) &&
    IsAvailableAndWritable(m_camera->AcquisitionFrameRate))
  {
    m_camera->AcquisitionFrameRateEnable.SetValue(true);
    m_camera->AcquisitionFrameRate.SetValue(camera_settings.get_fps());
  } else {
    throw std::invalid_argument("Cannot set frame rate.");
  }
  if (IsAvailableAndWritable(m_camera->PixelFormat)) {
    m_camera->PixelFormat.SetValue(
      convert_to_pixel_format_enum(camera_settings.get_pixel_format()));
  } else {
    throw std::invalid_argument("Cannot set pixel format.");
  }
  if (IsAvailableAndWritable(m_camera->AcquisitionMode)) {
    m_camera->AcquisitionMode.SetValue(
      Spinnaker::AcquisitionModeEnums::AcquisitionMode_Continuous);
  } else {
    throw std::invalid_argument("Cannot set continuous acquisition mode.");
  }

  if (camera_should_capture) {
    start_capturing();
  }
}

}  //  namespace spinnaker
}  //  namespace camera
}  //  namespace drivers
}  //  namespace autoware
