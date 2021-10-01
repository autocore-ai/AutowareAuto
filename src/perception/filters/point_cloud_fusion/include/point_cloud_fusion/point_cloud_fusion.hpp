// Copyright 2019-2021 the Autoware Foundation
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

#ifndef POINT_CLOUD_FUSION__POINT_CLOUD_FUSION_HPP_
#define POINT_CLOUD_FUSION__POINT_CLOUD_FUSION_HPP_

#include <point_cloud_fusion/visibility_control.hpp>

#include <common/types.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <vector>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace point_cloud_fusion
{
using autoware::common::types::PointXYZI;

class POINT_CLOUD_FUSION_PUBLIC PointCloudFusion
{
public:
  enum class Error : uint8_t
  {
    NONE = 0U,
    TOO_LARGE,
    INSERT_FAILED
  };  // enum class Error
  using PointCloudMsgT = sensor_msgs::msg::PointCloud2;

  /// \brief     constructor
  /// \param[in] cloud_capacity
  /// \param[in] input_topics_size
  explicit PointCloudFusion(
    uint32_t cloud_capacity,
    size_t input_topics_size);

  /// \brief This function goes through all of the messages and adds them to the concatenated
  /// point cloud. If a pointcloud cannot be transformed to the output frame, it's ignored. If
  /// concatenation exceeds the maximum capacity, fusion stops and the partially concatenated cloud
  /// is still published.
  /// \param[in]  msgs msgs to be fused.
  /// \param[out] cloud_concatenated fused msgs.
  /// \return     Size of the concatenated pointcloud.
  uint32_t fuse_pc_msgs(
    const std::array<PointCloudMsgT::ConstSharedPtr, 8> & msgs,
    PointCloudMsgT & cloud_concatenated);

private:
  void concatenate_pointcloud(
    const PointCloudMsgT & pc_in,
    uint32_t & concat_idx,
    point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> & modifier) const;

  uint32_t m_cloud_capacity;
  size_t m_input_topics_size;
};

}  // namespace point_cloud_fusion
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // POINT_CLOUD_FUSION__POINT_CLOUD_FUSION_HPP_
