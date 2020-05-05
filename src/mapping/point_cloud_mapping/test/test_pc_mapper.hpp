// Copyright 2020 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#ifndef POINT_CLOUD_MAPPING__TEST_PC_MAPPER_HPP_
#define POINT_CLOUD_MAPPING__TEST_PC_MAPPER_HPP_

#include <point_cloud_mapping/point_cloud_mapper.hpp>
#include <localization_common/localizer_base.hpp>

namespace autoware
{
namespace mapping
{
namespace point_cloud_mapping
{
struct MockLocalizerSummary {};

using Cloud = sensor_msgs::msg::PointCloud2;
using PCLCloud = pcl::PointCloud<pcl::PointXYZI>;

void append_to_pcl(const Cloud & pc, PCLCloud & res);

class PCMapperTestContext
{
public:
  PCMapperTestContext();

  static constexpr auto frame0{"frame0"};
  static constexpr auto frame1{"frame1"};
  static constexpr auto frame2{"frame2"};
  static constexpr auto map_frame{"map"};

protected:
  Cloud m_pc0; // initial message.
  geometry_msgs::msg::TransformStamped m_tf1;
  Cloud m_pc1;
  geometry_msgs::msg::TransformStamped m_tf2;
  Cloud m_pc2;
  PCLCloud m_expected_map;

private:
  void make_map();
};


class MockLocalizer : public localization::localization_common::RelativeLocalizerBase<Cloud, Cloud,
    MockLocalizerSummary>
{
public:
  using RegistrationSummary = MockLocalizerSummary;
  using Base = localization::localization_common::RelativeLocalizerBase<Cloud, Cloud,
      MockLocalizerSummary>;

  MockLocalizer(
    const geometry_msgs::msg::TransformStamped & tf1,
    const geometry_msgs::msg::TransformStamped & tf2);
  /// Get the frame id of the current map.
  const std::string & map_frame_id() const noexcept override;

  /// Get the timestamp of the current map.
  std::chrono::system_clock::time_point map_stamp() const noexcept override;

protected:
  /// `set_map` implementation.
  void set_map_impl(const Cloud & msg) override;

  /// `insert_to_map` implementation
  void insert_to_map_impl(const Cloud & msg) override;

  MockLocalizerSummary register_measurement_impl(
    const Cloud & msg,
    const Transform & transform_initial, PoseWithCovarianceStamped & pose_out) override;

private:
  geometry_msgs::msg::PoseWithCovarianceStamped m_fixed_estimate1;
  geometry_msgs::msg::PoseWithCovarianceStamped m_fixed_estimate2;
  const std::string m_map_frame{PCMapperTestContext::map_frame};
};

bool check_pc_equal(PCLCloud & pc1, PCLCloud & pc2)
{
  auto float_eq = [](auto n1, auto n2) {
      return std::fabs(n1 - n2) <= std::numeric_limits<float_t>::epsilon();
    };
  if (pc1.size() != pc2.size()) {
    return false;
  }
  // TODO(yunus.caliskan): do the check orderless?
  return std::equal(pc1.begin(), pc1.end(), pc2.begin(),
           [float_eq](const auto & pt1, const auto & pt2) {
             return float_eq(pt1.x, pt2.x) && float_eq(pt1.y, pt2.y) && float_eq(pt1.z, pt2.z);
           });
}

}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware

#endif  // POINT_CLOUD_MAPPING__TEST_PC_MAPPER_HPP_
