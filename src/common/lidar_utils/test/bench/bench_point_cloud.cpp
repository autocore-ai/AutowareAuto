// Copyright 2021 Apex.AI, Inc.
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

#include <benchmark/benchmark.h>
#include <helper_functions/float_comparisons.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <limits>

namespace
{

constexpr auto kCloudSize = 100UL;

autoware::common::types::bool8_t operator==(
  const autoware::common::types::PointXYZIF & p1,
  const autoware::common::types::PointXYZIF & p2) noexcept
{
  using autoware::common::helper_functions::comparisons::rel_eq;
  const auto epsilon = std::numeric_limits<autoware::common::types::float32_t>::epsilon();
  return rel_eq(p1.x, p2.x, epsilon) &&
         rel_eq(p1.y, p2.y, epsilon) &&
         rel_eq(p1.z, p2.z, epsilon) &&
         rel_eq(p1.intensity, p2.intensity, epsilon) &&
         (p1.id == p2.id);
}

sensor_msgs::msg::PointCloud2 create_point_cloud_through_wrapper(const std::size_t size)
{
  sensor_msgs::msg::PointCloud2 msg;
  using autoware::common::types::PointXYZIF;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{msg, "frame_id"};
  modifier.resize(size);
  return msg;
}

sensor_msgs::msg::PointCloud2 create_point_cloud_through_utils(const std::size_t kCloudSize)
{
  sensor_msgs::msg::PointCloud2 msg;
  using autoware::common::types::PointXYZIF;
  autoware::common::lidar_utils::init_pcl_msg(msg, "frame_id", kCloudSize);
  std::uint32_t idx{};
  autoware::common::lidar_utils::reset_pcl_msg(msg, kCloudSize, idx);
  return msg;
}

}  // namespace

static void BenchLidarUtilsAddPointToCloud(benchmark::State & state)
{
  auto msg = create_point_cloud_through_utils(kCloudSize);
  const autoware::common::types::PointXYZIF point{};
  std::uint32_t idx{};
  for (auto _ : state) {
    idx = 0;
    for (auto i = 0U; i < kCloudSize; ++i) {
      autoware::common::lidar_utils::add_point_to_cloud(msg, point, idx);
      benchmark::DoNotOptimize(msg);
    }
  }
}


static void BenchMsgWrapperAddPointToCloud(benchmark::State & state)
{
  using autoware::common::types::PointXYZIF;
  auto msg = create_point_cloud_through_wrapper(kCloudSize);
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{msg};
  const PointXYZIF point{};
  for (auto _ : state) {
    for (auto i = 0U; i < kCloudSize; ++i) {
      modifier[i] = point;
    }
    benchmark::DoNotOptimize(msg);
  }
}

static void BenchMsgWrapperResizeAndAddPointToCloud(benchmark::State & state)
{
  using autoware::common::types::PointXYZIF;
  auto msg = create_point_cloud_through_wrapper(kCloudSize);
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{msg};
  const PointXYZIF point{};
  for (auto _ : state) {
    modifier.clear();
    modifier.resize(kCloudSize);
    for (auto i = 0U; i < kCloudSize; ++i) {
      modifier[i] = point;
    }
    benchmark::DoNotOptimize(msg);
  }
}

static void BenchMsgWrapperPushBackPointToCloud(benchmark::State & state)
{
  using autoware::common::types::PointXYZIF;
  auto msg = create_point_cloud_through_wrapper(kCloudSize);
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{msg};
  const PointXYZIF point{};
  for (auto _ : state) {
    modifier.clear();
    modifier.reserve(kCloudSize);
    for (auto i = 0U; i < kCloudSize; ++i) {
      modifier.push_back(point);
    }
    benchmark::DoNotOptimize(msg);
  }
}


static void BenchLidarUtilsAccessPoint(benchmark::State & state)
{
  auto msg = create_point_cloud_through_utils(kCloudSize);
  std::uint32_t idx{};
  const autoware::common::types::PointXYZIF point{};
  for (auto i = 0U; i < kCloudSize; ++i) {
    autoware::common::lidar_utils::add_point_to_cloud(msg, point, idx);
  }
  auto x = 0.0F;
  auto y = 0.0F;
  auto z = 0.0F;
  auto intensity = 0.0F;
  for (auto _ : state) {
    sensor_msgs::PointCloud2ConstIterator<float32_t> x_it{msg, "x"};
    sensor_msgs::PointCloud2ConstIterator<float32_t> y_it{msg, "y"};
    sensor_msgs::PointCloud2ConstIterator<float32_t> z_it{msg, "z"};
    sensor_msgs::PointCloud2ConstIterator<float32_t> intensity_it{msg, "intensity"};
    while (x_it != x_it.end() && y_it != y_it.end() && z_it != z_it.end() &&
      intensity_it != intensity_it.end())
    {
      benchmark::DoNotOptimize(x);
      benchmark::DoNotOptimize(y);
      benchmark::DoNotOptimize(z);
      benchmark::DoNotOptimize(intensity);
      x = *x_it;
      y = *y_it;
      z = *z_it;
      intensity = *intensity_it;
      ++x_it;
      ++y_it;
      ++z_it;
      ++intensity_it;
    }
  }
}

static void BenchMsgWrapperAccessPoint(benchmark::State & state)
{
  using autoware::common::types::PointXYZIF;
  auto msg = create_point_cloud_through_wrapper(kCloudSize);
  const PointXYZIF point{};
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{msg};
  modifier.resize(kCloudSize);
  for (auto & p : modifier) {
    p = point;
  }
  auto x = 0.0F;
  auto y = 0.0F;
  auto z = 0.0F;
  auto intensity = 0.0F;
  auto id = 0;
  for (auto _ : state) {
    for (const auto & p : modifier) {
      benchmark::DoNotOptimize(x);
      benchmark::DoNotOptimize(y);
      benchmark::DoNotOptimize(z);
      benchmark::DoNotOptimize(intensity);
      benchmark::DoNotOptimize(id);
      x = p.x;
      y = p.y;
      z = p.z;
      intensity = p.intensity;
      id = p.id;
    }
  }
}

BENCHMARK(BenchMsgWrapperAddPointToCloud);
BENCHMARK(BenchMsgWrapperResizeAndAddPointToCloud);
BENCHMARK(BenchMsgWrapperPushBackPointToCloud);
BENCHMARK(BenchLidarUtilsAddPointToCloud);


BENCHMARK(BenchMsgWrapperAccessPoint);
BENCHMARK(BenchLidarUtilsAccessPoint);
