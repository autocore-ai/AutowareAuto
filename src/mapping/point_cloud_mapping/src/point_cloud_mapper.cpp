// Copyright 2020 the Autoware Foundation
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

#include <localization_common/localizer_base.hpp>
#include <point_cloud_mapping/point_cloud_mapper.hpp>
#include <string>

namespace autoware
{
namespace mapping
{
namespace point_cloud_mapping
{
std::string TimeStampPrefixGenerator::get_(const std::string & base_prefix) const noexcept
{
  auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  return base_prefix + "_" + std::to_string(timestamp);
}
}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware
