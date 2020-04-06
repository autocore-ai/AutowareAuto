// Copyright 2018 Apex.AI, Inc.
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

#include <common/types.hpp>
#include <limits>
#include <utility>

#include "velodyne_driver/vlp16_translator.hpp"

using autoware::common::types::float32_t;

namespace autoware
{
namespace drivers
{
namespace velodyne_driver
{
////////////////////////////////////////////////////////////////////////////////
Vlp16Translator::Config::Config(const float32_t rpm)
: m_rpm(rpm)
{
}
////////////////////////////////////////////////////////////////////////////////
float32_t Vlp16Translator::Config::get_rpm() const
{
  return m_rpm;
}

}  // namespace velodyne_driver
}  // namespace drivers
}  // namespace autoware
