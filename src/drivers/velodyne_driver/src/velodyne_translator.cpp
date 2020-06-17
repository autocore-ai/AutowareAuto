// Copyright 2018-2020 Apex.AI, Inc.
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

#include <velodyne_driver/vlp16_data.hpp>
#include <velodyne_driver/velodyne_translator.hpp>

namespace autoware
{
namespace drivers
{
namespace velodyne_driver
{
template class VelodyneTranslator<VLP16Data>;
}  // namespace velodyne_driver
}  // namespace drivers
}  // namespace autoware
