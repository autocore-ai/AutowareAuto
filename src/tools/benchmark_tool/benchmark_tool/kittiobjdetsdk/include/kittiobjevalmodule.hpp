// Copyright 2021 Arm Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KITTIOBJEVALMODULE_HPP_
#define KITTIOBJEVALMODULE_HPP_

#include <common/types.hpp>
#include <string>

#include "visibility_control.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;

namespace kittisdk
{
extern "C" {
KITTIOBJEVALMODULE_PUBLIC int32_t eval(
  std::basic_string<char8_t> ground_truth_path,
  std::basic_string<char8_t> detection_path,
  std::basic_string<char8_t> output_path,
  bool8_t eval_2d_res,
  bool8_t eval_ground_red,
  bool8_t eval_3d_res,
  bool8_t print_stdout = false,
  bool8_t create_plot = false,
  int32_t n_testimages = 7480
);
}
}  // namespace kittisdk

#endif  // KITTIOBJEVALMODULE_HPP_
