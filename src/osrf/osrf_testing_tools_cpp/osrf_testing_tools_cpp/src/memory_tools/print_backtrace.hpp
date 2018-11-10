// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef MEMORY_TOOLS__PRINT_BACKTRACE_HPP_
#define MEMORY_TOOLS__PRINT_BACKTRACE_HPP_

#ifndef _WIN32

# pragma GCC diagnostic push
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wgnu-include-next"
#  pragma clang diagnostic ignored "-Wunused-parameter"
# endif

#include "./vendor/bombela/backward-cpp/backward.hpp"

# pragma GCC diagnostic pop

#endif  // _WIN32

namespace osrf_testing_tools_cpp
{
namespace memory_tools
{

template<int MaxStackDepth = 64>
void
print_backtrace(FILE * out = stderr)
{
#if !defined(_WIN32)
  backward::StackTrace st;
  st.load_here(MaxStackDepth);
  backward::Printer p;
  p.print(st, out);
#else
  fprintf(out, "backtrace unavailable on Windows\n");
#endif
}

}  // namespace memory_tools
}  // namespace osrf_testing_tools_cpp

#endif  // MEMORY_TOOLS__PRINT_BACKTRACE_HPP_
