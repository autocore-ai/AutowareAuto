// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho's mpc code
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

#ifndef PARKING_PLANNER_NODE__VISIBILITY_CONTROL_HPP_
#define PARKING_PLANNER_NODE__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(PARKING_PLANNER_NODE_BUILDING_DLL) || defined(PARKING_PLANNER_NODE_EXPORTS)
    #define PARKING_PLANNER_NODE_PUBLIC __declspec(dllexport)
    #define PARKING_PLANNER_NODE_LOCAL
  #else
// defined(PARKING_PLANNER_NODE_BUILDING_DLL) || defined(PARKING_PLANNER_NODE_EXPORTS)
    #define PARKING_PLANNER_NODE_PUBLIC __declspec(dllimport)
    #define PARKING_PLANNER_NODE_LOCAL
  #endif
// defined(PARKING_PLANNER_NODE_BUILDING_DLL) || defined(PARKING_PLANNER_NODE_EXPORTS)
#elif defined(__linux__)
  #define PARKING_PLANNER_NODE_PUBLIC __attribute__((visibility("default")))
  #define PARKING_PLANNER_NODE_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define PARKING_PLANNER_NODE_PUBLIC __attribute__((visibility("default")))
  #define PARKING_PLANNER_NODE_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // PARKING_PLANNER_NODE__VISIBILITY_CONTROL_HPP_
