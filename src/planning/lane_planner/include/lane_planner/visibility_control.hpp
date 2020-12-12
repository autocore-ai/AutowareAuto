// Copyright 2020 The Autoware Foundation
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

#ifndef LANE_PLANNER__VISIBILITY_CONTROL_HPP_
#define LANE_PLANNER__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(LANE_PLANNER_BUILDING_DLL) || defined(LANE_PLANNER_EXPORTS)
    #define LANE_PLANNER_PUBLIC __declspec(dllexport)
    #define LANE_PLANNER_LOCAL
  #else  // defined(LANE_PLANNER_BUILDING_DLL) || defined(LANE_PLANNER_EXPORTS)
    #define LANE_PLANNER_PUBLIC __declspec(dllimport)
    #define LANE_PLANNER_LOCAL
  #endif  // defined(LANE_PLANNER_BUILDING_DLL) || defined(LANE_PLANNER_EXPORTS)
#elif defined(__linux__)
  #define LANE_PLANNER_PUBLIC __attribute__((visibility("default")))
  #define LANE_PLANNER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define LANE_PLANNER_PUBLIC __attribute__((visibility("default")))
  #define LANE_PLANNER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // LANE_PLANNER__VISIBILITY_CONTROL_HPP_
