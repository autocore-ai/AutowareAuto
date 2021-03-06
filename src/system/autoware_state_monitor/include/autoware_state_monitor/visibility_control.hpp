// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#ifndef AUTOWARE_STATE_MONITOR__VISIBILITY_CONTROL_HPP_
#define AUTOWARE_STATE_MONITOR__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(AUTOWARE_STATE_MONITOR_BUILDING_DLL) || defined(AUTOWARE_STATE_MONITOR_EXPORTS)
    #define AUTOWARE_STATE_MONITOR_PUBLIC __declspec(dllexport)
    #define AUTOWARE_STATE_MONITOR_LOCAL
  #else  // defined(AUTOWARE_STATE_MONITOR_BUILDING_DLL) || defined(AUTOWARE_STATE_MONITOR_EXPORTS)
    #define AUTOWARE_STATE_MONITOR_PUBLIC __declspec(dllimport)
    #define AUTOWARE_STATE_MONITOR_LOCAL
  #endif  // defined(AUTOWARE_STATE_MONITOR_BUILDING_DLL) || defined(AUTOWARE_STATE_MONITOR_EXPORTS)
#elif defined(__linux__)
  #define AUTOWARE_STATE_MONITOR_PUBLIC __attribute__((visibility("default")))
  #define AUTOWARE_STATE_MONITOR_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define AUTOWARE_STATE_MONITOR_PUBLIC __attribute__((visibility("default")))
  #define AUTOWARE_STATE_MONITOR_LOCAL __attribute__((visibility("hidden")))
#else  // defined(__linux__)
  #error "Unsupported Build Configuration"
#endif  // defined(__WIN32)

#endif  // AUTOWARE_STATE_MONITOR__VISIBILITY_CONTROL_HPP_
