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

#ifndef GNSS_CONVERSION_NODES__VISIBILITY_CONTROL_HPP_
#define GNSS_CONVERSION_NODES__VISIBILITY_CONTROL_HPP_


#if defined(__WIN32)
  #if defined(GNSS_CONVERSION_NODE_BUILDING_DLL) || defined(GNSS_CONVERSION_NODE_EXPORTS)
    #define GNSS_CONVERSION_NODE_PUBLIC __declspec(dllexport)
    #define GNSS_CONVERSION_NODE_LOCAL
  #else  // defined(GNSS_CONVERSION_NODE_BUILDING_DLL) || defined(GNSS_CONVERSION_NODE_EXPORTS)
    #define GNSS_CONVERSION_NODE_PUBLIC __declspec(dllimport)
    #define GNSS_CONVERSION_NODE_LOCAL
  #endif  // defined(GNSS_CONVERSION_NODE_BUILDING_DLL) || defined(GNSS_CONVERSION_NODE_EXPORTS)
#elif defined(__linux__)
  #define GNSS_CONVERSION_NODE_PUBLIC __attribute__((visibility("default")))
  #define GNSS_CONVERSION_NODE_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define GNSS_CONVERSION_NODE_PUBLIC __attribute__((visibility("default")))
  #define GNSS_CONVERSION_NODE_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // GNSS_CONVERSION_NODES__VISIBILITY_CONTROL_HPP_
