// Copyright 2021 Arm Ltd.
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

#ifndef APOLLO_LIDAR_SEGMENTATION_NODES__VISIBILITY_CONTROL_HPP_
#define APOLLO_LIDAR_SEGMENTATION_NODES__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(APOLLO_LIDAR_SEGMENTATION_NODES_BUILDING_DLL) || \
  defined(APOLLO_LIDAR_SEGMENTATION_NODES_EXPORTS)
    #define APOLLO_LIDAR_SEGMENTATION_NODES_PUBLIC __declspec(dllexport)
    #define APOLLO_LIDAR_SEGMENTATION_NODES_LOCAL
  #else
    #define APOLLO_LIDAR_SEGMENTATION_NODES_PUBLIC __declspec(dllimport)
    #define APOLLO_LIDAR_SEGMENTATION_NODES_LOCAL
  #endif
#elif defined(__linux__)
  #define APOLLO_LIDAR_SEGMENTATION_NODES_PUBLIC __attribute__((visibility("default")))
  #define APOLLO_LIDAR_SEGMENTATION_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define APOLLO_LIDAR_SEGMENTATION_NODES_PUBLIC __attribute__((visibility("default")))
  #define APOLLO_LIDAR_SEGMENTATION_NODES_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // APOLLO_LIDAR_SEGMENTATION_NODES__VISIBILITY_CONTROL_HPP_
