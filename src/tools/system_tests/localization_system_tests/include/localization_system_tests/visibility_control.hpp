// Copyright 2021 the Autoware Foundation
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
/// \file
/// \brief Visibility control
#ifndef LOCALIZATION_SYSTEM_TESTS__VISIBILITY_CONTROL_HPP_
#define LOCALIZATION_SYSTEM_TESTS__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define LOCALIZATION_SYSTEM_TESTS_EXPORT __attribute__ ((dllexport))
    #define LOCALIZATION_SYSTEM_TESTS_IMPORT __attribute__ ((dllimport))
  #else
    #define LOCALIZATION_SYSTEM_TESTS_EXPORT __declspec(dllexport)
    #define LOCALIZATION_SYSTEM_TESTS_IMPORT __declspec(dllimport)
  #endif
  #ifdef LOCALIZATION_SYSTEM_TESTS_BUILDING_LIBRARY
    #define LOCALIZATION_SYSTEM_TESTS_PUBLIC LOCALIZATION_SYSTEM_TESTS_EXPORT
  #else
    #define LOCALIZATION_SYSTEM_TESTS_PUBLIC LOCALIZATION_SYSTEM_TESTS_IMPORT
  #endif
  #define LOCALIZATION_SYSTEM_TESTS_PUBLIC_TYPE LOCALIZATION_SYSTEM_TESTS_PUBLIC
  #define LOCALIZATION_SYSTEM_TESTS_LOCAL
#else
  #define LOCALIZATION_SYSTEM_TESTS_EXPORT __attribute__ ((visibility("default")))
  #define LOCALIZATION_SYSTEM_TESTS_IMPORT
  #if __GNUC__ >= 4
    #define LOCALIZATION_SYSTEM_TESTS_PUBLIC __attribute__ ((visibility("default")))
    #define LOCALIZATION_SYSTEM_TESTS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define LOCALIZATION_SYSTEM_TESTS_PUBLIC
    #define LOCALIZATION_SYSTEM_TESTS_LOCAL
  #endif
  #define LOCALIZATION_SYSTEM_TESTS_PUBLIC_TYPE
#endif

#endif  // LOCALIZATION_SYSTEM_TESTS__VISIBILITY_CONTROL_HPP_
