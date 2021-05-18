# Copyright 2021 Arm Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(neural_networks)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(SUPPORTED_ARCH "aarch64" "x86_64")
set(USER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/user/${CMAKE_SYSTEM_PROCESSOR})

set(PREFIX_DIR ${CMAKE_BINARY_DIR}/download)
file(MAKE_DIRECTORY ${PREFIX_DIR}/src/networks)

if(CMAKE_SYSTEM_PROCESSOR IN_LIST SUPPORTED_ARCH)
  # Prevent re-downloading the pre-compiled networks on every build command.
  if(NOT EXISTS ${PREFIX_DIR}/src/networks-${CMAKE_SYSTEM_PROCESSOR}.tar.gz)
    # Check if the user wants to download the artifacts.
    if(DOWNLOAD_ARTIFACTS)
      include(ExternalProject)
      externalproject_add(networks
        PREFIX ${PREFIX_DIR}
        DOWNLOAD_DIR ${PREFIX_DIR}/src
        SOURCE_DIR ${PREFIX_DIR}/src/networks
        URL https://autoware-modelzoo.s3.us-east-2.amazonaws.com/networks-${CMAKE_SYSTEM_PROCESSOR}.tar.gz
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
      )
    else()
      message(WARNING "Skipped download (enable by setting DOWNLOAD_ARTIFACTS)")
    endif()
  endif()
else()
  message(WARNING "Skipped download (unsupported architecture)")
endif()

# Install the pre-compiled networks
install(
  DIRECTORY ${PREFIX_DIR}/src/networks
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

# Install user-provided networks over the pre-compiled ones
if(EXISTS ${USER_DIR})
  install(
    DIRECTORY ${USER_DIR}/
    DESTINATION share/${PROJECT_NAME}/networks/
    USE_SOURCE_PERMISSIONS
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "neural_networks-extras.cmake")
ament_auto_package()
