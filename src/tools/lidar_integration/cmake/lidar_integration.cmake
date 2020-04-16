# Copyright 2018 Apex.AI, Inc.
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# :param CMD: (bash) command to be run, should be a node of some kind,
#             must be compatible with timeout
# :type CMD: a string
# :param CMD2: (bash) command to be run, should be a node of some kind,
#             must be compatible with timeout
# :type CMD2: a string
# :param CMD3: (bash) command to be run, should be a node of some kind,
#             must be compatible with timeout
# :type CMD3: a string
# :param PORT1: port to reserve for first spoofer
# :type PORT1: a valid port number
# :param PORT2: port to reserver for second spoofer
# :type PORT2: a valid port number
# :param PCL_TOPIC1: topic of first point cloud to listen to
# :type PCL_TOPIC1: a string
# :param PCL_TOPIC2:  topic of second point cloud to listen to
# :type PCL_TOPIC2: a string
# :param PCL_TOPIC3: topic of third point cloud to listen to
# :type PCL_TOPIC3: a string
# :param BOX_TOPIC:  topic of bounding boxes to listen to
# :type BOX_TOPIC: a string
# :param PCL_SIZE1: expected size of first point cloud
# :type PCL_SIZE1: a number
# :param PCL_SIZE2: expected size of first point cloud
# :type PCL_SIZE2: a number
# :param PCL_SIZE3:expected size of first point cloud
# :type PCL_SIZE3: a number
# :param BOX_SIZE: expected size of first point cloud
# :type BOX_SIZE: a number
# :param SIZE_TOL: relative tolerance for size
# :type SIZE_TOL: a number in (0, 1)
# :param PERIOD_TOL: relative tolerance for average periodicity, typically very
#                    loose (e.g. 0.6 or higher) due to slow build servers
# :type PERIOD_TOL: a number in (0, 1)
# :param PERIOD: expected time in ms between messages
# :type PERIOD: a number
# :param RPM: effective spin rate of spoofer
# :type RPM: a number in [300, 1200]
# :param RUNTIME: time to run test for in seconds
# :type RUNTIME: a number
# :param NAME: name of test
# :type NAME: string
# :param LIFECYCLE_NODE: regular node (False) or lifecycle managed node (True)
# :type LIFECYCLE_NODE: boolean
# :param SUFFIX: optional suffix of this test. This is to handle
#                        multipletests in same packages.
# :type SUFFIX: a string
# :param IGNORE_CONSOLE: Explicitly ignore log_writer output by
#                        matching it with .*. Otherwise, developers have to
#                        prepare `${TEST_CMD}[${SUFFIX}].regex` in
#                        `<package>/test/expected_outputs/`
# :type IGNORE_CONSOLE: option

function(lidar_integration_test)
  # parse arguments
  set(ARGNAMES
    "CMD;PORT1;PORT2;PCL_TOPIC1;PCL_TOPIC2;PCL_TOPIC3;BLK_TOPIC1;BLK_TOPIC2;"
    "BOX_TOPIC;PCL_SIZE1;PCL_SIZE2;PCL_SIZE3;BOX_SIZE;BLK_SIZE1;BLK_SIZE2;"
    "SIZE_TOL;PERIOD_TOL;PERIOD;RPM;RUNTIME;CMD2;NAME;LIFECYCLE_NODE;CMD3"
    "SUFFIX"
  )
  cmake_parse_arguments(ARG
    "IGNORE_CONSOLE"
    "${ARGNAMES}"
    ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_add_test() called with unused arguments: "
      "${ARG_UNPARSED_ARGUMENTS}")
  endif()
  if(NOT ARG_CMD)
    message(FATAL_ERROR "lidar_integration_test() must have CMD")
  endif()
  if(NOT ARG_PORT1)
    message(FATAL_ERROR "lidar_integration_test() must have at least one PORT")
  endif()
  if(ARG_PORT2)
    set(DO_SECOND_SPOOF "--port2=${ARG_PORT2} --do_second_spoof")
  endif()
  if(NOT ARG_PCL_TOPIC1)
    set(ARG_PCL_TOPIC1 "")
  endif()
  if(NOT ARG_PCL_TOPIC2)
    set(ARG_PCL_TOPIC2 "")
  endif()
  if(NOT ARG_PCL_TOPIC3)
    set(ARG_PCL_TOPIC3 "")
  endif()
  if(NOT BOX_TOPIC)
    set(BOX_TOPIC "")
  endif()
  if(NOT ARG_BLK_TOPIC1)
    set(ARG_BLK_TOPIC1 "")
  endif()
  if(NOT ARG_BLK_TOPIC2)
    set(ARG_BLK_TOPIC2 "")
  endif()
  if((NOT ARG_PCL_TOPIC1) AND (NOT ARG_PCL_TOPIC2) AND (NOT ARG_PCL_TOPIC3) AND (NOT ARG_BOX_TOPIC
    ) AND (NOT ARG_BLK_TOPIC1) AND (NOT ARG_BLK_TOPIC2))
    message(FATAL_ERROR "lidar_integration_test() must have at least one topic")
  endif()
  if(ARG_PCL_TOPIC1 AND NOT ARG_PCL_SIZE1)
    message(FATAL_ERROR "lidar_integration_test() must have SIZE specified if TOPIC is")
  endif()
  if(ARG_PCL_TOPIC2 AND NOT ARG_PCL_SIZE2)
    message(FATAL_ERROR "lidar_integration_test() must have SIZE specified if TOPIC is")
  endif()
  if(ARG_PCL_TOPIC3 AND NOT ARG_PCL_SIZE3)
    message(FATAL_ERROR "lidar_integration_test() must have SIZE specified if TOPIC is")
  endif()
  if(ARG_BOX_TOPIC AND NOT ARG_BOX_SIZE)
    message(FATAL_ERROR "lidar_integration_test() must have SIZE specified if TOPIC is")
  endif()
  if(ARG_BLK_TOPIC1 AND NOT ARG_BLK_SIZE1)
    message(FATAL_ERROR "lidar_integration_test() must have SIZE specified if TOPIC is")
  endif()
  if(ARG_BLK_TOPIC2 AND NOT ARG_BLK_SIZE2)
    message(FATAL_ERROR "lidar_integration_test() must have SIZE specified if TOPIC is")
  endif()
  if(CMAKE_BUILD_TYPE MATCHES "Coverage")
    message(STATUS "\n\n\nCoverage run active!\n\n\n")
    set(COVERAGE_RUN ON)
  endif()
  if(NOT ARG_PERIOD_TOL)
    if(COVERAGE_RUN)
      # arbitrarily high value to pass coverage, could maybe be as low as 5
      set(ARG_PERIOD_TOL "15.0")
    else()
      # corresponds to 70% error, CI server is pretty jittery
      set(ARG_PERIOD_TOL "0.7")
    endif()
  endif()
  if(NOT ARG_SIZE_TOL)
    if(COVERAGE_RUN)
      # arbitrarily high value to pass coverage, could maybe be as low as 5
      set(ARG_SIZE_TOL "15.0")
    else()
      # corresponds to 10% error, typically size is accurate on CI server
      set(ARG_SIZE_TOL "0.1")
    endif()
  endif()
  if(NOT ARG_PERIOD AND NOT ARG_RPM)
    # corresponds to 100ms period == 600 rpm on vlp16 hires, our default sensor setting
    set(ARG_PERIOD "100")
    set(ARG_RPM "600")
  elseif(NOT ARG_PERIOD)
    # TODO compute period
    message(FATAL_ERROR "lidar_integration_test() must have PERIOD and RPM as a pair")
  elseif(NOT ARG_RPM)
    # TODO compute rpm
    message(FATAL_ERROR "lidar_integration_test() must have PERIOD and RPM as a pair")
  endif()
  if(NOT ARG_NAME)
    set(TEST_NAME "${PROJECT_NAME}")
  else()
    set(TEST_NAME "${ARG_NAME}")
  endif()
  if(NOT ARG_RUNTIME)
    set(ARG_RUNTIME 10)
  endif()
  if(NOT ARG_LIFECYCLE_NODE)
    set(ARG_LIFECYCLE_NODE "")
  endif()
  if(NOT ARG_SUFFIX)
    set(ARG_SUFFIX "")
  endif()
  if(ARG_CMD3 AND (NOT ARG_CMD2))
    message(FATAL_ERROR "lidar_integration_test() Using CMD3 without CMD2")
  endif()

  # localhost
  set(IP "127.0.0.1")

  string(CONCAT PORT_MESSAGE "Using ports " ${ARG_PORT1} ", " ${ARG_PORT2})
  message(STATUS ${PORT_MESSAGE})

  # get dependencies
  find_package(autoware_auto_integration_tests REQUIRED)

  # set up input arguments
  string(CONCAT SPOOF_CMD
    "lidar_integration::vlp16_integration_spoofer_exe "
    "--rpm=${ARG_RPM} --runtime=${ARG_RUNTIME} --ip1=${IP} --ip2=${IP} --port1=${ARG_PORT1} "
    "${DO_SECOND_SPOOF}")

  string(CONCAT LISTEN_CMD
    "lidar_integration::lidar_integration_listener_exe "
    "--pcl_topic1=${ARG_PCL_TOPIC1} --pcl_topic2=${ARG_PCL_TOPIC2} --pcl_topic3=${ARG_PCL_TOPIC3} "
    "--box_topic=${ARG_BOX_TOPIC} --pcl_size1=${ARG_PCL_SIZE1} --pcl_size2=${ARG_PCL_SIZE2} "
    "--pcl_size3=${ARG_PCL_SIZE3} --box_size=${ARG_BOX_SIZE} --period=${ARG_PERIOD} "
    "--blk_size1=${ARG_BLK_SIZE1} --blk_size2=${ARG_BLK_SIZE2} --blk_topic1=${ARG_BLK_TOPIC1} "
    "--blk_topic2=${ARG_BLK_TOPIC2} "
    "--period_tolerance=${ARG_PERIOD_TOL} --size_tolerance=${ARG_SIZE_TOL} --runtime=${ARG_RUNTIME} "
    "${ARG_LIFECYCLE_NODE}")

  string(CONCAT FULL_TEST_CMD
    "${CONSOLE_CMD}:::"
    "${SPOOF_CMD}:::"
    "${ARG_CMD}:::"
    "${LISTEN_CMD}")

  if(ARG_CMD2)
    # Add second command
    string(CONCAT FULL_TEST_CMD
      "${ARG_CMD2}:::"
      "${FULL_TEST_CMD}"
    )

    # Create tested node REGEX with ARG_SUFFIX appended.
    string(REPLACE " " ";" TEST_CMD_LIST2 ${ARG_CMD2})
    list(GET TEST_CMD_LIST2 0 TEST_EXE2_)
    string(REPLACE "::" "__" TEST_EXE2 ${TEST_EXE2_})
    file(WRITE "${CMAKE_BINARY_DIR}/expected_outputs/${TEST_EXE2}${ARG_SUFFIX}.regex" ".*")

    if(ARG_CMD3)
      # Add third command
      string(CONCAT FULL_TEST_CMD
        "${ARG_CMD3}:::"
        "${FULL_TEST_CMD}"
      )

      # Create tested node REGEX with ARG_SUFFIX appended.
      string(REPLACE " " ";" TEST_CMD_LIST3 ${ARG_CMD3})
      list(GET TEST_CMD_LIST3 0 TEST_EXE3_)
      string(REPLACE "::" "__" TEST_EXE3 ${TEST_EXE3_})
      file(WRITE "${CMAKE_BINARY_DIR}/expected_outputs/${TEST_EXE3}${ARG_SUFFIX}.regex" ".*")
    endif()
  endif()

  # Copy spoofer/listener regex to node binary folder
  set(SRC_REGEX_FOLDER "${lidar_integration_DIR}/../expected_outputs")
  file(GLOB REGEX_FILES "${SRC_REGEX_FOLDER}/*.regex")
  foreach(file ${REGEX_FILES})
    get_filename_component(file_name ${file} NAME_WE)
    configure_file(${file}
      "${CMAKE_CURRENT_BINARY_DIR}/expected_outputs/${file_name}${ARG_SUFFIX}.regex")
  endforeach()

  integration_tests(
    TESTNAME ${TEST_NAME}
    COMMANDS ${FULL_TEST_CMD}
    LABELS "lidar_integration"
    SUFFIX ${ARG_SUFFIX}
  )
endfunction()
