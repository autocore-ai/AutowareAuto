# Create tests for the memory tools library.
add_executable(test_memory_tools test_memory_tools.cpp)
target_link_libraries(test_memory_tools
  memory_tools
  gtest_main
)

# memory_tools doesn't work on Windows right now
# TODO(wjwwood): disable on aarch64 for now, until fixed, see:
#   https://github.com/osrf/osrf_testing_tools_cpp/issues/3
if(NOT WIN32 AND NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  add_test(
    NAME "test_memory_tools"
    COMMAND
      "$<TARGET_FILE:test_runner>"
      --env
        ${memory_tools_extra_test_env}
      --
      "$<TARGET_FILE:test_memory_tools>"
  )
endif()
