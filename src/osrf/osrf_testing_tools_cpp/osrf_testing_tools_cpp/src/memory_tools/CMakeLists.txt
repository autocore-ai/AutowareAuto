add_library(memory_tools SHARED
  custom_memory_functions.cpp
  implementation_monitoring_override.cpp
  initialize.cpp
  is_working.cpp
  memory_tools_service.cpp
  monitoring.cpp
  register_hooks.cpp
  stack_trace.cpp
  testing_helpers.cpp
  verbosity.cpp
)
target_include_directories(memory_tools
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
if(UNIX AND NOT APPLE)
  # On Linux like systems, add dl and use the normal library and LD_PRELOAD.
  target_link_libraries(memory_tools dl)
endif()
target_compile_definitions(memory_tools
  PRIVATE "OSRF_TESTING_TOOLS_CPP_MEMORY_TOOLS_BUILDING_DLL")

add_library(memory_tools_interpose SHARED
  memory_tools.cpp
)
target_link_libraries(memory_tools_interpose memory_tools)

# TODO(wjwwood): If on aarch64, so not set the preload environment variables, until fixed.
# see: https://github.com/osrf/osrf_testing_tools_cpp/issues/3
if(NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  if(APPLE)
    list(APPEND memory_tools_extra_test_env
      DYLD_INSERT_LIBRARIES=$<TARGET_FILE:memory_tools_interpose>)
  elseif(UNIX)
    list(APPEND memory_tools_extra_test_env
      LD_PRELOAD=$<TARGET_FILE:memory_tools_interpose>)
  endif()
endif()

install(TARGETS memory_tools
  EXPORT memory_tools
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(EXPORT memory_tools
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE "${PROJECT_NAME}::"
  FILE "memory_toolsExport.cmake"
)

install(TARGETS memory_tools_interpose
  EXPORT memory_tools_interpose
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(EXPORT memory_tools_interpose
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE "${PROJECT_NAME}::"
  FILE "memory_tools_interposeExport.cmake"
)

set(memory_tools_extra_test_env "${memory_tools_extra_test_env}" PARENT_SCOPE)
