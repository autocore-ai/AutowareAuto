# FindSpinnaker.cmake
#
# Finds the PointGrey SDK installed on the system.
#
# This will define the following variables
#
#    Spinnaker_FOUND
#    Spinnaker_INCLUDE_DIRS
#    Spinnaker_LIBRARIES
#
# and the following imported targets
#
#     Spinnaker::Spinnaker

find_path(Spinnaker_INCLUDE_DIR
  NAMES spinnaker/Spinnaker.h
)
find_library(Spinnaker_LIBRARY NAMES Spinnaker)

mark_as_advanced(Spinnaker_FOUND Spinnaker_INCLUDE_DIR Spinnaker_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Spinnaker
  REQUIRED_VARS Spinnaker_INCLUDE_DIR Spinnaker_LIBRARY
)

if(Spinnaker_FOUND)
  list(APPEND Spinnaker_INCLUDE_DIRS
    "${Spinnaker_INCLUDE_DIR}"
    "${Spinnaker_INCLUDE_DIR}/spinnaker")
  list(APPEND Spinnaker_LIBRARIES "${Spinnaker_LIBRARY}")
endif()

if(Spinnaker_FOUND AND NOT TARGET Spinnaker::Spinnaker)
  add_library(Spinnaker::Spinnaker IMPORTED SHARED)
  set_target_properties(Spinnaker::Spinnaker PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${Spinnaker_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${Spinnaker_LIBRARIES}"
  )
endif()