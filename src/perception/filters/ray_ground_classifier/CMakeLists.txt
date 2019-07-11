cmake_minimum_required(VERSION 3.5)

### Export headers
project(ray_ground_classifier)

## dependencies
find_package(autoware_auto_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(lidar_utils REQUIRED)
find_package(autoware_auto_algorithm REQUIRED)

# includes
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  include/ray_ground_classifier/ray_aggregator.hpp
  include/ray_ground_classifier/ray_ground_classifier.hpp
  include/ray_ground_classifier/ray_ground_point_classifier.hpp
  include/ray_ground_classifier/visibility_control.hpp
  src/ray_aggregator.cpp
  src/ray_ground_point_classifier.cpp
  src/ray_ground_classifier.cpp
  src/ray_ground_classifier_types.cpp)
autoware_set_compile_options(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}
  "lidar_utils"
  "autoware_auto_algorithm"
)
if(BUILD_TESTING)
  autoware_static_code_analysis()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ray_ground_classifier_gtest
    test/src/test_ray_ground_classifier.cpp
  )
  target_include_directories(test_ray_ground_classifier_gtest
    PRIVATE "test/include" "include"
  )
  target_link_libraries(test_ray_ground_classifier_gtest
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_ray_ground_classifier_raytrace_gtest
    test/src/test_ray_ground_classifier_raytrace.cpp
  )
  target_include_directories(test_ray_ground_classifier_raytrace_gtest
    PRIVATE "test/include" "include"
  )
  target_link_libraries(test_ray_ground_classifier_raytrace_gtest
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_ray_aggregator_gtest
    test/src/test_ray_aggregator.cpp
  )
  target_include_directories(test_ray_aggregator_gtest
    PRIVATE "test/include" "include"
  )
  target_link_libraries(test_ray_aggregator_gtest
    ${PROJECT_NAME}
  )
endif()

## install stuff
autoware_install(
  HAS_INCLUDE
  LIBRARIES ${PROJECT_NAME}
)

# workaround to disable sign conversion errors from sensor_msgs::PointCloud2Iterator
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion)

# Ament Exporting
ament_export_dependencies(
  "lidar_utils"
  "autoware_auto_algorithm"
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

ament_package()
