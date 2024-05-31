cmake_minimum_required(VERSION 3.5)
project(my_controller)

# Warnings
add_compile_options(-Wall -Wextra
  -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls -Wcast-qual
  -Wno-unused-parameter -Wno-unused-function)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # This too often has false-positives
  add_compile_options(-Wno-maybe-uninitialized)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_2d_msgs
  dwb_msgs
  nav2_costmap_2d
  pluginlib
  sensor_msgs
  visualization_msgs
  nav_2d_utils
  nav_msgs
  tf2_ros
  nav2_util
  nav2_core
)

add_library(my_controller SHARED
  src/controller.cpp
)

ament_target_dependencies(my_controller
  ${dependencies}
)

install(TARGETS my_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(my_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

ament_package()
