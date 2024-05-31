cmake_minimum_required(VERSION 3.5)
project(simulation_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PersonEntry.msg"
  "msg/PersonEntryArray.msg"
  "msg/RealDepthImage.msg"
  "msg/SceneInfo.msg"
  "msg/TrialStart.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

install(
  FILES bridge_mapping.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
