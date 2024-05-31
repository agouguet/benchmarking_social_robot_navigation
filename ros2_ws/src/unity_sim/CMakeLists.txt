cmake_minimum_required(VERSION 3.5)
project(unity_sim)

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
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

ament_export_dependencies(rosidl_default_runtime)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  maps
  graphs
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/trial_info_listener.py
  scripts/graph_publisher.py
  scripts/map_publisher.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()





