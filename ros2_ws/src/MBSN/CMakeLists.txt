cmake_minimum_required(VERSION 3.5)
project(mbsn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
# find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/graph_visualization.py
  scripts/MBSN.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
