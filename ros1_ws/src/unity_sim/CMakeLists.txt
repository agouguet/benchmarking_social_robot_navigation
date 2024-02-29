cmake_minimum_required(VERSION 2.8.3)
project(unity_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
    nav_msgs
    realtime_tools
    tf
    urdf
    pluginlib
    message_generation
    std_msgs
    geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})