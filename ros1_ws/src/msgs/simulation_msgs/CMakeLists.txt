cmake_minimum_required(VERSION 2.8.12)
project(simulation_msgs)

set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    controller_manager
    control_msgs
    nav_msgs
    realtime_tools
    tf
    urdf
    pluginlib
    message_generation
    std_msgs
    geometry_msgs
)

## add message, service and action files

add_message_files(
  FILES
    PersonEntry.msg
    PersonEntryArray.msg
    RealDepthImage.msg
    SceneInfo.msg
    TrialStart.msg
)


generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

## declare catkin package
catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    geometry_msgs
    tf
    visualization_msgs
)

