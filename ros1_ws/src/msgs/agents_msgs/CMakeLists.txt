cmake_minimum_required(VERSION 2.8.3)
project(agents_msgs)

set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  nav_msgs
  tf
  visualization_msgs
)

## add message, service and action files

add_message_files( DIRECTORY msg
  FILES
    Agent.msg
    AgentArray.msg
    AgentTrajectory.msg
    AgentTrajectories.msg
)

## generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
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
