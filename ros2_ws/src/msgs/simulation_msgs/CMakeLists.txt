cmake_minimum_required(VERSION 3.5)
project(simulation_msgs)


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
