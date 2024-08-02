// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from simulation_msgs:msg/SceneInfo.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__SCENE_INFO__STRUCT_H_
#define SIMULATION_MSGS__MSG__DETAIL__SCENE_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'scenario_name'
// Member 'environment'
#include "rosidl_runtime_c/string.h"
// Member 'robot_start_pose'
// Member 'robot_target_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/SceneInfo in the package simulation_msgs.
/**
  * Message containing the parameters for Unity Scene trials
 */
typedef struct simulation_msgs__msg__SceneInfo
{
  std_msgs__msg__Header header;
  /// Which scene we are running
  rosidl_runtime_c__String scenario_name;
  /// Pose of robot start location
  geometry_msgs__msg__Pose robot_start_pose;
  /// Pose of robot target location
  geometry_msgs__msg__Pose robot_target_pose;
  /// Number of people in scene
  uint16_t num_people;
  /// Number of groups in scene
  uint16_t num_groups;
  /// Which environment
  rosidl_runtime_c__String environment;
} simulation_msgs__msg__SceneInfo;

// Struct for a sequence of simulation_msgs__msg__SceneInfo.
typedef struct simulation_msgs__msg__SceneInfo__Sequence
{
  simulation_msgs__msg__SceneInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} simulation_msgs__msg__SceneInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SIMULATION_MSGS__MSG__DETAIL__SCENE_INFO__STRUCT_H_
