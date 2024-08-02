// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from agents_msgs:msg/AgentTrajectory.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORY__STRUCT_H_
#define AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORY__STRUCT_H_

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
// Member 'poses'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/AgentTrajectory in the package agents_msgs.
/**
  * Message defining an trajectory of a agent
 */
typedef struct agents_msgs__msg__AgentTrajectory
{
  /// Age of the track
  std_msgs__msg__Header header;
  /// Unique ID for each agent
  uint64_t id;
  /// Poses of the trajectory
  geometry_msgs__msg__Point__Sequence poses;
} agents_msgs__msg__AgentTrajectory;

// Struct for a sequence of agents_msgs__msg__AgentTrajectory.
typedef struct agents_msgs__msg__AgentTrajectory__Sequence
{
  agents_msgs__msg__AgentTrajectory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} agents_msgs__msg__AgentTrajectory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORY__STRUCT_H_
