// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from agents_msgs:msg/AgentTrajectories.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORIES__STRUCT_H_
#define AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORIES__STRUCT_H_

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
// Member 'trajectories'
#include "agents_msgs/msg/detail/agent_trajectory__struct.h"

/// Struct defined in msg/AgentTrajectories in the package agents_msgs.
typedef struct agents_msgs__msg__AgentTrajectories
{
  /// Age of the track
  std_msgs__msg__Header header;
  agents_msgs__msg__AgentTrajectory__Sequence trajectories;
} agents_msgs__msg__AgentTrajectories;

// Struct for a sequence of agents_msgs__msg__AgentTrajectories.
typedef struct agents_msgs__msg__AgentTrajectories__Sequence
{
  agents_msgs__msg__AgentTrajectories * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} agents_msgs__msg__AgentTrajectories__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORIES__STRUCT_H_
