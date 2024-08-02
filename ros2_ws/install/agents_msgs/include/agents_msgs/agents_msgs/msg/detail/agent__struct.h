// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from agents_msgs:msg/Agent.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT__STRUCT_H_
#define AGENTS_MSGS__MSG__DETAIL__AGENT__STRUCT_H_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/Agent in the package agents_msgs.
typedef struct agents_msgs__msg__Agent
{
  std_msgs__msg__Header header;
  uint64_t id;
  geometry_msgs__msg__Pose pose;
  geometry_msgs__msg__Twist velocity;
  bool visible_by_robot;
} agents_msgs__msg__Agent;

// Struct for a sequence of agents_msgs__msg__Agent.
typedef struct agents_msgs__msg__Agent__Sequence
{
  agents_msgs__msg__Agent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} agents_msgs__msg__Agent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT__STRUCT_H_
