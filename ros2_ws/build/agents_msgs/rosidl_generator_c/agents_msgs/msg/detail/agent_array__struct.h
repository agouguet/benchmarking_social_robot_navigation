// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from agents_msgs:msg/AgentArray.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT_ARRAY__STRUCT_H_
#define AGENTS_MSGS__MSG__DETAIL__AGENT_ARRAY__STRUCT_H_

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
// Member 'agents'
#include "agents_msgs/msg/detail/agent__struct.h"

/// Struct defined in msg/AgentArray in the package agents_msgs.
typedef struct agents_msgs__msg__AgentArray
{
  std_msgs__msg__Header header;
  agents_msgs__msg__Agent__Sequence agents;
} agents_msgs__msg__AgentArray;

// Struct for a sequence of agents_msgs__msg__AgentArray.
typedef struct agents_msgs__msg__AgentArray__Sequence
{
  agents_msgs__msg__AgentArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} agents_msgs__msg__AgentArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT_ARRAY__STRUCT_H_
