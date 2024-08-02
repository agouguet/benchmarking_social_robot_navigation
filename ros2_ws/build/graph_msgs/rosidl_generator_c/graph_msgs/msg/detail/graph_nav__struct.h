// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from graph_msgs:msg/GraphNav.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSGS__MSG__DETAIL__GRAPH_NAV__STRUCT_H_
#define GRAPH_MSGS__MSG__DETAIL__GRAPH_NAV__STRUCT_H_

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
// Member 'edges'
#include "graph_msgs/msg/detail/graph_edge__struct.h"
// Member 'nodes'
#include "graph_msgs/msg/detail/graph_node__struct.h"

/// Struct defined in msg/GraphNav in the package graph_msgs.
typedef struct graph_msgs__msg__GraphNav
{
  std_msgs__msg__Header header;
  graph_msgs__msg__GraphEdge__Sequence edges;
  graph_msgs__msg__GraphNode__Sequence nodes;
} graph_msgs__msg__GraphNav;

// Struct for a sequence of graph_msgs__msg__GraphNav.
typedef struct graph_msgs__msg__GraphNav__Sequence
{
  graph_msgs__msg__GraphNav * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} graph_msgs__msg__GraphNav__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GRAPH_MSGS__MSG__DETAIL__GRAPH_NAV__STRUCT_H_
