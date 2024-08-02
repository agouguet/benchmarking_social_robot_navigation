// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from graph_msgs:msg/GraphNode.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSGS__MSG__DETAIL__GRAPH_NODE__STRUCT_H_
#define GRAPH_MSGS__MSG__DETAIL__GRAPH_NODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/GraphNode in the package graph_msgs.
typedef struct graph_msgs__msg__GraphNode
{
  uint64_t id;
  double x;
  double y;
  bool occupied;
} graph_msgs__msg__GraphNode;

// Struct for a sequence of graph_msgs__msg__GraphNode.
typedef struct graph_msgs__msg__GraphNode__Sequence
{
  graph_msgs__msg__GraphNode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} graph_msgs__msg__GraphNode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GRAPH_MSGS__MSG__DETAIL__GRAPH_NODE__STRUCT_H_
