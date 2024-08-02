// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from graph_msgs:msg/GraphEdge.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__STRUCT_H_
#define GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/GraphEdge in the package graph_msgs.
typedef struct graph_msgs__msg__GraphEdge
{
  uint64_t id_n1;
  uint64_t id_n2;
} graph_msgs__msg__GraphEdge;

// Struct for a sequence of graph_msgs__msg__GraphEdge.
typedef struct graph_msgs__msg__GraphEdge__Sequence
{
  graph_msgs__msg__GraphEdge * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} graph_msgs__msg__GraphEdge__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__STRUCT_H_
