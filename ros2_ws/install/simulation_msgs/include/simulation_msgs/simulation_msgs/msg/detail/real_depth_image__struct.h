// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from simulation_msgs:msg/RealDepthImage.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__REAL_DEPTH_IMAGE__STRUCT_H_
#define SIMULATION_MSGS__MSG__DETAIL__REAL_DEPTH_IMAGE__STRUCT_H_

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
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/RealDepthImage in the package simulation_msgs.
/**
  * Message defining an array of real depth image
 */
typedef struct simulation_msgs__msg__RealDepthImage
{
  /// Header
  std_msgs__msg__Header header;
  /// Array float32 data of real depth image
  rosidl_runtime_c__float__Sequence data;
} simulation_msgs__msg__RealDepthImage;

// Struct for a sequence of simulation_msgs__msg__RealDepthImage.
typedef struct simulation_msgs__msg__RealDepthImage__Sequence
{
  simulation_msgs__msg__RealDepthImage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} simulation_msgs__msg__RealDepthImage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SIMULATION_MSGS__MSG__DETAIL__REAL_DEPTH_IMAGE__STRUCT_H_
