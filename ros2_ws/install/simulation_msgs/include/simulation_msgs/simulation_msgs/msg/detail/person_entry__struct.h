// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from simulation_msgs:msg/PersonEntry.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY__STRUCT_H_
#define SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY__STRUCT_H_

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
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/PersonEntry in the package simulation_msgs.
/**
  * Message defining an entry of a person
 */
typedef struct simulation_msgs__msg__PersonEntry
{
  /// Age of the track
  std_msgs__msg__Header header;
  /// Unique ID for each person
  uint64_t track_id;
  /// The following fields are extracted from the Kalman state x and its covariance C
  /// Pose of the track
  geometry_msgs__msg__Pose pose;
  /// Velocity of the track
  geometry_msgs__msg__Twist twist;
} simulation_msgs__msg__PersonEntry;

// Struct for a sequence of simulation_msgs__msg__PersonEntry.
typedef struct simulation_msgs__msg__PersonEntry__Sequence
{
  simulation_msgs__msg__PersonEntry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} simulation_msgs__msg__PersonEntry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY__STRUCT_H_
