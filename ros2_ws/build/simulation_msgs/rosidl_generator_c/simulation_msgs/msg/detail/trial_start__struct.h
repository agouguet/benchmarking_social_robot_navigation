// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from simulation_msgs:msg/TrialStart.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__TRIAL_START__STRUCT_H_
#define SIMULATION_MSGS__MSG__DETAIL__TRIAL_START__STRUCT_H_

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
// Member 'trial_name'
#include "rosidl_runtime_c/string.h"
// Member 'spawn'
// Member 'target'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'people'
#include "geometry_msgs/msg/detail/pose_array__struct.h"

/// Struct defined in msg/TrialStart in the package simulation_msgs.
/**
  * Message containing the parameters to start an A-B navigation trial
 */
typedef struct simulation_msgs__msg__TrialStart
{
  std_msgs__msg__Header header;
  /// Which trial name are we running
  rosidl_runtime_c__String trial_name;
  /// Which trial number are we running
  uint16_t trial_number;
  /// Robot spawn position
  geometry_msgs__msg__Pose spawn;
  /// Robot target position
  geometry_msgs__msg__Pose target;
  /// People spawn positions
  geometry_msgs__msg__PoseArray people;
  /// Time limit for the trial (in seconds)
  double time_limit;
} simulation_msgs__msg__TrialStart;

// Struct for a sequence of simulation_msgs__msg__TrialStart.
typedef struct simulation_msgs__msg__TrialStart__Sequence
{
  simulation_msgs__msg__TrialStart * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} simulation_msgs__msg__TrialStart__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SIMULATION_MSGS__MSG__DETAIL__TRIAL_START__STRUCT_H_
