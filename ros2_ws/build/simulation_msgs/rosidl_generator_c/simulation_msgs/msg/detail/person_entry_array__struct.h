// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from simulation_msgs:msg/PersonEntryArray.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY_ARRAY__STRUCT_H_
#define SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY_ARRAY__STRUCT_H_

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
// Member 'people'
#include "simulation_msgs/msg/detail/person_entry__struct.h"

/// Struct defined in msg/PersonEntryArray in the package simulation_msgs.
/**
  * Message defining an array of all people entries
 */
typedef struct simulation_msgs__msg__PersonEntryArray
{
  /// Age of the track
  std_msgs__msg__Header header;
  /// Array containing the entries for the N tracked persons
  simulation_msgs__msg__PersonEntry__Sequence people;
} simulation_msgs__msg__PersonEntryArray;

// Struct for a sequence of simulation_msgs__msg__PersonEntryArray.
typedef struct simulation_msgs__msg__PersonEntryArray__Sequence
{
  simulation_msgs__msg__PersonEntryArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} simulation_msgs__msg__PersonEntryArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY_ARRAY__STRUCT_H_
