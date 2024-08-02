// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from simulation_msgs:msg/TrialStart.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "simulation_msgs/msg/detail/trial_start__rosidl_typesupport_introspection_c.h"
#include "simulation_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "simulation_msgs/msg/detail/trial_start__functions.h"
#include "simulation_msgs/msg/detail/trial_start__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `trial_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `spawn`
// Member `target`
#include "geometry_msgs/msg/pose.h"
// Member `spawn`
// Member `target`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `people`
#include "geometry_msgs/msg/pose_array.h"
// Member `people`
#include "geometry_msgs/msg/detail/pose_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  simulation_msgs__msg__TrialStart__init(message_memory);
}

void simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_fini_function(void * message_memory)
{
  simulation_msgs__msg__TrialStart__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__TrialStart, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "trial_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__TrialStart, trial_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "trial_number",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__TrialStart, trial_number),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "spawn",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__TrialStart, spawn),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__TrialStart, target),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "people",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__TrialStart, people),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_limit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__TrialStart, time_limit),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_message_members = {
  "simulation_msgs__msg",  // message namespace
  "TrialStart",  // message name
  7,  // number of fields
  sizeof(simulation_msgs__msg__TrialStart),
  simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_message_member_array,  // message members
  simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_init_function,  // function to initialize message memory (memory has to be allocated)
  simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_message_type_support_handle = {
  0,
  &simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_simulation_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, simulation_msgs, msg, TrialStart)() {
  simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseArray)();
  if (!simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_message_type_support_handle.typesupport_identifier) {
    simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &simulation_msgs__msg__TrialStart__rosidl_typesupport_introspection_c__TrialStart_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
