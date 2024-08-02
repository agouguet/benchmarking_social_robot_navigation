// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from simulation_msgs:msg/SceneInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "simulation_msgs/msg/detail/scene_info__rosidl_typesupport_introspection_c.h"
#include "simulation_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "simulation_msgs/msg/detail/scene_info__functions.h"
#include "simulation_msgs/msg/detail/scene_info__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `scenario_name`
// Member `environment`
#include "rosidl_runtime_c/string_functions.h"
// Member `robot_start_pose`
// Member `robot_target_pose`
#include "geometry_msgs/msg/pose.h"
// Member `robot_start_pose`
// Member `robot_target_pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  simulation_msgs__msg__SceneInfo__init(message_memory);
}

void simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_fini_function(void * message_memory)
{
  simulation_msgs__msg__SceneInfo__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__SceneInfo, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "scenario_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__SceneInfo, scenario_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_start_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__SceneInfo, robot_start_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_target_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__SceneInfo, robot_target_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_people",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__SceneInfo, num_people),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_groups",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__SceneInfo, num_groups),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "environment",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__SceneInfo, environment),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_message_members = {
  "simulation_msgs__msg",  // message namespace
  "SceneInfo",  // message name
  7,  // number of fields
  sizeof(simulation_msgs__msg__SceneInfo),
  simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_message_member_array,  // message members
  simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_message_type_support_handle = {
  0,
  &simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_simulation_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, simulation_msgs, msg, SceneInfo)() {
  simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_message_type_support_handle.typesupport_identifier) {
    simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &simulation_msgs__msg__SceneInfo__rosidl_typesupport_introspection_c__SceneInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
