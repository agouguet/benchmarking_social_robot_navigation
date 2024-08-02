// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from agents_msgs:msg/AgentTrajectory.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "agents_msgs/msg/detail/agent_trajectory__rosidl_typesupport_introspection_c.h"
#include "agents_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "agents_msgs/msg/detail/agent_trajectory__functions.h"
#include "agents_msgs/msg/detail/agent_trajectory__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `poses`
#include "geometry_msgs/msg/point.h"
// Member `poses`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  agents_msgs__msg__AgentTrajectory__init(message_memory);
}

void agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_fini_function(void * message_memory)
{
  agents_msgs__msg__AgentTrajectory__fini(message_memory);
}

size_t agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__size_function__AgentTrajectory__poses(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__get_const_function__AgentTrajectory__poses(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__get_function__AgentTrajectory__poses(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__fetch_function__AgentTrajectory__poses(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__get_const_function__AgentTrajectory__poses(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__assign_function__AgentTrajectory__poses(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__get_function__AgentTrajectory__poses(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__resize_function__AgentTrajectory__poses(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(agents_msgs__msg__AgentTrajectory, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(agents_msgs__msg__AgentTrajectory, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "poses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(agents_msgs__msg__AgentTrajectory, poses),  // bytes offset in struct
    NULL,  // default value
    agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__size_function__AgentTrajectory__poses,  // size() function pointer
    agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__get_const_function__AgentTrajectory__poses,  // get_const(index) function pointer
    agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__get_function__AgentTrajectory__poses,  // get(index) function pointer
    agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__fetch_function__AgentTrajectory__poses,  // fetch(index, &value) function pointer
    agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__assign_function__AgentTrajectory__poses,  // assign(index, value) function pointer
    agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__resize_function__AgentTrajectory__poses  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_message_members = {
  "agents_msgs__msg",  // message namespace
  "AgentTrajectory",  // message name
  3,  // number of fields
  sizeof(agents_msgs__msg__AgentTrajectory),
  agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_message_member_array,  // message members
  agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_init_function,  // function to initialize message memory (memory has to be allocated)
  agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_message_type_support_handle = {
  0,
  &agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_agents_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agents_msgs, msg, AgentTrajectory)() {
  agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_message_type_support_handle.typesupport_identifier) {
    agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &agents_msgs__msg__AgentTrajectory__rosidl_typesupport_introspection_c__AgentTrajectory_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
