// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from agents_msgs:msg/AgentTrajectories.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "agents_msgs/msg/detail/agent_trajectories__rosidl_typesupport_introspection_c.h"
#include "agents_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "agents_msgs/msg/detail/agent_trajectories__functions.h"
#include "agents_msgs/msg/detail/agent_trajectories__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `trajectories`
#include "agents_msgs/msg/agent_trajectory.h"
// Member `trajectories`
#include "agents_msgs/msg/detail/agent_trajectory__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  agents_msgs__msg__AgentTrajectories__init(message_memory);
}

void agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_fini_function(void * message_memory)
{
  agents_msgs__msg__AgentTrajectories__fini(message_memory);
}

size_t agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__size_function__AgentTrajectories__trajectories(
  const void * untyped_member)
{
  const agents_msgs__msg__AgentTrajectory__Sequence * member =
    (const agents_msgs__msg__AgentTrajectory__Sequence *)(untyped_member);
  return member->size;
}

const void * agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__get_const_function__AgentTrajectories__trajectories(
  const void * untyped_member, size_t index)
{
  const agents_msgs__msg__AgentTrajectory__Sequence * member =
    (const agents_msgs__msg__AgentTrajectory__Sequence *)(untyped_member);
  return &member->data[index];
}

void * agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__get_function__AgentTrajectories__trajectories(
  void * untyped_member, size_t index)
{
  agents_msgs__msg__AgentTrajectory__Sequence * member =
    (agents_msgs__msg__AgentTrajectory__Sequence *)(untyped_member);
  return &member->data[index];
}

void agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__fetch_function__AgentTrajectories__trajectories(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const agents_msgs__msg__AgentTrajectory * item =
    ((const agents_msgs__msg__AgentTrajectory *)
    agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__get_const_function__AgentTrajectories__trajectories(untyped_member, index));
  agents_msgs__msg__AgentTrajectory * value =
    (agents_msgs__msg__AgentTrajectory *)(untyped_value);
  *value = *item;
}

void agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__assign_function__AgentTrajectories__trajectories(
  void * untyped_member, size_t index, const void * untyped_value)
{
  agents_msgs__msg__AgentTrajectory * item =
    ((agents_msgs__msg__AgentTrajectory *)
    agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__get_function__AgentTrajectories__trajectories(untyped_member, index));
  const agents_msgs__msg__AgentTrajectory * value =
    (const agents_msgs__msg__AgentTrajectory *)(untyped_value);
  *item = *value;
}

bool agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__resize_function__AgentTrajectories__trajectories(
  void * untyped_member, size_t size)
{
  agents_msgs__msg__AgentTrajectory__Sequence * member =
    (agents_msgs__msg__AgentTrajectory__Sequence *)(untyped_member);
  agents_msgs__msg__AgentTrajectory__Sequence__fini(member);
  return agents_msgs__msg__AgentTrajectory__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(agents_msgs__msg__AgentTrajectories, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "trajectories",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(agents_msgs__msg__AgentTrajectories, trajectories),  // bytes offset in struct
    NULL,  // default value
    agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__size_function__AgentTrajectories__trajectories,  // size() function pointer
    agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__get_const_function__AgentTrajectories__trajectories,  // get_const(index) function pointer
    agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__get_function__AgentTrajectories__trajectories,  // get(index) function pointer
    agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__fetch_function__AgentTrajectories__trajectories,  // fetch(index, &value) function pointer
    agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__assign_function__AgentTrajectories__trajectories,  // assign(index, value) function pointer
    agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__resize_function__AgentTrajectories__trajectories  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_message_members = {
  "agents_msgs__msg",  // message namespace
  "AgentTrajectories",  // message name
  2,  // number of fields
  sizeof(agents_msgs__msg__AgentTrajectories),
  agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_message_member_array,  // message members
  agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_init_function,  // function to initialize message memory (memory has to be allocated)
  agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_message_type_support_handle = {
  0,
  &agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_agents_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agents_msgs, msg, AgentTrajectories)() {
  agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, agents_msgs, msg, AgentTrajectory)();
  if (!agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_message_type_support_handle.typesupport_identifier) {
    agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &agents_msgs__msg__AgentTrajectories__rosidl_typesupport_introspection_c__AgentTrajectories_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
