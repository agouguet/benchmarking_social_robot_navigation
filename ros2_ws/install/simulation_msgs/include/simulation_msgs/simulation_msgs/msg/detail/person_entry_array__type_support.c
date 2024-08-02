// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from simulation_msgs:msg/PersonEntryArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "simulation_msgs/msg/detail/person_entry_array__rosidl_typesupport_introspection_c.h"
#include "simulation_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "simulation_msgs/msg/detail/person_entry_array__functions.h"
#include "simulation_msgs/msg/detail/person_entry_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `people`
#include "simulation_msgs/msg/person_entry.h"
// Member `people`
#include "simulation_msgs/msg/detail/person_entry__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  simulation_msgs__msg__PersonEntryArray__init(message_memory);
}

void simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_fini_function(void * message_memory)
{
  simulation_msgs__msg__PersonEntryArray__fini(message_memory);
}

size_t simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__size_function__PersonEntryArray__people(
  const void * untyped_member)
{
  const simulation_msgs__msg__PersonEntry__Sequence * member =
    (const simulation_msgs__msg__PersonEntry__Sequence *)(untyped_member);
  return member->size;
}

const void * simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__get_const_function__PersonEntryArray__people(
  const void * untyped_member, size_t index)
{
  const simulation_msgs__msg__PersonEntry__Sequence * member =
    (const simulation_msgs__msg__PersonEntry__Sequence *)(untyped_member);
  return &member->data[index];
}

void * simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__get_function__PersonEntryArray__people(
  void * untyped_member, size_t index)
{
  simulation_msgs__msg__PersonEntry__Sequence * member =
    (simulation_msgs__msg__PersonEntry__Sequence *)(untyped_member);
  return &member->data[index];
}

void simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__fetch_function__PersonEntryArray__people(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const simulation_msgs__msg__PersonEntry * item =
    ((const simulation_msgs__msg__PersonEntry *)
    simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__get_const_function__PersonEntryArray__people(untyped_member, index));
  simulation_msgs__msg__PersonEntry * value =
    (simulation_msgs__msg__PersonEntry *)(untyped_value);
  *value = *item;
}

void simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__assign_function__PersonEntryArray__people(
  void * untyped_member, size_t index, const void * untyped_value)
{
  simulation_msgs__msg__PersonEntry * item =
    ((simulation_msgs__msg__PersonEntry *)
    simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__get_function__PersonEntryArray__people(untyped_member, index));
  const simulation_msgs__msg__PersonEntry * value =
    (const simulation_msgs__msg__PersonEntry *)(untyped_value);
  *item = *value;
}

bool simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__resize_function__PersonEntryArray__people(
  void * untyped_member, size_t size)
{
  simulation_msgs__msg__PersonEntry__Sequence * member =
    (simulation_msgs__msg__PersonEntry__Sequence *)(untyped_member);
  simulation_msgs__msg__PersonEntry__Sequence__fini(member);
  return simulation_msgs__msg__PersonEntry__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__PersonEntryArray, header),  // bytes offset in struct
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
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulation_msgs__msg__PersonEntryArray, people),  // bytes offset in struct
    NULL,  // default value
    simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__size_function__PersonEntryArray__people,  // size() function pointer
    simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__get_const_function__PersonEntryArray__people,  // get_const(index) function pointer
    simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__get_function__PersonEntryArray__people,  // get(index) function pointer
    simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__fetch_function__PersonEntryArray__people,  // fetch(index, &value) function pointer
    simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__assign_function__PersonEntryArray__people,  // assign(index, value) function pointer
    simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__resize_function__PersonEntryArray__people  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_message_members = {
  "simulation_msgs__msg",  // message namespace
  "PersonEntryArray",  // message name
  2,  // number of fields
  sizeof(simulation_msgs__msg__PersonEntryArray),
  simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_message_member_array,  // message members
  simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_init_function,  // function to initialize message memory (memory has to be allocated)
  simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_message_type_support_handle = {
  0,
  &simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_simulation_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, simulation_msgs, msg, PersonEntryArray)() {
  simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, simulation_msgs, msg, PersonEntry)();
  if (!simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_message_type_support_handle.typesupport_identifier) {
    simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &simulation_msgs__msg__PersonEntryArray__rosidl_typesupport_introspection_c__PersonEntryArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
