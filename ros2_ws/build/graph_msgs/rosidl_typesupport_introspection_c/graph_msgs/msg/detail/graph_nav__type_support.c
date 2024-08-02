// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from graph_msgs:msg/GraphNav.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "graph_msgs/msg/detail/graph_nav__rosidl_typesupport_introspection_c.h"
#include "graph_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "graph_msgs/msg/detail/graph_nav__functions.h"
#include "graph_msgs/msg/detail/graph_nav__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `edges`
#include "graph_msgs/msg/graph_edge.h"
// Member `edges`
#include "graph_msgs/msg/detail/graph_edge__rosidl_typesupport_introspection_c.h"
// Member `nodes`
#include "graph_msgs/msg/graph_node.h"
// Member `nodes`
#include "graph_msgs/msg/detail/graph_node__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  graph_msgs__msg__GraphNav__init(message_memory);
}

void graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_fini_function(void * message_memory)
{
  graph_msgs__msg__GraphNav__fini(message_memory);
}

size_t graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__size_function__GraphNav__edges(
  const void * untyped_member)
{
  const graph_msgs__msg__GraphEdge__Sequence * member =
    (const graph_msgs__msg__GraphEdge__Sequence *)(untyped_member);
  return member->size;
}

const void * graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__get_const_function__GraphNav__edges(
  const void * untyped_member, size_t index)
{
  const graph_msgs__msg__GraphEdge__Sequence * member =
    (const graph_msgs__msg__GraphEdge__Sequence *)(untyped_member);
  return &member->data[index];
}

void * graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__get_function__GraphNav__edges(
  void * untyped_member, size_t index)
{
  graph_msgs__msg__GraphEdge__Sequence * member =
    (graph_msgs__msg__GraphEdge__Sequence *)(untyped_member);
  return &member->data[index];
}

void graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__fetch_function__GraphNav__edges(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const graph_msgs__msg__GraphEdge * item =
    ((const graph_msgs__msg__GraphEdge *)
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__get_const_function__GraphNav__edges(untyped_member, index));
  graph_msgs__msg__GraphEdge * value =
    (graph_msgs__msg__GraphEdge *)(untyped_value);
  *value = *item;
}

void graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__assign_function__GraphNav__edges(
  void * untyped_member, size_t index, const void * untyped_value)
{
  graph_msgs__msg__GraphEdge * item =
    ((graph_msgs__msg__GraphEdge *)
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__get_function__GraphNav__edges(untyped_member, index));
  const graph_msgs__msg__GraphEdge * value =
    (const graph_msgs__msg__GraphEdge *)(untyped_value);
  *item = *value;
}

bool graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__resize_function__GraphNav__edges(
  void * untyped_member, size_t size)
{
  graph_msgs__msg__GraphEdge__Sequence * member =
    (graph_msgs__msg__GraphEdge__Sequence *)(untyped_member);
  graph_msgs__msg__GraphEdge__Sequence__fini(member);
  return graph_msgs__msg__GraphEdge__Sequence__init(member, size);
}

size_t graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__size_function__GraphNav__nodes(
  const void * untyped_member)
{
  const graph_msgs__msg__GraphNode__Sequence * member =
    (const graph_msgs__msg__GraphNode__Sequence *)(untyped_member);
  return member->size;
}

const void * graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__get_const_function__GraphNav__nodes(
  const void * untyped_member, size_t index)
{
  const graph_msgs__msg__GraphNode__Sequence * member =
    (const graph_msgs__msg__GraphNode__Sequence *)(untyped_member);
  return &member->data[index];
}

void * graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__get_function__GraphNav__nodes(
  void * untyped_member, size_t index)
{
  graph_msgs__msg__GraphNode__Sequence * member =
    (graph_msgs__msg__GraphNode__Sequence *)(untyped_member);
  return &member->data[index];
}

void graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__fetch_function__GraphNav__nodes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const graph_msgs__msg__GraphNode * item =
    ((const graph_msgs__msg__GraphNode *)
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__get_const_function__GraphNav__nodes(untyped_member, index));
  graph_msgs__msg__GraphNode * value =
    (graph_msgs__msg__GraphNode *)(untyped_value);
  *value = *item;
}

void graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__assign_function__GraphNav__nodes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  graph_msgs__msg__GraphNode * item =
    ((graph_msgs__msg__GraphNode *)
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__get_function__GraphNav__nodes(untyped_member, index));
  const graph_msgs__msg__GraphNode * value =
    (const graph_msgs__msg__GraphNode *)(untyped_value);
  *item = *value;
}

bool graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__resize_function__GraphNav__nodes(
  void * untyped_member, size_t size)
{
  graph_msgs__msg__GraphNode__Sequence * member =
    (graph_msgs__msg__GraphNode__Sequence *)(untyped_member);
  graph_msgs__msg__GraphNode__Sequence__fini(member);
  return graph_msgs__msg__GraphNode__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_msgs__msg__GraphNav, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "edges",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_msgs__msg__GraphNav, edges),  // bytes offset in struct
    NULL,  // default value
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__size_function__GraphNav__edges,  // size() function pointer
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__get_const_function__GraphNav__edges,  // get_const(index) function pointer
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__get_function__GraphNav__edges,  // get(index) function pointer
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__fetch_function__GraphNav__edges,  // fetch(index, &value) function pointer
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__assign_function__GraphNav__edges,  // assign(index, value) function pointer
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__resize_function__GraphNav__edges  // resize(index) function pointer
  },
  {
    "nodes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_msgs__msg__GraphNav, nodes),  // bytes offset in struct
    NULL,  // default value
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__size_function__GraphNav__nodes,  // size() function pointer
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__get_const_function__GraphNav__nodes,  // get_const(index) function pointer
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__get_function__GraphNav__nodes,  // get(index) function pointer
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__fetch_function__GraphNav__nodes,  // fetch(index, &value) function pointer
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__assign_function__GraphNav__nodes,  // assign(index, value) function pointer
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__resize_function__GraphNav__nodes  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_message_members = {
  "graph_msgs__msg",  // message namespace
  "GraphNav",  // message name
  3,  // number of fields
  sizeof(graph_msgs__msg__GraphNav),
  graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_message_member_array,  // message members
  graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_init_function,  // function to initialize message memory (memory has to be allocated)
  graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_message_type_support_handle = {
  0,
  &graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_graph_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, graph_msgs, msg, GraphNav)() {
  graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, graph_msgs, msg, GraphEdge)();
  graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, graph_msgs, msg, GraphNode)();
  if (!graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_message_type_support_handle.typesupport_identifier) {
    graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &graph_msgs__msg__GraphNav__rosidl_typesupport_introspection_c__GraphNav_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
