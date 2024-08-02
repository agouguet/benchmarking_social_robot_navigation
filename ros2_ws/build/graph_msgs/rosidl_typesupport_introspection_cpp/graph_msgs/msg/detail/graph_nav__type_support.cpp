// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from graph_msgs:msg/GraphNav.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "graph_msgs/msg/detail/graph_nav__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace graph_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void GraphNav_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) graph_msgs::msg::GraphNav(_init);
}

void GraphNav_fini_function(void * message_memory)
{
  auto typed_message = static_cast<graph_msgs::msg::GraphNav *>(message_memory);
  typed_message->~GraphNav();
}

size_t size_function__GraphNav__edges(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<graph_msgs::msg::GraphEdge> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GraphNav__edges(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<graph_msgs::msg::GraphEdge> *>(untyped_member);
  return &member[index];
}

void * get_function__GraphNav__edges(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<graph_msgs::msg::GraphEdge> *>(untyped_member);
  return &member[index];
}

void fetch_function__GraphNav__edges(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const graph_msgs::msg::GraphEdge *>(
    get_const_function__GraphNav__edges(untyped_member, index));
  auto & value = *reinterpret_cast<graph_msgs::msg::GraphEdge *>(untyped_value);
  value = item;
}

void assign_function__GraphNav__edges(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<graph_msgs::msg::GraphEdge *>(
    get_function__GraphNav__edges(untyped_member, index));
  const auto & value = *reinterpret_cast<const graph_msgs::msg::GraphEdge *>(untyped_value);
  item = value;
}

void resize_function__GraphNav__edges(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<graph_msgs::msg::GraphEdge> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GraphNav__nodes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<graph_msgs::msg::GraphNode> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GraphNav__nodes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<graph_msgs::msg::GraphNode> *>(untyped_member);
  return &member[index];
}

void * get_function__GraphNav__nodes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<graph_msgs::msg::GraphNode> *>(untyped_member);
  return &member[index];
}

void fetch_function__GraphNav__nodes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const graph_msgs::msg::GraphNode *>(
    get_const_function__GraphNav__nodes(untyped_member, index));
  auto & value = *reinterpret_cast<graph_msgs::msg::GraphNode *>(untyped_value);
  value = item;
}

void assign_function__GraphNav__nodes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<graph_msgs::msg::GraphNode *>(
    get_function__GraphNav__nodes(untyped_member, index));
  const auto & value = *reinterpret_cast<const graph_msgs::msg::GraphNode *>(untyped_value);
  item = value;
}

void resize_function__GraphNav__nodes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<graph_msgs::msg::GraphNode> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GraphNav_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_msgs::msg::GraphNav, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "edges",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<graph_msgs::msg::GraphEdge>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_msgs::msg::GraphNav, edges),  // bytes offset in struct
    nullptr,  // default value
    size_function__GraphNav__edges,  // size() function pointer
    get_const_function__GraphNav__edges,  // get_const(index) function pointer
    get_function__GraphNav__edges,  // get(index) function pointer
    fetch_function__GraphNav__edges,  // fetch(index, &value) function pointer
    assign_function__GraphNav__edges,  // assign(index, value) function pointer
    resize_function__GraphNav__edges  // resize(index) function pointer
  },
  {
    "nodes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<graph_msgs::msg::GraphNode>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(graph_msgs::msg::GraphNav, nodes),  // bytes offset in struct
    nullptr,  // default value
    size_function__GraphNav__nodes,  // size() function pointer
    get_const_function__GraphNav__nodes,  // get_const(index) function pointer
    get_function__GraphNav__nodes,  // get(index) function pointer
    fetch_function__GraphNav__nodes,  // fetch(index, &value) function pointer
    assign_function__GraphNav__nodes,  // assign(index, value) function pointer
    resize_function__GraphNav__nodes  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GraphNav_message_members = {
  "graph_msgs::msg",  // message namespace
  "GraphNav",  // message name
  3,  // number of fields
  sizeof(graph_msgs::msg::GraphNav),
  GraphNav_message_member_array,  // message members
  GraphNav_init_function,  // function to initialize message memory (memory has to be allocated)
  GraphNav_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GraphNav_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GraphNav_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace graph_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<graph_msgs::msg::GraphNav>()
{
  return &::graph_msgs::msg::rosidl_typesupport_introspection_cpp::GraphNav_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, graph_msgs, msg, GraphNav)() {
  return &::graph_msgs::msg::rosidl_typesupport_introspection_cpp::GraphNav_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
