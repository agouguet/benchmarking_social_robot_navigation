// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from graph_msgs:msg/GraphNode.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSGS__MSG__DETAIL__GRAPH_NODE__TRAITS_HPP_
#define GRAPH_MSGS__MSG__DETAIL__GRAPH_NODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "graph_msgs/msg/detail/graph_node__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace graph_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GraphNode & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: occupied
  {
    out << "occupied: ";
    rosidl_generator_traits::value_to_yaml(msg.occupied, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GraphNode & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: occupied
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "occupied: ";
    rosidl_generator_traits::value_to_yaml(msg.occupied, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GraphNode & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace graph_msgs

namespace rosidl_generator_traits
{

[[deprecated("use graph_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const graph_msgs::msg::GraphNode & msg,
  std::ostream & out, size_t indentation = 0)
{
  graph_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use graph_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const graph_msgs::msg::GraphNode & msg)
{
  return graph_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<graph_msgs::msg::GraphNode>()
{
  return "graph_msgs::msg::GraphNode";
}

template<>
inline const char * name<graph_msgs::msg::GraphNode>()
{
  return "graph_msgs/msg/GraphNode";
}

template<>
struct has_fixed_size<graph_msgs::msg::GraphNode>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<graph_msgs::msg::GraphNode>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<graph_msgs::msg::GraphNode>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GRAPH_MSGS__MSG__DETAIL__GRAPH_NODE__TRAITS_HPP_
