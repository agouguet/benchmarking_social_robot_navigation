// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from graph_msgs:msg/GraphEdge.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__TRAITS_HPP_
#define GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "graph_msgs/msg/detail/graph_edge__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace graph_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GraphEdge & msg,
  std::ostream & out)
{
  out << "{";
  // member: id_n1
  {
    out << "id_n1: ";
    rosidl_generator_traits::value_to_yaml(msg.id_n1, out);
    out << ", ";
  }

  // member: id_n2
  {
    out << "id_n2: ";
    rosidl_generator_traits::value_to_yaml(msg.id_n2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GraphEdge & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id_n1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id_n1: ";
    rosidl_generator_traits::value_to_yaml(msg.id_n1, out);
    out << "\n";
  }

  // member: id_n2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id_n2: ";
    rosidl_generator_traits::value_to_yaml(msg.id_n2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GraphEdge & msg, bool use_flow_style = false)
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
  const graph_msgs::msg::GraphEdge & msg,
  std::ostream & out, size_t indentation = 0)
{
  graph_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use graph_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const graph_msgs::msg::GraphEdge & msg)
{
  return graph_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<graph_msgs::msg::GraphEdge>()
{
  return "graph_msgs::msg::GraphEdge";
}

template<>
inline const char * name<graph_msgs::msg::GraphEdge>()
{
  return "graph_msgs/msg/GraphEdge";
}

template<>
struct has_fixed_size<graph_msgs::msg::GraphEdge>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<graph_msgs::msg::GraphEdge>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<graph_msgs::msg::GraphEdge>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__TRAITS_HPP_
