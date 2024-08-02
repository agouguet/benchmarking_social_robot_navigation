// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from graph_msgs:msg/GraphNav.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSGS__MSG__DETAIL__GRAPH_NAV__TRAITS_HPP_
#define GRAPH_MSGS__MSG__DETAIL__GRAPH_NAV__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "graph_msgs/msg/detail/graph_nav__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'edges'
#include "graph_msgs/msg/detail/graph_edge__traits.hpp"
// Member 'nodes'
#include "graph_msgs/msg/detail/graph_node__traits.hpp"

namespace graph_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GraphNav & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: edges
  {
    if (msg.edges.size() == 0) {
      out << "edges: []";
    } else {
      out << "edges: [";
      size_t pending_items = msg.edges.size();
      for (auto item : msg.edges) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: nodes
  {
    if (msg.nodes.size() == 0) {
      out << "nodes: []";
    } else {
      out << "nodes: [";
      size_t pending_items = msg.nodes.size();
      for (auto item : msg.nodes) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GraphNav & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: edges
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.edges.size() == 0) {
      out << "edges: []\n";
    } else {
      out << "edges:\n";
      for (auto item : msg.edges) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: nodes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.nodes.size() == 0) {
      out << "nodes: []\n";
    } else {
      out << "nodes:\n";
      for (auto item : msg.nodes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GraphNav & msg, bool use_flow_style = false)
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
  const graph_msgs::msg::GraphNav & msg,
  std::ostream & out, size_t indentation = 0)
{
  graph_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use graph_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const graph_msgs::msg::GraphNav & msg)
{
  return graph_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<graph_msgs::msg::GraphNav>()
{
  return "graph_msgs::msg::GraphNav";
}

template<>
inline const char * name<graph_msgs::msg::GraphNav>()
{
  return "graph_msgs/msg/GraphNav";
}

template<>
struct has_fixed_size<graph_msgs::msg::GraphNav>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<graph_msgs::msg::GraphNav>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<graph_msgs::msg::GraphNav>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // GRAPH_MSGS__MSG__DETAIL__GRAPH_NAV__TRAITS_HPP_
