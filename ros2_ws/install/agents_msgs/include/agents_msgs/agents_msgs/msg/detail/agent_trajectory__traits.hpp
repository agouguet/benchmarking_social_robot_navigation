// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from agents_msgs:msg/AgentTrajectory.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORY__TRAITS_HPP_
#define AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "agents_msgs/msg/detail/agent_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'poses'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace agents_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AgentTrajectory & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: poses
  {
    if (msg.poses.size() == 0) {
      out << "poses: []";
    } else {
      out << "poses: [";
      size_t pending_items = msg.poses.size();
      for (auto item : msg.poses) {
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
  const AgentTrajectory & msg,
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

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poses.size() == 0) {
      out << "poses: []\n";
    } else {
      out << "poses:\n";
      for (auto item : msg.poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AgentTrajectory & msg, bool use_flow_style = false)
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

}  // namespace agents_msgs

namespace rosidl_generator_traits
{

[[deprecated("use agents_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const agents_msgs::msg::AgentTrajectory & msg,
  std::ostream & out, size_t indentation = 0)
{
  agents_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use agents_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const agents_msgs::msg::AgentTrajectory & msg)
{
  return agents_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<agents_msgs::msg::AgentTrajectory>()
{
  return "agents_msgs::msg::AgentTrajectory";
}

template<>
inline const char * name<agents_msgs::msg::AgentTrajectory>()
{
  return "agents_msgs/msg/AgentTrajectory";
}

template<>
struct has_fixed_size<agents_msgs::msg::AgentTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<agents_msgs::msg::AgentTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<agents_msgs::msg::AgentTrajectory>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORY__TRAITS_HPP_
