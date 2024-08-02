// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from agents_msgs:msg/AgentTrajectories.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORIES__TRAITS_HPP_
#define AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORIES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "agents_msgs/msg/detail/agent_trajectories__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'trajectories'
#include "agents_msgs/msg/detail/agent_trajectory__traits.hpp"

namespace agents_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AgentTrajectories & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: trajectories
  {
    if (msg.trajectories.size() == 0) {
      out << "trajectories: []";
    } else {
      out << "trajectories: [";
      size_t pending_items = msg.trajectories.size();
      for (auto item : msg.trajectories) {
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
  const AgentTrajectories & msg,
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

  // member: trajectories
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.trajectories.size() == 0) {
      out << "trajectories: []\n";
    } else {
      out << "trajectories:\n";
      for (auto item : msg.trajectories) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AgentTrajectories & msg, bool use_flow_style = false)
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
  const agents_msgs::msg::AgentTrajectories & msg,
  std::ostream & out, size_t indentation = 0)
{
  agents_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use agents_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const agents_msgs::msg::AgentTrajectories & msg)
{
  return agents_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<agents_msgs::msg::AgentTrajectories>()
{
  return "agents_msgs::msg::AgentTrajectories";
}

template<>
inline const char * name<agents_msgs::msg::AgentTrajectories>()
{
  return "agents_msgs/msg/AgentTrajectories";
}

template<>
struct has_fixed_size<agents_msgs::msg::AgentTrajectories>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<agents_msgs::msg::AgentTrajectories>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<agents_msgs::msg::AgentTrajectories>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORIES__TRAITS_HPP_
