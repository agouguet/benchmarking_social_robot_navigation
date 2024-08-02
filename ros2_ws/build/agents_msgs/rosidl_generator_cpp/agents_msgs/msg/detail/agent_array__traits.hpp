// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from agents_msgs:msg/AgentArray.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT_ARRAY__TRAITS_HPP_
#define AGENTS_MSGS__MSG__DETAIL__AGENT_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "agents_msgs/msg/detail/agent_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'agents'
#include "agents_msgs/msg/detail/agent__traits.hpp"

namespace agents_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AgentArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: agents
  {
    if (msg.agents.size() == 0) {
      out << "agents: []";
    } else {
      out << "agents: [";
      size_t pending_items = msg.agents.size();
      for (auto item : msg.agents) {
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
  const AgentArray & msg,
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

  // member: agents
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.agents.size() == 0) {
      out << "agents: []\n";
    } else {
      out << "agents:\n";
      for (auto item : msg.agents) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AgentArray & msg, bool use_flow_style = false)
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
  const agents_msgs::msg::AgentArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  agents_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use agents_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const agents_msgs::msg::AgentArray & msg)
{
  return agents_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<agents_msgs::msg::AgentArray>()
{
  return "agents_msgs::msg::AgentArray";
}

template<>
inline const char * name<agents_msgs::msg::AgentArray>()
{
  return "agents_msgs/msg/AgentArray";
}

template<>
struct has_fixed_size<agents_msgs::msg::AgentArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<agents_msgs::msg::AgentArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<agents_msgs::msg::AgentArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT_ARRAY__TRAITS_HPP_
