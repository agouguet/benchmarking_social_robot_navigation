// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from agents_msgs:msg/Agent.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT__TRAITS_HPP_
#define AGENTS_MSGS__MSG__DETAIL__AGENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "agents_msgs/msg/detail/agent__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace agents_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Agent & msg,
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

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: visible_by_robot
  {
    out << "visible_by_robot: ";
    rosidl_generator_traits::value_to_yaml(msg.visible_by_robot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Agent & msg,
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

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }

  // member: visible_by_robot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visible_by_robot: ";
    rosidl_generator_traits::value_to_yaml(msg.visible_by_robot, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Agent & msg, bool use_flow_style = false)
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
  const agents_msgs::msg::Agent & msg,
  std::ostream & out, size_t indentation = 0)
{
  agents_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use agents_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const agents_msgs::msg::Agent & msg)
{
  return agents_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<agents_msgs::msg::Agent>()
{
  return "agents_msgs::msg::Agent";
}

template<>
inline const char * name<agents_msgs::msg::Agent>()
{
  return "agents_msgs/msg/Agent";
}

template<>
struct has_fixed_size<agents_msgs::msg::Agent>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<agents_msgs::msg::Agent>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<agents_msgs::msg::Agent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT__TRAITS_HPP_
