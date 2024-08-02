// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from simulation_msgs:msg/PersonEntry.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY__TRAITS_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "simulation_msgs/msg/detail/person_entry__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__traits.hpp"

namespace simulation_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PersonEntry & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: track_id
  {
    out << "track_id: ";
    rosidl_generator_traits::value_to_yaml(msg.track_id, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: twist
  {
    out << "twist: ";
    to_flow_style_yaml(msg.twist, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PersonEntry & msg,
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

  // member: track_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "track_id: ";
    rosidl_generator_traits::value_to_yaml(msg.track_id, out);
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

  // member: twist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "twist:\n";
    to_block_style_yaml(msg.twist, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PersonEntry & msg, bool use_flow_style = false)
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

}  // namespace simulation_msgs

namespace rosidl_generator_traits
{

[[deprecated("use simulation_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const simulation_msgs::msg::PersonEntry & msg,
  std::ostream & out, size_t indentation = 0)
{
  simulation_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use simulation_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const simulation_msgs::msg::PersonEntry & msg)
{
  return simulation_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<simulation_msgs::msg::PersonEntry>()
{
  return "simulation_msgs::msg::PersonEntry";
}

template<>
inline const char * name<simulation_msgs::msg::PersonEntry>()
{
  return "simulation_msgs/msg/PersonEntry";
}

template<>
struct has_fixed_size<simulation_msgs::msg::PersonEntry>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<simulation_msgs::msg::PersonEntry>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<simulation_msgs::msg::PersonEntry>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY__TRAITS_HPP_
