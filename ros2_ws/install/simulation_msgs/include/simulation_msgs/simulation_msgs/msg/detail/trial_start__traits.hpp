// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from simulation_msgs:msg/TrialStart.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__TRIAL_START__TRAITS_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__TRIAL_START__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "simulation_msgs/msg/detail/trial_start__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'spawn'
// Member 'target'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'people'
#include "geometry_msgs/msg/detail/pose_array__traits.hpp"

namespace simulation_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrialStart & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: trial_name
  {
    out << "trial_name: ";
    rosidl_generator_traits::value_to_yaml(msg.trial_name, out);
    out << ", ";
  }

  // member: trial_number
  {
    out << "trial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.trial_number, out);
    out << ", ";
  }

  // member: spawn
  {
    out << "spawn: ";
    to_flow_style_yaml(msg.spawn, out);
    out << ", ";
  }

  // member: target
  {
    out << "target: ";
    to_flow_style_yaml(msg.target, out);
    out << ", ";
  }

  // member: people
  {
    out << "people: ";
    to_flow_style_yaml(msg.people, out);
    out << ", ";
  }

  // member: time_limit
  {
    out << "time_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.time_limit, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrialStart & msg,
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

  // member: trial_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trial_name: ";
    rosidl_generator_traits::value_to_yaml(msg.trial_name, out);
    out << "\n";
  }

  // member: trial_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.trial_number, out);
    out << "\n";
  }

  // member: spawn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spawn:\n";
    to_block_style_yaml(msg.spawn, out, indentation + 2);
  }

  // member: target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target:\n";
    to_block_style_yaml(msg.target, out, indentation + 2);
  }

  // member: people
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "people:\n";
    to_block_style_yaml(msg.people, out, indentation + 2);
  }

  // member: time_limit
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_limit: ";
    rosidl_generator_traits::value_to_yaml(msg.time_limit, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrialStart & msg, bool use_flow_style = false)
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
  const simulation_msgs::msg::TrialStart & msg,
  std::ostream & out, size_t indentation = 0)
{
  simulation_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use simulation_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const simulation_msgs::msg::TrialStart & msg)
{
  return simulation_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<simulation_msgs::msg::TrialStart>()
{
  return "simulation_msgs::msg::TrialStart";
}

template<>
inline const char * name<simulation_msgs::msg::TrialStart>()
{
  return "simulation_msgs/msg/TrialStart";
}

template<>
struct has_fixed_size<simulation_msgs::msg::TrialStart>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<simulation_msgs::msg::TrialStart>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<simulation_msgs::msg::TrialStart>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SIMULATION_MSGS__MSG__DETAIL__TRIAL_START__TRAITS_HPP_
