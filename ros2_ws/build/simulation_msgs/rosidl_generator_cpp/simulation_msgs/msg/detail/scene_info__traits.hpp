// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from simulation_msgs:msg/SceneInfo.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__SCENE_INFO__TRAITS_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__SCENE_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "simulation_msgs/msg/detail/scene_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'robot_start_pose'
// Member 'robot_target_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace simulation_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SceneInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: scenario_name
  {
    out << "scenario_name: ";
    rosidl_generator_traits::value_to_yaml(msg.scenario_name, out);
    out << ", ";
  }

  // member: robot_start_pose
  {
    out << "robot_start_pose: ";
    to_flow_style_yaml(msg.robot_start_pose, out);
    out << ", ";
  }

  // member: robot_target_pose
  {
    out << "robot_target_pose: ";
    to_flow_style_yaml(msg.robot_target_pose, out);
    out << ", ";
  }

  // member: num_people
  {
    out << "num_people: ";
    rosidl_generator_traits::value_to_yaml(msg.num_people, out);
    out << ", ";
  }

  // member: num_groups
  {
    out << "num_groups: ";
    rosidl_generator_traits::value_to_yaml(msg.num_groups, out);
    out << ", ";
  }

  // member: environment
  {
    out << "environment: ";
    rosidl_generator_traits::value_to_yaml(msg.environment, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SceneInfo & msg,
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

  // member: scenario_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scenario_name: ";
    rosidl_generator_traits::value_to_yaml(msg.scenario_name, out);
    out << "\n";
  }

  // member: robot_start_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_start_pose:\n";
    to_block_style_yaml(msg.robot_start_pose, out, indentation + 2);
  }

  // member: robot_target_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_target_pose:\n";
    to_block_style_yaml(msg.robot_target_pose, out, indentation + 2);
  }

  // member: num_people
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_people: ";
    rosidl_generator_traits::value_to_yaml(msg.num_people, out);
    out << "\n";
  }

  // member: num_groups
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_groups: ";
    rosidl_generator_traits::value_to_yaml(msg.num_groups, out);
    out << "\n";
  }

  // member: environment
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "environment: ";
    rosidl_generator_traits::value_to_yaml(msg.environment, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SceneInfo & msg, bool use_flow_style = false)
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
  const simulation_msgs::msg::SceneInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  simulation_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use simulation_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const simulation_msgs::msg::SceneInfo & msg)
{
  return simulation_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<simulation_msgs::msg::SceneInfo>()
{
  return "simulation_msgs::msg::SceneInfo";
}

template<>
inline const char * name<simulation_msgs::msg::SceneInfo>()
{
  return "simulation_msgs/msg/SceneInfo";
}

template<>
struct has_fixed_size<simulation_msgs::msg::SceneInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<simulation_msgs::msg::SceneInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<simulation_msgs::msg::SceneInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SIMULATION_MSGS__MSG__DETAIL__SCENE_INFO__TRAITS_HPP_
