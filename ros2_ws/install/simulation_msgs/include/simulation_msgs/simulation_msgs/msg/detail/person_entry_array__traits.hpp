// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from simulation_msgs:msg/PersonEntryArray.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY_ARRAY__TRAITS_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "simulation_msgs/msg/detail/person_entry_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'people'
#include "simulation_msgs/msg/detail/person_entry__traits.hpp"

namespace simulation_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PersonEntryArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: people
  {
    if (msg.people.size() == 0) {
      out << "people: []";
    } else {
      out << "people: [";
      size_t pending_items = msg.people.size();
      for (auto item : msg.people) {
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
  const PersonEntryArray & msg,
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

  // member: people
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.people.size() == 0) {
      out << "people: []\n";
    } else {
      out << "people:\n";
      for (auto item : msg.people) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PersonEntryArray & msg, bool use_flow_style = false)
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
  const simulation_msgs::msg::PersonEntryArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  simulation_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use simulation_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const simulation_msgs::msg::PersonEntryArray & msg)
{
  return simulation_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<simulation_msgs::msg::PersonEntryArray>()
{
  return "simulation_msgs::msg::PersonEntryArray";
}

template<>
inline const char * name<simulation_msgs::msg::PersonEntryArray>()
{
  return "simulation_msgs/msg/PersonEntryArray";
}

template<>
struct has_fixed_size<simulation_msgs::msg::PersonEntryArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<simulation_msgs::msg::PersonEntryArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<simulation_msgs::msg::PersonEntryArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY_ARRAY__TRAITS_HPP_
