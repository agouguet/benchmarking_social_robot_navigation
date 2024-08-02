// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from simulation_msgs:msg/PersonEntryArray.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY_ARRAY__BUILDER_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "simulation_msgs/msg/detail/person_entry_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace simulation_msgs
{

namespace msg
{

namespace builder
{

class Init_PersonEntryArray_people
{
public:
  explicit Init_PersonEntryArray_people(::simulation_msgs::msg::PersonEntryArray & msg)
  : msg_(msg)
  {}
  ::simulation_msgs::msg::PersonEntryArray people(::simulation_msgs::msg::PersonEntryArray::_people_type arg)
  {
    msg_.people = std::move(arg);
    return std::move(msg_);
  }

private:
  ::simulation_msgs::msg::PersonEntryArray msg_;
};

class Init_PersonEntryArray_header
{
public:
  Init_PersonEntryArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PersonEntryArray_people header(::simulation_msgs::msg::PersonEntryArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PersonEntryArray_people(msg_);
  }

private:
  ::simulation_msgs::msg::PersonEntryArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::simulation_msgs::msg::PersonEntryArray>()
{
  return simulation_msgs::msg::builder::Init_PersonEntryArray_header();
}

}  // namespace simulation_msgs

#endif  // SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY_ARRAY__BUILDER_HPP_
