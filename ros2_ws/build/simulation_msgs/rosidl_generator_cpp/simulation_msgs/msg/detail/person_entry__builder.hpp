// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from simulation_msgs:msg/PersonEntry.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY__BUILDER_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "simulation_msgs/msg/detail/person_entry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace simulation_msgs
{

namespace msg
{

namespace builder
{

class Init_PersonEntry_twist
{
public:
  explicit Init_PersonEntry_twist(::simulation_msgs::msg::PersonEntry & msg)
  : msg_(msg)
  {}
  ::simulation_msgs::msg::PersonEntry twist(::simulation_msgs::msg::PersonEntry::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::simulation_msgs::msg::PersonEntry msg_;
};

class Init_PersonEntry_pose
{
public:
  explicit Init_PersonEntry_pose(::simulation_msgs::msg::PersonEntry & msg)
  : msg_(msg)
  {}
  Init_PersonEntry_twist pose(::simulation_msgs::msg::PersonEntry::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_PersonEntry_twist(msg_);
  }

private:
  ::simulation_msgs::msg::PersonEntry msg_;
};

class Init_PersonEntry_track_id
{
public:
  explicit Init_PersonEntry_track_id(::simulation_msgs::msg::PersonEntry & msg)
  : msg_(msg)
  {}
  Init_PersonEntry_pose track_id(::simulation_msgs::msg::PersonEntry::_track_id_type arg)
  {
    msg_.track_id = std::move(arg);
    return Init_PersonEntry_pose(msg_);
  }

private:
  ::simulation_msgs::msg::PersonEntry msg_;
};

class Init_PersonEntry_header
{
public:
  Init_PersonEntry_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PersonEntry_track_id header(::simulation_msgs::msg::PersonEntry::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PersonEntry_track_id(msg_);
  }

private:
  ::simulation_msgs::msg::PersonEntry msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::simulation_msgs::msg::PersonEntry>()
{
  return simulation_msgs::msg::builder::Init_PersonEntry_header();
}

}  // namespace simulation_msgs

#endif  // SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY__BUILDER_HPP_
