// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from agents_msgs:msg/Agent.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT__BUILDER_HPP_
#define AGENTS_MSGS__MSG__DETAIL__AGENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "agents_msgs/msg/detail/agent__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace agents_msgs
{

namespace msg
{

namespace builder
{

class Init_Agent_visible_by_robot
{
public:
  explicit Init_Agent_visible_by_robot(::agents_msgs::msg::Agent & msg)
  : msg_(msg)
  {}
  ::agents_msgs::msg::Agent visible_by_robot(::agents_msgs::msg::Agent::_visible_by_robot_type arg)
  {
    msg_.visible_by_robot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::agents_msgs::msg::Agent msg_;
};

class Init_Agent_velocity
{
public:
  explicit Init_Agent_velocity(::agents_msgs::msg::Agent & msg)
  : msg_(msg)
  {}
  Init_Agent_visible_by_robot velocity(::agents_msgs::msg::Agent::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Agent_visible_by_robot(msg_);
  }

private:
  ::agents_msgs::msg::Agent msg_;
};

class Init_Agent_pose
{
public:
  explicit Init_Agent_pose(::agents_msgs::msg::Agent & msg)
  : msg_(msg)
  {}
  Init_Agent_velocity pose(::agents_msgs::msg::Agent::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_Agent_velocity(msg_);
  }

private:
  ::agents_msgs::msg::Agent msg_;
};

class Init_Agent_id
{
public:
  explicit Init_Agent_id(::agents_msgs::msg::Agent & msg)
  : msg_(msg)
  {}
  Init_Agent_pose id(::agents_msgs::msg::Agent::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Agent_pose(msg_);
  }

private:
  ::agents_msgs::msg::Agent msg_;
};

class Init_Agent_header
{
public:
  Init_Agent_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Agent_id header(::agents_msgs::msg::Agent::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Agent_id(msg_);
  }

private:
  ::agents_msgs::msg::Agent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::agents_msgs::msg::Agent>()
{
  return agents_msgs::msg::builder::Init_Agent_header();
}

}  // namespace agents_msgs

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT__BUILDER_HPP_
