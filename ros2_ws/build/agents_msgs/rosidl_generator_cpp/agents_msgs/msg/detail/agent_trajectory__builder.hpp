// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from agents_msgs:msg/AgentTrajectory.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORY__BUILDER_HPP_
#define AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "agents_msgs/msg/detail/agent_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace agents_msgs
{

namespace msg
{

namespace builder
{

class Init_AgentTrajectory_poses
{
public:
  explicit Init_AgentTrajectory_poses(::agents_msgs::msg::AgentTrajectory & msg)
  : msg_(msg)
  {}
  ::agents_msgs::msg::AgentTrajectory poses(::agents_msgs::msg::AgentTrajectory::_poses_type arg)
  {
    msg_.poses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::agents_msgs::msg::AgentTrajectory msg_;
};

class Init_AgentTrajectory_id
{
public:
  explicit Init_AgentTrajectory_id(::agents_msgs::msg::AgentTrajectory & msg)
  : msg_(msg)
  {}
  Init_AgentTrajectory_poses id(::agents_msgs::msg::AgentTrajectory::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_AgentTrajectory_poses(msg_);
  }

private:
  ::agents_msgs::msg::AgentTrajectory msg_;
};

class Init_AgentTrajectory_header
{
public:
  Init_AgentTrajectory_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AgentTrajectory_id header(::agents_msgs::msg::AgentTrajectory::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AgentTrajectory_id(msg_);
  }

private:
  ::agents_msgs::msg::AgentTrajectory msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::agents_msgs::msg::AgentTrajectory>()
{
  return agents_msgs::msg::builder::Init_AgentTrajectory_header();
}

}  // namespace agents_msgs

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORY__BUILDER_HPP_
