// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from agents_msgs:msg/AgentTrajectories.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORIES__BUILDER_HPP_
#define AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORIES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "agents_msgs/msg/detail/agent_trajectories__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace agents_msgs
{

namespace msg
{

namespace builder
{

class Init_AgentTrajectories_trajectories
{
public:
  explicit Init_AgentTrajectories_trajectories(::agents_msgs::msg::AgentTrajectories & msg)
  : msg_(msg)
  {}
  ::agents_msgs::msg::AgentTrajectories trajectories(::agents_msgs::msg::AgentTrajectories::_trajectories_type arg)
  {
    msg_.trajectories = std::move(arg);
    return std::move(msg_);
  }

private:
  ::agents_msgs::msg::AgentTrajectories msg_;
};

class Init_AgentTrajectories_header
{
public:
  Init_AgentTrajectories_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AgentTrajectories_trajectories header(::agents_msgs::msg::AgentTrajectories::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AgentTrajectories_trajectories(msg_);
  }

private:
  ::agents_msgs::msg::AgentTrajectories msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::agents_msgs::msg::AgentTrajectories>()
{
  return agents_msgs::msg::builder::Init_AgentTrajectories_header();
}

}  // namespace agents_msgs

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORIES__BUILDER_HPP_
