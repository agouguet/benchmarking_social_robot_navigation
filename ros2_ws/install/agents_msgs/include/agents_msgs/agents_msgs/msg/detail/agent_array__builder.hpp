// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from agents_msgs:msg/AgentArray.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT_ARRAY__BUILDER_HPP_
#define AGENTS_MSGS__MSG__DETAIL__AGENT_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "agents_msgs/msg/detail/agent_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace agents_msgs
{

namespace msg
{

namespace builder
{

class Init_AgentArray_agents
{
public:
  explicit Init_AgentArray_agents(::agents_msgs::msg::AgentArray & msg)
  : msg_(msg)
  {}
  ::agents_msgs::msg::AgentArray agents(::agents_msgs::msg::AgentArray::_agents_type arg)
  {
    msg_.agents = std::move(arg);
    return std::move(msg_);
  }

private:
  ::agents_msgs::msg::AgentArray msg_;
};

class Init_AgentArray_header
{
public:
  Init_AgentArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AgentArray_agents header(::agents_msgs::msg::AgentArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AgentArray_agents(msg_);
  }

private:
  ::agents_msgs::msg::AgentArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::agents_msgs::msg::AgentArray>()
{
  return agents_msgs::msg::builder::Init_AgentArray_header();
}

}  // namespace agents_msgs

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT_ARRAY__BUILDER_HPP_
