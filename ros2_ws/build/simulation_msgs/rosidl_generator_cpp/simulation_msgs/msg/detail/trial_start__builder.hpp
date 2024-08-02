// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from simulation_msgs:msg/TrialStart.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__TRIAL_START__BUILDER_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__TRIAL_START__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "simulation_msgs/msg/detail/trial_start__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace simulation_msgs
{

namespace msg
{

namespace builder
{

class Init_TrialStart_time_limit
{
public:
  explicit Init_TrialStart_time_limit(::simulation_msgs::msg::TrialStart & msg)
  : msg_(msg)
  {}
  ::simulation_msgs::msg::TrialStart time_limit(::simulation_msgs::msg::TrialStart::_time_limit_type arg)
  {
    msg_.time_limit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::simulation_msgs::msg::TrialStart msg_;
};

class Init_TrialStart_people
{
public:
  explicit Init_TrialStart_people(::simulation_msgs::msg::TrialStart & msg)
  : msg_(msg)
  {}
  Init_TrialStart_time_limit people(::simulation_msgs::msg::TrialStart::_people_type arg)
  {
    msg_.people = std::move(arg);
    return Init_TrialStart_time_limit(msg_);
  }

private:
  ::simulation_msgs::msg::TrialStart msg_;
};

class Init_TrialStart_target
{
public:
  explicit Init_TrialStart_target(::simulation_msgs::msg::TrialStart & msg)
  : msg_(msg)
  {}
  Init_TrialStart_people target(::simulation_msgs::msg::TrialStart::_target_type arg)
  {
    msg_.target = std::move(arg);
    return Init_TrialStart_people(msg_);
  }

private:
  ::simulation_msgs::msg::TrialStart msg_;
};

class Init_TrialStart_spawn
{
public:
  explicit Init_TrialStart_spawn(::simulation_msgs::msg::TrialStart & msg)
  : msg_(msg)
  {}
  Init_TrialStart_target spawn(::simulation_msgs::msg::TrialStart::_spawn_type arg)
  {
    msg_.spawn = std::move(arg);
    return Init_TrialStart_target(msg_);
  }

private:
  ::simulation_msgs::msg::TrialStart msg_;
};

class Init_TrialStart_trial_number
{
public:
  explicit Init_TrialStart_trial_number(::simulation_msgs::msg::TrialStart & msg)
  : msg_(msg)
  {}
  Init_TrialStart_spawn trial_number(::simulation_msgs::msg::TrialStart::_trial_number_type arg)
  {
    msg_.trial_number = std::move(arg);
    return Init_TrialStart_spawn(msg_);
  }

private:
  ::simulation_msgs::msg::TrialStart msg_;
};

class Init_TrialStart_trial_name
{
public:
  explicit Init_TrialStart_trial_name(::simulation_msgs::msg::TrialStart & msg)
  : msg_(msg)
  {}
  Init_TrialStart_trial_number trial_name(::simulation_msgs::msg::TrialStart::_trial_name_type arg)
  {
    msg_.trial_name = std::move(arg);
    return Init_TrialStart_trial_number(msg_);
  }

private:
  ::simulation_msgs::msg::TrialStart msg_;
};

class Init_TrialStart_header
{
public:
  Init_TrialStart_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrialStart_trial_name header(::simulation_msgs::msg::TrialStart::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TrialStart_trial_name(msg_);
  }

private:
  ::simulation_msgs::msg::TrialStart msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::simulation_msgs::msg::TrialStart>()
{
  return simulation_msgs::msg::builder::Init_TrialStart_header();
}

}  // namespace simulation_msgs

#endif  // SIMULATION_MSGS__MSG__DETAIL__TRIAL_START__BUILDER_HPP_
