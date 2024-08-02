// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from simulation_msgs:msg/SceneInfo.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__SCENE_INFO__BUILDER_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__SCENE_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "simulation_msgs/msg/detail/scene_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace simulation_msgs
{

namespace msg
{

namespace builder
{

class Init_SceneInfo_environment
{
public:
  explicit Init_SceneInfo_environment(::simulation_msgs::msg::SceneInfo & msg)
  : msg_(msg)
  {}
  ::simulation_msgs::msg::SceneInfo environment(::simulation_msgs::msg::SceneInfo::_environment_type arg)
  {
    msg_.environment = std::move(arg);
    return std::move(msg_);
  }

private:
  ::simulation_msgs::msg::SceneInfo msg_;
};

class Init_SceneInfo_num_groups
{
public:
  explicit Init_SceneInfo_num_groups(::simulation_msgs::msg::SceneInfo & msg)
  : msg_(msg)
  {}
  Init_SceneInfo_environment num_groups(::simulation_msgs::msg::SceneInfo::_num_groups_type arg)
  {
    msg_.num_groups = std::move(arg);
    return Init_SceneInfo_environment(msg_);
  }

private:
  ::simulation_msgs::msg::SceneInfo msg_;
};

class Init_SceneInfo_num_people
{
public:
  explicit Init_SceneInfo_num_people(::simulation_msgs::msg::SceneInfo & msg)
  : msg_(msg)
  {}
  Init_SceneInfo_num_groups num_people(::simulation_msgs::msg::SceneInfo::_num_people_type arg)
  {
    msg_.num_people = std::move(arg);
    return Init_SceneInfo_num_groups(msg_);
  }

private:
  ::simulation_msgs::msg::SceneInfo msg_;
};

class Init_SceneInfo_robot_target_pose
{
public:
  explicit Init_SceneInfo_robot_target_pose(::simulation_msgs::msg::SceneInfo & msg)
  : msg_(msg)
  {}
  Init_SceneInfo_num_people robot_target_pose(::simulation_msgs::msg::SceneInfo::_robot_target_pose_type arg)
  {
    msg_.robot_target_pose = std::move(arg);
    return Init_SceneInfo_num_people(msg_);
  }

private:
  ::simulation_msgs::msg::SceneInfo msg_;
};

class Init_SceneInfo_robot_start_pose
{
public:
  explicit Init_SceneInfo_robot_start_pose(::simulation_msgs::msg::SceneInfo & msg)
  : msg_(msg)
  {}
  Init_SceneInfo_robot_target_pose robot_start_pose(::simulation_msgs::msg::SceneInfo::_robot_start_pose_type arg)
  {
    msg_.robot_start_pose = std::move(arg);
    return Init_SceneInfo_robot_target_pose(msg_);
  }

private:
  ::simulation_msgs::msg::SceneInfo msg_;
};

class Init_SceneInfo_scenario_name
{
public:
  explicit Init_SceneInfo_scenario_name(::simulation_msgs::msg::SceneInfo & msg)
  : msg_(msg)
  {}
  Init_SceneInfo_robot_start_pose scenario_name(::simulation_msgs::msg::SceneInfo::_scenario_name_type arg)
  {
    msg_.scenario_name = std::move(arg);
    return Init_SceneInfo_robot_start_pose(msg_);
  }

private:
  ::simulation_msgs::msg::SceneInfo msg_;
};

class Init_SceneInfo_header
{
public:
  Init_SceneInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SceneInfo_scenario_name header(::simulation_msgs::msg::SceneInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SceneInfo_scenario_name(msg_);
  }

private:
  ::simulation_msgs::msg::SceneInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::simulation_msgs::msg::SceneInfo>()
{
  return simulation_msgs::msg::builder::Init_SceneInfo_header();
}

}  // namespace simulation_msgs

#endif  // SIMULATION_MSGS__MSG__DETAIL__SCENE_INFO__BUILDER_HPP_
