// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from metric_msgs:msg/TrialInfo.idl
// generated code does not contain a copyright notice

#ifndef METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__BUILDER_HPP_
#define METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "metric_msgs/msg/detail/trial_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace metric_msgs
{

namespace msg
{

namespace builder
{

class Init_TrialInfo_movement_jerk
{
public:
  explicit Init_TrialInfo_movement_jerk(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  ::metric_msgs::msg::TrialInfo movement_jerk(::metric_msgs::msg::TrialInfo::_movement_jerk_type arg)
  {
    msg_.movement_jerk = std::move(arg);
    return std::move(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_minimum_time_to_collision
{
public:
  explicit Init_TrialInfo_minimum_time_to_collision(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_movement_jerk minimum_time_to_collision(::metric_msgs::msg::TrialInfo::_minimum_time_to_collision_type arg)
  {
    msg_.minimum_time_to_collision = std::move(arg);
    return Init_TrialInfo_movement_jerk(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_time_in_personal_space
{
public:
  explicit Init_TrialInfo_time_in_personal_space(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_minimum_time_to_collision time_in_personal_space(::metric_msgs::msg::TrialInfo::_time_in_personal_space_type arg)
  {
    msg_.time_in_personal_space = std::move(arg);
    return Init_TrialInfo_minimum_time_to_collision(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_time_not_moving
{
public:
  explicit Init_TrialInfo_time_not_moving(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_time_in_personal_space time_not_moving(::metric_msgs::msg::TrialInfo::_time_not_moving_type arg)
  {
    msg_.time_not_moving = std::move(arg);
    return Init_TrialInfo_time_in_personal_space(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_path_irregularity
{
public:
  explicit Init_TrialInfo_path_irregularity(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_time_not_moving path_irregularity(::metric_msgs::msg::TrialInfo::_path_irregularity_type arg)
  {
    msg_.path_irregularity = std::move(arg);
    return Init_TrialInfo_time_not_moving(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_path_length
{
public:
  explicit Init_TrialInfo_path_length(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_path_irregularity path_length(::metric_msgs::msg::TrialInfo::_path_length_type arg)
  {
    msg_.path_length = std::move(arg);
    return Init_TrialInfo_path_irregularity(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_obj_collisions
{
public:
  explicit Init_TrialInfo_obj_collisions(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_path_length obj_collisions(::metric_msgs::msg::TrialInfo::_obj_collisions_type arg)
  {
    msg_.obj_collisions = std::move(arg);
    return Init_TrialInfo_path_length(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_person_on_robot_collisions
{
public:
  explicit Init_TrialInfo_person_on_robot_collisions(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_obj_collisions person_on_robot_collisions(::metric_msgs::msg::TrialInfo::_person_on_robot_collisions_type arg)
  {
    msg_.person_on_robot_collisions = std::move(arg);
    return Init_TrialInfo_obj_collisions(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_robot_on_person_collisions
{
public:
  explicit Init_TrialInfo_robot_on_person_collisions(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_person_on_robot_collisions robot_on_person_collisions(::metric_msgs::msg::TrialInfo::_robot_on_person_collisions_type arg)
  {
    msg_.robot_on_person_collisions = std::move(arg);
    return Init_TrialInfo_person_on_robot_collisions(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_person_on_robot_personal_dist_violations
{
public:
  explicit Init_TrialInfo_person_on_robot_personal_dist_violations(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_robot_on_person_collisions person_on_robot_personal_dist_violations(::metric_msgs::msg::TrialInfo::_person_on_robot_personal_dist_violations_type arg)
  {
    msg_.person_on_robot_personal_dist_violations = std::move(arg);
    return Init_TrialInfo_robot_on_person_collisions(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_robot_on_person_personal_dist_violations
{
public:
  explicit Init_TrialInfo_robot_on_person_personal_dist_violations(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_person_on_robot_personal_dist_violations robot_on_person_personal_dist_violations(::metric_msgs::msg::TrialInfo::_robot_on_person_personal_dist_violations_type arg)
  {
    msg_.robot_on_person_personal_dist_violations = std::move(arg);
    return Init_TrialInfo_person_on_robot_personal_dist_violations(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_person_on_robot_intimate_dist_violations
{
public:
  explicit Init_TrialInfo_person_on_robot_intimate_dist_violations(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_robot_on_person_personal_dist_violations person_on_robot_intimate_dist_violations(::metric_msgs::msg::TrialInfo::_person_on_robot_intimate_dist_violations_type arg)
  {
    msg_.person_on_robot_intimate_dist_violations = std::move(arg);
    return Init_TrialInfo_robot_on_person_personal_dist_violations(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_robot_on_person_intimate_dist_violations
{
public:
  explicit Init_TrialInfo_robot_on_person_intimate_dist_violations(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_person_on_robot_intimate_dist_violations robot_on_person_intimate_dist_violations(::metric_msgs::msg::TrialInfo::_robot_on_person_intimate_dist_violations_type arg)
  {
    msg_.robot_on_person_intimate_dist_violations = std::move(arg);
    return Init_TrialInfo_person_on_robot_intimate_dist_violations(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_min_dist_to_ped
{
public:
  explicit Init_TrialInfo_min_dist_to_ped(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_robot_on_person_intimate_dist_violations min_dist_to_ped(::metric_msgs::msg::TrialInfo::_min_dist_to_ped_type arg)
  {
    msg_.min_dist_to_ped = std::move(arg);
    return Init_TrialInfo_robot_on_person_intimate_dist_violations(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_robot_poses_ts
{
public:
  explicit Init_TrialInfo_robot_poses_ts(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_min_dist_to_ped robot_poses_ts(::metric_msgs::msg::TrialInfo::_robot_poses_ts_type arg)
  {
    msg_.robot_poses_ts = std::move(arg);
    return Init_TrialInfo_min_dist_to_ped(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_robot_poses
{
public:
  explicit Init_TrialInfo_robot_poses(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_robot_poses_ts robot_poses(::metric_msgs::msg::TrialInfo::_robot_poses_type arg)
  {
    msg_.robot_poses = std::move(arg);
    return Init_TrialInfo_robot_poses_ts(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_min_dist_to_target
{
public:
  explicit Init_TrialInfo_min_dist_to_target(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_robot_poses min_dist_to_target(::metric_msgs::msg::TrialInfo::_min_dist_to_target_type arg)
  {
    msg_.min_dist_to_target = std::move(arg);
    return Init_TrialInfo_robot_poses(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_dist_to_target
{
public:
  explicit Init_TrialInfo_dist_to_target(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_min_dist_to_target dist_to_target(::metric_msgs::msg::TrialInfo::_dist_to_target_type arg)
  {
    msg_.dist_to_target = std::move(arg);
    return Init_TrialInfo_min_dist_to_target(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_robot_goal
{
public:
  explicit Init_TrialInfo_robot_goal(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_dist_to_target robot_goal(::metric_msgs::msg::TrialInfo::_robot_goal_type arg)
  {
    msg_.robot_goal = std::move(arg);
    return Init_TrialInfo_dist_to_target(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_robot_start
{
public:
  explicit Init_TrialInfo_robot_start(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_robot_goal robot_start(::metric_msgs::msg::TrialInfo::_robot_start_type arg)
  {
    msg_.robot_start = std::move(arg);
    return Init_TrialInfo_robot_goal(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_num_actors
{
public:
  explicit Init_TrialInfo_num_actors(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_robot_start num_actors(::metric_msgs::msg::TrialInfo::_num_actors_type arg)
  {
    msg_.num_actors = std::move(arg);
    return Init_TrialInfo_robot_start(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_trial_number
{
public:
  explicit Init_TrialInfo_trial_number(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_num_actors trial_number(::metric_msgs::msg::TrialInfo::_trial_number_type arg)
  {
    msg_.trial_number = std::move(arg);
    return Init_TrialInfo_num_actors(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_trial_name
{
public:
  explicit Init_TrialInfo_trial_name(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_trial_number trial_name(::metric_msgs::msg::TrialInfo::_trial_name_type arg)
  {
    msg_.trial_name = std::move(arg);
    return Init_TrialInfo_trial_number(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_timeout_time
{
public:
  explicit Init_TrialInfo_timeout_time(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_trial_name timeout_time(::metric_msgs::msg::TrialInfo::_timeout_time_type arg)
  {
    msg_.timeout_time = std::move(arg);
    return Init_TrialInfo_trial_name(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_trial_start
{
public:
  explicit Init_TrialInfo_trial_start(::metric_msgs::msg::TrialInfo & msg)
  : msg_(msg)
  {}
  Init_TrialInfo_timeout_time trial_start(::metric_msgs::msg::TrialInfo::_trial_start_type arg)
  {
    msg_.trial_start = std::move(arg);
    return Init_TrialInfo_timeout_time(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

class Init_TrialInfo_header
{
public:
  Init_TrialInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrialInfo_trial_start header(::metric_msgs::msg::TrialInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TrialInfo_trial_start(msg_);
  }

private:
  ::metric_msgs::msg::TrialInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::metric_msgs::msg::TrialInfo>()
{
  return metric_msgs::msg::builder::Init_TrialInfo_header();
}

}  // namespace metric_msgs

#endif  // METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__BUILDER_HPP_
