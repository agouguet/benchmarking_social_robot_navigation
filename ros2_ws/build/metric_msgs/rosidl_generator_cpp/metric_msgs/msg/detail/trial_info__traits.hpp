// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from metric_msgs:msg/TrialInfo.idl
// generated code does not contain a copyright notice

#ifndef METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__TRAITS_HPP_
#define METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "metric_msgs/msg/detail/trial_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'trial_start'
// Member 'robot_poses_ts'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'robot_start'
// Member 'robot_goal'
// Member 'robot_poses'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace metric_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrialInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: trial_start
  {
    out << "trial_start: ";
    to_flow_style_yaml(msg.trial_start, out);
    out << ", ";
  }

  // member: timeout_time
  {
    out << "timeout_time: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout_time, out);
    out << ", ";
  }

  // member: trial_name
  {
    out << "trial_name: ";
    rosidl_generator_traits::value_to_yaml(msg.trial_name, out);
    out << ", ";
  }

  // member: trial_number
  {
    out << "trial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.trial_number, out);
    out << ", ";
  }

  // member: num_actors
  {
    out << "num_actors: ";
    rosidl_generator_traits::value_to_yaml(msg.num_actors, out);
    out << ", ";
  }

  // member: robot_start
  {
    out << "robot_start: ";
    to_flow_style_yaml(msg.robot_start, out);
    out << ", ";
  }

  // member: robot_goal
  {
    out << "robot_goal: ";
    to_flow_style_yaml(msg.robot_goal, out);
    out << ", ";
  }

  // member: dist_to_target
  {
    out << "dist_to_target: ";
    rosidl_generator_traits::value_to_yaml(msg.dist_to_target, out);
    out << ", ";
  }

  // member: min_dist_to_target
  {
    out << "min_dist_to_target: ";
    rosidl_generator_traits::value_to_yaml(msg.min_dist_to_target, out);
    out << ", ";
  }

  // member: robot_poses
  {
    if (msg.robot_poses.size() == 0) {
      out << "robot_poses: []";
    } else {
      out << "robot_poses: [";
      size_t pending_items = msg.robot_poses.size();
      for (auto item : msg.robot_poses) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: robot_poses_ts
  {
    if (msg.robot_poses_ts.size() == 0) {
      out << "robot_poses_ts: []";
    } else {
      out << "robot_poses_ts: [";
      size_t pending_items = msg.robot_poses_ts.size();
      for (auto item : msg.robot_poses_ts) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: min_dist_to_ped
  {
    out << "min_dist_to_ped: ";
    rosidl_generator_traits::value_to_yaml(msg.min_dist_to_ped, out);
    out << ", ";
  }

  // member: robot_on_person_intimate_dist_violations
  {
    out << "robot_on_person_intimate_dist_violations: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_on_person_intimate_dist_violations, out);
    out << ", ";
  }

  // member: person_on_robot_intimate_dist_violations
  {
    out << "person_on_robot_intimate_dist_violations: ";
    rosidl_generator_traits::value_to_yaml(msg.person_on_robot_intimate_dist_violations, out);
    out << ", ";
  }

  // member: robot_on_person_personal_dist_violations
  {
    out << "robot_on_person_personal_dist_violations: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_on_person_personal_dist_violations, out);
    out << ", ";
  }

  // member: person_on_robot_personal_dist_violations
  {
    out << "person_on_robot_personal_dist_violations: ";
    rosidl_generator_traits::value_to_yaml(msg.person_on_robot_personal_dist_violations, out);
    out << ", ";
  }

  // member: robot_on_person_collisions
  {
    out << "robot_on_person_collisions: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_on_person_collisions, out);
    out << ", ";
  }

  // member: person_on_robot_collisions
  {
    out << "person_on_robot_collisions: ";
    rosidl_generator_traits::value_to_yaml(msg.person_on_robot_collisions, out);
    out << ", ";
  }

  // member: obj_collisions
  {
    out << "obj_collisions: ";
    rosidl_generator_traits::value_to_yaml(msg.obj_collisions, out);
    out << ", ";
  }

  // member: path_length
  {
    out << "path_length: ";
    rosidl_generator_traits::value_to_yaml(msg.path_length, out);
    out << ", ";
  }

  // member: path_irregularity
  {
    out << "path_irregularity: ";
    rosidl_generator_traits::value_to_yaml(msg.path_irregularity, out);
    out << ", ";
  }

  // member: time_not_moving
  {
    out << "time_not_moving: ";
    rosidl_generator_traits::value_to_yaml(msg.time_not_moving, out);
    out << ", ";
  }

  // member: time_in_personal_space
  {
    out << "time_in_personal_space: ";
    rosidl_generator_traits::value_to_yaml(msg.time_in_personal_space, out);
    out << ", ";
  }

  // member: minimum_time_to_collision
  {
    out << "minimum_time_to_collision: ";
    rosidl_generator_traits::value_to_yaml(msg.minimum_time_to_collision, out);
    out << ", ";
  }

  // member: movement_jerk
  {
    out << "movement_jerk: ";
    rosidl_generator_traits::value_to_yaml(msg.movement_jerk, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrialInfo & msg,
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

  // member: trial_start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trial_start:\n";
    to_block_style_yaml(msg.trial_start, out, indentation + 2);
  }

  // member: timeout_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timeout_time: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout_time, out);
    out << "\n";
  }

  // member: trial_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trial_name: ";
    rosidl_generator_traits::value_to_yaml(msg.trial_name, out);
    out << "\n";
  }

  // member: trial_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.trial_number, out);
    out << "\n";
  }

  // member: num_actors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_actors: ";
    rosidl_generator_traits::value_to_yaml(msg.num_actors, out);
    out << "\n";
  }

  // member: robot_start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_start:\n";
    to_block_style_yaml(msg.robot_start, out, indentation + 2);
  }

  // member: robot_goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_goal:\n";
    to_block_style_yaml(msg.robot_goal, out, indentation + 2);
  }

  // member: dist_to_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dist_to_target: ";
    rosidl_generator_traits::value_to_yaml(msg.dist_to_target, out);
    out << "\n";
  }

  // member: min_dist_to_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_dist_to_target: ";
    rosidl_generator_traits::value_to_yaml(msg.min_dist_to_target, out);
    out << "\n";
  }

  // member: robot_poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.robot_poses.size() == 0) {
      out << "robot_poses: []\n";
    } else {
      out << "robot_poses:\n";
      for (auto item : msg.robot_poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: robot_poses_ts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.robot_poses_ts.size() == 0) {
      out << "robot_poses_ts: []\n";
    } else {
      out << "robot_poses_ts:\n";
      for (auto item : msg.robot_poses_ts) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: min_dist_to_ped
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_dist_to_ped: ";
    rosidl_generator_traits::value_to_yaml(msg.min_dist_to_ped, out);
    out << "\n";
  }

  // member: robot_on_person_intimate_dist_violations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_on_person_intimate_dist_violations: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_on_person_intimate_dist_violations, out);
    out << "\n";
  }

  // member: person_on_robot_intimate_dist_violations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "person_on_robot_intimate_dist_violations: ";
    rosidl_generator_traits::value_to_yaml(msg.person_on_robot_intimate_dist_violations, out);
    out << "\n";
  }

  // member: robot_on_person_personal_dist_violations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_on_person_personal_dist_violations: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_on_person_personal_dist_violations, out);
    out << "\n";
  }

  // member: person_on_robot_personal_dist_violations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "person_on_robot_personal_dist_violations: ";
    rosidl_generator_traits::value_to_yaml(msg.person_on_robot_personal_dist_violations, out);
    out << "\n";
  }

  // member: robot_on_person_collisions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_on_person_collisions: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_on_person_collisions, out);
    out << "\n";
  }

  // member: person_on_robot_collisions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "person_on_robot_collisions: ";
    rosidl_generator_traits::value_to_yaml(msg.person_on_robot_collisions, out);
    out << "\n";
  }

  // member: obj_collisions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obj_collisions: ";
    rosidl_generator_traits::value_to_yaml(msg.obj_collisions, out);
    out << "\n";
  }

  // member: path_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "path_length: ";
    rosidl_generator_traits::value_to_yaml(msg.path_length, out);
    out << "\n";
  }

  // member: path_irregularity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "path_irregularity: ";
    rosidl_generator_traits::value_to_yaml(msg.path_irregularity, out);
    out << "\n";
  }

  // member: time_not_moving
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_not_moving: ";
    rosidl_generator_traits::value_to_yaml(msg.time_not_moving, out);
    out << "\n";
  }

  // member: time_in_personal_space
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_in_personal_space: ";
    rosidl_generator_traits::value_to_yaml(msg.time_in_personal_space, out);
    out << "\n";
  }

  // member: minimum_time_to_collision
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "minimum_time_to_collision: ";
    rosidl_generator_traits::value_to_yaml(msg.minimum_time_to_collision, out);
    out << "\n";
  }

  // member: movement_jerk
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "movement_jerk: ";
    rosidl_generator_traits::value_to_yaml(msg.movement_jerk, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrialInfo & msg, bool use_flow_style = false)
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

}  // namespace metric_msgs

namespace rosidl_generator_traits
{

[[deprecated("use metric_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const metric_msgs::msg::TrialInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  metric_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use metric_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const metric_msgs::msg::TrialInfo & msg)
{
  return metric_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<metric_msgs::msg::TrialInfo>()
{
  return "metric_msgs::msg::TrialInfo";
}

template<>
inline const char * name<metric_msgs::msg::TrialInfo>()
{
  return "metric_msgs/msg/TrialInfo";
}

template<>
struct has_fixed_size<metric_msgs::msg::TrialInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<metric_msgs::msg::TrialInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<metric_msgs::msg::TrialInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__TRAITS_HPP_
