// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from metric_msgs:msg/TrialInfo.idl
// generated code does not contain a copyright notice

#ifndef METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__STRUCT_H_
#define METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'trial_start'
// Member 'robot_poses_ts'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'trial_name'
#include "rosidl_runtime_c/string.h"
// Member 'robot_start'
// Member 'robot_goal'
// Member 'robot_poses'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/TrialInfo in the package metric_msgs.
/**
  * Message containing the information of running a single A-B navigation trial
 */
typedef struct metric_msgs__msg__TrialInfo
{
  std_msgs__msg__Header header;
  /// Information about the current interaction
  /// When did we start tracking metrics for this trial
  builtin_interfaces__msg__Time trial_start;
  /// How long is allowed for the episode have to complete?
  double timeout_time;
  /// Which trial name are we running
  rosidl_runtime_c__String trial_name;
  /// Which trial number are we running
  uint16_t trial_number;
  /// How many people in the scene?
  uint32_t num_actors;
  /// Robot start / goal locations
  geometry_msgs__msg__Pose robot_start;
  geometry_msgs__msg__Pose robot_goal;
  /// Robot location / distance relative to start / goal
  /// Current distance to target
  double dist_to_target;
  /// Closest difference to the target the robot has come
  double min_dist_to_target;
  /// Poses of the robot over the episode
  geometry_msgs__msg__Pose__Sequence robot_poses;
  /// Timestamps of the robot poses over the episode
  builtin_interfaces__msg__Time__Sequence robot_poses_ts;
  /// Robot location relative to pedestrians
  /// Minimum distance to any pedestrian throughout the trial
  double min_dist_to_ped;
  /// Collisions between robots and people
  /// Robot passes w/in the intimate distance of a pedestrian
  uint32_t robot_on_person_intimate_dist_violations;
  /// A person passes w/in the intimate distance of the robot
  uint32_t person_on_robot_intimate_dist_violations;
  /// Robot passes w/in the personal distance of a pedestrian
  uint32_t robot_on_person_personal_dist_violations;
  /// A person passes w/in the personal distance of the robot
  uint32_t person_on_robot_personal_dist_violations;
  /// Number of times that the robot collided with a person
  uint32_t robot_on_person_collisions;
  /// Number of times that a person collided with the robot
  uint32_t person_on_robot_collisions;
  /// Collisions w/ static objects
  /// Number of times that the robot collided with a static object
  uint32_t obj_collisions;
  ///  Computed post-hoc
  /// bool                     completed                                      # was the robots finals distance to the goal within the desired distance?
  /// float64                  targ_dist_norm                                 # dist_to_target normalized by path length
  ///  Approximate distance traveled by robot
  double path_length;
  /// float64                  mean_dist_to_target_not_moving                 # How far away from the target was the robot while not moving
  /// bool                     episode_timed_out
  double path_irregularity;
  /// float64                  path_efficiency
  ///  Seconds the robot was not moving. Computed by checking if the robot moved more than 0.05m since a check the last 1s ago. If not, accumulate the time since the last check.
  double time_not_moving;
  double time_in_personal_space;
  double minimum_time_to_collision;
  double movement_jerk;
} metric_msgs__msg__TrialInfo;

// Struct for a sequence of metric_msgs__msg__TrialInfo.
typedef struct metric_msgs__msg__TrialInfo__Sequence
{
  metric_msgs__msg__TrialInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} metric_msgs__msg__TrialInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__STRUCT_H_
