// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from metric_msgs:msg/TrialInfo.idl
// generated code does not contain a copyright notice

#ifndef METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__STRUCT_HPP_
#define METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'trial_start'
// Member 'robot_poses_ts'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'robot_start'
// Member 'robot_goal'
// Member 'robot_poses'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__metric_msgs__msg__TrialInfo __attribute__((deprecated))
#else
# define DEPRECATED__metric_msgs__msg__TrialInfo __declspec(deprecated)
#endif

namespace metric_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrialInfo_
{
  using Type = TrialInfo_<ContainerAllocator>;

  explicit TrialInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    trial_start(_init),
    robot_start(_init),
    robot_goal(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timeout_time = 0.0;
      this->trial_name = "";
      this->trial_number = 0;
      this->num_actors = 0ul;
      this->dist_to_target = 0.0;
      this->min_dist_to_target = 0.0;
      this->min_dist_to_ped = 0.0;
      this->robot_on_person_intimate_dist_violations = 0ul;
      this->person_on_robot_intimate_dist_violations = 0ul;
      this->robot_on_person_personal_dist_violations = 0ul;
      this->person_on_robot_personal_dist_violations = 0ul;
      this->robot_on_person_collisions = 0ul;
      this->person_on_robot_collisions = 0ul;
      this->obj_collisions = 0ul;
      this->path_length = 0.0;
      this->path_irregularity = 0.0;
      this->time_not_moving = 0.0;
      this->time_in_personal_space = 0.0;
      this->minimum_time_to_collision = 0.0;
      this->movement_jerk = 0.0;
    }
  }

  explicit TrialInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    trial_start(_alloc, _init),
    trial_name(_alloc),
    robot_start(_alloc, _init),
    robot_goal(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timeout_time = 0.0;
      this->trial_name = "";
      this->trial_number = 0;
      this->num_actors = 0ul;
      this->dist_to_target = 0.0;
      this->min_dist_to_target = 0.0;
      this->min_dist_to_ped = 0.0;
      this->robot_on_person_intimate_dist_violations = 0ul;
      this->person_on_robot_intimate_dist_violations = 0ul;
      this->robot_on_person_personal_dist_violations = 0ul;
      this->person_on_robot_personal_dist_violations = 0ul;
      this->robot_on_person_collisions = 0ul;
      this->person_on_robot_collisions = 0ul;
      this->obj_collisions = 0ul;
      this->path_length = 0.0;
      this->path_irregularity = 0.0;
      this->time_not_moving = 0.0;
      this->time_in_personal_space = 0.0;
      this->minimum_time_to_collision = 0.0;
      this->movement_jerk = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _trial_start_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _trial_start_type trial_start;
  using _timeout_time_type =
    double;
  _timeout_time_type timeout_time;
  using _trial_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _trial_name_type trial_name;
  using _trial_number_type =
    uint16_t;
  _trial_number_type trial_number;
  using _num_actors_type =
    uint32_t;
  _num_actors_type num_actors;
  using _robot_start_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _robot_start_type robot_start;
  using _robot_goal_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _robot_goal_type robot_goal;
  using _dist_to_target_type =
    double;
  _dist_to_target_type dist_to_target;
  using _min_dist_to_target_type =
    double;
  _min_dist_to_target_type min_dist_to_target;
  using _robot_poses_type =
    std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>>;
  _robot_poses_type robot_poses;
  using _robot_poses_ts_type =
    std::vector<builtin_interfaces::msg::Time_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<builtin_interfaces::msg::Time_<ContainerAllocator>>>;
  _robot_poses_ts_type robot_poses_ts;
  using _min_dist_to_ped_type =
    double;
  _min_dist_to_ped_type min_dist_to_ped;
  using _robot_on_person_intimate_dist_violations_type =
    uint32_t;
  _robot_on_person_intimate_dist_violations_type robot_on_person_intimate_dist_violations;
  using _person_on_robot_intimate_dist_violations_type =
    uint32_t;
  _person_on_robot_intimate_dist_violations_type person_on_robot_intimate_dist_violations;
  using _robot_on_person_personal_dist_violations_type =
    uint32_t;
  _robot_on_person_personal_dist_violations_type robot_on_person_personal_dist_violations;
  using _person_on_robot_personal_dist_violations_type =
    uint32_t;
  _person_on_robot_personal_dist_violations_type person_on_robot_personal_dist_violations;
  using _robot_on_person_collisions_type =
    uint32_t;
  _robot_on_person_collisions_type robot_on_person_collisions;
  using _person_on_robot_collisions_type =
    uint32_t;
  _person_on_robot_collisions_type person_on_robot_collisions;
  using _obj_collisions_type =
    uint32_t;
  _obj_collisions_type obj_collisions;
  using _path_length_type =
    double;
  _path_length_type path_length;
  using _path_irregularity_type =
    double;
  _path_irregularity_type path_irregularity;
  using _time_not_moving_type =
    double;
  _time_not_moving_type time_not_moving;
  using _time_in_personal_space_type =
    double;
  _time_in_personal_space_type time_in_personal_space;
  using _minimum_time_to_collision_type =
    double;
  _minimum_time_to_collision_type minimum_time_to_collision;
  using _movement_jerk_type =
    double;
  _movement_jerk_type movement_jerk;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__trial_start(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->trial_start = _arg;
    return *this;
  }
  Type & set__timeout_time(
    const double & _arg)
  {
    this->timeout_time = _arg;
    return *this;
  }
  Type & set__trial_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->trial_name = _arg;
    return *this;
  }
  Type & set__trial_number(
    const uint16_t & _arg)
  {
    this->trial_number = _arg;
    return *this;
  }
  Type & set__num_actors(
    const uint32_t & _arg)
  {
    this->num_actors = _arg;
    return *this;
  }
  Type & set__robot_start(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->robot_start = _arg;
    return *this;
  }
  Type & set__robot_goal(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->robot_goal = _arg;
    return *this;
  }
  Type & set__dist_to_target(
    const double & _arg)
  {
    this->dist_to_target = _arg;
    return *this;
  }
  Type & set__min_dist_to_target(
    const double & _arg)
  {
    this->min_dist_to_target = _arg;
    return *this;
  }
  Type & set__robot_poses(
    const std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>> & _arg)
  {
    this->robot_poses = _arg;
    return *this;
  }
  Type & set__robot_poses_ts(
    const std::vector<builtin_interfaces::msg::Time_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<builtin_interfaces::msg::Time_<ContainerAllocator>>> & _arg)
  {
    this->robot_poses_ts = _arg;
    return *this;
  }
  Type & set__min_dist_to_ped(
    const double & _arg)
  {
    this->min_dist_to_ped = _arg;
    return *this;
  }
  Type & set__robot_on_person_intimate_dist_violations(
    const uint32_t & _arg)
  {
    this->robot_on_person_intimate_dist_violations = _arg;
    return *this;
  }
  Type & set__person_on_robot_intimate_dist_violations(
    const uint32_t & _arg)
  {
    this->person_on_robot_intimate_dist_violations = _arg;
    return *this;
  }
  Type & set__robot_on_person_personal_dist_violations(
    const uint32_t & _arg)
  {
    this->robot_on_person_personal_dist_violations = _arg;
    return *this;
  }
  Type & set__person_on_robot_personal_dist_violations(
    const uint32_t & _arg)
  {
    this->person_on_robot_personal_dist_violations = _arg;
    return *this;
  }
  Type & set__robot_on_person_collisions(
    const uint32_t & _arg)
  {
    this->robot_on_person_collisions = _arg;
    return *this;
  }
  Type & set__person_on_robot_collisions(
    const uint32_t & _arg)
  {
    this->person_on_robot_collisions = _arg;
    return *this;
  }
  Type & set__obj_collisions(
    const uint32_t & _arg)
  {
    this->obj_collisions = _arg;
    return *this;
  }
  Type & set__path_length(
    const double & _arg)
  {
    this->path_length = _arg;
    return *this;
  }
  Type & set__path_irregularity(
    const double & _arg)
  {
    this->path_irregularity = _arg;
    return *this;
  }
  Type & set__time_not_moving(
    const double & _arg)
  {
    this->time_not_moving = _arg;
    return *this;
  }
  Type & set__time_in_personal_space(
    const double & _arg)
  {
    this->time_in_personal_space = _arg;
    return *this;
  }
  Type & set__minimum_time_to_collision(
    const double & _arg)
  {
    this->minimum_time_to_collision = _arg;
    return *this;
  }
  Type & set__movement_jerk(
    const double & _arg)
  {
    this->movement_jerk = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    metric_msgs::msg::TrialInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const metric_msgs::msg::TrialInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<metric_msgs::msg::TrialInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<metric_msgs::msg::TrialInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      metric_msgs::msg::TrialInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<metric_msgs::msg::TrialInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      metric_msgs::msg::TrialInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<metric_msgs::msg::TrialInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<metric_msgs::msg::TrialInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<metric_msgs::msg::TrialInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__metric_msgs__msg__TrialInfo
    std::shared_ptr<metric_msgs::msg::TrialInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__metric_msgs__msg__TrialInfo
    std::shared_ptr<metric_msgs::msg::TrialInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrialInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->trial_start != other.trial_start) {
      return false;
    }
    if (this->timeout_time != other.timeout_time) {
      return false;
    }
    if (this->trial_name != other.trial_name) {
      return false;
    }
    if (this->trial_number != other.trial_number) {
      return false;
    }
    if (this->num_actors != other.num_actors) {
      return false;
    }
    if (this->robot_start != other.robot_start) {
      return false;
    }
    if (this->robot_goal != other.robot_goal) {
      return false;
    }
    if (this->dist_to_target != other.dist_to_target) {
      return false;
    }
    if (this->min_dist_to_target != other.min_dist_to_target) {
      return false;
    }
    if (this->robot_poses != other.robot_poses) {
      return false;
    }
    if (this->robot_poses_ts != other.robot_poses_ts) {
      return false;
    }
    if (this->min_dist_to_ped != other.min_dist_to_ped) {
      return false;
    }
    if (this->robot_on_person_intimate_dist_violations != other.robot_on_person_intimate_dist_violations) {
      return false;
    }
    if (this->person_on_robot_intimate_dist_violations != other.person_on_robot_intimate_dist_violations) {
      return false;
    }
    if (this->robot_on_person_personal_dist_violations != other.robot_on_person_personal_dist_violations) {
      return false;
    }
    if (this->person_on_robot_personal_dist_violations != other.person_on_robot_personal_dist_violations) {
      return false;
    }
    if (this->robot_on_person_collisions != other.robot_on_person_collisions) {
      return false;
    }
    if (this->person_on_robot_collisions != other.person_on_robot_collisions) {
      return false;
    }
    if (this->obj_collisions != other.obj_collisions) {
      return false;
    }
    if (this->path_length != other.path_length) {
      return false;
    }
    if (this->path_irregularity != other.path_irregularity) {
      return false;
    }
    if (this->time_not_moving != other.time_not_moving) {
      return false;
    }
    if (this->time_in_personal_space != other.time_in_personal_space) {
      return false;
    }
    if (this->minimum_time_to_collision != other.minimum_time_to_collision) {
      return false;
    }
    if (this->movement_jerk != other.movement_jerk) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrialInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrialInfo_

// alias to use template instance with default allocator
using TrialInfo =
  metric_msgs::msg::TrialInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace metric_msgs

#endif  // METRIC_MSGS__MSG__DETAIL__TRIAL_INFO__STRUCT_HPP_
