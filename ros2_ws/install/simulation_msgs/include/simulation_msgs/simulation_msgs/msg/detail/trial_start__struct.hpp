// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from simulation_msgs:msg/TrialStart.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__TRIAL_START__STRUCT_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__TRIAL_START__STRUCT_HPP_

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
// Member 'spawn'
// Member 'target'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'people'
#include "geometry_msgs/msg/detail/pose_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__simulation_msgs__msg__TrialStart __attribute__((deprecated))
#else
# define DEPRECATED__simulation_msgs__msg__TrialStart __declspec(deprecated)
#endif

namespace simulation_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrialStart_
{
  using Type = TrialStart_<ContainerAllocator>;

  explicit TrialStart_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    spawn(_init),
    target(_init),
    people(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->trial_name = "";
      this->trial_number = 0;
      this->time_limit = 0.0;
    }
  }

  explicit TrialStart_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    trial_name(_alloc),
    spawn(_alloc, _init),
    target(_alloc, _init),
    people(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->trial_name = "";
      this->trial_number = 0;
      this->time_limit = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _trial_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _trial_name_type trial_name;
  using _trial_number_type =
    uint16_t;
  _trial_number_type trial_number;
  using _spawn_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _spawn_type spawn;
  using _target_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _target_type target;
  using _people_type =
    geometry_msgs::msg::PoseArray_<ContainerAllocator>;
  _people_type people;
  using _time_limit_type =
    double;
  _time_limit_type time_limit;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
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
  Type & set__spawn(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->spawn = _arg;
    return *this;
  }
  Type & set__target(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->target = _arg;
    return *this;
  }
  Type & set__people(
    const geometry_msgs::msg::PoseArray_<ContainerAllocator> & _arg)
  {
    this->people = _arg;
    return *this;
  }
  Type & set__time_limit(
    const double & _arg)
  {
    this->time_limit = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    simulation_msgs::msg::TrialStart_<ContainerAllocator> *;
  using ConstRawPtr =
    const simulation_msgs::msg::TrialStart_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<simulation_msgs::msg::TrialStart_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<simulation_msgs::msg::TrialStart_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      simulation_msgs::msg::TrialStart_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<simulation_msgs::msg::TrialStart_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      simulation_msgs::msg::TrialStart_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<simulation_msgs::msg::TrialStart_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<simulation_msgs::msg::TrialStart_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<simulation_msgs::msg::TrialStart_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__simulation_msgs__msg__TrialStart
    std::shared_ptr<simulation_msgs::msg::TrialStart_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__simulation_msgs__msg__TrialStart
    std::shared_ptr<simulation_msgs::msg::TrialStart_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrialStart_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->trial_name != other.trial_name) {
      return false;
    }
    if (this->trial_number != other.trial_number) {
      return false;
    }
    if (this->spawn != other.spawn) {
      return false;
    }
    if (this->target != other.target) {
      return false;
    }
    if (this->people != other.people) {
      return false;
    }
    if (this->time_limit != other.time_limit) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrialStart_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrialStart_

// alias to use template instance with default allocator
using TrialStart =
  simulation_msgs::msg::TrialStart_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace simulation_msgs

#endif  // SIMULATION_MSGS__MSG__DETAIL__TRIAL_START__STRUCT_HPP_
