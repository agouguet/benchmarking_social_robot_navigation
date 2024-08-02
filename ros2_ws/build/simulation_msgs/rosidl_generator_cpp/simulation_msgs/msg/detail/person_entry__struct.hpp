// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from simulation_msgs:msg/PersonEntry.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY__STRUCT_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY__STRUCT_HPP_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__simulation_msgs__msg__PersonEntry __attribute__((deprecated))
#else
# define DEPRECATED__simulation_msgs__msg__PersonEntry __declspec(deprecated)
#endif

namespace simulation_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PersonEntry_
{
  using Type = PersonEntry_<ContainerAllocator>;

  explicit PersonEntry_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    twist(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->track_id = 0ull;
    }
  }

  explicit PersonEntry_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init),
    twist(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->track_id = 0ull;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _track_id_type =
    uint64_t;
  _track_id_type track_id;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _twist_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _twist_type twist;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__track_id(
    const uint64_t & _arg)
  {
    this->track_id = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__twist(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->twist = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    simulation_msgs::msg::PersonEntry_<ContainerAllocator> *;
  using ConstRawPtr =
    const simulation_msgs::msg::PersonEntry_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<simulation_msgs::msg::PersonEntry_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<simulation_msgs::msg::PersonEntry_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      simulation_msgs::msg::PersonEntry_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<simulation_msgs::msg::PersonEntry_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      simulation_msgs::msg::PersonEntry_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<simulation_msgs::msg::PersonEntry_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<simulation_msgs::msg::PersonEntry_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<simulation_msgs::msg::PersonEntry_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__simulation_msgs__msg__PersonEntry
    std::shared_ptr<simulation_msgs::msg::PersonEntry_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__simulation_msgs__msg__PersonEntry
    std::shared_ptr<simulation_msgs::msg::PersonEntry_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PersonEntry_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->track_id != other.track_id) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->twist != other.twist) {
      return false;
    }
    return true;
  }
  bool operator!=(const PersonEntry_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PersonEntry_

// alias to use template instance with default allocator
using PersonEntry =
  simulation_msgs::msg::PersonEntry_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace simulation_msgs

#endif  // SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY__STRUCT_HPP_
