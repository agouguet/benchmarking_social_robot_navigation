// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from agents_msgs:msg/Agent.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT__STRUCT_HPP_
#define AGENTS_MSGS__MSG__DETAIL__AGENT__STRUCT_HPP_

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
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__agents_msgs__msg__Agent __attribute__((deprecated))
#else
# define DEPRECATED__agents_msgs__msg__Agent __declspec(deprecated)
#endif

namespace agents_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Agent_
{
  using Type = Agent_<ContainerAllocator>;

  explicit Agent_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ull;
      this->visible_by_robot = false;
    }
  }

  explicit Agent_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init),
    velocity(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ull;
      this->visible_by_robot = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    uint64_t;
  _id_type id;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _velocity_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _velocity_type velocity;
  using _visible_by_robot_type =
    bool;
  _visible_by_robot_type visible_by_robot;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id(
    const uint64_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__visible_by_robot(
    const bool & _arg)
  {
    this->visible_by_robot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    agents_msgs::msg::Agent_<ContainerAllocator> *;
  using ConstRawPtr =
    const agents_msgs::msg::Agent_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<agents_msgs::msg::Agent_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<agents_msgs::msg::Agent_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      agents_msgs::msg::Agent_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<agents_msgs::msg::Agent_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      agents_msgs::msg::Agent_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<agents_msgs::msg::Agent_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<agents_msgs::msg::Agent_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<agents_msgs::msg::Agent_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__agents_msgs__msg__Agent
    std::shared_ptr<agents_msgs::msg::Agent_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__agents_msgs__msg__Agent
    std::shared_ptr<agents_msgs::msg::Agent_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Agent_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->visible_by_robot != other.visible_by_robot) {
      return false;
    }
    return true;
  }
  bool operator!=(const Agent_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Agent_

// alias to use template instance with default allocator
using Agent =
  agents_msgs::msg::Agent_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace agents_msgs

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT__STRUCT_HPP_
