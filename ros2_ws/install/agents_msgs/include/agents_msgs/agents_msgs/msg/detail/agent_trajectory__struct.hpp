// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from agents_msgs:msg/AgentTrajectory.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORY__STRUCT_HPP_
#define AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORY__STRUCT_HPP_

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
// Member 'poses'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__agents_msgs__msg__AgentTrajectory __attribute__((deprecated))
#else
# define DEPRECATED__agents_msgs__msg__AgentTrajectory __declspec(deprecated)
#endif

namespace agents_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AgentTrajectory_
{
  using Type = AgentTrajectory_<ContainerAllocator>;

  explicit AgentTrajectory_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ull;
    }
  }

  explicit AgentTrajectory_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ull;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    uint64_t;
  _id_type id;
  using _poses_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _poses_type poses;

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
  Type & set__poses(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->poses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    agents_msgs::msg::AgentTrajectory_<ContainerAllocator> *;
  using ConstRawPtr =
    const agents_msgs::msg::AgentTrajectory_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<agents_msgs::msg::AgentTrajectory_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<agents_msgs::msg::AgentTrajectory_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      agents_msgs::msg::AgentTrajectory_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<agents_msgs::msg::AgentTrajectory_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      agents_msgs::msg::AgentTrajectory_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<agents_msgs::msg::AgentTrajectory_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<agents_msgs::msg::AgentTrajectory_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<agents_msgs::msg::AgentTrajectory_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__agents_msgs__msg__AgentTrajectory
    std::shared_ptr<agents_msgs::msg::AgentTrajectory_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__agents_msgs__msg__AgentTrajectory
    std::shared_ptr<agents_msgs::msg::AgentTrajectory_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AgentTrajectory_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->poses != other.poses) {
      return false;
    }
    return true;
  }
  bool operator!=(const AgentTrajectory_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AgentTrajectory_

// alias to use template instance with default allocator
using AgentTrajectory =
  agents_msgs::msg::AgentTrajectory_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace agents_msgs

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT_TRAJECTORY__STRUCT_HPP_
