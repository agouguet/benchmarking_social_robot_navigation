// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from simulation_msgs:msg/RealDepthImage.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__REAL_DEPTH_IMAGE__STRUCT_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__REAL_DEPTH_IMAGE__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__simulation_msgs__msg__RealDepthImage __attribute__((deprecated))
#else
# define DEPRECATED__simulation_msgs__msg__RealDepthImage __declspec(deprecated)
#endif

namespace simulation_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RealDepthImage_
{
  using Type = RealDepthImage_<ContainerAllocator>;

  explicit RealDepthImage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit RealDepthImage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _data_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    simulation_msgs::msg::RealDepthImage_<ContainerAllocator> *;
  using ConstRawPtr =
    const simulation_msgs::msg::RealDepthImage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<simulation_msgs::msg::RealDepthImage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<simulation_msgs::msg::RealDepthImage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      simulation_msgs::msg::RealDepthImage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<simulation_msgs::msg::RealDepthImage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      simulation_msgs::msg::RealDepthImage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<simulation_msgs::msg::RealDepthImage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<simulation_msgs::msg::RealDepthImage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<simulation_msgs::msg::RealDepthImage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__simulation_msgs__msg__RealDepthImage
    std::shared_ptr<simulation_msgs::msg::RealDepthImage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__simulation_msgs__msg__RealDepthImage
    std::shared_ptr<simulation_msgs::msg::RealDepthImage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RealDepthImage_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const RealDepthImage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RealDepthImage_

// alias to use template instance with default allocator
using RealDepthImage =
  simulation_msgs::msg::RealDepthImage_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace simulation_msgs

#endif  // SIMULATION_MSGS__MSG__DETAIL__REAL_DEPTH_IMAGE__STRUCT_HPP_
