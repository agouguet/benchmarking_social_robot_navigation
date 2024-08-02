// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from agents_msgs:msg/AgentArray.idl
// generated code does not contain a copyright notice

#ifndef AGENTS_MSGS__MSG__DETAIL__AGENT_ARRAY__STRUCT_HPP_
#define AGENTS_MSGS__MSG__DETAIL__AGENT_ARRAY__STRUCT_HPP_

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
// Member 'agents'
#include "agents_msgs/msg/detail/agent__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__agents_msgs__msg__AgentArray __attribute__((deprecated))
#else
# define DEPRECATED__agents_msgs__msg__AgentArray __declspec(deprecated)
#endif

namespace agents_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AgentArray_
{
  using Type = AgentArray_<ContainerAllocator>;

  explicit AgentArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit AgentArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _agents_type =
    std::vector<agents_msgs::msg::Agent_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<agents_msgs::msg::Agent_<ContainerAllocator>>>;
  _agents_type agents;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__agents(
    const std::vector<agents_msgs::msg::Agent_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<agents_msgs::msg::Agent_<ContainerAllocator>>> & _arg)
  {
    this->agents = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    agents_msgs::msg::AgentArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const agents_msgs::msg::AgentArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<agents_msgs::msg::AgentArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<agents_msgs::msg::AgentArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      agents_msgs::msg::AgentArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<agents_msgs::msg::AgentArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      agents_msgs::msg::AgentArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<agents_msgs::msg::AgentArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<agents_msgs::msg::AgentArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<agents_msgs::msg::AgentArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__agents_msgs__msg__AgentArray
    std::shared_ptr<agents_msgs::msg::AgentArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__agents_msgs__msg__AgentArray
    std::shared_ptr<agents_msgs::msg::AgentArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AgentArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->agents != other.agents) {
      return false;
    }
    return true;
  }
  bool operator!=(const AgentArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AgentArray_

// alias to use template instance with default allocator
using AgentArray =
  agents_msgs::msg::AgentArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace agents_msgs

#endif  // AGENTS_MSGS__MSG__DETAIL__AGENT_ARRAY__STRUCT_HPP_
