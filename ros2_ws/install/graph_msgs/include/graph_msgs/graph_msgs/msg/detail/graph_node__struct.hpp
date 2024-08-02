// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from graph_msgs:msg/GraphNode.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSGS__MSG__DETAIL__GRAPH_NODE__STRUCT_HPP_
#define GRAPH_MSGS__MSG__DETAIL__GRAPH_NODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__graph_msgs__msg__GraphNode __attribute__((deprecated))
#else
# define DEPRECATED__graph_msgs__msg__GraphNode __declspec(deprecated)
#endif

namespace graph_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GraphNode_
{
  using Type = GraphNode_<ContainerAllocator>;

  explicit GraphNode_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ull;
      this->x = 0.0;
      this->y = 0.0;
      this->occupied = false;
    }
  }

  explicit GraphNode_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ull;
      this->x = 0.0;
      this->y = 0.0;
      this->occupied = false;
    }
  }

  // field types and members
  using _id_type =
    uint64_t;
  _id_type id;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;
  using _occupied_type =
    bool;
  _occupied_type occupied;

  // setters for named parameter idiom
  Type & set__id(
    const uint64_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__occupied(
    const bool & _arg)
  {
    this->occupied = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    graph_msgs::msg::GraphNode_<ContainerAllocator> *;
  using ConstRawPtr =
    const graph_msgs::msg::GraphNode_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<graph_msgs::msg::GraphNode_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<graph_msgs::msg::GraphNode_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      graph_msgs::msg::GraphNode_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<graph_msgs::msg::GraphNode_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      graph_msgs::msg::GraphNode_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<graph_msgs::msg::GraphNode_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<graph_msgs::msg::GraphNode_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<graph_msgs::msg::GraphNode_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__graph_msgs__msg__GraphNode
    std::shared_ptr<graph_msgs::msg::GraphNode_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__graph_msgs__msg__GraphNode
    std::shared_ptr<graph_msgs::msg::GraphNode_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GraphNode_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->occupied != other.occupied) {
      return false;
    }
    return true;
  }
  bool operator!=(const GraphNode_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GraphNode_

// alias to use template instance with default allocator
using GraphNode =
  graph_msgs::msg::GraphNode_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace graph_msgs

#endif  // GRAPH_MSGS__MSG__DETAIL__GRAPH_NODE__STRUCT_HPP_
