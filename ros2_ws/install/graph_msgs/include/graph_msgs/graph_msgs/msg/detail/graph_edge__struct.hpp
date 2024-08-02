// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from graph_msgs:msg/GraphEdge.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__STRUCT_HPP_
#define GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__graph_msgs__msg__GraphEdge __attribute__((deprecated))
#else
# define DEPRECATED__graph_msgs__msg__GraphEdge __declspec(deprecated)
#endif

namespace graph_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GraphEdge_
{
  using Type = GraphEdge_<ContainerAllocator>;

  explicit GraphEdge_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id_n1 = 0ull;
      this->id_n2 = 0ull;
    }
  }

  explicit GraphEdge_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id_n1 = 0ull;
      this->id_n2 = 0ull;
    }
  }

  // field types and members
  using _id_n1_type =
    uint64_t;
  _id_n1_type id_n1;
  using _id_n2_type =
    uint64_t;
  _id_n2_type id_n2;

  // setters for named parameter idiom
  Type & set__id_n1(
    const uint64_t & _arg)
  {
    this->id_n1 = _arg;
    return *this;
  }
  Type & set__id_n2(
    const uint64_t & _arg)
  {
    this->id_n2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    graph_msgs::msg::GraphEdge_<ContainerAllocator> *;
  using ConstRawPtr =
    const graph_msgs::msg::GraphEdge_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<graph_msgs::msg::GraphEdge_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<graph_msgs::msg::GraphEdge_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      graph_msgs::msg::GraphEdge_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<graph_msgs::msg::GraphEdge_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      graph_msgs::msg::GraphEdge_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<graph_msgs::msg::GraphEdge_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<graph_msgs::msg::GraphEdge_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<graph_msgs::msg::GraphEdge_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__graph_msgs__msg__GraphEdge
    std::shared_ptr<graph_msgs::msg::GraphEdge_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__graph_msgs__msg__GraphEdge
    std::shared_ptr<graph_msgs::msg::GraphEdge_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GraphEdge_ & other) const
  {
    if (this->id_n1 != other.id_n1) {
      return false;
    }
    if (this->id_n2 != other.id_n2) {
      return false;
    }
    return true;
  }
  bool operator!=(const GraphEdge_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GraphEdge_

// alias to use template instance with default allocator
using GraphEdge =
  graph_msgs::msg::GraphEdge_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace graph_msgs

#endif  // GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__STRUCT_HPP_
