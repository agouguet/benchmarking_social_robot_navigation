// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from graph_msgs:msg/GraphNav.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSGS__MSG__DETAIL__GRAPH_NAV__STRUCT_HPP_
#define GRAPH_MSGS__MSG__DETAIL__GRAPH_NAV__STRUCT_HPP_

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
// Member 'edges'
#include "graph_msgs/msg/detail/graph_edge__struct.hpp"
// Member 'nodes'
#include "graph_msgs/msg/detail/graph_node__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__graph_msgs__msg__GraphNav __attribute__((deprecated))
#else
# define DEPRECATED__graph_msgs__msg__GraphNav __declspec(deprecated)
#endif

namespace graph_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GraphNav_
{
  using Type = GraphNav_<ContainerAllocator>;

  explicit GraphNav_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit GraphNav_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _edges_type =
    std::vector<graph_msgs::msg::GraphEdge_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<graph_msgs::msg::GraphEdge_<ContainerAllocator>>>;
  _edges_type edges;
  using _nodes_type =
    std::vector<graph_msgs::msg::GraphNode_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<graph_msgs::msg::GraphNode_<ContainerAllocator>>>;
  _nodes_type nodes;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__edges(
    const std::vector<graph_msgs::msg::GraphEdge_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<graph_msgs::msg::GraphEdge_<ContainerAllocator>>> & _arg)
  {
    this->edges = _arg;
    return *this;
  }
  Type & set__nodes(
    const std::vector<graph_msgs::msg::GraphNode_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<graph_msgs::msg::GraphNode_<ContainerAllocator>>> & _arg)
  {
    this->nodes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    graph_msgs::msg::GraphNav_<ContainerAllocator> *;
  using ConstRawPtr =
    const graph_msgs::msg::GraphNav_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<graph_msgs::msg::GraphNav_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<graph_msgs::msg::GraphNav_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      graph_msgs::msg::GraphNav_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<graph_msgs::msg::GraphNav_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      graph_msgs::msg::GraphNav_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<graph_msgs::msg::GraphNav_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<graph_msgs::msg::GraphNav_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<graph_msgs::msg::GraphNav_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__graph_msgs__msg__GraphNav
    std::shared_ptr<graph_msgs::msg::GraphNav_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__graph_msgs__msg__GraphNav
    std::shared_ptr<graph_msgs::msg::GraphNav_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GraphNav_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->edges != other.edges) {
      return false;
    }
    if (this->nodes != other.nodes) {
      return false;
    }
    return true;
  }
  bool operator!=(const GraphNav_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GraphNav_

// alias to use template instance with default allocator
using GraphNav =
  graph_msgs::msg::GraphNav_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace graph_msgs

#endif  // GRAPH_MSGS__MSG__DETAIL__GRAPH_NAV__STRUCT_HPP_
