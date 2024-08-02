// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from graph_msgs:msg/GraphNode.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSGS__MSG__DETAIL__GRAPH_NODE__BUILDER_HPP_
#define GRAPH_MSGS__MSG__DETAIL__GRAPH_NODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "graph_msgs/msg/detail/graph_node__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace graph_msgs
{

namespace msg
{

namespace builder
{

class Init_GraphNode_occupied
{
public:
  explicit Init_GraphNode_occupied(::graph_msgs::msg::GraphNode & msg)
  : msg_(msg)
  {}
  ::graph_msgs::msg::GraphNode occupied(::graph_msgs::msg::GraphNode::_occupied_type arg)
  {
    msg_.occupied = std::move(arg);
    return std::move(msg_);
  }

private:
  ::graph_msgs::msg::GraphNode msg_;
};

class Init_GraphNode_y
{
public:
  explicit Init_GraphNode_y(::graph_msgs::msg::GraphNode & msg)
  : msg_(msg)
  {}
  Init_GraphNode_occupied y(::graph_msgs::msg::GraphNode::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_GraphNode_occupied(msg_);
  }

private:
  ::graph_msgs::msg::GraphNode msg_;
};

class Init_GraphNode_x
{
public:
  explicit Init_GraphNode_x(::graph_msgs::msg::GraphNode & msg)
  : msg_(msg)
  {}
  Init_GraphNode_y x(::graph_msgs::msg::GraphNode::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_GraphNode_y(msg_);
  }

private:
  ::graph_msgs::msg::GraphNode msg_;
};

class Init_GraphNode_id
{
public:
  Init_GraphNode_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GraphNode_x id(::graph_msgs::msg::GraphNode::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_GraphNode_x(msg_);
  }

private:
  ::graph_msgs::msg::GraphNode msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::graph_msgs::msg::GraphNode>()
{
  return graph_msgs::msg::builder::Init_GraphNode_id();
}

}  // namespace graph_msgs

#endif  // GRAPH_MSGS__MSG__DETAIL__GRAPH_NODE__BUILDER_HPP_
