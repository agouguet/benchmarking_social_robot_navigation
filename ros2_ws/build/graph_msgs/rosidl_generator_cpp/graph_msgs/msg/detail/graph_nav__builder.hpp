// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from graph_msgs:msg/GraphNav.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSGS__MSG__DETAIL__GRAPH_NAV__BUILDER_HPP_
#define GRAPH_MSGS__MSG__DETAIL__GRAPH_NAV__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "graph_msgs/msg/detail/graph_nav__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace graph_msgs
{

namespace msg
{

namespace builder
{

class Init_GraphNav_nodes
{
public:
  explicit Init_GraphNav_nodes(::graph_msgs::msg::GraphNav & msg)
  : msg_(msg)
  {}
  ::graph_msgs::msg::GraphNav nodes(::graph_msgs::msg::GraphNav::_nodes_type arg)
  {
    msg_.nodes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::graph_msgs::msg::GraphNav msg_;
};

class Init_GraphNav_edges
{
public:
  explicit Init_GraphNav_edges(::graph_msgs::msg::GraphNav & msg)
  : msg_(msg)
  {}
  Init_GraphNav_nodes edges(::graph_msgs::msg::GraphNav::_edges_type arg)
  {
    msg_.edges = std::move(arg);
    return Init_GraphNav_nodes(msg_);
  }

private:
  ::graph_msgs::msg::GraphNav msg_;
};

class Init_GraphNav_header
{
public:
  Init_GraphNav_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GraphNav_edges header(::graph_msgs::msg::GraphNav::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GraphNav_edges(msg_);
  }

private:
  ::graph_msgs::msg::GraphNav msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::graph_msgs::msg::GraphNav>()
{
  return graph_msgs::msg::builder::Init_GraphNav_header();
}

}  // namespace graph_msgs

#endif  // GRAPH_MSGS__MSG__DETAIL__GRAPH_NAV__BUILDER_HPP_
