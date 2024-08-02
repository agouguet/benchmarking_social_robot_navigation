// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from graph_msgs:msg/GraphEdge.idl
// generated code does not contain a copyright notice

#ifndef GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__BUILDER_HPP_
#define GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "graph_msgs/msg/detail/graph_edge__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace graph_msgs
{

namespace msg
{

namespace builder
{

class Init_GraphEdge_id_n2
{
public:
  explicit Init_GraphEdge_id_n2(::graph_msgs::msg::GraphEdge & msg)
  : msg_(msg)
  {}
  ::graph_msgs::msg::GraphEdge id_n2(::graph_msgs::msg::GraphEdge::_id_n2_type arg)
  {
    msg_.id_n2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::graph_msgs::msg::GraphEdge msg_;
};

class Init_GraphEdge_id_n1
{
public:
  Init_GraphEdge_id_n1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GraphEdge_id_n2 id_n1(::graph_msgs::msg::GraphEdge::_id_n1_type arg)
  {
    msg_.id_n1 = std::move(arg);
    return Init_GraphEdge_id_n2(msg_);
  }

private:
  ::graph_msgs::msg::GraphEdge msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::graph_msgs::msg::GraphEdge>()
{
  return graph_msgs::msg::builder::Init_GraphEdge_id_n1();
}

}  // namespace graph_msgs

#endif  // GRAPH_MSGS__MSG__DETAIL__GRAPH_EDGE__BUILDER_HPP_
