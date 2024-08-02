// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from simulation_msgs:msg/RealDepthImage.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__REAL_DEPTH_IMAGE__BUILDER_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__REAL_DEPTH_IMAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "simulation_msgs/msg/detail/real_depth_image__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace simulation_msgs
{

namespace msg
{

namespace builder
{

class Init_RealDepthImage_data
{
public:
  explicit Init_RealDepthImage_data(::simulation_msgs::msg::RealDepthImage & msg)
  : msg_(msg)
  {}
  ::simulation_msgs::msg::RealDepthImage data(::simulation_msgs::msg::RealDepthImage::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::simulation_msgs::msg::RealDepthImage msg_;
};

class Init_RealDepthImage_header
{
public:
  Init_RealDepthImage_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RealDepthImage_data header(::simulation_msgs::msg::RealDepthImage::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RealDepthImage_data(msg_);
  }

private:
  ::simulation_msgs::msg::RealDepthImage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::simulation_msgs::msg::RealDepthImage>()
{
  return simulation_msgs::msg::builder::Init_RealDepthImage_header();
}

}  // namespace simulation_msgs

#endif  // SIMULATION_MSGS__MSG__DETAIL__REAL_DEPTH_IMAGE__BUILDER_HPP_
