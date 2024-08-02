// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from simulation_msgs:msg/SceneInfo.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__SCENE_INFO__STRUCT_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__SCENE_INFO__STRUCT_HPP_

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
// Member 'robot_start_pose'
// Member 'robot_target_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__simulation_msgs__msg__SceneInfo __attribute__((deprecated))
#else
# define DEPRECATED__simulation_msgs__msg__SceneInfo __declspec(deprecated)
#endif

namespace simulation_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SceneInfo_
{
  using Type = SceneInfo_<ContainerAllocator>;

  explicit SceneInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    robot_start_pose(_init),
    robot_target_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->scenario_name = "";
      this->num_people = 0;
      this->num_groups = 0;
      this->environment = "";
    }
  }

  explicit SceneInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    scenario_name(_alloc),
    robot_start_pose(_alloc, _init),
    robot_target_pose(_alloc, _init),
    environment(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->scenario_name = "";
      this->num_people = 0;
      this->num_groups = 0;
      this->environment = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _scenario_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _scenario_name_type scenario_name;
  using _robot_start_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _robot_start_pose_type robot_start_pose;
  using _robot_target_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _robot_target_pose_type robot_target_pose;
  using _num_people_type =
    uint16_t;
  _num_people_type num_people;
  using _num_groups_type =
    uint16_t;
  _num_groups_type num_groups;
  using _environment_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _environment_type environment;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__scenario_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->scenario_name = _arg;
    return *this;
  }
  Type & set__robot_start_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->robot_start_pose = _arg;
    return *this;
  }
  Type & set__robot_target_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->robot_target_pose = _arg;
    return *this;
  }
  Type & set__num_people(
    const uint16_t & _arg)
  {
    this->num_people = _arg;
    return *this;
  }
  Type & set__num_groups(
    const uint16_t & _arg)
  {
    this->num_groups = _arg;
    return *this;
  }
  Type & set__environment(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->environment = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    simulation_msgs::msg::SceneInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const simulation_msgs::msg::SceneInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<simulation_msgs::msg::SceneInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<simulation_msgs::msg::SceneInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      simulation_msgs::msg::SceneInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<simulation_msgs::msg::SceneInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      simulation_msgs::msg::SceneInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<simulation_msgs::msg::SceneInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<simulation_msgs::msg::SceneInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<simulation_msgs::msg::SceneInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__simulation_msgs__msg__SceneInfo
    std::shared_ptr<simulation_msgs::msg::SceneInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__simulation_msgs__msg__SceneInfo
    std::shared_ptr<simulation_msgs::msg::SceneInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SceneInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->scenario_name != other.scenario_name) {
      return false;
    }
    if (this->robot_start_pose != other.robot_start_pose) {
      return false;
    }
    if (this->robot_target_pose != other.robot_target_pose) {
      return false;
    }
    if (this->num_people != other.num_people) {
      return false;
    }
    if (this->num_groups != other.num_groups) {
      return false;
    }
    if (this->environment != other.environment) {
      return false;
    }
    return true;
  }
  bool operator!=(const SceneInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SceneInfo_

// alias to use template instance with default allocator
using SceneInfo =
  simulation_msgs::msg::SceneInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace simulation_msgs

#endif  // SIMULATION_MSGS__MSG__DETAIL__SCENE_INFO__STRUCT_HPP_
