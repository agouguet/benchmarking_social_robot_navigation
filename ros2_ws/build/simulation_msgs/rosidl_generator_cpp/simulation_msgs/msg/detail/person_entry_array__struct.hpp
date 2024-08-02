// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from simulation_msgs:msg/PersonEntryArray.idl
// generated code does not contain a copyright notice

#ifndef SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY_ARRAY__STRUCT_HPP_
#define SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY_ARRAY__STRUCT_HPP_

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
// Member 'people'
#include "simulation_msgs/msg/detail/person_entry__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__simulation_msgs__msg__PersonEntryArray __attribute__((deprecated))
#else
# define DEPRECATED__simulation_msgs__msg__PersonEntryArray __declspec(deprecated)
#endif

namespace simulation_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PersonEntryArray_
{
  using Type = PersonEntryArray_<ContainerAllocator>;

  explicit PersonEntryArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit PersonEntryArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _people_type =
    std::vector<simulation_msgs::msg::PersonEntry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<simulation_msgs::msg::PersonEntry_<ContainerAllocator>>>;
  _people_type people;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__people(
    const std::vector<simulation_msgs::msg::PersonEntry_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<simulation_msgs::msg::PersonEntry_<ContainerAllocator>>> & _arg)
  {
    this->people = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    simulation_msgs::msg::PersonEntryArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const simulation_msgs::msg::PersonEntryArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<simulation_msgs::msg::PersonEntryArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<simulation_msgs::msg::PersonEntryArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      simulation_msgs::msg::PersonEntryArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<simulation_msgs::msg::PersonEntryArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      simulation_msgs::msg::PersonEntryArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<simulation_msgs::msg::PersonEntryArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<simulation_msgs::msg::PersonEntryArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<simulation_msgs::msg::PersonEntryArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__simulation_msgs__msg__PersonEntryArray
    std::shared_ptr<simulation_msgs::msg::PersonEntryArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__simulation_msgs__msg__PersonEntryArray
    std::shared_ptr<simulation_msgs::msg::PersonEntryArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PersonEntryArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->people != other.people) {
      return false;
    }
    return true;
  }
  bool operator!=(const PersonEntryArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PersonEntryArray_

// alias to use template instance with default allocator
using PersonEntryArray =
  simulation_msgs::msg::PersonEntryArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace simulation_msgs

#endif  // SIMULATION_MSGS__MSG__DETAIL__PERSON_ENTRY_ARRAY__STRUCT_HPP_
