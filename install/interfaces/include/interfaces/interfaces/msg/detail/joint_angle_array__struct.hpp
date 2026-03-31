// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/JointAngleArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/joint_angle_array.hpp"


#ifndef INTERFACES__MSG__DETAIL__JOINT_ANGLE_ARRAY__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__JOINT_ANGLE_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__JointAngleArray __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__JointAngleArray __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointAngleArray_
{
  using Type = JointAngleArray_<ContainerAllocator>;

  explicit JointAngleArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 6>::iterator, double>(this->angles.begin(), this->angles.end(), 0.0);
    }
  }

  explicit JointAngleArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : angles(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 6>::iterator, double>(this->angles.begin(), this->angles.end(), 0.0);
    }
  }

  // field types and members
  using _angles_type =
    std::array<double, 6>;
  _angles_type angles;

  // setters for named parameter idiom
  Type & set__angles(
    const std::array<double, 6> & _arg)
  {
    this->angles = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::JointAngleArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::JointAngleArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::JointAngleArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::JointAngleArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::JointAngleArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::JointAngleArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::JointAngleArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::JointAngleArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::JointAngleArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::JointAngleArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__JointAngleArray
    std::shared_ptr<interfaces::msg::JointAngleArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__JointAngleArray
    std::shared_ptr<interfaces::msg::JointAngleArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointAngleArray_ & other) const
  {
    if (this->angles != other.angles) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointAngleArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointAngleArray_

// alias to use template instance with default allocator
using JointAngleArray =
  interfaces::msg::JointAngleArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__JOINT_ANGLE_ARRAY__STRUCT_HPP_
