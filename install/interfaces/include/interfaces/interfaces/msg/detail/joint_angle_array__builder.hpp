// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/JointAngleArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/joint_angle_array.hpp"


#ifndef INTERFACES__MSG__DETAIL__JOINT_ANGLE_ARRAY__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__JOINT_ANGLE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/joint_angle_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_JointAngleArray_angles
{
public:
  Init_JointAngleArray_angles()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::msg::JointAngleArray angles(::interfaces::msg::JointAngleArray::_angles_type arg)
  {
    msg_.angles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::JointAngleArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::JointAngleArray>()
{
  return interfaces::msg::builder::Init_JointAngleArray_angles();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__JOINT_ANGLE_ARRAY__BUILDER_HPP_
