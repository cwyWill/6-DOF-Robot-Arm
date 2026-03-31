// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/JointAngle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/joint_angle.hpp"


#ifndef INTERFACES__MSG__DETAIL__JOINT_ANGLE__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__JOINT_ANGLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/joint_angle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_JointAngle_angle
{
public:
  Init_JointAngle_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::msg::JointAngle angle(::interfaces::msg::JointAngle::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::JointAngle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::JointAngle>()
{
  return interfaces::msg::builder::Init_JointAngle_angle();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__JOINT_ANGLE__BUILDER_HPP_
