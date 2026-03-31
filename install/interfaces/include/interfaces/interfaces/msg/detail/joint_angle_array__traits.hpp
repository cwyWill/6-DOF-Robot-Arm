// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/JointAngleArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/joint_angle_array.hpp"


#ifndef INTERFACES__MSG__DETAIL__JOINT_ANGLE_ARRAY__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__JOINT_ANGLE_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/joint_angle_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const JointAngleArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: angles
  {
    if (msg.angles.size() == 0) {
      out << "angles: []";
    } else {
      out << "angles: [";
      size_t pending_items = msg.angles.size();
      for (auto item : msg.angles) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointAngleArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: angles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.angles.size() == 0) {
      out << "angles: []\n";
    } else {
      out << "angles:\n";
      for (auto item : msg.angles) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointAngleArray & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::msg::JointAngleArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::JointAngleArray & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::JointAngleArray>()
{
  return "interfaces::msg::JointAngleArray";
}

template<>
inline const char * name<interfaces::msg::JointAngleArray>()
{
  return "interfaces/msg/JointAngleArray";
}

template<>
struct has_fixed_size<interfaces::msg::JointAngleArray>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::msg::JointAngleArray>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::msg::JointAngleArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__JOINT_ANGLE_ARRAY__TRAITS_HPP_
