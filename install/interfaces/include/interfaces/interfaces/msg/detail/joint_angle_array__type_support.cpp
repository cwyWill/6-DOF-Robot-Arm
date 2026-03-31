// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from interfaces:msg/JointAngleArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "interfaces/msg/detail/joint_angle_array__functions.h"
#include "interfaces/msg/detail/joint_angle_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void JointAngleArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) interfaces::msg::JointAngleArray(_init);
}

void JointAngleArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<interfaces::msg::JointAngleArray *>(message_memory);
  typed_message->~JointAngleArray();
}

size_t size_function__JointAngleArray__angles(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__JointAngleArray__angles(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__JointAngleArray__angles(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointAngleArray__angles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointAngleArray__angles(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointAngleArray__angles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointAngleArray__angles(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember JointAngleArray_message_member_array[1] = {
  {
    "angles",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(interfaces::msg::JointAngleArray, angles),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointAngleArray__angles,  // size() function pointer
    get_const_function__JointAngleArray__angles,  // get_const(index) function pointer
    get_function__JointAngleArray__angles,  // get(index) function pointer
    fetch_function__JointAngleArray__angles,  // fetch(index, &value) function pointer
    assign_function__JointAngleArray__angles,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers JointAngleArray_message_members = {
  "interfaces::msg",  // message namespace
  "JointAngleArray",  // message name
  1,  // number of fields
  sizeof(interfaces::msg::JointAngleArray),
  false,  // has_any_key_member_
  JointAngleArray_message_member_array,  // message members
  JointAngleArray_init_function,  // function to initialize message memory (memory has to be allocated)
  JointAngleArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t JointAngleArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &JointAngleArray_message_members,
  get_message_typesupport_handle_function,
  &interfaces__msg__JointAngleArray__get_type_hash,
  &interfaces__msg__JointAngleArray__get_type_description,
  &interfaces__msg__JointAngleArray__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<interfaces::msg::JointAngleArray>()
{
  return &::interfaces::msg::rosidl_typesupport_introspection_cpp::JointAngleArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, interfaces, msg, JointAngleArray)() {
  return &::interfaces::msg::rosidl_typesupport_introspection_cpp::JointAngleArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
