// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/JointAngleArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/joint_angle_array.h"


#ifndef INTERFACES__MSG__DETAIL__JOINT_ANGLE_ARRAY__STRUCT_H_
#define INTERFACES__MSG__DETAIL__JOINT_ANGLE_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/JointAngleArray in the package interfaces.
typedef struct interfaces__msg__JointAngleArray
{
  double angles[6];
} interfaces__msg__JointAngleArray;

// Struct for a sequence of interfaces__msg__JointAngleArray.
typedef struct interfaces__msg__JointAngleArray__Sequence
{
  interfaces__msg__JointAngleArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__JointAngleArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__JOINT_ANGLE_ARRAY__STRUCT_H_
