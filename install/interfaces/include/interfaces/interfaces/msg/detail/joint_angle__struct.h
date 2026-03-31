// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/JointAngle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/joint_angle.h"


#ifndef INTERFACES__MSG__DETAIL__JOINT_ANGLE__STRUCT_H_
#define INTERFACES__MSG__DETAIL__JOINT_ANGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/JointAngle in the package interfaces.
typedef struct interfaces__msg__JointAngle
{
  double angle;
} interfaces__msg__JointAngle;

// Struct for a sequence of interfaces__msg__JointAngle.
typedef struct interfaces__msg__JointAngle__Sequence
{
  interfaces__msg__JointAngle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__JointAngle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__JOINT_ANGLE__STRUCT_H_
