// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from interfaces:msg/JointAngleArray.idl
// generated code does not contain a copyright notice

#include "interfaces/msg/detail/joint_angle_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_interfaces
const rosidl_type_hash_t *
interfaces__msg__JointAngleArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc6, 0x42, 0x63, 0x0f, 0x25, 0xce, 0xca, 0x20,
      0xfe, 0x88, 0x6e, 0xe4, 0x36, 0xaf, 0xd5, 0x1c,
      0x32, 0xf0, 0x0b, 0x71, 0x58, 0x52, 0x6f, 0x2d,
      0xe1, 0xb3, 0x32, 0x68, 0x0b, 0x51, 0x5c, 0x2d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char interfaces__msg__JointAngleArray__TYPE_NAME[] = "interfaces/msg/JointAngleArray";

// Define type names, field names, and default values
static char interfaces__msg__JointAngleArray__FIELD_NAME__angles[] = "angles";

static rosidl_runtime_c__type_description__Field interfaces__msg__JointAngleArray__FIELDS[] = {
  {
    {interfaces__msg__JointAngleArray__FIELD_NAME__angles, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interfaces__msg__JointAngleArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interfaces__msg__JointAngleArray__TYPE_NAME, 30, 30},
      {interfaces__msg__JointAngleArray__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64[6] angles";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
interfaces__msg__JointAngleArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interfaces__msg__JointAngleArray__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 17, 17},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interfaces__msg__JointAngleArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interfaces__msg__JointAngleArray__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
