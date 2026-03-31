// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from interfaces:msg/JointAngle.idl
// generated code does not contain a copyright notice

#include "interfaces/msg/detail/joint_angle__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_interfaces
const rosidl_type_hash_t *
interfaces__msg__JointAngle__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbe, 0x99, 0x94, 0xbe, 0xb8, 0x6a, 0x44, 0x95,
      0x40, 0xbb, 0x6b, 0x40, 0xe9, 0x2e, 0x93, 0xde,
      0x21, 0x7a, 0x33, 0xc2, 0x8a, 0xf4, 0xc4, 0x18,
      0xa2, 0x99, 0x29, 0x84, 0x0e, 0x21, 0xeb, 0x8a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char interfaces__msg__JointAngle__TYPE_NAME[] = "interfaces/msg/JointAngle";

// Define type names, field names, and default values
static char interfaces__msg__JointAngle__FIELD_NAME__angle[] = "angle";

static rosidl_runtime_c__type_description__Field interfaces__msg__JointAngle__FIELDS[] = {
  {
    {interfaces__msg__JointAngle__FIELD_NAME__angle, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interfaces__msg__JointAngle__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interfaces__msg__JointAngle__TYPE_NAME, 25, 25},
      {interfaces__msg__JointAngle__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 angle";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
interfaces__msg__JointAngle__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interfaces__msg__JointAngle__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 13, 13},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interfaces__msg__JointAngle__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interfaces__msg__JointAngle__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
