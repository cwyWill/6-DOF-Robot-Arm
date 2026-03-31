// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:action/JointCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/action/joint_command.h"


#ifndef INTERFACES__ACTION__DETAIL__JOINT_COMMAND__STRUCT_H_
#define INTERFACES__ACTION__DETAIL__JOINT_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'cmd_angle'
#include "interfaces/msg/detail/joint_angle_array__struct.h"

/// Struct defined in action/JointCommand in the package interfaces.
typedef struct interfaces__action__JointCommand_Goal
{
  interfaces__msg__JointAngleArray cmd_angle;
} interfaces__action__JointCommand_Goal;

// Struct for a sequence of interfaces__action__JointCommand_Goal.
typedef struct interfaces__action__JointCommand_Goal__Sequence
{
  interfaces__action__JointCommand_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__JointCommand_Goal__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result_angle'
// already included above
// #include "interfaces/msg/detail/joint_angle_array__struct.h"

/// Struct defined in action/JointCommand in the package interfaces.
typedef struct interfaces__action__JointCommand_Result
{
  interfaces__msg__JointAngleArray result_angle;
} interfaces__action__JointCommand_Result;

// Struct for a sequence of interfaces__action__JointCommand_Result.
typedef struct interfaces__action__JointCommand_Result__Sequence
{
  interfaces__action__JointCommand_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__JointCommand_Result__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'fb_angle'
// already included above
// #include "interfaces/msg/detail/joint_angle_array__struct.h"

/// Struct defined in action/JointCommand in the package interfaces.
typedef struct interfaces__action__JointCommand_Feedback
{
  interfaces__msg__JointAngleArray fb_angle;
} interfaces__action__JointCommand_Feedback;

// Struct for a sequence of interfaces__action__JointCommand_Feedback.
typedef struct interfaces__action__JointCommand_Feedback__Sequence
{
  interfaces__action__JointCommand_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__JointCommand_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "interfaces/action/detail/joint_command__struct.h"

/// Struct defined in action/JointCommand in the package interfaces.
typedef struct interfaces__action__JointCommand_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  interfaces__action__JointCommand_Goal goal;
} interfaces__action__JointCommand_SendGoal_Request;

// Struct for a sequence of interfaces__action__JointCommand_SendGoal_Request.
typedef struct interfaces__action__JointCommand_SendGoal_Request__Sequence
{
  interfaces__action__JointCommand_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__JointCommand_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/JointCommand in the package interfaces.
typedef struct interfaces__action__JointCommand_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} interfaces__action__JointCommand_SendGoal_Response;

// Struct for a sequence of interfaces__action__JointCommand_SendGoal_Response.
typedef struct interfaces__action__JointCommand_SendGoal_Response__Sequence
{
  interfaces__action__JointCommand_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__JointCommand_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  interfaces__action__JointCommand_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  interfaces__action__JointCommand_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/JointCommand in the package interfaces.
typedef struct interfaces__action__JointCommand_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  interfaces__action__JointCommand_SendGoal_Request__Sequence request;
  interfaces__action__JointCommand_SendGoal_Response__Sequence response;
} interfaces__action__JointCommand_SendGoal_Event;

// Struct for a sequence of interfaces__action__JointCommand_SendGoal_Event.
typedef struct interfaces__action__JointCommand_SendGoal_Event__Sequence
{
  interfaces__action__JointCommand_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__JointCommand_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/JointCommand in the package interfaces.
typedef struct interfaces__action__JointCommand_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} interfaces__action__JointCommand_GetResult_Request;

// Struct for a sequence of interfaces__action__JointCommand_GetResult_Request.
typedef struct interfaces__action__JointCommand_GetResult_Request__Sequence
{
  interfaces__action__JointCommand_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__JointCommand_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "interfaces/action/detail/joint_command__struct.h"

/// Struct defined in action/JointCommand in the package interfaces.
typedef struct interfaces__action__JointCommand_GetResult_Response
{
  int8_t status;
  interfaces__action__JointCommand_Result result;
} interfaces__action__JointCommand_GetResult_Response;

// Struct for a sequence of interfaces__action__JointCommand_GetResult_Response.
typedef struct interfaces__action__JointCommand_GetResult_Response__Sequence
{
  interfaces__action__JointCommand_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__JointCommand_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  interfaces__action__JointCommand_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  interfaces__action__JointCommand_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/JointCommand in the package interfaces.
typedef struct interfaces__action__JointCommand_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  interfaces__action__JointCommand_GetResult_Request__Sequence request;
  interfaces__action__JointCommand_GetResult_Response__Sequence response;
} interfaces__action__JointCommand_GetResult_Event;

// Struct for a sequence of interfaces__action__JointCommand_GetResult_Event.
typedef struct interfaces__action__JointCommand_GetResult_Event__Sequence
{
  interfaces__action__JointCommand_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__JointCommand_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "interfaces/action/detail/joint_command__struct.h"

/// Struct defined in action/JointCommand in the package interfaces.
typedef struct interfaces__action__JointCommand_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  interfaces__action__JointCommand_Feedback feedback;
} interfaces__action__JointCommand_FeedbackMessage;

// Struct for a sequence of interfaces__action__JointCommand_FeedbackMessage.
typedef struct interfaces__action__JointCommand_FeedbackMessage__Sequence
{
  interfaces__action__JointCommand_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__JointCommand_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__ACTION__DETAIL__JOINT_COMMAND__STRUCT_H_
