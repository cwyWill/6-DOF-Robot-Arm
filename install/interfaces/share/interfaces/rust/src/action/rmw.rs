
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_Goal() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__action__JointCommand_Goal__init(msg: *mut JointCommand_Goal) -> bool;
    fn interfaces__action__JointCommand_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_Goal>, size: usize) -> bool;
    fn interfaces__action__JointCommand_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_Goal>);
    fn interfaces__action__JointCommand_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointCommand_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<JointCommand_Goal>) -> bool;
}

// Corresponds to interfaces__action__JointCommand_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointCommand_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub cmd_angle: super::super::msg::rmw::JointAngleArray,

}



impl Default for JointCommand_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__action__JointCommand_Goal__init(&mut msg as *mut _) {
        panic!("Call to interfaces__action__JointCommand_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointCommand_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointCommand_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointCommand_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/action/JointCommand_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_Goal() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_Result() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__action__JointCommand_Result__init(msg: *mut JointCommand_Result) -> bool;
    fn interfaces__action__JointCommand_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_Result>, size: usize) -> bool;
    fn interfaces__action__JointCommand_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_Result>);
    fn interfaces__action__JointCommand_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointCommand_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<JointCommand_Result>) -> bool;
}

// Corresponds to interfaces__action__JointCommand_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointCommand_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub result_angle: super::super::msg::rmw::JointAngleArray,

}



impl Default for JointCommand_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__action__JointCommand_Result__init(&mut msg as *mut _) {
        panic!("Call to interfaces__action__JointCommand_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointCommand_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointCommand_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointCommand_Result where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/action/JointCommand_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_Result() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__action__JointCommand_Feedback__init(msg: *mut JointCommand_Feedback) -> bool;
    fn interfaces__action__JointCommand_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_Feedback>, size: usize) -> bool;
    fn interfaces__action__JointCommand_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_Feedback>);
    fn interfaces__action__JointCommand_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointCommand_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<JointCommand_Feedback>) -> bool;
}

// Corresponds to interfaces__action__JointCommand_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointCommand_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub fb_angle: super::super::msg::rmw::JointAngleArray,

}



impl Default for JointCommand_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__action__JointCommand_Feedback__init(&mut msg as *mut _) {
        panic!("Call to interfaces__action__JointCommand_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointCommand_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointCommand_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointCommand_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/action/JointCommand_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_Feedback() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__action__JointCommand_FeedbackMessage__init(msg: *mut JointCommand_FeedbackMessage) -> bool;
    fn interfaces__action__JointCommand_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_FeedbackMessage>, size: usize) -> bool;
    fn interfaces__action__JointCommand_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_FeedbackMessage>);
    fn interfaces__action__JointCommand_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointCommand_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<JointCommand_FeedbackMessage>) -> bool;
}

// Corresponds to interfaces__action__JointCommand_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointCommand_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::JointCommand_Feedback,

}



impl Default for JointCommand_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__action__JointCommand_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to interfaces__action__JointCommand_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointCommand_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointCommand_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointCommand_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/action/JointCommand_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_FeedbackMessage() }
  }
}




#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__action__JointCommand_SendGoal_Request__init(msg: *mut JointCommand_SendGoal_Request) -> bool;
    fn interfaces__action__JointCommand_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_SendGoal_Request>, size: usize) -> bool;
    fn interfaces__action__JointCommand_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_SendGoal_Request>);
    fn interfaces__action__JointCommand_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointCommand_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<JointCommand_SendGoal_Request>) -> bool;
}

// Corresponds to interfaces__action__JointCommand_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointCommand_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::JointCommand_Goal,

}



impl Default for JointCommand_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__action__JointCommand_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to interfaces__action__JointCommand_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointCommand_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointCommand_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointCommand_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/action/JointCommand_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_SendGoal_Request() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__action__JointCommand_SendGoal_Response__init(msg: *mut JointCommand_SendGoal_Response) -> bool;
    fn interfaces__action__JointCommand_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_SendGoal_Response>, size: usize) -> bool;
    fn interfaces__action__JointCommand_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_SendGoal_Response>);
    fn interfaces__action__JointCommand_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointCommand_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<JointCommand_SendGoal_Response>) -> bool;
}

// Corresponds to interfaces__action__JointCommand_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointCommand_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for JointCommand_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__action__JointCommand_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to interfaces__action__JointCommand_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointCommand_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointCommand_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointCommand_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/action/JointCommand_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_SendGoal_Response() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__action__JointCommand_GetResult_Request__init(msg: *mut JointCommand_GetResult_Request) -> bool;
    fn interfaces__action__JointCommand_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_GetResult_Request>, size: usize) -> bool;
    fn interfaces__action__JointCommand_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_GetResult_Request>);
    fn interfaces__action__JointCommand_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointCommand_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<JointCommand_GetResult_Request>) -> bool;
}

// Corresponds to interfaces__action__JointCommand_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointCommand_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for JointCommand_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__action__JointCommand_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to interfaces__action__JointCommand_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointCommand_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointCommand_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointCommand_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/action/JointCommand_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_GetResult_Request() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__action__JointCommand_GetResult_Response__init(msg: *mut JointCommand_GetResult_Response) -> bool;
    fn interfaces__action__JointCommand_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_GetResult_Response>, size: usize) -> bool;
    fn interfaces__action__JointCommand_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointCommand_GetResult_Response>);
    fn interfaces__action__JointCommand_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointCommand_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<JointCommand_GetResult_Response>) -> bool;
}

// Corresponds to interfaces__action__JointCommand_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointCommand_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::JointCommand_Result,

}



impl Default for JointCommand_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__action__JointCommand_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to interfaces__action__JointCommand_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointCommand_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__action__JointCommand_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointCommand_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointCommand_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/action/JointCommand_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__action__JointCommand_GetResult_Response() }
  }
}






#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__interfaces__action__JointCommand_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to interfaces__action__JointCommand_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct JointCommand_SendGoal;

impl rosidl_runtime_rs::Service for JointCommand_SendGoal {
    type Request = JointCommand_SendGoal_Request;
    type Response = JointCommand_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__interfaces__action__JointCommand_SendGoal() }
    }
}




#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__interfaces__action__JointCommand_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to interfaces__action__JointCommand_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct JointCommand_GetResult;

impl rosidl_runtime_rs::Service for JointCommand_GetResult {
    type Request = JointCommand_GetResult_Request;
    type Response = JointCommand_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__interfaces__action__JointCommand_GetResult() }
    }
}


