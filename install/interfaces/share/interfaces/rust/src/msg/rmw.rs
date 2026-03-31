#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__JointAngle() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__JointAngle__init(msg: *mut JointAngle) -> bool;
    fn interfaces__msg__JointAngle__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointAngle>, size: usize) -> bool;
    fn interfaces__msg__JointAngle__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointAngle>);
    fn interfaces__msg__JointAngle__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointAngle>, out_seq: *mut rosidl_runtime_rs::Sequence<JointAngle>) -> bool;
}

// Corresponds to interfaces__msg__JointAngle
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointAngle {

    // This member is not documented.
    #[allow(missing_docs)]
    pub angle: f64,

}



impl Default for JointAngle {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__JointAngle__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__JointAngle__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointAngle {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__JointAngle__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__JointAngle__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__JointAngle__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointAngle {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointAngle where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/JointAngle";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__JointAngle() }
  }
}


#[link(name = "interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__JointAngleArray() -> *const std::ffi::c_void;
}

#[link(name = "interfaces__rosidl_generator_c")]
extern "C" {
    fn interfaces__msg__JointAngleArray__init(msg: *mut JointAngleArray) -> bool;
    fn interfaces__msg__JointAngleArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<JointAngleArray>, size: usize) -> bool;
    fn interfaces__msg__JointAngleArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<JointAngleArray>);
    fn interfaces__msg__JointAngleArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<JointAngleArray>, out_seq: *mut rosidl_runtime_rs::Sequence<JointAngleArray>) -> bool;
}

// Corresponds to interfaces__msg__JointAngleArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointAngleArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub angles: [f64; 6],

}



impl Default for JointAngleArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !interfaces__msg__JointAngleArray__init(&mut msg as *mut _) {
        panic!("Call to interfaces__msg__JointAngleArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for JointAngleArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__JointAngleArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__JointAngleArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { interfaces__msg__JointAngleArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for JointAngleArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for JointAngleArray where Self: Sized {
  const TYPE_NAME: &'static str = "interfaces/msg/JointAngleArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__interfaces__msg__JointAngleArray() }
  }
}


