#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to interfaces__msg__JointAngle

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointAngle {

    // This member is not documented.
    #[allow(missing_docs)]
    pub angle: f64,

}



impl Default for JointAngle {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::JointAngle::default())
  }
}

impl rosidl_runtime_rs::Message for JointAngle {
  type RmwMsg = super::msg::rmw::JointAngle;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        angle: msg.angle,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      angle: msg.angle,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      angle: msg.angle,
    }
  }
}


// Corresponds to interfaces__msg__JointAngleArray

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct JointAngleArray {

    // This member is not documented.
    #[allow(missing_docs)]
    pub angles: [f64; 6],

}



impl Default for JointAngleArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::JointAngleArray::default())
  }
}

impl rosidl_runtime_rs::Message for JointAngleArray {
  type RmwMsg = super::msg::rmw::JointAngleArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        angles: msg.angles,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        angles: msg.angles,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      angles: msg.angles,
    }
  }
}


