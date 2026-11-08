//! AUBO i-series arms on the ARCS controller, without ROS and without
//! AUBO's binary SDK.
//!
//! ARCS, the controller software of AUBO's current i-series arms, serves the
//! interfaces of AUBO's own SDK (`aubo_sdk`, `pyaubo_sdk`) over the network:
//!
//! | Port | Interface | Used here |
//! |---|---|---|
//! | TCP 30004 (HTTP and WebSocket on 9012) | JSON-RPC 2.0: every SDK call, as `rob1.MotionControl.moveJoint` and so on | commands and queries ([`rpc`]) |
//! | TCP 30010 (WebSocket on 9013) | RTDE: JSON arrays, a subscription per channel at a chosen rate | joint state at 200 Hz ([`rtde`]) |
//! | TCP 30002 | script (Lua) | not used |
//!
//! [`AuboArm`] speaks the first two. Commands are SDK calls on JSON-RPC:
//! power and brakes through `RobotManage`, point-to-point motion through
//! `MotionControl.moveJoint`, streaming through the servo mode of
//! `MotionControl.setServoModeSelect` and `servoJoint`. The joint state comes
//! from an RTDE subscription, one data packet per 5 ms controller cycle, and
//! paces every operation.
//!
//! The older AUBO controllers (OurControl, the `libauborobotcontroller`
//! library of AUBO's ROS 1 driver) use a different, undocumented binary
//! protocol and are not supported.
//!
//! ```no_run
//! use libaubo::{AuboI5, AuboOptions};
//! use roplat_behavior::{ControlWith, JointPositionControl, JointSpace, MoveTo, Robot};
//!
//! let mut arm = AuboI5::connect([192, 168, 1, 100].into(), AuboOptions::default())?;
//! arm.enable()?; // power on and release the brakes
//! let home = [0.0, -0.2618, 1.7453, 0.4363, 1.5708, 0.0];
//! <AuboI5 as MoveTo<JointSpace<6>>>::move_to(&mut arm, home)?;
//! let mut t = 0.0;
//! <AuboI5 as ControlWith<JointPositionControl<6>>>::control_with(&mut arm, |_, dt| {
//!     t += dt.as_secs_f64();
//!     let mut q = home;
//!     q[0] += 0.1 * (t * 1.5).sin();
//!     (q, t >= 4.0)
//! })?;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! The README lists the source of every constant and what has not been
//! checked on a controller.

mod driver;
mod error;
mod json;
pub mod model;
pub mod rpc;
pub mod rtde;
pub mod status;

pub use driver::{
    AuboArm, AuboOptions, AuboState, CONTROL_PERIOD, RECIPE, SERVO_GAIN, SERVO_LOOKAHEAD_TIME,
    SERVO_MODE,
};
pub use error::AuboError;
pub use json::Param;
pub use model::{AuboModel, Dh, I3, I5, I10};
pub use status::{ReturnCode, RobotMode, SafetyMode};

/// An AUBO i3 on ARCS.
pub type AuboI3 = AuboArm<I3>;
/// An AUBO i5 on ARCS.
pub type AuboI5 = AuboArm<I5>;
/// An AUBO i10 on ARCS.
pub type AuboI10 = AuboArm<I10>;
