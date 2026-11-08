//! Why a driver operation failed.

use std::{fmt, io};

use roplat_behavior::RobotException;

use crate::{ReturnCode, RobotMode, SafetyMode};

/// Why an [`AuboArm`](crate::AuboArm) operation failed.
#[derive(Debug)]
pub enum AuboError {
    /// A connection failed.
    Io(io::Error),
    /// The controller closed a connection.
    Closed,
    /// Nothing arrived in time.
    Timeout { waiting_for: &'static str },
    /// Bytes or a JSON value that are not what the protocol sends.
    Malformed { what: &'static str },
    /// The controller answered a call with a JSON-RPC error object: an
    /// unknown method (`-32601`), parameters it does not take (`-32602`),
    /// or a failure inside the call.
    Rpc {
        method: &'static str,
        code: i64,
        /// The error's `message`, as the controller wrote it.
        message: String,
    },
    /// A command call returned a code other than 0.
    Refused {
        method: &'static str,
        code: ReturnCode,
    },
    /// `getRobotNames` named no robot.
    NoRobot,
    /// The controller's arm does not have six joints.
    Dof(i64),
    /// The controller's servo cycle (`getCycletime`, in seconds) is not the
    /// 5 ms [`CONTROL_PERIOD`](crate::CONTROL_PERIOD).
    CycleTime(f64),
    /// The arm cannot move: not running with its brakes released, or held
    /// by the safety system.
    Halted {
        robot_mode: RobotMode,
        safety_mode: SafetyMode,
    },
    /// The arm did not reach the robot mode a power or brake change waits
    /// for within [`AuboOptions::power_timeout`](crate::AuboOptions).
    ModeTimeout {
        waiting_for: RobotMode,
        robot_mode: RobotMode,
    },
    /// A motion is queued or running, so a servo session cannot start.
    Busy,
    /// A target outside the joint's limits, or not finite.
    OutOfRange { joint: usize, value: f64 },
    /// A parameter that is not finite; JSON cannot carry it.
    NotFinite,
    /// `move_to` ended its motion, but `joint` stayed `error` rad from the
    /// target for the settling time.
    NotSettled { joint: usize, error: f64 },
    /// An [`AuboOptions`](crate::AuboOptions) field out of its range, by
    /// name.
    InvalidOptions(&'static str),
    /// Neither interface latches or clears an emergency stop; the safety
    /// system, its inputs and the teach pendant do.
    NoEmergencyStop,
}

impl fmt::Display for AuboError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "aubo connection: {e}"),
            Self::Closed => write!(f, "the controller closed the connection"),
            Self::Timeout { waiting_for } => {
                write!(f, "no {waiting_for} from the controller in time")
            }
            Self::Malformed { what } => write!(f, "malformed {what} from the controller"),
            Self::Rpc { method, code, message } => {
                write!(f, "{method}: json-rpc error {code}: {message}")
            }
            Self::Refused { method, code } => write!(f, "{method} returned {code}"),
            Self::NoRobot => write!(f, "the controller names no robot"),
            Self::Dof(dof) => write!(f, "the arm has {dof} joints, not 6"),
            Self::CycleTime(t) => write!(f, "the controller's servo cycle is {t} s, not 0.005 s"),
            Self::Halted { robot_mode, safety_mode } => write!(
                f,
                "the arm cannot move: robot mode {robot_mode}, safety mode {safety_mode}"
            ),
            Self::ModeTimeout { waiting_for, robot_mode } => write!(
                f,
                "the arm did not reach robot mode {waiting_for} in time; it is {robot_mode}"
            ),
            Self::Busy => write!(f, "a motion is queued or running"),
            Self::OutOfRange { joint, value } => {
                write!(f, "joint {joint}: {value} is outside its limits")
            }
            Self::NotFinite => write!(f, "a parameter is not finite"),
            Self::NotSettled { joint, error } => {
                write!(f, "joint {joint} stayed {error} rad from the target")
            }
            Self::InvalidOptions(field) => write!(f, "AuboOptions::{field} is out of its range"),
            Self::NoEmergencyStop => {
                write!(
                    f,
                    "json-rpc and rtde can neither latch nor clear an emergency stop"
                )
            }
        }
    }
}

impl std::error::Error for AuboError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<io::Error> for AuboError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

/// Connection failures as network errors, the rest as unprocessable
/// instructions, as the roplat_protocols drivers map theirs.
pub(crate) fn robot_error(e: AuboError) -> RobotException {
    match e {
        AuboError::Io(_) | AuboError::Closed | AuboError::Timeout { .. } => {
            RobotException::NetworkError(e.to_string())
        }
        e => RobotException::from_algorithm(e),
    }
}
