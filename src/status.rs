//! The controller's state enumerations and the return codes of its calls.
//!
//! Names and values are those of `type_def.h` in AUBO's SDK reference
//! (`RobotModeType`, `SafetyModeType`); JSON-RPC sends these enumerations by
//! name, as the examples of `getRobotModeType` (`"Running"`) and
//! `getSafetyModeType` (`"Normal"`) show.

use std::fmt;

macro_rules! named {
    (
        $(#[$meta:meta])*
        $name:ident { $($(#[$vmeta:meta])* $variant:ident = $code:literal),* $(,)? }
    ) => {
        $(#[$meta])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
        pub enum $name {
            $($(#[$vmeta])* $variant),*
        }

        impl $name {
            /// Every value, in the order of the SDK's enumeration.
            pub const ALL: &'static [Self] = &[$(Self::$variant),*];

            /// The name JSON-RPC sends.
            pub fn name(self) -> &'static str {
                match self {
                    $(Self::$variant => stringify!($variant)),*
                }
            }

            /// The value in the SDK's enumeration.
            pub fn code(self) -> i32 {
                match self {
                    $(Self::$variant => $code),*
                }
            }

            /// The value called `name`, if there is one.
            pub fn from_name(name: &str) -> Option<Self> {
                Self::ALL.iter().copied().find(|v| v.name() == name)
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(self.name())
            }
        }
    };
}

named! {
    /// `RobotModeType`: where the arm is in its power-up sequence.
    RobotMode {
        NoController = -1,
        Disconnected = 0,
        ConfirmSafety = 1,
        Booting = 2,
        PowerOff = 3,
        PowerOn = 4,
        /// Powered, brakes holding.
        Idle = 5,
        BrakeReleasing = 6,
        BackDrive = 7,
        /// Powered with the brakes released: the arm can move.
        Running = 8,
        /// The SDK's spelling.
        Maintaince = 9,
        Error = 10,
        PowerOffing = 11,
    }
}

named! {
    /// `SafetyModeType`: what the safety system allows.
    SafetyMode {
        Undefined = 0,
        Normal = 1,
        ReducedMode = 2,
        Recovery = 3,
        Violation = 4,
        ProtectiveStop = 5,
        SafeguardStop = 6,
        SystemEmergencyStop = 7,
        RobotEmergencyStop = 8,
        Fault = 9,
    }
}

impl SafetyMode {
    /// The modes in which motion calls are accepted: the SDK reference
    /// answers `moveJoint` and `servoJoint` with `AUBO_BAD_STATE` in any
    /// other.
    pub fn allows_motion(self) -> bool {
        matches!(self, Self::Normal | Self::ReducedMode | Self::Recovery)
    }
}

/// The integer a command call returns: 0 on success. The named values are
/// those the SDK reference lists for `moveJoint`, `servoJoint` and
/// `speedJoint`; positive values say the call was refused in the
/// controller's current state, negative values that it could not be
/// carried out.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ReturnCode(pub i64);

impl ReturnCode {
    pub const OK: Self = Self(0);
    /// `AUBO_BAD_STATE`: the safety mode is not normal, reduced or
    /// recovery.
    pub const BAD_STATE: Self = Self(1);
    /// `AUBO_QUEUE_FULL`: the planning queue is full.
    pub const QUEUE_FULL: Self = Self(2);
    /// `AUBO_BUSY`: the previous instruction is still executing.
    pub const BUSY: Self = Self(3);
    /// `-AUBO_BAD_STATE`: the robot mode is not running, or the
    /// controller's thread is gone.
    pub const NOT_RUNNING: Self = Self(-1);
    /// `-AUBO_TIMEOUT`: the call timed out inside the controller.
    pub const TIMEOUT: Self = Self(-4);
    /// `-AUBO_INVL_ARGUMENT`: a parameter is out of range, too short, or
    /// (for `servoJoint`) more than 15 degrees from the previous target.
    pub const INVALID_ARGUMENT: Self = Self(-5);
    /// `-AUBO_REQUEST_IGNORE`: `servoJoint` outside servo mode.
    pub const NOT_IN_SERVO: Self = Self(-13);
    /// `-AUBO_POS_BOUND_ERR`: a joint position beyond its limit.
    pub const POSITION_BOUND: Self = Self(-18);
    /// `-AUBO_TRAJ_SELF_COLLISION`: the trajectory risks a self-collision.
    pub const SELF_COLLISION: Self = Self(-22);
    /// `-AUBO_ALGORITHM_PLAN_FAILED`: planning failed.
    pub const PLAN_FAILED: Self = Self(-30);

    /// What the SDK reference says the code means, when it names it.
    pub fn meaning(self) -> Option<&'static str> {
        Some(match self {
            Self::OK => "success",
            Self::BAD_STATE => "safety mode does not allow motion",
            Self::QUEUE_FULL => "planning queue full",
            Self::BUSY => "previous instruction still executing",
            Self::NOT_RUNNING => "robot mode is not running",
            Self::TIMEOUT => "timed out in the controller",
            Self::INVALID_ARGUMENT => "invalid argument",
            Self::NOT_IN_SERVO => "not in servo mode",
            Self::POSITION_BOUND => "joint position beyond its limit",
            Self::SELF_COLLISION => "self-collision risk",
            Self::PLAN_FAILED => "planning failed",
            _ => return None,
        })
    }
}

impl fmt::Display for ReturnCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.meaning() {
            Some(meaning) => write!(f, "{} ({meaning})", self.0),
            None => write!(f, "{}", self.0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_and_codes_are_the_sdk_enumerations() {
        assert_eq!(RobotMode::from_name("Running"), Some(RobotMode::Running));
        assert_eq!(RobotMode::Running.code(), 8);
        assert_eq!(RobotMode::NoController.code(), -1);
        assert_eq!(RobotMode::ALL.len(), 13);
        assert_eq!(SafetyMode::from_name("Normal"), Some(SafetyMode::Normal));
        assert_eq!(SafetyMode::Fault.code(), 9);
        assert_eq!(SafetyMode::ALL.len(), 10);
        assert_eq!(RobotMode::from_name("running"), None);
    }

    #[test]
    fn motion_is_allowed_in_three_safety_modes() {
        let allowed: Vec<_> = SafetyMode::ALL
            .iter()
            .filter(|m| m.allows_motion())
            .collect();
        assert_eq!(
            allowed,
            [
                &SafetyMode::Normal,
                &SafetyMode::ReducedMode,
                &SafetyMode::Recovery
            ]
        );
    }
}
