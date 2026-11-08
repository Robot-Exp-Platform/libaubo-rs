//! The arms: joint envelope and kinematic table of each model.
//!
//! # Joint convention
//!
//! The controller's joint angles are the variables of its own DH table
//! (`RobotConfig.getKinematicsParam`): joint `i` turns link `i` by
//! `theta_i + q_i` about its axis, every joint about `+z`, without coupling.
//! A URDF built from [`AuboModel::DH`] as [`Dh`] describes, with each
//! `theta_i` in the fixed part of joint `i`, has the same zero and sign, so
//! the serial convention of [`Joints`] is the controller's and the driver
//! passes angles through unchanged, in both directions.
//!
//! # Sources
//!
//! - The structure of the table (twists `alpha`, offsets `theta`, the base
//!   height in `d_1`, the shoulder offset in `d_2`) is the example result of
//!   `getKinematicsParam` in AUBO's SDK reference (an `aubo_i5H`).
//! - Link lengths: the `aubo_description` URDFs of AuboRobot/aubo_robot
//!   (BSD-3-Clause, commit `3b3b9b4`), whose joint origins are this table
//!   for the i5; the reaches they give (625, 886.5 and 1350 mm for the i3,
//!   i5 and i10) are AUBO's published reaches.
//! - Position limits: +-175 degrees on every joint, AUBO's published range
//!   for the i-series (the same URDFs: +-3.05 rad). Speeds: the i5's
//!   published 150 degrees/s (joints 1 to 3) and 180 degrees/s (joints 4 to
//!   6). The i3 and i10 speeds are not declared: no source at hand states
//!   them, so they stay unbounded.
//!
//! Newer arms of the same names may differ: the SDK reference's i5H example
//! has a 116 mm flange distance and allows +-360 degrees. The driver reads
//! the controller's own speed and acceleration limits when it connects.

use std::f64::consts::{FRAC_PI_2, PI};

use roplat_behavior::{JointLimits, Joints};

/// One row of a modified (Craig) DH table, in AUBO's names: link `i` is
/// placed on link `i - 1` by `Rx(alpha) Tx(a) Rz(theta + q) Tz(d)`. Lengths
/// in metres, angles in radians.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dh {
    pub alpha: f64,
    pub a: f64,
    pub d: f64,
    pub theta: f64,
}

/// An AUBO i-series arm. Implemented by the model types of this module; the
/// driver is generic over it.
pub trait AuboModel: Joints<6> {
    /// The model's name, as AUBO writes it.
    const NAME: &'static str;
    /// The nominal kinematic table, base first, for the caller's kinematics
    /// and for simulated controllers. The driver itself does not compute
    /// kinematics.
    const DH: [Dh; 6];
}

const fn table(d1: f64, d2: f64, a2: f64, a3: f64, d5: f64, d6: f64) -> [Dh; 6] {
    [
        Dh { alpha: 0.0, a: 0.0, d: d1, theta: PI },
        Dh { alpha: -FRAC_PI_2, a: 0.0, d: d2, theta: -FRAC_PI_2 },
        Dh { alpha: PI, a: a2, d: 0.0, theta: 0.0 },
        Dh { alpha: PI, a: a3, d: 0.0, theta: -FRAC_PI_2 },
        Dh { alpha: -FRAC_PI_2, a: 0.0, d: d5, theta: 0.0 },
        Dh { alpha: FRAC_PI_2, a: 0.0, d: d6, theta: 0.0 },
    ]
}

/// +-175 degrees.
const RANGE: f64 = 175.0 * PI / 180.0;

const POSITIONS: JointLimits<6> = JointLimits::positions([-RANGE; 6], [RANGE; 6]);

/// The posture of AUBO's JSON-RPC guide (`moveJoint` to
/// `[0, -0.2618, 1.74533, 0.436333, 1.570797, 0]`): upper arm forward and
/// up, forearm back, tool pointing down.
const HOME: [f64; 6] = [0.0, -0.2618, 1.74533, 0.436333, 1.570797, 0.0];

/// AUBO i3: 3 kg payload, 625 mm reach.
#[derive(Clone, Copy, Debug)]
pub struct I3;

/// AUBO i5: 5 kg payload, 886.5 mm reach.
#[derive(Clone, Copy, Debug)]
pub struct I5;

/// AUBO i10: 10 kg payload, 1350 mm reach.
#[derive(Clone, Copy, Debug)]
pub struct I10;

impl Joints<6> for I3 {
    const JOINT_DEFAULT: [f64; 6] = HOME;
    const JOINT_LIMITS: JointLimits<6> = POSITIONS;
}

impl AuboModel for I3 {
    const NAME: &'static str = "AUBO-i3";
    const DH: [Dh; 6] = table(0.157, 0.119, 0.266, 0.2565, 0.1025, 0.094);
}

impl Joints<6> for I5 {
    const JOINT_DEFAULT: [f64; 6] = HOME;
    const JOINT_LIMITS: JointLimits<6> = JointLimits {
        dq_max: [
            150.0 * PI / 180.0,
            150.0 * PI / 180.0,
            150.0 * PI / 180.0,
            180.0 * PI / 180.0,
            180.0 * PI / 180.0,
            180.0 * PI / 180.0,
        ],
        ..POSITIONS
    };
}

impl AuboModel for I5 {
    const NAME: &'static str = "AUBO-i5";
    const DH: [Dh; 6] = table(0.122, 0.1215, 0.408, 0.376, 0.1025, 0.094);
}

impl Joints<6> for I10 {
    const JOINT_DEFAULT: [f64; 6] = HOME;
    const JOINT_LIMITS: JointLimits<6> = POSITIONS;
}

impl AuboModel for I10 {
    const NAME: &'static str = "AUBO-i10";
    const DH: [Dh; 6] = table(0.1632, 0.2013, 0.647, 0.6005, 0.1025, 0.094);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The horizontal reach the published figures give: upper arm, forearm
    /// and wrist offset.
    fn reach<M: AuboModel>() -> f64 {
        M::DH[2].a + M::DH[3].a + M::DH[4].d
    }

    #[test]
    fn the_tables_give_the_published_reaches() {
        assert!((reach::<I3>() - 0.625).abs() < 1e-9);
        assert!((reach::<I5>() - 0.8865).abs() < 1e-9);
        assert!((reach::<I10>() - 1.35).abs() < 1e-9);
    }
}
