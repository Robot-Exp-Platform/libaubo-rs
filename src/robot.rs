use std::marker::PhantomData;

use robot_behavior::{
    Arm, ArmDOF, ArmParam, ArmState, Coord, LoadState, OverrideOnce, RobotResult,
};

pub trait AuboType {
    const N: usize;
}

pub struct AuboRobot<T: AuboType, const N: usize> {
    marker: PhantomData<T>,
    pub(crate) coord: OverrideOnce<Coord>,
    pub(crate) max_vel: OverrideOnce<[f64; N]>,
    pub(crate) max_acc: OverrideOnce<[f64; N]>,
    pub(crate) max_cartesian_vel: OverrideOnce<f64>,
    pub(crate) max_cartesian_acc: OverrideOnce<f64>,
    pub(crate) max_rotation_vel: OverrideOnce<f64>,
    pub(crate) max_rotation_acc: OverrideOnce<f64>,
}

impl<T: AuboType> ArmDOF for AuboRobot<T, { T::N }> {
    const N: usize = { T::N };
}

impl<T: AuboType, const N: usize> AuboRobot<T, N>
where
    Self: ArmParam<N>,
{
    pub fn new(_ip: &str) -> Self {
        Self {
            marker: PhantomData,
            coord: OverrideOnce::new(Coord::OCS),
            max_vel: OverrideOnce::new(Self::JOINT_VEL_BOUND),
            max_acc: OverrideOnce::new(Self::JOINT_ACC_BOUND),
            max_cartesian_vel: OverrideOnce::new(Self::CARTESIAN_VEL_BOUND),
            max_cartesian_acc: OverrideOnce::new(Self::CARTESIAN_ACC_BOUND),
            max_rotation_vel: OverrideOnce::new(Self::ROTATION_VEL_BOUND),
            max_rotation_acc: OverrideOnce::new(Self::ROTATION_ACC_BOUND),
        }
    }
}

impl<T: AuboType, const N: usize> Arm<N> for AuboRobot<T, N>
where
    AuboRobot<T, N>: ArmParam<N>,
{
    fn state(&mut self) -> RobotResult<ArmState<N>> {
        unimplemented!()
    }
    fn set_load(&mut self, _load: LoadState) -> RobotResult<()> {
        unimplemented!()
    }
    fn set_coord(&mut self, coord: Coord) -> RobotResult<()> {
        self.coord.set(coord);
        Ok(())
    }
    fn set_speed(&mut self, speed: f64) -> RobotResult<()> {
        self.max_vel.set(Self::JOINT_VEL_BOUND.map(|v| v * speed));
        self.max_acc.set(Self::JOINT_ACC_BOUND.map(|v| v * speed));
        self.max_cartesian_vel
            .set(Self::CARTESIAN_VEL_BOUND * speed);
        self.max_cartesian_acc
            .set(Self::CARTESIAN_ACC_BOUND * speed);
        Ok(())
    }

    fn with_coord(&mut self, coord: Coord) -> &mut Self {
        self.coord.once(coord);
        self
    }
    fn with_speed(&mut self, speed: f64) -> &mut Self {
        self.max_vel.once(Self::JOINT_VEL_BOUND.map(|v| v * speed));
        self.max_acc.once(Self::JOINT_ACC_BOUND.map(|v| v * speed));
        self.max_cartesian_vel
            .once(Self::CARTESIAN_VEL_BOUND * speed);
        self.max_cartesian_acc
            .once(Self::CARTESIAN_ACC_BOUND * speed);
        self
    }
    fn with_velocity(&mut self, joint_vel: &[f64; N]) -> &mut Self {
        self.max_vel.once(*joint_vel);
        self
    }
    fn with_acceleration(&mut self, joint_acc: &[f64; N]) -> &mut Self {
        self.max_acc.once(*joint_acc);
        self
    }
    fn with_jerk(&mut self, _joint_jerk: &[f64; N]) -> &mut Self {
        self
    }
    fn with_cartesian_velocity(&mut self, cartesian_vel: f64) -> &mut Self {
        self.max_cartesian_vel.once(cartesian_vel);
        self
    }
    fn with_cartesian_acceleration(&mut self, cartesian_acc: f64) -> &mut Self {
        self.max_cartesian_acc.once(cartesian_acc);
        self
    }
    fn with_cartesian_jerk(&mut self, _cartesian_jerk: f64) -> &mut Self {
        self
    }
    fn with_rotation_velocity(&mut self, rotation_vel: f64) -> &mut Self {
        self.max_rotation_vel.once(rotation_vel);
        self
    }
    fn with_rotation_acceleration(&mut self, rotation_acc: f64) -> &mut Self {
        self.max_rotation_acc.once(rotation_acc);
        self
    }
    fn with_rotation_jerk(&mut self, _rotation_jerk: f64) -> &mut Self {
        self
    }
}

impl<T: AuboType, const N: usize> ArmParam<N> for AuboRobot<T, N>
where
    T: ArmParam<N>,
{
    const DH: [[f64; 4]; N] = T::DH;
    const JOINT_DEFAULT: [f64; N] = T::JOINT_DEFAULT;
    const JOINT_MIN: [f64; N] = T::JOINT_MIN;
    const JOINT_MAX: [f64; N] = T::JOINT_MAX;
    const JOINT_VEL_BOUND: [f64; N] = T::JOINT_VEL_BOUND;
    const JOINT_ACC_BOUND: [f64; N] = T::JOINT_ACC_BOUND;
    const JOINT_JERK_BOUND: [f64; N] = T::JOINT_JERK_BOUND;
    const CARTESIAN_VEL_BOUND: f64 = T::CARTESIAN_VEL_BOUND;
    const CARTESIAN_ACC_BOUND: f64 = T::CARTESIAN_ACC_BOUND;
    const CARTESIAN_JERK_BOUND: f64 = T::CARTESIAN_JERK_BOUND;
    const ROTATION_VEL_BOUND: f64 = T::ROTATION_VEL_BOUND;
    const ROTATION_ACC_BOUND: f64 = T::ROTATION_ACC_BOUND;
    const ROTATION_JERK_BOUND: f64 = T::ROTATION_JERK_BOUND;
    const TORQUE_BOUND: [f64; N] = T::TORQUE_BOUND;
    const TORQUE_DOT_BOUND: [f64; N] = T::TORQUE_DOT_BOUND;
}
