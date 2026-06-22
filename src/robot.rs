use std::marker::PhantomData;

use robot_behavior::{
    Arm, ArmState, Coord, EndPoint, FlangeSpace, JointSpace, Joints, LoadState, MoveTo,
    OverrideOnce, Pose, Robot, RobotResult,
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

impl<T: AuboType, const N: usize> AuboRobot<T, N>
where
    Self: Joints<N> + EndPoint,
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

    pub fn set_coord(&mut self, coord: Coord) -> RobotResult<()> {
        self.coord.set(coord);
        Ok(())
    }

    pub fn set_scale(&mut self, scale: f64) -> RobotResult<()> {
        self.max_vel.set(Self::JOINT_VEL_BOUND.map(|v| v * scale));
        self.max_acc.set(Self::JOINT_ACC_BOUND.map(|v| v * scale));
        self.max_cartesian_vel
            .set(Self::CARTESIAN_VEL_BOUND * scale);
        self.max_cartesian_acc
            .set(Self::CARTESIAN_ACC_BOUND * scale);
        Ok(())
    }

    pub fn with_coord(&mut self, coord: Coord) -> &mut Self {
        self.coord.once(coord);
        self
    }

    pub fn with_scale(&mut self, scale: f64) -> &mut Self {
        self.max_vel.once(Self::JOINT_VEL_BOUND.map(|v| v * scale));
        self.max_acc.once(Self::JOINT_ACC_BOUND.map(|v| v * scale));
        self.max_cartesian_vel
            .once(Self::CARTESIAN_VEL_BOUND * scale);
        self.max_cartesian_acc
            .once(Self::CARTESIAN_ACC_BOUND * scale);
        self
    }

    pub fn with_velocity(&mut self, joint_vel: &[f64; N]) -> &mut Self {
        self.max_vel.once(*joint_vel);
        self
    }

    pub fn with_acceleration(&mut self, joint_acc: &[f64; N]) -> &mut Self {
        self.max_acc.once(*joint_acc);
        self
    }

    pub fn with_jerk(&mut self, _joint_jerk: &[f64; N]) -> &mut Self {
        self
    }

    pub fn with_cartesian_velocity(&mut self, cartesian_vel: f64) -> &mut Self {
        self.max_cartesian_vel.once(cartesian_vel);
        self
    }

    pub fn with_cartesian_acceleration(&mut self, cartesian_acc: f64) -> &mut Self {
        self.max_cartesian_acc.once(cartesian_acc);
        self
    }

    pub fn with_cartesian_jerk(&mut self, _cartesian_jerk: f64) -> &mut Self {
        self
    }

    pub fn with_rotation_velocity(&mut self, rotation_vel: f64) -> &mut Self {
        self.max_rotation_vel.once(rotation_vel);
        self
    }

    pub fn with_rotation_acceleration(&mut self, rotation_acc: f64) -> &mut Self {
        self.max_rotation_acc.once(rotation_acc);
        self
    }

    pub fn with_rotation_jerk(&mut self, _rotation_jerk: f64) -> &mut Self {
        self
    }
}

impl<T: AuboType, const N: usize> Robot for AuboRobot<T, N> {
    type State = ArmState<N>;
    const CONTROL_PERIOD: f64 = 1e-3;

    fn version() -> String {
        "AuboRobot".to_string()
    }

    fn read_state(&mut self) -> RobotResult<Self::State> {
        Ok(ArmState::default())
    }
}

impl<T: AuboType, const N: usize> Joints<N> for AuboRobot<T, N> {
    const JOINT_MIN: [f64; N] = [-std::f64::consts::TAU; N];
    const JOINT_MAX: [f64; N] = [std::f64::consts::TAU; N];
    const JOINT_VEL_BOUND: [f64; N] = [std::f64::consts::PI; N];
    const JOINT_ACC_BOUND: [f64; N] = [std::f64::consts::TAU; N];
}

impl<T: AuboType, const N: usize> EndPoint for AuboRobot<T, N> {
    const CARTESIAN_VEL_BOUND: f64 = 1.0;
    const CARTESIAN_ACC_BOUND: f64 = 1.0;
    const ROTATION_VEL_BOUND: f64 = std::f64::consts::PI;
    const ROTATION_ACC_BOUND: f64 = std::f64::consts::TAU;
}

impl<T: AuboType, const N: usize> MoveTo<JointSpace<N>> for AuboRobot<T, N> {
    fn move_to(&mut self, _target: [f64; N]) -> RobotResult<()> {
        unimplemented!()
    }
}

impl<T: AuboType, const N: usize> MoveTo<FlangeSpace> for AuboRobot<T, N> {
    fn move_to(&mut self, _target: Pose) -> RobotResult<()> {
        unimplemented!()
    }
}

impl<T: AuboType, const N: usize> Arm<N> for AuboRobot<T, N> {
    fn state(&mut self) -> RobotResult<ArmState<N>> {
        self.read_state()
    }

    fn set_load(&mut self, _load: LoadState) -> RobotResult<()> {
        unimplemented!()
    }

    fn get_joint(&self) -> [f64; N] {
        [0.; N]
    }

    fn get_endpoint(&self) -> Pose {
        Pose::default()
    }

    fn with_joint_vel(mut self, vel_bound: [f64; N]) -> Self {
        self.max_vel.once(vel_bound);
        self
    }

    fn with_joint_acc(mut self, acc_bound: [f64; N]) -> Self {
        self.max_acc.once(acc_bound);
        self
    }

    fn with_joint_jerk(self, _jerk_bound: [f64; N]) -> Self {
        self
    }

    fn with_torque(self, _torque_bound: [f64; N]) -> Self {
        self
    }

    fn with_torque_dot(self, _torque_dot_bound: [f64; N]) -> Self {
        self
    }

    fn with_cartesian_vel(mut self, vel_bound: f64) -> Self {
        self.max_cartesian_vel.once(vel_bound);
        self
    }

    fn with_cartesian_acc(mut self, acc_bound: f64) -> Self {
        self.max_cartesian_acc.once(acc_bound);
        self
    }

    fn with_cartesian_jerk(self, _jerk_bound: f64) -> Self {
        self
    }

    fn with_rotation_vel(mut self, vel_bound: f64) -> Self {
        self.max_rotation_vel.once(vel_bound);
        self
    }

    fn with_rotation_acc(mut self, acc_bound: f64) -> Self {
        self.max_rotation_acc.once(acc_bound);
        self
    }

    fn with_rotation_jerk(self, _jerk_bound: f64) -> Self {
        self
    }
}
