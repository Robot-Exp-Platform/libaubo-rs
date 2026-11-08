use std::{marker::PhantomData, time::Duration};

use roplat_behavior::{
    Arm, ArmState, CartesianLimits, EndPoint, EndPointPose, FlangeSpace, JointLimits, JointSpace,
    Joints, LoadState, MotionLimits, MoveTo, Pose, Robot, RobotResult,
};

pub trait AuboType {
    const N: usize;
}

pub struct AuboRobot<T: AuboType, const N: usize> {
    marker: PhantomData<T>,
    limits: MotionLimits<N>,
}

impl<T: AuboType, const N: usize> AuboRobot<T, N>
where
    Self: Joints<N> + EndPoint,
{
    pub fn new(_ip: &str) -> Self {
        Self {
            marker: PhantomData,
            limits: MotionLimits::envelope::<Self>(),
        }
    }
}

impl<T: AuboType, const N: usize> Robot for AuboRobot<T, N> {
    type State = ArmState<N>;
    const CONTROL_PERIOD: Duration = Duration::from_millis(1);

    fn version() -> String {
        "AuboRobot".to_string()
    }

    fn read_state(&mut self) -> RobotResult<Self::State> {
        Ok(ArmState::default())
    }
}

impl<T: AuboType, const N: usize> Joints<N> for AuboRobot<T, N> {
    const JOINT_LIMITS: JointLimits<N> = JointLimits {
        dq_max: [std::f64::consts::PI; N],
        ddq_max: [std::f64::consts::TAU; N],
        ..JointLimits::positions([-std::f64::consts::TAU; N], [std::f64::consts::TAU; N])
    };
}

impl<T: AuboType, const N: usize> EndPoint for AuboRobot<T, N> {
    const CARTESIAN_LIMITS: CartesianLimits = CartesianLimits {
        linear_velocity_max: 1.0,
        linear_acceleration_max: 1.0,
        angular_velocity_max: std::f64::consts::PI,
        angular_acceleration_max: std::f64::consts::TAU,
        ..CartesianLimits::UNBOUNDED
    };
}

impl<T: AuboType, const N: usize> EndPointPose for AuboRobot<T, N> {
    fn get_endpoint(&self) -> Pose {
        Pose::default()
    }
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

    fn motion_limits(&self) -> MotionLimits<N> {
        self.limits
    }

    fn set_motion_limits(&mut self, limits: MotionLimits<N>) -> RobotResult<()> {
        limits.within(&MotionLimits::envelope::<Self>())?;
        self.limits = limits;
        Ok(())
    }
}
