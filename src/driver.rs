//! An AUBO arm on ARCS: commands over JSON-RPC, state over RTDE.

use std::{
    io::{self, Write},
    marker::PhantomData,
    net::{IpAddr, SocketAddr, TcpStream},
    ops::ControlFlow,
    thread,
    time::{Duration, Instant},
};

use roplat_behavior::{
    ControlExit, ControlStep, ControlWith, JointLimits, JointPositionControl, JointSample,
    JointSpace, JointState, Joints, MoveTo, Robot, RobotException, RobotResult,
};
use serde_json::Value;

use crate::{
    AuboError, AuboModel, Param, ReturnCode, RobotMode, SafetyMode,
    error::robot_error,
    json::Framer,
    rpc::{self, Rpc, six},
    rtde,
};

/// The controller's servo cycle, `RobotConfig.getCycletime` (0.005 in the
/// SDK reference's example), and the period of the driver's RTDE
/// subscription, its control sessions and every `servoJoint` time `t`.
/// [`connect`](AuboArm::connect) refuses a controller with another cycle.
pub const CONTROL_PERIOD: Duration = Duration::from_millis(5);

/// The RTDE channel the driver subscribes on.
const CHANNEL: u8 = 0;

/// The driver's RTDE subscription, in the recipe table's names: controller
/// time (s), actual joint positions (rad) and speeds (rad/s), and the
/// controller's joint targets (rad).
pub const RECIPE: [&str; 4] = ["timestamp", "R1_actual_q", "R1_actual_qd", "R1_target_q"];

/// The servo mode sessions run in (`setServoModeSelect`): 1, the planning
/// servo mode, the default of AUBO's ROS 2 driver. 0 leaves servo mode.
pub const SERVO_MODE: i64 = 1;

/// The `lookahead_time` of every `servoJoint` (s), inside the documented
/// range 0.03 to 0.2. AUBO's ServoJ notes say current firmware uses only `q`
/// and `t`.
pub const SERVO_LOOKAHEAD_TIME: f64 = 0.1;

/// The `gain` of every `servoJoint`, inside the documented range 100 to
/// 200; unused by current firmware, as [`SERVO_LOOKAHEAD_TIME`].
pub const SERVO_GAIN: f64 = 200.0;

/// How long `move_to` gives a queued motion to show up in `getExecId`
/// (controller time, s) before it takes the empty queue as the motion's end.
const START_GRACE: f64 = 0.5;

/// How close the controller's target must come to a session's last command
/// before the driver leaves servo mode (rad).
const TARGET_REACHED: f64 = 1e-4;

/// Polling period while waiting for a power or brake change.
const MODE_POLL: Duration = Duration::from_millis(20);

/// Ports, timeouts, speed and tolerances of the driver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AuboOptions {
    /// The JSON-RPC port, [`rpc::PORT`] on a controller.
    pub rpc_port: u16,
    /// The RTDE port, [`rtde::PORT`] on a controller.
    pub rtde_port: u16,
    /// Longest wait for a connection, a call's result or a data packet
    /// (wall clock).
    pub timeout: Duration,
    /// Longest wait for a power or brake change in `enable` and `disable`
    /// (wall clock).
    pub power_timeout: Duration,
    /// The fraction, in `(0, 1]`, of the slowest joint's speed and
    /// acceleration limit that `move_to` runs at. The limits are the
    /// controller's `getJointMaxSpeeds` and `getJointMaxAccelerations`, the
    /// speeds no higher than the model's `dq_max`.
    pub speed: f64,
    /// How close `move_to` must come to its target (rad).
    pub position_tolerance: f64,
    /// Longest settling after a motion's end (controller time).
    pub settle_timeout: Duration,
}

impl Default for AuboOptions {
    fn default() -> Self {
        Self {
            rpc_port: rpc::PORT,
            rtde_port: rtde::PORT,
            timeout: Duration::from_secs(2),
            power_timeout: Duration::from_secs(30),
            speed: 0.25,
            position_tolerance: 0.005,
            settle_timeout: Duration::from_secs(2),
        }
    }
}

impl AuboOptions {
    fn check(&self) -> Result<(), AuboError> {
        if self.timeout.is_zero() {
            return Err(AuboError::InvalidOptions("timeout"));
        }
        if self.power_timeout.is_zero() {
            return Err(AuboError::InvalidOptions("power_timeout"));
        }
        if !(self.speed > 0.0 && self.speed <= 1.0) {
            return Err(AuboError::InvalidOptions("speed"));
        }
        if !(self.position_tolerance > 0.0 && self.position_tolerance.is_finite()) {
            return Err(AuboError::InvalidOptions("position_tolerance"));
        }
        if self.settle_timeout.is_zero() {
            return Err(AuboError::InvalidOptions("settle_timeout"));
        }
        Ok(())
    }
}

/// One RTDE data packet of [`RECIPE`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AuboState {
    /// `timestamp`: controller time (s).
    pub timestamp: f64,
    /// `R1_actual_q` and `R1_actual_qd` in `meas`, `R1_target_q` in `cmd`.
    pub joint: JointState<6>,
}

impl AuboState {
    fn decode(values: &[Value]) -> Option<Self> {
        let [timestamp, q, dq, target] = values else {
            return None;
        };
        let timestamp = timestamp.as_f64().filter(|t| t.is_finite())?;
        let mut joint = JointState::from_meas(JointSample::EMPTY.with_q(six(q)?).with_dq(six(dq)?));
        joint.cmd = JointSample::EMPTY.with_q(six(target)?);
        Some(Self { timestamp, joint })
    }

    /// `R1_actual_q`.
    pub fn q(&self) -> [f64; 6] {
        self.joint.meas.q().expect("decoded with positions")
    }

    /// `R1_actual_qd`.
    pub fn dq(&self) -> [f64; 6] {
        self.joint.meas.dq().expect("decoded with speeds")
    }

    /// `R1_target_q`.
    pub fn target_q(&self) -> [f64; 6] {
        self.joint.cmd.q().expect("decoded with targets")
    }
}

/// An AUBO arm of model `M` on an ARCS controller.
///
/// [`connect`](Self::connect) opens JSON-RPC, reads the robot's name
/// (`getRobotNames`), checks six joints (`getDof`) and a 5 ms servo cycle
/// (`getCycletime`), reads the joint speed and acceleration limits, and
/// subscribes to [`RECIPE`] at 200 Hz on RTDE. Every operation paces itself
/// on the data packets: a cycle waits for the next packet.
///
/// - [`Robot`]: `enable` powers the arm on (`RobotManage.poweron`, until the
///   robot mode is idle) and releases the brakes (`startup`, until running),
///   then requires a safety mode that allows motion. `disable` powers it off
///   (`poweroff`, until power off): the brakes hold. `stop` is
///   `MotionControl.stopJoint` at the slowest joint's acceleration limit,
///   which also ends servo mode; `emergency_stop` stops the same way and
///   reports [`AuboError::NoEmergencyStop`]. `reset` clears a protective
///   stop (`setUnlockProtectiveStop`). `is_moving` is `RobotState.isSteady`
///   negated.
/// - [`MoveTo<JointSpace<6>>`]: one `moveJoint` at [`AuboOptions::speed`],
///   then it waits until `getExecId` reports the queue empty and every joint
///   is within the tolerance.
/// - [`ControlWith<JointPositionControl<6>>`]: servo mode
///   ([`SERVO_MODE`]), then one `servoJoint` per data packet with
///   `t = 0.005`. The observation is the actual joints in `meas` and the
///   controller's targets in `cmd`; the duration is [`CONTROL_PERIOD`], or
///   `k + 1` periods after `k` missed packets, counted from `timestamp`.
///
/// After a session that returns `Ok`, the driver repeats the last command
/// until the controller's target has reached it, then leaves servo mode
/// (`setServoModeSelect(0)`): the arm stays at the last commanded position,
/// powered with the brakes released, and the next motion or session needs
/// no `enable`. A session ended by an error also leaves servo mode, where
/// the controller stopped the arm. The controller leaves servo mode by
/// itself when no `servoJoint` comes for `5 t` (25 ms); the next
/// `servoJoint` then returns [`ReturnCode::NOT_IN_SERVO`].
///
/// Joint angles are the controller's, which are the serial convention of
/// the model's URDF (see [`model`](crate::model)).
///
/// Not done: Cartesian motion and servoing, tool and payload settings, I/O,
/// RTDE inputs, reconnecting.
pub struct AuboArm<M> {
    rpc: Rpc,
    rtde: TcpStream,
    framer: Framer,
    options: AuboOptions,
    latest: Option<AuboState>,
    robot_type: String,
    software_version: i64,
    max_speeds: [f64; 6],
    max_accelerations: [f64; 6],
    /// The driver has put the controller in servo mode.
    servo: bool,
    out: Vec<u8>,
    model: PhantomData<fn() -> M>,
}

impl<M: AuboModel> AuboArm<M> {
    /// Connects to the controller at `ip`, on the ports of `options`.
    pub fn connect(ip: IpAddr, options: AuboOptions) -> Result<Self, AuboError> {
        options.check()?;
        let mut rpc = Rpc::connect(SocketAddr::new(ip, options.rpc_port), options.timeout)?;
        let dof = rpc.int("RobotConfig.getDof", &[])?;
        if dof != 6 {
            return Err(AuboError::Dof(dof));
        }
        let cycle = rpc.float("RobotConfig.getCycletime", &[])?;
        if (cycle - CONTROL_PERIOD.as_secs_f64()).abs() > 1e-6 {
            return Err(AuboError::CycleTime(cycle));
        }
        let software_version = rpc
            .call_controller("SystemInfo.getControlSoftwareVersionCode", &[])?
            .as_i64()
            .ok_or(AuboError::Malformed { what: "getControlSoftwareVersionCode" })?;
        let robot_type = rpc.string("RobotConfig.getRobotType")?;
        let max_speeds = positive(rpc.joints("RobotConfig.getJointMaxSpeeds")?)
            .ok_or(AuboError::Malformed { what: "getJointMaxSpeeds" })?;
        let max_accelerations = positive(rpc.joints("RobotConfig.getJointMaxAccelerations")?)
            .ok_or(AuboError::Malformed { what: "getJointMaxAccelerations" })?;

        let rtde =
            TcpStream::connect_timeout(&SocketAddr::new(ip, options.rtde_port), options.timeout)?;
        rtde.set_nodelay(true)?;
        rtde.set_read_timeout(Some(options.timeout))?;
        rtde.set_write_timeout(Some(options.timeout))?;
        let mut arm = Self {
            rpc,
            rtde,
            framer: Framer::new(),
            options,
            latest: None,
            robot_type,
            software_version,
            max_speeds,
            max_accelerations,
            servo: false,
            out: Vec::with_capacity(512),
            model: PhantomData,
        };
        arm.subscribe(1.0 / CONTROL_PERIOD.as_secs_f64())?;
        arm.next_state()?;
        Ok(arm)
    }

    fn subscribe(&mut self, frequency: f64) -> Result<(), AuboError> {
        self.out.clear();
        let segments: &[&str] = if frequency > 0.0 { &RECIPE } else { &[] };
        rtde::encode_setup(&mut self.out, CHANNEL, frequency, segments, false)?;
        self.rtde.write_all(&self.out)?;
        Ok(())
    }

    pub fn options(&self) -> &AuboOptions {
        &self.options
    }

    /// The robot's name, the scope of its JSON-RPC calls (`rob1`).
    pub fn robot_name(&self) -> &str {
        self.rpc.robot()
    }

    /// `RobotConfig.getRobotType`, as `aubo_i5`. The driver does not check
    /// it against `M`.
    pub fn robot_type(&self) -> &str {
        &self.robot_type
    }

    /// `SystemInfo.getControlSoftwareVersionCode`: 28000 is ARCS 0.28.0.
    pub fn software_version(&self) -> i64 {
        self.software_version
    }

    /// `RobotConfig.getJointMaxSpeeds` (rad/s), read at connection.
    pub fn max_speeds(&self) -> [f64; 6] {
        self.max_speeds
    }

    /// `RobotConfig.getJointMaxAccelerations` (rad/s²), read at connection.
    pub fn max_accelerations(&self) -> [f64; 6] {
        self.max_accelerations
    }

    /// The newest data packet received so far, without waiting.
    pub fn latest(&self) -> Option<&AuboState> {
        self.latest.as_ref()
    }

    /// Calls a function of the robot by its SDK name without the robot's
    /// name (`RobotState.getTcpPose`) and returns its result: for what the
    /// driver does not wrap.
    pub fn call(&mut self, name: &'static str, params: &[Param<'_>]) -> Result<Value, AuboError> {
        self.rpc.call(name, params)
    }

    /// `RobotState.getRobotModeType`.
    pub fn robot_mode(&mut self) -> Result<RobotMode, AuboError> {
        let name = self.rpc.string("RobotState.getRobotModeType")?;
        RobotMode::from_name(&name).ok_or(AuboError::Malformed { what: "robot mode" })
    }

    /// `RobotState.getSafetyModeType`.
    pub fn safety_mode(&mut self) -> Result<SafetyMode, AuboError> {
        let name = self.rpc.string("RobotState.getSafetyModeType")?;
        SafetyMode::from_name(&name).ok_or(AuboError::Malformed { what: "safety mode" })
    }

    /// `MotionControl.getExecId`: the motion being executed, -1 when none
    /// is queued.
    pub fn exec_id(&mut self) -> Result<i64, AuboError> {
        self.rpc.int("MotionControl.getExecId", &[])
    }

    /// Hands out every complete packet received; `Ok(true)` when a data
    /// packet came.
    fn absorb(&mut self) -> Result<bool, AuboError> {
        let mut fresh = false;
        while let Some(packet) = self.framer.next_value()? {
            // Packets of other channels, and answers to setup messages if
            // the controller sends any, are skipped.
            if let Some(values) = rtde::data(&packet, CHANNEL)? {
                let state =
                    AuboState::decode(values).ok_or(AuboError::Malformed { what: "rtde data" })?;
                self.latest = Some(state);
                fresh = true;
            }
        }
        Ok(fresh)
    }

    /// Waits for at least one data packet and returns the newest.
    fn next_state(&mut self) -> Result<AuboState, AuboError> {
        let deadline = Instant::now() + self.options.timeout;
        loop {
            if self.absorb()? {
                return Ok(self.latest.expect("a data packet came"));
            }
            if Instant::now() >= deadline {
                return Err(AuboError::Timeout { waiting_for: "rtde data" });
            }
            match self.framer.fill(&mut self.rtde) {
                Ok(0) => return Err(AuboError::Closed),
                Ok(_) => {}
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) =>
                {
                    return Err(AuboError::Timeout { waiting_for: "rtde data" });
                }
                Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) => return Err(AuboError::Io(e)),
            }
        }
    }

    /// Reads everything received so far without waiting.
    fn drain(&mut self) -> Result<(), AuboError> {
        self.rtde.set_nonblocking(true)?;
        let drained = loop {
            if let Err(e) = self.absorb() {
                break Err(e);
            }
            match self.framer.fill(&mut self.rtde) {
                Ok(0) => break Err(AuboError::Closed),
                Ok(_) => {}
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break Ok(()),
                Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) => break Err(AuboError::Io(e)),
            }
        };
        self.rtde.set_nonblocking(false)?;
        drained
    }

    /// The first data packet newer than everything received so far: the
    /// state at the start of an operation.
    fn fresh_state(&mut self) -> Result<AuboState, AuboError> {
        self.drain()?;
        self.next_state()
    }

    /// The newest data packet received, after reading everything that has
    /// arrived; waits only when none has come yet.
    fn newest_state(&mut self) -> Result<AuboState, AuboError> {
        self.drain()?;
        match self.latest {
            Some(state) => Ok(state),
            None => self.next_state(),
        }
    }

    /// Why the arm cannot move, if it cannot.
    fn halted(&mut self) -> Result<Option<AuboError>, AuboError> {
        let robot_mode = self.robot_mode()?;
        let safety_mode = self.safety_mode()?;
        Ok(
            (robot_mode != RobotMode::Running || !safety_mode.allows_motion())
                .then_some(AuboError::Halted { robot_mode, safety_mode }),
        )
    }

    /// Polls the robot mode until it is one of `wanted`, keeping RTDE read.
    fn wait_mode(&mut self, wanted: &[RobotMode]) -> Result<RobotMode, AuboError> {
        let deadline = Instant::now() + self.options.power_timeout;
        loop {
            let mode = self.robot_mode()?;
            if wanted.contains(&mode) {
                return Ok(mode);
            }
            if Instant::now() >= deadline {
                return Err(AuboError::ModeTimeout { waiting_for: wanted[0], robot_mode: mode });
            }
            self.drain()?;
            thread::sleep(MODE_POLL);
        }
    }

    /// Powers the arm on and releases its brakes, then checks the safety
    /// mode.
    pub fn power_up(&mut self) -> Result<(), AuboError> {
        use RobotMode::*;
        let mut mode = self.robot_mode()?;
        if matches!(mode, Booting | PowerOn | BrakeReleasing | PowerOffing) {
            mode = self.wait_mode(&[Idle, Running, PowerOff])?;
        }
        if mode == PowerOff {
            self.rpc.command("RobotManage.poweron", &[])?;
            mode = self.wait_mode(&[Idle, Running])?;
        }
        if mode == Idle {
            self.rpc.command("RobotManage.startup", &[])?;
            mode = self.wait_mode(&[Running])?;
        }
        let safety_mode = self.safety_mode()?;
        if mode != Running || !safety_mode.allows_motion() {
            return Err(AuboError::Halted { robot_mode: mode, safety_mode });
        }
        Ok(())
    }

    /// Leaves servo mode if the driver entered it, then powers the arm off.
    pub fn power_down(&mut self) -> Result<(), AuboError> {
        self.leave_servo()?;
        self.rpc.command("RobotManage.poweroff", &[])?;
        self.wait_mode(&[RobotMode::PowerOff]).map(drop)
    }

    /// `stopJoint` at the slowest joint's acceleration limit.
    pub fn halt(&mut self) -> Result<(), AuboError> {
        let deceleration = min(self.max_accelerations);
        // stopJoint also ends servo mode.
        self.servo = false;
        self.rpc
            .command("MotionControl.stopJoint", &[Param::Float(deceleration)])
    }

    fn check_target(target: &[f64; 6]) -> Result<(), AuboError> {
        let limits: JointLimits<6> = M::JOINT_LIMITS;
        for (joint, &value) in target.iter().enumerate() {
            if !value.is_finite() || value < limits.q_min[joint] || value > limits.q_max[joint] {
                return Err(AuboError::OutOfRange { joint, value });
            }
        }
        Ok(())
    }

    /// One `moveJoint` to `target`; returns once the queue is empty and
    /// every joint is within the tolerance.
    pub fn move_joints(&mut self, target: [f64; 6]) -> Result<(), AuboError> {
        Self::check_target(&target)?;
        let limits: JointLimits<6> = M::JOINT_LIMITS;
        let speed = self.options.speed;
        let v = speed
            * min(std::array::from_fn(|j| {
                self.max_speeds[j].min(limits.dq_max[j])
            }));
        let a = speed
            * min(std::array::from_fn(|j| {
                self.max_accelerations[j].min(limits.ddq_max[j])
            }));
        let start = self.fresh_state()?;
        self.rpc.command(
            "MotionControl.moveJoint",
            &[
                Param::Floats(&target),
                Param::Float(a),
                Param::Float(v),
                Param::Float(0.0),
                Param::Float(0.0),
            ],
        )?;
        let distance = (0..6)
            .map(|j| (target[j] - start.q()[j]).abs())
            .fold(0.0, f64::max);
        let limit = 2.0 * profile_time(distance, v, a) + self.options.settle_timeout.as_secs_f64();
        let mut seen = false;
        let mut state;
        loop {
            state = self.next_state()?;
            let elapsed = state.timestamp - start.timestamp;
            if self.exec_id()? != -1 {
                seen = true;
                if elapsed > limit {
                    return Err(AuboError::Timeout { waiting_for: "the end of moveJoint" });
                }
                continue;
            }
            if seen
                || elapsed >= START_GRACE
                || worst(state.q(), target).1 <= self.options.position_tolerance
            {
                break;
            }
        }
        let settle = state.timestamp;
        loop {
            let (joint, error) = worst(state.q(), target);
            if error <= self.options.position_tolerance {
                return Ok(());
            }
            if state.timestamp - settle > self.options.settle_timeout.as_secs_f64() {
                return Err(self
                    .halted()?
                    .unwrap_or(AuboError::NotSettled { joint, error }));
            }
            state = self.next_state()?;
        }
    }

    /// Enters servo mode if the driver has not.
    fn enter_servo(&mut self) -> Result<(), AuboError> {
        if !self.servo {
            self.rpc.command(
                "MotionControl.setServoModeSelect",
                &[Param::Int(SERVO_MODE)],
            )?;
            self.servo = true;
        }
        Ok(())
    }

    /// Leaves servo mode if the driver entered it.
    fn leave_servo(&mut self) -> Result<(), AuboError> {
        if self.servo {
            self.servo = false;
            self.rpc
                .command("MotionControl.setServoModeSelect", &[Param::Int(0)])?;
        }
        Ok(())
    }

    /// One `servoJoint` to `target`, reached in one cycle.
    pub fn servo(&mut self, target: [f64; 6]) -> Result<(), AuboError> {
        Self::check_target(&target)?;
        let result = self.rpc.command(
            "MotionControl.servoJoint",
            &[
                Param::Floats(&target),
                Param::Float(0.0),
                Param::Float(0.0),
                Param::Float(CONTROL_PERIOD.as_secs_f64()),
                Param::Float(SERVO_LOOKAHEAD_TIME),
                Param::Float(SERVO_GAIN),
            ],
        );
        if let Err(AuboError::Refused { code: ReturnCode::NOT_IN_SERVO, .. }) = result {
            // The controller left servo mode by itself.
            self.servo = false;
        }
        result
    }

    /// Repeats `last` until the controller's target is within
    /// [`TARGET_REACHED`] of it, for at most the settling time.
    fn reach(&mut self, last: [f64; 6]) -> Result<(), AuboError> {
        // The newest packet, not the next: a session ended by `Break` sent
        // no command after it, and a controller paced by commands sends
        // nothing more until the next one.
        let mut state = self.newest_state()?;
        let begin = state.timestamp;
        loop {
            let (joint, error) = worst(state.target_q(), last);
            if error <= TARGET_REACHED {
                return Ok(());
            }
            if state.timestamp - begin > self.options.settle_timeout.as_secs_f64() {
                return Err(AuboError::NotSettled { joint, error });
            }
            self.servo(last)?;
            state = self.next_state()?;
        }
    }

    /// The end of a session that returns `Ok`: the controller's target on
    /// the last command, if there was one, then out of servo mode.
    fn settle_servo(&mut self, last: Option<[f64; 6]>) -> Result<(), AuboError> {
        let reached = last.map_or(Ok(()), |last| self.reach(last));
        let left = self.leave_servo();
        reached.and(left)
    }

    fn run_session<F>(&mut self, mut closure: F) -> RobotResult<Option<[f64; 6]>>
    where
        F: FnMut(JointState<6>, Duration) -> ControlStep<[f64; 6]>,
    {
        if self.exec_id().map_err(robot_error)? != -1 {
            return Err(robot_error(AuboError::Busy));
        }
        self.enter_servo().map_err(robot_error)?;
        // The newest packet once in servo mode: a controller paced by
        // commands sends the next one only after the first command.
        let mut state = self.newest_state().map_err(robot_error)?;
        let mut duration = CONTROL_PERIOD;
        let mut last = None;
        loop {
            match closure(state.joint, duration) {
                ControlFlow::Break(ControlExit::Stopped) => return Ok(last),
                ControlFlow::Break(ControlExit::Failed(error)) => return Err(error),
                ControlFlow::Continue((command, done)) => {
                    self.servo(command).map_err(robot_error)?;
                    last = Some(command);
                    if done {
                        return Ok(last);
                    }
                }
            }
            let next = self.next_state().map_err(robot_error)?;
            duration = CONTROL_PERIOD * cycles(state.timestamp, next.timestamp);
            state = next;
        }
    }

    /// Unsubscribes from RTDE and closes both connections.
    pub fn close(mut self) -> Result<(), AuboError> {
        self.leave_servo()?;
        self.subscribe(0.0)?;
        self.rtde.shutdown(std::net::Shutdown::Both)?;
        Ok(())
    }
}

/// Every value positive and finite.
fn positive(values: [f64; 6]) -> Option<[f64; 6]> {
    values
        .iter()
        .all(|v| v.is_finite() && *v > 0.0)
        .then_some(values)
}

fn min(values: [f64; 6]) -> f64 {
    values.into_iter().fold(f64::INFINITY, f64::min)
}

/// The joint furthest from `target`, and how far.
fn worst(q: [f64; 6], target: [f64; 6]) -> (usize, f64) {
    (0..6)
        .map(|j| (j, (q[j] - target[j]).abs()))
        .fold((0, 0.0), |a, b| if b.1 > a.1 { b } else { a })
}

/// The time of a trapezoidal move over `distance` at speed `v` and
/// acceleration `a`.
fn profile_time(distance: f64, v: f64, a: f64) -> f64 {
    if distance * a > v * v {
        distance / v + v / a
    } else {
        2.0 * (distance / a).sqrt()
    }
}

/// Whole controller cycles between two timestamps, at least one.
fn cycles(from: f64, to: f64) -> u32 {
    ((to - from) / CONTROL_PERIOD.as_secs_f64())
        .round()
        .clamp(1.0, f64::from(u32::MAX)) as u32
}

impl<M: AuboModel> Joints<6> for AuboArm<M> {
    const JOINT_DEFAULT: [f64; 6] = M::JOINT_DEFAULT;
    const JOINT_PACKED: [f64; 6] = M::JOINT_PACKED;
    const JOINT_LIMITS: JointLimits<6> = M::JOINT_LIMITS;
}

impl<M: AuboModel> Robot for AuboArm<M> {
    type State = AuboState;
    const CONTROL_PERIOD: Duration = CONTROL_PERIOD;

    fn version() -> String {
        env!("CARGO_PKG_VERSION").to_owned()
    }

    /// The first data packet after everything received so far.
    fn read_state(&mut self) -> RobotResult<AuboState> {
        self.fresh_state().map_err(robot_error)
    }

    /// [`power_up`](AuboArm::power_up).
    fn enable(&mut self) -> RobotResult<()> {
        self.power_up().map_err(robot_error)
    }

    /// [`power_down`](AuboArm::power_down): the brakes hold.
    fn disable(&mut self) -> RobotResult<()> {
        self.power_down().map_err(robot_error)
    }

    /// Clears a protective stop.
    fn reset(&mut self) -> RobotResult<()> {
        self.rpc
            .command("RobotManage.setUnlockProtectiveStop", &[])
            .map_err(robot_error)
    }

    fn is_moving(&mut self) -> RobotResult<bool> {
        self.rpc
            .bool("RobotState.isSteady")
            .map(|steady| !steady)
            .map_err(robot_error)
    }

    /// [`halt`](AuboArm::halt).
    fn stop(&mut self) -> RobotResult<()> {
        self.halt().map_err(robot_error)
    }

    /// Stops as [`stop`](Robot::stop) does, then reports that the driver
    /// cannot latch a stop: neither interface offers one.
    fn emergency_stop(&mut self) -> RobotResult<()> {
        self.halt().map_err(robot_error)?;
        Err(robot_error(AuboError::NoEmergencyStop))
    }

    fn clear_emergency_stop(&mut self) -> RobotResult<()> {
        Err(robot_error(AuboError::NoEmergencyStop))
    }
}

impl<M: AuboModel> MoveTo<JointSpace<6>> for AuboArm<M> {
    fn move_to(&mut self, target: [f64; 6]) -> RobotResult<()> {
        self.move_joints(target).map_err(robot_error)
    }
}

/// Each cycle's command is a `servoJoint` target. A session that returns
/// `Ok` holds at its last command; every session leaves servo mode.
impl<M: AuboModel> ControlWith<JointPositionControl<6>> for AuboArm<M> {
    /// The controller's `R1_target_q`: where it holds the joints.
    fn hold_command(obs: &JointState<6>) -> [f64; 6] {
        obs.cmd.q().expect("AuboArm observations carry R1_target_q")
    }

    fn control_with_flow<F>(&mut self, closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<6>, Duration) -> ControlStep<[f64; 6]>,
    {
        match self.run_session(closure) {
            Ok(last) => self.settle_servo(last).map_err(robot_error),
            Err(primary) => match self.leave_servo() {
                Ok(()) => Err(primary),
                Err(cleanup) => Err(RobotException::ControlSession {
                    primary: Box::new(primary),
                    cleanup: Box::new(robot_error(cleanup)),
                }),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missed_cycles_are_counted_from_the_timestamps() {
        assert_eq!(cycles(1.0, 1.005), 1);
        assert_eq!(cycles(1.0, 1.0), 1);
        assert_eq!(cycles(1.0, 1.0201), 4);
    }

    #[test]
    fn a_profile_takes_its_time() {
        // Cruising: 1 rad at 1 rad/s and 2 rad/s² takes 1 + 0.5 s.
        assert!((profile_time(1.0, 1.0, 2.0) - 1.5).abs() < 1e-12);
        // Triangular: 0.5 rad at 2 rad/s² takes 2 sqrt(0.25) s.
        assert!((profile_time(0.5, 10.0, 2.0) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn a_packet_of_the_recipe_decodes() {
        let packet: Value =
            serde_json::from_str("[0,12.5,[0,1,2,3,4,5],[0,0,0,0,0,0.5],[0,1,2,3,4,5.5]]").unwrap();
        let values = rtde::data(&packet, CHANNEL).unwrap().unwrap();
        let state = AuboState::decode(values).unwrap();
        assert_eq!(state.timestamp, 12.5);
        assert_eq!(state.q(), [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]);
        assert_eq!(state.dq()[5], 0.5);
        assert_eq!(state.target_q()[5], 5.5);
        assert!(AuboState::decode(&values[..3]).is_none());
    }
}
