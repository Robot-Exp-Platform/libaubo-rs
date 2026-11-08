# libaubo

AUBO i-series arms on the ARCS controller for roplat, without ROS and without AUBO's binary SDK: `AuboArm`, a driver that sends SDK calls as JSON-RPC over TCP and reads the joint state from an RTDE subscription, implementing the [roplat_behavior](https://github.com/Robot-Exp-Platform/roplat_behavior) contracts `Robot`, `Joints<6>`, `MoveTo<JointSpace<6>>` and `ControlWith<JointPositionControl<6>>`.

> **Status: not yet run on hardware.** Every message follows AUBO's published documentation and is tested against the simulated controller of `roplat_aubo_sim` (in [roplat_protocols](https://github.com/Robot-Exp-Platform/roplat_protocols)), including an i5 in rsbullet under gravity. Go through the checklist at the end before relying on it.

## Which AUBO interface, and why

| Interface | What it is | Used here |
|---|---|---|
| JSON-RPC 2.0, TCP 30004 (HTTP and WebSocket on 9012) | Every function of AUBO's SDK (`aubo_sdk`, `pyaubo_sdk`) as a call: `rob1.RobotManage.poweron`, `rob1.MotionControl.moveJoint`, `rob1.MotionControl.servoJoint`, … | commands and queries |
| RTDE, TCP 30010 (WebSocket on 9013) | JSON arrays: a subscription per channel at a chosen rate, values named in AUBO's RTDE recipe table | joint state at 200 Hz |
| Script, TCP 30002 | Lua scripts sent to the controller | not used |
| `aubo_sdk` / `pyaubo_sdk` | AUBO's C++ and Python SDK, distributed as binaries | not used: the driver speaks the SDK's own protocols |
| OurControl, `libauborobotcontroller` (older controllers, AUBO's ROS 1 `aubo_driver`) | a closed binary library over an undocumented protocol | not supported |

A program that is not ROS should use the ARCS SDK protocols: they are what AUBO's own SDK speaks, they are documented, and they carry both point-to-point motion and the servo mode. JSON-RPC gives every SDK call; RTDE gives a timestamped joint state on every 5 ms controller cycle, which paces the driver and lets it count missed cycles.

## Supported arms and software

| | |
|---|---|
| Arms | i5 (`AuboI5`), i3 (`AuboI3`), i10 (`AuboI10`): joint envelope and nominal DH table each (`model`) |
| Controller | ARCS, servo cycle 5 ms (`connect` refuses another `getCycletime`), six joints (`getDof`) |
| Rate | RTDE at 200 Hz; `CONTROL_PERIOD` is 5 ms |
| Not supported | other i-series models (no model types yet), older controllers, Cartesian motion and servoing, tool and payload settings, I/O, RTDE inputs, reconnecting |

## Network and controller setup

- Ethernet from the computer to the controller (default address 192.168.1.100 per AUBO's SDK guide). The driver opens TCP 30004 and 30010 on the controller; `AuboOptions::rpc_port` and `rtde_port` change them, for simulators.
- Whether the controller needs a setting on the pendant before it takes remote calls is not documented in the sources at hand; check its network settings if `connect` times out.
- Power, brake release and protective-stop clearing can be done by the driver (`enable`, `reset`) or on the pendant.

## Use

```rust
use libaubo::{AuboI5, AuboOptions};
use roplat_behavior::{ControlWith, JointPositionControl, JointSpace, MoveTo, Robot};

let mut arm = AuboI5::connect([192, 168, 1, 100].into(), AuboOptions::default())?;
arm.enable()?;                                    // poweron, then startup (brake release)
let home = [0.0, -0.2618, 1.74533, 0.436333, 1.570797, 0.0];
<AuboI5 as MoveTo<JointSpace<6>>>::move_to(&mut arm, home)?;
let mut t = 0.0;
<AuboI5 as ControlWith<JointPositionControl<6>>>::control_with(&mut arm, |_, dt| {
    t += dt.as_secs_f64();
    let mut q = home;
    q[0] += 0.1 * (t * 1.5).sin();
    (q, t >= 4.0)
})?;
arm.close()?;
```

- `connect(ip, options)`: checks the `AuboOptions`, calls `getRobotNames` (the robot's name scopes every later call: `rob1.…`), `RobotConfig.getDof` (must be 6), `getCycletime` (must be 0.005), `SystemInfo.getControlSoftwareVersionCode`, `getRobotType`, `getJointMaxSpeeds` and `getJointMaxAccelerations`, then subscribes to `timestamp`, `R1_actual_q`, `R1_actual_qd`, `R1_target_q` at 200 Hz on RTDE channel 0.
- `Robot`: `State` is `AuboState` (controller time, actual joints and speeds in `meas`, the controller's targets in `cmd`). `enable` powers on (`RobotManage.poweron`, until `getRobotModeType` is `Idle`) and releases the brakes (`startup`, until `Running`), each within `power_timeout` (30 s), then requires a safety mode of `Normal`, `ReducedMode` or `Recovery`. `disable` powers off (`poweroff`, until `PowerOff`): the brakes hold. `stop` is `MotionControl.stopJoint` at the slowest joint's acceleration limit, which also ends servo mode. `emergency_stop` stops the same way and returns an error: neither interface latches a stop. `reset` is `setUnlockProtectiveStop`. `is_moving` is `RobotState.isSteady` negated.
- `MoveTo<JointSpace<6>>`: one `moveJoint(q, a, v, 0, 0)` with `v` and `a` the `speed` fraction (default 0.25) of the slowest joint's speed and acceleration limits (the controller's, the speeds no higher than the model's `dq_max`); it returns once `getExecId` is -1 and every joint is within `position_tolerance` (0.005 rad), within `settle_timeout` (2 s of controller time) after the motion. A queued motion has 0.5 s to show up in `getExecId`. Targets outside the model's limits are refused before anything is sent.
- `ControlWith<JointPositionControl<6>>`: refuses to start over a queued motion (`getExecId` not -1), enters servo mode (`setServoModeSelect(1)`, the planning servo mode), then sends one `servoJoint(q, 0, 0, 0.005, 0.1, 200)` per data packet. The duration is 5 ms, or `k + 1` periods after `k` missed packets, counted from `timestamp`. `hold_command` is `R1_target_q`.
- **After a session.** One that returns `Ok` (on `done` or `Break(Stopped)`): the driver repeats the last command until `R1_target_q` is within 1e-4 rad of it (at most `settle_timeout`, else `NotSettled`), then leaves servo mode (`setServoModeSelect(0)`). The arm stays at the last commanded position, powered with brakes released; the next motion or session needs no `enable`. One that fails also leaves servo mode, where the controller stopped the arm.
- **The controller's watchdog.** ARCS leaves servo mode by itself when no `servoJoint` comes for `5 t` = 25 ms; the next `servoJoint` returns -13 and the session ends with that error.
- **Joint convention.** The controller's angles are the joint variables of its own DH table (every joint about `+z`, no coupling), which is the serial convention of the model URDFs built on that table; angles pass through unchanged.
- **Errors.** `AuboError` names each condition; a command call's non-zero result is `Refused { method, code }` with the SDK's codes (`ReturnCode`); a JSON-RPC error object is `Rpc { code, message }`. Mapping to `RobotException`: `Io`, `Closed` and `Timeout` become `NetworkError`, everything else `UnprocessableInstructionError`.
- `AuboArm::call(name, params)` sends any other robot call (`"RobotState.getTcpPose"`) and returns its JSON result.

## Protocol sources

| Item | Source |
|---|---|
| Ports 30004 (TCP), 9012 (HTTP, WebSocket); request, result and error members; error codes | AUBO SDK guide, "JSON-RPC 协议说明" (developer.aubo-robotics.cn, `guide_sdk/3_protocol/1_json_rpc_protocol`) |
| Call names `getRobotNames`, `SystemInfo.getControlSoftwareVersionCode`, `rob1.RobotState.getRobotModeType`, `rob1.RobotManage.poweron` / `startup` / `poweroff`, `rob1.MotionControl.moveJoint` (rad, rad/s², rad/s), `setSpeedFraction`; the order power on, brake release, move | AUBO application note 33, "JSON-RPC API User Guide" (docs.aubo-robotics.cn) |
| `moveJoint`, `servoJoint`, `setServoModeSelect` (0 off, 1 planning, 2 and 3 transparent), `stopJoint`, `getExecId` (-1 when the queue is empty), `speedJoint`; return codes 0, 1, 2, 3, -1, -4, -5, -13, -18, -22, -30 | AUBO SDK reference, `MotionControl` (docs.aubo-robotics.cn/arcs_api) |
| `getJointPositions`, `getRobotModeType`, `getSafetyModeType` (enums sent by name), `isSteady` | SDK reference, `RobotState` |
| `poweron`, `startup`, `poweroff`, `setUnlockProtectiveStop`; no call that latches an emergency stop | SDK reference, `RobotManage` |
| `getDof`, `getCycletime` (0.005), `getRobotType`, `getJointMaxSpeeds`, `getJointMaxAccelerations`, `getKinematicsParam` (the DH structure) | SDK reference, `RobotConfig` |
| `RobotModeType`, `SafetyModeType` names and values | SDK reference, `type_def.h` |
| `servoJoint` uses only `q` and `t`; `t` ≥ 0.005 and equal to the call interval; servo mode ends after `5 t` without a call, on `stopJoint`, or in drag mode | AUBO SDK guide, "ServoJ 功能说明" (`guide_sdk/7_servoj_func_test`) |
| RTDE: port 30010 (WebSocket 9013), the setup message `[100, {"chanel", "frequency", "segments", "to_server", "trigger"}]`, data `[channel, values…]`, unsubscribing with frequency 0 | AUBO SDK guide, "RTDE 协议说明" (`guide_sdk/3_protocol/2_rtde_protocol`) |
| RTDE names `timestamp` (s), `R1_actual_q`, `R1_actual_qd`, `R1_target_q` | AUBO SDK guide, "RTDE 菜单" (`guide_sdk/9_appendix/2_rtde_recipe`) |
| Planning servo mode as the default for streaming | AuboRobot/aubo_ros2_driver README (no license stated; read only) |
| Link lengths of the i3, i5 and i10 | AuboRobot/aubo_robot, `aubo_description/urdf/aubo_i{3,5,10}.urdf`, commit `3b3b9b4` (BSD-3-Clause): the joint origins of these files, read and converted into the DH table of `model.rs`; nothing copied |
| Reach (625, 886.5, 1350 mm), joint range ±175°, i5 joint speeds 150°/s (1–3) and 180°/s (4–6) | AUBO's published specifications, as distributors reproduce them (no AUBO datasheet at hand) |

### Independent check against AUBO's own examples

`tests/documented_messages.rs` writes requests with the driver's encoders and compares them with AUBO's printed examples: byte for byte for `servoJoint` (`[[0,0,0,0,0,0],0,0,10,0,0]`), `setServoModeSelect`, `stopJoint`, `setSpeedFraction`, `setServoMode`, `getKinematicsParam`, `getRobotNames`, `getControlSoftwareVersionCode`, `getRobotModeType`, `poweron`, `startup`, and the three RTDE setup messages; `moveJoint` after removing the spaces AUBO prints in it. `rpc.rs` reads the documented results (`servoJoint` → -13, `getRobotNames`, `getCycletime`, `getJointMaxSpeeds`, `getRobotModeType`). In `roplat_aubo_sim`, the i5's DH table at AUBO's home posture puts the flange at x 0.54887 m, y -0.1215 m, tool down, which are the x, y and orientation of the guide's `moveLine` examples from that posture (with the guide's TCP offset, x 0.64887).

## Unverified and not handled

- **TCP framing.** The guide documents no delimiter for JSON-RPC on TCP and sends bare requests from a test tool; the driver sends bare JSON texts and reads concatenated ones. RTDE is assumed the same.
- **RTDE details.** Whether 200 Hz is accepted, whether a setup message gets an answer (answers on other channels are skipped), and whether `timestamp` advances by exactly 5 ms per packet.
- **`getExecId` timing.** Whether a queued `moveJoint` shows up before it starts (the driver allows 0.5 s).
- **Servo mode.** How the planning servo mode follows a 5 ms stream, its lag, and what the arm does when the watchdog ends servo mode; the order of the codes 1 and -1 when both apply.
- **Models.** The DH structure is the SDK reference's i5H example; i3 and i10 lengths come from AUBO's ROS 1 URDFs, whose i3 file turns joints 4 and 6 about `-z`, unlike the table here: the i3's joint signs are unverified. Newer arms of the same names (the i5H example: 116 mm flange, ±360°) may differ; the i3 and i10 speeds are not declared.
- **Emergency stop.** No JSON-RPC call latches one; `emergency_stop` is a `stopJoint` that reports it did not latch.
- Not handled: Cartesian motion (`moveLine`, `servoCartesian`), tool and payload (`setTcpOffset`, `setPayload`), speed fraction, I/O, force control, RTDE inputs, reconnecting after a dropped connection.

## Real-device checklist (for the hardware group)

1. **Read only.** `AuboI5::connect` with the arm powered off. Check `robot_name()` (`rob1`), `robot_type()` (does it name the model you have? the driver does not check), `software_version()`, `max_speeds()`, `max_accelerations()`. `read_state()`: `q()` against the pendant's joint readout, in radians; read twice and check `timestamp` rises by about 5 ms per packet (`latest()`).
2. **Model.** Compare `call("RobotConfig.getKinematicsParam", &[Param::Bool(false)])` with `I5::DH` (a, d, alpha, theta), and `getJointMaxPositions` / `getJointMinPositions` with ±175°. Jog joints 4 and 6 on the pendant and check the signs of `q()`.
3. **Power.** `enable()`: the arm powers on and releases its brakes without moving; `robot_mode()` is `Running`. `disable()`: brakes engage. `enable()` again.
4. **Small move_to.** `AuboOptions { speed: 0.05, .. }`, 0.1 rad on the base joint, then on wrist 3. Compare the final angles with the pendant; time a move and check `getExecId` goes from an id to -1.
5. **Session.** A session that holds (`hold_command`) for 2 s: no motion. Then a sine of 0.05 rad at 0.5 Hz on joint 1, then 0.2 rad. Log callback durations (5 ms expected) and the tracking error; note any vibration (AUBO's ServoJ page discusses velocity feedforward).
6. **Watchdog.** During a slow session, stop the program (or sleep 50 ms in the callback): the arm must stop, and the next `servoJoint` return -13.
7. **After a session.** The arm stays still at the last command; `getServoModeSelect` is 0; a `move_to` follows without `enable`.
8. **Stops.** `stop()` during a `move_to` from another connection or the pendant; trigger a protective stop gently: the operation must end with `Halted` or `Refused` (code 1), `reset()` clears it.
9. `check_control_session` with 10 cycles where holding is safe; then repeat 1, 4 and 5 on an i3 and an i10, checking joint signs first.

## License

Apache-2.0 ([LICENSE](LICENSE)). AUBO is the manufacturer's name; this crate does not bundle AUBO's SDK or confer rights to AUBO's software.
