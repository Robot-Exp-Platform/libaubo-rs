//! The driver's encoders against the messages of AUBO's own documentation:
//! byte for byte where the documentation writes them without whitespace,
//! ignoring whitespace where it adds some. Sources (see the README):
//! the SDK reference (`MotionControl`, `RobotManage`, `RobotConfig`), the
//! JSON-RPC guide (application note 33) and the RTDE protocol page.

use libaubo::{
    AuboArm, I3, I5, I10, Param, RobotMode, SafetyMode,
    rpc::encode_request,
    rtde::{self, encode_setup},
};
use roplat_behavior::conformance::check_joints;

fn request(scope: &str, name: &str, params: &[Param<'_>], id: u64) -> String {
    let mut out = Vec::new();
    encode_request(&mut out, scope, name, params, id).unwrap();
    String::from_utf8(out).unwrap()
}

#[test]
fn servo_joint_is_the_reference_example() {
    let params = [
        Param::Floats(&[0.0; 6]),
        Param::Float(0.0),
        Param::Float(0.0),
        Param::Float(10.0),
        Param::Float(0.0),
        Param::Float(0.0),
    ];
    assert_eq!(
        request("rob1", "MotionControl.servoJoint", &params, 1),
        r#"{"jsonrpc":"2.0","method":"rob1.MotionControl.servoJoint","params":[[0,0,0,0,0,0],0,0,10,0,0],"id":1}"#
    );
}

#[test]
fn mode_and_stop_calls_are_the_reference_examples() {
    assert_eq!(
        request(
            "rob1",
            "MotionControl.setServoModeSelect",
            &[Param::Int(0)],
            1
        ),
        r#"{"jsonrpc":"2.0","method":"rob1.MotionControl.setServoModeSelect","params":[0],"id":1}"#
    );
    assert_eq!(
        request("rob1", "MotionControl.stopJoint", &[Param::Float(31.0)], 1),
        r#"{"jsonrpc":"2.0","method":"rob1.MotionControl.stopJoint","params":[31],"id":1}"#
    );
    assert_eq!(
        request(
            "rob1",
            "MotionControl.setSpeedFraction",
            &[Param::Float(0.8)],
            1
        ),
        r#"{"jsonrpc":"2.0","method":"rob1.MotionControl.setSpeedFraction","params":[0.8],"id":1}"#
    );
    assert_eq!(
        request(
            "rob1",
            "MotionControl.setServoMode",
            &[Param::Bool(true)],
            1
        ),
        r#"{"jsonrpc":"2.0","method":"rob1.MotionControl.setServoMode","params":[true],"id":1}"#
    );
    assert_eq!(
        request(
            "rob1",
            "RobotConfig.getKinematicsParam",
            &[Param::Bool(true)],
            1
        ),
        r#"{"jsonrpc":"2.0","method":"rob1.RobotConfig.getKinematicsParam","params":[true],"id":1}"#
    );
}

#[test]
fn guide_calls_are_the_guide_examples() {
    assert_eq!(
        request("", "getRobotNames", &[], 1),
        r#"{"jsonrpc":"2.0","method":"getRobotNames","params":[],"id":1}"#
    );
    assert_eq!(
        request("", "SystemInfo.getControlSoftwareVersionCode", &[], 2),
        r#"{"jsonrpc":"2.0","method":"SystemInfo.getControlSoftwareVersionCode","params":[],"id":2}"#
    );
    assert_eq!(
        request("rob1", "RobotState.getRobotModeType", &[], 9),
        r#"{"jsonrpc":"2.0","method":"rob1.RobotState.getRobotModeType","params":[],"id":9}"#
    );
    assert_eq!(
        request("rob1", "RobotManage.poweron", &[], 10),
        r#"{"jsonrpc":"2.0","method":"rob1.RobotManage.poweron","params":[],"id":10}"#
    );
    assert_eq!(
        request("rob1", "RobotManage.startup", &[], 11),
        r#"{"jsonrpc":"2.0","method":"rob1.RobotManage.startup","params":[],"id":11}"#
    );
}

#[test]
fn move_joint_is_the_reference_example_without_its_spaces() {
    let q = [-2.05177, -0.400292, 1.19625, 0.0285152, 1.57033, -2.28774];
    let params = [
        Param::Floats(&q),
        Param::Float(0.3),
        Param::Float(0.3),
        Param::Float(0.0),
        Param::Float(0.0),
    ];
    let documented = r#"{"jsonrpc":"2.0","method":"rob1.MotionControl.moveJoint","params":[[-2.05177, -0.400292, 1.19625, 0.0285152, 1.57033, -2.28774],0.3,0.3,0,0],"id":1}"#;
    assert_eq!(
        request("rob1", "MotionControl.moveJoint", &params, 1),
        documented.replace(' ', "")
    );
}

fn setup(channel: u8, frequency: f64, segments: &[&str], to_server: bool) -> String {
    let mut out = Vec::new();
    encode_setup(&mut out, channel, frequency, segments, to_server).unwrap();
    String::from_utf8(out).unwrap()
}

#[test]
fn rtde_setups_are_the_protocol_examples() {
    assert_eq!(
        setup(0, 50.0, &["R1_actual_q"], false),
        r#"[100,{"chanel":0,"frequency":50.0,"segments":["R1_actual_q"],"to_server":false,"trigger":0}]"#
    );
    assert_eq!(
        setup(0, 0.0, &[], false),
        r#"[100,{"chanel":0,"frequency":0.0,"segments":[],"to_server":false,"trigger":0}]"#
    );
    assert_eq!(
        setup(
            1,
            1.0,
            &["input_float_registers_0", "input_double_registers_1"],
            true
        ),
        r#"[100,{"chanel":1,"frequency":1.0,"segments":["input_float_registers_0","input_double_registers_1"],"to_server":true,"trigger":0}]"#
    );
}

#[test]
fn the_rtde_data_example_is_one_channel_and_one_value_per_segment() {
    let packet: serde_json::Value = serde_json::from_str(
        "[0,[-0.027531569059195515,-0.28143723119618147,1.6961810563264004,\
         0.4177235477634482,1.5627876064666573,-0.03521353669519756]]",
    )
    .unwrap();
    let values = rtde::data(&packet, 0).unwrap().unwrap();
    assert_eq!(values.len(), 1);
    assert_eq!(values[0][2], 1.6961810563264004);
    let pushed: serde_json::Value = serde_json::from_str("[1,3.1,4.1]").unwrap();
    assert_eq!(rtde::data(&pushed, 1).unwrap().unwrap().len(), 2);
}

#[test]
fn documented_mode_names_are_known() {
    // getRobotModeType answers "Idle" in the guide and "Running" in the
    // reference; getSafetyModeType answers "Normal".
    assert_eq!(RobotMode::from_name("Idle"), Some(RobotMode::Idle));
    assert_eq!(RobotMode::from_name("Running"), Some(RobotMode::Running));
    assert_eq!(SafetyMode::from_name("Normal"), Some(SafetyMode::Normal));
}

#[test]
fn the_models_declare_coherent_envelopes() {
    check_joints::<I3, 6>().unwrap();
    check_joints::<I5, 6>().unwrap();
    check_joints::<I10, 6>().unwrap();
    check_joints::<AuboArm<I5>, 6>().unwrap();
}
