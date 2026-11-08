//! JSON-RPC 2.0 on TCP port 30004.
//!
//! AUBO's guide to the protocol lists the ports (TCP 30004; HTTP and
//! WebSocket 9012) and the members of requests, results and errors, and
//! shows each SDK function as a call: controller functions by name
//! (`getRobotNames`, `SystemInfo.getControlSoftwareVersionCode`), robot
//! functions behind the robot's name (`rob1.RobotManage.poweron`,
//! `rob1.MotionControl.moveJoint`). Parameters are positional, in the order
//! of the C++ signature. On TCP the texts follow each other without a
//! delimiter: the guide sends a bare request from a TCP test tool.
//!
//! [`encode_request`] writes a request; the driver's client reads results
//! with the same framing as RTDE.

use std::{
    io::Write,
    net::{SocketAddr, TcpStream},
    time::{Duration, Instant},
};

use serde_json::Value;

use crate::{AuboError, Param, ReturnCode, json::Framer};

pub use crate::json::encode_request;

/// The controller's JSON-RPC port.
pub const PORT: u16 = 30004;

/// A JSON-RPC connection: one call at a time, each waiting for its result.
pub(crate) struct Rpc {
    stream: TcpStream,
    framer: Framer,
    out: Vec<u8>,
    next_id: u64,
    /// The scope of robot calls, `rob1` on a single-arm controller.
    robot: String,
    timeout: Duration,
}

impl Rpc {
    pub(crate) fn connect(address: SocketAddr, timeout: Duration) -> Result<Self, AuboError> {
        let stream = TcpStream::connect_timeout(&address, timeout)?;
        stream.set_nodelay(true)?;
        stream.set_read_timeout(Some(timeout))?;
        stream.set_write_timeout(Some(timeout))?;
        let mut rpc = Self {
            stream,
            framer: Framer::new(),
            out: Vec::with_capacity(512),
            next_id: 1,
            robot: String::new(),
            timeout,
        };
        let names = rpc.call_controller("getRobotNames", &[])?;
        let first = names
            .as_array()
            .ok_or(AuboError::Malformed { what: "getRobotNames result" })?
            .first()
            .ok_or(AuboError::NoRobot)?;
        rpc.robot = first
            .as_str()
            .ok_or(AuboError::Malformed { what: "getRobotNames result" })?
            .to_owned();
        Ok(rpc)
    }

    pub(crate) fn robot(&self) -> &str {
        &self.robot
    }

    /// Calls a controller function, `name` as it is (`getRobotNames`).
    pub(crate) fn call_controller(
        &mut self,
        name: &'static str,
        params: &[Param<'_>],
    ) -> Result<Value, AuboError> {
        self.call_in(false, name, params)
    }

    /// Calls a function of the robot, `name` without the robot's name
    /// (`RobotState.getJointPositions`).
    pub(crate) fn call(
        &mut self,
        name: &'static str,
        params: &[Param<'_>],
    ) -> Result<Value, AuboError> {
        self.call_in(true, name, params)
    }

    /// The reply is parsed into a `Value`: a few small allocations per call,
    /// next to a network round trip.
    fn call_in(
        &mut self,
        robot: bool,
        name: &'static str,
        params: &[Param<'_>],
    ) -> Result<Value, AuboError> {
        let id = self.next_id;
        self.next_id += 1;
        self.out.clear();
        let scope = if robot { self.robot.as_str() } else { "" };
        encode_request(&mut self.out, scope, name, params, id)?;
        self.stream.write_all(&self.out)?;
        let deadline = Instant::now() + self.timeout;
        loop {
            while let Some(reply) = self.framer.next_value()? {
                if let Some(result) = take_reply(name, id, reply)? {
                    return Ok(result);
                }
            }
            if Instant::now() >= deadline {
                return Err(AuboError::Timeout { waiting_for: name });
            }
            match self.framer.fill(&mut self.stream) {
                Ok(0) => return Err(AuboError::Closed),
                Ok(_) => {}
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) =>
                {
                    return Err(AuboError::Timeout { waiting_for: name });
                }
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
                Err(e) => return Err(AuboError::Io(e)),
            }
        }
    }

    /// Calls a robot function that returns a [`ReturnCode`]; anything but 0
    /// is [`AuboError::Refused`].
    pub(crate) fn command(
        &mut self,
        name: &'static str,
        params: &[Param<'_>],
    ) -> Result<(), AuboError> {
        let code = self.int(name, params)?;
        match ReturnCode(code) {
            ReturnCode::OK => Ok(()),
            code => Err(AuboError::Refused { method: name, code }),
        }
    }

    pub(crate) fn int(
        &mut self,
        name: &'static str,
        params: &[Param<'_>],
    ) -> Result<i64, AuboError> {
        self.call(name, params)?
            .as_i64()
            .ok_or(AuboError::Malformed { what: name })
    }

    pub(crate) fn float(
        &mut self,
        name: &'static str,
        params: &[Param<'_>],
    ) -> Result<f64, AuboError> {
        self.call(name, params)?
            .as_f64()
            .ok_or(AuboError::Malformed { what: name })
    }

    pub(crate) fn bool(&mut self, name: &'static str) -> Result<bool, AuboError> {
        self.call(name, &[])?
            .as_bool()
            .ok_or(AuboError::Malformed { what: name })
    }

    pub(crate) fn joints(&mut self, name: &'static str) -> Result<[f64; 6], AuboError> {
        let value = self.call(name, &[])?;
        six(&value).ok_or(AuboError::Malformed { what: name })
    }

    pub(crate) fn string(&mut self, name: &'static str) -> Result<String, AuboError> {
        match self.call(name, &[])? {
            Value::String(s) => Ok(s),
            _ => Err(AuboError::Malformed { what: name }),
        }
    }
}

/// Six finite numbers.
pub(crate) fn six(value: &Value) -> Option<[f64; 6]> {
    let values = value.as_array().filter(|a| a.len() == 6)?;
    let mut out = [0.0; 6];
    for (o, v) in out.iter_mut().zip(values) {
        *o = v.as_f64().filter(|v| v.is_finite())?;
    }
    Some(out)
}

/// The result of the reply to call `id`, `None` for a reply to another
/// call (one that timed out before), or the call's error.
fn take_reply(method: &'static str, id: u64, reply: Value) -> Result<Option<Value>, AuboError> {
    let Value::Object(mut reply) = reply else {
        return Err(AuboError::Malformed { what: "json-rpc reply" });
    };
    if reply.get("jsonrpc").and_then(Value::as_str) != Some("2.0") {
        return Err(AuboError::Malformed { what: "json-rpc reply" });
    }
    let reply_id = reply.get("id").cloned().unwrap_or(Value::Null);
    let error = reply.remove("error");
    match (reply_id.as_u64(), error) {
        (Some(other), _) if other != id => Ok(None),
        // An error without an id answers a request the controller could not
        // read: the last one.
        (Some(_) | None, Some(error)) => {
            let code = error.get("code").and_then(Value::as_i64);
            let message = error.get("message").and_then(Value::as_str);
            match (code, message) {
                (Some(code), Some(message)) => {
                    Err(AuboError::Rpc { method, code, message: message.to_owned() })
                }
                _ => Err(AuboError::Malformed { what: "json-rpc error" }),
            }
        }
        (Some(_), None) => reply
            .remove("result")
            .map(Some)
            .ok_or(AuboError::Malformed { what: "json-rpc reply" }),
        (None, None) => Err(AuboError::Malformed { what: "json-rpc reply" }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(text: &str) -> Value {
        serde_json::from_str(text).unwrap()
    }

    #[test]
    fn a_result_is_taken_by_its_id() {
        let reply = parse(r#"{"id":4,"jsonrpc":"2.0","result":[0.5]}"#);
        assert_eq!(take_reply("m", 4, reply).unwrap(), Some(parse("[0.5]")));
        let stale = parse(r#"{"id":3,"jsonrpc":"2.0","result":0}"#);
        assert_eq!(take_reply("m", 4, stale).unwrap(), None);
    }

    #[test]
    fn an_error_object_is_an_rpc_error() {
        let reply = parse(
            r#"{"id":2,"jsonrpc":"2.0","error":{"code":-32601,"message":"Method not found."}}"#,
        );
        assert!(matches!(
            take_reply("m", 2, reply),
            Err(AuboError::Rpc { method: "m", code: -32601, .. })
        ));
        let unread = parse(
            r#"{"id":null,"jsonrpc":"2.0","error":{"code":-32700,"message":"Parse error."}}"#,
        );
        assert!(matches!(
            take_reply("m", 2, unread),
            Err(AuboError::Rpc { code: -32700, .. })
        ));
    }

    /// Results as the SDK reference and the JSON-RPC guide print them.
    #[test]
    fn documented_results_are_read() {
        let servo = parse(r#"{"id":1,"jsonrpc":"2.0","result":-13}"#);
        let code = take_reply("servoJoint", 1, servo).unwrap().unwrap();
        assert_eq!(ReturnCode(code.as_i64().unwrap()), ReturnCode::NOT_IN_SERVO);
        let names = parse(r#"{"id":1,"jsonrpc":"2.0","result":["rob1"]}"#);
        assert_eq!(
            take_reply("getRobotNames", 1, names).unwrap(),
            Some(parse(r#"["rob1"]"#))
        );
        let cycle = parse(r#"{"id":1,"jsonrpc":"2.0","result":0.005}"#);
        assert_eq!(
            take_reply("getCycletime", 1, cycle).unwrap(),
            Some(parse("0.005"))
        );
        let speeds = parse(
            r#"{"id":1,"jsonrpc":"2.0","result":[3.892084231947355,3.892084231947355,3.892084231947355,3.1066860685499065,3.1066860685499065,3.1066860685499065]}"#,
        );
        let speeds = take_reply("getJointMaxSpeeds", 1, speeds).unwrap().unwrap();
        assert_eq!(six(&speeds).unwrap()[3], 3.1066860685499065);
        let mode = parse(r#"{"id":9,"jsonrpc":"2.0","result":"Idle"}"#);
        assert_eq!(
            take_reply("getRobotModeType", 9, mode).unwrap(),
            Some(parse(r#""Idle""#))
        );
    }

    #[test]
    fn six_takes_six_finite_numbers() {
        assert_eq!(
            six(&parse("[1,2,3,4,5,6.5]")),
            Some([1.0, 2.0, 3.0, 4.0, 5.0, 6.5])
        );
        assert_eq!(six(&parse("[1,2,3,4,5]")), None);
        assert_eq!(six(&parse("[1,2,3,4,5,\"x\"]")), None);
    }
}
