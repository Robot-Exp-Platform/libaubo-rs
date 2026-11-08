//! JSON on a byte stream. Both controller ports send JSON texts back to back
//! with nothing between them: no length, no delimiter.

use std::io::{self, Read, Write};

use serde_json::Value;

use crate::AuboError;

/// One parameter of a JSON-RPC call, written as the AUBO documentation
/// writes it: a float in Rust's shortest decimal form, so `0.0` is `0` and
/// `0.3` is `0.3`, never an exponent.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Param<'a> {
    Float(f64),
    Int(i64),
    Bool(bool),
    Floats(&'a [f64]),
}

impl Param<'_> {
    fn write(&self, out: &mut Vec<u8>) -> Result<(), AuboError> {
        match *self {
            Self::Float(v) => float(out, v)?,
            Self::Int(v) => write!(out, "{v}")?,
            Self::Bool(v) => write!(out, "{v}")?,
            Self::Floats(values) => {
                out.push(b'[');
                for (i, &v) in values.iter().enumerate() {
                    if i > 0 {
                        out.push(b',');
                    }
                    float(out, v)?;
                }
                out.push(b']');
            }
        }
        Ok(())
    }
}

/// A finite float in Rust's `Display` form, which is never exponential.
/// JSON has no NaN or infinity: those are refused before anything is
/// written.
fn float(out: &mut Vec<u8>, v: f64) -> Result<(), AuboError> {
    if !v.is_finite() {
        return Err(AuboError::NotFinite);
    }
    write!(out, "{v}")?;
    Ok(())
}

/// Writes a JSON-RPC 2.0 request in the member order of AUBO's examples:
/// `{"jsonrpc":"2.0","method":"<scope>.<name>","params":[...],"id":<id>}`,
/// without whitespace. `scope` is the robot name for robot calls
/// (`rob1.RobotState.getJointPositions`) and empty for controller calls
/// (`getRobotNames`, `SystemInfo.getControlSoftwareVersionCode`). Nothing is
/// written when a parameter is not finite.
pub fn encode_request(
    out: &mut Vec<u8>,
    scope: &str,
    name: &str,
    params: &[Param<'_>],
    id: u64,
) -> Result<(), AuboError> {
    let start = out.len();
    let result = (|| {
        out.extend_from_slice(b"{\"jsonrpc\":\"2.0\",\"method\":\"");
        if !scope.is_empty() {
            out.extend_from_slice(scope.as_bytes());
            out.push(b'.');
        }
        out.extend_from_slice(name.as_bytes());
        out.extend_from_slice(b"\",\"params\":[");
        for (i, p) in params.iter().enumerate() {
            if i > 0 {
                out.push(b',');
            }
            p.write(out)?;
        }
        write!(out, "],\"id\":{id}}}")?;
        Ok(())
    })();
    if result.is_err() {
        out.truncate(start);
    }
    result
}

/// Bytes read from a stream, cut into JSON texts.
#[derive(Debug, Default)]
pub(crate) struct Framer {
    buffer: Vec<u8>,
    start: usize,
}

/// How much one read takes at most.
const CHUNK: usize = 64 * 1024;

impl Framer {
    pub(crate) fn new() -> Self {
        Self { buffer: Vec::with_capacity(CHUNK), start: 0 }
    }

    /// Reads once from `stream`; `Ok(0)` at the end of the stream.
    pub(crate) fn fill(&mut self, stream: &mut impl Read) -> io::Result<usize> {
        if self.start > 0 && self.start * 2 >= self.buffer.len() {
            self.buffer.drain(..self.start);
            self.start = 0;
        }
        let end = self.buffer.len();
        self.buffer.resize(end + CHUNK, 0);
        let read = stream.read(&mut self.buffer[end..]);
        self.buffer.truncate(end + *read.as_ref().unwrap_or(&0));
        read
    }

    /// The next complete JSON text received, if there is one. A text that
    /// cannot be JSON is an error; the stream cannot be resynchronised
    /// after it.
    pub(crate) fn next_value(&mut self) -> Result<Option<Value>, AuboError> {
        let rest = &self.buffer[self.start..];
        let mut texts = serde_json::Deserializer::from_slice(rest).into_iter::<Value>();
        match texts.next() {
            None => {
                // Only whitespace so far.
                self.start = self.buffer.len();
                Ok(None)
            }
            Some(Ok(value)) => {
                self.start += texts.byte_offset();
                Ok(Some(value))
            }
            Some(Err(e)) if e.is_eof() => Ok(None),
            Some(Err(_)) => Err(AuboError::Malformed { what: "json text" }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn floats_are_written_as_the_documentation_writes_them() {
        let mut out = Vec::new();
        let params = [
            Param::Floats(&[0.0, -0.2618, 1.74533]),
            Param::Float(10.0),
            Param::Int(1),
            Param::Bool(true),
        ];
        encode_request(&mut out, "rob1", "A.b", &params, 7).unwrap();
        assert_eq!(
            String::from_utf8(out).unwrap(),
            r#"{"jsonrpc":"2.0","method":"rob1.A.b","params":[[0,-0.2618,1.74533],10,1,true],"id":7}"#
        );
    }

    #[test]
    fn a_request_with_a_non_finite_parameter_writes_nothing() {
        let mut out = b"kept".to_vec();
        let result = encode_request(&mut out, "", "m", &[Param::Float(f64::NAN)], 1);
        assert!(matches!(result, Err(AuboError::NotFinite)));
        assert_eq!(out, b"kept");
    }

    #[test]
    fn texts_back_to_back_are_cut_apart_across_reads() {
        let bytes = br#"{"id":1,"result":0}  [0,[1.5,2]]{"id":2,"res"#;
        let mut framer = Framer::new();
        framer.fill(&mut &bytes[..]).unwrap();
        assert_eq!(framer.next_value().unwrap().unwrap()["id"], 1);
        assert_eq!(framer.next_value().unwrap().unwrap()[1][0], 1.5);
        assert!(framer.next_value().unwrap().is_none());
        framer.fill(&mut &br#"ult":3}"#[..]).unwrap();
        assert_eq!(framer.next_value().unwrap().unwrap()["result"], 3);
        assert!(framer.next_value().unwrap().is_none());
    }

    #[test]
    fn garbage_is_an_error() {
        let mut framer = Framer::new();
        framer.fill(&mut &b"{]"[..]).unwrap();
        assert!(matches!(
            framer.next_value(),
            Err(AuboError::Malformed { .. })
        ));
    }
}
