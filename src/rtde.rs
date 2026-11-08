//! RTDE on TCP port 30010: JSON arrays.
//!
//! AUBO's RTDE guide: a setup message is `[100, {...}]` whose object names a
//! channel (`"chanel"`, the SDK's spelling, 0 to 99), a `frequency`, the
//! `segments` (variable names of the RTDE recipe table), `to_server` (false
//! to subscribe, true to publish inputs) and `trigger` (0 periodic; 1, on
//! change, is not implemented). A subscription is cancelled by the same
//! message with frequency 0 and no segments. Data packets are
//! `[channel, value, ...]`, one value per segment in the order subscribed.

use std::io::Write;

use serde_json::Value;

use crate::AuboError;

/// The controller's RTDE port.
pub const PORT: u16 = 30010;

/// The first element of a setup message.
pub const SETUP: u64 = 100;

/// Writes a setup message in the member order of AUBO's examples, the
/// frequency with its decimal point (`50.0`), without whitespace: periodic
/// (`trigger` 0), subscribing unless `to_server`.
pub fn encode_setup(
    out: &mut Vec<u8>,
    channel: u8,
    frequency: f64,
    segments: &[&str],
    to_server: bool,
) -> Result<(), AuboError> {
    if !(frequency.is_finite() && frequency >= 0.0) || channel >= 100 {
        return Err(AuboError::InvalidOptions("rtde setup"));
    }
    write!(
        out,
        "[{SETUP},{{\"chanel\":{channel},\"frequency\":{frequency:?},\"segments\":["
    )?;
    for (i, segment) in segments.iter().enumerate() {
        if i > 0 {
            out.push(b',');
        }
        // Segment names are identifiers: nothing to escape.
        write!(out, "\"{segment}\"")?;
    }
    write!(out, "],\"to_server\":{to_server},\"trigger\":0}}]")?;
    Ok(())
}

/// The values of a data packet on `channel`, or `None` for a packet on
/// another channel. A text that is not an array starting with a channel
/// number is an error.
pub fn data(packet: &Value, channel: u8) -> Result<Option<&[Value]>, AuboError> {
    let malformed = AuboError::Malformed { what: "rtde packet" };
    let items = packet.as_array().ok_or(malformed)?;
    let Some(first) = items.first().and_then(Value::as_u64) else {
        return Err(AuboError::Malformed { what: "rtde packet" });
    };
    Ok((first == u64::from(channel)).then(|| &items[1..]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_packet_is_its_channel_and_values() {
        let packet: Value = serde_json::from_str("[3,1.5,[1,2]]").unwrap();
        let values = data(&packet, 3).unwrap().unwrap();
        assert_eq!(values.len(), 2);
        assert_eq!(values[0], 1.5);
        assert!(data(&packet, 0).unwrap().is_none());
        let bad: Value = serde_json::from_str(r#"{"chanel":0}"#).unwrap();
        assert!(data(&bad, 0).is_err());
    }

    #[test]
    fn a_setup_rejects_what_the_protocol_cannot_carry() {
        let mut out = Vec::new();
        assert!(encode_setup(&mut out, 100, 1.0, &[], false).is_err());
        assert!(encode_setup(&mut out, 0, f64::NAN, &[], false).is_err());
    }
}
