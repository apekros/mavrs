//! Error types for the crate.

use mavlink::dialects::ardupilotmega::{MavCmd, MavResult};

/// Convenience alias used across the crate.
pub type Result<T, E = Error> = core::result::Result<T, E>;

/// Everything that can go wrong when talking to a vehicle.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// The underlying transport failed (TCP/UDP/serial).
    #[error("transport error: {0}")]
    Io(#[from] std::io::Error),

    /// A send on the mavlink connection failed.
    #[error("failed to send message: {0}")]
    Send(#[from] mavlink::error::MessageWriteError),

    /// The background IO task is gone, the connection is dead.
    #[error("connection closed")]
    ConnectionClosed,

    /// We waited too long for something.
    #[error("timed out waiting for {what} after {after:?}")]
    Timeout {
        /// Human readable description of what we were waiting for.
        what: &'static str,
        /// How long we waited before giving up.
        after: std::time::Duration,
    },

    /// The autopilot acknowledged a command with a non-success result.
    #[error("command {command:?} rejected with {result:?}")]
    CommandRejected {
        /// The command that was rejected.
        command: MavCmd,
        /// The result the autopilot reported.
        result: MavResult,
    },

    /// The autopilot did not switch to the requested mode.
    #[error("mode change to {requested} not confirmed, vehicle reports {actual}")]
    ModeChangeFailed {
        /// The mode we asked for.
        requested: crate::Mode,
        /// The mode the vehicle is actually in.
        actual: crate::Mode,
    },

    /// A parameter operation referenced a name longer than 16 bytes.
    #[error("parameter name {0:?} exceeds 16 characters")]
    ParamNameTooLong(String),

    /// The autopilot rejected or ignored a parameter write.
    #[error("parameter {name} readback mismatch: wrote {wrote}, vehicle reports {got}")]
    ParamSetMismatch {
        /// Parameter name.
        name: String,
        /// Value we wrote.
        wrote: f32,
        /// Value the vehicle echoed back.
        got: f32,
    },

    /// Mission upload was rejected by the autopilot.
    #[error("mission transfer failed: {0:?}")]
    MissionRejected(mavlink::dialects::ardupilotmega::MavMissionResult),

    /// The vehicle kept requesting mission items without ever completing the
    /// transfer.
    #[error("mission transfer did not complete after {messages} protocol messages")]
    MissionTransferStalled {
        /// How many mission protocol messages were exchanged before giving up.
        messages: usize,
    },

    /// `MAVLink` reserves source system id zero.
    #[error("invalid MAVLink source system id {0}, expected 1..=255")]
    InvalidSystemId(u8),

    /// `CopterMode::Unknown` cannot be commanded.
    #[error("cannot command an unknown ArduCopter mode")]
    UnknownCopterMode,

    /// A Copter setpoint was sent outside the mode that accepts it.
    #[error("body motion requires Copter mode {required}, vehicle is in {actual}")]
    WrongCopterMode {
        /// Required flight mode.
        required: crate::CopterMode,
        /// Current flight mode.
        actual: crate::CopterMode,
    },

    /// A body motion setpoint selected no controlled dimensions.
    #[error("body motion setpoint does not contain velocity, acceleration, or yaw control")]
    EmptyBodyMotionSetpoint,

    /// `MAVLink` control setpoints must be finite.
    #[error("{field} setpoint is not finite: {value}")]
    NonFiniteSetpoint {
        /// Invalid setpoint field.
        field: &'static str,
        /// Supplied value.
        value: f64,
    },

    /// A normalized control input was outside its valid range.
    #[error("{field} control input {value} is outside {min}..={max}")]
    ControlOutOfRange {
        /// Name of the invalid field.
        field: &'static str,
        /// Supplied value.
        value: f64,
        /// Minimum accepted value.
        min: f64,
        /// Maximum accepted value.
        max: f64,
    },

    /// RC overrides would be ignored because this connection is not the
    /// configured GCS.
    #[error(
        "RC override authority belongs to MAVLink system {configured}, connection uses {actual}"
    )]
    RcOverrideAuthority {
        /// `MAV_GCS_SYSID` reported by the autopilot.
        configured: u8,
        /// Source system id used by this connection.
        actual: u8,
    },
}
