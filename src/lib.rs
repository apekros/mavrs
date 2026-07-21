//! # mavrs
//!
//! Idiomatic async control of `ArduPilot` planes, copters, and quadplanes
//! (including tailsitters) from a companion computer, built on
//! [rust-mavlink](https://github.com/mavlink/rust-mavlink).
//!
//! Physical quantities use [`uom`] and spatial vectors use [`sguaba`] frame
//! markers. Latitude cannot be confused with a scalar, body FRD velocity
//! cannot be mixed with local NED velocity, and wire-scaled integers stay at
//! the `MAVLink` boundary.
//!
//! ## Quick start
//!
//! ```no_run
//! use std::time::Duration;
//! use mavrs::uom::si::{angle::degree, length::meter};
//! use mavrs::{Angle, Length, Plane, Target};
//!
//! # async fn fly() -> mavrs::Result<()> {
//! let plane = Plane::connect("tcpout:127.0.0.1:5760").await?;
//! plane.wait_ready(Duration::from_secs(30)).await?;
//! plane.wait_armable(Duration::from_secs(60)).await?;
//!
//! // Auto takeoff, then fly somewhere in GUIDED.
//! plane
//!     .takeoff(Length::new::<meter>(30.0), Duration::from_secs(120))
//!     .await?;
//! plane
//!     .goto(Target {
//!         latitude: Angle::new::<degree>(-35.360),
//!         longitude: Angle::new::<degree>(149.170),
//!         altitude_above_home: Length::new::<meter>(100.0),
//!     })
//!     .await?;
//! # Ok(())
//! # }
//! ```
//!
//! For a multicopter, use [`Copter`] and [`BodyMotionSetpoint`] for typed
//! body-frame GUIDED control. For a quadplane or tailsitter, use [`QuadPlane`],
//! which adds VTOL takeoff, landing, and flight transitions on top of the full
//! [`Plane`] API.

mod copter;
mod error;
mod mission;
mod modes;
mod plane;
mod quadplane;
mod types;
mod vehicle;

pub use copter::{BodyMotionSetpoint, Copter, CopterMode};
pub use error::{Error, Result};
pub use mission::{Mission, MissionItem};
pub use modes::Mode;
pub use plane::{AcroControl, AcroRateLimits, HeadingReference, Plane};
pub use quadplane::{QuadPlane, Transition};
pub use types::{
    Attitude, Battery, BodyAcceleration, BodyRates, BodyVelocity, FlightData, ImuSample,
    LocalNedFrame, NedOrientation, NedVelocity, Position, Target, VehicleBodyFrame, VehicleState,
};
pub use vehicle::{MavlinkIdentity, MavlinkRole, Vehicle};

/// The generated `ArduPilotMega` dialect, re-exported for anything this crate
/// does not wrap. See [`Vehicle::send`] and [`Vehicle::messages`].
pub use mavlink::dialects::ardupilotmega as dialect;
/// Coordinate and frame types used by this crate's public API.
pub use sguaba;
/// Units-of-measurement types used by this crate's public API.
pub use uom;
/// Common SI quantity aliases used by this crate.
pub use uom::si::f64::{
    Acceleration, Angle, AngularVelocity, ElectricCurrent, ElectricPotential, Frequency, Length,
    Ratio, Velocity,
};
