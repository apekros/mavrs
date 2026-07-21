//! `QuadPlane` (including tailsitter) control.
//!
//! A quadplane is a fixed-wing plane with VTOL motors. It flies all the
//! regular plane modes plus the `Q*` VTOL modes, and can transition between
//! hover and forward flight. Tailsitters are quadplanes as far as control is
//! concerned; the airframe geometry differs but the `MAVLink` interface is the
//! same.

use std::time::Duration;

use mavlink::dialects::ardupilotmega::{MavCmd, MavFrame, MavVtolState};

use crate::Mode;
use crate::error::Result;
use crate::plane::Plane;
use crate::types::Position;
use crate::vehicle::Vehicle;
use uom::si::f64::Length;
use uom::si::length::meter;

/// Direction of a commanded VTOL transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Transition {
    /// Transition to hover (multicopter) flight.
    Hover,
    /// Transition to forward (fixed-wing) flight.
    ForwardFlight,
}

impl Transition {
    /// The `MAV_VTOL_STATE` value for this transition.
    const fn vtol_state(self) -> f32 {
        let state = match self {
            Self::Hover => MavVtolState::MAV_VTOL_STATE_MC,
            Self::ForwardFlight => MavVtolState::MAV_VTOL_STATE_FW,
        };
        state as u8 as f32
    }
}

/// A quadplane or tailsitter `ArduPlane` vehicle.
///
/// Derefs to [`Plane`], which derefs to [`Vehicle`], so the entire
/// fixed-wing and generic API is available too.
#[derive(Debug, Clone)]
pub struct QuadPlane {
    /// The fixed-wing API this quadplane extends.
    plane: Plane,
}

impl std::ops::Deref for QuadPlane {
    type Target = Plane;

    fn deref(&self) -> &Plane {
        &self.plane
    }
}

impl QuadPlane {
    /// Connect to a quadplane. See [`Vehicle::connect`] for address formats.
    pub async fn connect(address: &str) -> Result<Self> {
        Ok(Self {
            plane: Plane::connect(address).await?,
        })
    }

    /// Wrap an already-connected vehicle.
    #[must_use]
    pub const fn from_vehicle(vehicle: Vehicle) -> Self {
        Self {
            plane: Plane::from_vehicle(vehicle),
        }
    }

    /// The fixed-wing view of this vehicle.
    #[must_use]
    pub const fn plane(&self) -> &Plane {
        &self.plane
    }

    /// Vertical takeoff in GUIDED mode to `altitude_above_home`.
    ///
    /// Switches to GUIDED, arms, and commands a VTOL takeoff. Returns once
    /// the vehicle reaches 95% of the requested altitude.
    pub async fn vtol_takeoff(
        &self,
        altitude_above_home: Length,
        timeout: Duration,
    ) -> Result<Position> {
        if self.mode() != Mode::Guided {
            self.set_mode(Mode::Guided).await?;
        }
        self.arm().await?;
        // ArduPlane wants NAV_TAKEOFF as COMMAND_INT in LOCAL_OFFSET_NED
        // with z = -altitude (down is positive in NED).
        self.command_int(
            MavCmd::MAV_CMD_NAV_TAKEOFF,
            MavFrame::MAV_FRAME_LOCAL_OFFSET_NED,
            [0.0; 4],
            0,
            0,
            -(altitude_above_home.get::<meter>() as f32),
        )
        .await?;
        self.wait_altitude(altitude_above_home * 0.95, timeout)
            .await
    }

    /// Command a transition between hover and forward flight.
    ///
    /// `ArduPilot` only accepts this in AUTO mission flight (it overrides how
    /// the current mission leg is flown). In GUIDED there is nothing to
    /// command: a [`Plane::goto`] after a VTOL takeoff automatically
    /// transitions to forward flight, and [`Self::hover`] (QLOITER) brings it
    /// back to hovering.
    pub async fn transition(&self, to: Transition) -> Result<()> {
        self.command_long(
            MavCmd::MAV_CMD_DO_VTOL_TRANSITION,
            [to.vtol_state(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await
    }

    /// Hover in place: switch to QLOITER (GPS position hold).
    pub async fn hover(&self) -> Result<()> {
        self.set_mode(Mode::QLoiter).await
    }

    /// Land vertically at the current position (QLAND) and wait for
    /// disarm.
    pub async fn vtol_land(&self, timeout: Duration) -> Result<()> {
        self.set_mode(Mode::QLand).await?;
        self.wait_disarmed(timeout).await
    }

    /// Return to launch and land vertically (QRTL).
    pub async fn vtol_rtl(&self) -> Result<()> {
        self.set_mode(Mode::QRtl).await
    }
}
