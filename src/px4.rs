//! PX4 vehicle control.

use std::time::Duration;

use mavlink::dialects::ardupilotmega::{
    MANUAL_CONTROL_DATA, MavCmd, MavFrame, MavMessage, MavVtolState, SpeedType,
};
use tokio::sync::watch;
use uom::ConstZero;
use uom::si::angle::degree;
use uom::si::f64::{Angle, Length, Ratio, Velocity};
use uom::si::length::meter;
use uom::si::ratio::ratio;
use uom::si::velocity::meter_per_second;

use crate::copter::{BodyMotionSetpoint, Copter};
use crate::error::Result;
use crate::plane::AcroControl;
use crate::quadplane::Transition;
use crate::types::{Position, Target};
use crate::vehicle::{MavlinkIdentity, Vehicle};

/// A PX4 flight mode.
///
/// PX4 encodes its main mode in bits 16..24 and its sub-mode in bits 24..32
/// of the heartbeat `custom_mode` field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Px4Mode {
    /// Manual direct control.
    Manual,
    /// Manual altitude control.
    Altitude,
    /// Position hold.
    Position,
    /// Position mode with reduced maximum velocity.
    PositionSlow,
    /// Orbit a point.
    Orbit,
    /// Execute the uploaded mission.
    Mission,
    /// PX4's automatic mode is ready but has not begun navigation.
    AutoReady,
    /// Hold or loiter at the current location.
    Hold,
    /// Return to launch.
    Return,
    /// Automatic takeoff.
    Takeoff,
    /// Automatic landing.
    Land,
    /// Automatic VTOL takeoff and transition.
    VtolTakeoff,
    /// Follow a moving target.
    FollowTarget,
    /// Precision landing.
    PrecisionLand,
    /// Maintain a commanded fixed-wing course.
    GuidedCourse,
    /// Controlled emergency descent.
    Descend,
    /// External navigation mode 1.
    External1,
    /// External navigation mode 2.
    External2,
    /// External navigation mode 3.
    External3,
    /// External navigation mode 4.
    External4,
    /// External navigation mode 5.
    External5,
    /// External navigation mode 6.
    External6,
    /// External navigation mode 7.
    External7,
    /// External navigation mode 8.
    External8,
    /// Acrobatic rate control.
    Acro,
    /// Accept setpoints from a companion computer.
    Offboard,
    /// Stabilized manual control.
    Stabilized,
    /// Fixed-wing altitude and course control.
    AltitudeCruise,
    /// Flight termination mode.
    Termination,
    /// PX4 reports a mode this crate does not know.
    Unknown(u32),
}

impl Default for Px4Mode {
    fn default() -> Self {
        Self::Unknown(0)
    }
}

impl Px4Mode {
    /// Encode the mode for PX4's heartbeat `custom_mode` field.
    #[must_use]
    pub const fn custom_mode(self) -> Option<u32> {
        let (main, sub) = match self {
            Self::Manual => (1, 0),
            Self::Altitude => (2, 0),
            Self::Position => (3, 0),
            Self::PositionSlow => (3, 2),
            Self::Orbit => (3, 1),
            Self::AutoReady => (4, 1),
            Self::Mission => (4, 4),
            Self::Hold => (4, 3),
            Self::Return => (4, 5),
            Self::Takeoff => (4, 2),
            Self::Land => (4, 6),
            Self::VtolTakeoff => (4, 10),
            Self::FollowTarget => (4, 8),
            Self::PrecisionLand => (4, 9),
            Self::External1 => (4, 11),
            Self::External2 => (4, 12),
            Self::External3 => (4, 13),
            Self::External4 => (4, 14),
            Self::External5 => (4, 15),
            Self::External6 => (4, 16),
            Self::External7 => (4, 17),
            Self::External8 => (4, 18),
            Self::GuidedCourse => (4, 19),
            Self::Descend => (4, 20),
            Self::Acro => (5, 0),
            Self::Offboard => (6, 0),
            Self::Stabilized => (7, 0),
            Self::AltitudeCruise => (11, 0),
            Self::Termination => (10, 0),
            Self::Unknown(_) => return None,
        };
        Some(Self::encode(main, sub))
    }

    /// Decode PX4's heartbeat `custom_mode` field.
    #[must_use]
    pub const fn from_custom_mode(value: u32) -> Self {
        let main = ((value >> 16) & 0xff) as u8;
        let sub = ((value >> 24) & 0xff) as u8;
        match (main, sub) {
            (1, 0) => Self::Manual,
            (2, 0) => Self::Altitude,
            (3, 0) => Self::Position,
            (3, 1) => Self::Orbit,
            (3, 2) => Self::PositionSlow,
            (4, 1) => Self::AutoReady,
            (4, 2) => Self::Takeoff,
            (4, 3) => Self::Hold,
            (4, 4) => Self::Mission,
            (4, 5) => Self::Return,
            (4, 6) => Self::Land,
            (4, 10) => Self::VtolTakeoff,
            (4, 8) => Self::FollowTarget,
            (4, 9) => Self::PrecisionLand,
            (4, 11) => Self::External1,
            (4, 12) => Self::External2,
            (4, 13) => Self::External3,
            (4, 14) => Self::External4,
            (4, 15) => Self::External5,
            (4, 16) => Self::External6,
            (4, 17) => Self::External7,
            (4, 18) => Self::External8,
            (4, 19) => Self::GuidedCourse,
            (4, 20) => Self::Descend,
            (5, 0) => Self::Acro,
            (6, 0) => Self::Offboard,
            (7, 0) => Self::Stabilized,
            (11, 0) => Self::AltitudeCruise,
            (10, 0) => Self::Termination,
            _ => Self::Unknown(value),
        }
    }

    /// Split the wire encoding into PX4's command parameters.
    const fn command_parts(self) -> Option<(u8, u8)> {
        let Some(encoded) = self.custom_mode() else {
            return None;
        };
        Some((
            ((encoded >> 16) & 0xff) as u8,
            ((encoded >> 24) & 0xff) as u8,
        ))
    }

    /// Pack main and sub modes into PX4's heartbeat representation.
    const fn encode(main: u8, sub: u8) -> u32 {
        ((main as u32) << 16) | ((sub as u32) << 24)
    }
}

impl std::fmt::Display for Px4Mode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Manual => "MANUAL",
            Self::Altitude => "ALTITUDE",
            Self::Position => "POSITION",
            Self::PositionSlow => "POSITION_SLOW",
            Self::Orbit => "ORBIT",
            Self::Mission => "MISSION",
            Self::AutoReady => "AUTO_READY",
            Self::Hold => "HOLD",
            Self::Return => "RETURN",
            Self::Takeoff => "TAKEOFF",
            Self::Land => "LAND",
            Self::VtolTakeoff => "VTOL_TAKEOFF",
            Self::FollowTarget => "FOLLOW_TARGET",
            Self::PrecisionLand => "PRECISION_LAND",
            Self::GuidedCourse => "GUIDED_COURSE",
            Self::Descend => "DESCEND",
            Self::External1 => "EXTERNAL1",
            Self::External2 => "EXTERNAL2",
            Self::External3 => "EXTERNAL3",
            Self::External4 => "EXTERNAL4",
            Self::External5 => "EXTERNAL5",
            Self::External6 => "EXTERNAL6",
            Self::External7 => "EXTERNAL7",
            Self::External8 => "EXTERNAL8",
            Self::Acro => "ACRO",
            Self::Offboard => "OFFBOARD",
            Self::Stabilized => "STABILIZED",
            Self::AltitudeCruise => "ALTITUDE_CRUISE",
            Self::Termination => "TERMINATION",
            Self::Unknown(value) => return write!(f, "UNKNOWN({value})"),
        })
    }
}

/// Common control surface for PX4 multicopters, fixed-wing aircraft, and VTOLs.
#[derive(Debug, Clone)]
pub struct Px4Vehicle {
    /// Underlying generic `MAVLink` vehicle.
    vehicle: Vehicle,
}

impl std::ops::Deref for Px4Vehicle {
    type Target = Vehicle;

    fn deref(&self) -> &Vehicle {
        &self.vehicle
    }
}

impl Px4Vehicle {
    /// Connect to a PX4 vehicle. See [`Vehicle::connect`] for address formats.
    pub async fn connect(address: &str) -> Result<Self> {
        Ok(Self::from_vehicle(Vehicle::connect(address).await?))
    }

    /// Connect using an explicit `MAVLink` source identity.
    pub async fn connect_with_identity(address: &str, identity: MavlinkIdentity) -> Result<Self> {
        Ok(Self::from_vehicle(
            Vehicle::connect_with_identity(address, identity).await?,
        ))
    }

    /// Wrap an already connected vehicle.
    #[must_use]
    pub const fn from_vehicle(vehicle: Vehicle) -> Self {
        Self { vehicle }
    }

    /// Underlying generic vehicle handle.
    #[must_use]
    pub const fn vehicle(&self) -> &Vehicle {
        &self.vehicle
    }

    /// Current PX4 mode.
    #[must_use]
    pub fn mode(&self) -> Px4Mode {
        Px4Mode::from_custom_mode(self.state().custom_mode)
    }

    /// Change PX4 mode and wait for heartbeat confirmation.
    pub async fn set_mode(&self, mode: Px4Mode) -> Result<()> {
        let Some((main, sub)) = mode.command_parts() else {
            return Err(crate::Error::UnknownPx4Mode);
        };
        self.command_long(
            MavCmd::MAV_CMD_DO_SET_MODE,
            [1.0, f32::from(main), f32::from(sub), 0.0, 0.0, 0.0, 0.0],
        )
        .await?;
        let expected = mode.custom_mode().ok_or(crate::Error::UnknownPx4Mode)?;
        self.wait_state(Duration::from_secs(5), |state| {
            state.custom_mode == expected
        })
        .await
        .map(|_| ())
    }

    /// Arm and take off to a height above home.
    pub async fn takeoff(
        &self,
        altitude_above_home: Length,
        timeout: Duration,
    ) -> Result<Position> {
        self.set_param("MIS_TAKEOFF_ALT", altitude_above_home.get::<meter>() as f32)
            .await?;
        self.arm().await?;
        self.set_mode(Px4Mode::Takeoff).await?;
        self.wait_altitude(altitude_above_home * 0.95, timeout)
            .await
    }

    /// Reposition to a global-relative target.
    pub async fn goto(&self, target: Target) -> Result<()> {
        self.command_int(
            MavCmd::MAV_CMD_DO_REPOSITION,
            MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            // Bit 0 of param 2 asks PX4 to enter Hold while applying the
            // reposition target. Without it PX4 only accepts updates when it
            // is already in Hold.
            [-1.0, 1.0, 0.0, f32::NAN],
            target.latitude_e7(),
            target.longitude_e7(),
            target.altitude_above_home.get::<meter>() as f32,
        )
        .await
    }

    /// Reposition and wait until the target is reached.
    pub async fn goto_and_wait(
        &self,
        target: Target,
        acceptance_radius: Length,
        timeout: Duration,
    ) -> Result<Position> {
        self.goto(target).await?;
        self.wait_position(timeout, |position| {
            position.distance_to(&target) <= acceptance_radius
        })
        .await
    }

    /// Return to launch.
    pub async fn rtl(&self) -> Result<()> {
        self.set_mode(Px4Mode::Return).await
    }

    /// Land at the current position.
    pub async fn land(&self) -> Result<()> {
        self.set_mode(Px4Mode::Land).await
    }

    /// Start the uploaded mission from its beginning.
    pub async fn start_mission(&self) -> Result<()> {
        self.command_long(MavCmd::MAV_CMD_MISSION_START, [0.0; 7])
            .await
    }

    /// Wait until the vehicle reports disarmed.
    pub async fn wait_disarmed(&self, timeout: Duration) -> Result<()> {
        self.wait_state(timeout, |state| !state.armed)
            .await
            .map(|_| ())
    }
}

/// A PX4 multicopter with offboard body-motion control.
#[derive(Debug, Clone)]
pub struct Px4Copter {
    /// Shared PX4 operations.
    px4: Px4Vehicle,
}

impl std::ops::Deref for Px4Copter {
    type Target = Px4Vehicle;

    fn deref(&self) -> &Px4Vehicle {
        &self.px4
    }
}

impl Px4Copter {
    /// Connect to a PX4 multicopter.
    pub async fn connect(address: &str) -> Result<Self> {
        Ok(Self::from_vehicle(Vehicle::connect(address).await?))
    }

    /// Connect using an explicit `MAVLink` source identity.
    pub async fn connect_with_identity(address: &str, identity: MavlinkIdentity) -> Result<Self> {
        Ok(Self::from_vehicle(
            Vehicle::connect_with_identity(address, identity).await?,
        ))
    }

    /// Wrap an already connected PX4 multicopter.
    #[must_use]
    pub const fn from_vehicle(vehicle: Vehicle) -> Self {
        Self {
            px4: Px4Vehicle::from_vehicle(vehicle),
        }
    }

    /// Prime PX4's offboard watchdog, enter Offboard mode, and keep the
    /// setpoint alive while the mode transition is acknowledged.
    pub async fn enter_offboard(&self, setpoint: BodyMotionSetpoint) -> Result<()> {
        for _ in 0..12 {
            self.send_body_motion_unchecked(setpoint).await?;
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
        let change_mode = self.set_mode(Px4Mode::Offboard);
        let keep_alive = async {
            for _ in 0..15 {
                self.send_body_motion_unchecked(setpoint).await?;
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
            Ok::<(), crate::Error>(())
        };
        tokio::try_join!(change_mode, keep_alive)?;
        Ok(())
    }

    /// Enter Offboard mode and start a managed 10 Hz setpoint stream.
    ///
    /// The returned session keeps PX4's Offboard watchdog fed. Dropping the
    /// session closes the stream and asynchronously requests Hold mode. Prefer
    /// [`Px4OffboardSession::stop`] when the caller can await a confirmed exit.
    pub async fn start_offboard(&self, setpoint: BodyMotionSetpoint) -> Result<Px4OffboardSession> {
        self.enter_offboard(setpoint).await?;
        let (setpoints, mut updates) = watch::channel(setpoint);
        let copter = self.clone();
        let task = tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_millis(100));
            interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            loop {
                tokio::select! {
                    biased;
                    changed = updates.changed() => {
                        if changed.is_err() {
                            break;
                        }
                    }
                    _ = interval.tick() => {
                        let setpoint = *updates.borrow();
                        if copter.send_body_motion(setpoint).await.is_err() {
                            break;
                        }
                    }
                }
            }
            let _ = copter.set_mode(Px4Mode::Hold).await;
        });
        Ok(Px4OffboardSession {
            copter: self.clone(),
            setpoints: Some(setpoints),
            task: Some(task),
        })
    }

    /// Send one body-frame velocity/acceleration/yaw setpoint.
    ///
    /// PX4 requires this to be refreshed above 2 Hz or it leaves Offboard
    /// mode. Call [`Self::enter_offboard`] before starting the control loop.
    pub async fn send_body_motion(&self, setpoint: BodyMotionSetpoint) -> Result<()> {
        if self.mode() != Px4Mode::Offboard {
            return Err(crate::Error::WrongPx4Mode {
                required: Px4Mode::Offboard,
                actual: self.mode(),
            });
        }
        self.send_body_motion_unchecked(setpoint).await
    }

    /// Send a setpoint without checking mode, used to prime Offboard mode.
    async fn send_body_motion_unchecked(&self, setpoint: BodyMotionSetpoint) -> Result<()> {
        #[expect(
            deprecated,
            reason = "PX4 currently accepts MAV_FRAME_BODY_NED, not MAV_FRAME_BODY_FRD"
        )]
        let frame = MavFrame::MAV_FRAME_BODY_NED;
        Copter::from_vehicle(self.vehicle().clone())
            .send_body_motion_frame(setpoint, frame)
            .await
    }
}

/// A managed PX4 multicopter Offboard control session.
///
/// It publishes the latest setpoint at 10 Hz, safely above PX4's required
/// 2 Hz minimum. Use [`Self::set_setpoint`] to update the command without
/// building a separate watchdog loop.
pub struct Px4OffboardSession {
    /// Vehicle controlled by this session.
    copter: Px4Copter,
    /// Latest setpoint sent to the publisher task.
    setpoints: Option<watch::Sender<BodyMotionSetpoint>>,
    /// Background setpoint publisher.
    task: Option<tokio::task::JoinHandle<()>>,
}

impl std::fmt::Debug for Px4OffboardSession {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("Px4OffboardSession")
            .field("active", &self.setpoints.is_some())
            .finish_non_exhaustive()
    }
}

impl Px4OffboardSession {
    /// Validate, send, and retain a new body-motion setpoint.
    pub async fn set_setpoint(&self, setpoint: BodyMotionSetpoint) -> Result<()> {
        self.copter.send_body_motion(setpoint).await?;
        self.setpoints.as_ref().map_or_else(
            || Err(crate::Error::ConnectionClosed),
            |setpoints| {
                setpoints.send_replace(setpoint);
                Ok(())
            },
        )
    }

    /// Exit Offboard mode into Hold and wait for the publisher to stop.
    pub async fn stop(mut self) -> Result<()> {
        let mode_result = self.copter.set_mode(Px4Mode::Hold).await;
        self.setpoints.take();
        if let Some(task) = self.task.take() {
            let _ = task.await;
        }
        mode_result
    }
}

impl Drop for Px4OffboardSession {
    fn drop(&mut self) {
        // Closing the channel wakes the detached task, which requests Hold.
        self.setpoints.take();
    }
}

/// A PX4 fixed-wing aircraft.
#[derive(Debug, Clone)]
pub struct Px4Plane {
    /// Shared PX4 operations.
    px4: Px4Vehicle,
}

impl std::ops::Deref for Px4Plane {
    type Target = Px4Vehicle;

    fn deref(&self) -> &Px4Vehicle {
        &self.px4
    }
}

impl Px4Plane {
    /// Connect to a PX4 fixed-wing aircraft.
    pub async fn connect(address: &str) -> Result<Self> {
        Ok(Self::from_vehicle(Vehicle::connect(address).await?))
    }

    /// Connect using an explicit `MAVLink` source identity.
    pub async fn connect_with_identity(address: &str, identity: MavlinkIdentity) -> Result<Self> {
        Ok(Self::from_vehicle(
            Vehicle::connect_with_identity(address, identity).await?,
        ))
    }

    /// Wrap an already connected PX4 fixed-wing aircraft.
    #[must_use]
    pub const fn from_vehicle(vehicle: Vehicle) -> Self {
        Self {
            px4: Px4Vehicle::from_vehicle(vehicle),
        }
    }

    /// Change the fixed-wing target airspeed.
    pub async fn set_airspeed(&self, airspeed: Velocity) -> Result<()> {
        self.command_long(
            MavCmd::MAV_CMD_DO_CHANGE_SPEED,
            [
                f32::from(SpeedType::SPEED_TYPE_AIRSPEED as u8),
                airspeed.get::<meter_per_second>() as f32,
                -1.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        )
        .await
    }

    /// Change the altitude setpoint while in Hold or Guided Course mode.
    pub async fn change_altitude(&self, altitude_above_home: Length) -> Result<()> {
        let position = match self.position() {
            Some(position) => position,
            None => self.wait_position(Duration::from_secs(5), |_| true).await?,
        };
        let altitude_msl =
            position.altitude_msl + altitude_above_home - position.altitude_above_home;
        self.command_long(
            MavCmd::MAV_CMD_DO_CHANGE_ALTITUDE,
            [
                altitude_msl.get::<meter>() as f32,
                f32::from(MavFrame::MAV_FRAME_GLOBAL as u8),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        )
        .await
    }

    /// Enter Guided Course mode and command a course over ground.
    pub async fn change_course(&self, course: Angle) -> Result<()> {
        if self.mode() != Px4Mode::GuidedCourse {
            self.set_mode(Px4Mode::GuidedCourse).await?;
        }
        self.command_long(
            MavCmd::MAV_CMD_GUIDED_CHANGE_HEADING,
            [0.0, course.get::<degree>() as f32, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await
    }

    /// Prime PX4's `MAVLink` manual-control source and enter ACRO mode.
    ///
    /// `COM_RC_IN_MODE` must permit this `MAVLink` instance. This method does
    /// not rewrite the operator's input-priority configuration.
    pub async fn enter_acro_control(&self, throttle: Ratio) -> Result<()> {
        let neutral = AcroControl::new(Ratio::ZERO, Ratio::ZERO, Ratio::ZERO, throttle)?;
        for _ in 0..5 {
            self.send_acro_control(neutral).await?;
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
        let change_mode = self.set_mode(Px4Mode::Acro);
        let keep_alive = async {
            for _ in 0..15 {
                self.send_acro_control(neutral).await?;
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
            Ok::<(), crate::Error>(())
        };
        tokio::try_join!(change_mode, keep_alive)?;
        Ok(())
    }

    /// Send normalized PX4 ACRO pilot inputs through `MANUAL_CONTROL`.
    ///
    /// Refresh this message at the control-loop rate while ACRO is active.
    pub async fn send_acro_control(&self, control: AcroControl) -> Result<()> {
        let control = AcroControl::new(control.roll, control.pitch, control.yaw, control.throttle)?;
        let (target_system, _) = self.target().unwrap_or((0, 0));
        self.send(&MavMessage::MANUAL_CONTROL(MANUAL_CONTROL_DATA {
            x: normalized_axis(-control.pitch),
            y: normalized_axis(control.roll),
            z: normalized_throttle(control.throttle),
            r: normalized_axis(control.yaw),
            buttons: 0,
            target: target_system,
        }))
        .await
    }

    /// Hand ACRO control back to PX4 and enter Hold mode.
    pub async fn release_acro_control(&self) -> Result<()> {
        let neutral = AcroControl::new(Ratio::ZERO, Ratio::ZERO, Ratio::ZERO, Ratio::ZERO)?;
        self.send_acro_control(neutral).await?;
        self.set_mode(Px4Mode::Hold).await
    }
}

/// Convert a checked normalized signed axis to `MAVLink`'s integer range.
fn normalized_axis(value: Ratio) -> i16 {
    (value.get::<ratio>() * 1000.0).round() as i16
}

/// Convert a checked normalized throttle to `MAVLink`'s legacy 0..=1000 range.
fn normalized_throttle(value: Ratio) -> i16 {
    (value.get::<ratio>() * 1000.0).round() as i16
}

/// A PX4 VTOL aircraft.
#[derive(Debug, Clone)]
pub struct Px4Vtol {
    /// Shared PX4 operations.
    px4: Px4Vehicle,
}

impl std::ops::Deref for Px4Vtol {
    type Target = Px4Vehicle;

    fn deref(&self) -> &Px4Vehicle {
        &self.px4
    }
}

impl Px4Vtol {
    /// Connect to a PX4 VTOL.
    pub async fn connect(address: &str) -> Result<Self> {
        Ok(Self::from_vehicle(Vehicle::connect(address).await?))
    }

    /// Connect using an explicit `MAVLink` source identity.
    pub async fn connect_with_identity(address: &str, identity: MavlinkIdentity) -> Result<Self> {
        Ok(Self::from_vehicle(
            Vehicle::connect_with_identity(address, identity).await?,
        ))
    }

    /// Wrap an already connected PX4 VTOL.
    #[must_use]
    pub const fn from_vehicle(vehicle: Vehicle) -> Self {
        Self {
            px4: Px4Vehicle::from_vehicle(vehicle),
        }
    }

    /// Transition between multicopter and fixed-wing flight.
    pub async fn transition(&self, to: Transition) -> Result<()> {
        self.command_long(
            MavCmd::MAV_CMD_DO_VTOL_TRANSITION,
            [to.vtol_state(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await?;
        let expected = match to {
            Transition::Hover => MavVtolState::MAV_VTOL_STATE_MC,
            Transition::ForwardFlight => MavVtolState::MAV_VTOL_STATE_FW,
        };
        self.wait_vtol_state(expected, Duration::from_secs(30))
            .await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use anyhow::Result as TestResult;

    #[test]
    fn px4_modes_roundtrip() -> TestResult<()> {
        let modes = [
            Px4Mode::Manual,
            Px4Mode::Altitude,
            Px4Mode::Position,
            Px4Mode::PositionSlow,
            Px4Mode::Orbit,
            Px4Mode::AutoReady,
            Px4Mode::Mission,
            Px4Mode::Hold,
            Px4Mode::Return,
            Px4Mode::Takeoff,
            Px4Mode::Land,
            Px4Mode::VtolTakeoff,
            Px4Mode::FollowTarget,
            Px4Mode::PrecisionLand,
            Px4Mode::GuidedCourse,
            Px4Mode::Descend,
            Px4Mode::External1,
            Px4Mode::External2,
            Px4Mode::External3,
            Px4Mode::External4,
            Px4Mode::External5,
            Px4Mode::External6,
            Px4Mode::External7,
            Px4Mode::External8,
            Px4Mode::Acro,
            Px4Mode::Offboard,
            Px4Mode::Stabilized,
            Px4Mode::AltitudeCruise,
            Px4Mode::Termination,
        ];
        for mode in modes {
            let encoded = mode.custom_mode().ok_or(crate::Error::UnknownPx4Mode)?;
            assert_eq!(Px4Mode::from_custom_mode(encoded), mode);
        }
        let unknown = Px4Mode::from_custom_mode(0xfeed_beef);
        assert_eq!(unknown, Px4Mode::Unknown(0xfeed_beef));
        assert!(unknown.custom_mode().is_none());
        Ok(())
    }
}
