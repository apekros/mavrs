//! Multicopter control.

use std::time::Duration;

use mavlink::dialects::ardupilotmega::{
    MavCmd, MavFrame, MavMessage, PositionTargetTypemask, SET_POSITION_TARGET_LOCAL_NED_DATA,
};

use crate::error::{Error, Result};
use crate::types::{BodyAcceleration, BodyVelocity, Position, Target};
use crate::vehicle::{MavlinkIdentity, Vehicle};
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angle::radian;
use uom::si::angular_velocity::radian_per_second;
use uom::si::f64::{Angle, AngularVelocity, Length};
use uom::si::length::meter;
use uom::si::velocity::meter_per_second;

/// `ArduCopter` flight mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub enum CopterMode {
    /// Self-leveling manual flight.
    Stabilize,
    /// Manual body-rate control.
    Acro,
    /// Barometric altitude hold.
    AltHold,
    /// Fly the loaded mission.
    Auto,
    /// Accept navigation commands from a companion computer.
    Guided,
    /// GPS position hold.
    Loiter,
    /// Return to launch.
    Rtl,
    /// Circle around a point.
    Circle,
    /// Land at the current position.
    Land,
    /// Drift mode.
    Drift,
    /// Sport mode.
    Sport,
    /// Flip maneuver.
    Flip,
    /// Automatic tuning.
    Autotune,
    /// Position hold.
    PosHold,
    /// Brake to a stop.
    Brake,
    /// Throw launch.
    Throw,
    /// Automatic ADS-B collision avoidance.
    AvoidAdsb,
    /// Guided flight without GPS.
    GuidedNoGps,
    /// Smart return to launch.
    SmartRtl,
    /// Optical-flow position hold.
    FlowHold,
    /// Follow another vehicle.
    Follow,
    /// Zig-zag survey flight.
    ZigZag,
    /// System identification mode.
    SystemId,
    /// Helicopter autorotation.
    Autorotate,
    /// Automatic return-to-launch mission mode.
    AutoRtl,
    /// Turtle recovery mode.
    Turtle,
    /// Direct body-rate ACRO mode.
    RateAcro,
    /// Boot-time or unknown mode value.
    #[default]
    Unknown,
}

impl CopterMode {
    /// `ArduCopter` `custom_mode` value.
    #[must_use]
    pub const fn custom_mode(self) -> Option<u32> {
        match self {
            Self::Stabilize => Some(0),
            Self::Acro => Some(1),
            Self::AltHold => Some(2),
            Self::Auto => Some(3),
            Self::Guided => Some(4),
            Self::Loiter => Some(5),
            Self::Rtl => Some(6),
            Self::Circle => Some(7),
            Self::Land => Some(9),
            Self::Drift => Some(11),
            Self::Sport => Some(13),
            Self::Flip => Some(14),
            Self::Autotune => Some(15),
            Self::PosHold => Some(16),
            Self::Brake => Some(17),
            Self::Throw => Some(18),
            Self::AvoidAdsb => Some(19),
            Self::GuidedNoGps => Some(20),
            Self::SmartRtl => Some(21),
            Self::FlowHold => Some(22),
            Self::Follow => Some(23),
            Self::ZigZag => Some(24),
            Self::SystemId => Some(25),
            Self::Autorotate => Some(26),
            Self::AutoRtl => Some(27),
            Self::Turtle => Some(28),
            Self::RateAcro => Some(29),
            Self::Unknown => None,
        }
    }

    /// Decode an `ArduCopter` heartbeat mode.
    #[must_use]
    pub const fn from_custom_mode(value: u32) -> Self {
        match value {
            0 => Self::Stabilize,
            1 => Self::Acro,
            2 => Self::AltHold,
            3 => Self::Auto,
            4 => Self::Guided,
            5 => Self::Loiter,
            6 => Self::Rtl,
            7 => Self::Circle,
            9 => Self::Land,
            11 => Self::Drift,
            13 => Self::Sport,
            14 => Self::Flip,
            15 => Self::Autotune,
            16 => Self::PosHold,
            17 => Self::Brake,
            18 => Self::Throw,
            19 => Self::AvoidAdsb,
            20 => Self::GuidedNoGps,
            21 => Self::SmartRtl,
            22 => Self::FlowHold,
            23 => Self::Follow,
            24 => Self::ZigZag,
            25 => Self::SystemId,
            26 => Self::Autorotate,
            27 => Self::AutoRtl,
            28 => Self::Turtle,
            29 => Self::RateAcro,
            _ => Self::Unknown,
        }
    }
}

impl std::fmt::Display for CopterMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = match self {
            Self::Stabilize => "STABILIZE",
            Self::Acro => "ACRO",
            Self::AltHold => "ALT_HOLD",
            Self::Auto => "AUTO",
            Self::Guided => "GUIDED",
            Self::Loiter => "LOITER",
            Self::Rtl => "RTL",
            Self::Circle => "CIRCLE",
            Self::Land => "LAND",
            Self::Drift => "DRIFT",
            Self::Sport => "SPORT",
            Self::Flip => "FLIP",
            Self::Autotune => "AUTOTUNE",
            Self::PosHold => "POSHOLD",
            Self::Brake => "BRAKE",
            Self::Throw => "THROW",
            Self::AvoidAdsb => "AVOID_ADSB",
            Self::GuidedNoGps => "GUIDED_NOGPS",
            Self::SmartRtl => "SMART_RTL",
            Self::FlowHold => "FLOWHOLD",
            Self::Follow => "FOLLOW",
            Self::ZigZag => "ZIGZAG",
            Self::SystemId => "SYSTEMID",
            Self::Autorotate => "AUTOROTATE",
            Self::AutoRtl => "AUTO_RTL",
            Self::Turtle => "TURTLE",
            Self::RateAcro => "RATE_ACRO",
            Self::Unknown => "UNKNOWN",
        };
        f.write_str(name)
    }
}

/// Yaw portion of a body-motion demand.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
enum YawControl {
    /// Leave yaw under the current controller.
    #[default]
    Ignore,
    /// Control absolute yaw heading.
    Heading(Angle),
    /// Control yaw rate.
    Rate(AngularVelocity),
}

/// A body-frame motion demand for `ArduCopter` GUIDED mode.
///
/// The command is sent with `SET_POSITION_TARGET_LOCAL_NED` in
/// `MAV_FRAME_BODY_OFFSET_NED`. Like all GUIDED setpoints, it must be refreshed
/// continuously by the caller; `ArduCopter` stops after its setpoint timeout.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BodyMotionSetpoint {
    /// Optional body velocity.
    velocity: Option<BodyVelocity>,
    /// Optional body acceleration.
    acceleration: Option<BodyAcceleration>,
    /// Optional yaw control.
    yaw: YawControl,
}

impl BodyMotionSetpoint {
    /// Control body velocity only.
    #[must_use]
    pub const fn velocity(velocity: BodyVelocity) -> Self {
        Self {
            velocity: Some(velocity),
            acceleration: None,
            yaw: YawControl::Ignore,
        }
    }

    /// Control body acceleration only.
    #[must_use]
    pub const fn acceleration(acceleration: BodyAcceleration) -> Self {
        Self {
            velocity: None,
            acceleration: Some(acceleration),
            yaw: YawControl::Ignore,
        }
    }

    /// Control body velocity and acceleration together.
    #[must_use]
    pub const fn velocity_and_acceleration(
        velocity: BodyVelocity,
        acceleration: BodyAcceleration,
    ) -> Self {
        Self {
            velocity: Some(velocity),
            acceleration: Some(acceleration),
            yaw: YawControl::Ignore,
        }
    }

    /// Add an absolute body-relative yaw heading.
    #[must_use]
    pub const fn with_yaw_heading(mut self, yaw: Angle) -> Self {
        self.yaw = YawControl::Heading(yaw);
        self
    }

    /// Add a yaw-rate demand.
    #[must_use]
    pub const fn with_yaw_rate(mut self, yaw_rate: AngularVelocity) -> Self {
        self.yaw = YawControl::Rate(yaw_rate);
        self
    }
}

/// An `ArduCopter` vehicle.
#[derive(Debug, Clone)]
pub struct Copter {
    /// Underlying generic vehicle connection.
    vehicle: Vehicle,
}

impl std::ops::Deref for Copter {
    type Target = Vehicle;

    fn deref(&self) -> &Vehicle {
        &self.vehicle
    }
}

impl Copter {
    /// Connect to a copter. See [`Vehicle::connect`] for address formats.
    pub async fn connect(address: &str) -> Result<Self> {
        Ok(Self {
            vehicle: Vehicle::connect(address).await?,
        })
    }

    /// Connect using an explicit `MAVLink` source identity.
    pub async fn connect_with_identity(address: &str, identity: MavlinkIdentity) -> Result<Self> {
        Ok(Self {
            vehicle: Vehicle::connect_with_identity(address, identity).await?,
        })
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

    /// Current `ArduCopter` mode.
    #[must_use]
    pub fn mode(&self) -> CopterMode {
        CopterMode::from_custom_mode(self.state().custom_mode)
    }

    /// Change `ArduCopter` mode and wait for heartbeat confirmation.
    pub async fn set_mode(&self, mode: CopterMode) -> Result<()> {
        let Some(custom_mode) = mode.custom_mode() else {
            return Err(Error::UnknownCopterMode);
        };
        self.set_custom_mode(custom_mode).await.map(|_| ())
    }

    /// Arm and take off vertically in GUIDED mode.
    pub async fn takeoff(
        &self,
        altitude_above_home: Length,
        timeout: Duration,
    ) -> Result<Position> {
        if self.mode() != CopterMode::Guided {
            self.set_mode(CopterMode::Guided).await?;
        }
        self.arm().await?;
        self.command_long(
            MavCmd::MAV_CMD_NAV_TAKEOFF,
            [
                0.0,
                0.0,
                0.0,
                f32::NAN,
                0.0,
                0.0,
                altitude_above_home.get::<meter>() as f32,
            ],
        )
        .await?;
        self.wait_altitude(altitude_above_home * 0.95, timeout)
            .await
    }

    /// Fly to a global-relative target in GUIDED mode.
    pub async fn goto(&self, target: Target) -> Result<()> {
        if self.mode() != CopterMode::Guided {
            self.set_mode(CopterMode::Guided).await?;
        }
        self.command_int(
            MavCmd::MAV_CMD_DO_REPOSITION,
            MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            [-1.0, 0.0, 0.0, f32::NAN],
            target.latitude_e7(),
            target.longitude_e7(),
            target.altitude_above_home.get::<meter>() as f32,
        )
        .await
    }

    /// Send body velocity, acceleration, and optional yaw control.
    pub async fn send_body_motion(&self, setpoint: BodyMotionSetpoint) -> Result<()> {
        validate_setpoint(setpoint)?;
        if self.mode() != CopterMode::Guided {
            return Err(Error::WrongCopterMode {
                required: CopterMode::Guided,
                actual: self.mode(),
            });
        }

        self.send_body_motion_frame(setpoint, {
            // ArduPilot's Copter handler still requires the legacy body-offset
            // frame for body-resolved velocity and acceleration commands.
            #[expect(
                deprecated,
                reason = "`ArduCopter` does not yet accept MAV_FRAME_BODY_FRD for this message"
            )]
            let frame = MavFrame::MAV_FRAME_BODY_OFFSET_NED;
            frame
        })
        .await
    }

    /// Encode and send a body motion setpoint in a stack-specific frame.
    pub(crate) async fn send_body_motion_frame(
        &self,
        setpoint: BodyMotionSetpoint,
        coordinate_frame: MavFrame,
    ) -> Result<()> {
        validate_setpoint(setpoint)?;
        let mut mask = PositionTargetTypemask::POSITION_TARGET_TYPEMASK_X_IGNORE
            | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Y_IGNORE
            | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_Z_IGNORE;

        let velocity = setpoint.velocity.unwrap_or_else(|| {
            mask |= PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VX_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VY_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_VZ_IGNORE;
            BodyVelocity::default()
        });
        let acceleration = setpoint.acceleration.unwrap_or_else(|| {
            mask |= PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AX_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AY_IGNORE
                | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_AZ_IGNORE;
            BodyAcceleration::default()
        });
        let (yaw, yaw_rate) = match setpoint.yaw {
            YawControl::Ignore => {
                mask |= PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_IGNORE
                    | PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
                (0.0, 0.0)
            }
            YawControl::Heading(yaw) => {
                mask |= PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
                (yaw.get::<radian>() as f32, 0.0)
            }
            YawControl::Rate(rate) => {
                mask |= PositionTargetTypemask::POSITION_TARGET_TYPEMASK_YAW_IGNORE;
                (0.0, rate.get::<radian_per_second>() as f32)
            }
        };

        let [forward_velocity, right_velocity, down_velocity] = velocity.to_cartesian();
        let [forward_acceleration, right_acceleration, down_acceleration] =
            acceleration.to_cartesian();
        let (target_system, target_component) = self.target().unwrap_or((0, 0));
        self.send(&MavMessage::SET_POSITION_TARGET_LOCAL_NED(
            SET_POSITION_TARGET_LOCAL_NED_DATA {
                time_boot_ms: 0,
                x: 0.0,
                y: 0.0,
                z: 0.0,
                vx: forward_velocity.get::<meter_per_second>() as f32,
                vy: right_velocity.get::<meter_per_second>() as f32,
                vz: down_velocity.get::<meter_per_second>() as f32,
                afx: forward_acceleration.get::<meter_per_second_squared>() as f32,
                afy: right_acceleration.get::<meter_per_second_squared>() as f32,
                afz: down_acceleration.get::<meter_per_second_squared>() as f32,
                yaw,
                yaw_rate,
                type_mask: mask,
                target_system,
                target_component,
                coordinate_frame,
            },
        ))
        .await
    }

    /// Return to launch.
    pub async fn rtl(&self) -> Result<()> {
        self.set_mode(CopterMode::Rtl).await
    }

    /// Land at the current position.
    pub async fn land(&self) -> Result<()> {
        self.set_mode(CopterMode::Land).await
    }
}

/// Validate all caller-provided floating point fields before putting them on
/// the wire. `ArduPilot` treats NaN and infinity as a request to stop.
fn validate_setpoint(setpoint: BodyMotionSetpoint) -> Result<()> {
    if setpoint.velocity.is_none()
        && setpoint.acceleration.is_none()
        && matches!(setpoint.yaw, YawControl::Ignore)
    {
        return Err(Error::EmptyBodyMotionSetpoint);
    }
    if let Some(velocity) = setpoint.velocity {
        for (field, value) in ["forward velocity", "right velocity", "down velocity"]
            .into_iter()
            .zip(velocity.to_cartesian())
        {
            check_finite(field, value.get::<meter_per_second>())?;
        }
    }
    if let Some(acceleration) = setpoint.acceleration {
        for (field, value) in [
            "forward acceleration",
            "right acceleration",
            "down acceleration",
        ]
        .into_iter()
        .zip(acceleration.to_cartesian())
        {
            check_finite(field, value.get::<meter_per_second_squared>())?;
        }
    }
    match setpoint.yaw {
        YawControl::Ignore => {}
        YawControl::Heading(value) => check_finite("yaw heading", value.get::<radian>())?,
        YawControl::Rate(value) => {
            check_finite("yaw rate", value.get::<radian_per_second>())?;
        }
    }
    Ok(())
}

/// Reject non-finite setpoint values.
const fn check_finite(field: &'static str, value: f64) -> Result<()> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(Error::NonFiniteSetpoint { field, value })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use uom::ConstZero;

    #[test]
    fn copter_modes_roundtrip() {
        for value in 0..30 {
            let mode = CopterMode::from_custom_mode(value);
            if let Some(encoded) = mode.custom_mode() {
                assert_eq!(encoded, value);
            }
        }
    }

    #[test]
    fn body_motion_rejects_empty_and_non_finite_setpoints() {
        assert!(validate_setpoint(BodyMotionSetpoint::default()).is_err());
        assert!(
            validate_setpoint(BodyMotionSetpoint::velocity(sguaba::vector!(
                f = uom::si::f64::Velocity::new::<meter_per_second>(f64::NAN),
                r = uom::si::f64::Velocity::ZERO,
                d = uom::si::f64::Velocity::ZERO;
                in crate::VehicleBodyFrame
            )))
            .is_err()
        );
    }
}
