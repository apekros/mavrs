//! Fixed-wing plane control.

use std::time::Duration;

use mavlink::dialects::ardupilotmega::{
    MavCmd, MavFrame, MavMessage, RC_CHANNELS_OVERRIDE_DATA, SpeedType,
};

use crate::Mode;
use crate::error::{Error, Result};
use crate::types::{BodyRates, Position, Target};
use crate::vehicle::{MavlinkIdentity, Vehicle};
use uom::ConstZero;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angle::degree;
use uom::si::angular_velocity::{degree_per_second, radian_per_second};
use uom::si::f64::{Acceleration, Angle, AngularVelocity, Length, Ratio, Velocity};
use uom::si::length::meter;
use uom::si::ratio::ratio;
use uom::si::velocity::meter_per_second;

/// `MAVLink` value meaning an RC override field should be ignored.
const RC_OVERRIDE_IGNORE: u16 = u16::MAX;

/// Which heading `ArduPlane` should control in GUIDED mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeadingReference {
    /// Course over ground, accounting for wind drift.
    CourseOverGround,
    /// The vehicle's nose heading.
    VehicleHeading,
}

impl HeadingReference {
    /// `ArduPilot` `HEADING_TYPE` encoded as a command parameter.
    const fn command_value(self) -> f32 {
        match self {
            Self::CourseOverGround => 0.0,
            Self::VehicleHeading => 1.0,
        }
    }
}

/// Normalized pilot inputs for `ArduPlane` ACRO mode.
///
/// Roll, pitch, and yaw are in `-1.0..=1.0`; throttle is in `0.0..=1.0`.
/// `ArduPilot` maps the axis inputs through its ACRO rate and expo parameters,
/// so these are deliberately called inputs rather than rate setpoints.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AcroControl {
    /// Roll stick ratio, positive right.
    pub roll: Ratio,
    /// Pitch stick ratio, positive nose up.
    pub pitch: Ratio,
    /// Yaw stick ratio, positive right.
    pub yaw: Ratio,
    /// Throttle ratio from idle to full.
    pub throttle: Ratio,
}

impl AcroControl {
    /// Construct a checked ACRO control demand.
    pub fn new(roll: Ratio, pitch: Ratio, yaw: Ratio, throttle: Ratio) -> Result<Self> {
        check_control("roll", roll, -1.0, 1.0)?;
        check_control("pitch", pitch, -1.0, 1.0)?;
        check_control("yaw", yaw, -1.0, 1.0)?;
        check_control("throttle", throttle, 0.0, 1.0)?;
        Ok(Self {
            roll,
            pitch,
            yaw,
            throttle,
        })
    }
}

/// ACRO stick-to-rate scaling reported by `ArduPlane`, in SI units.
///
/// Load this once with [`Plane::acro_rate_limits`], then reuse it in the
/// control loop. A zero axis limit means that axis is raw pilot input rather
/// than a rate-controlled axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AcroRateLimits {
    /// Maximum commanded roll rate.
    pub roll: AngularVelocity,
    /// Maximum commanded pitch rate.
    pub pitch: AngularVelocity,
    /// Maximum commanded yaw rate, or zero when yaw rate control is disabled.
    pub yaw: AngularVelocity,
}

impl AcroRateLimits {
    /// Convert physical body rates and throttle to normalized ACRO inputs.
    pub fn control(self, rates: BodyRates, throttle: Ratio) -> Result<AcroControl> {
        AcroControl::new(
            normalize_rate("roll rate", rates.roll, self.roll)?,
            normalize_rate("pitch rate", rates.pitch, self.pitch)?,
            normalize_rate("yaw rate", rates.yaw, self.yaw)?,
            throttle,
        )
    }
}

/// A fixed-wing `ArduPlane` vehicle.
///
/// Wraps a [`Vehicle`] with plane-specific operations. Derefs to [`Vehicle`]
/// so all the generic operations (params, mode, arm, telemetry) are available
/// directly.
#[derive(Debug, Clone)]
pub struct Plane {
    /// The underlying vehicle handle.
    vehicle: Vehicle,
}

impl std::ops::Deref for Plane {
    type Target = Vehicle;

    fn deref(&self) -> &Vehicle {
        &self.vehicle
    }
}

impl Plane {
    /// Connect to a plane. See [`Vehicle::connect`] for address formats.
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

    /// Wrap an already-connected vehicle.
    #[must_use]
    pub const fn from_vehicle(vehicle: Vehicle) -> Self {
        Self { vehicle }
    }

    /// The underlying vehicle handle.
    #[must_use]
    pub const fn vehicle(&self) -> &Vehicle {
        &self.vehicle
    }

    /// Current `ArduPlane` flight mode.
    #[must_use]
    pub fn mode(&self) -> Mode {
        Mode::from_custom_mode(self.state().custom_mode)
    }

    /// Switch `ArduPlane` flight mode and wait for heartbeat confirmation.
    pub async fn set_mode(&self, mode: Mode) -> Result<()> {
        match self.set_custom_mode(mode.custom_mode()).await {
            Ok(_) => Ok(()),
            Err(Error::Timeout { .. }) => Err(Error::ModeChangeFailed {
                requested: mode,
                actual: self.mode(),
            }),
            Err(err) => Err(err),
        }
    }

    /// Automatic fixed-wing takeoff: switches to TAKEOFF mode and arms.
    ///
    /// The plane climbs to `TKOFF_ALT` (a parameter, default 50 m) and
    /// loiters there. Returns once `minimum_altitude` is reached.
    pub async fn takeoff(&self, minimum_altitude: Length, timeout: Duration) -> Result<Position> {
        self.set_mode(Mode::Takeoff).await?;
        self.arm().await?;
        self.wait_altitude(minimum_altitude, timeout).await
    }

    /// Fly to a location in GUIDED mode. Returns as soon as the command is
    /// accepted; the plane will loiter around the target when it arrives.
    ///
    /// Switches to GUIDED if not already there.
    pub async fn goto(&self, target: Target) -> Result<()> {
        if self.mode() != Mode::Guided {
            self.set_mode(Mode::Guided).await?;
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

    /// Fly to a location in GUIDED mode and wait until the plane is within
    /// `acceptance_radius` of it. Pass `None` for a sensible default.
    pub async fn goto_and_wait(
        &self,
        target: Target,
        acceptance_radius: Option<Length>,
        timeout: Duration,
    ) -> Result<Position> {
        self.goto(target).await?;
        let radius = acceptance_radius.unwrap_or_else(|| Length::new::<meter>(100.0));
        self.wait_position(timeout, |position| position.distance_to(&target) <= radius)
            .await
    }

    /// Change target airspeed in GUIDED mode.
    pub async fn set_airspeed(&self, airspeed: Velocity) -> Result<()> {
        self.command_int(
            MavCmd::MAV_CMD_GUIDED_CHANGE_SPEED,
            MavFrame::MAV_FRAME_GLOBAL,
            [
                f32::from(SpeedType::SPEED_TYPE_AIRSPEED as u8),
                airspeed.get::<meter_per_second>() as f32,
                0.0,
                0.0,
            ],
            0,
            0,
            0.0,
        )
        .await
    }

    /// Change target altitude in GUIDED mode without changing the target
    /// location. A zero `climb_rate` means "as fast as possible".
    pub async fn change_altitude(
        &self,
        altitude_above_home: Length,
        climb_rate: Velocity,
    ) -> Result<()> {
        self.command_int(
            MavCmd::MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            [0.0, 0.0, climb_rate.get::<meter_per_second>() as f32, 0.0],
            0,
            0,
            altitude_above_home.get::<meter>() as f32,
        )
        .await
    }

    /// Change heading in GUIDED mode.
    ///
    /// `max_lateral_accel_mss` limits how aggressively the aircraft turns and
    /// corresponds to `ArduPilot`'s `MAV_CMD_GUIDED_CHANGE_HEADING` parameter 3.
    pub async fn change_heading(
        &self,
        reference: HeadingReference,
        heading: Angle,
        maximum_lateral_acceleration: Acceleration,
    ) -> Result<()> {
        if self.mode() != Mode::Guided {
            self.set_mode(Mode::Guided).await?;
        }
        let heading_deg = heading.get::<degree>().rem_euclid(360.0);
        let max_lateral_accel_mss = maximum_lateral_acceleration.get::<meter_per_second_squared>();
        if !max_lateral_accel_mss.is_finite() || max_lateral_accel_mss < 0.0 {
            return Err(Error::ControlOutOfRange {
                field: "maximum lateral acceleration",
                value: max_lateral_accel_mss,
                min: 0.0,
                max: f64::MAX,
            });
        }
        self.command_int(
            MavCmd::MAV_CMD_GUIDED_CHANGE_HEADING,
            MavFrame::MAV_FRAME_GLOBAL,
            [
                reference.command_value(),
                heading_deg as f32,
                max_lateral_accel_mss as f32,
                0.0,
            ],
            0,
            0,
            0.0,
        )
        .await
    }

    /// Read the ACRO axis rate limits from the aircraft.
    ///
    /// This is deliberately separate from [`Self::send_acro_rates`] so a
    /// high-rate control loop does not perform parameter reads.
    pub async fn acro_rate_limits(&self) -> Result<AcroRateLimits> {
        let roll_deg_s = self.get_param("ACRO_ROLL_RATE").await?;
        let pitch_deg_s = self.get_param("ACRO_PITCH_RATE").await?;
        let yaw_deg_s = self.get_param("ACRO_YAW_RATE").await?;
        for (field, value) in [
            ("ACRO_ROLL_RATE", roll_deg_s),
            ("ACRO_PITCH_RATE", pitch_deg_s),
            ("ACRO_YAW_RATE", yaw_deg_s),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(Error::ControlOutOfRange {
                    field,
                    value: f64::from(value),
                    min: 0.0,
                    max: f64::MAX,
                });
            }
        }
        Ok(AcroRateLimits {
            roll: AngularVelocity::new::<degree_per_second>(f64::from(roll_deg_s)),
            pitch: AngularVelocity::new::<degree_per_second>(f64::from(pitch_deg_s)),
            yaw: AngularVelocity::new::<degree_per_second>(f64::from(yaw_deg_s)),
        })
    }

    /// Assign RC override authority to a `MAVLink` system id.
    pub async fn set_rc_override_authority(&self, system_id: u8) -> Result<()> {
        if system_id == 0 {
            return Err(Error::InvalidSystemId(system_id));
        }
        self.set_param("MAV_GCS_SYSID", f32::from(system_id)).await
    }

    /// Confirm this connection's source system owns RC override authority.
    pub async fn ensure_rc_override_authority(&self) -> Result<()> {
        let configured_param = self.get_param("MAV_GCS_SYSID").await?;
        let configured = u8::try_from(configured_param.round() as i32).map_err(|_| {
            Error::ControlOutOfRange {
                field: "MAV_GCS_SYSID",
                value: f64::from(configured_param),
                min: 1.0,
                max: 255.0,
            }
        })?;
        let actual = self.identity().system_id();
        if configured != actual {
            return Err(Error::RcOverrideAuthority { configured, actual });
        }
        Ok(())
    }

    /// Prime throttle and neutral axes, then enter ACRO without a throttle gap.
    ///
    /// RC authority is checked before changing mode.
    pub async fn enter_acro_control(&self, throttle: Ratio) -> Result<()> {
        self.ensure_rc_override_authority().await?;
        let neutral = AcroControl::new(Ratio::ZERO, Ratio::ZERO, Ratio::ZERO, throttle)?;
        self.send_acro_control(neutral).await?;
        self.set_mode(Mode::Acro).await?;
        self.send_acro_control(neutral).await
    }

    /// Send physical ACRO body rates through `MAVLink` RC overrides.
    ///
    /// Load `limits` once with [`Self::acro_rate_limits`]. The caller must
    /// refresh the command at its control-loop rate.
    pub async fn send_acro_rates(
        &self,
        limits: AcroRateLimits,
        rates: BodyRates,
        throttle: Ratio,
    ) -> Result<()> {
        self.send_acro_control(limits.control(rates, throttle)?)
            .await
    }

    /// Send normalized ACRO pilot inputs through `MAVLink` RC overrides.
    ///
    /// The caller must refresh the command at its control-loop rate. Use
    /// [`Self::release_acro_control`] when handing control back to a receiver.
    pub async fn send_acro_control(&self, control: AcroControl) -> Result<()> {
        let (target_system, target_component) = self.target().unwrap_or((0, 0));
        self.send(&MavMessage::RC_CHANNELS_OVERRIDE(
            RC_CHANNELS_OVERRIDE_DATA {
                chan1_raw: signed_input_to_pwm(control.roll),
                chan2_raw: signed_input_to_pwm(control.pitch),
                chan3_raw: throttle_to_pwm(control.throttle),
                chan4_raw: signed_input_to_pwm(control.yaw),
                chan5_raw: RC_OVERRIDE_IGNORE,
                chan6_raw: RC_OVERRIDE_IGNORE,
                chan7_raw: RC_OVERRIDE_IGNORE,
                chan8_raw: RC_OVERRIDE_IGNORE,
                target_system,
                target_component,
            },
        ))
        .await
    }

    /// Release roll, pitch, throttle, and yaw RC overrides.
    pub async fn release_acro_control(&self) -> Result<()> {
        let (target_system, target_component) = self.target().unwrap_or((0, 0));
        self.send(&MavMessage::RC_CHANNELS_OVERRIDE(
            RC_CHANNELS_OVERRIDE_DATA {
                chan1_raw: 0,
                chan2_raw: 0,
                chan3_raw: 0,
                chan4_raw: 0,
                chan5_raw: RC_OVERRIDE_IGNORE,
                chan6_raw: RC_OVERRIDE_IGNORE,
                chan7_raw: RC_OVERRIDE_IGNORE,
                chan8_raw: RC_OVERRIDE_IGNORE,
                target_system,
                target_component,
            },
        ))
        .await
    }

    /// Release ACRO inputs and command GUIDED recovery to `target`.
    ///
    /// Overrides are released before the mode change, preventing stale
    /// throttle or rate inputs from leaking into recovery.
    pub async fn recover_from_acro(&self, target: Target) -> Result<()> {
        self.release_acro_control().await?;
        self.goto(target).await
    }

    /// Return to launch.
    pub async fn rtl(&self) -> Result<()> {
        self.set_mode(Mode::Rtl).await
    }

    /// Loiter at the current position.
    pub async fn loiter(&self) -> Result<()> {
        self.set_mode(Mode::Loiter).await
    }

    /// Start the loaded mission from the beginning (switches to AUTO).
    pub async fn start_mission(&self) -> Result<()> {
        if self.mode() != Mode::Auto {
            self.set_mode(Mode::Auto).await?;
        }
        self.command_long(MavCmd::MAV_CMD_MISSION_START, [0.0; 7])
            .await
    }

    /// Wait until the plane reports being disarmed (e.g. after an
    /// auto-land).
    pub async fn wait_disarmed(&self, timeout: Duration) -> Result<()> {
        self.wait_state(timeout, |s| !s.armed).await.map(|_| ())
    }
}

/// Validate a finite normalized control value.
fn check_control(field: &'static str, value: Ratio, min: f64, max: f64) -> Result<()> {
    let value = value.get::<ratio>();
    if !value.is_finite() || value < min || value > max {
        return Err(Error::ControlOutOfRange {
            field,
            value,
            min,
            max,
        });
    }
    Ok(())
}

/// Convert one physical ACRO rate to its normalized stick ratio.
fn normalize_rate(
    field: &'static str,
    requested: AngularVelocity,
    limit: AngularVelocity,
) -> Result<Ratio> {
    let requested_rad_s = requested.get::<radian_per_second>();
    let limit_rad_s = limit.get::<radian_per_second>();
    if !requested_rad_s.is_finite()
        || requested_rad_s.abs() > limit_rad_s
        || (limit_rad_s == 0.0 && requested_rad_s != 0.0)
    {
        return Err(Error::ControlOutOfRange {
            field,
            value: requested_rad_s,
            min: -limit_rad_s,
            max: limit_rad_s,
        });
    }
    if limit_rad_s == 0.0 {
        Ok(Ratio::ZERO)
    } else {
        Ok(Ratio::new::<ratio>(requested_rad_s / limit_rad_s))
    }
}

/// Convert a signed normalized stick ratio to conventional RC PWM.
fn signed_input_to_pwm(input: Ratio) -> u16 {
    float_pwm_to_u16(input.get::<ratio>().mul_add(500.0, 1500.0))
}

/// Convert normalized throttle to conventional RC PWM.
fn throttle_to_pwm(throttle: Ratio) -> u16 {
    float_pwm_to_u16(throttle.get::<ratio>().mul_add(1000.0, 1000.0))
}

/// Convert a PWM value to the wire type, clamped to the conventional RC
/// range. Never produces 0, which `RC_CHANNELS_OVERRIDE` treats as "release
/// this channel"; a math error must not silently drop an override mid-flight.
fn float_pwm_to_u16(value: f64) -> u16 {
    let clamped = value.round().clamp(1000.0, 2000.0) as i32;
    u16::try_from(clamped).unwrap_or(1500)
}

#[cfg(test)]
mod tests {
    use anyhow::Result;

    use super::*;

    #[test]
    fn acro_control_validates_ranges() -> Result<()> {
        let control = AcroControl::new(
            Ratio::new::<ratio>(-1.0),
            Ratio::new::<ratio>(1.0),
            Ratio::ZERO,
            Ratio::new::<ratio>(0.73),
        )?;
        assert_eq!(signed_input_to_pwm(control.roll), 1000);
        assert_eq!(signed_input_to_pwm(control.pitch), 2000);
        assert_eq!(signed_input_to_pwm(control.yaw), 1500);
        assert_eq!(throttle_to_pwm(control.throttle), 1730);
        assert!(
            AcroControl::new(
                Ratio::new::<ratio>(1.01),
                Ratio::ZERO,
                Ratio::ZERO,
                Ratio::new::<ratio>(0.5),
            )
            .is_err()
        );
        assert!(
            AcroControl::new(
                Ratio::ZERO,
                Ratio::ZERO,
                Ratio::ZERO,
                Ratio::new::<ratio>(f64::NAN),
            )
            .is_err()
        );
        Ok(())
    }

    #[test]
    fn acro_rate_limits_convert_si_rates_and_reject_disabled_yaw() -> Result<()> {
        let limits = AcroRateLimits {
            roll: AngularVelocity::new::<degree_per_second>(40.0),
            pitch: AngularVelocity::new::<degree_per_second>(40.0),
            yaw: AngularVelocity::ZERO,
        };
        let control = limits.control(
            BodyRates {
                roll: AngularVelocity::new::<degree_per_second>(20.0),
                pitch: AngularVelocity::new::<degree_per_second>(-10.0),
                yaw: AngularVelocity::ZERO,
            },
            Ratio::new::<ratio>(0.73),
        )?;
        assert!((control.roll.get::<ratio>() - 0.5).abs() < f64::EPSILON);
        assert!((control.pitch.get::<ratio>() + 0.25).abs() < f64::EPSILON);
        assert!(
            limits
                .control(
                    BodyRates {
                        yaw: AngularVelocity::new::<degree_per_second>(1.0),
                        ..BodyRates::default()
                    },
                    Ratio::new::<ratio>(0.73),
                )
                .is_err()
        );
        Ok(())
    }
}
