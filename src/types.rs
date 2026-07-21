//! Typed telemetry and command values.
//!
//! Physical quantities use [`uom`]. Cartesian vectors additionally use
//! [`sguaba`] coordinate-system markers so body FRD and local NED values cannot
//! be mixed accidentally.

use sguaba::engineering::Orientation;
use uom::si::angle::radian;
use uom::si::f64::{
    Angle, AngularVelocity, ElectricCurrent, ElectricPotential, Length, Ratio, Velocity,
};
use uom::si::length::meter;

sguaba::system!(
    /// Vehicle-fixed forward-right-down frame.
    pub struct VehicleBodyFrame using FRD
);
sguaba::system!(
    /// Local north-east-down navigation frame.
    pub struct LocalNedFrame using NED
);

/// Velocity resolved in the vehicle forward-right-down body frame.
pub type BodyVelocity = sguaba::vector::VelocityVector<VehicleBodyFrame>;
/// Acceleration resolved in the vehicle forward-right-down body frame.
pub type BodyAcceleration = sguaba::vector::AccelerationVector<VehicleBodyFrame>;
/// Velocity resolved in the local north-east-down navigation frame.
pub type NedVelocity = sguaba::vector::VelocityVector<LocalNedFrame>;
/// Vehicle orientation relative to the local north-east-down frame.
pub type NedOrientation = Orientation<LocalNedFrame>;

/// A global navigation target.
///
/// `MAVLink`'s global-relative frames carry latitude and longitude plus altitude
/// above home, not WGS84 ellipsoid altitude. This therefore uses `uom`
/// quantities directly rather than misrepresenting the altitude with
/// [`sguaba::systems::Wgs84`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Target {
    /// Geodetic latitude.
    pub latitude: Angle,
    /// Geodetic longitude.
    pub longitude: Angle,
    /// Altitude above the vehicle's home datum.
    pub altitude_above_home: Length,
}

impl Target {
    /// Latitude as `MAVLink` `degE7`.
    #[must_use]
    pub fn latitude_e7(&self) -> i32 {
        angle_to_e7(self.latitude)
    }

    /// Longitude as `MAVLink` `degE7`.
    #[must_use]
    pub fn longitude_e7(&self) -> i32 {
        angle_to_e7(self.longitude)
    }
}

/// Angular velocity about the vehicle body axes.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct BodyRates {
    /// Roll rate, positive right wing down.
    pub roll: AngularVelocity,
    /// Pitch rate, positive nose up.
    pub pitch: AngularVelocity,
    /// Yaw rate, positive nose right.
    pub yaw: AngularVelocity,
}

/// Secondary IMU sample from `SCALED_IMU2`.
///
/// Acceleration is specific force and therefore includes gravity.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ImuSample {
    /// Time since the autopilot booted.
    pub time_since_boot: std::time::Duration,
    /// Specific force in the vehicle body frame.
    pub acceleration: BodyAcceleration,
    /// Angular velocity about the vehicle body axes.
    pub angular_velocity: BodyRates,
}

/// Fused global position from `GLOBAL_POSITION_INT`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Position {
    /// Geodetic latitude.
    pub latitude: Angle,
    /// Geodetic longitude.
    pub longitude: Angle,
    /// Altitude above mean sea level.
    pub altitude_msl: Length,
    /// Altitude above the vehicle's home datum.
    pub altitude_above_home: Length,
    /// Velocity in the local north-east-down frame.
    pub velocity: NedVelocity,
    /// Compass heading, or `None` when unknown.
    pub heading: Option<Angle>,
}

impl Position {
    /// Great-circle surface distance to a target.
    #[must_use]
    pub fn distance_to(&self, target: &Target) -> Length {
        let earth_radius = Length::new::<meter>(6_371_000.0);
        let lat1 = self.latitude.get::<radian>();
        let lat2 = target.latitude.get::<radian>();
        let dlat = (target.latitude - self.latitude).get::<radian>();
        let dlon = (target.longitude - self.longitude).get::<radian>();
        let a = lat1.cos().mul_add(
            lat2.cos() * (dlon / 2.0).sin().powi(2),
            (dlat / 2.0).sin().powi(2),
        );
        earth_radius * (2.0 * a.sqrt().asin())
    }
}

/// Vehicle attitude from `ATTITUDE`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Attitude {
    /// Vehicle body orientation relative to local NED.
    pub orientation: NedOrientation,
    /// Angular velocity about the vehicle body axes.
    pub angular_velocity: BodyRates,
}

impl Default for Attitude {
    fn default() -> Self {
        Self {
            orientation: NedOrientation::aligned(),
            angular_velocity: BodyRates::default(),
        }
    }
}

/// Pilot-facing flight data from `VFR_HUD`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FlightData {
    /// Indicated airspeed.
    pub airspeed: Velocity,
    /// GPS ground speed.
    pub groundspeed: Velocity,
    /// Altitude above mean sea level.
    pub altitude_msl: Length,
    /// Climb rate, positive up.
    pub climb_rate: Velocity,
    /// Compass heading.
    pub heading: Angle,
    /// Throttle output as a ratio from zero to one.
    pub throttle: Ratio,
}

/// Battery state from `SYS_STATUS`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Battery {
    /// Pack voltage.
    pub voltage: ElectricPotential,
    /// Current draw when measured.
    pub current: Option<ElectricCurrent>,
    /// Remaining capacity ratio when known.
    pub remaining: Option<Ratio>,
}

/// Overall link and readiness state derived from `HEARTBEAT` and `SYS_STATUS`.
///
/// The mode is exposed as the raw autopilot-specific `custom_mode` number;
/// decode it with the platform wrapper's `mode()` method ([`crate::Plane`],
/// [`crate::Copter`]), since the same number means different modes on
/// different vehicle types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct VehicleState {
    /// Raw autopilot-specific mode number from `HEARTBEAT`.
    pub custom_mode: u32,
    /// Motors armed.
    pub armed: bool,
    /// All prearm checks currently passing.
    pub prearm_ok: bool,
}

/// Convert an angle to `MAVLink`'s signed degrees-times-1e7 representation.
fn angle_to_e7(angle: Angle) -> i32 {
    use uom::si::angle::degree;

    let scaled = (angle.get::<degree>() * 1e7).round();
    scaled as i32
}

#[cfg(test)]
mod tests {
    use super::*;
    use uom::ConstZero;
    use uom::si::angle::degree;

    #[test]
    fn target_scaling() {
        let target = Target {
            latitude: Angle::new::<degree>(-35.363_261),
            longitude: Angle::new::<degree>(149.165_230),
            altitude_above_home: Length::new::<meter>(30.0),
        };
        assert_eq!(target.latitude_e7(), -353_632_610);
        assert_eq!(target.longitude_e7(), 1_491_652_300);
    }

    #[test]
    fn haversine_sanity() {
        let position = Position {
            latitude: Angle::new::<degree>(-35.363_261),
            longitude: Angle::new::<degree>(149.165_230),
            ..Position::default()
        };
        let target = Target {
            latitude: Angle::new::<degree>(-35.362_261),
            longitude: Angle::new::<degree>(149.165_230),
            altitude_above_home: Length::ZERO,
        };
        let distance = position.distance_to(&target).get::<meter>();
        assert!((distance - 111.2).abs() < 1.0);
    }
}
