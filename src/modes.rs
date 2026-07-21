//! `ArduPlane` flight modes as a proper Rust enum.

/// `ArduPlane` flight mode.
///
/// Covers both fixed-wing and quadplane (`Q*`) modes. The discriminants match
/// `ArduPilot`'s `custom_mode` values as sent in `HEARTBEAT`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub enum Mode {
    /// Direct stick-to-surface passthrough.
    Manual,
    /// Fly in circles at the current altitude.
    Circle,
    /// Attitude stabilization only, pilot controls everything else.
    Stabilize,
    /// Training mode with attitude limits.
    Training,
    /// Aerobatic rate-controlled mode.
    Acro,
    /// Fly-by-wire A: stabilized attitude, manual throttle.
    FlyByWireA,
    /// Fly-by-wire B: stabilized attitude, altitude and airspeed hold.
    FlyByWireB,
    /// Like FBWB with automatic ground track hold.
    Cruise,
    /// Automatic tuning of attitude control gains.
    Autotune,
    /// Fly the loaded mission.
    Auto,
    /// Return to launch, then loiter or land depending on config.
    Rtl,
    /// Loiter around the point where the mode was entered.
    Loiter,
    /// Automatic takeoff, then loiter at `TKOFF_ALT`.
    Takeoff,
    /// Automatic ADS-B collision avoidance.
    AvoidAdsb,
    /// Accept navigation commands from a companion computer. That's us.
    Guided,
    /// Boot-time state, never commanded.
    #[default]
    Initializing,
    /// VTOL: attitude stabilization, manual throttle.
    QStabilize,
    /// VTOL: altitude hold.
    QHover,
    /// VTOL: position and altitude hold.
    QLoiter,
    /// VTOL: land at the current position.
    QLand,
    /// VTOL: return to launch and land vertically.
    QRtl,
    /// VTOL: automatic tuning of VTOL gains.
    QAutotune,
    /// VTOL: aerobatic rate mode.
    QAcro,
    /// Soar in thermals automatically.
    Thermal,
    /// Fixed-wing loiter down to altitude, then QLAND.
    LoiterAltQLand,
    /// Automatic fixed-wing landing.
    AutoLand,
    /// A mode this crate does not know about (newer firmware).
    Unknown(u32),
}

impl Mode {
    /// The `custom_mode` value `ArduPilot` uses for this mode.
    #[must_use]
    pub const fn custom_mode(self) -> u32 {
        match self {
            Self::Manual => 0,
            Self::Circle => 1,
            Self::Stabilize => 2,
            Self::Training => 3,
            Self::Acro => 4,
            Self::FlyByWireA => 5,
            Self::FlyByWireB => 6,
            Self::Cruise => 7,
            Self::Autotune => 8,
            Self::Auto => 10,
            Self::Rtl => 11,
            Self::Loiter => 12,
            Self::Takeoff => 13,
            Self::AvoidAdsb => 14,
            Self::Guided => 15,
            Self::Initializing => 16,
            Self::QStabilize => 17,
            Self::QHover => 18,
            Self::QLoiter => 19,
            Self::QLand => 20,
            Self::QRtl => 21,
            Self::QAutotune => 22,
            Self::QAcro => 23,
            Self::Thermal => 24,
            Self::LoiterAltQLand => 25,
            Self::AutoLand => 26,
            Self::Unknown(v) => v,
        }
    }

    /// Decode a `custom_mode` value from a plane `HEARTBEAT`.
    #[must_use]
    pub const fn from_custom_mode(value: u32) -> Self {
        match value {
            0 => Self::Manual,
            1 => Self::Circle,
            2 => Self::Stabilize,
            3 => Self::Training,
            4 => Self::Acro,
            5 => Self::FlyByWireA,
            6 => Self::FlyByWireB,
            7 => Self::Cruise,
            8 => Self::Autotune,
            10 => Self::Auto,
            11 => Self::Rtl,
            12 => Self::Loiter,
            13 => Self::Takeoff,
            14 => Self::AvoidAdsb,
            15 => Self::Guided,
            16 => Self::Initializing,
            17 => Self::QStabilize,
            18 => Self::QHover,
            19 => Self::QLoiter,
            20 => Self::QLand,
            21 => Self::QRtl,
            22 => Self::QAutotune,
            23 => Self::QAcro,
            24 => Self::Thermal,
            25 => Self::LoiterAltQLand,
            26 => Self::AutoLand,
            v => Self::Unknown(v),
        }
    }

    /// True for the VTOL (`Q*`) modes of a quadplane.
    #[must_use]
    pub const fn is_vtol(self) -> bool {
        matches!(
            self,
            Self::QStabilize
                | Self::QHover
                | Self::QLoiter
                | Self::QLand
                | Self::QRtl
                | Self::QAutotune
                | Self::QAcro
        )
    }
}

impl std::fmt::Display for Mode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = match self {
            Self::Manual => "MANUAL",
            Self::Circle => "CIRCLE",
            Self::Stabilize => "STABILIZE",
            Self::Training => "TRAINING",
            Self::Acro => "ACRO",
            Self::FlyByWireA => "FBWA",
            Self::FlyByWireB => "FBWB",
            Self::Cruise => "CRUISE",
            Self::Autotune => "AUTOTUNE",
            Self::Auto => "AUTO",
            Self::Rtl => "RTL",
            Self::Loiter => "LOITER",
            Self::Takeoff => "TAKEOFF",
            Self::AvoidAdsb => "AVOID_ADSB",
            Self::Guided => "GUIDED",
            Self::Initializing => "INITIALISING",
            Self::QStabilize => "QSTABILIZE",
            Self::QHover => "QHOVER",
            Self::QLoiter => "QLOITER",
            Self::QLand => "QLAND",
            Self::QRtl => "QRTL",
            Self::QAutotune => "QAUTOTUNE",
            Self::QAcro => "QACRO",
            Self::Thermal => "THERMAL",
            Self::LoiterAltQLand => "LOITER_ALT_QLAND",
            Self::AutoLand => "AUTOLAND",
            Self::Unknown(v) => return write!(f, "UNKNOWN({v})"),
        };
        f.write_str(name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn custom_mode_roundtrip() {
        for v in 0..30u32 {
            let mode = Mode::from_custom_mode(v);
            assert_eq!(mode.custom_mode(), v);
        }
    }
}
