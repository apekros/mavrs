# mavrs

## Why

MAVLink on the wire is scaled integers and shared numbers: latitude is
`i32` degrees times 1e7, velocity is `i16` cm/s, `custom_mode` 4 is GUIDED
on a copter and ACRO on a plane, and a body-frame velocity looks exactly
like a NED one. mavrs does the conversions at the wire boundary so the
compiler catches this class of mistake.

mavrs supports both ArduPilot and PX4. Their mode encodings, integer parameter
encoding, mission indexing, and accepted setpoint frames differ, and those
differences stay behind stack-specific typed APIs.

## Features

- [`uom`](https://docs.rs/uom) quantities everywhere, no raw floats in the
  API
- [`sguaba`](https://docs.rs/sguaba) frame markers, vehicle FRD and local
  NED don't mix
- mode enums per platform (`Mode::Guided`, `CopterMode::Guided`) instead of
  `custom_mode` numbers
- complete PX4 heartbeat mode decoding, managed multicopter Offboard,
  fixed-wing Guided Course/ACRO, and confirmed VTOL transition APIs
- commands wait for `COMMAND_ACK`, retry on timeout, follow `IN_PROGRESS`,
  rejection is an `Err`
- telemetry (position, attitude, body IMU, battery, mode/armed) as
  `tokio::sync::watch` channels, `wait_*` helpers on top
- `Vehicle::send` / `Vehicle::messages` for anything not wrapped

## Quick start

```rust
use std::time::Duration;
use mavrs::uom::si::{angle::degree, length::meter, velocity::meter_per_second};
use mavrs::{Angle, Length, Plane, Target, Velocity};

let plane = Plane::connect("tcpout:127.0.0.1:5760").await?;
plane.wait_ready(Duration::from_secs(30)).await?;
plane.wait_armable(Duration::from_secs(60)).await?;

plane
    .takeoff(Length::new::<meter>(30.0), Duration::from_secs(120))
    .await?;
plane
    .goto(Target {
        latitude: Angle::new::<degree>(-35.360),
        longitude: Angle::new::<degree>(149.170),
        altitude_above_home: Length::new::<meter>(100.0),
    })
    .await?;
plane
    .set_airspeed(Velocity::new::<meter_per_second>(16.0))
    .await?;
plane.rtl().await?;
```

Quadplanes and tailsitters get the full `Plane` API plus VTOL operations:

```rust
use mavrs::uom::si::length::meter;
use mavrs::{Length, QuadPlane};

let quad = QuadPlane::connect("tcpout:127.0.0.1:5760").await?;
quad.wait_ready(Duration::from_secs(30)).await?;
quad.wait_armable(Duration::from_secs(60)).await?;

quad
    .vtol_takeoff(Length::new::<meter>(20.0), Duration::from_secs(60))
    .await?;
quad.goto(target).await?;
quad.hover().await?;
quad.vtol_land(Duration::from_secs(120)).await?;
```

Copters take typed body-frame motion setpoints in GUIDED mode:

```rust
use mavrs::uom::si::{
    acceleration::meter_per_second_squared,
    velocity::meter_per_second,
};
use mavrs::{Acceleration, BodyMotionSetpoint, Copter, VehicleBodyFrame, Velocity};

let copter = Copter::connect("tcpout:127.0.0.1:5760").await?;
copter.wait_ready(Duration::from_secs(30)).await?;
let velocity = mavrs::sguaba::vector!(
    f = Velocity::new::<meter_per_second>(20.0),
    r = Velocity::new::<meter_per_second>(0.0),
    d = Velocity::new::<meter_per_second>(0.0);
    in VehicleBodyFrame
);
let acceleration = mavrs::sguaba::vector!(
    f = Acceleration::new::<meter_per_second_squared>(0.0),
    r = Acceleration::new::<meter_per_second_squared>(4.0),
    d = Acceleration::new::<meter_per_second_squared>(0.0);
    in VehicleBodyFrame
);
let setpoint = BodyMotionSetpoint::velocity_and_acceleration(velocity, acceleration);
copter.send_body_motion(setpoint).await?;
```

Missions are typed too:

```rust
use mavrs::uom::si::length::meter;
use mavrs::{Length, Mission, MissionItem};

let mission = Mission::new()
    .then(MissionItem::vtol_takeoff(Length::new::<meter>(25.0)))
    .then(MissionItem::waypoint(wp))
    .then(MissionItem::vtol_land(None));
quad.upload_mission(&mission).await?;
quad.start_mission().await?;
```

PX4 uses a common mode system across airframes. Multicopter body motion runs
in Offboard mode. A managed session handles the required setpoint priming,
10 Hz watchdog feed, setpoint updates, and a Hold transition on shutdown:

```rust
use mavrs::{BodyMotionSetpoint, Px4Copter};

let copter = Px4Copter::connect("udpin:0.0.0.0:14540").await?;
copter.wait_ready(Duration::from_secs(30)).await?;
let session = copter.start_offboard(setpoint).await?;

session.set_setpoint(next_setpoint).await?;
session.stop().await?;
```

## Testing against SITL

The integration tests fly real autopilots and physics simulators end to end.
`tests/sitl.rs` covers ArduPilot fixed-wing guided flight and ACRO control,
copter body-frame motion, tailsitter transitions, and complete VTOL missions.
`tests/px4_sitl.rs` provides matching PX4 coverage using Gazebo Harmonic:
fixed-wing takeoff/reposition, altitude and sustained course control, real
ACRO response, managed multicopter Offboard body motion, confirmed VTOL
transitions through `EXTENDED_SYS_STATE`, RTL landing, and a complete VTOL
mission.

The repository's Nix flake supplies Rust, the PX4 build toolchain, its Python
dependencies, and Gazebo Harmonic from
[`gazebros2nix`](https://github.com/Gepetto/gazebros2nix):

```sh
nix develop
```

You need an ArduPilot checkout with a built `sitl` target:

```sh
./waf configure --board sitl
./waf plane
./waf copter
```

Point `MAVRS_ARDUPILOT_DIR` at the checkout (it defaults to
`~/work/ardupilot`), then:

```sh
cargo test --test sitl
```

For PX4, clone its source with submodules and build SITL from the Nix shell:

```sh
git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/work/PX4-Autopilot
cd ~/work/PX4-Autopilot
make px4_sitl
```

Point `MAVRS_PX4_DIR` at that checkout (it defaults to
`~/work/PX4-Autopilot`), then run the four isolated headless Gazebo tests:

```sh
cargo test --test px4_sitl
```

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms or
conditions.

## Disclaimer

A significant portion of this was aided with GPT 5.6 Sol!
