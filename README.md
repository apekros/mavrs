# mavrs

## Why

MAVLink on the wire is scaled integers and shared numbers: latitude is
`i32` degrees times 1e7, velocity is `i16` cm/s, `custom_mode` 4 is GUIDED
on a copter and ACRO on a plane, and a body-frame velocity looks exactly
like a NED one. mavrs does the conversions at the wire boundary so the
compiler catches this class of mistake.

## Features

- [`uom`](https://docs.rs/uom) quantities everywhere, no raw floats in the
  API
- [`sguaba`](https://docs.rs/sguaba) frame markers, vehicle FRD and local
  NED don't mix
- mode enums per platform (`Mode::Guided`, `CopterMode::Guided`) instead of
  `custom_mode` numbers
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

## Testing against SITL

The integration tests in `tests/sitl.rs` fly real ArduPilot SITL instances
end to end: fixed-wing guided flight and ACRO control, copter body-frame
motion, tailsitter transitions, and complete VTOL missions.

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

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the Apache-2.0
license, shall be dual licensed as above, without any additional terms or
conditions.

## Disclaimer

A significant portion of this was aided with GPT 5.6 Sol!
