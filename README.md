# mavrs

Type safe wrappers for PX4 and ArduPilot behaviour targeted towards being used for offboard control.

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

## Testing with simulated aircraft

These tests run the actual ArduPilot and PX4 software with simulated aircraft.
They cover plane and copter flight, switching between hover and forward flight,
and full missions. ArduPilot tests live in `tests/sitl.rs`. PX4 tests live in
`tests/px4_sitl.rs` and use Gazebo Harmonic for the simulation.

Start by entering the Nix shell. It includes Rust, the tools needed to build
PX4, and Gazebo Harmonic:

```sh
nix develop
```

### ArduPilot

In your ArduPilot source directory, build the plane and copter simulators:

```sh
./waf configure --board sitl
./waf plane
./waf copter
```

If your ArduPilot source is somewhere other than `~/work/ardupilot`, set
`MAVRS_ARDUPILOT_DIR` to that path. Then run the tests from the mavrs directory:

```sh
cargo test --test sitl
```

### PX4

From the Nix shell, download PX4 and build its simulator:

```sh
git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/work/PX4-Autopilot
cd ~/work/PX4-Autopilot
make px4_sitl
```

If you downloaded PX4 somewhere other than `~/work/PX4-Autopilot`, set
`MAVRS_PX4_DIR` to that path. Return to the mavrs directory and run the tests.
Each test starts its own simulation, with no GUI:

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
