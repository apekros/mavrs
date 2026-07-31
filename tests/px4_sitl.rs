//! End-to-end tests against PX4 SITL with Gazebo Harmonic.
//!
//! Enter the repository's Nix development shell, then point
//! `MAVRS_PX4_DIR` at a PX4-Autopilot checkout built with
//! `make px4_sitl` (the default is `~/work/PX4-Autopilot`).

use std::path::PathBuf;
use std::process::Stdio;
use std::time::Duration;

#[cfg(unix)]
use std::os::unix::process::CommandExt as _;

use anyhow::{Context, Result, bail, ensure};
use mavrs::uom::ConstZero;
use mavrs::uom::si::acceleration::meter_per_second_squared;
use mavrs::uom::si::angle::degree;
use mavrs::uom::si::angular_velocity::radian_per_second;
use mavrs::uom::si::length::meter;
use mavrs::uom::si::ratio::ratio;
use mavrs::uom::si::velocity::meter_per_second;
use mavrs::{
    Acceleration, AcroControl, Angle, AngularVelocity, BodyMotionSetpoint, BodyVelocity, Length,
    Mission, MissionItem, Px4Copter, Px4Mode, Px4Plane, Px4Vtol, Ratio, Target, Transition,
    Vehicle, VehicleBodyFrame, Velocity,
};
use tokio::process::{Child, Command};

/// Gazebo lockstep speed multiplier.
const SPEEDUP: u32 = 4;

/// Set up test logging (respects `RUST_LOG`).
fn init_tracing() {
    let _ = tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .with_test_writer()
        .try_init();
}

/// Locate the PX4-Autopilot checkout.
fn px4_dir() -> PathBuf {
    std::env::var_os("MAVRS_PX4_DIR").map_or_else(
        || {
            let home = std::env::var_os("HOME").unwrap_or_default();
            PathBuf::from(home).join("work/PX4-Autopilot")
        },
        PathBuf::from,
    )
}

/// A PX4 and isolated Gazebo instance, terminated on drop.
struct Px4Sitl {
    /// PX4 process.
    child: Child,
    /// Process group shared by PX4 and the Gazebo server it launches.
    process_group: u32,
    /// Per-test PX4 parameter and log storage.
    workdir: tempfile::TempDir,
    /// UDP endpoint to which PX4's onboard `MAVLink` link sends.
    address: String,
}

impl Px4Sitl {
    /// Launch an isolated PX4/Gazebo model.
    fn spawn(instance: u8, model: &str) -> Result<Self> {
        let dir = px4_dir();
        let build = dir.join("build/px4_sitl_default");
        let binary = build.join("bin/px4");
        let etc = build.join("etc");
        let models = dir.join("Tools/simulation/gz/models");
        let worlds = dir.join("Tools/simulation/gz/worlds");
        let plugins = build.join("src/modules/simulation/gz_plugins");
        let server_config = dir.join("src/modules/simulation/gz_bridge/server.config");
        ensure!(
            binary.exists() && etc.exists(),
            "PX4 SITL build not found at {}; enter `nix develop` and run `make px4_sitl`, or set MAVRS_PX4_DIR",
            build.display()
        );

        let workdir = tempfile::TempDir::new()?;
        let log = std::fs::File::create(workdir.path().join("px4-sitl.log"))?;
        let mut command = Command::new(&binary);
        #[cfg(unix)]
        command.as_std_mut().process_group(0);
        command
            .current_dir(workdir.path())
            .args(["-i", &instance.to_string(), "-d"])
            .arg(&etc)
            .env("PX4_SIM_MODEL", model)
            .env("PX4_SIM_SPEED_FACTOR", SPEEDUP.to_string())
            .env("HEADLESS", "1")
            .env("PX4_GZ_MODELS", &models)
            .env("PX4_GZ_WORLDS", &worlds)
            .env("PX4_GZ_PLUGINS", &plugins)
            .env("PX4_GZ_SERVER_CONFIG", &server_config)
            .env(
                "GZ_SIM_RESOURCE_PATH",
                join_search_paths("GZ_SIM_RESOURCE_PATH", [&models, &worlds])?,
            )
            .env(
                "GZ_SIM_SYSTEM_PLUGIN_PATH",
                join_search_paths("GZ_SIM_SYSTEM_PLUGIN_PATH", [&plugins])?,
            )
            .env("GZ_SIM_SERVER_CONFIG_PATH", &server_config)
            // Gazebo Transport partitions isolate concurrently running worlds.
            .env("GZ_PARTITION", format!("mavrs-px4-{instance}"))
            .stdout(Stdio::from(log.try_clone()?))
            .stderr(Stdio::from(log))
            .kill_on_drop(true);
        let child = command.spawn().context("failed to launch PX4 SITL")?;
        let process_group = child.id().context("PX4 process has no id")?;
        let remote_port = 14540_u16 + u16::from(instance);
        Ok(Self {
            child,
            process_group,
            workdir,
            address: format!("udpin:0.0.0.0:{remote_port}"),
        })
    }

    /// Bind the endpoint PX4 streams to, and fail if SITL exits first.
    async fn connect(&mut self) -> Result<Vehicle> {
        if let Some(status) = self.child.try_wait()? {
            bail!("PX4 SITL exited early: {status}");
        }
        let vehicle = Vehicle::connect(&self.address)
            .await
            .with_context(|| format!("bind PX4 MAVLink endpoint {}", self.address))?;
        if let Err(error) = vehicle.wait_ready(Duration::from_secs(90)).await {
            let log = std::fs::read_to_string(self.workdir.path().join("px4-sitl.log"))
                .unwrap_or_else(|read_error| format!("could not read SITL log: {read_error}"));
            bail!("PX4 did not become ready: {error}\n--- PX4 SITL log ---\n{log}");
        }
        Ok(vehicle)
    }
}

/// Prepend paths while preserving a search path inherited from the Nix shell.
fn join_search_paths<const N: usize>(
    variable: &str,
    paths: [&std::path::Path; N],
) -> Result<std::ffi::OsString> {
    let mut paths = paths
        .into_iter()
        .map(std::path::Path::to_path_buf)
        .collect::<Vec<_>>();
    if let Some(existing) = std::env::var_os(variable) {
        paths.extend(std::env::split_paths(&existing));
    }
    std::env::join_paths(paths).context("construct Gazebo search path")
}

impl Drop for Px4Sitl {
    fn drop(&mut self) {
        // PX4 starts Gazebo through its startup shell. Kill the dedicated
        // process group so neither the shell nor gz-server survives a test.
        for signal in ["-TERM", "-KILL"] {
            let _ = std::process::Command::new("kill")
                .args([signal, "--", &format!("-{}", self.process_group)])
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .status();
        }
        let _ = std::process::Command::new("pkill")
            .args(["-KILL", "-g", &self.process_group.to_string()])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        let _ = self.child.start_kill();
    }
}

/// Build a target offset from the vehicle's actual Gazebo home position.
fn offset_target(
    position: mavrs::Position,
    north_degrees: f64,
    east_degrees: f64,
    altitude: f64,
) -> Target {
    Target {
        latitude: position.latitude + Angle::new::<degree>(north_degrees),
        longitude: position.longitude + Angle::new::<degree>(east_degrees),
        altitude_above_home: Length::new::<meter>(altitude),
    }
}

/// PX4 fixed wing: parameter round trip, takeoff, reposition, altitude and
/// course control, ACRO response, and return.
#[tokio::test]
async fn px4_plane_takeoff_guided_flight() -> Result<()> {
    init_tracing();
    let mut sitl = Px4Sitl::spawn(0, "gz_rc_cessna")?;
    let plane = Px4Plane::from_vehicle(sitl.connect().await?);
    plane.wait_ready(Duration::from_secs(90)).await?;
    plane.wait_armable(Duration::from_secs(120)).await?;
    let home = plane
        .wait_position(Duration::from_secs(30), |_| true)
        .await?;

    plane.set_param("MIS_TAKEOFF_ALT", 45.0).await?;
    let takeoff_altitude = plane.get_param("MIS_TAKEOFF_ALT").await?;
    ensure!(
        (takeoff_altitude - 45.0).abs() < 0.01,
        "parameter readback got {takeoff_altitude}"
    );

    plane
        .takeoff(Length::new::<meter>(40.0), Duration::from_secs(180))
        .await?;
    let target = offset_target(home, 0.004, 0.004, 80.0);
    plane
        .goto_and_wait(
            target,
            Length::new::<meter>(150.0),
            Duration::from_secs(240),
        )
        .await?;
    plane
        .set_airspeed(Velocity::new::<meter_per_second>(18.0))
        .await?;

    plane.change_altitude(Length::new::<meter>(100.0)).await?;
    plane
        .wait_position(Duration::from_secs(120), |position| {
            position.altitude_above_home > Length::new::<meter>(95.0)
        })
        .await?;

    let east = Angle::new::<degree>(90.0);
    plane.change_course(east).await?;
    assert_course_held(&plane, east).await?;

    plane.enter_acro_control(Ratio::new::<ratio>(0.65)).await?;
    let acro = AcroControl::new(
        Ratio::new::<ratio>(0.5),
        Ratio::ZERO,
        Ratio::ZERO,
        Ratio::new::<ratio>(0.7),
    )?;
    for _ in 0..20 {
        plane.send_acro_control(acro).await?;
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    let attitude = plane
        .attitude()
        .context("no PX4 attitude after ACRO input")?;
    let (_, _, roll) = attitude.orientation.to_tait_bryan_angles();
    ensure!(
        roll.get::<degree>().abs() > 8.0
            || attitude.angular_velocity.roll.abs()
                > AngularVelocity::new::<radian_per_second>(0.15),
        "PX4 ACRO input produced no roll response: {attitude:?}"
    );
    plane.release_acro_control().await?;
    plane.rtl().await?;
    ensure!(plane.mode() == Px4Mode::Return);
    Ok(())
}

/// Prove PX4 Guided Course holds the requested track rather than merely
/// passing through it while circling.
async fn assert_course_held(plane: &Px4Plane, course: Angle) -> Result<()> {
    let settled = plane
        .wait_position(Duration::from_secs(120), |position| {
            position
                .heading
                .is_some_and(|heading| heading_error_deg(heading, course) < 15.0)
        })
        .await?;
    let start = Target {
        latitude: settled.latitude,
        longitude: settled.longitude,
        altitude_above_home: settled.altitude_above_home,
    };
    let mut positions = plane.position_watch();
    let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
    let mut samples = 0_u32;
    let mut max_error = 0.0_f64;
    while tokio::time::Instant::now() < deadline {
        tokio::time::timeout(Duration::from_millis(500), positions.changed())
            .await
            .context("no PX4 position update while checking course")?
            .context("PX4 position stream closed while checking course")?;
        if let Some(position) = *positions.borrow_and_update()
            && let Some(heading) = position.heading
        {
            max_error = max_error.max(heading_error_deg(heading, course));
            samples += 1;
        }
    }
    let end = plane
        .position()
        .context("no PX4 position after course hold")?;
    ensure!(samples >= 5, "only received {samples} PX4 course samples");
    ensure!(
        max_error < 25.0,
        "PX4 course error reached {max_error:.1} deg"
    );
    ensure!(
        end.distance_to(&start) > Length::new::<meter>(50.0),
        "PX4 course command did not produce sustained flight"
    );
    Ok(())
}

/// Smallest absolute difference between compass headings.
fn heading_error_deg(a: Angle, b: Angle) -> f64 {
    ((a.get::<degree>() - b.get::<degree>() + 180.0).rem_euclid(360.0) - 180.0).abs()
}

/// PX4 multicopter: autonomous takeoff, mode handling, body velocity and
/// acceleration, yaw-rate control, and landing.
#[tokio::test]
async fn px4_copter_offboard_body_motion() -> Result<()> {
    init_tracing();
    let mut sitl = Px4Sitl::spawn(1, "gz_x500")?;
    let copter = Px4Copter::from_vehicle(sitl.connect().await?);
    copter.wait_ready(Duration::from_secs(90)).await?;
    copter.wait_armable(Duration::from_secs(120)).await?;
    copter
        .wait_position(Duration::from_secs(30), |_| true)
        .await?;
    copter
        .takeoff(Length::new::<meter>(15.0), Duration::from_secs(120))
        .await?;

    copter.set_mode(Px4Mode::Hold).await?;
    ensure!(copter.mode() == Px4Mode::Hold);

    let motion = BodyMotionSetpoint::velocity_and_acceleration(
        mavrs::sguaba::vector!(
            f = Velocity::new::<meter_per_second>(4.0),
            r = Velocity::ZERO,
            d = Velocity::ZERO;
            in VehicleBodyFrame
        ),
        mavrs::sguaba::vector!(
            f = Acceleration::ZERO,
            r = Acceleration::new::<meter_per_second_squared>(1.0),
            d = Acceleration::ZERO;
            in VehicleBodyFrame
        ),
    );
    let session = copter.start_offboard(motion).await?;
    tokio::time::sleep(Duration::from_secs(1)).await;
    let moving = copter.position().context("no PX4 copter position")?;
    let horizontal_speed = moving
        .velocity
        .ned_north()
        .hypot(moving.velocity.ned_east());
    ensure!(
        horizontal_speed > Velocity::new::<meter_per_second>(1.0),
        "offboard body motion produced no movement: {moving:?}"
    );

    let (yaw_before, _, _) = copter
        .attitude()
        .context("no PX4 copter attitude")?
        .orientation
        .to_tait_bryan_angles();
    let yaw_motion = BodyMotionSetpoint::velocity(BodyVelocity::default())
        .with_yaw_rate(AngularVelocity::new::<radian_per_second>(0.8));
    session.set_setpoint(yaw_motion).await?;
    tokio::time::sleep(Duration::from_secs(1)).await;
    let (yaw_after, _, _) = copter
        .attitude()
        .context("no PX4 copter attitude after yaw command")?
        .orientation
        .to_tait_bryan_angles();
    ensure!(
        (yaw_after - yaw_before).get::<degree>().abs() > 10.0,
        "offboard yaw-rate command produced no turn"
    );

    drop(session);
    copter
        .wait_state(Duration::from_secs(10), |state| {
            Px4Mode::from_custom_mode(state.custom_mode) == Px4Mode::Hold
        })
        .await?;
    copter.land().await?;
    copter.wait_disarmed(Duration::from_secs(180)).await?;
    Ok(())
}

/// PX4 standard VTOL: multicopter takeoff, transition to fixed-wing flight,
/// reposition, transition back to hover, and RTL.
#[tokio::test]
async fn px4_vtol_takeoff_transition() -> Result<()> {
    init_tracing();
    let mut sitl = Px4Sitl::spawn(2, "gz_standard_vtol")?;
    let vtol = Px4Vtol::from_vehicle(sitl.connect().await?);
    vtol.wait_ready(Duration::from_secs(90)).await?;
    vtol.wait_armable(Duration::from_secs(120)).await?;
    let home = vtol
        .wait_position(Duration::from_secs(30), |_| true)
        .await?;
    vtol.takeoff(Length::new::<meter>(20.0), Duration::from_secs(120))
        .await?;

    vtol.transition(Transition::ForwardFlight).await?;
    let target = offset_target(home, 0.004, 0.0, 60.0);
    vtol.goto_and_wait(
        target,
        Length::new::<meter>(150.0),
        Duration::from_secs(240),
    )
    .await?;
    vtol.transition(Transition::Hover).await?;
    vtol.rtl().await?;
    ensure!(vtol.mode() == Px4Mode::Return);
    vtol.wait_disarmed(Duration::from_secs(300)).await?;
    Ok(())
}

/// PX4 VTOL mission: upload VTOL takeoff, waypoint, and VTOL landing, then
/// execute the mission end to end.
#[tokio::test]
async fn px4_vtol_auto_mission() -> Result<()> {
    init_tracing();
    let mut sitl = Px4Sitl::spawn(3, "gz_standard_vtol")?;
    let vtol = Px4Vtol::from_vehicle(sitl.connect().await?);
    vtol.wait_ready(Duration::from_secs(90)).await?;
    vtol.wait_armable(Duration::from_secs(120)).await?;
    let home = vtol
        .wait_position(Duration::from_secs(30), |_| true)
        .await?;
    let waypoint = offset_target(home, 0.003, 0.0, 50.0);
    let takeoff = Target {
        latitude: home.latitude,
        longitude: home.longitude,
        altitude_above_home: Length::new::<meter>(30.0),
    };
    let landing = Target {
        latitude: home.latitude,
        longitude: home.longitude,
        altitude_above_home: Length::ZERO,
    };
    let mission = Mission::new()
        .then(MissionItem::vtol_takeoff_at(takeoff))
        .then(MissionItem::waypoint(waypoint))
        .then(MissionItem::vtol_land(Some(landing)));
    vtol.upload_mission(&mission).await?;

    vtol.arm().await?;
    vtol.start_mission().await?;
    vtol.wait_altitude(Length::new::<meter>(20.0), Duration::from_secs(120))
        .await?;
    vtol.wait_disarmed(Duration::from_secs(600)).await?;
    Ok(())
}
