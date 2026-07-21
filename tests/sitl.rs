//! End-to-end tests against real `ArduPilot` `SITL` instances.
//!
//! Requires an `ArduPilot` checkout with a built `sitl` target. Point
//! `MAVRS_ARDUPILOT_DIR` at it, or keep the default of
//! `~/work/ardupilot`.

use std::path::PathBuf;
use std::process::Stdio;
use std::time::Duration;

use std::fmt::Write as _;

use anyhow::{Context, Result, bail, ensure};
use mavrs::uom::ConstZero;
use mavrs::uom::si::acceleration::meter_per_second_squared;
use mavrs::uom::si::angle::{degree, radian};
use mavrs::uom::si::angular_velocity::radian_per_second;
use mavrs::uom::si::length::meter;
use mavrs::uom::si::ratio::ratio;
use mavrs::uom::si::velocity::meter_per_second;
use mavrs::{
    Acceleration, AcroControl, Angle, AngularVelocity, BodyMotionSetpoint, BodyVelocity, Copter,
    CopterMode, HeadingReference, Length, MavlinkIdentity, Mission, MissionItem, Mode, Plane,
    QuadPlane, Ratio, Target, Transition, Vehicle, VehicleBodyFrame, Velocity,
};
use tokio::process::{Child, Command};

/// `SITL` home location (CMAC, the `ArduPilot` test field).
const HOME: (f64, f64, f64, f64) = (-35.363_261, 149.165_230, 584.0, 353.0);
/// Simulation speedup. Keeps wall-clock time sane.
const SPEEDUP: u32 = 10;

/// Set up test logging (respects `RUST_LOG`).
fn init_tracing() {
    let _ = tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .with_test_writer()
        .try_init();
}

/// Where the `ArduPilot` checkout lives.
fn ardupilot_dir() -> PathBuf {
    std::env::var_os("MAVRS_ARDUPILOT_DIR").map_or_else(
        || {
            let home = std::env::var_os("HOME").unwrap_or_default();
            PathBuf::from(home).join("work/ardupilot")
        },
        PathBuf::from,
    )
}

/// A running `SITL` instance, killed on drop.
struct Sitl {
    /// The arduplane process.
    child: Child,
    /// Scratch dir holding eeprom and logs, cleaned up on drop.
    _workdir: tempfile::TempDir,
    /// `MAVLink` TCP address of SERIAL0.
    address: String,
}

impl Sitl {
    /// Launch `arduplane` with the given simulation model and defaults
    /// files (relative to the `ArduPilot` checkout), plus explicit extra
    /// parameters applied on top.
    ///
    /// Boots twice: enable params like `Q_ENABLE` only instantiate their
    /// parameter subtree on the boot *after* the wipe, so defaults for those
    /// subtrees (all the `Q_*` tuning) do not fully apply until the second
    /// start. `ArduPilot`'s own autotest reboots after wiping for the same
    /// reason.
    async fn spawn(
        instance: u32,
        binary: &str,
        model: &str,
        defaults: &[&str],
        extra_params: &[(&str, f32)],
    ) -> Result<Self> {
        let dir = ardupilot_dir();
        let bin = dir.join("build/sitl/bin").join(binary);
        ensure!(
            bin.exists(),
            "SITL binary {} not found; build ArduPilot sitl or set MAVRS_ARDUPILOT_DIR",
            bin.display()
        );
        let workdir = tempfile::TempDir::new()?;
        let mut defaults = defaults
            .iter()
            .map(|default| {
                let path = std::path::Path::new(default);
                if path.is_absolute() {
                    path.to_path_buf()
                } else {
                    dir.join(path)
                }
                .to_string_lossy()
                .into_owned()
            })
            .collect::<Vec<_>>();
        if !extra_params.is_empty() {
            let extra = workdir.path().join("extra.parm");
            let mut body = String::new();
            for (name, value) in extra_params {
                writeln!(body, "{name} {value}")?;
            }
            std::fs::write(&extra, body)?;
            defaults.push(extra.to_string_lossy().into_owned());
        }
        let defaults = defaults.join(",");
        let port = 5760 + 10 * instance;

        // Priming boot: wipe eeprom and load defaults, then shut down. No
        // connections are made to it; it just needs long enough to write the
        // parameter defaults to eeprom.
        let mut primer = launch(
            &bin,
            workdir.path(),
            model,
            &defaults,
            instance,
            SPEEDUP,
            true,
        )?;
        tokio::time::sleep(Duration::from_secs(8)).await;
        primer.kill().await?;
        wait_port_closed(port, Duration::from_secs(10)).await?;

        // Real boot with the fully instantiated parameter tree.
        //
        // Do not connect right away: SITL's flight controller init blocks on
        // the first SERIAL0 connection while the simulated physics settles
        // independently. Connecting during the settling transient makes the
        // initial attitude alignment garbage, which a tailsitter (gimbal
        // locked on its tail) never recovers from on the ground.
        let child = launch(
            &bin,
            workdir.path(),
            model,
            &defaults,
            instance,
            SPEEDUP,
            false,
        )?;
        tokio::time::sleep(Duration::from_secs(3)).await;
        Ok(Self {
            child,
            _workdir: workdir,
            address: format!("tcpout:127.0.0.1:{port}"),
        })
    }

    /// Connect a vehicle handle, retrying until `SITL` opens its TCP port.
    async fn connect(&mut self) -> Result<Vehicle> {
        self.connect_as(MavlinkIdentity::default()).await
    }

    /// Connect using an explicit `MAVLink` source identity.
    async fn connect_as(&mut self, identity: MavlinkIdentity) -> Result<Vehicle> {
        for _ in 0..50 {
            if let Some(status) = self.child.try_wait()? {
                bail!("SITL exited early: {status}");
            }
            match Vehicle::connect_with_identity(&self.address, identity).await {
                Ok(vehicle) => return Ok(vehicle),
                Err(_) => tokio::time::sleep(Duration::from_millis(200)).await,
            }
        }
        bail!("could not connect to SITL at {}", self.address)
    }
}

/// Start one arduplane process.
fn launch(
    bin: &std::path::Path,
    workdir: &std::path::Path,
    model: &str,
    defaults: &str,
    instance: u32,
    speedup: u32,
    wipe: bool,
) -> Result<Child> {
    let home = format!("{},{},{},{}", HOME.0, HOME.1, HOME.2, HOME.3);
    let log =
        std::fs::File::create(workdir.join(if wipe { "sitl-prime.log" } else { "sitl.log" }))?;
    let mut cmd = Command::new(bin);
    cmd.current_dir(workdir)
        .args([
            "--model",
            model,
            "--synthetic-clock",
            "--speedup",
            &speedup.to_string(),
            "--defaults",
            defaults,
            "--home",
            &home,
            "--instance",
            &instance.to_string(),
        ])
        .stdout(Stdio::from(log.try_clone()?))
        .stderr(Stdio::from(log))
        .kill_on_drop(true);
    if wipe {
        cmd.arg("--wipe");
    }
    cmd.spawn().context("failed to launch arduplane")
}

/// Wait until `SITL`'s SERIAL0 TCP port stops accepting connections.
async fn wait_port_closed(port: u32, timeout: Duration) -> Result<()> {
    let deadline = tokio::time::Instant::now() + timeout;
    while tokio::time::Instant::now() < deadline {
        if tokio::net::TcpStream::connect(("127.0.0.1", u16::try_from(port)?))
            .await
            .is_err()
        {
            return Ok(());
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
    bail!("SITL port {port} did not close within {timeout:?}")
}

/// Fixed-wing plane: auto takeoff, guided reposition, airspeed change,
/// altitude change, RTL.
#[tokio::test]
async fn plane_takeoff_guided_flight() -> Result<()> {
    init_tracing();
    let mut sitl = Sitl::spawn(
        10,
        "arduplane",
        "plane",
        &["Tools/autotest/models/plane.parm"],
        &[],
    )
    .await?;
    // ArduPilot's default MAV_GCS_SYSID is 255, and manual-control messages
    // are intentionally ignored from any other source id.
    let plane = Plane::from_vehicle(sitl.connect_as(MavlinkIdentity::new(255, 191)?).await?);
    plane.wait_ready(Duration::from_secs(60)).await?;
    plane.wait_armable(Duration::from_secs(120)).await?;

    // Parameter round trip while we are here.
    plane
        .set_param("TKOFF_ALT", 40.0)
        .await
        .context("set TKOFF_ALT")?;
    let alt = plane
        .get_param("TKOFF_ALT")
        .await
        .context("get TKOFF_ALT")?;
    ensure!((alt - 40.0).abs() < 0.01, "param readback got {alt}");
    // Auto takeoff and climb.
    let pos = plane
        .takeoff(Length::new::<meter>(30.0), Duration::from_secs(120))
        .await?;
    ensure!(
        pos.altitude_above_home >= Length::new::<meter>(30.0),
        "takeoff altitude {:?}",
        pos.altitude_above_home
    );

    // Guided reposition ~600 m north-east of home.
    let target = Target {
        latitude: Angle::new::<degree>(HOME.0 + 0.004),
        longitude: Angle::new::<degree>(HOME.1 + 0.004),
        altitude_above_home: Length::new::<meter>(80.0),
    };
    plane
        .goto_and_wait(
            target,
            Some(Length::new::<meter>(120.0)),
            Duration::from_secs(180),
        )
        .await?;
    ensure!(plane.mode() == Mode::Guided);

    plane
        .set_airspeed(Velocity::new::<meter_per_second>(16.0))
        .await?;
    plane
        .change_altitude(
            Length::new::<meter>(100.0),
            Velocity::new::<meter_per_second>(2.0),
        )
        .await?;
    plane
        .wait_position(Duration::from_secs(120), |position| {
            position.altitude_above_home > Length::new::<meter>(95.0)
        })
        .await?;

    // Search steering uses ArduPlane's GUIDED heading slew command.
    plane
        .change_heading(
            HeadingReference::CourseOverGround,
            Angle::new::<degree>(90.0),
            Acceleration::new::<meter_per_second_squared>(5.0),
        )
        .await?;
    assert_heading_held(&plane, Angle::new::<degree>(90.0)).await?;

    // Exercise ACRO control of the four primary RC axes and prove the
    // override produces a real roll response before handing control back.
    plane.set_mode(Mode::Acro).await?;
    let acro = AcroControl::new(
        Ratio::new::<ratio>(0.35),
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
        .context("no attitude after ACRO override")?;
    let (_, _, roll) = attitude.orientation.to_tait_bryan_angles();
    ensure!(
        roll.abs() > Angle::new::<radian>(0.15)
            || attitude.angular_velocity.roll.abs()
                > AngularVelocity::new::<radian_per_second>(0.15),
        "ACRO override produced no roll response: {attitude:?}"
    );
    plane.release_acro_control().await?;

    plane.rtl().await?;
    ensure!(plane.mode() == Mode::Rtl);
    Ok(())
}

/// Prove GUIDED heading control holds a course rather than merely crossing it
/// while loitering around the previous position target.
async fn assert_heading_held(plane: &Plane, heading: Angle) -> Result<()> {
    let settled = plane
        .wait_position(Duration::from_secs(120), |position| {
            position
                .heading
                .is_some_and(|actual| heading_error_deg(actual, heading) < 15.0)
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
    let mut max_error_deg = 0.0_f64;
    while tokio::time::Instant::now() < deadline {
        tokio::time::timeout(Duration::from_millis(500), positions.changed())
            .await
            .context("no position update while checking heading hold")?
            .context("position stream closed while checking heading hold")?;
        let position = *positions.borrow_and_update();
        if let Some(position) = position
            && let Some(actual_heading) = position.heading
        {
            max_error_deg = max_error_deg.max(heading_error_deg(actual_heading, heading));
            samples += 1;
        }
    }

    let end = plane.position().context("no position after heading hold")?;
    let displacement = end.distance_to(&start);
    ensure!(samples >= 5, "only received {samples} heading samples");
    ensure!(
        max_error_deg < 25.0,
        "heading was not held: maximum error {max_error_deg:.1} deg"
    );
    ensure!(
        displacement > Length::new::<meter>(100.0),
        "heading command did not produce sustained straight flight: displacement {displacement:?}"
    );
    Ok(())
}

/// Smallest absolute difference between compass headings.
fn heading_error_deg(a: Angle, b: Angle) -> f64 {
    ((a.get::<degree>() - b.get::<degree>() + 180.0).rem_euclid(360.0) - 180.0).abs()
}

/// Multicopter: guided takeoff, body velocity and acceleration control,
/// yaw-rate control, and landing.
#[tokio::test]
async fn copter_body_motion_control() -> Result<()> {
    init_tracing();
    let mut sitl = Sitl::spawn(
        13,
        "arducopter",
        "+",
        &["Tools/autotest/default_params/copter.parm"],
        &[],
    )
    .await?;
    let copter = Copter::from_vehicle(sitl.connect().await?);
    copter.wait_ready(Duration::from_secs(60)).await?;
    copter.wait_armable(Duration::from_secs(120)).await?;
    // Copter's SYS_STATUS prearm bit can lead EKF position readiness by a
    // fraction of a second; wait for the origin/position estimate to settle.
    copter
        .wait_position(Duration::from_secs(30), |_| true)
        .await?;
    tokio::time::sleep(Duration::from_secs(2)).await;
    copter
        .takeoff(Length::new::<meter>(15.0), Duration::from_secs(120))
        .await
        .context("copter takeoff")?;
    ensure!(copter.mode() == CopterMode::Guided);

    // Explicit mode round trip through copter mode numbering. Copter GUIDED
    // is custom_mode 4 and LOITER is 5, which the plane enum would decode as
    // ACRO and FBWA; this catches any regression to plane-mode handling in
    // the shared set_custom_mode/heartbeat path.
    copter.set_mode(CopterMode::Loiter).await?;
    ensure!(copter.mode() == CopterMode::Loiter);
    ensure!(copter.state().custom_mode == 5);
    copter.set_mode(CopterMode::Guided).await?;
    ensure!(copter.mode() == CopterMode::Guided);

    let motion = BodyMotionSetpoint::velocity_and_acceleration(
        mavrs::sguaba::vector!(
            f = Velocity::new::<meter_per_second>(4.0),
            r = Velocity::new::<meter_per_second>(0.0),
            d = Velocity::new::<meter_per_second>(0.0);
            in VehicleBodyFrame
        ),
        mavrs::sguaba::vector!(
            f = Acceleration::new::<meter_per_second_squared>(0.0),
            r = Acceleration::new::<meter_per_second_squared>(1.5),
            d = Acceleration::new::<meter_per_second_squared>(0.0);
            in VehicleBodyFrame
        ),
    );
    for _ in 0..20 {
        copter.send_body_motion(motion).await?;
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    let moving = copter.position().context("no copter position")?;
    let horizontal_speed = moving
        .velocity
        .ned_north()
        .hypot(moving.velocity.ned_east());
    ensure!(
        horizontal_speed > Velocity::new::<meter_per_second>(1.0),
        "body motion did not move copter: {moving:?}"
    );

    let (yaw_before, _, _) = copter
        .attitude()
        .context("no copter attitude")?
        .orientation
        .to_tait_bryan_angles();
    let yaw_motion = BodyMotionSetpoint::velocity(BodyVelocity::default())
        .with_yaw_rate(AngularVelocity::new::<radian_per_second>(0.8));
    for _ in 0..20 {
        copter.send_body_motion(yaw_motion).await?;
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    let (yaw_after, _, _) = copter
        .attitude()
        .context("no copter attitude")?
        .orientation
        .to_tait_bryan_angles();
    ensure!(
        (yaw_after - yaw_before).abs() > Angle::new::<radian>(0.2),
        "yaw-rate command produced no turn: before={yaw_before:?}, after={yaw_after:?}"
    );

    copter.land().await?;
    copter
        .wait_state(Duration::from_secs(180), |state| !state.armed)
        .await?;
    Ok(())
}

/// Copter tailsitter quadplane: guided VTOL takeoff, hover, fixed-wing
/// reposition (`ArduPilot` transitions automatically), then QRTL home.
#[tokio::test]
async fn tailsitter_vtol_takeoff_transition() -> Result<()> {
    init_tracing();
    let mut sitl = Sitl::spawn(
        11,
        "arduplane",
        "quadplane-copter_tailsitter",
        &[
            "Tools/autotest/default_params/quadplane.parm",
            "Tools/autotest/default_params/quadplane-copter_tailsitter.parm",
        ],
        // Not in the upstream defaults file; without it ArduPilot relies on a
        // boot-time heuristic (tailsitter.cpp Tailsitter::setup) that races
        // against defaults loading and silently flies the airframe as a
        // regular level-hover quadplane.
        &[("Q_TAILSIT_ENABLE", 1.0)],
    )
    .await?;
    let quad = QuadPlane::from_vehicle(sitl.connect().await?);
    quad.wait_ready(Duration::from_secs(60)).await?;
    // Sanity: the priming boot must have applied the tailsitter defaults.
    let motmx = quad.get_param("Q_TAILSIT_MOTMX").await?;
    ensure!(
        (motmx - 3.0).abs() < 0.1,
        "tailsitter defaults not applied, Q_TAILSIT_MOTMX={motmx}"
    );
    quad.wait_armable(Duration::from_secs(120)).await?;

    // Guided VTOL takeoff.
    let pos = quad
        .vtol_takeoff(Length::new::<meter>(20.0), Duration::from_secs(120))
        .await
        .context("vtol takeoff")?;
    ensure!(
        pos.altitude_above_home >= Length::new::<meter>(15.0),
        "VTOL takeoff altitude {:?}",
        pos.altitude_above_home
    );

    // Hover in place.
    quad.hover().await?;
    ensure!(quad.mode() == Mode::QLoiter);

    // Back to guided, fly somewhere. The reposition clears the VTOL state,
    // so ArduPilot transitions to forward flight on its own.
    let target = Target {
        latitude: Angle::new::<degree>(HOME.0 + 0.004),
        longitude: Angle::new::<degree>(HOME.1),
        altitude_above_home: Length::new::<meter>(50.0),
    };
    quad.goto(target).await.context("goto")?;
    quad.wait_position(Duration::from_secs(180), |position| {
        position.distance_to(&target) < Length::new::<meter>(150.0)
    })
    .await
    .context("reach target")?;

    // Come home and land vertically.
    quad.vtol_rtl().await?;
    ensure!(quad.mode() == Mode::QRtl);
    quad.wait_disarmed(Duration::from_secs(300)).await?;
    Ok(())
}

/// Tailsitter AUTO mission: upload VTOL takeoff -> waypoint -> VTOL land,
/// fly it end to end.
#[tokio::test]
async fn tailsitter_auto_mission() -> Result<()> {
    init_tracing();
    let mut sitl = Sitl::spawn(
        12,
        "arduplane",
        "quadplane-copter_tailsitter",
        &[
            "Tools/autotest/default_params/quadplane.parm",
            "Tools/autotest/default_params/quadplane-copter_tailsitter.parm",
        ],
        &[("Q_TAILSIT_ENABLE", 1.0)],
    )
    .await?;
    let quad = QuadPlane::from_vehicle(sitl.connect().await?);
    quad.wait_ready(Duration::from_secs(60)).await?;
    quad.wait_armable(Duration::from_secs(120)).await?;

    let wp = Target {
        latitude: Angle::new::<degree>(HOME.0 + 0.003),
        longitude: Angle::new::<degree>(HOME.1),
        altitude_above_home: Length::new::<meter>(40.0),
    };
    let mission = Mission::new()
        .then(MissionItem::vtol_takeoff(Length::new::<meter>(25.0)))
        .then(MissionItem::waypoint(wp))
        .then(MissionItem::vtol_land(None));
    quad.upload_mission(&mission).await?;

    quad.set_mode(Mode::Guided).await?;
    quad.arm().await?;
    quad.start_mission().await.context("start mission")?;
    quad.wait_altitude(Length::new::<meter>(20.0), Duration::from_secs(120))
        .await
        .context("mission takeoff")?;
    // Exercise DO_VTOL_TRANSITION, which ArduPilot only accepts in AUTO:
    // fly the current leg VTOL for a moment, then back to fixed wing.
    quad.transition(Transition::Hover)
        .await
        .context("to hover")?;
    quad.transition(Transition::ForwardFlight)
        .await
        .context("to forward flight")?;
    quad.wait_disarmed(Duration::from_secs(600))
        .await
        .context("mission landing")?;
    Ok(())
}
