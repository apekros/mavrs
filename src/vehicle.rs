//! The core [`Vehicle`] handle: connection, telemetry, commands, parameters.

use std::future::Future;
use std::sync::Arc;
use std::time::Duration;

use mavlink::dialects::ardupilotmega::{
    ATTITUDE_DATA, COMMAND_INT_DATA, COMMAND_LONG_DATA, GLOBAL_POSITION_INT_DATA, HEARTBEAT_DATA,
    MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavParamType, MavResult, MavState,
    MavSysStatusSensor, MavType, PARAM_REQUEST_READ_DATA, PARAM_SET_DATA, SCALED_IMU2_DATA,
    SYS_STATUS_DATA, VFR_HUD_DATA,
};
use mavlink::{AsyncMavConnection, MavHeader, MessageData};
use sguaba::engineering::Orientation;
use tokio::sync::{broadcast, watch};
use tracing::{debug, trace, warn};
use uom::si::acceleration::standard_gravity;
use uom::si::angle::{degree, radian};
use uom::si::angular_velocity::radian_per_second;
use uom::si::electric_current::ampere;
use uom::si::electric_potential::volt;
use uom::si::f64::{
    Acceleration, Angle, AngularVelocity, ElectricCurrent, ElectricPotential, Frequency, Length,
    Ratio, Velocity,
};
use uom::si::frequency::hertz;
use uom::si::length::meter;
use uom::si::ratio::percent;
use uom::si::velocity::meter_per_second;

use crate::error::{Error, Result};
use crate::types::{
    Attitude, Battery, BodyRates, FlightData, ImuSample, LocalNedFrame, Position, VehicleBodyFrame,
    VehicleState,
};

/// Default system id used when talking to the vehicle.
const DEFAULT_SYSTEM_ID: u8 = 255;
/// Default component id used for the companion computer.
const DEFAULT_COMPONENT_ID: u8 = 191;
/// How long to wait for a `COMMAND_ACK` before retrying.
const ACK_TIMEOUT: Duration = Duration::from_millis(1500);
/// How many times to send a command before giving up.
const COMMAND_RETRIES: u32 = 3;
/// Capacity of the raw message broadcast channel.
const EVENT_CAPACITY: usize = 2048;

/// A raw message event as seen on the link.
type Event = Arc<(MavHeader, MavMessage)>;

/// Role advertised in this connection's heartbeat.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MavlinkRole {
    /// A GCS, eligible for GCS failsafe tracking and RC override authority.
    GroundControlStation,
    /// A companion or onboard controller that does not satisfy GCS heartbeat
    /// monitoring.
    OnboardController,
}

impl MavlinkRole {
    /// `MAVLink` vehicle type used in the heartbeat.
    const fn mav_type(self) -> MavType {
        match self {
            Self::GroundControlStation => MavType::MAV_TYPE_GCS,
            Self::OnboardController => MavType::MAV_TYPE_ONBOARD_CONTROLLER,
        }
    }
}

/// `MAVLink` identity used for outgoing messages.
///
/// `ArduPilot` can restrict manual and RC override input to `MAV_GCS_SYSID`, so
/// controlling applications must be able to choose their source system id and
/// advertise a GCS heartbeat while owning control.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MavlinkIdentity {
    /// Source system id, in the `MAVLink` range 1..=255.
    system_id: u8,
    /// Source component id.
    component_id: u8,
    /// Role advertised by periodic heartbeats.
    role: MavlinkRole,
}

impl MavlinkIdentity {
    /// Construct a GCS identity. System id zero is reserved by `MAVLink`.
    ///
    /// A controlling companion must advertise a GCS heartbeat for `ArduPilot`'s
    /// GCS failsafe timer, not merely use the configured GCS system id.
    pub const fn new(system_id: u8, component_id: u8) -> Result<Self> {
        Self::with_role(system_id, component_id, MavlinkRole::GroundControlStation)
    }

    /// Construct an onboard-controller identity that does not own GCS control.
    pub const fn onboard_controller(system_id: u8, component_id: u8) -> Result<Self> {
        Self::with_role(system_id, component_id, MavlinkRole::OnboardController)
    }

    /// Construct an identity with an explicit heartbeat role.
    pub const fn with_role(system_id: u8, component_id: u8, role: MavlinkRole) -> Result<Self> {
        if system_id == 0 {
            return Err(Error::InvalidSystemId(system_id));
        }
        Ok(Self {
            system_id,
            component_id,
            role,
        })
    }

    /// Source system id.
    #[must_use]
    pub const fn system_id(self) -> u8 {
        self.system_id
    }

    /// Source component id.
    #[must_use]
    pub const fn component_id(self) -> u8 {
        self.component_id
    }

    /// Heartbeat role.
    #[must_use]
    pub const fn role(self) -> MavlinkRole {
        self.role
    }
}

impl Default for MavlinkIdentity {
    fn default() -> Self {
        Self {
            system_id: DEFAULT_SYSTEM_ID,
            component_id: DEFAULT_COMPONENT_ID,
            role: MavlinkRole::GroundControlStation,
        }
    }
}

/// Shared connection state between the [`Vehicle`] handle and its IO tasks.
struct Inner {
    /// The underlying mavlink connection.
    conn: Box<dyn AsyncMavConnection<MavMessage> + Send + Sync>,
    /// Header identity stamped on outgoing messages.
    identity: MavlinkIdentity,
    /// Broadcast of every received message, for waiters.
    events: broadcast::Sender<Event>,
    /// Discovered autopilot (system id, component id).
    target: watch::Sender<Option<(u8, u8)>>,
    /// Mode / armed / prearm state.
    state: watch::Sender<VehicleState>,
    /// Fused global position.
    position: watch::Sender<Option<Position>>,
    /// Attitude.
    attitude: watch::Sender<Option<Attitude>>,
    /// VFR HUD data.
    flight_data: watch::Sender<Option<FlightData>>,
    /// Battery state.
    battery: watch::Sender<Option<Battery>>,
    /// Secondary body IMU sample.
    imu: watch::Sender<Option<ImuSample>>,
    /// Becomes `true` when the connection closes, by either the receive loop
    /// dying or the last user handle dropping. Level-triggered: late
    /// subscribers still observe it.
    shutdown: watch::Sender<bool>,
}

/// Owns the shared state on behalf of user-facing [`Vehicle`] handles.
///
/// The IO tasks hold their own references to [`Inner`], so this extra layer
/// gives the user handles a distinct refcount: when the last [`Vehicle`]
/// clone drops, this guard drops and shuts the connection down.
struct ConnectionGuard {
    /// The shared connection state.
    inner: Arc<Inner>,
}

impl std::ops::Deref for ConnectionGuard {
    type Target = Inner;

    fn deref(&self) -> &Inner {
        &self.inner
    }
}

impl Drop for ConnectionGuard {
    fn drop(&mut self) {
        self.inner.shutdown.send_replace(true);
    }
}

/// Handle to a `MAVLink` vehicle.
///
/// Cheap to clone; all clones share one connection. Dropping the last clone
/// shuts down the background IO tasks.
#[derive(Clone)]
pub struct Vehicle {
    /// Shared state, behind the guard that signals shutdown on last drop.
    inner: Arc<ConnectionGuard>,
}

impl std::fmt::Debug for Vehicle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Vehicle")
            .field("target", &*self.inner.target.borrow())
            .field("state", &*self.inner.state.borrow())
            .finish_non_exhaustive()
    }
}

impl Vehicle {
    /// Connect to a vehicle.
    ///
    /// Address formats follow rust-mavlink: `tcpout:host:port`,
    /// `tcpin:host:port`, `udpin:host:port`, `udpout:host:port`,
    /// `serial:/dev/ttyUSB0:baud`, `file:path`.
    ///
    /// This establishes the transport and spawns background tasks that stream
    /// telemetry and send heartbeats. It does not wait for the autopilot to
    /// show up; call [`Vehicle::wait_ready`] for that.
    pub async fn connect(address: &str) -> Result<Self> {
        Self::connect_with_identity(address, MavlinkIdentity::default()).await
    }

    /// Connect using an explicit `MAVLink` source identity.
    ///
    /// Set the system id to the vehicle's `MAV_GCS_SYSID` value when
    /// `ArduPilot` is configured to accept manual control from only one GCS.
    pub async fn connect_with_identity(address: &str, identity: MavlinkIdentity) -> Result<Self> {
        let conn = mavlink::connect_async::<MavMessage>(address).await?;
        let (events, _) = broadcast::channel(EVENT_CAPACITY);
        let inner = Arc::new(Inner {
            conn,
            identity,
            events,
            target: watch::Sender::new(None),
            state: watch::Sender::new(VehicleState::default()),
            position: watch::Sender::new(None),
            attitude: watch::Sender::new(None),
            flight_data: watch::Sender::new(None),
            battery: watch::Sender::new(None),
            imu: watch::Sender::new(None),
            shutdown: watch::Sender::new(false),
        });

        tokio::spawn(recv_loop(Arc::clone(&inner)));
        tokio::spawn(heartbeat_loop(Arc::downgrade(&inner)));

        Ok(Self {
            inner: Arc::new(ConnectionGuard { inner }),
        })
    }

    /// Wait until the autopilot is heard from and basic telemetry streams are
    /// flowing. Requests message intervals for the telemetry this crate
    /// exposes.
    pub async fn wait_ready(&self, timeout: Duration) -> Result<()> {
        let deadline = tokio::time::Instant::now() + timeout;
        let mut target_rx = self.inner.target.subscribe();
        let wait = async {
            loop {
                if target_rx.borrow_and_update().is_some() {
                    return Ok::<(), Error>(());
                }
                if target_rx.changed().await.is_err() {
                    return Err(Error::ConnectionClosed);
                }
            }
        };
        tokio::time::timeout_at(deadline, self.until_closed(wait))
            .await
            .map_err(|_| Error::Timeout {
                what: "autopilot heartbeat",
                after: timeout,
            })??;

        self.request_default_streams().await?;
        Ok(())
    }

    /// Ask the autopilot to stream the messages backing our telemetry watches.
    async fn request_default_streams(&self) -> Result<()> {
        /// Message ids and rates (hz) we want streamed.
        const STREAMS: &[(u32, f64)] = &[
            (GLOBAL_POSITION_INT_DATA::ID, 4.0),
            (ATTITUDE_DATA::ID, 4.0),
            (VFR_HUD_DATA::ID, 2.0),
            (SYS_STATUS_DATA::ID, 1.0),
        ];
        for &(msg_id, hz) in STREAMS {
            self.request_message_rate(msg_id, Frequency::new::<hertz>(hz))
                .await?;
        }
        Ok(())
    }

    /// Request one `MAVLink` message at `rate_hz`.
    pub async fn request_message_rate(&self, message_id: u32, rate: Frequency) -> Result<()> {
        let rate_hz = rate.get::<hertz>();
        if !rate_hz.is_finite() || rate_hz <= 0.0 {
            return Err(Error::ControlOutOfRange {
                field: "message rate",
                value: rate_hz,
                min: f64::MIN_POSITIVE,
                max: f64::MAX,
            });
        }
        let interval_us = (1e6 / rate_hz).round();
        self.command_long(
            MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            [
                u32_to_f32_param(message_id),
                interval_us as f32,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        )
        .await
    }

    /// Request the position, attitude, and secondary IMU feeds used by a
    /// companion-computer control loop.
    pub async fn request_control_telemetry(&self, rate: Frequency) -> Result<()> {
        for message_id in [
            GLOBAL_POSITION_INT_DATA::ID,
            ATTITUDE_DATA::ID,
            SCALED_IMU2_DATA::ID,
        ] {
            self.request_message_rate(message_id, rate).await?;
        }
        Ok(())
    }

    /// `MAVLink` identity stamped on outgoing messages.
    #[must_use]
    pub fn identity(&self) -> MavlinkIdentity {
        self.inner.identity
    }

    /// The autopilot (system id, component id), if discovered yet.
    #[must_use]
    pub fn target(&self) -> Option<(u8, u8)> {
        *self.inner.target.borrow()
    }

    /// Current mode / armed / prearm snapshot.
    #[must_use]
    pub fn state(&self) -> VehicleState {
        *self.inner.state.borrow()
    }

    /// Latest fused global position, if any received yet.
    #[must_use]
    pub fn position(&self) -> Option<Position> {
        *self.inner.position.borrow()
    }

    /// Latest attitude, if any received yet.
    #[must_use]
    pub fn attitude(&self) -> Option<Attitude> {
        *self.inner.attitude.borrow()
    }

    /// Latest VFR HUD data (airspeed, throttle, climb), if any.
    #[must_use]
    pub fn flight_data(&self) -> Option<FlightData> {
        *self.inner.flight_data.borrow()
    }

    /// Latest battery state, if any.
    #[must_use]
    pub fn battery(&self) -> Option<Battery> {
        *self.inner.battery.borrow()
    }

    /// Latest secondary body IMU sample, if any.
    #[must_use]
    pub fn imu(&self) -> Option<ImuSample> {
        *self.inner.imu.borrow()
    }

    /// Watch channel for mode / armed / prearm changes.
    #[must_use]
    pub fn state_watch(&self) -> watch::Receiver<VehicleState> {
        self.inner.state.subscribe()
    }

    /// Watch channel for position updates.
    #[must_use]
    pub fn position_watch(&self) -> watch::Receiver<Option<Position>> {
        self.inner.position.subscribe()
    }

    /// Watch channel for attitude updates.
    #[must_use]
    pub fn attitude_watch(&self) -> watch::Receiver<Option<Attitude>> {
        self.inner.attitude.subscribe()
    }

    /// Watch channel for secondary body IMU updates.
    #[must_use]
    pub fn imu_watch(&self) -> watch::Receiver<Option<ImuSample>> {
        self.inner.imu.subscribe()
    }

    /// Subscribe to every raw message on the link. For advanced use; the
    /// typed accessors cover the common cases.
    #[must_use]
    pub fn messages(&self) -> broadcast::Receiver<Event> {
        self.inner.events.subscribe()
    }

    /// Send a raw message to the vehicle. Escape hatch for anything this
    /// crate does not wrap.
    pub async fn send(&self, msg: &MavMessage) -> Result<()> {
        if *self.inner.shutdown.borrow() {
            return Err(Error::ConnectionClosed);
        }
        self.inner.conn.send(&self.header(), msg).await?;
        Ok(())
    }

    /// Run a wait future, aborting with [`Error::ConnectionClosed`] as soon
    /// as the connection shuts down.
    async fn until_closed<T>(&self, wait: impl Future<Output = Result<T>>) -> Result<T> {
        let mut shutdown = self.inner.shutdown.subscribe();
        tokio::select! {
            // Poll the wait first so a final telemetry update that satisfies
            // the condition wins over a simultaneous close.
            biased;
            result = wait => result,
            _ = shutdown.wait_for(|&closed| closed) => Err(Error::ConnectionClosed),
        }
    }

    /// Header stamped on outgoing messages.
    fn header(&self) -> MavHeader {
        MavHeader {
            system_id: self.inner.identity.system_id,
            component_id: self.inner.identity.component_id,
            sequence: 0,
        }
    }

    /// The autopilot target, or (0, 0) broadcast if not yet discovered.
    fn target_or_broadcast(&self) -> (u8, u8) {
        self.target().unwrap_or((0, 0))
    }

    /// Send a `COMMAND_LONG` and wait for a matching `COMMAND_ACK`.
    ///
    /// Retries on timeout, follows `MAV_RESULT_IN_PROGRESS`, and maps any
    /// terminal non-accepted result to [`Error::CommandRejected`].
    pub async fn command_long(&self, command: MavCmd, params: [f32; 7]) -> Result<()> {
        let (target_system, target_component) = self.target_or_broadcast();
        let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            param1: params[0],
            param2: params[1],
            param3: params[2],
            param4: params[3],
            param5: params[4],
            param6: params[5],
            param7: params[6],
            command,
            target_system,
            target_component,
            confirmation: 0,
        });
        self.command_with_ack(command, &msg).await
    }

    /// Send a `COMMAND_INT` and wait for a matching `COMMAND_ACK`.
    pub async fn command_int(
        &self,
        command: MavCmd,
        frame: mavlink::dialects::ardupilotmega::MavFrame,
        params: [f32; 4],
        x: i32,
        y: i32,
        z: f32,
    ) -> Result<()> {
        let (target_system, target_component) = self.target_or_broadcast();
        let msg = MavMessage::COMMAND_INT(COMMAND_INT_DATA {
            param1: params[0],
            param2: params[1],
            param3: params[2],
            param4: params[3],
            x,
            y,
            z,
            command,
            target_system,
            target_component,
            frame,
            current: 0,
            autocontinue: 0,
        });
        self.command_with_ack(command, &msg).await
    }

    /// Shared command/ack retry machinery.
    async fn command_with_ack(&self, command: MavCmd, msg: &MavMessage) -> Result<()> {
        let mut last_err = Error::Timeout {
            what: "COMMAND_ACK",
            after: ACK_TIMEOUT,
        };
        for attempt in 0..COMMAND_RETRIES {
            let mut events = self.messages();
            self.send(msg).await?;
            trace!(?command, attempt, "command sent");
            match self.wait_for_ack(&mut events, command).await {
                Ok(()) => return Ok(()),
                Err(err @ Error::CommandRejected { .. }) => return Err(err),
                Err(err) => {
                    debug!(?command, attempt, %err, "command attempt failed, retrying");
                    last_err = err;
                }
            }
        }
        Err(last_err)
    }

    /// Wait for a `COMMAND_ACK` matching `command` on an event stream.
    async fn wait_for_ack(
        &self,
        events: &mut broadcast::Receiver<Event>,
        command: MavCmd,
    ) -> Result<()> {
        let mut deadline = tokio::time::Instant::now() + ACK_TIMEOUT;
        loop {
            let event = match tokio::time::timeout_at(deadline, events.recv()).await {
                Ok(Ok(event)) => event,
                Ok(Err(broadcast::error::RecvError::Lagged(n))) => {
                    warn!(missed = n, "event stream lagged while waiting for ack");
                    continue;
                }
                Ok(Err(broadcast::error::RecvError::Closed)) => {
                    return Err(Error::ConnectionClosed);
                }
                Err(_) => {
                    return Err(Error::Timeout {
                        what: "COMMAND_ACK",
                        after: ACK_TIMEOUT,
                    });
                }
            };
            let MavMessage::COMMAND_ACK(ack) = &event.1 else {
                continue;
            };
            if ack.command != command {
                continue;
            }
            match ack.result {
                MavResult::MAV_RESULT_ACCEPTED => return Ok(()),
                MavResult::MAV_RESULT_IN_PROGRESS => {
                    // The vehicle is working on it; give it more time.
                    deadline = tokio::time::Instant::now() + ACK_TIMEOUT * 4;
                }
                result => {
                    return Err(Error::CommandRejected { command, result });
                }
            }
        }
    }

    /// Send an autopilot-specific custom mode command and wait for heartbeat
    /// confirmation. Prefer the typed `set_mode` on [`crate::Plane`] or
    /// [`crate::Copter`], which know their platform's mode numbering.
    pub async fn set_custom_mode(&self, custom_mode: u32) -> Result<VehicleState> {
        self.command_long(
            MavCmd::MAV_CMD_DO_SET_MODE,
            [1.0, u32_to_f32_param(custom_mode), 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await?;
        self.wait_state(Duration::from_secs(5), |state| {
            state.custom_mode == custom_mode
        })
        .await
    }

    /// Arm the motors. Fails if the autopilot rejects arming (prearm checks).
    pub async fn arm(&self) -> Result<()> {
        self.command_long(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await?;
        self.wait_state(Duration::from_secs(5), |s| s.armed)
            .await
            .map(|_| ())
    }

    /// Disarm the motors. Set `force` to disarm even in flight (you probably
    /// do not want that).
    pub async fn disarm(&self, force: bool) -> Result<()> {
        let magic = if force { 21196.0 } else { 0.0 };
        self.command_long(
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            [0.0, magic, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        .await?;
        self.wait_state(Duration::from_secs(5), |s| !s.armed)
            .await
            .map(|_| ())
    }

    /// Wait until prearm checks pass and the vehicle can be armed.
    pub async fn wait_armable(&self, timeout: Duration) -> Result<()> {
        self.wait_state(timeout, |s| s.prearm_ok).await.map(|_| ())
    }

    /// Wait until the vehicle state satisfies a predicate.
    pub async fn wait_state(
        &self,
        timeout: Duration,
        mut pred: impl FnMut(&VehicleState) -> bool,
    ) -> Result<VehicleState> {
        let mut rx = self.inner.state.subscribe();
        let wait = async {
            loop {
                let current = *rx.borrow_and_update();
                if pred(&current) {
                    return Ok::<VehicleState, Error>(current);
                }
                if rx.changed().await.is_err() {
                    return Err(Error::ConnectionClosed);
                }
            }
        };
        tokio::time::timeout(timeout, self.until_closed(wait))
            .await
            .map_err(|_| Error::Timeout {
                what: "vehicle state condition",
                after: timeout,
            })?
    }

    /// Wait until the position satisfies a predicate.
    pub async fn wait_position(
        &self,
        timeout: Duration,
        mut pred: impl FnMut(&Position) -> bool,
    ) -> Result<Position> {
        let mut rx = self.inner.position.subscribe();
        let wait = async {
            loop {
                let current = *rx.borrow_and_update();
                if let Some(pos) = current
                    && pred(&pos)
                {
                    return Ok::<Position, Error>(pos);
                }
                if rx.changed().await.is_err() {
                    return Err(Error::ConnectionClosed);
                }
            }
        };
        tokio::time::timeout(timeout, self.until_closed(wait))
            .await
            .map_err(|_| Error::Timeout {
                what: "position condition",
                after: timeout,
            })?
    }

    /// Wait until relative altitude reaches `altitude_above_home`.
    pub async fn wait_altitude(
        &self,
        altitude_above_home: Length,
        timeout: Duration,
    ) -> Result<Position> {
        self.wait_position(timeout, |position| {
            position.altitude_above_home >= altitude_above_home
        })
        .await
    }

    /// Read a parameter value.
    pub async fn get_param(&self, name: &str) -> Result<f32> {
        self.get_param_typed(name).await.map(|(value, _)| value)
    }

    /// Read a parameter and its `MAVLink` storage type.
    async fn get_param_typed(&self, name: &str) -> Result<(f32, MavParamType)> {
        let param_id = encode_param_name(name)?;
        let (target_system, target_component) = self.target_or_broadcast();
        let msg = MavMessage::PARAM_REQUEST_READ(PARAM_REQUEST_READ_DATA {
            param_index: -1,
            target_system,
            target_component,
            param_id: param_id.into(),
        });
        let mut last_err = Error::Timeout {
            what: "PARAM_VALUE",
            after: ACK_TIMEOUT,
        };
        for _ in 0..COMMAND_RETRIES {
            let mut events = self.messages();
            self.send(&msg).await?;
            match self.wait_param_value(&mut events, name).await {
                Ok(value) => return Ok(value),
                Err(err) => last_err = err,
            }
        }
        Err(last_err)
    }

    /// Write a parameter and confirm the echo.
    pub async fn set_param(&self, name: &str, value: f32) -> Result<()> {
        // PARAM_SET carries the parameter's actual storage type. `ArduPilot`
        // rejects integer parameters such as MAV_GCS_SYSID if clients blindly
        // label every value REAL32.
        let (_, param_type) = self.get_param_typed(name).await?;
        // ArduPilot rounds writes to integer-typed parameters; mirror that so
        // the echo comparison cannot reject a successful write of e.g. 40.7.
        let value = match param_type {
            MavParamType::MAV_PARAM_TYPE_REAL32 | MavParamType::MAV_PARAM_TYPE_REAL64 => value,
            _ => value.round(),
        };
        let param_id = encode_param_name(name)?;
        let (target_system, target_component) = self.target_or_broadcast();
        let msg = MavMessage::PARAM_SET(PARAM_SET_DATA {
            param_value: value,
            target_system,
            target_component,
            param_id: param_id.into(),
            param_type,
        });
        let mut last_err = Error::Timeout {
            what: "PARAM_VALUE",
            after: ACK_TIMEOUT,
        };
        for _ in 0..COMMAND_RETRIES {
            let mut events = self.messages();
            self.send(&msg).await?;
            match self.wait_param_value(&mut events, name).await {
                Ok((got, _)) => {
                    if (got - value).abs() <= f32::EPSILON.max(value.abs() * 1e-6) {
                        return Ok(());
                    }
                    return Err(Error::ParamSetMismatch {
                        name: name.to_owned(),
                        wrote: value,
                        got,
                    });
                }
                Err(err) => last_err = err,
            }
        }
        Err(last_err)
    }

    /// Wait for a `PARAM_VALUE` for `name` on an event stream.
    async fn wait_param_value(
        &self,
        events: &mut broadcast::Receiver<Event>,
        name: &str,
    ) -> Result<(f32, MavParamType)> {
        let deadline = tokio::time::Instant::now() + ACK_TIMEOUT;
        loop {
            let event = match tokio::time::timeout_at(deadline, events.recv()).await {
                Ok(Ok(event)) => event,
                Ok(Err(broadcast::error::RecvError::Lagged(_))) => continue,
                Ok(Err(broadcast::error::RecvError::Closed)) => {
                    return Err(Error::ConnectionClosed);
                }
                Err(_) => {
                    return Err(Error::Timeout {
                        what: "PARAM_VALUE",
                        after: ACK_TIMEOUT,
                    });
                }
            };
            let MavMessage::PARAM_VALUE(pv) = &event.1 else {
                continue;
            };
            if decode_param_name(&pv.param_id[..]) == name {
                return Ok((pv.param_value, pv.param_type));
            }
        }
    }
}

/// Encode a parameter name into the fixed 16-byte `MAVLink` field.
fn encode_param_name(name: &str) -> Result<[u8; 16]> {
    let bytes = name.as_bytes();
    if bytes.len() > 16 {
        return Err(Error::ParamNameTooLong(name.to_owned()));
    }
    let mut id = [0u8; 16];
    id[..bytes.len()].copy_from_slice(bytes);
    Ok(id)
}

/// Decode a fixed 16-byte parameter name field.
fn decode_param_name(id: &[u8]) -> String {
    let end = id.iter().position(|&b| b == 0).unwrap_or(id.len());
    String::from_utf8_lossy(&id[..end]).into_owned()
}

/// Convert a small integer (message id, custom mode) to the f32 `MAVLink`
/// param encoding.
///
/// Message ids and mode numbers fit in f32's 24-bit mantissa, so this is
/// lossless for every value `ArduPilot` will ever send us.
fn u32_to_f32_param(value: u32) -> f32 {
    debug_assert!(
        value < (1 << 24),
        "value {value} does not fit losslessly in f32"
    );
    value as f32
}

/// Background task: receive messages, update watches, broadcast events.
///
/// Exits when the transport dies or the last user handle drops; the shared
/// state (and with it the transport) is released on exit.
async fn recv_loop(inner: Arc<Inner>) {
    let mut shutdown = inner.shutdown.subscribe();
    loop {
        let received = tokio::select! {
            received = inner.conn.recv() => received,
            _ = shutdown.wait_for(|&closed| closed) => break,
        };
        let (header, message) = match received {
            Ok(m) => m,
            Err(mavlink::error::MessageReadError::Io(err)) => {
                warn!(%err, "mavlink connection closed");
                break;
            }
            Err(mavlink::error::MessageReadError::Parse(err)) => {
                trace!(?err, "skipping unparseable message");
                continue;
            }
        };
        handle_message(&inner, header, &message);
        // Only fails when there are no subscribers, which is fine.
        let _ = inner.events.send(Arc::new((header, message)));
    }
    inner.shutdown.send_replace(true);
}

/// Update the typed telemetry watches from one message.
fn handle_message(inner: &Inner, header: MavHeader, msg: &MavMessage) {
    match msg {
        MavMessage::HEARTBEAT(hb) => handle_heartbeat(inner, header, hb),
        MavMessage::GLOBAL_POSITION_INT(position) => handle_position(inner, position),
        MavMessage::ATTITUDE(attitude) => handle_attitude(inner, attitude),
        MavMessage::VFR_HUD(flight_data) => handle_flight_data(inner, flight_data),
        MavMessage::SCALED_IMU2(imu) => handle_imu(inner, imu),
        MavMessage::SYS_STATUS(status) => handle_system_status(inner, status),
        MavMessage::STATUSTEXT(st) => {
            let end = st
                .text
                .iter()
                .position(|&b| b == 0)
                .unwrap_or(st.text.len());
            let text = String::from_utf8_lossy(&st.text[..end]);
            debug!(severity = ?st.severity, %text, "statustext");
        }
        _ => {}
    }
}

/// Update global position and NED velocity telemetry.
fn handle_position(inner: &Inner, position: &GLOBAL_POSITION_INT_DATA) {
    let heading =
        (position.hdg != u16::MAX).then(|| Angle::new::<degree>(f64::from(position.hdg) / 100.0));
    inner.position.send_replace(Some(Position {
        latitude: Angle::new::<degree>(f64::from(position.lat) / 1e7),
        longitude: Angle::new::<degree>(f64::from(position.lon) / 1e7),
        altitude_msl: Length::new::<meter>(f64::from(position.alt) / 1000.0),
        altitude_above_home: Length::new::<meter>(f64::from(position.relative_alt) / 1000.0),
        velocity: sguaba::vector!(
            n = Velocity::new::<meter_per_second>(f64::from(position.vx) / 100.0),
            e = Velocity::new::<meter_per_second>(f64::from(position.vy) / 100.0),
            d = Velocity::new::<meter_per_second>(f64::from(position.vz) / 100.0);
            in LocalNedFrame
        ),
        heading,
    }));
}

/// Update orientation and body angular velocity telemetry.
fn handle_attitude(inner: &Inner, attitude: &ATTITUDE_DATA) {
    inner.attitude.send_replace(Some(Attitude {
        orientation: Orientation::<LocalNedFrame>::tait_bryan_builder()
            .yaw(Angle::new::<radian>(f64::from(attitude.yaw)))
            .pitch(Angle::new::<radian>(f64::from(attitude.pitch)))
            .roll(Angle::new::<radian>(f64::from(attitude.roll)))
            .build(),
        angular_velocity: BodyRates {
            roll: AngularVelocity::new::<radian_per_second>(f64::from(attitude.rollspeed)),
            pitch: AngularVelocity::new::<radian_per_second>(f64::from(attitude.pitchspeed)),
            yaw: AngularVelocity::new::<radian_per_second>(f64::from(attitude.yawspeed)),
        },
    }));
}

/// Update pilot-facing air data.
fn handle_flight_data(inner: &Inner, flight_data: &VFR_HUD_DATA) {
    inner.flight_data.send_replace(Some(FlightData {
        airspeed: Velocity::new::<meter_per_second>(f64::from(flight_data.airspeed)),
        groundspeed: Velocity::new::<meter_per_second>(f64::from(flight_data.groundspeed)),
        altitude_msl: Length::new::<meter>(f64::from(flight_data.alt)),
        climb_rate: Velocity::new::<meter_per_second>(f64::from(flight_data.climb)),
        heading: Angle::new::<degree>(f64::from(flight_data.heading)),
        throttle: Ratio::new::<percent>(f64::from(flight_data.throttle)),
    }));
}

/// Update secondary body IMU telemetry.
fn handle_imu(inner: &Inner, imu: &SCALED_IMU2_DATA) {
    inner.imu.send_replace(Some(ImuSample {
        time_since_boot: Duration::from_millis(u64::from(imu.time_boot_ms)),
        acceleration: sguaba::vector!(
            f = Acceleration::new::<standard_gravity>(f64::from(imu.xacc) / 1000.0),
            r = Acceleration::new::<standard_gravity>(f64::from(imu.yacc) / 1000.0),
            d = Acceleration::new::<standard_gravity>(f64::from(imu.zacc) / 1000.0);
            in VehicleBodyFrame
        ),
        angular_velocity: BodyRates {
            roll: AngularVelocity::new::<radian_per_second>(f64::from(imu.xgyro) / 1000.0),
            pitch: AngularVelocity::new::<radian_per_second>(f64::from(imu.ygyro) / 1000.0),
            yaw: AngularVelocity::new::<radian_per_second>(f64::from(imu.zgyro) / 1000.0),
        },
    }));
}

/// Update battery and prearm health telemetry.
fn handle_system_status(inner: &Inner, status: &SYS_STATUS_DATA) {
    let current = (status.current_battery >= 0)
        .then(|| ElectricCurrent::new::<ampere>(f64::from(status.current_battery) / 100.0));
    let remaining = (status.battery_remaining >= 0)
        .then(|| Ratio::new::<percent>(f64::from(status.battery_remaining)));
    inner.battery.send_replace(Some(Battery {
        voltage: ElectricPotential::new::<volt>(f64::from(status.voltage_battery) / 1000.0),
        current,
        remaining,
    }));
    let prearm = MavSysStatusSensor::MAV_SYS_STATUS_PREARM_CHECK;
    let prearm_ok = status.onboard_control_sensors_enabled.contains(prearm)
        && status.onboard_control_sensors_health.contains(prearm);
    inner.state.send_if_modified(|state| {
        if state.prearm_ok == prearm_ok {
            false
        } else {
            state.prearm_ok = prearm_ok;
            true
        }
    });
}

/// Handle a heartbeat: discover the autopilot and track mode/armed state.
fn handle_heartbeat(inner: &Inner, header: MavHeader, hb: &HEARTBEAT_DATA) {
    if hb.autopilot != MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA {
        return;
    }
    if matches!(
        hb.mavtype,
        MavType::MAV_TYPE_GCS | MavType::MAV_TYPE_ONBOARD_CONTROLLER
    ) {
        return;
    }
    let discovered = (header.system_id, header.component_id);
    inner.target.send_if_modified(|target| {
        if *target == Some(discovered) {
            false
        } else {
            debug!(
                system = discovered.0,
                component = discovered.1,
                "autopilot discovered"
            );
            *target = Some(discovered);
            true
        }
    });
    let armed = hb
        .base_mode
        .contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED);
    inner.state.send_if_modified(|state| {
        if state.custom_mode == hb.custom_mode && state.armed == armed {
            false
        } else {
            state.custom_mode = hb.custom_mode;
            state.armed = armed;
            true
        }
    });
}

/// Background task: send a heartbeat every second while the vehicle handle
/// is alive.
async fn heartbeat_loop(inner: std::sync::Weak<Inner>) {
    let mut ticker = tokio::time::interval(Duration::from_secs(1));
    loop {
        ticker.tick().await;
        let Some(inner) = inner.upgrade() else { break };
        if *inner.shutdown.borrow() {
            break;
        }
        let heartbeat = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: inner.identity.role.mav_type(),
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        });
        let header = MavHeader {
            system_id: inner.identity.system_id,
            component_id: inner.identity.component_id,
            sequence: 0,
        };
        if let Err(err) = inner.conn.send(&header, &heartbeat).await {
            warn!(%err, "failed to send heartbeat");
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use anyhow::{Result, ensure};
    use tokio::io::AsyncReadExt;
    use tokio::net::TcpListener;

    /// Bind a local listener and connect a vehicle to it over TCP.
    async fn connect_pair() -> Result<(Vehicle, tokio::net::TcpStream)> {
        let listener = TcpListener::bind("127.0.0.1:0").await?;
        let port = listener.local_addr()?.port();
        let vehicle = Vehicle::connect(&format!("tcpout:127.0.0.1:{port}")).await?;
        let (stream, _) = listener.accept().await?;
        Ok((vehicle, stream))
    }

    #[tokio::test]
    async fn dropping_last_handle_closes_connection() -> Result<()> {
        let (vehicle, mut stream) = connect_pair().await?;
        let clone = vehicle.clone();
        drop(vehicle);
        drop(clone);
        // The peer must see EOF once the IO tasks release the transport;
        // heartbeat bytes may arrive first.
        let eof = async {
            let mut buf = [0_u8; 256];
            loop {
                if stream.read(&mut buf).await? == 0 {
                    return Ok::<(), std::io::Error>(());
                }
            }
        };
        tokio::time::timeout(Duration::from_secs(5), eof).await??;
        Ok(())
    }

    #[tokio::test]
    async fn waiters_observe_peer_close() -> Result<()> {
        let (vehicle, stream) = connect_pair().await?;
        drop(stream);
        // A predicate that never matches must fail with ConnectionClosed,
        // not run into its own timeout.
        let result = vehicle.wait_state(Duration::from_secs(5), |_| false).await;
        ensure!(
            matches!(result, Err(Error::ConnectionClosed)),
            "expected ConnectionClosed, got {result:?}"
        );
        let ready = vehicle.wait_ready(Duration::from_secs(5)).await;
        ensure!(
            matches!(ready, Err(Error::ConnectionClosed)),
            "expected ConnectionClosed, got {ready:?}"
        );
        Ok(())
    }
}
