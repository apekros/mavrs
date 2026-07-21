//! Mission building and upload.
//!
//! A thin, typed layer over the `MAVLink` mission protocol. Build a
//! [`Mission`] with the item constructors, then push it with
//! [`crate::Vehicle::upload_mission`].

use std::time::Duration;

use mavlink::dialects::ardupilotmega::{
    MISSION_ACK_DATA, MISSION_COUNT_DATA, MISSION_ITEM_INT_DATA, MavCmd, MavFrame, MavMessage,
    MavMissionResult,
};
use tokio::sync::broadcast;
use tracing::{debug, trace};

use crate::error::{Error, Result};
use crate::types::Target;
use crate::vehicle::Vehicle;
use uom::si::angle::degree;
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;

/// How long to wait for each mission protocol message from the vehicle.
const MISSION_TIMEOUT: Duration = Duration::from_secs(5);

/// One mission item, in sane units.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MissionItem {
    /// The `MAVLink` command for this item.
    command: MavCmd,
    /// The coordinate frame for `x`/`y`/`z`.
    frame: MavFrame,
    /// Params 1-4 as defined by the command.
    params: [f32; 4],
    /// Latitude in `degE7` or command-dependent local X.
    x: i32,
    /// Longitude in `degE7` or command-dependent local Y.
    y: i32,
    /// Command-dependent raw Z value.
    z: f32,
}

impl MissionItem {
    /// A navigation waypoint.
    #[must_use]
    pub fn waypoint(target: Target) -> Self {
        Self {
            command: MavCmd::MAV_CMD_NAV_WAYPOINT,
            frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            params: [0.0; 4],
            x: target.latitude_e7(),
            y: target.longitude_e7(),
            z: target.altitude_above_home.get::<meter>() as f32,
        }
    }

    /// Fixed-wing takeoff, climbing to the requested altitude at up to `pitch`.
    #[must_use]
    pub fn takeoff(pitch: Angle, altitude_above_home: Length) -> Self {
        Self {
            command: MavCmd::MAV_CMD_NAV_TAKEOFF,
            frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            params: [pitch.get::<degree>() as f32, 0.0, 0.0, 0.0],
            x: 0,
            y: 0,
            z: altitude_above_home.get::<meter>() as f32,
        }
    }

    /// VTOL takeoff straight up to the requested altitude.
    #[must_use]
    pub fn vtol_takeoff(altitude_above_home: Length) -> Self {
        Self {
            command: MavCmd::MAV_CMD_NAV_VTOL_TAKEOFF,
            frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            params: [0.0; 4],
            x: 0,
            y: 0,
            z: altitude_above_home.get::<meter>() as f32,
        }
    }

    /// VTOL landing at a location (zero lat/lon lands at the current
    /// position).
    #[must_use]
    pub fn vtol_land(target: Option<Target>) -> Self {
        let (x, y) = target.map_or((0, 0), |target| {
            (target.latitude_e7(), target.longitude_e7())
        });
        Self {
            command: MavCmd::MAV_CMD_NAV_VTOL_LAND,
            frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            params: [0.0; 4],
            x,
            y,
            z: 0.0,
        }
    }

    /// Return to launch.
    #[must_use]
    pub const fn rtl() -> Self {
        Self {
            command: MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH,
            frame: MavFrame::MAV_FRAME_MISSION,
            params: [0.0; 4],
            x: 0,
            y: 0,
            z: 0.0,
        }
    }

    /// Loiter at a location for `duration`.
    #[must_use]
    pub fn loiter_time(target: Target, duration: Duration) -> Self {
        Self {
            command: MavCmd::MAV_CMD_NAV_LOITER_TIME,
            frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            params: [duration.as_secs_f32(), 0.0, 0.0, 0.0],
            x: target.latitude_e7(),
            y: target.longitude_e7(),
            z: target.altitude_above_home.get::<meter>() as f32,
        }
    }
}

/// A mission: an ordered list of items.
///
/// Item 0 (the home location) is added automatically on upload; do not
/// include it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Mission {
    /// The mission items, in execution order.
    items: Vec<MissionItem>,
}

impl Mission {
    /// An empty mission.
    #[must_use]
    pub const fn new() -> Self {
        Self { items: Vec::new() }
    }

    /// Append an item, builder style.
    #[must_use]
    pub fn then(mut self, item: MissionItem) -> Self {
        self.items.push(item);
        self
    }

    /// The items in this mission (not counting the implicit home item).
    #[must_use]
    pub fn items(&self) -> &[MissionItem] {
        &self.items
    }
}

impl Vehicle {
    /// Upload a mission, replacing whatever is on the vehicle.
    pub async fn upload_mission(&self, mission: &Mission) -> Result<()> {
        let (target_system, target_component) = self.target().unwrap_or((0, 0));
        // +1 for the implicit home item at seq 0.
        let count = u16::try_from(mission.items().len() + 1)
            .map_err(|_| Error::MissionRejected(MavMissionResult::MAV_MISSION_NO_SPACE))?;

        let mut events = self.messages();
        self.send(&MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
            count,
            target_system,
            target_component,
        }))
        .await?;

        // A conforming autopilot requests each item once, maybe retransmits a
        // few. Bound the exchange so a peer stuck re-requesting the same item
        // cannot spin this loop forever.
        let max_messages = usize::from(count) * 4 + 8;
        for _ in 0..max_messages {
            let msg = next_mission_message(&mut events).await?;
            let requested_seq = match msg {
                MissionUploadMsg::Request(seq) => seq,
                MissionUploadMsg::Ack(MavMissionResult::MAV_MISSION_ACCEPTED) => return Ok(()),
                MissionUploadMsg::Ack(result) => return Err(Error::MissionRejected(result)),
            };
            trace!(seq = requested_seq, "mission item requested");
            let item = build_item(mission, requested_seq, target_system, target_component)?;
            self.send(&MavMessage::MISSION_ITEM_INT(item)).await?;
        }
        Err(Error::MissionTransferStalled {
            messages: max_messages,
        })
    }

    /// Clear the mission stored on the vehicle.
    pub async fn clear_mission(&self) -> Result<()> {
        let (target_system, target_component) = self.target().unwrap_or((0, 0));
        let mut events = self.messages();
        self.send(&MavMessage::MISSION_CLEAR_ALL(
            mavlink::dialects::ardupilotmega::MISSION_CLEAR_ALL_DATA {
                target_system,
                target_component,
            },
        ))
        .await?;
        match next_mission_message(&mut events).await? {
            MissionUploadMsg::Ack(MavMissionResult::MAV_MISSION_ACCEPTED) => {
                debug!("mission cleared");
                Ok(())
            }
            MissionUploadMsg::Ack(result) => Err(Error::MissionRejected(result)),
            MissionUploadMsg::Request(_) => {
                Err(Error::MissionRejected(MavMissionResult::MAV_MISSION_ERROR))
            }
        }
    }
}

/// Messages we care about during a mission upload.
enum MissionUploadMsg {
    /// The vehicle wants item `seq`.
    Request(u16),
    /// The vehicle finished (or aborted) the transfer.
    Ack(MavMissionResult),
}

/// Wait for the next mission request or ack from the vehicle.
async fn next_mission_message(
    events: &mut broadcast::Receiver<std::sync::Arc<(mavlink::MavHeader, MavMessage)>>,
) -> Result<MissionUploadMsg> {
    let deadline = tokio::time::Instant::now() + MISSION_TIMEOUT;
    loop {
        let event = match tokio::time::timeout_at(deadline, events.recv()).await {
            Ok(Ok(event)) => event,
            Ok(Err(broadcast::error::RecvError::Lagged(_))) => continue,
            Ok(Err(broadcast::error::RecvError::Closed)) => return Err(Error::ConnectionClosed),
            Err(_) => {
                return Err(Error::Timeout {
                    what: "mission protocol message",
                    after: MISSION_TIMEOUT,
                });
            }
        };
        match &event.1 {
            // MISSION_REQUEST is deprecated in the spec but ArduPilot still
            // sends it to peers it has not seen MAVLink 2 traffic from, so we
            // answer it (with MISSION_ITEM_INT, as the spec asks).
            #[expect(deprecated, reason = "ArduPilot still emits MISSION_REQUEST")]
            MavMessage::MISSION_REQUEST(r) => return Ok(MissionUploadMsg::Request(r.seq)),
            MavMessage::MISSION_REQUEST_INT(r) => return Ok(MissionUploadMsg::Request(r.seq)),
            MavMessage::MISSION_ACK(MISSION_ACK_DATA { mavtype, .. }) => {
                return Ok(MissionUploadMsg::Ack(*mavtype));
            }
            _ => {}
        }
    }
}

/// Build the wire item for a sequence number. Seq 0 is the implicit home
/// placeholder (`ArduPilot` overwrites it with the actual home).
fn build_item(
    mission: &Mission,
    seq: u16,
    target_system: u8,
    target_component: u8,
) -> Result<MISSION_ITEM_INT_DATA> {
    let item = if seq == 0 {
        MissionItem {
            command: MavCmd::MAV_CMD_NAV_WAYPOINT,
            frame: MavFrame::MAV_FRAME_GLOBAL,
            params: [0.0; 4],
            x: 0,
            y: 0,
            z: 0.0,
        }
    } else {
        *mission
            .items()
            .get(usize::from(seq) - 1)
            .ok_or(Error::MissionRejected(
                MavMissionResult::MAV_MISSION_INVALID_SEQUENCE,
            ))?
    };
    Ok(MISSION_ITEM_INT_DATA {
        param1: item.params[0],
        param2: item.params[1],
        param3: item.params[2],
        param4: item.params[3],
        x: item.x,
        y: item.y,
        z: item.z,
        seq,
        command: item.command,
        target_system,
        target_component,
        frame: item.frame,
        current: 0,
        autocontinue: 1,
    })
}
