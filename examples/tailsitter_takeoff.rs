//! Guided VTOL takeoff demo/diagnostic for a quadplane tailsitter.
//!
//! Run SITL first, e.g.:
//! ```sh
//! arduplane --model quadplane-copter_tailsitter -w -S --speedup 10 \
//!   --defaults quadplane.parm,quadplane-copter_tailsitter.parm
//! ```
//! then `cargo run --example tailsitter_takeoff`.

use std::time::Duration;

use mavrs::uom::si::length::meter;
use mavrs::{Length, Mode, QuadPlane};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();

    let addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "tcpout:127.0.0.1:5760".to_owned());
    let quad = QuadPlane::connect(&addr).await?;
    quad.wait_ready(Duration::from_secs(60)).await?;
    println!("connected, state: {:?}", quad.state());

    // Print telemetry in the background.
    {
        let quad = quad.clone();
        tokio::spawn(async move {
            loop {
                let s = quad.state();
                let p = quad.position();
                println!(
                    "mode={} armed={} prearm={} alt_rel={:?}",
                    quad.mode(),
                    s.armed,
                    s.prearm_ok,
                    p.map(|position| position.altitude_above_home.get::<meter>())
                );
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        });
    }

    quad.wait_armable(Duration::from_secs(120)).await?;
    println!("armable, taking off");
    let pos = quad
        .vtol_takeoff(Length::new::<meter>(20.0), Duration::from_secs(120))
        .await?;
    println!(
        "takeoff complete at {} m",
        pos.altitude_above_home.get::<meter>()
    );
    quad.set_mode(Mode::QLoiter).await?;
    tokio::time::sleep(Duration::from_secs(5)).await;
    quad.vtol_land(Duration::from_secs(300)).await?;
    println!("landed and disarmed");
    Ok(())
}
