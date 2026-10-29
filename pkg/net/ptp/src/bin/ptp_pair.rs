#[macro_use]
extern crate common;
#[macro_use]
extern crate macros;

/*
Runs one TimeSyncNode as a leader or a follower without the mocap manager, to
test the sync between two cameras.

    ptp_pair --mode=follower --iface=eth0
    ptp_pair --mode=leader --iface=eth0 --follower=192.168.2.221

The follower prints one line per round it used, and one per round it skipped
because its packets were queued. FollowerConfig fields can be overridden with
--max_network_rtt and --max_rtt_above_min (seconds).
*/

use std::sync::Arc;
use std::time::{Duration, Instant};

use common::errors::*;
use executor_multitask::RootResource;
use net::route::NetworkInterfaceRoute;
use ptp_proto::ptp::*;

#[derive(Args)]
struct Args {
    mode: Mode,

    iface: String,

    #[arg(default = 8002)]
    rpc_port: u16,

    #[arg(default = 319)]
    ptp_port: u16,

    // Leader only: IP address of the follower.
    follower: Option<String>,

    max_network_rtt: Option<String>,

    max_rtt_above_min: Option<String>,
}

#[derive(Args)]
enum Mode {
    #[arg(name = "leader")]
    Leader,
    #[arg(name = "follower")]
    Follower,
}

fn interface_route(name: &str) -> Result<NetworkInterfaceRoute> {
    let index = std::fs::read_to_string(format!("/sys/class/net/{}/ifindex", name))?
        .trim()
        .parse()?;

    Ok(NetworkInterfaceRoute {
        name: name.to_string(),
        addr: "0.0.0.0".parse()?,
        index,
    })
}

async fn print_follower_status(inst: Arc<ptp::TimeSyncNode>) -> Result<()> {
    let start = Instant::now();
    let mut last_age = f64::MAX;
    let mut last_queued = 0;

    loop {
        executor::sleep(Duration::from_millis(20)).await?;

        let status = inst.status().await?;
        let follower = status.follower();
        let t = start.elapsed().as_secs_f64();

        if follower.num_queued_rounds() != last_queued {
            last_queued = follower.num_queued_rounds();
            println!("{:.3} queued total {}", t, last_queued);
        }

        if follower.got_sync() && follower.last_sync_age() < last_age {
            println!(
                "{:.3} err_ns {} rtt_ns {} min_rtt_ns {}",
                t,
                (follower.last_leader_error() * 1e9).round(),
                (follower.last_leader_rtt() * 1e9).round(),
                (follower.min_rtt() * 1e9).round()
            );
        }
        last_age = follower.last_sync_age();
    }
}

#[executor_main]
async fn main() -> Result<()> {
    let args = common::args::parse_args::<Args>()?;

    net::enable_hardware_timestamp_filters(&args.iface)?;

    let ptp_device = Arc::new(ptp::PTPDevice::open_default()?);

    let ptp_socket = ptp::TimestampedUdpSocket::create(
        format!("0.0.0.0:{}", args.ptp_port).parse()?,
        &args.iface,
    )
    .await?;

    let basic_node =
        ptp_core::BasicTimeNode::create("0.0.0.0:400".parse()?, interface_route(&args.iface)?)
            .await?;

    let inst = Arc::new(ptp::TimeSyncNode::create(ptp_device, ptp_socket, Arc::new(basic_node)).await);

    let mut config = ptp::TimeSyncNode::default_config_template();
    config.clear_basic_client();

    match args.mode {
        Mode::Leader => {
            let follower = args
                .follower
                .as_ref()
                .ok_or_else(|| err_msg("--follower is required for a leader"))?;

            config.clear_follower();
            let peer = config.leader_mut().new_followers();
            peer.set_rpc_addr(&format!("{}:{}", follower, args.rpc_port));
            peer.set_ptp_addr(&format!("{}:{}", follower, args.ptp_port));
        }
        Mode::Follower => {
            config.clear_leader();
            if let Some(v) = &args.max_network_rtt {
                config.follower_mut().set_max_network_rtt(v.parse::<f32>()?);
            }
            if let Some(v) = &args.max_rtt_above_min {
                config.follower_mut().set_max_rtt_above_min(v.parse::<f32>()?);
            }
        }
    }

    inst.configure(&config).await?;

    let service = RootResource::new();
    service.register_dependency(inst.clone()).await;

    let mut server = rpc::Http2Server::new(Some(args.rpc_port));
    server.add_service(inst.clone().into_service())?;
    service.register_dependency(server.start()).await;

    if let Mode::Follower = args.mode {
        service
            .spawn_interruptable("ptp_pair::print_follower_status", print_follower_status(inst.clone()))
            .await;
    }

    service.wait().await
}
