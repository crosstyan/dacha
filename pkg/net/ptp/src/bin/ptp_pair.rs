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

--time_server=host:port gives the leader a basic client that syncs its PTP
clock to that time server (the mocap manager's role). The time server can be
run on its own, for example on a PC without root:

    ptp_pair --mode=time_server --iface=eno1 --time_port=4000

--sync_system_clock steers the node's system clock to its PTP clock (like
phc2sys). Stop phc2sys and any NTP daemon first.
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

    // Leader only: address (host:port) of a time server for the basic client.
    time_server: Option<String>,

    // UDP port of this node's BasicTimeNode (the time server in time_server mode).
    #[arg(default = 400)]
    time_port: u16,

    #[arg(default = false)]
    sync_system_clock: bool,

    max_network_rtt: Option<String>,

    max_rtt_above_min: Option<String>,
}

#[derive(Args)]
enum Mode {
    #[arg(name = "leader")]
    Leader,
    #[arg(name = "follower")]
    Follower,
    #[arg(name = "time_server")]
    TimeServer,
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

async fn print_status(inst: Arc<ptp::TimeSyncNode>) -> Result<()> {
    let start = Instant::now();
    let mut last_age = f64::MAX;
    let mut last_queued = 0;
    let mut last_basic_age = f64::MAX;
    let mut last_system_age = f64::MAX;
    let mut last_system_reason = String::new();

    loop {
        executor::sleep(Duration::from_millis(20)).await?;

        let status = inst.status().await?;
        let t = start.elapsed().as_secs_f64();

        if status.config().has_basic_client() {
            let basic = status.basic_client();
            if basic.got_sync() && basic.last_sync_age() < last_basic_age {
                println!(
                    "{:.3} basic err_ns {} rtt_ns {}",
                    t,
                    (basic.last_leader_error() * 1e9).round(),
                    (basic.last_leader_rtt() * 1e9).round()
                );
            }
            last_basic_age = basic.last_sync_age();
        }

        if status.has_system_clock() {
            let system = status.system_clock();
            if system.steering() && system.last_sample_age() < last_system_age {
                println!(
                    "{:.3} sys err_ns {} delay_ns {} freq_ppm {:.3} steps {}",
                    t,
                    (system.last_error() * 1e9).round(),
                    (system.last_read_delay() * 1e9).round(),
                    system.freq_ppm(),
                    system.num_steps()
                );
            }
            last_system_age = system.last_sample_age();

            if !system.steering() && system.reason() != last_system_reason {
                println!("{:.3} sys not steering: {}", t, system.reason());
            }
            last_system_reason = system.reason().to_string();
        }

        if !status.config().has_follower() {
            continue;
        }

        let follower = status.follower();

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

    if let Mode::TimeServer = args.mode {
        let basic_node = ptp_core::BasicTimeNode::create(
            format!("0.0.0.0:{}", args.time_port).parse()?,
            interface_route(&args.iface)?,
        )
        .await?;
        println!("Time server on {}", basic_node.local_addr()?.to_string());

        let service = RootResource::new();
        service.register_dependency(Arc::new(basic_node)).await;
        return service.wait().await;
    }

    net::enable_hardware_timestamp_filters(&args.iface)?;

    let ptp_device = Arc::new(ptp::PTPDevice::open_default()?);

    let ptp_socket = ptp::TimestampedUdpSocket::create(
        format!("0.0.0.0:{}", args.ptp_port).parse()?,
        &args.iface,
    )
    .await?;

    let basic_node =
        ptp_core::BasicTimeNode::create(
            format!("0.0.0.0:{}", args.time_port).parse()?,
            interface_route(&args.iface)?,
        )
        .await?;

    let inst = Arc::new(ptp::TimeSyncNode::create(ptp_device, ptp_socket, Arc::new(basic_node)).await);

    let mut config = ptp::TimeSyncNode::default_config_template();

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

            if let Some(server) = &args.time_server {
                config.basic_client_mut().set_server_addr(server);
            } else {
                config.clear_basic_client();
            }
        }
        Mode::Follower => {
            config.clear_leader();
            if let Some(v) = &args.max_network_rtt {
                config.follower_mut().set_max_network_rtt(v.parse::<f32>()?);
            }
            if let Some(v) = &args.max_rtt_above_min {
                config.follower_mut().set_max_rtt_above_min(v.parse::<f32>()?);
            }
            config.clear_basic_client();
        }
        Mode::TimeServer => unreachable!(),
    }

    inst.configure(&config).await?;

    if args.sync_system_clock {
        inst.set_system_clock_sync(Some(ptp_core::default_system_clock_sync())).await?;
    }

    let service = RootResource::new();
    service.register_dependency(inst.clone()).await;

    let mut server = rpc::Http2Server::new(Some(args.rpc_port));
    server.add_service(inst.clone().into_service())?;
    service.register_dependency(server.start()).await;

    service
        .spawn_interruptable("ptp_pair::print_status", print_status(inst.clone()))
        .await;

    service.wait().await
}
