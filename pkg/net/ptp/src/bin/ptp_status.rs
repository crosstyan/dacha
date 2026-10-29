#[macro_use]
extern crate common;
#[macro_use]
extern crate macros;

/*
Prints the status of a TimeSyncNode, like pmc does for ptp4l.

    ptp_status --addr=192.168.2.221:8002
*/

use std::sync::Arc;
use std::time::Duration;

use common::errors::*;
use ptp_proto::ptp::*;

#[derive(Args)]
struct Args {
    // RPC address of the node.
    #[arg(default = "127.0.0.1:8002")]
    addr: String,
}

fn ns(seconds: f64) -> String {
    format!("{}", (seconds * 1e9).round())
}

/// 'rtt_window': also print the minimum RTT and the queued rounds, which only a follower
/// of a leader has. They are printed even before the first usable sync, when they tell why
/// there is none (a minimum of 0 is printed as such).
fn print_follower(name: &str, s: &FollowerStatus, rtt_window: bool) {
    println!("{}", name);
    println!("  got_sync           {}", s.got_sync());
    if s.got_sync() {
        println!("  last_error_ns      {}", ns(s.last_leader_error()));
        println!("  last_rtt_ns        {}", ns(s.last_leader_rtt()));
        println!("  last_sync_age_s    {:.3}", s.last_sync_age());
    }
    if rtt_window {
        println!("  min_rtt_ns         {}", ns(s.min_rtt()));
        println!("  queued_rounds      {}", s.num_queued_rounds());
    }
}

fn print_status(status: &StatusResponse) {
    let config = status.config();

    let mut roles = vec![];
    if config.has_leader() {
        roles.push(format!("leader of {}", config.leader().followers().len()));
    }
    if config.has_follower() {
        roles.push("follower".to_string());
    }
    if config.has_basic_client() {
        roles.push(format!("time server {}", config.basic_client().server_addr()));
    }
    if roles.is_empty() {
        roles.push("not configured".to_string());
    }
    println!("role                 {}", roles.join(", "));

    if config.has_follower() {
        print_follower("follower", status.follower(), true);
    }

    if config.has_basic_client() {
        print_follower("basic_client", status.basic_client(), false);
    }

    if status.has_system_clock() {
        let s = status.system_clock();
        println!("system_clock");
        println!("  steering           {}", s.steering());
        if !s.steering() {
            println!("  reason             {}", s.reason());
        }
        if s.last_sample_age() != 0.0 {
            println!("  last_error_ns      {}", ns(s.last_error()));
            println!("  last_read_delay_ns {}", ns(s.last_read_delay()));
            println!("  last_sample_age_s  {:.3}", s.last_sample_age());
        }
        println!("  freq_ppm           {:.3}", s.freq_ppm());
        println!("  num_steps          {}", s.num_steps());
    }

    if status.has_leader() {
        println!("leader");
        for f in status.leader().followers() {
            println!("  follower {} (ptp {})", f.rpc_addr(), f.ptp_addr());
            if f.got_round() {
                println!("    last_round_age_s {:.3}", f.last_round_age());
            } else {
                println!("    no completed round");
            }
            println!(
                "    failures         {} total, {} in a row",
                f.num_failures(),
                f.num_consecutive_failures()
            );
            if !f.last_error().is_empty() {
                println!("    last_error       {}", f.last_error());
            }
        }
    }
}

#[executor_main]
async fn main() -> Result<()> {
    let args = common::args::parse_args::<Args>()?;

    let channel = Arc::new(rpc::Http2Channel::create(format!("http://{}", args.addr).as_str()).await?);
    let stub = TimeSyncStub::new(channel);

    let request_context = rpc::ClientRequestContext::default();
    let status = executor::timeout(
        Duration::from_secs(2),
        stub.Status(&request_context, &StatusRequest::default()),
    )
    .await
    .map_err(|_| format_err!("No answer from {} within 2 s", args.addr))?
    .result?;

    print_status(&status);

    Ok(())
}
