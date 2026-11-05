#[macro_use]
extern crate common;
#[macro_use]
extern crate macros;

/*
Talks to one camera's mocap.Camera service directly, without the manager.

    mocap_camera_probe --addr=192.168.2.222:82 --command=status
    mocap_camera_probe --addr=192.168.2.222:82 --command=configure --config="frame_rate: 120 exposure_micros: 500"
    mocap_camera_probe --addr=192.168.2.222:82 --command=blobs --count=1200
    mocap_camera_probe --addr=192.168.2.222:82 --command=frame --out=frame.jpg

'configure' merges the given text proto into the camera's current config.
'blobs' prints a summary of the frame timestamps it received: their spacing
and their distance from the frame grid of the camera's frame rate.
*/

use std::sync::Arc;
use std::time::{Duration, Instant};

use common::errors::*;
use mocap_proto::mocap::*;
use protobuf_json::*;

#[derive(Args)]
struct Args {
    addr: String,

    command: String,

    #[arg(default = "")]
    config: String,

    #[arg(default = 600)]
    count: usize,

    #[arg(default = "frame.jpg")]
    out: String,
}

async fn status(stub: &CameraStub) -> Result<MocapCameraStatus> {
    let ctx = rpc::ClientRequestContext::default();
    executor::timeout(Duration::from_secs(5), stub.Status(&ctx, &StatusRequest::default()))
        .await
        .map_err(|_| err_msg("No Status answer within 5 s"))?
        .result
}

fn frame_interval_ns(frame_rate: u32) -> u64 {
    1_000_000_000 / (frame_rate.max(1) as u64)
}

#[executor_main]
async fn main() -> Result<()> {
    let args = common::args::parse_args::<Args>()?;

    let channel = Arc::new(rpc::Http2Channel::create(format!("http://{}", args.addr).as_str()).await?);
    let stub = CameraStub::new(channel);
    let ctx = rpc::ClientRequestContext::default();

    match args.command.as_str() {
        "status" => {
            let s = status(&stub).await?;
            println!("{}", s.serialize_json(&SerializerOptions::default())?);
        }
        "configure" => {
            let mut config = status(&stub).await?.config().clone();
            protobuf::text::parse_text_proto(&args.config, &mut config)?;
            stub.Configure(&ctx, &config).await.result?;

            let s = status(&stub).await?;
            println!("{}", s.config().serialize_json(&SerializerOptions::default())?);
        }
        "blobs" => {
            let frame_rate = status(&stub).await?.config().frame_rate();
            let interval = frame_interval_ns(frame_rate);

            let mut res_stream = stub.ReadBlobs(&ctx, &ReadBlobsRequest::default()).await;

            let start = Instant::now();
            let mut n = 0;
            let mut last_ts: Option<u64> = None;
            let mut gaps = std::collections::BTreeMap::<u64, usize>::new();
            let mut off_grid = 0;
            let mut blob_counts = std::collections::BTreeMap::<usize, usize>::new();

            while n < args.count {
                let res = match executor::timeout(Duration::from_secs(5), res_stream.recv()).await {
                    Ok(Some(v)) => v,
                    Ok(None) => break,
                    Err(_) => return Err(err_msg("No blobs within 5 s")),
                };

                let ts = res.frame_timestamp();
                // The grid is k * (1e9 / frame_rate) ns within each second.
                if (ts % 1_000_000_000) % interval != 0 {
                    off_grid += 1;
                }
                if let Some(last) = last_ts {
                    *gaps.entry((ts.saturating_sub(last) + interval / 2) / interval).or_default() += 1;
                }
                last_ts = Some(ts);

                let num_blobs = res.cameras().get(0).map(|c| c.results().blobs().len()).unwrap_or(0);
                *blob_counts.entry(num_blobs).or_default() += 1;

                if n < 3 || n % 120 == 0 {
                    print!("frame_timestamp {} blobs {}", ts, num_blobs);
                    if let Some(c) = res.cameras().get(0) {
                        for b in c.results().blobs().iter().take(3) {
                            print!("  ({:.1}, {:.1}) r {:.1}", b.x(), b.y(), b.radius_a());
                        }
                    }
                    println!();
                }

                n += 1;
            }

            let secs = start.elapsed().as_secs_f64();
            println!("{} responses in {:.2} s ({:.1}/s) at frame_rate {}", n, secs, (n as f64) / secs, frame_rate);
            println!("timestamps off the grid: {}", off_grid);
            println!("gaps in frame intervals (intervals: count): {:?}", gaps);
            println!("blobs per frame (blobs: count): {:?}", blob_counts);
        }
        "frame" => {
            let mut res_stream = stub.ReadFrames(&ctx, &ReadFramesRequest::default()).await;
            let res = executor::timeout(Duration::from_secs(10), res_stream.recv())
                .await
                .map_err(|_| err_msg("No frame within 10 s"))?
                .ok_or_else(|| err_msg("The frame stream ended"))?;

            std::fs::write(&args.out, res.mjpeg())?;
            println!("Wrote {} bytes to {}", res.mjpeg().len(), args.out);
        }
        other => {
            return Err(format_err!("Unknown command: {}", other));
        }
    }

    Ok(())
}
