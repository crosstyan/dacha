#[macro_use]
extern crate common;
#[macro_use]
extern crate macros;

/*
The manager without the cluster services and the UI: it finds the cameras,
runs their time sync and merges their blobs, and serves mocap.Manager over
plain HTTP/2 RPC.

    MOCAP_CAMERA_IFACE=eno1 cargo run --bin mocap_manager_headless -- \
        --port=8000 --data_dir=/tmp/mocap_data_dir
*/

use std::sync::Arc;

use common::errors::*;
use executor_multitask::RootResource;
use file::{project_path, LocalPathBuf};
use mocap_manager::*;
use mocap_proto::mocap::*;

#[derive(Args)]
struct Args {
    #[arg(default = 8000)]
    port: u16,

    data_dir: LocalPathBuf,

    /// Text proto merged into pkg/vision/mocap/config/manager.txtpb.
    #[arg(default = "")]
    config: String,
}

#[executor_main]
async fn main() -> Result<()> {
    let args = common::args::parse_args::<Args>()?;

    let mut config = MocapManagerConfig::default();
    protobuf::text::parse_text_proto(
        &file::read_to_string(project_path!("pkg/vision/mocap/config/manager.txtpb")).await?,
        &mut config
    )?;
    protobuf::text::parse_text_proto(&args.config, &mut config)?;

    let service = RootResource::new();

    let manager = Arc::new(MocapManager::create(config, args.data_dir).await?);
    service.register_dependency(manager.clone()).await;

    let mut server = rpc::Http2Server::new(Some(args.port));
    server.add_service(manager.to_service())?;
    service.register_dependency(server.start()).await;

    println!("Manager RPC on port {}", args.port);
    service.wait().await
}
