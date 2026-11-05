use std::time::Duration;

use common::errors::*;
use executor::sync::AsyncMutex;
use executor::lock;
use mocap_proto::mocap::*;

/// Camera service for a board without a working sensor. It keeps the config
/// and answers Status, but never sends frames or blobs. Run next to the real
/// TimeSyncNode it lets the board take part in time sync, e.g. to test PTP
/// leader failover with the manager.
pub struct NoSensorMocapCamera {
    hardware_config: CameraHardwareConfig,
    config: AsyncMutex<MocapCameraConfigureRequest>,
}

impl NoSensorMocapCamera {
    pub fn new(hardware_config: CameraHardwareConfig) -> Self {
        Self {
            hardware_config,
            config: AsyncMutex::new(MocapCameraConfigureRequest::default()),
        }
    }
}

#[async_trait]
impl CameraService for NoSensorMocapCamera {
    async fn Status(
        &self,
        request: rpc::ServerRequest<StatusRequest>,
        response: &mut rpc::ServerResponse<MocapCameraStatus>
    ) -> Result<()> {
        response.set_hardware_config(self.hardware_config.clone());
        response.sensor_mut().set_model_name("none");
        lock!(config <= self.config.lock().await?, {
            response.set_config(config.clone());
        });
        Ok(())
    }

    async fn Configure(
        &self,
        request: rpc::ServerRequest<MocapCameraConfigureRequest>,
        response: &mut rpc::ServerResponse<ConfigureResponse>
    ) -> Result<()> {
        lock!(config <= self.config.lock().await?, {
            *config = request.value.clone();
        });
        Ok(())
    }

    async fn ReadBlobs(
        &self,
        request: rpc::ServerRequest<ReadBlobsRequest>,
        response: &mut rpc::ServerStreamResponse<ReadBlobsResponse>
    ) -> Result<()> {
        response.send_head().await?;
        loop {
            executor::sleep(Duration::from_secs(3600)).await?;
        }
    }

    async fn ReadFrames(
        &self,
        request: rpc::ServerRequest<ReadFramesRequest>,
        response: &mut rpc::ServerStreamResponse<ReadFramesResponse>
    ) -> Result<()> {
        Err(err_msg("This camera has no sensor"))
    }

    async fn FlashMCU(
        &self,
        request: rpc::ServerRequest<FlashMCURequest>,
        response: &mut rpc::ServerResponse<FlashMCUResponse>
    ) -> Result<()> {
        Err(err_msg("This camera has no MCU"))
    }
}
