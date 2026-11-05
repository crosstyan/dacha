use std::sync::Arc;
use std::collections::VecDeque;
use std::time::{Instant, Duration};

use common::bytes::Bytes;
use common::errors::*;
#[cfg(feature = "rpi_pio")]
use pio_rp1::PIO;
use rpi::ws2812::WS2812SPIController;
use peripherals::pwm::*;
use peripherals::i2c::I2CHostController;
use executor::sync::AsyncMutex;
use executor::{lock, lock_async};
use executor::channel;
use executor::channel::oneshot;
use media_camera::rp1_direct::*;
use media_camera::mjpeg::*;
use executor_multitask::{impl_resource_passthrough, ServiceResource, ServiceResourceGroup, BroadcastChannel};
use mocap_proto::mocap::*;
use rpi::temp::CPUTemperatureReader;
use http::header::CONTENT_TYPE;
use ptp::MonotonicClockTimeSyncer;
use v4l2::Controllable;
use media_camera::v4l2::controls::*;
use media_camera::v4l2::capture_buffer::*;
use ptp::SignedDuration;
use rpi::model::Model;
use mocap_camera_core::*;
use math_proto_util::VectorProtoExt;

use crate::sensors::*;
use crate::pps_divider_client::*;
use crate::accelerometer::*;
use crate::timestamp::*;
use crate::strobe::*;
use crate::capture::*;
use crate::capture_dequeue::*;
use crate::ptp_trigger::*;

// TODO: Dedup this constant.
const STROBE_DIMMING_FREQUENCY: f32 = 10000.0;

// Currently the MCU models the entire strip as one color.
const NUM_RGB_LEDS: usize = 1;

/// Pixel rate of the camera-module ov9281 driver in Y8 (its PIXEL_RATE is a
/// 64-bit control, which VIDIOC_QUERYCTRL does not report).
const OV9281_Y8_PIXEL_RATE: f64 = 192e6;

/// What makes the frame trigger pulses.
enum FrameTrigger {
    /// The PPS divider MCU on the Raspberry Pi camera boards.
    PPSDivider(PPSDividerClient),

    /// A SoC PWM locked to the PTP clock (RV1126B).
    PtpPwm(Arc<PtpPwmTrigger>),
}

enum CaptureProcessor {
    /// Pipelined processing on FRAME_SYNC events (RP1).
    FrameSync(Arc<MocapCameraCaptureProcessor>),

    /// Processing after each frame is dequeued (rkcif).
    Dequeue(Arc<DequeueCaptureProcessor>),
}

impl CaptureProcessor {
    fn blob_subscribers(&self) -> &BroadcastChannel<ReadBlobsResponse> {
        match self {
            CaptureProcessor::FrameSync(p) => p.blob_subscribers(),
            CaptureProcessor::Dequeue(p) => p.blob_subscribers(),
        }
    }

    fn mjpeg_subscribers(&self) -> &BroadcastChannel<Bytes> {
        match self {
            CaptureProcessor::FrameSync(p) => p.mjpeg_subscribers(),
            CaptureProcessor::Dequeue(p) => p.mjpeg_subscribers(),
        }
    }
}

struct OV9281FrameTiming {
    driver: OV9281Driver,
    trigger: Arc<PtpPwmTrigger>,
}

impl FrameTiming for OV9281FrameTiming {
    fn trigger_to_frame_start(&self, config: &MocapCameraConfigureRequest) -> Duration {
        self.driver.trigger_to_frame_start(config.exposure_micros())
    }

    fn frames_on_grid(&self, config: &MocapCameraConfigureRequest) -> bool {
        let status = self.trigger.status();
        status.locked && status.frame_rate == config.frame_rate()
    }
}


pub struct MocapCamera {
    resources: ServiceResourceGroup,

    #[cfg(feature = "rpi_pio")]
    pio_forwarder: Option<PIO>,

    shared: Arc<Shared>,
}

impl_resource_passthrough!(MocapCamera, resources);

struct Shared {
    hardware_config: Arc<CameraHardwareConfigContainer>,

    camera_sensor: CameraSensorData,

    v4l2_controls: Vec<v4l2::ControlDefinition>,

    /// All controls that we directly expose to the user in the Configure RPC.
    user_controls: Vec<v4l2::ControlDefinition>,

    user_controls_prop: media_camera_proto::media::camera::Property,

    sensor: SensorDriver,

    trigger: FrameTrigger,

    /// This state is locked while we are reading the status of the camera.
    status_state: AsyncMutex<StatusState>,

    /// This state is locked while we are reconfiguring part of the sensor.
    configure_state: AsyncMutex<ConfigureState>,

    /// NOTE: This should never be locked for more than a short period of time to prevent blocking
    /// frame processing.
    config: Arc<AsyncMutex<Arc<MocapCameraConfigureRequest>>>,

    // state: AsyncMutex<State>,

    ptp_device: Arc<ptp::PTPDevice>,

    mono_time_sync: Arc<MonotonicClockTimeSyncer>,

    capture_processor: CaptureProcessor,
}

#[derive(Clone)]
pub(crate) struct CameraSensorData {
    pub model_name: String,

    pub width: u32,

    pub height: u32,
}


struct StatusState {
    cpu_temp: CPUTemperatureReader,
    accelerometer: Option<Box<dyn Accelerometer>>,
}

struct ConfigureState {
    strobe_dimming: Option<PWMChannel>,

    last_pps_config: PPSDividerConfigRequest,

    rgb_leds: Option<WS2812SPIController>,

    /// Use this for configuring controls.
    camera_subdev: v4l2::SubDevice,
}

struct State {
    config: Arc<MocapCameraConfigureRequest>,
}

impl MocapCamera {

    pub async fn create(
        hardware_config: Arc<CameraHardwareConfigContainer>,
        ptp_device: Arc<ptp::PTPDevice>
    ) -> Result<Self> {
        let trigger = match hardware_config.trigger_pwm() {
            Some((chip, channel)) => {
                // EXTTS channel 0 is the input the PWM is looped back to.
                FrameTrigger::PtpPwm(Arc::new(PtpPwmTrigger::create("/dev/ptp0", 0, chip, channel)?))
            }
            None => {
                // Note that will reset the MCU immediately so it will be back in sync with the default
                // config on the host side.
                FrameTrigger::PPSDivider(PPSDividerClient::create(hardware_config.clone()).await?)
            }
        };

        let mut strobe_dimming = None;
        if hardware_config.local_strobe_dimming() {
            strobe_dimming = Some(PWMChannel::open(0, 0).await?);
            strobe_dimming.as_mut().unwrap().write(STROBE_DIMMING_FREQUENCY, 0.0).await?;
        }

        #[cfg(feature = "rpi_pio")]
        let mut pio_forwarder = None;
        #[cfg(feature = "rpi_pio")]
        if hardware_config.enable_pio_trigger_forwarder() {
            pio_forwarder = Some(PIO::create_pin_forwarder(16, 22)?);
        }
        
        let mut rgb_leds = None;
        if hardware_config.local_rgb_control() {
            rgb_leds = Some(WS2812SPIController::create("/dev/spidev0.0")?);
        }

        let mut accelerometer = None;
        if hardware_config.has_accelerometer() {
            println!("Setup accelerometer...");

            let mut i2c_bus = I2CHostController::open(&hardware_config.accelerometer_i2c_device())?;
            accelerometer = Some(create_accelerometer(i2c_bus.device(0x19)).await?);

            println!("Accelerometer good");
        }

        // TODO: Have all key controls like exposure and analog/digital gains exported to a config file that we can re-initialize everything on boot.
        let mut cam = RP1DirectCamera::open_sensor(hardware_config.camera_sensor_name()).await?;

        let v4l2_controls = cam.camera_subdev.list_controls().await?;

        let reserved_controls = vec![
            v4l2::V4L2_CID_VBLANK,
            v4l2::V4L2_CID_EXPOSURE,
            v4l2::V4L2_CID_HFLIP,
            v4l2::V4L2_CID_VFLIP,
            v4l2::V4L2_CID_DIGITAL_GAIN,
            v4l2::V4L2_CID_TEST_PATTERN,
            v4l2::V4L2_CID_TEST_PATTERN_RED,
            v4l2::V4L2_CID_TEST_PATTERN_GREENR,
            v4l2::V4L2_CID_TEST_PATTERN_BLUE,
            v4l2::V4L2_CID_TEST_PATTERN_GREENB,
            // Set from the frame rate (OV9281 frame sync).
            OV9281_TRIGGER_MODE_CID,
        ];

        let user_controls = v4l2_controls.iter().cloned().filter(|control| {
            !reserved_controls.contains(&control.id()) &&
            !control.flags().contains(v4l2::ControlFlags::READ_ONLY)
        }).collect::<Vec<_>>();

        let sensor = match &trigger {
            FrameTrigger::PPSDivider(_) => SensorDriver::AR0234(AR0234Driver::default()),
            FrameTrigger::PtpPwm(_) => {
                let hblank = v4l2_controls.iter().find(|c| c.id() == v4l2::V4L2_CID_HBLANK)
                    .ok_or_else(|| err_msg("The sensor has no HBLANK control"))?;
                let hblank = cam.camera_subdev.get_control_value(hblank).await?;

                let driver = OV9281Driver::new(OV9281_Y8_PIXEL_RATE, (cam.width as f64) + (hblank as f64));
                println!("OV9281 row time: {:?}", driver.row_time());

                // Frame sync takes effect at stream on, so it is set before the
                // capture starts. Without trigger pulses the sensor free-runs.
                let trigger_mode = v4l2_controls.iter().find(|c| c.id() == OV9281_TRIGGER_MODE_CID)
                    .ok_or_else(|| err_msg("The sensor has no trigger_mode control"))?;
                cam.camera_subdev.set_control_value(trigger_mode, OV9281_TRIGGER_FRAME_SYNC).await?;

                SensorDriver::OV9281(driver)
            }
        };

        let resources = ServiceResourceGroup::new("MocapCamera");

        // TODO: Need to wait for this to be ready before doing anything.
        let mono_time_sync = Arc::new(MonotonicClockTimeSyncer::create(ptp_device.clone())?);
        resources.register_dependency(mono_time_sync.clone()).await;

        let mut user_controls_prop = media_camera_proto::media::camera::Property::default();

        let mut initial_config = MocapCameraConfigureRequest::default();
        initial_config.set_pixel_threshold(255u32);

        initial_config.set_blob_filter(FrameProcessor::default_blob_filter_config()?);
        initial_config.set_mjpeg(MJPEGEncoder::default_config()?);
        controls_to_proto(
            &user_controls, &cam.camera_subdev, "",
            &mut user_controls_prop, initial_config.camera_controls_mut()
        ).await?;
        Self::validate_config(&mut initial_config, &sensor)?;

        let config = Arc::new(AsyncMutex::new(Arc::new(initial_config)));

        let camera_sensor = CameraSensorData {
            model_name: cam.model_name,
            width: cam.width,
            height: cam.height,
        };

        let capture_processor = match (&trigger, &sensor) {
            (FrameTrigger::PtpPwm(trigger), SensorDriver::OV9281(driver)) => {
                let p = Arc::new(DequeueCaptureProcessor::create(
                    config.clone(),
                    camera_sensor.clone(),
                    cam.capture_stream,
                    mono_time_sync.clone(),
                    Arc::new(OV9281FrameTiming { driver: driver.clone(), trigger: trigger.clone() }),
                    hardware_config.frame_buffer_dma_heap(),
                ).await?);
                resources.register_dependency(p.clone()).await;
                CaptureProcessor::Dequeue(p)
            }
            _ => {
                let p = Arc::new(MocapCameraCaptureProcessor::create(
                    config.clone(),
                    camera_sensor.clone(),
                    cam.capture_device,
                    cam.capture_stream,
                    mono_time_sync.clone()
                ).await?);
                resources.register_dependency(p.clone()).await;
                CaptureProcessor::FrameSync(p)
            }
        };

        let shared = Arc::new(Shared {
            hardware_config,
            sensor,
            trigger,
            camera_sensor,
            v4l2_controls,
            user_controls,
            user_controls_prop,
            status_state: AsyncMutex::new(StatusState {
                cpu_temp: CPUTemperatureReader::create().await?,
                accelerometer,
            }),
            configure_state: AsyncMutex::new(ConfigureState {
                strobe_dimming,
                rgb_leds,
                // TODO: Always ensure it is initialized to this just in case we need to restart the server.
                last_pps_config: PPSDividerConfigRequest::default(),
                camera_subdev: cam.camera_subdev,
            }),
            config,
            ptp_device,
            capture_processor,
            mono_time_sync,
        });

        Ok(Self {
            resources,
            #[cfg(feature = "rpi_pio")]
            pio_forwarder,
            shared
        })
    }

    pub async fn configure(&self, config: &MocapCameraConfigureRequest) -> Result<()> {
        let mut config = config.clone();
        Self::validate_config(&mut config, &self.shared.sensor)?;

        if let FrameTrigger::PPSDivider(client) = &self.shared.trigger {
            if client.last_heartbeat().await?.is_none() {
                return Err(err_msg("PPS divider not yet healthy"));
            }
        }

        executor::spawn(Self::configure_inner(self.shared.clone(), config)).join().await
    }

    // NOT CANCEL SAFE
    async fn configure_inner(
        shared: Arc<Shared>,
        mut config: MocapCameraConfigureRequest
    ) -> Result<()> {
        // TODO: Have a concept of twhether or not the old config is dirty (must be re-synced)
        
        // TODO: Use a try_lock?
        lock_async!(configure_state <= shared.configure_state.lock().await?, {
            let old_config: Arc<MocapCameraConfigureRequest> = lock!(config <= shared.config.lock().await?, {
                config.clone()
            });

            if config.strobe_power() != old_config.strobe_power() {
                if let Some(strobe_dimming) = &mut configure_state.strobe_dimming {
                    // TODO: Have a timeout on this.
                    strobe_dimming.write(STROBE_DIMMING_FREQUENCY, config.strobe_power()).await?;
                }
            }

            // TODO: If using local SPI, re-run this every few seconds since it is flaky
            if config.rgb_led_colors() != old_config.rgb_led_colors() {
                if let Some(rgb_leds) = &mut configure_state.rgb_leds {
                    rgb_leds.write(config.rgb_led_colors())?;
                }
            }

            // 

            // The frame length must cover the trigger period, and it bounds the
            // exposure, so it goes first.
            if let SensorDriver::OV9281(driver) = &shared.sensor {
                let controls = configure_state.camera_subdev.list_controls().await?;
                let vblank = controls.iter().find(|c| c.id() == v4l2::V4L2_CID_VBLANK)
                    .ok_or_else(|| err_msg("Couldn't find the vblank control"))?;
                configure_state.camera_subdev.set_control_value(
                    vblank,
                    driver.vblank_for_frame_rate(config.frame_rate()) as i32
                ).await?;
            }

            // TODO: Exposure only needs to be controlled via v4l2 for some sensors.
            {
                let mut exposure_control = None;

                let controls = configure_state.camera_subdev.list_controls().await?;
                for control in controls {
                    if control.id() == v4l2::V4L2_CID_EXPOSURE {
                        exposure_control = Some(control);
                        break;
                    }
                }

                let exposure_control = exposure_control
                    .ok_or_else(|| err_msg("Couldn't find the exposure control"))?;

    
                let exposure_value = shared.sensor.exposure_time_to_value(config.exposure_micros());

                configure_state.camera_subdev.set_control_value(
                    &exposure_control,
                    exposure_value as i32
                ).await?;
            }

            let new_camera_states = set_controls_from_proto(
                &configure_state.camera_subdev,
                &shared.user_controls,
                "",
                old_config.camera_controls(),
                config.camera_controls()
            ).await?;
            config.set_camera_controls(new_camera_states);

            // Low values must desable the strobe entirely since we might accidentally enter
            // 'hybrid dimming' mode on the LED driver.
            let strobe_width = {
                if config.strobe_power() >= 0.02 {
                    Some(Duration::from_micros(config.exposure_micros() as u64))
                } else {
                    None
                }
            };

            /*

            frame_offset - exposure_

            After I send the trigger pulse, exposure ends after 8.1ms
            so exposure 
            */

            // TODO: Use better than microsecond precision for the exposure here.

            let pps_divider_client = match &shared.trigger {
                FrameTrigger::PPSDivider(client) => client,
                FrameTrigger::PtpPwm(trigger) => {
                    trigger.configure(config.frame_rate());

                    lock!(c <= shared.config.lock().await?, {
                        *c = Arc::new(config);
                    });

                    return Ok(());
                }
            };

            let driver = AR0234Driver::default();

            let pps_config = PPSDividerConfigRequest {
                unlock: false,
                pulse_rate: config.frame_rate() as u8,
                frame_width: Some(driver.frame_width()),
                // TODO: Based on the pulse_rate, maybe pick the negative of this.
                // frame_offset: Some(driver.frame_offset() - Duration::from_micros(config.exposure_micros() as u64)),
                frame_offset: Some(SignedDuration::from_micros(-8160 -126 + (config.exposure_micros() as i64))),

                strobe_width,
                strobe_offset: None,
                strobe_power: config.strobe_power(),

                rgb_color: config.rgb_led_colors()[0]
            };
            if pps_config != configure_state.last_pps_config {
                pps_divider_client.configure(&pps_config).await?;
                configure_state.last_pps_config = pps_config;
            }

            lock!(c <= shared.config.lock().await?, {
                *c = Arc::new(config);
            });

            Ok(())
        })
    }

    fn validate_config(config: &mut MocapCameraConfigureRequest, sensor: &SensorDriver) -> Result<()> {

        if config.frame_rate() == 0 {
            // config.set_strobe_power(0.0);
        } else {
            // NOTE: The LED driver we are using will turn off completely if the LEDs are off for more than 50ms. If the driver is fully off, it won't be enable to re-enable the LEDs quickly so we have to strobe frequently enough to overcome this (so the minimum rate is 20 FPS with some buffer.)

            if config.frame_rate() < 24 {
                config.set_frame_rate(24u32);
            }

            if config.frame_rate() > sensor.max_frame_rate() {
                config.set_frame_rate(sensor.max_frame_rate());
            }
        }

        {
            let v = config.exposure_micros();
            config.set_exposure_micros(
                v.min(sensor.max_exposure_micros(config.frame_rate())).max(sensor.min_exposure_micros())
            );
        }

        {
            let mut v = config.strobe_power();

            /*
            let mut v = StrobeSimulationModel::settings_r1().clamp_power_percent(&StrobeSettings {
                frequency: (config.frame_rate() as f32),
                pulse_width: (config.exposure_micros() as f32) / 1000000.0,
                power_percent: config.strobe_power(),
            }, 0.05);
            */

            if v < 0.05 {
                v = 0.0;
            }

            config.set_strobe_power(v);
        }

        {
            let v = config.pixel_threshold();
            config.set_pixel_threshold(v.min(255).max(1));
        }

        config.rgb_led_colors_mut().resize(NUM_RGB_LEDS, 0);
        

        Ok(())
    }

    pub async fn status(&self) -> Result<MocapCameraStatus> {
        let mut proto = MocapCameraStatus::default();

        proto.set_hardware_config((&**self.shared.hardware_config).clone());

        {
            let p = proto.sensor_mut();
            p.set_model_name(&self.shared.camera_sensor.model_name);
            p.set_frame_width(self.shared.camera_sensor.width);
            p.set_frame_height(self.shared.camera_sensor.height);
            p.set_max_frame_rate(self.shared.sensor.max_frame_rate());
            p.set_min_exposure_micros(self.shared.sensor.min_exposure_micros());
            p.set_max_exposure_micros(self.shared.sensor.max_exposure_micros(self.shared.sensor.max_frame_rate()));
        }

        proto.set_camera_controls(self.shared.user_controls_prop.clone());

        let pps_divider_client = match &self.shared.trigger {
            FrameTrigger::PPSDivider(client) => Some(client),
            FrameTrigger::PtpPwm(trigger) => {
                // The trigger's lock to the PTP clock in the MCU's fields:
                // prediction_error is the last edge's distance from the frame grid
                // and output_errors counts edges that never arrived.
                let status = trigger.status();
                let p = proto.pps_divider_telemetry_mut();
                p.set_locked(status.locked);
                p.set_prediction_error(status.last_error_secs());
                p.set_output_errors(status.missed_edges.min(u32::MAX as u64) as u32);
                None
            }
        };

        if let Some(client) = pps_divider_client {
            self.pps_divider_status(client, &mut proto).await?;
        }

        let shared = self.shared.clone();
        proto = executor::spawn(async move {
            lock_async!(state <= shared.status_state.lock().await?, {

                // TODO: All this needs a timeout.

                proto.set_cpu_temperature(state.cpu_temp.read().await? as f32);
                
                if let Some(accelerometer) = &mut state.accelerometer {
                    // TODO: Cache this.
                    let accel = accelerometer.read_acceleration().await?;

                    // TODO: Customize whether or not this needs to be flipped based on the camera model.
                    proto.accelerometer_mut().set_value(accel.to_proto());
                }

                Result::<_, Error>::Ok(proto)
            })
        }).join().await?;

        lock!(config <= self.shared.config.lock().await?, {
            proto.set_config(config.as_ref().clone());
        });

        Ok(proto)
    }

    async fn pps_divider_status(&self, client: &PPSDividerClient, proto: &mut MocapCameraStatus) -> Result<()> {
        if let Some((pkt, time)) = client.last_heartbeat().await? {
            let now = Instant::now();

            let p = proto.pps_divider_heartbeat_mut();
            p.set_mcu_temperature(pkt.temp_c);
            p.set_vcc_voltage_min(pkt.vcc_min_v);
            p.set_vcc_voltage_max(pkt.vcc_max_v);

            fn get_poe_voltage(v: f32) -> f32 {
                electronics::undivide_voltage(
                    v,
                    24.9,
                    1.0
                )
            }

            p.set_poe_voltage_min(get_poe_voltage(pkt.poe_min_v));
            p.set_poe_voltage_max(get_poe_voltage(pkt.poe_max_v));
        }

        if let Some((pkt, time)) = client.last_telemetry().await? {

            let p = proto.pps_divider_telemetry_mut();
            p.set_locked(client.is_locked().await?);

            let freq = 1.0 / ((pkt.frequency_mhz as f64) * 1_000_000.0);
            p.set_prediction_error((pkt.prediction_error as f64) * freq);
            p.set_pps_width((pkt.width as f64) * freq);
            p.set_output_errors(pkt.output_errors);
        }

        Ok(())
    }

    // TODO: Ensure that this has low prioritization over the network.
    pub fn live_stream(&self) -> http::Response {
        // Have the queue always overwrite any pending data in the queue if the queue is full.
        let body = MJPGCameraStreamBody::new(self.shared.capture_processor.mjpeg_subscribers().subscribe(1));

        http::ResponseBuilder::new()
            .status(http::status_code::OK)
            .header(CONTENT_TYPE, body.content_type())
            .body(Box::new(body))
            .build().unwrap()
    }



}

#[async_trait]
impl CameraService for MocapCamera {

    async fn Status(
        &self,
        request: rpc::ServerRequest<StatusRequest>,
        response: &mut rpc::ServerResponse<MocapCameraStatus>
    ) -> Result<()> {
        response.value = self.status().await?;
        Ok(())
    }

    async fn Configure(
        &self,
        request: rpc::ServerRequest<MocapCameraConfigureRequest>,
        response: &mut rpc::ServerResponse<ConfigureResponse>
    ) -> Result<()> {
        self.configure(&request.value).await?;
        Ok(())
    }

    // TODO: Ensure that this RPC is prioritized for network bandwidth relative to everything else
    // on the HTTP server (mainly need to ensure that all MJPEG carrying requests are low priority)
    async fn ReadBlobs(
        &self,
        request: rpc::ServerRequest<ReadBlobsRequest>,
        response: &mut rpc::ServerStreamResponse<ReadBlobsResponse>
    ) -> Result<()> {        
        // TODO: Need some logging if we ever drop frames
        let mut subscriber = self.shared.capture_processor.blob_subscribers().subscribe(1024);

        response.send_head().await?;

        let mut last_response = Instant::now();

        let mut min_interval = Duration::ZERO;
        if request.max_rate() != 0 {
            min_interval = Duration::from_secs_f32(1.0 / (request.max_rate() as f32));
        }

        loop {
            let res = subscriber.recv().await?;
            
            let now = Instant::now();
            if now - last_response >= min_interval {
                response.send(res).await?;
                last_response = now;
            }
        }

        Ok(())
    }

    async fn ReadFrames(
        &self,
        request: rpc::ServerRequest<ReadFramesRequest>,
        response: &mut rpc::ServerStreamResponse<ReadFramesResponse>
    ) -> Result<()> {
        let mut subscriber = self.shared.capture_processor.mjpeg_subscribers().subscribe(8);

        loop {
            let frame = subscriber.recv().await?;

            let mut res = ReadFramesResponse::default();
            // TODO: Make this zero copy
            res.set_mjpeg(&frame[..]);
            response.send(res).await?;
        }

        Ok(())
    }

    async fn FlashMCU(
        &self,
        request: rpc::ServerRequest<FlashMCURequest>,
        response: &mut rpc::ServerResponse<FlashMCUResponse>
    ) -> Result<()> {
        let shared = self.shared.clone();
        executor::spawn(async move {
            let client = match &shared.trigger {
                FrameTrigger::PPSDivider(client) => client,
                FrameTrigger::PtpPwm(_) => return Err(err_msg("This camera has no MCU")),
            };

            lock_async!(configure_state <= shared.configure_state.lock().await?, {
                client.flash(request.firmware()).await
            })
        }).join().await
    }
}


#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn validate_config() {
        let mut initial_config = MocapCameraConfigureRequest::default();
        MocapCamera::validate_config(&mut initial_config, &SensorDriver::AR0234(AR0234Driver::default())).unwrap();
    }

    #[test]
    fn validate_config_uses_the_ov9281_limits() {
        let sensor = SensorDriver::OV9281(OV9281Driver::new(192e6, 1456.0));

        let mut config = MocapCameraConfigureRequest::default();
        config.set_frame_rate(200u32);
        config.set_exposure_micros(5000u32);
        MocapCamera::validate_config(&mut config, &sensor).unwrap();
        assert_eq!(config.frame_rate(), 120);
        // At 120 fps the readout leaves 1.888 ms for exposure.
        assert_eq!(config.exposure_micros(), 1888);

        config.set_frame_rate(60u32);
        config.set_exposure_micros(5000u32);
        MocapCamera::validate_config(&mut config, &sensor).unwrap();
        assert_eq!(config.exposure_micros(), 5000);

        // The AR0234 keeps the upstream limits.
        let mut config = MocapCameraConfigureRequest::default();
        config.set_frame_rate(200u32);
        config.set_exposure_micros(5000u32);
        MocapCamera::validate_config(&mut config, &SensorDriver::AR0234(AR0234Driver::default())).unwrap();
        assert_eq!(config.frame_rate(), 119);
        assert_eq!(config.exposure_micros(), 5000);
    }

}

