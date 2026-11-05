use std::collections::VecDeque;
use std::sync::Arc;
use std::time::{Duration, Instant};

use common::bytes::Bytes;
use common::errors::*;
use executor::channel;
use executor::sync::{AsyncMutex, SyncMutex};
use executor::lock;
use executor_multitask::{impl_resource_passthrough, ServiceResource, ServiceResourceGroup, BroadcastChannel};
use media_camera::v4l2::capture_buffer::CaptureDMABuffer;
use mocap_proto::mocap::*;
use mocap_camera_core::*;
use ptp::MonotonicClockTimeSyncer;

use crate::inst::CameraSensorData;
use crate::timestamp::*;

/// At least the number of processing threads + 2, so the driver always has a
/// buffer to fill while the others are being processed.
const NUM_FRAME_BUFFERS: usize = 5;

const NUM_BLOB_PROCESSING_THREADS: usize = 2;

const NUM_MJPEG_THREADS: usize = 1;

/// How the camera's trigger relates frames to the PTP frame grid.
pub trait FrameTiming: Send + Sync + 'static {
    /// Time from the trigger edge to the frame start timestamp.
    fn trigger_to_frame_start(&self, config: &MocapCameraConfigureRequest) -> Duration;

    /// Whether the trigger edges are on the frame grid of this config's frame
    /// rate now, so that frames can be given grid timestamps.
    fn frames_on_grid(&self, config: &MocapCameraConfigureRequest) -> bool;
}

/// Capture for camera drivers that send no FRAME_SYNC events and have no
/// partial cache sync (rkcif on the RV1126B). Unlike
/// MocapCameraCaptureProcessor, each frame is processed after it is fully
/// dequeued, which adds up to one frame transfer time of latency:
///
/// - The capture thread dequeues a buffer and hands it to a free blob
///   processing thread, or queues it again at once if none is free.
/// - The blob processing thread invalidates the CPU cache for the whole
///   buffer, finds the blobs and publishes them in frame order.
/// - The buffer then goes to the MJPEG thread if a stream wants a frame, and
///   back to the driver.
///
/// Blobs are only published while the trigger is on the frame grid: the
/// frame timestamp is the grid point nearest to the trigger time worked out
/// from the frame start.
pub struct DequeueCaptureProcessor {
    resources: ServiceResourceGroup,
    shared: Arc<Shared>,
}

impl_resource_passthrough!(DequeueCaptureProcessor, resources);

struct Shared {
    config: Arc<AsyncMutex<Arc<MocapCameraConfigureRequest>>>,
    camera_sensor: CameraSensorData,
    mono_time_sync: Arc<MonotonicClockTimeSyncer>,
    timing: Arc<dyn FrameTiming>,
    blob_subscribers: BroadcastChannel<ReadBlobsResponse>,
    mjpeg_subscribers: BroadcastChannel<Bytes>,
    capture_stream: Arc<v4l2::Stream<v4l2::DMABuffer<CaptureDMABuffer>>>,
    state: SyncMutex<State>,
}

#[derive(Default)]
struct State {
    /// Frames being processed by the blob threads, oldest first.
    blob_processing_queue: VecDeque<u64>,

    mjpeg_busy: usize,

    stats: Stats,
}

#[derive(Default, Clone)]
struct Stats {
    frames: u64,
    corrupted: u64,
    /// No blob thread was free.
    dropped: u64,
    /// Finished after a later frame.
    out_of_order: u64,
    published: u64,
    off_grid: u64,
}

struct Frame {
    sequence: u64,

    /// CLOCK_MONOTONIC time of the frame start.
    sof_time: Duration,

    buf: v4l2::DMABuffer<CaptureDMABuffer>,
}

impl DequeueCaptureProcessor {
    pub async fn create(
        config: Arc<AsyncMutex<Arc<MocapCameraConfigureRequest>>>,
        camera_sensor: CameraSensorData,
        capture_stream: v4l2::UnconfiguredStream,
        mono_time_sync: Arc<MonotonicClockTimeSyncer>,
        timing: Arc<dyn FrameTiming>,
        dma_heap: &str,
    ) -> Result<Self> {
        let (capture_stream, capture_buffers) = capture_stream.configure_dma(NUM_FRAME_BUFFERS).await?;
        let capture_stream = Arc::new(capture_stream);

        let dma_buffers = CaptureDMABuffer::allocate_from_heap(
            dma_heap,
            (camera_sensor.width as usize) * (camera_sensor.height as usize),
            capture_buffers.len(),
        )?;

        let shared = Arc::new(Shared {
            config,
            camera_sensor,
            mono_time_sync,
            timing,
            blob_subscribers: BroadcastChannel::default(),
            mjpeg_subscribers: BroadcastChannel::default(),
            capture_stream,
            state: SyncMutex::default(),
        });

        let resources = ServiceResourceGroup::new("DequeueCaptureProcessor");

        let (processing_sender, processing_receiver) = channel::unbounded();
        let (mjpeg_sender, mjpeg_receiver) = channel::unbounded();

        let mut buffers = vec![];
        for (mut capture_buf, dma_buffer) in capture_buffers.into_iter().zip(dma_buffers.into_iter()) {
            capture_buf.set_data(dma_buffer);
            buffers.push(capture_buf);
        }

        resources.spawn_interruptable(
            "DequeueCaptureProcessor::capture_thread",
            Self::capture_thread(shared.clone(), buffers, processing_sender)
        ).await;

        for i in 0..NUM_BLOB_PROCESSING_THREADS {
            resources.spawn_interruptable(
                &format!("DequeueCaptureProcessor::blob_processing_thread[{}]", i),
                Self::blob_processing_thread(shared.clone(), processing_receiver.clone(), mjpeg_sender.clone())
            ).await;
        }

        for i in 0..NUM_MJPEG_THREADS {
            resources.spawn_interruptable(
                &format!("DequeueCaptureProcessor::mjpeg_processing_thread[{}]", i),
                Self::mjpeg_processing_thread(shared.clone(), mjpeg_receiver.clone())
            ).await;
        }

        resources.spawn_interruptable(
            "DequeueCaptureProcessor::stats_thread",
            Self::stats_thread(shared.clone())
        ).await;

        Ok(Self { resources, shared })
    }

    pub fn blob_subscribers(&self) -> &BroadcastChannel<ReadBlobsResponse> {
        &self.shared.blob_subscribers
    }

    pub fn mjpeg_subscribers(&self) -> &BroadcastChannel<Bytes> {
        &self.shared.mjpeg_subscribers
    }

    async fn capture_thread(
        shared: Arc<Shared>,
        buffers: Vec<v4l2::DMABuffer<CaptureDMABuffer>>,
        processing_sender: channel::Sender<Frame>,
    ) -> Result<()> {
        for buf in buffers {
            shared.capture_stream.enqueue_buffer(buf).await?;
        }

        shared.capture_stream.turn_on().await?;

        let mut sequence = 0;
        loop {
            let (buf, corrupted) = shared.capture_stream.dequeue_buffer_allow_errors().await?;

            if corrupted {
                shared.state.apply(|s| s.stats.corrupted += 1)?;
                shared.capture_stream.enqueue_buffer(buf).await?;
                continue;
            }

            // rkcif marks the timestamp as MONOTONIC but takes it from
            // CLOCK_BOOTTIME on the RV1126B. The two differ by the time spent
            // in suspend.
            let stamp = buf.monotonic_timestamp()
                .ok_or_else(|| err_msg("The driver gave a frame without a monotonic timestamp"))?;
            let boot_now: Duration = sys::ClockId::BOOTTIME.get_time()?.into();
            let mono_now: Duration = sys::ClockId::MONOTONIC.get_time()?.into();
            let sof_time = stamp.saturating_sub(boot_now.saturating_sub(mono_now));

            let free = shared.state.apply(|s| {
                s.stats.frames += 1;
                if s.blob_processing_queue.len() < NUM_BLOB_PROCESSING_THREADS {
                    s.blob_processing_queue.push_back(sequence);
                    true
                } else {
                    s.stats.dropped += 1;
                    false
                }
            })?;

            if free {
                if processing_sender.try_send(Frame { sequence, sof_time, buf }).is_err() {
                    return Err(err_msg("The blob processing threads have stopped"));
                }
            } else {
                shared.capture_stream.enqueue_buffer(buf).await?;
            }

            sequence += 1;
        }
    }

    async fn blob_processing_thread(
        shared: Arc<Shared>,
        receiver: channel::Receiver<Frame>,
        mjpeg_sender: channel::Sender<v4l2::DMABuffer<CaptureDMABuffer>>,
    ) -> Result<()> {
        let width = shared.camera_sensor.width as usize;
        let height = shared.camera_sensor.height as usize;
        let mut frame_processor = FrameProcessor::new(width, height);

        loop {
            let frame = receiver.recv().await?;

            let config: Arc<MocapCameraConfigureRequest> = lock!(config <= shared.config.lock().await?, {
                config.clone()
            });

            let on_grid = config.frame_rate() > 0 && shared.timing.frames_on_grid(&config);

            let mut res = None;
            if on_grid && shared.blob_subscribers.active() {
                let sof_time_ptp = shared.mono_time_sync.monotonic_to_ptp_time(frame.sof_time);
                let trigger_time_ptp = sof_time_ptp.saturating_sub(shared.timing.trigger_to_frame_start(&config));
                let frame_timestamp = rounded_frame_timestamp(trigger_time_ptp, config.frame_rate());

                let data = frame.buf.data().unwrap().read()?;

                frame_processor.reset();
                frame_processor.process_lines(&data[..(width * height)], config.pixel_threshold() as u8);
                let results = frame_processor.finish(config.blob_filter());

                let r = res.insert(ReadBlobsResponse::default());
                r.set_frame_timestamp(frame_timestamp.as_nanos() as u64);
                r.new_cameras().set_results(results);
            }

            let to_mjpeg = shared.state.apply(|s| {
                if s.blob_processing_queue.front() == Some(&frame.sequence) {
                    if let Some(res) = res {
                        shared.blob_subscribers.send(res);
                        s.stats.published += 1;
                    }
                } else {
                    s.stats.out_of_order += 1;
                }

                if !on_grid {
                    s.stats.off_grid += 1;
                }

                s.blob_processing_queue.retain(|v| *v != frame.sequence);

                if shared.mjpeg_subscribers.active() && s.mjpeg_busy < NUM_MJPEG_THREADS {
                    s.mjpeg_busy += 1;
                    true
                } else {
                    false
                }
            })?;

            if to_mjpeg {
                if mjpeg_sender.try_send(frame.buf).is_err() {
                    return Err(err_msg("The MJPEG thread has stopped"));
                }
            } else {
                shared.capture_stream.enqueue_buffer(frame.buf).await?;
            }
        }
    }

    async fn mjpeg_processing_thread(
        shared: Arc<Shared>,
        receiver: channel::Receiver<v4l2::DMABuffer<CaptureDMABuffer>>,
    ) -> Result<()> {
        let mut encoder = MJPEGEncoder::new();
        let mut last_mjpeg_frame: Option<Instant> = None;

        loop {
            let buf = receiver.recv().await?;

            let config: Arc<MocapCameraConfigureRequest> = lock!(config <= shared.config.lock().await?, {
                config.clone()
            });

            let now = Instant::now();
            let interval = Duration::from_secs_f32(1.0 / (config.mjpeg().max_fps().max(1) as f32));
            let due = last_mjpeg_frame.map(|t| now - t >= interval).unwrap_or(true);

            let mut output = None;
            if due {
                last_mjpeg_frame = Some(now);

                let raw_data = buf.data().unwrap().read()?;
                let image = image::ImageRef {
                    width: shared.camera_sensor.width as usize,
                    height: shared.camera_sensor.height as usize,
                    channels: 1,
                    data: raw_data,
                };

                let data = encoder.encode(image, config.pixel_threshold() as u8, config.mjpeg())?;
                output = Some(Bytes::from(data));
            }

            shared.capture_stream.enqueue_buffer(buf).await?;
            shared.state.apply(|s| s.mjpeg_busy -= 1)?;

            if let Some(data) = output {
                let _ = shared.mjpeg_subscribers.send(data);
            }
        }
    }

    async fn stats_thread(shared: Arc<Shared>) -> Result<()> {
        let mut last = Stats::default();
        loop {
            executor::sleep(Duration::from_secs(10)).await?;

            let stats = shared.state.apply(|s| s.stats.clone())?;
            if stats.frames != last.frames {
                println!(
                    "[Capture] 10 s: frames {} published {} off-grid {} dropped {} out-of-order {} corrupted {}",
                    stats.frames - last.frames,
                    stats.published - last.published,
                    stats.off_grid - last.off_grid,
                    stats.dropped - last.dropped,
                    stats.out_of_order - last.out_of_order,
                    stats.corrupted - last.corrupted,
                );
            }
            last = stats;
        }
    }
}
