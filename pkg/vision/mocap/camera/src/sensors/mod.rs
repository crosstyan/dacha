mod ar0234;
mod ov9281;

pub use ar0234::*;
pub use ov9281::*;

/// Timing and limits of the sensor the camera runs.
pub enum SensorDriver {
    /// Raspberry Pi cameras, triggered by the PPS divider MCU. (Upstream uses
    /// the AR0234 timing for every sensor on that path.)
    AR0234(AR0234Driver),

    /// OV9281 in frame sync mode on the RV1126B, triggered by the PTP-locked PWM.
    OV9281(OV9281Driver),
}

impl SensorDriver {
    pub fn max_frame_rate(&self) -> u32 {
        match self {
            // Slightly less than the sensor's maximum so that a frame is never
            // missed while the previous one is still in progress.
            SensorDriver::AR0234(_) => 119,
            SensorDriver::OV9281(d) => d.max_frame_rate(),
        }
    }

    pub fn min_exposure_micros(&self) -> u32 {
        20
    }

    pub fn max_exposure_micros(&self, frame_rate: u32) -> u32 {
        match self {
            SensorDriver::AR0234(_) => 6000,
            SensorDriver::OV9281(d) => d.max_exposure_micros(frame_rate).min(6000),
        }
    }

    pub fn exposure_time_to_value(&self, exposure_micros: u32) -> u32 {
        match self {
            SensorDriver::AR0234(d) => d.exposure_time_to_value(exposure_micros),
            SensorDriver::OV9281(d) => d.exposure_time_to_value(exposure_micros),
        }
    }
}
