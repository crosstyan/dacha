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

#[cfg(test)]
mod tests {
    use super::*;

    /// configure_inner() does not relock the trigger on an exposure change
    /// alone: the estimated trigger time (frame start minus the time from the
    /// edge) moves with the exposure, so every allowed change must stay below
    /// half a frame interval, with room for the other errors (under 50 us).
    #[test]
    fn an_exposure_change_moves_the_trigger_estimate_less_than_half_a_period() {
        let driver = OV9281Driver::new(192e6, 1456.0);
        let sensor = SensorDriver::OV9281(driver.clone());

        for frame_rate in 24..=sensor.max_frame_rate() {
            let shortest = driver.trigger_to_frame_start(sensor.min_exposure_micros());
            let longest = driver.trigger_to_frame_start(sensor.max_exposure_micros(frame_rate));
            let half_period = std::time::Duration::from_nanos(1_000_000_000 / (frame_rate as u64) / 2);

            assert!(
                longest - shortest + std::time::Duration::from_micros(100) < half_period,
                "{} fps: {:?} vs {:?}", frame_rate, longest - shortest, half_period
            );
        }
    }
}
