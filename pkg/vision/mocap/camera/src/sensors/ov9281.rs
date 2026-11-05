/*
OV9281 on the RV1126B (camera-module ov9281 driver, rkcif), 1280x800 Y8 in
"frame sync" mode: the sensor streams with its own frame timer and each
rising edge on FSIN restarts the frame, with integration starting at the
edge. The rules below were measured on the board in rows (README of the
camera-module repository, "Frame sync rules"):

- The free-running frame (800 + VBLANK rows) must last at least one trigger
  period, or the sensor starts frames of its own between the edges.
- EXPOSURE <= period - 833 rows, or the next edge comes during the readout
  and the bottom rows are corrupt.
- The frame start (the rkcif buffer timestamp) comes EXPOSURE + about 14 rows
  after the edge.

A row is (width + HBLANK) / PIXEL_RATE: 1456 / 192 MHz = 7.583 us in Y8.
*/

use std::time::Duration;

use crate::ptp_trigger::MAX_PERIOD_CORRECTION;

/// The driver's private 'trigger_mode' control (V4L2_CID_USER_BASE + 0x1f80).
pub const OV9281_TRIGGER_MODE_CID: u32 = 0x0098_0900 + 0x1f80;

/// 'trigger_mode' value for frame sync.
pub const OV9281_TRIGGER_FRAME_SYNC: i32 = 2;

const HEIGHT_ROWS: f64 = 800.0;

/// The driver's minimum VBLANK.
const MIN_VBLANK: u32 = 110;

/// Free-running frames are made this many rows longer than the trigger period.
const VBLANK_MARGIN_ROWS: f64 = 15.0;

/// Rows from the end of integration to the end of the readout, plus the 19
/// rows before an edge that it already corrupts.
const READOUT_ROWS: f64 = 833.0;

/// The frame start comes EXPOSURE + this many rows after the trigger edge.
const FRAME_START_ROWS: f64 = 14.0;

/// The driver's minimum exposure.
const MIN_EXPOSURE_ROWS: u32 = 4;

/// Highest frame rate offered. The sensor free-runs at 144 fps in Y8, but at
/// 120 fps exposures up to 1.9 ms already fit.
const MAX_FRAME_RATE: u32 = 120;

#[derive(Clone, Debug)]
pub struct OV9281Driver {
    /// Length of one row in nanoseconds.
    row_ns: f64,
}

impl OV9281Driver {
    /// 'pixel_rate' and 'line_length' (width + HBLANK) are the driver's
    /// V4L2_CID_PIXEL_RATE and the line length in pixel-rate cycles.
    pub fn new(pixel_rate: f64, line_length: f64) -> Self {
        Self {
            row_ns: line_length / pixel_rate * 1e9,
        }
    }

    pub fn row_time(&self) -> Duration {
        Duration::from_nanos(self.row_ns.round() as u64)
    }

    pub fn max_frame_rate(&self) -> u32 {
        MAX_FRAME_RATE
    }

    /// Frame rate used for the sensor timing while the trigger is off.
    fn effective_frame_rate(frame_rate: u32) -> u32 {
        if frame_rate == 0 { MAX_FRAME_RATE } else { frame_rate }
    }

    fn period_rows(&self, frame_rate: u32) -> f64 {
        1e9 / (Self::effective_frame_rate(frame_rate) as f64) / self.row_ns
    }

    /// VBLANK that makes the free-running frame slightly longer than the period.
    pub fn vblank_for_frame_rate(&self, frame_rate: u32) -> u32 {
        let rows = (self.period_rows(frame_rate) + VBLANK_MARGIN_ROWS - HEIGHT_ROWS).ceil();
        (rows.max(0.0) as u32).max(MIN_VBLANK)
    }

    /// Longest exposure whose readout still ends before the next edge, also
    /// while the trigger servo shortens a period to pull in the phase.
    pub fn max_exposure_micros(&self, frame_rate: u32) -> u32 {
        let rows = (self.period_rows(frame_rate) * (1.0 - MAX_PERIOD_CORRECTION) - READOUT_ROWS).floor();
        let rows = rows.max(MIN_EXPOSURE_ROWS as f64);
        (rows * self.row_ns / 1000.0).floor() as u32
    }

    pub fn exposure_time_to_value(&self, exposure_micros: u32) -> u32 {
        let rows = ((exposure_micros as f64) * 1000.0 / self.row_ns).round() as u32;
        rows.max(MIN_EXPOSURE_ROWS)
    }

    /// Time from the trigger edge to the frame start timestamp.
    pub fn trigger_to_frame_start(&self, exposure_micros: u32) -> Duration {
        let rows = self.exposure_time_to_value(exposure_micros) as f64 + FRAME_START_ROWS;
        Duration::from_nanos((rows * self.row_ns).round() as u64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn y8() -> OV9281Driver {
        OV9281Driver::new(192e6, 1456.0)
    }

    #[test]
    fn y8_numbers_at_120_hz_match_the_measured_rules() {
        let d = y8();
        assert_eq!(d.row_time(), Duration::from_nanos(7583));

        // 8333333 ns / 7583.3 ns = 1098.9 rows; + 15 - 800, rounded up.
        assert_eq!(d.vblank_for_frame_rate(120), 314);

        // 1098.9 * 0.985 - 833 = 249 rows = 1888 us.
        assert_eq!(d.max_exposure_micros(120), 1888);
        assert_eq!(d.exposure_time_to_value(1888), 249);

        // The board run at exposure 264 rows saw the frame start 2114 us after
        // the edge (plus interrupt latency).
        let start = d.trigger_to_frame_start(2002);
        assert!(start > Duration::from_micros(2100) && start < Duration::from_micros(2114), "{:?}", start);
    }

    #[test]
    fn slower_rates_allow_longer_exposures_and_frames() {
        let d = y8();
        assert!(d.vblank_for_frame_rate(60) > d.vblank_for_frame_rate(120));
        assert!(d.max_exposure_micros(60) > 9000);

        // The trigger off uses the 120 fps timing.
        assert_eq!(d.vblank_for_frame_rate(0), d.vblank_for_frame_rate(120));

        // Never below the driver's limits.
        assert_eq!(d.exposure_time_to_value(1), MIN_EXPOSURE_ROWS);
        assert_eq!(OV9281Driver::new(192e6, 1456.0).vblank_for_frame_rate(1000), MIN_VBLANK);
    }
}
