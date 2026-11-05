//! Frame trigger made by a SoC PWM and locked to the PTP clock, for boards with
//! no PPS divider MCU (the RV1126B).
//!
//! The PWM output drives the sensor's trigger input and is also wired to the
//! PTP clock's timestamp input (EXTTS), so the clock records the time of every
//! rising edge. A PI servo compares each edge with the frame grid that
//! rounded_frame_timestamp() uses (frames at whole multiples of 1/frame_rate
//! within each PTP second, the same on every camera) and sets the length of
//! the next PWM period. The Rockchip PWM v4 takes a new period at the next
//! period boundary without a glitch, so a period written after edge n ends at
//! edge n + 2.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use common::errors::*;

use crate::timestamp::rounded_frame_timestamp;

/// Width of each trigger pulse. The OV9281 frame sync starts on the rising
/// edge, so only the edge matters.
const PULSE_WIDTH_NS: u64 = 500_000;

/// The most the servo moves one period away from nominal, as a fraction of
/// it, while it pulls in the phase (125 us at 120 Hz, so a 4 ms phase error
/// takes about 33 frames). A shorter period also leaves less time for
/// exposure and readout, so the sensor's exposure limit keeps this margin.
pub const MAX_PERIOD_CORRECTION: f64 = 0.015;

/// Proportional gain on the phase error, per period. With the one-period
/// delay, 0.25 is the critically damped value.
const KP: f64 = 0.25;

/// Integral gain: the part of the phase error added to the frequency term
/// each period. It takes out the rate difference between the PWM's clock and
/// the PTP clock.
const KI: f64 = 0.02;

/// Limit of the frequency term (crystal tolerance plus the PTP correction).
const MAX_FREQ: f64 = 1e-3;

/// The trigger counts as locked after this many edges in a row within
/// LOCK_ERROR_NS of the grid, and as unlocked at the first edge further than
/// UNLOCK_ERROR_NS away.
const LOCK_EDGES: u32 = 32;
const LOCK_ERROR_NS: i64 = 2_000;
const UNLOCK_ERROR_NS: i64 = 20_000;

/// Time from the PTP clock's grid to an edge, in nanoseconds; positive when
/// the edge is late.
pub fn phase_error_ns(edge: Duration, frame_rate: u32) -> i64 {
    let grid = rounded_frame_timestamp(edge, frame_rate);
    (edge.as_nanos() as i64) - (grid.as_nanos() as i64)
}

/// The PI servo, without any I/O.
pub struct TriggerServo {
    frame_rate: u32,

    /// Fractional correction of the nominal period that makes the PWM run at
    /// the PTP clock's rate. Kept across frame rate changes since it belongs
    /// to the clocks.
    freq: f64,

    good_edges: u32,
    locked: bool,
    last_edge: Option<Duration>,
    last_error_ns: i64,
    edges: u64,
    missed_edges: u64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoStep {
    /// Length to program for the next PWM period.
    pub period_ns: u64,

    pub error_ns: i64,
}

impl TriggerServo {
    pub fn new(frame_rate: u32) -> Self {
        Self {
            frame_rate,
            freq: 0.0,
            good_edges: 0,
            locked: false,
            last_edge: None,
            last_error_ns: 0,
            edges: 0,
            missed_edges: 0,
        }
    }

    pub fn frame_rate(&self) -> u32 {
        self.frame_rate
    }

    pub fn set_frame_rate(&mut self, frame_rate: u32) {
        if frame_rate == self.frame_rate {
            return;
        }

        self.frame_rate = frame_rate;
        self.good_edges = 0;
        self.locked = false;
        self.last_edge = None;
    }

    pub fn nominal_period_ns(&self) -> f64 {
        1e9 / (self.frame_rate.max(1) as f64)
    }

    pub fn locked(&self) -> bool {
        self.locked
    }

    pub fn last_error_ns(&self) -> i64 {
        self.last_error_ns
    }

    pub fn edges(&self) -> u64 {
        self.edges
    }

    /// Edges that never arrived, judged from the gaps between the ones that did.
    pub fn missed_edges(&self) -> u64 {
        self.missed_edges
    }

    pub fn freq(&self) -> f64 {
        self.freq
    }

    /// Takes the PTP clock time of a rising edge and returns the period to
    /// program next. Must not be called while the frame rate is 0.
    pub fn update(&mut self, edge: Duration) -> ServoStep {
        assert!(self.frame_rate > 0);
        let period = self.nominal_period_ns();

        if let Some(last) = self.last_edge {
            // A clock step backwards gives a gap of 0, which counts nothing.
            let gaps = (edge.saturating_sub(last).as_nanos() as f64 / period).round();
            if gaps > 1.0 {
                self.missed_edges += (gaps - 1.0) as u64;
            }
        }
        self.last_edge = Some(edge);
        self.edges += 1;

        let error_ns = phase_error_ns(edge, self.frame_rate);
        self.last_error_ns = error_ns;

        if error_ns.abs() > UNLOCK_ERROR_NS {
            self.locked = false;
            self.good_edges = 0;
        } else if error_ns.abs() <= LOCK_ERROR_NS {
            self.good_edges = self.good_edges.saturating_add(1);
            if self.good_edges >= LOCK_EDGES {
                self.locked = true;
            }
        } else {
            self.good_edges = 0;
        }

        let max_correction = MAX_PERIOD_CORRECTION * period;
        let wanted = period * self.freq - KP * (error_ns as f64);
        let correction = wanted.clamp(-max_correction, max_correction);

        // Anti-windup: while the output is limited the error says nothing about
        // the rate, so the frequency term holds.
        if correction == wanted {
            self.freq = (self.freq - KI * (error_ns as f64) / period).clamp(-MAX_FREQ, MAX_FREQ);
        }

        ServoStep {
            period_ns: (period + correction).round() as u64,
            error_ns,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct TriggerStatus {
    pub frame_rate: u32,
    pub locked: bool,

    /// Time of the last edge minus its grid point.
    pub last_error: Duration,
    pub last_error_negative: bool,

    pub edges: u64,
    pub missed_edges: u64,

    /// Fractional rate correction of the PWM (servo frequency term).
    pub freq: f64,

    /// The last I/O error of the trigger thread, if any.
    pub error: Option<String>,
}

impl TriggerStatus {
    /// last_error in seconds, signed.
    pub fn last_error_secs(&self) -> f64 {
        let v = self.last_error.as_secs_f64();
        if self.last_error_negative { -v } else { v }
    }
}

/// One sysfs PWM channel (/sys/class/pwm/pwmchipN/pwmM).
struct SysfsPwm {
    dir: PathBuf,
    period_ns: u64,
    duty_ns: u64,
}

impl SysfsPwm {
    fn open(chip: usize, channel: usize, period_ns: u64) -> Result<Self> {
        let chip_dir = PathBuf::from(format!("/sys/class/pwm/pwmchip{}", chip));
        let dir = chip_dir.join(format!("pwm{}", channel));

        if !dir.exists() {
            std::fs::write(chip_dir.join("export"), format!("{}\n", channel))
                .map_err(|e| format_err!("While exporting PWM {}:{}: {}", chip, channel, e))?;

            for _ in 0..100 {
                if dir.exists() {
                    break;
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        }

        let mut inst = Self { dir, period_ns: 0, duty_ns: u64::MAX };

        // Duty 0 first: it must never be more than the period.
        inst.set_duty(0)?;
        inst.write("period", period_ns)?;
        inst.period_ns = period_ns;
        if let Err(e) = std::fs::write(inst.dir.join("polarity"), "normal\n") {
            // Only allowed while disabled; the driver default is normal.
            eprintln!("[PtpPwmTrigger] Could not set the PWM polarity: {}", e);
        }
        std::fs::write(inst.dir.join("enable"), "1\n")?;

        Ok(inst)
    }

    fn write(&self, name: &str, value: u64) -> Result<()> {
        std::fs::write(self.dir.join(name), format!("{}\n", value))
            .map_err(|e| format_err!("While writing {} = {} to {:?}: {}", name, value, self.dir, e))?;
        Ok(())
    }

    fn set_period(&mut self, period_ns: u64) -> Result<()> {
        if period_ns != self.period_ns {
            self.write("period", period_ns)?;
            self.period_ns = period_ns;
        }
        Ok(())
    }

    fn set_duty(&mut self, duty_ns: u64) -> Result<()> {
        if duty_ns != self.duty_ns {
            self.write("duty_cycle", duty_ns)?;
            self.duty_ns = duty_ns;
        }
        Ok(())
    }
}

struct Shared {
    stop: AtomicBool,
    state: Mutex<State>,
}

#[derive(Default)]
struct State {
    requested_frame_rate: u32,
    status: TriggerStatus,
}

/// The trigger: a thread that runs the servo on the EXTTS events and writes
/// the PWM. A frame rate of 0 holds the output low.
pub struct PtpPwmTrigger {
    shared: Arc<Shared>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl PtpPwmTrigger {
    pub fn create(
        ptp_path: &str,
        extts_index: u32,
        pwm_chip: usize,
        pwm_channel: usize,
    ) -> Result<Self> {
        let pwm = SysfsPwm::open(pwm_chip, pwm_channel, 1_000_000_000 / 120)?;
        let extts = ptp::ExttsChannel::open(ptp_path, extts_index)?;

        let shared = Arc::new(Shared {
            stop: AtomicBool::new(false),
            state: Mutex::new(State::default()),
        });

        let thread = {
            let shared = shared.clone();
            std::thread::Builder::new()
                .name("ptp_trigger".into())
                .spawn(move || Self::run(shared, pwm, extts))?
        };

        Ok(Self {
            shared,
            thread: Some(thread),
        })
    }

    /// Sets the trigger rate. 0 stops the pulses.
    pub fn configure(&self, frame_rate: u32) {
        self.shared.state.lock().unwrap().requested_frame_rate = frame_rate;
    }

    pub fn status(&self) -> TriggerStatus {
        self.shared.state.lock().unwrap().status.clone()
    }

    fn run(shared: Arc<Shared>, mut pwm: SysfsPwm, mut extts: ptp::ExttsChannel) {
        let mut servo = TriggerServo::new(0);
        let mut current_rate = None;

        while !shared.stop.load(Ordering::Relaxed) {
            let rate = shared.state.lock().unwrap().requested_frame_rate;

            let res = Self::step(&mut servo, &mut current_rate, rate, &mut pwm, &mut extts);

            let mut state = shared.state.lock().unwrap();
            state.status = TriggerStatus {
                frame_rate: servo.frame_rate(),
                locked: servo.frame_rate() > 0 && servo.locked(),
                last_error: Duration::from_nanos(servo.last_error_ns().unsigned_abs()),
                last_error_negative: servo.last_error_ns() < 0,
                edges: servo.edges(),
                missed_edges: servo.missed_edges(),
                freq: servo.freq(),
                error: state.status.error.take(),
            };

            if let Err(e) = res {
                eprintln!("[PtpPwmTrigger] {}", e);
                state.status.error = Some(e.to_string());
                drop(state);
                std::thread::sleep(Duration::from_millis(100));
            }
        }

        if let Err(e) = pwm.set_duty(0) {
            eprintln!("[PtpPwmTrigger] While stopping the pulses: {}", e);
        }
    }

    fn step(
        servo: &mut TriggerServo,
        current_rate: &mut Option<u32>,
        rate: u32,
        pwm: &mut SysfsPwm,
        extts: &mut ptp::ExttsChannel,
    ) -> Result<()> {
        if *current_rate != Some(rate) {
            if rate == 0 {
                pwm.set_duty(0)?;
            } else {
                servo.set_frame_rate(rate);
                pwm.set_period(servo.nominal_period_ns().round() as u64)?;
                pwm.set_duty(PULSE_WIDTH_NS)?;
            }

            *current_rate = Some(rate);
        }

        let edges = extts.wait_events(Duration::from_millis(100))?;
        if rate == 0 {
            return Ok(());
        }

        let mut next = None;
        for edge in edges {
            next = Some(servo.update(edge));
        }

        if let Some(step) = next {
            pwm.set_period(step.period_ns)?;
        }

        Ok(())
    }
}

impl Drop for PtpPwmTrigger {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// PWM model: the programmed period is a whole number of 24 MHz counts, and
    /// those counts run 'rate_error' fast relative to the PTP clock. A period
    /// written after edge n is used from edge n + 1.
    struct Sim {
        servo: TriggerServo,
        edge_ns: f64,
        running_period_ns: f64,
        pending_period_ns: f64,
        rate_error: f64,
        jitter_ns: f64,
        seed: u64,
    }

    impl Sim {
        fn new(frame_rate: u32, start_ns: f64, rate_error: f64) -> Self {
            let servo = TriggerServo::new(frame_rate);
            let period = Self::pwm_period(servo.nominal_period_ns().round() as u64, rate_error);
            Self {
                servo,
                edge_ns: start_ns,
                running_period_ns: period,
                pending_period_ns: period,
                rate_error,
                jitter_ns: 10.0,
                seed: 1,
            }
        }

        fn pwm_period(programmed_ns: u64, rate_error: f64) -> f64 {
            let tick = 1e9 / 24e6;
            ((programmed_ns as f64) / tick).round() * tick / (1.0 + rate_error)
        }

        fn noise(&mut self) -> f64 {
            // xorshift, uniform in [-1, 1)
            self.seed ^= self.seed << 13;
            self.seed ^= self.seed >> 7;
            self.seed ^= self.seed << 17;
            ((self.seed % 2001) as f64 - 1000.0) / 1000.0
        }

        /// Runs one edge; returns its phase error as the servo measured it.
        fn edge(&mut self) -> i64 {
            self.edge_ns += self.running_period_ns;
            self.running_period_ns = self.pending_period_ns;

            let measured = (self.edge_ns + self.jitter_ns * self.noise()).round() as u64;
            let step = self.servo.update(Duration::from_nanos(measured));
            self.pending_period_ns = Self::pwm_period(step.period_ns, self.rate_error);
            step.error_ns
        }

        fn true_error_ns(&self) -> i64 {
            phase_error_ns(Duration::from_nanos(self.edge_ns.round() as u64), self.servo.frame_rate())
        }
    }

    // A whole second, small enough for f64 to hold nanoseconds exactly.
    const START: f64 = 1_000_000_000_000.0;

    #[test]
    fn phase_error_is_signed_distance_to_the_nearest_grid_point() {
        let s = Duration::from_secs(100);
        assert_eq!(phase_error_ns(s + Duration::from_nanos(8_333_333 + 500), 120), 500);
        assert_eq!(phase_error_ns(s + Duration::from_nanos(8_333_333 - 700), 120), -700);
        assert_eq!(phase_error_ns(s + Duration::from_nanos(3_000_000), 120), 3_000_000);
        assert_eq!(phase_error_ns(s + Duration::from_nanos(5_000_000), 120), 5_000_000 - 8_333_333);
        // Near the end of a second the grid point is 120 * 8333333 ns.
        assert_eq!(phase_error_ns(s + Duration::from_nanos(999_999_990), 120), 30);
    }

    #[test]
    fn locks_from_a_large_phase_error_and_holds_with_a_rate_offset() {
        // 4 ms off the grid (almost half a period) and a PWM clock 30 ppm fast.
        let mut sim = Sim::new(120, START + 4_000_000.0, 30e-6);

        let mut lock_edge = None;
        for i in 0..1200 {
            sim.edge();
            if lock_edge.is_none() && sim.servo.locked() {
                lock_edge = Some(i);
            }
        }

        // 4 ms at 125 us per frame is 32 frames, then 32 good edges to lock.
        let lock_edge = lock_edge.expect("never locked");
        assert!(lock_edge < 120, "locked only after {} edges", lock_edge);

        let mut worst = 0;
        for _ in 0..1200 {
            sim.edge();
            assert!(sim.servo.locked());
            worst = worst.max(sim.true_error_ns().abs());
        }
        // A 41.7 ns PWM step and 10 ns of timestamp jitter.
        assert!(worst < 150, "worst true error {} ns", worst);

        // The frequency term has taken out the rate difference.
        assert!((sim.servo.freq() - 30e-6).abs() < 3e-6, "freq {}", sim.servo.freq());
        assert_eq!(sim.servo.missed_edges(), 0);
    }

    #[test]
    fn never_moves_a_period_more_than_the_limit() {
        let mut sim = Sim::new(120, START + 4_000_000.0, 0.0);
        let nominal = sim.servo.nominal_period_ns();

        for _ in 0..200 {
            sim.edge();
            let programmed = sim.pending_period_ns;
            assert!(
                (programmed - nominal).abs() <= nominal * MAX_PERIOD_CORRECTION + 42.0,
                "period {} vs {}", programmed, nominal
            );
        }
    }

    #[test]
    fn a_clock_step_unlocks_and_it_locks_again() {
        let mut sim = Sim::new(120, START + 1_000_000.0, -20e-6);
        for _ in 0..600 {
            sim.edge();
        }
        assert!(sim.servo.locked());
        let freq = sim.servo.freq();

        // The PTP clock is stepped by 3 ms: the edges are suddenly off the grid.
        sim.edge_ns += 3_000_000.0;
        sim.edge();
        assert!(!sim.servo.locked());

        for _ in 0..240 {
            sim.edge();
        }
        assert!(sim.servo.locked());
        assert!(sim.true_error_ns().abs() < 150);
        // The frequency term held while the output was limited.
        assert!((sim.servo.freq() - freq).abs() < 2e-6, "{} vs {}", sim.servo.freq(), freq);
    }

    #[test]
    fn counts_missed_edges_from_the_gaps() {
        let mut servo = TriggerServo::new(100);
        let s = Duration::from_secs(50);
        servo.update(s);
        servo.update(s + Duration::from_millis(10));
        servo.update(s + Duration::from_millis(40)); // 2 missing
        assert_eq!(servo.missed_edges(), 2);
        assert_eq!(servo.edges(), 3);

        // A frame rate change forgets the last edge.
        servo.set_frame_rate(50);
        servo.update(s + Duration::from_millis(1000));
        assert_eq!(servo.missed_edges(), 2);
    }
}
