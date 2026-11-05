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
//!
//! The lock fails closed: it is dropped when edges stop coming, when the
//! thread hits an error, when the camera asks for it (sensor timing changes),
//! and a status that is not refreshed counts as unlocked.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use common::errors::*;

use crate::timestamp::rounded_frame_timestamp;

/// Width of each trigger pulse. The OV9281 frame sync starts on the rising
/// edge, so only the edge matters.
const PULSE_WIDTH_NS: u64 = 500_000;

/// The most the servo moves one period away from nominal, as a fraction of
/// it, while it pulls in the phase (125 us at 120 Hz, so a 4 ms phase error
/// takes about 33 frames). The sensor timing leaves room for it: the
/// exposure limit for the shortest period, the frame length for the longest.
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

/// The frequency term only integrates errors this small. Larger ones are
/// phase still being pulled in (where the output may also be limited); they
/// would wind the term up to its limit and overshoot.
const INTEGRATE_ERROR_NS: i64 = 20_000;

/// The trigger counts as locked after this many edges in a row within
/// LOCK_ERROR_NS of the grid, and as unlocked at the first edge further than
/// UNLOCK_ERROR_NS away.
const LOCK_EDGES: u32 = 32;
const LOCK_ERROR_NS: i64 = 2_000;
const UNLOCK_ERROR_NS: i64 = 20_000;

/// How long the thread waits for edges. No edge in that time unlocks the
/// trigger: at 24 fps or more, at least two are due in it.
const EDGE_TIMEOUT: Duration = Duration::from_millis(100);

/// A status older than this means the trigger thread is stuck or gone.
const MAX_STATUS_AGE: Duration = Duration::from_millis(500);

/// How often the same trigger error is logged again.
const ERROR_LOG_INTERVAL: Duration = Duration::from_secs(10);

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
    /// the PTP clock's rate. Kept across frame rate changes and unlocks since
    /// it belongs to the clocks.
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

    /// A new rate, also 0 (stopped), drops the lock.
    pub fn set_frame_rate(&mut self, frame_rate: u32) {
        if frame_rate == self.frame_rate {
            return;
        }

        self.frame_rate = frame_rate;
        self.unlock();
    }

    /// Drops the lock and forgets the last edge; LOCK_EDGES good edges lock
    /// again.
    pub fn unlock(&mut self) {
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

        let error_ns = phase_error_ns(edge, self.frame_rate);
        self.last_error_ns = error_ns;
        self.edges += 1;

        // After a clock step the gap includes the step, so it says nothing
        // about missing edges.
        if let Some(last) = self.last_edge {
            if error_ns.abs() <= UNLOCK_ERROR_NS {
                // A clock step backwards gives a gap of 0, which counts nothing.
                let gaps = (edge.saturating_sub(last).as_nanos() as f64 / period).round();
                if gaps > 1.0 {
                    self.missed_edges += (gaps - 1.0) as u64;
                }
            }
        }
        self.last_edge = Some(edge);

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
        let correction = (period * self.freq - KP * (error_ns as f64))
            .clamp(-max_correction, max_correction);

        if error_ns.abs() <= INTEGRATE_ERROR_NS {
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
    pub last_error_ns: i64,

    pub edges: u64,
    pub missed_edges: u64,

    /// Fractional rate correction of the PWM (servo frequency term).
    pub freq: f64,

    /// The error of the trigger thread's last step, if it failed.
    pub error: Option<String>,

    /// When the trigger thread last wrote this status.
    pub updated: Option<Instant>,
}

impl TriggerStatus {
    pub fn last_error_secs(&self) -> f64 {
        (self.last_error_ns as f64) / 1e9
    }

    /// Whether the edges are on the grid of 'frame_rate' now: locked at that
    /// rate, with a status the thread wrote recently.
    pub fn on_grid(&self, frame_rate: u32, now: Instant) -> bool {
        let fresh = self.updated
            .map(|t| now.saturating_duration_since(t) < MAX_STATUS_AGE)
            .unwrap_or(false);

        frame_rate > 0 && self.frame_rate == frame_rate && self.locked && fresh
    }
}

/// The trigger output, as the trigger loop drives it.
pub trait TriggerOutput: Send {
    fn set_period(&mut self, period_ns: u64) -> Result<()>;

    fn set_duty(&mut self, duty_ns: u64) -> Result<()>;
}

/// Where the trigger loop gets the PTP clock times of the edges.
pub trait EdgeSource: Send {
    /// Edges seen within 'timeout', oldest first. Empty on a timeout.
    fn wait_events(&mut self, timeout: Duration) -> Result<Vec<Duration>>;
}

impl EdgeSource for ptp::ExttsChannel {
    fn wait_events(&mut self, timeout: Duration) -> Result<Vec<Duration>> {
        ptp::ExttsChannel::wait_events(self, timeout)
    }
}

/// One sysfs PWM channel (/sys/class/pwm/pwmchipN/pwmM).
struct SysfsPwm {
    dir: PathBuf,
    period_ns: u64,
    duty_ns: u64,
}

impl SysfsPwm {
    /// Finds the pwmchip whose controller is the platform device 'device' (e.g.
    /// "20720000.pwm"); the chip numbers follow the probe order.
    fn find_chip(device: &str) -> Result<PathBuf> {
        for entry in std::fs::read_dir("/sys/class/pwm")? {
            let chip_dir = entry?.path();
            let controller = match std::fs::canonicalize(chip_dir.join("device")) {
                Ok(path) => path,
                Err(_) => continue,
            };

            // rockchip-pwm-v4 registers each channel as a child device of its
            // controller (.../20720000.pwm/rockchip-pwm-v4.3).
            let matches = |p: &Path| p.file_name().map(|n| n == device).unwrap_or(false);
            if matches(&controller) || controller.parent().map(|p| matches(p)).unwrap_or(false) {
                return Ok(chip_dir);
            }
        }

        Err(format_err!("No PWM chip for device {}", device))
    }

    fn open(device: &str, channel: usize, period_ns: u64) -> Result<Self> {
        let chip_dir = Self::find_chip(device)?;
        let dir = chip_dir.join(format!("pwm{}", channel));

        if !dir.exists() {
            std::fs::write(chip_dir.join("export"), format!("{}\n", channel))
                .map_err(|e| format_err!("While exporting PWM {}:{}: {}", device, channel, e))?;

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
            // Some drivers only take it while disabled; the default is normal.
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
}

impl TriggerOutput for SysfsPwm {
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

/// The servo with its output and edge source: one step per wait for edges.
struct TriggerLoop<O: TriggerOutput, E: EdgeSource> {
    servo: TriggerServo,
    current_rate: Option<u32>,
    output: O,
    edge_source: E,
}

impl<O: TriggerOutput, E: EdgeSource> TriggerLoop<O, E> {
    fn new(output: O, edge_source: E) -> Self {
        Self {
            servo: TriggerServo::new(0),
            current_rate: None,
            output,
            edge_source,
        }
    }

    fn step(&mut self, rate: u32, unlock: bool) -> Result<()> {
        if unlock {
            self.servo.unlock();
        }

        if self.current_rate != Some(rate) {
            self.servo.set_frame_rate(rate);

            if rate == 0 {
                self.output.set_duty(0)?;
            } else {
                // The pulse width is far below any period at 24 fps or more,
                // so the duty never exceeds the period here.
                self.output.set_period(self.servo.nominal_period_ns().round() as u64)?;
                self.output.set_duty(PULSE_WIDTH_NS)?;
            }

            self.current_rate = Some(rate);
        }

        let edges = self.edge_source.wait_events(EDGE_TIMEOUT)?;
        if rate == 0 {
            return Ok(());
        }

        if edges.is_empty() {
            self.servo.unlock();
            return Ok(());
        }

        let mut next = None;
        for edge in edges {
            next = Some(self.servo.update(edge));
        }

        if let Some(step) = next {
            self.output.set_period(step.period_ns)?;
        }

        Ok(())
    }

    fn status(&self) -> TriggerStatus {
        TriggerStatus {
            frame_rate: self.servo.frame_rate(),
            locked: self.servo.frame_rate() > 0 && self.servo.locked(),
            last_error_ns: self.servo.last_error_ns(),
            edges: self.servo.edges(),
            missed_edges: self.servo.missed_edges(),
            freq: self.servo.freq(),
            error: None,
            updated: None,
        }
    }
}

struct Shared {
    stop: AtomicBool,
    state: Mutex<State>,
}

#[derive(Default)]
struct State {
    requested_frame_rate: u32,

    /// Set by restart_lock(), cleared once the loop has dropped the lock.
    unlock_requested: bool,

    status: TriggerStatus,
}

/// The trigger: a thread that runs the servo on the EXTTS events and writes
/// the PWM. A frame rate of 0 holds the output low.
pub struct PtpPwmTrigger {
    shared: Arc<Shared>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl PtpPwmTrigger {
    /// 'pwm_device' is the PWM controller's platform device name.
    pub fn create(
        ptp_path: &str,
        extts_index: u32,
        pwm_device: &str,
        pwm_channel: usize,
    ) -> Result<Self> {
        let pwm = SysfsPwm::open(pwm_device, pwm_channel, 1_000_000_000 / 120)?;
        let extts = ptp::ExttsChannel::open(ptp_path, extts_index)?;

        let shared = Arc::new(Shared {
            stop: AtomicBool::new(false),
            state: Mutex::new(State::default()),
        });

        let thread = {
            let shared = shared.clone();
            let trigger_loop = TriggerLoop::new(pwm, extts);
            std::thread::Builder::new()
                .name("ptp_trigger".into())
                .spawn(move || Self::run(shared, trigger_loop))?
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

    /// Drops the lock now, e.g. before the sensor timing changes: the frames
    /// count as off the grid until LOCK_EDGES good edges after this call.
    pub fn restart_lock(&self) {
        let mut state = self.shared.state.lock().unwrap();
        state.unlock_requested = true;
        state.status.locked = false;
    }

    pub fn status(&self) -> TriggerStatus {
        self.shared.state.lock().unwrap().status.clone()
    }

    fn run<O: TriggerOutput, E: EdgeSource>(shared: Arc<Shared>, mut trigger_loop: TriggerLoop<O, E>) {
        let mut last_error_log: Option<Instant> = None;

        while !shared.stop.load(Ordering::Relaxed) {
            let (rate, unlock) = {
                let mut state = shared.state.lock().unwrap();
                let unlock = state.unlock_requested;
                state.unlock_requested = false;
                (state.requested_frame_rate, unlock)
            };

            let res = trigger_loop.step(rate, unlock);
            if res.is_err() {
                trigger_loop.servo.unlock();
            }

            let mut state = shared.state.lock().unwrap();
            let mut status = trigger_loop.status();
            // A restart_lock() during the step must not be undone by the lock
            // from before it.
            if state.unlock_requested {
                status.locked = false;
            }
            status.error = res.as_ref().err().map(|e| e.to_string());
            status.updated = Some(Instant::now());
            state.status = status;
            drop(state);

            if let Err(e) = res {
                let now = Instant::now();
                if last_error_log.map(|t| now - t >= ERROR_LOG_INTERVAL).unwrap_or(true) {
                    eprintln!("[PtpPwmTrigger] {}", e);
                    last_error_log = Some(now);
                }
                std::thread::sleep(Duration::from_millis(100));
            }
        }

        if let Err(e) = trigger_loop.output.set_duty(0) {
            eprintln!("[PtpPwmTrigger] While stopping the pulses: {}", e);
        }
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
    use std::collections::VecDeque;

    /// PWM model: the programmed period is a whole number of 24 MHz counts
    /// (pwm-rockchip-v4 rounds down), and those counts run 'rate_error' fast
    /// relative to the PTP clock. A period written after edge n is used from
    /// edge n + 1.
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
            ((programmed_ns as f64) / tick).floor() * tick / (1.0 + rate_error)
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

    /// The PWM rounds each period down to whole counts, half a count (20.8 ns)
    /// short on average; the frequency term makes that up too.
    fn floor_bias(period_ns: f64) -> f64 {
        (1e9 / 24e6) / 2.0 / period_ns
    }

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
    fn locks_from_a_large_phase_error_without_overshoot_and_holds_with_a_rate_offset() {
        // 4 ms off the grid (almost half a period) and a PWM clock 30 ppm fast.
        let mut sim = Sim::new(120, START + 4_000_000.0, 30e-6);

        let mut lock_edge = None;
        let mut near = false;
        for i in 0..1200 {
            sim.edge();
            // Once within the band, the error stays in it: no overshoot that
            // unlocks again.
            if near {
                assert!(sim.true_error_ns().abs() <= UNLOCK_ERROR_NS, "edge {}: {} ns", i, sim.true_error_ns());
            } else if sim.true_error_ns().abs() <= UNLOCK_ERROR_NS {
                near = true;
            }
            if lock_edge.is_none() && sim.servo.locked() {
                lock_edge = Some(i);
            }
        }

        // 4 ms at 125 us per frame is 32 frames, then 32 good edges to lock.
        let lock_edge = lock_edge.expect("never locked");
        assert!(lock_edge < 90, "locked only after {} edges", lock_edge);

        let mut worst = 0;
        for _ in 0..1200 {
            sim.edge();
            assert!(sim.servo.locked());
            worst = worst.max(sim.true_error_ns().abs());
        }
        // A 41.7 ns PWM step and 10 ns of timestamp jitter.
        assert!(worst < 150, "worst true error {} ns", worst);

        // The frequency term has taken out the rate difference; it dithers by
        // about a PWM count's worth (5 ppm) around that.
        let wanted = 30e-6 + floor_bias(sim.servo.nominal_period_ns());
        assert!((sim.servo.freq() - wanted).abs() < 3e-6, "freq {} vs {}", sim.servo.freq(), wanted);
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
    fn a_clock_step_unlocks_and_it_locks_again_without_winding_up() {
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

        // While the phase is pulled in the frequency term holds exactly.
        for _ in 0..16 {
            sim.edge();
            assert!(sim.true_error_ns().abs() > INTEGRATE_ERROR_NS);
            assert_eq!(sim.servo.freq(), freq);
        }

        for _ in 0..240 {
            sim.edge();
        }
        assert!(sim.servo.locked());
        assert!(sim.true_error_ns().abs() < 150);
        // Back where it was, within the dither (a wound-up term would be near
        // its 1e-3 limit).
        let wanted = -20e-6 + floor_bias(sim.servo.nominal_period_ns());
        assert!((sim.servo.freq() - wanted).abs() < 3e-6, "{} vs {}", sim.servo.freq(), wanted);
        assert!((freq - wanted).abs() < 3e-6, "{} vs {}", freq, wanted);
    }

    #[test]
    fn locking_takes_lock_edges_good_edges_in_a_row() {
        // Edge i at 100 fps from second 10, 'err' ns off the grid.
        let edge = |i: u64, err: i64| Duration::from_nanos((10_000_000_000 + (i as i64) * 10_000_000 + err) as u64);

        let mut servo = TriggerServo::new(100);
        for i in 0..(LOCK_EDGES as u64 - 1) {
            servo.update(edge(i, LOCK_ERROR_NS));
        }
        assert!(!servo.locked());
        servo.update(edge(LOCK_EDGES as u64 - 1, -LOCK_ERROR_NS));
        assert!(servo.locked());

        // An edge between the bands neither keeps the count nor unlocks.
        let mut servo = TriggerServo::new(100);
        for i in 0..20 {
            servo.update(edge(i, 0));
        }
        servo.update(edge(20, LOCK_ERROR_NS + 1));
        for i in 21..(20 + LOCK_EDGES as u64) {
            servo.update(edge(i, 0));
        }
        assert!(!servo.locked());
        servo.update(edge(20 + LOCK_EDGES as u64, 0));
        assert!(servo.locked());

        servo.update(edge(100, UNLOCK_ERROR_NS));
        assert!(servo.locked());
        servo.update(edge(101, UNLOCK_ERROR_NS + 1));
        assert!(!servo.locked());
    }

    #[test]
    fn counts_missed_edges_from_the_gaps_but_not_across_a_clock_step() {
        let mut servo = TriggerServo::new(100);
        let s = Duration::from_secs(50);
        servo.update(s);
        servo.update(s + Duration::from_millis(10));
        servo.update(s + Duration::from_millis(40)); // 2 missing
        assert_eq!(servo.missed_edges(), 2);
        assert_eq!(servo.edges(), 3);

        // The clock steps forward by 3 periods and 1 ms: the edge is off the
        // grid, and the gap is not counted.
        servo.update(s + Duration::from_millis(40 + 10 + 30 + 1));
        assert_eq!(servo.missed_edges(), 2);

        // A frame rate change forgets the last edge.
        servo.set_frame_rate(50);
        servo.update(s + Duration::from_millis(1000));
        assert_eq!(servo.missed_edges(), 2);
    }

    /// Records the PWM writes; the duty must never exceed the period.
    #[derive(Default)]
    struct FakeOutput {
        period_ns: u64,
        duty_ns: u64,
        writes: Arc<Mutex<Vec<(&'static str, u64)>>>,
    }

    impl TriggerOutput for FakeOutput {
        fn set_period(&mut self, period_ns: u64) -> Result<()> {
            assert!(self.duty_ns <= period_ns);
            self.period_ns = period_ns;
            self.writes.lock().unwrap().push(("period", period_ns));
            Ok(())
        }

        fn set_duty(&mut self, duty_ns: u64) -> Result<()> {
            assert!(self.period_ns == 0 || duty_ns <= self.period_ns);
            self.duty_ns = duty_ns;
            self.writes.lock().unwrap().push(("duty", duty_ns));
            Ok(())
        }
    }

    /// Hands out queued batches of edges; when they run out, nothing (a
    /// timeout), or, with 'endless_from' set, 4 grid edges per millisecond.
    struct FakeEdges {
        batches: Arc<Mutex<VecDeque<Result<Vec<Duration>>>>>,
        endless_from: Option<u64>,
    }

    impl EdgeSource for FakeEdges {
        fn wait_events(&mut self, _timeout: Duration) -> Result<Vec<Duration>> {
            if let Some(v) = self.batches.lock().unwrap().pop_front() {
                return v;
            }

            match &mut self.endless_from {
                Some(next) => {
                    std::thread::sleep(Duration::from_millis(1));
                    *next += 4;
                    Ok(grid_edges(*next - 4, 4))
                }
                None => Ok(vec![]),
            }
        }
    }

    fn fake_loop() -> (
        TriggerLoop<FakeOutput, FakeEdges>,
        Arc<Mutex<Vec<(&'static str, u64)>>>,
        Arc<Mutex<VecDeque<Result<Vec<Duration>>>>>,
    ) {
        let output = FakeOutput::default();
        let writes = output.writes.clone();
        let batches = Arc::new(Mutex::new(VecDeque::new()));
        let edges = FakeEdges { batches: batches.clone(), endless_from: None };
        (TriggerLoop::new(output, edges), writes, batches)
    }

    /// 'n' edges on the 120 fps grid, from frame 'first' of second 20.
    fn grid_edges(first: u64, n: u64) -> Vec<Duration> {
        (first..(first + n))
            .map(|i| Duration::from_secs(20) + Duration::from_nanos(i * 8_333_333))
            .collect()
    }

    #[test]
    fn a_stop_drops_the_lock_and_a_restart_at_the_same_rate_must_lock_again() {
        let (mut lp, writes, batches) = fake_loop();

        batches.lock().unwrap().push_back(Ok(grid_edges(0, 40)));
        lp.step(120, false).unwrap();
        assert!(lp.status().locked);
        assert_eq!(writes.lock().unwrap()[..2], [("period", 8_333_333), ("duty", PULSE_WIDTH_NS)]);

        // Stop: the pulses end and the lock is gone at once.
        lp.step(0, false).unwrap();
        assert!(!lp.status().locked);
        assert_eq!(lp.status().frame_rate, 0);
        assert_eq!(*writes.lock().unwrap().last().unwrap(), ("duty", 0));

        // Back at the same rate: not locked before LOCK_EDGES new good edges.
        writes.lock().unwrap().clear();
        batches.lock().unwrap().push_back(Ok(grid_edges(50, LOCK_EDGES as u64 - 1)));
        lp.step(120, false).unwrap();
        assert!(!lp.status().locked);
        assert_eq!(writes.lock().unwrap()[..2], [("period", 8_333_333), ("duty", PULSE_WIDTH_NS)]);
        assert_eq!(lp.status().missed_edges, 0);

        batches.lock().unwrap().push_back(Ok(grid_edges(50 + LOCK_EDGES as u64 - 1, 1)));
        lp.step(120, false).unwrap();
        assert!(lp.status().locked);
    }

    #[test]
    fn no_edges_an_error_or_a_restart_request_drop_the_lock() {
        let (mut lp, _writes, batches) = fake_loop();

        batches.lock().unwrap().push_back(Ok(grid_edges(0, 40)));
        lp.step(120, false).unwrap();
        assert!(lp.status().locked);

        // No edge within the wait.
        lp.step(120, false).unwrap();
        assert!(!lp.status().locked);

        batches.lock().unwrap().push_back(Ok(grid_edges(40, 40)));
        lp.step(120, false).unwrap();
        assert!(lp.status().locked);

        // Asked to, e.g. before the sensor timing changes.
        batches.lock().unwrap().push_back(Ok(grid_edges(80, 10)));
        lp.step(120, true).unwrap();
        assert!(!lp.status().locked);

        batches.lock().unwrap().push_back(Ok(grid_edges(90, 40)));
        lp.step(120, false).unwrap();
        assert!(lp.status().locked);

        // A failing edge source.
        batches.lock().unwrap().push_back(Err(err_msg("EXTTS read failed")));
        assert!(lp.step(120, false).is_err());
    }

    #[test]
    fn a_status_counts_as_on_the_grid_only_while_fresh_and_at_the_rate() {
        let now = Instant::now();
        let mut status = TriggerStatus {
            frame_rate: 120,
            locked: true,
            updated: Some(now),
            ..Default::default()
        };
        assert!(status.on_grid(120, now));
        assert!(!status.on_grid(60, now));
        assert!(!status.on_grid(0, now));
        assert!(!status.on_grid(120, now + MAX_STATUS_AGE));

        status.updated = None;
        assert!(!status.on_grid(120, now));
    }

    #[test]
    fn the_thread_reports_an_error_unlocked_and_locks_again() {
        let (mut lp, _writes, batches) = fake_loop();
        lp.edge_source.endless_from = Some(400);
        {
            let mut b = batches.lock().unwrap();
            b.push_back(Ok(grid_edges(0, 40)));
            b.push_back(Err(err_msg("EXTTS read failed")));
        }

        let shared = Arc::new(Shared {
            stop: AtomicBool::new(false),
            state: Mutex::new(State::default()),
        });
        shared.state.lock().unwrap().requested_frame_rate = 120;

        let thread = {
            let shared = shared.clone();
            std::thread::spawn(move || PtpPwmTrigger::run(shared, lp))
        };

        let wait_for = |what: &str, f: &dyn Fn(&TriggerStatus) -> bool| {
            let deadline = Instant::now() + Duration::from_secs(5);
            loop {
                let status = shared.state.lock().unwrap().status.clone();
                if f(&status) {
                    return status;
                }
                assert!(Instant::now() < deadline, "never saw: {}", what);
                std::thread::sleep(Duration::from_micros(200));
            }
        };

        let status = wait_for("the error", &|s| s.error.is_some());
        assert!(!status.locked);
        assert!(status.updated.is_some());

        let status = wait_for("a new lock", &|s| s.locked);
        assert!(status.error.is_none());

        shared.stop.store(true, Ordering::Relaxed);
        thread.join().unwrap();
    }
}
