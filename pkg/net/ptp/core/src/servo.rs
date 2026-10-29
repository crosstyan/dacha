// Parts of the clock servo that do not touch any hardware, so that they can be
// tested on recorded sync rounds.

use std::collections::VecDeque;
use std::time::Instant;

use ptp_proto::ptp::{FollowerConfig, TimeAdjustmentConfig};

/// Tracks the smallest round trip time seen over the last few sync rounds.
///
/// Through a switch that does not take part in PTP, a sync packet that waits
/// behind other traffic is late in one direction only, which moves the
/// measured offset by half the wait. The round trip grows by the same wait, so
/// the rounds whose RTT is close to the recent minimum are the ones that did
/// not queue. A fixed RTT limit can not do this job: the RTT with no queueing
/// depends on the link speed and on the switches in the path (28 us between
/// two 100 Mbit/s cameras on one switch, against a 10 us default meant for
/// 1 Gbit/s).
pub struct MinRttFilter {
    window: usize,
    samples: VecDeque<u64>,
}

impl MinRttFilter {
    pub fn new(window: usize) -> Self {
        Self {
            window: window.max(1),
            samples: VecDeque::new(),
        }
    }

    /// Records one RTT in nanoseconds and returns the minimum over the window,
    /// this sample included.
    pub fn add(&mut self, rtt: u64) -> u64 {
        self.samples.push_back(rtt);
        while self.samples.len() > self.window {
            self.samples.pop_front();
        }

        *self.samples.iter().min().unwrap()
    }

    pub fn min(&self) -> Option<u64> {
        self.samples.iter().min().cloned()
    }

    /// Records the RTT of one round in nanoseconds and decides whether the
    /// round can be used to adjust the clock.
    pub fn check(&mut self, config: &FollowerConfig, rtt: u64) -> RttVerdict {
        let min_rtt = self.add(rtt);

        if rtt as f64 > config.max_network_rtt() as f64 * 1e9 {
            return RttVerdict::TooLong;
        }

        let max_above_min = config.max_rtt_above_min() as f64 * 1e9;
        if max_above_min > 0.0 && (rtt - min_rtt) as f64 > max_above_min {
            return RttVerdict::Queued;
        }

        RttVerdict::Usable
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RttVerdict {
    Usable,

    /// Above FollowerConfig.max_network_rtt.
    TooLong,

    /// More than FollowerConfig.max_rtt_above_min above the recent minimum:
    /// the packets waited in a queue on the way.
    Queued,
}

/// What the PI filter wants done to the clock.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClockCorrection {
    /// If present, the clock must first be stepped by this many seconds.
    pub step_seconds: Option<f64>,

    /// Frequency correction to apply, in ppm.
    pub freq_ppm: f64,
}

#[derive(Default)]
pub struct PIFilterState {
    integral: f64,
    last_time: Option<Instant>,
    last_output: f64,
}

impl PIFilterState {
    /// Feeds one measured error (remote time minus local time, in seconds)
    /// taken at 'now'.
    pub fn update(
        &mut self,
        config: &TimeAdjustmentConfig,
        mut error: f64,
        now: Instant,
    ) -> ClockCorrection {
        let mut step_seconds = None;
        if error.abs() > config.max_offset_seconds() {
            step_seconds = Some(error);
            *self = Self::default();
            error = 0.0;
        }

        let mut integral = self.integral;
        if let Some(last_time) = &self.last_time {
            // TODO: Disallow large jumps between syncs.

            let dt = (now - *last_time).as_secs_f64();
            integral = clamp_magnitude(integral + dt * error, config.max_error_integral_secs());
        }
        self.last_time = Some(now);

        let wanted = Self::output_ppm(config, error, integral);
        let out = self.limit_ppm(config, wanted);

        // Anti-windup: while a limit holds the output back, the integral must
        // not keep growing toward that limit. Otherwise it grows for the whole
        // of a long slew and then drives the clock far past zero (from a 3 ms
        // offset on an RV1126B: a +2 ms overshoot and about 140 s to settle).
        let held_back = (out - wanted).abs() > 1e-9;
        let integral_grows_toward_limit = (integral - self.integral) * wanted > 0.0;
        let out = if held_back && integral_grows_toward_limit {
            self.limit_ppm(config, Self::output_ppm(config, error, self.integral))
        } else {
            self.integral = integral;
            out
        };

        self.last_output = out;

        ClockCorrection {
            step_seconds,
            freq_ppm: out,
        }
    }

    /// Called while the filter is not fed (for example while its source is not
    /// trusted). Forgets the time of the last update, so that the next update
    /// does not integrate its error over the whole pause. The integral, which
    /// holds the frequency difference, is kept.
    pub fn hold(&mut self) {
        self.last_time = None;
    }

    fn output_ppm(config: &TimeAdjustmentConfig, error: f64, integral: f64) -> f64 {
        (config.p_scale() * error + config.i_scale() * integral) * 1_000_000.0
    }

    fn limit_ppm(&self, config: &TimeAdjustmentConfig, mut out: f64) -> f64 {
        out = clamp_magnitude(out, config.max_correction_ppm());

        if config.max_frequency_step_ppm() != 0.0 {
            let max_step = config.max_frequency_step_ppm();
            if (out - self.last_output).abs() > max_step {
                out = (out - self.last_output).signum() * max_step + self.last_output;
            }
        }

        out
    }
}

pub fn clamp_magnitude(mut v: f64, max: f64) -> f64 {
    let max_p = max.abs();
    let max_n = -max_p;
    if v > max_p {
        v = max_p;
    }
    if v < max_n {
        v = max_n;
    }

    v
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::time::Duration;

    use crate::default_config_template;

    /// Sync rounds recorded between two RV1126B cameras (stmmac PHC, 100 Mbit/s)
    /// on one CRS328 switch while the host sent 20 Mbit/s of UDP to the
    /// follower and the follower streamed video. One line per round:
    /// "rtt_ns err_ns". The follower's clock was held by ptp4l, so the error of
    /// a round that did not queue is within about 100 ns.
    const LOADED_ROUNDS: &str = include_str!("../testdata/rounds_100m_loaded.txt");

    /// The same link with no other traffic.
    const IDLE_ROUNDS: &str = include_str!("../testdata/rounds_100m_idle.txt");

    fn parse_rounds(data: &str) -> Vec<(u64, i64)> {
        data.lines()
            .filter(|l| !l.is_empty() && !l.starts_with('#'))
            .map(|l| {
                let mut it = l.split_whitespace();
                let rtt = it.next().unwrap().parse().unwrap();
                let err = it.next().unwrap().parse().unwrap();
                (rtt, err)
            })
            .collect()
    }

    /// Runs the follower's RTT checks on recorded rounds and returns the
    /// errors of the rounds it keeps.
    fn kept_errors(rounds: &[(u64, i64)]) -> Vec<i64> {
        let config = default_config_template();
        let follower = config.follower();

        let mut filter = MinRttFilter::new(follower.rtt_min_window() as usize);
        rounds.iter()
            .filter(|(rtt, _)| filter.check(follower, *rtt) == RttVerdict::Usable)
            .map(|(_, err)| *err)
            .collect()
    }

    #[test]
    fn min_rtt_filter_forgets_old_samples() {
        let mut f = MinRttFilter::new(3);
        assert_eq!(f.add(30), 30);
        assert_eq!(f.add(20), 20);
        assert_eq!(f.add(40), 20);
        assert_eq!(f.add(50), 20);
        assert_eq!(f.add(60), 40);
        assert_eq!(f.min(), Some(40));
    }

    #[test]
    fn default_rtt_checks_keep_idle_100mbit_rounds() {
        let rounds = parse_rounds(IDLE_ROUNDS);
        let kept = kept_errors(&rounds);
        assert!(kept.len() * 100 >= rounds.len() * 99, "kept {} of {}", kept.len(), rounds.len());
    }

    #[test]
    fn default_rtt_checks_drop_queued_rounds() {
        let rounds = parse_rounds(LOADED_ROUNDS);
        assert!(rounds.iter().any(|(_, e)| e.abs() > 10_000), "fixture has no queued rounds");

        let kept = kept_errors(&rounds);
        let worst = kept.iter().map(|e| e.abs()).max().unwrap();
        assert!(worst <= 200, "a kept round has an error of {} ns", worst);
        assert!(kept.len() * 100 >= rounds.len() * 75, "kept only {} of {}", kept.len(), rounds.len());
    }

    /// Simulates a follower whose clock starts 'offset' seconds behind the
    /// leader and runs 'drift_ppm' slow, syncing every 0.25 s with perfect
    /// measurements. Returns the error after each round.
    fn simulate(config: &TimeAdjustmentConfig, offset: f64, drift_ppm: f64, seconds: f64) -> Vec<f64> {
        simulate_every(config, 0.25, offset, drift_ppm, seconds)
    }

    fn simulate_every(
        config: &TimeAdjustmentConfig,
        dt: f64,
        offset: f64,
        drift_ppm: f64,
        seconds: f64,
    ) -> Vec<f64> {
        let mut pi = PIFilterState::default();
        let start = Instant::now();

        let mut error = offset;
        let mut errors = vec![];
        let mut t = 0.0;
        while t < seconds {
            let c = pi.update(config, error, start + Duration::from_secs_f64(t));
            if let Some(step) = c.step_seconds {
                error -= step;
            }
            error -= (c.freq_ppm - drift_ppm) * 1e-6 * dt;
            errors.push(error);
            t += dt;
        }

        errors
    }

    #[test]
    fn follower_slews_without_overshoot() {
        let config = default_config_template();
        let clock_sync = config.follower().leader_clock_sync();

        // 3 ms is below the 10 ms step threshold, so the whole offset is slewed
        // at the 100 ppm limit.
        let errors = simulate(clock_sync, 3e-3, 15.0, 120.0);

        let overshoot = errors.iter().cloned().fold(0.0f64, f64::min);
        assert!(overshoot > -100e-6, "overshoot of {} s", overshoot);

        let settled_from = (90.0 / 0.25) as usize;
        let worst_after = errors[settled_from..].iter().map(|e| e.abs()).fold(0.0, f64::max);
        assert!(worst_after < 100e-9, "still {} s off after 90 s", worst_after);
    }

    #[test]
    fn follower_steps_large_offsets() {
        let config = default_config_template();
        let errors = simulate(config.follower().leader_clock_sync(), 0.5, 15.0, 60.0);
        assert!(errors[0].abs() < 1e-3);
        assert!(errors.last().unwrap().abs() < 100e-9);
    }

    #[test]
    fn system_clock_locks_within_30_seconds() {
        // Just below the 1 ms step limit, with a system clock 30 ppm off the PTP clock.
        let config = crate::default_system_clock_sync();
        let errors = simulate_every(&config, 1.0, 0.9e-3, 30.0, 60.0);

        let worst_after = errors[30..].iter().map(|e| e.abs()).fold(0.0, f64::max);
        assert!(worst_after < 1e-6, "still {} s off after 30 s", worst_after);
    }

    #[test]
    fn held_filter_does_not_integrate_over_the_pause() {
        let config = crate::default_system_clock_sync();
        let error = 1e-6;
        let start = Instant::now();

        let mut pi = PIFilterState::default();
        pi.update(&config, error, start);
        pi.hold();
        let c = pi.update(&config, error, start + Duration::from_secs(100));

        // Only the proportional term. Integrating over the pause would add 30 ppm (a longer
        // pause would reach the 200 ppm clamp, where the anti-windup hides it).
        let expected = config.p_scale() * error * 1e6;
        assert!((c.freq_ppm - expected).abs() < 1e-9, "{} ppm instead of {} ppm", c.freq_ppm, expected);
    }
}
