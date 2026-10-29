// Choice of the camera that leads the PTP time sync of the others.
//
// The leader is the lowest enabled camera id that answers its TimeSync Status
// RPC, and it stays the leader while it works, even when a lower id comes
// back. It is replaced when its Status RPC has failed for STATUS_TIMEOUT, or
// when it has completed no round with any follower for ROUND_TIMEOUT although
// followers have been configured for that long. The followers keep their last
// frequency in the meantime, so they drift slowly rather than jump.
//
// The leader is judged by its own rounds (its status per follower), not by
// whether the followers used them: a round that a follower skipped as queued
// still shows that the leader works, and one broken follower must not make
// the leader change again and again.

use std::time::{Duration, Instant};

use ptp_proto::ptp::{StatusResponse, TimeSyncConfig};

/// A leader whose TimeSync Status RPC has not succeeded for this long is
/// replaced. A camera whose last successful status is older is not chosen.
pub const STATUS_TIMEOUT: Duration = Duration::from_secs(5);

/// Once followers have been configured with the leader for this long, the
/// leader must have completed a round with one of them within this time.
pub const ROUND_TIMEOUT: Duration = Duration::from_secs(10);

/// A replaced leader is not chosen again for this long, unless no other
/// camera can lead.
pub const FAILED_LEADER_BAR: Duration = Duration::from_secs(60);

/// What the manager has seen of one camera's time sync.
#[derive(Default, Clone, Debug)]
pub struct PtpHealth {
    /// Last time the camera's TimeSync Status RPC succeeded.
    pub last_status_ok: Option<Instant>,

    /// Since when the camera has reported the TimeSync config that the manager
    /// wants it to have.
    pub configured_since: Option<Instant>,

    /// Whether the camera's last status listed any followers in its leader
    /// status. Cameras from before the leader status never do; such a leader
    /// is judged by its Status RPC only.
    pub reports_leader_rounds: bool,

    /// Time of the leader's last completed round with any follower, from its
    /// last status.
    pub last_leader_round: Option<Instant>,

    /// Not chosen as the leader before this time, after it failed as one.
    pub barred_until: Option<Instant>,
}

/// What one camera's TimeSync status says, for PtpHealth::record_status().
#[derive(Default, Clone, Debug)]
pub struct PtpStatusSummary {
    /// The camera reported the config the manager wants it to have.
    pub configured: bool,

    /// Number of followers in the camera's leader status.
    pub num_leader_followers: usize,

    /// Age in seconds of the camera's last completed round with any follower,
    /// as a leader.
    pub last_leader_round_age: Option<f64>,
}

/// Summarizes a camera's TimeSync status. 'intended' is the config the manager wants the
/// camera to have.
pub fn ptp_status_summary(status: &StatusResponse, intended: &TimeSyncConfig) -> PtpStatusSummary {
    let followers = status.leader().followers();
    PtpStatusSummary {
        configured: status.config() == intended,
        num_leader_followers: if status.config().has_leader() { followers.len() } else { 0 },
        last_leader_round_age: followers.iter()
            .filter(|f| f.got_round())
            .map(|f| f.last_round_age())
            .fold(None, |min: Option<f64>, age| Some(min.map_or(age, |m| m.min(age)))),
    }
}

impl PtpHealth {
    /// Records a successful Status RPC made at 'now'.
    pub fn record_status(&mut self, now: Instant, status: &PtpStatusSummary) {
        self.last_status_ok = Some(now);

        if status.configured {
            self.configured_since.get_or_insert(now);
        } else {
            self.configured_since = None;
        }

        self.reports_leader_rounds = status.num_leader_followers > 0;
        self.last_leader_round = status.last_leader_round_age
            .filter(|age| age.is_finite() && *age >= 0.0)
            .and_then(|age| now.checked_sub(Duration::from_secs_f64(age)));
    }
}

pub struct PtpCandidate<'a> {
    pub camera_id: u64,
    pub enabled: bool,
    pub health: &'a PtpHealth,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LeaderFailure {
    /// The leader's Status RPC has not succeeded for STATUS_TIMEOUT.
    StatusFailing,

    /// The leader has completed no round with any follower within ROUND_TIMEOUT.
    NoRounds,
}

#[derive(Debug, Clone, PartialEq)]
pub struct LeaderDecision {
    pub leader: Option<u64>,

    /// Set when the current leader is replaced because it failed. The caller
    /// bars it for FAILED_LEADER_BAR.
    pub failed: Option<(u64, LeaderFailure)>,
}

/// Decides which camera leads.
///
/// 'current' is the current leader and when it became the leader. Until
/// 'settle_until' (a few seconds after the manager starts) a first leader is
/// only chosen once every connected enabled camera has answered, so that the
/// choice does not depend on which camera answered first.
pub fn choose_ptp_leader(
    cameras: &[PtpCandidate],
    current: Option<(u64, Instant)>,
    settle_until: Instant,
    now: Instant,
) -> LeaderDecision {
    let mut failed = None;

    if let Some((id, since)) = current {
        if let Some(leader) = cameras.iter().find(|c| c.camera_id == id && c.enabled) {
            match leader_failure(cameras, leader, since, now) {
                None => {
                    return LeaderDecision { leader: Some(id), failed: None };
                }
                Some(failure) => {
                    failed = Some((id, failure));
                }
            }
        }
    }

    let failed_id = failed.map(|(id, _)| id);
    let pick = |allow_barred: bool| {
        cameras.iter()
            .filter(|c| c.enabled && status_fresh(c.health, now))
            .filter(|c| Some(c.camera_id) != failed_id)
            .filter(|c| allow_barred || !c.health.barred_until.map(|t| now < t).unwrap_or(false))
            .map(|c| c.camera_id)
            .min()
    };

    if current.is_none()
        && now < settle_until
        && cameras.iter().any(|c| c.enabled && !status_fresh(c.health, now))
    {
        return LeaderDecision { leader: None, failed: None };
    }

    // A leader that answers but completes no rounds may itself be fine (its followers may be
    // the problem), so only a camera without a recent failure replaces it. A leader that does
    // not answer is replaced by any camera that does.
    let allow_barred = !matches!(failed, Some((_, LeaderFailure::NoRounds)));

    let choice = pick(false).or_else(|| if allow_barred { pick(true) } else { None });
    match choice {
        Some(id) => LeaderDecision { leader: Some(id), failed },
        // No other camera can lead: keep a failing leader rather than having none.
        None => LeaderDecision { leader: failed_id, failed: None },
    }
}

fn status_fresh(health: &PtpHealth, now: Instant) -> bool {
    health.last_status_ok
        .map(|t| now.saturating_duration_since(t) <= STATUS_TIMEOUT)
        .unwrap_or(false)
}

fn leader_failure(
    cameras: &[PtpCandidate],
    leader: &PtpCandidate,
    since: Instant,
    now: Instant,
) -> Option<LeaderFailure> {
    if !status_fresh(leader.health, now) && now.saturating_duration_since(since) > STATUS_TIMEOUT {
        return Some(LeaderFailure::StatusFailing);
    }

    // Cameras from before the leader status: nothing more to judge by.
    if !leader.health.reports_leader_rounds {
        return None;
    }

    // The leader must have had its config, and at least one answering follower its config,
    // for ROUND_TIMEOUT since this camera became the leader.
    let leader_configured = match leader.health.configured_since {
        Some(t) => t.max(since),
        None => return None,
    };
    let has_ready_follower = cameras.iter()
        .filter(|c| c.camera_id != leader.camera_id && c.enabled && status_fresh(c.health, now))
        .any(|c| {
            c.health.configured_since
                .map(|t| now.saturating_duration_since(t.max(leader_configured)) > ROUND_TIMEOUT)
                .unwrap_or(false)
        });

    if !has_ready_follower {
        return None;
    }

    let recent_round = leader.health.last_leader_round
        .map(|t| now.saturating_duration_since(t) <= ROUND_TIMEOUT)
        .unwrap_or(false);

    if recent_round {
        None
    } else {
        Some(LeaderFailure::NoRounds)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Cam {
        id: u64,
        enabled: bool,
        health: PtpHealth,
    }

    struct World {
        start: Instant,
        cams: Vec<Cam>,
    }

    impl World {
        fn new() -> Self {
            Self { start: Instant::now(), cams: vec![] }
        }

        fn at(&self, secs: f64) -> Instant {
            self.start + Duration::from_secs_f64(secs)
        }

        /// A camera that answered its last Status RPC at 'status_at' and has had its
        /// intended config since 'configured_at' (in seconds).
        fn cam(&mut self, id: u64, status_at: Option<f64>, configured_at: Option<f64>) -> &mut Self {
            let health = PtpHealth {
                last_status_ok: status_at.map(|s| self.at(s)),
                configured_since: configured_at.map(|s| self.at(s)),
                ..Default::default()
            };
            self.cams.push(Cam { id, enabled: true, health });
            self
        }

        /// Camera 'id' leads and reports its rounds; its last completed one was at
        /// 'round_at'.
        fn rounds(&mut self, id: u64, round_at: Option<f64>) -> &mut Self {
            let at = round_at.map(|s| self.at(s));
            let cam = self.cams.iter_mut().find(|c| c.id == id).unwrap();
            cam.health.reports_leader_rounds = true;
            cam.health.last_leader_round = at;
            self
        }

        fn health(&mut self, id: u64) -> &mut PtpHealth {
            &mut self.cams.iter_mut().find(|c| c.id == id).unwrap().health
        }

        fn choose(&self, current: Option<(u64, f64)>, now: f64) -> LeaderDecision {
            let candidates = self.cams.iter()
                .map(|c| PtpCandidate { camera_id: c.id, enabled: c.enabled, health: &c.health })
                .collect::<Vec<_>>();
            choose_ptp_leader(&candidates, current.map(|(id, s)| (id, self.at(s))), self.at(5.0), self.at(now))
        }
    }

    #[test]
    fn first_leader_is_the_lowest_enabled_answering_camera() {
        let mut w = World::new();
        w.cam(7, Some(99.0), None)
            .cam(5, Some(99.0), None)
            .cam(2, Some(80.0), None) // stale status
            .cam(3, Some(99.0), None);
        w.cams[1].enabled = false; // camera 5

        assert_eq!(w.choose(None, 100.0), LeaderDecision { leader: Some(3), failed: None });
    }

    #[test]
    fn first_leader_waits_for_all_cameras_while_settling() {
        let mut w = World::new();
        w.cam(7, Some(1.0), None).cam(3, None, None);

        // Camera 3 has not answered yet: no choice during the first 5 s.
        assert_eq!(w.choose(None, 2.0).leader, None);
        // After that the cameras that answer are enough.
        assert_eq!(w.choose(None, 5.5).leader, Some(7));

        // Once both answer, the lower id wins even while settling.
        w.health(3).last_status_ok = Some(w.at(1.5));
        assert_eq!(w.choose(None, 2.0).leader, Some(3));
    }

    #[test]
    fn working_leader_is_kept_over_a_lower_id() {
        let mut w = World::new();
        w.cam(7, Some(99.0), Some(10.0)).cam(3, Some(99.0), Some(10.0)).rounds(7, Some(99.5));

        assert_eq!(w.choose(Some((7, 10.0)), 100.0), LeaderDecision { leader: Some(7), failed: None });
    }

    #[test]
    fn leader_whose_status_fails_for_5s_is_replaced() {
        let mut w = World::new();
        w.cam(3, Some(94.0), Some(10.0))
            .cam(7, Some(99.0), Some(10.0))
            .cam(9, Some(99.0), Some(10.0))
            .rounds(3, Some(93.9));

        // 4 s without an answer: still the leader.
        assert_eq!(w.choose(Some((3, 10.0)), 98.0).leader, Some(3));

        // 6 s: replaced by the lowest other camera.
        assert_eq!(
            w.choose(Some((3, 10.0)), 100.0),
            LeaderDecision { leader: Some(7), failed: Some((3, LeaderFailure::StatusFailing)) }
        );
    }

    #[test]
    fn leader_is_replaced_when_it_completes_no_round_for_10s() {
        let mut w = World::new();
        w.cam(3, Some(99.0), Some(10.0))
            .cam(7, Some(99.0), Some(10.0))
            .cam(9, Some(99.0), Some(10.0))
            .rounds(3, Some(88.0));

        assert_eq!(
            w.choose(Some((3, 10.0)), 100.0),
            LeaderDecision { leader: Some(7), failed: Some((3, LeaderFailure::NoRounds)) }
        );

        // A round with any follower within 10 s keeps it, whatever the followers did with it.
        w.rounds(3, Some(95.0));
        assert_eq!(w.choose(Some((3, 10.0)), 100.0).leader, Some(3));
    }

    #[test]
    fn leader_gets_10s_after_it_and_a_follower_are_configured() {
        let mut w = World::new();
        w.cam(3, Some(99.0), Some(10.0))
            .cam(7, Some(99.0), Some(10.0))
            .cam(9, Some(99.0), Some(95.0))
            .rounds(3, None);

        // Leader since 92 s: the followers have had 8 s with it at most.
        assert_eq!(w.choose(Some((3, 92.0)), 100.0).leader, Some(3));

        // Leader since 50 s, but camera 7's status is stale and camera 9 was configured
        // 5 s ago: no follower that could have had a round.
        w.health(7).last_status_ok = Some(w.at(90.0));
        assert_eq!(w.choose(Some((3, 50.0)), 100.0).leader, Some(3));

        // The leader itself got its config 5 s ago (a new follower list).
        w.health(7).last_status_ok = Some(w.at(99.0));
        w.health(3).configured_since = Some(w.at(95.0));
        assert_eq!(w.choose(Some((3, 50.0)), 100.0).leader, Some(3));
    }

    #[test]
    fn leader_without_rounds_is_not_replaced_by_a_barred_camera() {
        let mut w = World::new();
        w.cam(3, Some(99.0), Some(10.0)).cam(7, Some(99.0), Some(10.0)).rounds(3, None);
        w.health(7).barred_until = Some(w.at(120.0));

        // Camera 7 failed as a leader recently, so it does not replace a leader that still
        // answers: the fault may be on the followers' side.
        assert_eq!(w.choose(Some((3, 10.0)), 100.0), LeaderDecision { leader: Some(3), failed: None });

        // A leader that does not answer is replaced even by a barred camera.
        w.health(3).last_status_ok = Some(w.at(90.0));
        assert_eq!(
            w.choose(Some((3, 10.0)), 100.0),
            LeaderDecision { leader: Some(7), failed: Some((3, LeaderFailure::StatusFailing)) }
        );
    }

    #[test]
    fn leader_that_reports_no_rounds_is_judged_by_its_status_only() {
        // A camera from before the leader status lists no followers.
        let mut w = World::new();
        w.cam(3, Some(99.0), Some(10.0)).cam(7, Some(99.0), Some(10.0));

        assert_eq!(w.choose(Some((3, 10.0)), 100.0), LeaderDecision { leader: Some(3), failed: None });
    }

    #[test]
    fn barred_camera_leads_only_when_no_other_can() {
        let mut w = World::new();
        w.cam(3, Some(99.0), None).cam(7, Some(99.0), None);
        w.health(3).barred_until = Some(w.at(120.0));

        assert_eq!(w.choose(None, 100.0).leader, Some(7));

        w.health(7).last_status_ok = Some(w.at(80.0));
        assert_eq!(w.choose(None, 100.0).leader, Some(3));
    }

    #[test]
    fn failing_leader_is_kept_when_no_other_camera_answers() {
        let mut w = World::new();
        w.cam(3, Some(80.0), Some(10.0)).cam(7, Some(80.0), Some(10.0));

        assert_eq!(w.choose(Some((3, 10.0)), 100.0), LeaderDecision { leader: Some(3), failed: None });
    }

    #[test]
    fn removed_or_disabled_leader_is_replaced() {
        let mut w = World::new();
        // Camera 7 completes rounds, so a leader 7 that stayed enabled would be kept.
        w.cam(7, Some(99.0), Some(10.0)).cam(9, Some(99.0), Some(10.0)).rounds(7, Some(99.5));

        // Camera 3 is gone from the list.
        assert_eq!(w.choose(Some((3, 10.0)), 100.0), LeaderDecision { leader: Some(7), failed: None });

        w.cams[0].enabled = false;
        assert_eq!(w.choose(Some((7, 10.0)), 100.0), LeaderDecision { leader: Some(9), failed: None });
    }

    #[test]
    fn status_summary_takes_the_latest_round_with_any_follower() {
        let mut intended = TimeSyncConfig::default();
        intended.set_leader(ptp_proto::ptp::LeaderConfig::default());

        let mut status = StatusResponse::default();
        status.set_config(intended.clone());
        let f = status.leader_mut().new_followers();
        f.set_got_round(true);
        f.set_last_round_age(3.0);
        let f = status.leader_mut().new_followers();
        f.set_got_round(true);
        f.set_last_round_age(0.5);
        let f = status.leader_mut().new_followers();
        f.set_last_round_age(0.1); // no completed round: the age means nothing

        let summary = ptp_status_summary(&status, &intended);
        assert!(summary.configured);
        assert_eq!(summary.num_leader_followers, 3);
        assert_eq!(summary.last_leader_round_age, Some(0.5));

        // Leader status from a camera that is no longer a leader does not count.
        let mut follower = TimeSyncConfig::default();
        follower.set_follower(ptp_proto::ptp::FollowerConfig::default());
        status.set_config(follower);
        let summary = ptp_status_summary(&status, &intended);
        assert!(!summary.configured);
        assert_eq!(summary.num_leader_followers, 0);
    }

    #[test]
    fn record_status_tracks_config_and_rounds() {
        let start = Instant::now();
        let at = |s: f64| start + Duration::from_secs_f64(s);

        let mut h = PtpHealth::default();
        let leading = PtpStatusSummary {
            configured: true,
            num_leader_followers: 2,
            last_leader_round_age: Some(0.25),
            ..Default::default()
        };
        h.record_status(at(10.0), &leading);
        h.record_status(at(11.0), &leading);
        assert_eq!(h.configured_since, Some(at(10.0)));
        assert_eq!(h.last_status_ok, Some(at(11.0)));
        assert!(h.reports_leader_rounds);
        assert_eq!(h.last_leader_round, Some(at(10.75)));

        h.record_status(at(12.0), &PtpStatusSummary::default());
        assert_eq!(h.configured_since, None);
        assert!(!h.reports_leader_rounds);
        assert_eq!(h.last_leader_round, None);
    }
}
