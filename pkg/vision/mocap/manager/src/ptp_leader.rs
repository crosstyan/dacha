// Choice of the camera that leads the PTP time sync of the others.
//
// The leader is the lowest enabled camera id that answers its TimeSync Status
// RPC, and it stays the leader while it works, even when a lower id comes
// back. It is replaced when its Status RPC has failed for STATUS_TIMEOUT, or
// when every follower that has had FOLLOWER_SYNC_TIMEOUT to sync to it reports
// no usable sync within that time. The followers keep their last frequency in
// the meantime, so they drift slowly rather than jump.

use std::time::{Duration, Instant};

/// A leader whose TimeSync Status RPC has not succeeded for this long is
/// replaced. A camera whose last successful status is older is not chosen.
pub const STATUS_TIMEOUT: Duration = Duration::from_secs(5);

/// A follower that has had this long since it and the leader were configured
/// must have had a usable sync within this time, or it counts against the
/// leader.
pub const FOLLOWER_SYNC_TIMEOUT: Duration = Duration::from_secs(10);

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

    /// Time of the camera's last usable sync as a follower, from its last
    /// status.
    pub last_follower_sync: Option<Instant>,

    /// Not chosen as the leader before this time, after it failed as one.
    pub barred_until: Option<Instant>,
}

impl PtpHealth {
    /// Records a successful Status RPC made at 'now'. 'configured' tells
    /// whether the camera reported the intended config, and
    /// 'follower_sync_age' is the age of its last usable sync as a follower
    /// (None if it is not a follower or never synced).
    pub fn record_status(&mut self, now: Instant, configured: bool, follower_sync_age: Option<f64>) {
        self.last_status_ok = Some(now);

        if configured {
            self.configured_since.get_or_insert(now);
        } else {
            self.configured_since = None;
        }

        self.last_follower_sync = follower_sync_age
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

    /// No follower has had a usable sync within FOLLOWER_SYNC_TIMEOUT.
    FollowersNotSyncing,
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
    } else if now < settle_until
        && cameras.iter().any(|c| c.enabled && !status_fresh(c.health, now))
    {
        return LeaderDecision { leader: None, failed: None };
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

    match pick(false).or_else(|| pick(true)) {
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

    // Followers that answer and have had time to sync to this leader.
    let mut judges = cameras.iter()
        .filter(|c| c.camera_id != leader.camera_id && c.enabled && status_fresh(c.health, now))
        .filter(|c| {
            c.health.configured_since
                .map(|t| now.saturating_duration_since(t.max(since)) > FOLLOWER_SYNC_TIMEOUT)
                .unwrap_or(false)
        })
        .peekable();

    if judges.peek().is_none() {
        return None;
    }

    let any_synced = judges.any(|c| {
        c.health.last_follower_sync
            .map(|t| now.saturating_duration_since(t) <= FOLLOWER_SYNC_TIMEOUT)
            .unwrap_or(false)
    });

    if any_synced {
        None
    } else {
        Some(LeaderFailure::FollowersNotSyncing)
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

        /// A camera that answered its last Status RPC at 'status_at', has had its
        /// intended config since 'configured_at' and last synced as a follower at
        /// 'synced_at' (all in seconds).
        fn cam(&mut self, id: u64, status_at: Option<f64>, configured_at: Option<f64>, synced_at: Option<f64>) -> &mut Self {
            let health = PtpHealth {
                last_status_ok: status_at.map(|s| self.at(s)),
                configured_since: configured_at.map(|s| self.at(s)),
                last_follower_sync: synced_at.map(|s| self.at(s)),
                barred_until: None,
            };
            self.cams.push(Cam { id, enabled: true, health });
            self
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
        w.cam(7, Some(99.0), None, None)
            .cam(5, Some(99.0), None, None)
            .cam(2, Some(80.0), None, None) // stale status
            .cam(3, Some(99.0), None, None);
        w.cams[1].enabled = false; // camera 5

        assert_eq!(w.choose(None, 100.0), LeaderDecision { leader: Some(3), failed: None });
    }

    #[test]
    fn first_leader_waits_for_all_cameras_while_settling() {
        let mut w = World::new();
        w.cam(7, Some(1.0), None, None).cam(3, None, None, None);

        // Camera 3 has not answered yet: no choice during the first 5 s.
        assert_eq!(w.choose(None, 2.0).leader, None);
        // After that the cameras that answer are enough.
        assert_eq!(w.choose(None, 5.5).leader, Some(7));

        // Once both answer, the lower id wins even while settling.
        w.cams[1].health.last_status_ok = Some(w.at(1.5));
        assert_eq!(w.choose(None, 2.0).leader, Some(3));
    }

    #[test]
    fn working_leader_is_kept_over_a_lower_id() {
        let mut w = World::new();
        w.cam(7, Some(99.0), Some(10.0), None)
            .cam(3, Some(99.0), Some(10.0), Some(99.5));

        assert_eq!(w.choose(Some((7, 10.0)), 100.0), LeaderDecision { leader: Some(7), failed: None });
    }

    #[test]
    fn leader_whose_status_fails_for_5s_is_replaced() {
        let mut w = World::new();
        w.cam(3, Some(94.0), Some(10.0), None)
            .cam(7, Some(99.0), Some(10.0), Some(99.5))
            .cam(9, Some(99.0), Some(10.0), Some(99.5));

        // 4 s without an answer: still the leader.
        assert_eq!(w.choose(Some((3, 10.0)), 98.0).leader, Some(3));

        // 6 s: replaced by the lowest other camera.
        assert_eq!(
            w.choose(Some((3, 10.0)), 100.0),
            LeaderDecision { leader: Some(7), failed: Some((3, LeaderFailure::StatusFailing)) }
        );
    }

    #[test]
    fn leader_is_replaced_when_no_follower_syncs_for_10s() {
        let mut w = World::new();
        w.cam(3, Some(99.0), Some(10.0), None)
            .cam(7, Some(99.0), Some(10.0), Some(88.0))
            .cam(9, Some(99.0), Some(10.0), None);

        assert_eq!(
            w.choose(Some((3, 10.0)), 100.0),
            LeaderDecision { leader: Some(7), failed: Some((3, LeaderFailure::FollowersNotSyncing)) }
        );

        // One follower that syncs is enough to keep it.
        w.cams[2].health.last_follower_sync = Some(w.at(95.0));
        assert_eq!(w.choose(Some((3, 10.0)), 100.0).leader, Some(3));
    }

    #[test]
    fn followers_get_10s_to_sync_to_a_new_leader() {
        let mut w = World::new();
        w.cam(3, Some(99.0), Some(10.0), None)
            .cam(7, Some(99.0), Some(10.0), Some(80.0))
            .cam(9, Some(99.0), Some(95.0), None);

        // Leader since 92 s: camera 7 has had 8 s, camera 9 was configured 5 s ago.
        assert_eq!(w.choose(Some((3, 92.0)), 100.0).leader, Some(3));
        // A follower whose status is stale does not count either.
        w.cams[1].health.last_status_ok = Some(w.at(90.0));
        assert_eq!(w.choose(Some((3, 50.0)), 100.0).leader, Some(3));
    }

    #[test]
    fn barred_camera_leads_only_when_no_other_can() {
        let mut w = World::new();
        w.cam(3, Some(99.0), None, None).cam(7, Some(99.0), None, None);
        w.cams[0].health.barred_until = Some(w.at(120.0));

        assert_eq!(w.choose(None, 100.0).leader, Some(7));

        w.cams[1].health.last_status_ok = Some(w.at(80.0));
        assert_eq!(w.choose(None, 100.0).leader, Some(3));
    }

    #[test]
    fn failing_leader_is_kept_when_no_other_camera_answers() {
        let mut w = World::new();
        w.cam(3, Some(80.0), Some(10.0), None).cam(7, Some(80.0), Some(10.0), None);

        assert_eq!(w.choose(Some((3, 10.0)), 100.0), LeaderDecision { leader: Some(3), failed: None });
    }

    #[test]
    fn removed_or_disabled_leader_is_replaced() {
        let mut w = World::new();
        // Camera 9 syncs as a follower, so a leader 7 that stayed enabled would be kept.
        w.cam(7, Some(99.0), Some(10.0), None).cam(9, Some(99.0), Some(10.0), Some(99.5));

        // Camera 3 is gone from the list.
        assert_eq!(w.choose(Some((3, 10.0)), 100.0), LeaderDecision { leader: Some(7), failed: None });

        w.cams[0].enabled = false;
        assert_eq!(w.choose(Some((7, 10.0)), 100.0), LeaderDecision { leader: Some(9), failed: None });
    }

    #[test]
    fn record_status_tracks_config_and_sync() {
        let start = Instant::now();
        let at = |s: f64| start + Duration::from_secs_f64(s);

        let mut h = PtpHealth::default();
        h.record_status(at(10.0), true, Some(0.25));
        h.record_status(at(11.0), true, None);
        assert_eq!(h.configured_since, Some(at(10.0)));
        assert_eq!(h.last_status_ok, Some(at(11.0)));
        assert_eq!(h.last_follower_sync, None);

        h.record_status(at(12.0), false, Some(0.5));
        assert_eq!(h.configured_since, None);
        assert_eq!(h.last_follower_sync, Some(at(11.5)));
    }
}
