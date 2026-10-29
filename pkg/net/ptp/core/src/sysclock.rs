// Decides when a node may steer its system clock (CLOCK_REALTIME) to its PTP
// clock, the job phc2sys does next to ptp4l.

/// Unix time of 2024-01-01. A PTP clock before this was never set: some PHCs
/// start at 0 when the driver loads, and a follower of such a leader copies it.
pub const MIN_PLAUSIBLE_UNIX_SECS: f64 = 1_704_067_200.0;

/// Default for the 'max_sync_age' argument of phc_trust(), in seconds. A
/// follower is synced 4 times a second and a basic client twice a second, so
/// this is many missed rounds.
pub const DEFAULT_MAX_SYNC_AGE: f64 = 5.0;

/// What phc_trust() looks at, all taken from the node's own state.
#[derive(Debug, Clone, Default)]
pub struct PhcTrustInputs {
    pub is_leader: bool,
    pub is_follower: bool,
    pub has_basic_client: bool,

    /// Seconds since the last usable sync from the leader, if any.
    pub follower_last_sync_age: Option<f64>,

    /// Seconds since the last sync of the basic client to the time server, if any.
    pub basic_client_last_sync_age: Option<f64>,

    /// Current time of the PTP clock in seconds since the Unix epoch.
    pub ptp_time_secs: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum PhcTrust {
    /// The PTP clock follows a time source, so the system clock can follow it.
    Trusted,

    /// The system clock must be left alone, for the given reason.
    Untrusted(&'static str),
}

/// Returns whether the PTP clock can be used as the source for the system clock.
///
/// A leader without a basic client is not trusted: its PTP clock is the free
/// running reference of the group and follows nothing, so it is no better than
/// the system clock (which may have an RTC or NTP behind it), and right after
/// boot it may still read 1970.
pub fn phc_trust(inputs: &PhcTrustInputs, max_sync_age: f64) -> PhcTrust {
    let recent = |age: Option<f64>| age.map(|a| a <= max_sync_age).unwrap_or(false);

    if inputs.ptp_time_secs < MIN_PLAUSIBLE_UNIX_SECS {
        return PhcTrust::Untrusted("the PTP clock reads a time before 2024, so it was never set");
    }

    if inputs.is_follower {
        if recent(inputs.follower_last_sync_age) {
            return PhcTrust::Trusted;
        }
        return PhcTrust::Untrusted("no recent usable sync from the leader");
    }

    if inputs.is_leader {
        if !inputs.has_basic_client {
            return PhcTrust::Untrusted("leader without a time server: its PTP clock follows nothing");
        }
        if recent(inputs.basic_client_last_sync_age) {
            return PhcTrust::Trusted;
        }
        return PhcTrust::Untrusted("no recent sync from the time server");
    }

    PhcTrust::Untrusted("not configured as a leader or follower")
}

#[cfg(test)]
mod tests {
    use super::*;

    // 2026-10-29
    const NOW: f64 = 1_793_232_000.0;

    fn follower(age: Option<f64>) -> PhcTrustInputs {
        PhcTrustInputs {
            is_follower: true,
            follower_last_sync_age: age,
            ptp_time_secs: NOW,
            ..Default::default()
        }
    }

    fn leader(has_basic_client: bool, age: Option<f64>) -> PhcTrustInputs {
        PhcTrustInputs {
            is_leader: true,
            has_basic_client,
            basic_client_last_sync_age: age,
            ptp_time_secs: NOW,
            ..Default::default()
        }
    }

    #[test]
    fn follower_is_trusted_only_with_a_recent_sync() {
        assert_eq!(phc_trust(&follower(Some(0.3)), DEFAULT_MAX_SYNC_AGE), PhcTrust::Trusted);
        assert_ne!(phc_trust(&follower(Some(6.0)), DEFAULT_MAX_SYNC_AGE), PhcTrust::Trusted);
        assert_ne!(phc_trust(&follower(None), DEFAULT_MAX_SYNC_AGE), PhcTrust::Trusted);
    }

    #[test]
    fn leader_is_trusted_only_with_a_synced_time_server() {
        assert_eq!(phc_trust(&leader(true, Some(0.5)), DEFAULT_MAX_SYNC_AGE), PhcTrust::Trusted);
        assert_ne!(phc_trust(&leader(true, Some(6.0)), DEFAULT_MAX_SYNC_AGE), PhcTrust::Trusted);
        assert_ne!(phc_trust(&leader(true, None), DEFAULT_MAX_SYNC_AGE), PhcTrust::Trusted);
    }

    #[test]
    fn leader_without_a_time_server_is_not_trusted() {
        // A recent sync age must not matter here: without a basic client the leader's PTP
        // clock follows nothing.
        assert_ne!(phc_trust(&leader(false, Some(0.5)), DEFAULT_MAX_SYNC_AGE), PhcTrust::Trusted);
    }

    #[test]
    fn unset_ptp_clock_is_never_trusted() {
        // A follower that synced to a leader whose clock was never set.
        let mut inputs = follower(Some(0.1));
        inputs.ptp_time_secs = 12.5;
        assert_ne!(phc_trust(&inputs, DEFAULT_MAX_SYNC_AGE), PhcTrust::Trusted);
    }

    #[test]
    fn unconfigured_node_is_not_trusted() {
        let inputs = PhcTrustInputs { ptp_time_secs: NOW, ..Default::default() };
        assert_ne!(phc_trust(&inputs, DEFAULT_MAX_SYNC_AGE), PhcTrust::Trusted);
    }
}
