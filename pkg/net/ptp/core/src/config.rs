use ptp_proto::ptp::{TimeAdjustmentConfig, TimeSyncConfig};


// NOTE: the max PPM adjustment allowed by linux is +/- 500ppm
// TODO: Do a much better job of tuning all these parameters.
const DEFAULT_CONFIG_TEMPLATE_PROTO: &'static str = r#"
    leader {
        sync_interval_seconds: 0.25
        followers: []
        ptp_timeout_seconds: 0.05
        sync_timeout_seconds: 0.1
    }

    follower {
        leader_clock_sync {
            p_scale: 0.5
            i_scale: 0.05

            # The set_offset feature of the BCM chip is not good and only gets us within 5ms of the right time.
            max_offset_seconds: 0.01 # 0.0001 # 100us
            max_error_integral_secs: 0.05

            # Must be larger than the leader -> ntp one
            max_correction_ppm: 100
        }
        # A ceiling for faults only; 'max_rtt_above_min' does the filtering. The RTT with
        # no queueing depends on the link speed and the switches (28 us between two
        # 100 Mbit/s cameras on one switch), and one full-size frame queued ahead at
        # 100 Mbit/s adds 123 us. A round above this limit is an RPC error.
        max_network_rtt: 0.001 # 1ms
        max_sample_age: 0.1

        # Recorded on two 100 Mbit/s cameras with 20 Mbit/s of other traffic: 0.5 us kept
        # 83% of the rounds with a worst error of 92 ns; 2 us let rounds with 0.8 us through.
        max_rtt_above_min: 0.0000005 # 0.5us
        rtt_min_window: 64 # 16 s of rounds
    }

    basic_client {
        server_addr: ""
        sync_interval_seconds: 0.5

        clock_sync {
            p_scale: 0.5
            i_scale: 0.05
            max_offset_seconds: 1
            max_error_integral_secs: 0.01
            max_correction_ppm: 50 # Must be > crystal ppm
            max_frequency_step_ppm: 0.001
        }

        max_network_rtt: 0.005 # 5ms
    }
    
    realtime_clock_sync {
        p_scale: 0 # 0.01
        i_scale: 0 # 0.0001
        max_offset_seconds: 1
        max_error_integral_secs: 10
        max_correction_ppm: 50
    }

"#;

// Steering of a node's system clock (CLOCK_REALTIME) to its PTP clock, once a second.
const DEFAULT_SYSTEM_CLOCK_SYNC_PROTO: &'static str = r#"
    # The gains of phc2sys's defaults for a 1 s interval.
    p_scale: 0.7
    i_scale: 0.3

    # Steps of CLOCK_REALTIME are exact, so a larger error is stepped, not slewed.
    max_offset_seconds: 0.001
    max_error_integral_secs: 0.01

    # Above the follower's 100 ppm, so the system clock keeps up while the PTP clock slews.
    max_correction_ppm: 200
"#;

lazy_static! {
    static ref DEFAULT_CONFIG_TEMPLATE: TimeSyncConfig = {
        let mut ptp_config = TimeSyncConfig::default();
        protobuf::text::parse_text_proto(DEFAULT_CONFIG_TEMPLATE_PROTO, &mut ptp_config).unwrap();
        ptp_config
    };
}


pub fn default_config_template() -> TimeSyncConfig {
    DEFAULT_CONFIG_TEMPLATE.clone()
}

lazy_static! {
    static ref DEFAULT_SYSTEM_CLOCK_SYNC: TimeAdjustmentConfig = {
        let mut config = TimeAdjustmentConfig::default();
        protobuf::text::parse_text_proto(DEFAULT_SYSTEM_CLOCK_SYNC_PROTO, &mut config).unwrap();
        config
    };
}

/// Default servo settings for TimeSyncNode::set_system_clock_sync().
pub fn default_system_clock_sync() -> TimeAdjustmentConfig {
    DEFAULT_SYSTEM_CLOCK_SYNC.clone()
}
