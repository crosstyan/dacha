use std::sync::Arc;
use std::time::{Duration, Instant};
use std::collections::{HashMap, HashSet};

use common::errors::*;
use executor_multitask::{impl_resource_passthrough, ServiceResourceGroup};
use ptp_proto::ptp::*;
use net::ip::SocketAddr;
use executor::{lock_async, lock};
use executor::sync::AsyncMutex;
use crypto::random::SharedRng;
use cluster_client::{ClusterMetaClient, ServiceResolver};
use protobuf::Message;
use http::Resolver;
use executor::sync::AsyncVariable;
use ptp_core::{BasicTimeNode, MinRttFilter, PIFilterState, RttVerdict, PhcTrust, PhcTrustInputs};

use crate::signed_duration::*;
use crate::socket::*;
use crate::device::*;

const ID_SIZE: usize = 8;

/// How often the system clock is compared with the PTP clock.
const SYSTEM_CLOCK_INTERVAL: Duration = Duration::from_secs(1);

/// A comparison of the system clock and the PTP clock whose read window is longer than this
/// (the thread was preempted between the reads) is not used: the window bounds the error at
/// half its length. On an RV1126B (stmmac, offset_by_reads) the window is about 5.5 us.
const MAX_SYSTEM_CLOCK_READ_DELAY: Duration = Duration::from_micros(20);

/// CLOCK_REALTIME advances by tick us per USER_HZ tick; USER_HZ is 100 on arm, arm64 and x86,
/// so the nominal tick is 10000. linuxptp (ptp4l -S, phc2sys) moves large corrections into the
/// tick and leaves them there when it exits: 9997 (-300 ppm) was left on a camera. The
/// frequency set here is only correct on top of the nominal tick, so it is set with it.
const NOMINAL_TICK_USEC: i64 = 10_000;



pub struct TimeSyncNode {
    resources: ServiceResourceGroup,
    shared: Arc<Shared>
}

impl_resource_passthrough!(TimeSyncNode, resources);

struct Shared {
    // meta_client: Arc<ClusterMetaClient>,
    ptp_device: Arc<PTPDevice>,
    ptp_socket: TimestampedUdpSocket,
    basic_node: Arc<BasicTimeNode>,
    state: AsyncVariable<State>,
}

#[derive(Default)]
struct State {
    config: TimeSyncConfig,

    follower_state: Option<FollowerState>,

    basic_client_state: Option<FollowerState>,

    /// Present while the system clock is steered to the PTP clock. Local to this node: not part
    /// of the TimeSyncConfig set over RPC.
    system_clock: Option<SystemClockState>,

    /// Leader only: the followers and how the rounds with them went, as of the last round.
    leader_peers: Vec<(TimeSyncPeerAddr, LeaderPeerHealth)>,

    // TODO: Probably keep one record per peer address so that we gracefully support leadership changes.
    last_rx: Option<ReceivedPacket>,
    last_tx: Option<SentPacket>,
}

#[derive(Default)]
struct FollowerState {
    pi_filter: PIFilterState,
    last_sync: Option<LastSyncMetadata>,
    rtt_filter: Option<MinRttFilter>,
    num_queued_rounds: u64,
    ignored: IgnoredRounds,
}

/// PTP packets a follower ignored because they did not come from its leader.
#[derive(Default)]
struct IgnoredRounds {
    count: u64,
    last_peer: Option<SocketAddr>,
}

impl IgnoredRounds {
    /// Records one ignored packet from 'peer'. Returns whether to log it: the first one from
    /// a peer (since the last ignored packet came from another), then every 100th, so that a
    /// leader whose packets all leave from an unexpected address shows up in the log without
    /// flooding it.
    fn record(&mut self, peer: &SocketAddr) -> bool {
        self.count += 1;
        let new_peer = self.last_peer.as_ref() != Some(peer);
        self.last_peer = Some(peer.clone());
        new_peer || self.count % 100 == 0
    }
}

struct SystemClockState {
    config: TimeAdjustmentConfig,
    pi_filter: PIFilterState,
    steering: bool,
    reason: String,
    last_sample: Option<SystemClockSample>,
    freq_ppm: f64,
    num_steps: u64,
}

struct SystemClockSample {
    time: Instant,
    error: f64,
    read_delay: f64,
}

struct LastSyncMetadata {
    time: Instant,
    rtt: f64,
    error: f64,
}

struct ReceivedPacket {
    time: Instant,
    peer_addr: SocketAddr,
    rx_id: Vec<u8>,
    rx_time: u64,
}

struct SentPacket {
    tx_id: Vec<u8>,
    tx_time: u64,
}

#[derive(Default)]
struct LeaderState {
    last_round: Option<Instant>,
    
    // The key is the serialized protos.
    followers: HashMap<Vec<u8>, LeaderPeer>,
}

#[derive(Default)]
struct BasicClientState {
    last_round: Option<Instant>,
}

#[derive(Default, Clone)]
struct LeaderPeerHealth {
    last_round: Option<Instant>,
    num_failures: u64,
    num_consecutive_failures: u64,
    last_error: String,
}

struct LeaderPeer {
    proto: TimeSyncPeerAddr,
    health: LeaderPeerHealth,
    configured: bool,
    rpc: Arc<TimeSyncStub>,
    ptp_addr: net::ip::SocketAddr,
    // ptp: ServiceResolver
}

impl TimeSyncNode {
    pub fn default_config_template() -> TimeSyncConfig {
        ptp_core::default_config_template()
    }

    pub async fn create(
        // meta_client: Arc<ClusterMetaClient>,
        ptp_device: Arc<PTPDevice>,
        ptp_socket: TimestampedUdpSocket,
        basic_node: Arc<BasicTimeNode>,
    ) -> Self {

        let shared = Arc::new(Shared {
            // meta_client,
            ptp_device,
            ptp_socket,
            basic_node: basic_node.clone(),
            state: AsyncVariable::default()
        });

        let mut resources = ServiceResourceGroup::new("TimeSyncNode");

        resources.register_dependency(basic_node.clone()).await;

        resources.spawn_interruptable(
            "TimeSyncNode::leader_thread", Self::leader_thread(shared.clone())
        ).await;
        resources.spawn_interruptable(
            "TimeSyncNode::basic_client_thread", Self::basic_client_thread(shared.clone())
        ).await;
        resources.spawn_interruptable(
            "TimeSyncNode::server_thread", Self::server_thread(shared.clone())
        ).await;
        resources.spawn_interruptable(
            "TimeSyncNode::system_clock_thread", Self::system_clock_thread(shared.clone())
        ).await;

        Self {
            shared,
            resources,
        }
    }

    pub async fn status(&self) -> Result<StatusResponse> {
        lock!(state <= self.shared.state.lock().await?, {
            let mut out = StatusResponse::default();
            out.set_config(state.config.clone());
            if let Some(follower_state) = &state.follower_state {
                let proto = out.follower_mut();
                
                if let Some(last_sync) = &follower_state.last_sync {
                    proto.set_got_sync(true);
                    proto.set_last_leader_error(last_sync.error);
                    proto.set_last_leader_rtt(last_sync.rtt);
                    proto.set_last_sync_age((Instant::now() - last_sync.time).as_secs_f64());
                }

                if let Some(min_rtt) = follower_state.rtt_filter.as_ref().and_then(|f| f.min()) {
                    proto.set_min_rtt(min_rtt as f64 * 1e-9);
                }
                proto.set_num_queued_rounds(follower_state.num_queued_rounds);
                proto.set_num_ignored_rounds(follower_state.ignored.count);
                if let Some(peer) = &follower_state.ignored.last_peer {
                    proto.set_last_ignored_peer(peer.to_string());
                }
            }

            // TODO: Dedup this with above.
            if let Some(follower_state) = &state.basic_client_state {
                let proto = out.basic_client_mut();
                
                if let Some(last_sync) = &follower_state.last_sync {
                    proto.set_got_sync(true);
                    proto.set_last_leader_error(last_sync.error);
                    proto.set_last_leader_rtt(last_sync.rtt);
                    proto.set_last_sync_age((Instant::now() - last_sync.time).as_secs_f64());
                }
            }

            if state.config.has_leader() {
                let proto = out.leader_mut();
                let now = Instant::now();
                for (addr, health) in &state.leader_peers {
                    let f = proto.new_followers();
                    f.set_rpc_addr(addr.rpc_addr());
                    f.set_ptp_addr(addr.ptp_addr());
                    if let Some(t) = &health.last_round {
                        f.set_got_round(true);
                        f.set_last_round_age((now - *t).as_secs_f64());
                    }
                    f.set_num_failures(health.num_failures);
                    f.set_num_consecutive_failures(health.num_consecutive_failures);
                    f.set_last_error(health.last_error.clone());
                }
            }

            if let Some(system_clock) = &state.system_clock {
                let proto = out.system_clock_mut();
                proto.set_steering(system_clock.steering);
                proto.set_reason(system_clock.reason.clone());
                proto.set_freq_ppm(system_clock.freq_ppm);
                proto.set_num_steps(system_clock.num_steps);
                if let Some(sample) = &system_clock.last_sample {
                    proto.set_last_error(sample.error);
                    proto.set_last_read_delay(sample.read_delay);
                    proto.set_last_sample_age((Instant::now() - sample.time).as_secs_f64());
                }
            }

            Ok(out)
        })
    }

    /// Enables or disables steering of the system clock (CLOCK_REALTIME) to the PTP clock, like
    /// phc2sys does next to ptp4l. It is only steered while the PTP clock follows a source (see
    /// ptp_core::phc_trust()). Disabling it leaves the system clock running at its last
    /// frequency.
    ///
    /// Nothing else may adjust the system clock meanwhile (no NTP daemon, no phc2sys).
    pub async fn set_system_clock_sync(&self, config: Option<TimeAdjustmentConfig>) -> Result<()> {
        lock!(state <= self.shared.state.lock().await?, {
            state.system_clock = config.map(|config| SystemClockState {
                config,
                pi_filter: PIFilterState::default(),
                steering: false,
                reason: "starting".to_string(),
                last_sample: None,
                freq_ppm: 0.0,
                num_steps: 0,
            });
        });

        Ok(())
    }

    pub async fn configure(&self, config: &TimeSyncConfig) -> Result<()> {
        let phc_freq_ppm = self.shared.ptp_device.clock().get_adjustments()?.freq() as f64 / 65536.0;

        lock!(state <= self.shared.state.lock().await?, {
            let old_config = std::mem::replace(&mut state.config, config.clone());

            if !config.has_leader() {
                state.leader_peers.clear();
            }

            state.follower_state = reconfigure_follower(&old_config, config, state.follower_state.take());
            state.basic_client_state = reconfigure_basic_client(
                &old_config, config, state.basic_client_state.take(), phc_freq_ppm);

            Ok(())
        })
    }

    pub async fn sync(&self, point: &TimeSyncPoint) -> Result<()> {
        Self::sync_impl(&self.shared, point).await
    }

    async fn sync_impl(shared: &Shared, remote_point: &TimeSyncPoint) -> Result<()> {
        lock!(state <= shared.state.lock().await?, {
            let state = &mut *state;

            let follower_state = state.follower_state.as_mut()
                .ok_or_else(|| err_msg("Not currently a follower"))?;

            let local_rx = state.last_rx.take()
                .ok_or_else(|| err_msg("No PTP packet received recently"))?;
            if !ptp_core::is_from_leader(state.config.follower(), &local_rx.peer_addr) {
                return Err(format_err!(
                    "PTP round from {:?}, which is not the leader {}",
                    local_rx.peer_addr, state.config.follower().leader_ptp_addr()
                ));
            }
            if &local_rx.rx_id != remote_point.tx_id() {
                return Err(err_msg("Local RX / remote TX mismatch"));
            }

            let local_tx = state.last_tx.take()
                .ok_or_else(|| err_msg("No PTP packet send recently"))?;
            if &local_tx.tx_id != remote_point.rx_id() {
                return Err(err_msg("Local TX / remote RX mismatch"));
            }

            let rtt = ptp_core::round_trip_time(
                remote_point.tx_time(), remote_point.rx_time(), local_rx.rx_time, local_tx.tx_time
            ).ok_or_else(|| err_msg("Negative PTP RTT: a clock was stepped during the round"))?;

            let follower_config = state.config.follower();
            let verdict = follower_state.rtt_filter
                .get_or_insert_with(|| MinRttFilter::new(follower_config.rtt_min_window() as usize))
                .check_from(&local_rx.peer_addr.to_string(), follower_config, rtt);

            let rtt = Duration::from_nanos(rtt);

            match verdict {
                RttVerdict::Usable => {}
                RttVerdict::TooLong => {
                    return Err(format_err!("Sync RTT is too long: {:?}", rtt));
                }
                RttVerdict::Queued => {
                    // The packets waited in a queue somewhere on the path. This is normal
                    // through a switch that carries other traffic, so the round is only
                    // counted.
                    follower_state.num_queued_rounds += 1;
                    return Ok(());
                }
            }

            let mut local_time = Duration::from_nanos(local_rx.rx_time);
            let mut remote_time = Duration::from_nanos(remote_point.tx_time()) + (rtt / 2);

            // Advance the times to the current point in time.
            // TODO: Base this on the PTP clock?
            {
                let now: Duration = shared.ptp_device.get_time()?.into();
                let elapsed_time = now - local_time;
                if elapsed_time > Duration::from_secs_f32(state.config.follower().max_sample_age()) {
                    return Err(format_err!("Sync sample age ({:?}) is too old", elapsed_time));
                }

                // local_time += elapsed_time;
                // remote_time += elapsed_time;
            }

            let error = SignedDuration::from(remote_time) - SignedDuration::from(local_time);

            // println!("Sync Error: {:?}; RTT: {:?}", error, rtt);

            Self::run_time_adjustment(
                shared,
                remote_time,
                error.as_secs_f64(),
                state.config.follower().leader_clock_sync(),
                &mut follower_state.pi_filter
            )?;

            follower_state.last_sync = Some(LastSyncMetadata {
                time: local_rx.time,
                error: error.as_secs_f64(),
                rtt: rtt.as_secs_f64()
            });

            Ok(())
        })
    }

    // The leader and the basic client run in separate threads: a time server that does not
    // answer costs the basic client a 1 s ping timeout and a 1 s pause per cycle, and in a
    // shared loop that slowed the leader's rounds from 4/s to one every 2 s.
    async fn leader_thread(shared: Arc<Shared>) -> Result<()> {
        let mut leader_state = None;

        loop {
            let config = lock!(state <= shared.state.lock().await?, {
                state.config.clone()
            });

            if config.has_leader() {
                let state = leader_state.get_or_insert_default();
                if let Err(e) = Self::run_leader_cycle(&shared, &config, state).await {
                    eprintln!("TimeSyncNode::run_leader_cycle failed: {}", e);
                    executor::sleep(Duration::from_millis(1000)).await?;
                }

            } else {
                leader_state = None;
            }

            // TODO: Add random jitter to this.
            executor::sleep(Duration::from_millis(100)).await?;
        }
    }

    async fn basic_client_thread(shared: Arc<Shared>) -> Result<()> {
        let mut basic_client_state = None;

        loop {
            let config = lock!(state <= shared.state.lock().await?, {
                state.config.clone()
            });

            if config.has_basic_client() {
                let state = basic_client_state.get_or_insert_default();
                if let Err(e) = Self::run_basic_client_cycle(&shared, &config, state).await {
                    eprintln!("TimeSyncNode::run_basic_client_cycle failed: {}", e);
                    executor::sleep(Duration::from_millis(1000)).await?;
                }

            } else {
                basic_client_state = None;
            }

            // TODO: Add random jitter to this.
            executor::sleep(Duration::from_millis(100)).await?;
        }
    }

    async fn system_clock_thread(shared: Arc<Shared>) -> Result<()> {
        loop {
            executor::sleep(SYSTEM_CLOCK_INTERVAL).await?;

            lock!(state <= shared.state.lock().await?, {
                if let Err(e) = Self::run_system_clock_cycle(&shared, &mut *state) {
                    eprintln!("TimeSyncNode::run_system_clock_cycle failed: {}", e);
                    if let Some(system_clock) = state.system_clock.as_mut() {
                        system_clock.steering = false;
                        system_clock.reason = e.to_string();
                        system_clock.pi_filter.hold();
                    }
                }
            });
        }
    }

    fn run_system_clock_cycle(shared: &Shared, state: &mut State) -> Result<()> {
        let now = Instant::now();

        let sync_age = |follower: &Option<FollowerState>| {
            follower.as_ref()
                .and_then(|f| f.last_sync.as_ref())
                .map(|s| (now - s.time).as_secs_f64())
        };

        if state.system_clock.is_none() {
            return Ok(());
        }

        let ptp_time: Duration = shared.ptp_device.get_time()?.into();

        let inputs = PhcTrustInputs {
            is_leader: state.config.has_leader(),
            is_follower: state.config.has_follower(),
            has_basic_client: state.config.has_basic_client(),
            follower_last_sync_age: sync_age(&state.follower_state),
            basic_client_last_sync_age: sync_age(&state.basic_client_state),
            ptp_time_secs: ptp_time.as_secs_f64(),
        };

        let system_clock = match state.system_clock.as_mut() {
            Some(v) => v,
            None => return Ok(()),
        };

        if let PhcTrust::Untrusted(reason) = ptp_core::phc_trust(&inputs, ptp_core::DEFAULT_MAX_SYNC_AGE) {
            system_clock.steering = false;
            system_clock.reason = reason.to_string();
            system_clock.pi_filter.hold();
            return Ok(());
        }

        let offset = shared.ptp_device.offset_to(sys::ClockId::REALTIME)?;
        if offset.rtt > MAX_SYSTEM_CLOCK_READ_DELAY {
            return Err(format_err!("Reading the system and PTP clocks took {:?}", offset.rtt));
        }

        let error = (SignedDuration::from(offset.ptp_time) - SignedDuration::from(offset.other_time))
            .as_secs_f64();

        let correction = system_clock.pi_filter.update(&system_clock.config, error, now);

        if let Some(step) = correction.step_seconds {
            sys::ClockId::REALTIME.set_offset_secs_f64(step)?;
            system_clock.num_steps += 1;
        }

        let mut adj = sys::ClockId::REALTIME.get_adjustments()?;
        adj.set_tick(NOMINAL_TICK_USEC);
        adj.set_freq((correction.freq_ppm * ((1 << 16) as f64)).round() as i64);
        sys::ClockId::REALTIME.set_adjustments(&adj)?;

        system_clock.steering = true;
        system_clock.reason.clear();
        system_clock.freq_ppm = correction.freq_ppm;
        system_clock.last_sample = Some(SystemClockSample {
            time: now,
            error,
            read_delay: offset.rtt.as_secs_f64(),
        });

        Ok(())
    }

    async fn run_leader_cycle(
        shared: &Arc<Shared>,
        config: &TimeSyncConfig,
        leader_state: &mut LeaderState
    ) -> Result<()> {

        let now = Instant::now();

        if let Some(last_time) = &leader_state.last_round {
            if (now - *last_time) < Duration::from_secs_f32(config.leader().sync_interval_seconds()) {
                return Ok(());
            }
        }
        leader_state.last_round = Some(now);

        Self::run_leader_all_followers_sync(shared, config, leader_state).await?;

        Ok(())
    }

    async fn run_basic_client_cycle(
        shared: &Arc<Shared>,
        config: &TimeSyncConfig,
        basic_state: &mut BasicClientState
    ) -> Result<()> {
        let now = Instant::now();

        if let Some(last_time) = &basic_state.last_round {
            if (now - *last_time) < Duration::from_secs_f32(config.basic_client().sync_interval_seconds()) {
                return Ok(());
            }
        }
        basic_state.last_round = Some(now);

        let server_addr = config.basic_client().server_addr().parse()?;
        
        let start_ptp_time: Duration = shared.ptp_device.get_time()?.into();

        let remote_time = Duration::from_nanos(executor::timeout(
            Duration::from_secs_f32(1.0),
            shared.basic_node.ping(&server_addr),
        ).await??);

        let end_ptp_time: Duration = shared.ptp_device.get_time()?.into();

        let rtt = end_ptp_time - start_ptp_time;
        if rtt > Duration::from_secs_f32(config.basic_client().max_network_rtt()) {
            return Err(format_err!("Sync RTT is too large: {:?}", rtt));
        }

        let local_time = start_ptp_time + rtt / 2;
        
        lock!(state <= shared.state.lock().await?, {

            let state = &mut *state;

            let follower_state = state.basic_client_state.as_mut()
                .ok_or_else(|| err_msg("Not currently a basic client"))?;

            let error = SignedDuration::from(remote_time) - SignedDuration::from(local_time);

            Self::run_time_adjustment(
                shared,
                remote_time,
                error.as_secs_f64(),
                state.config.basic_client().clock_sync(),
                &mut follower_state.pi_filter
            )?;

            follower_state.last_sync = Some(LastSyncMetadata {
                time: now,
                error: error.as_secs_f64(),
                rtt: rtt.as_secs_f64()
            });

            Ok(())
        })
    }

    /*
    async fn run_leader_ntp_sync(
        shared: &Shared,
        config: &TimeSyncConfig,
        leader_state: &mut LeaderState
    ) -> Result<()> {
        // TODO: Use a better syscall for capturing both of these times simultaneously.
        let ptp_time_raw = shared.ptp_device.get_time()?;
        let ptp_time: Duration = ptp_time_raw.into();
        let real_time_raw = sys::ClockId::REALTIME.get_time()?;
        let real_time: Duration = real_time_raw.into();

        let error = SignedDuration::from(real_time) - SignedDuration::from(ptp_time);

        // println!("NTP Error: {:?}", error);

        Self::run_time_adjustment(
            shared,
            real_time,
            error.as_secs_f64(),
            config.realtime_clock_sync(),
            &mut leader_state.pi_filter
        )?;

        Ok(())
    }
    */

    fn run_time_adjustment(
        shared: &Shared,
        remote_time: Duration,
        error: f64,
        config: &TimeAdjustmentConfig,
        state: &mut PIFilterState,
    ) -> Result<()> {
        let correction = state.update(config, error, Instant::now());

        if let Some(step) = correction.step_seconds {
            shared.ptp_device.clock().set_offset_secs_f64(step)?;
            // println!("=> time shifted!");
        }

        // println!("=> freq: {:.2}ppm", correction.freq_ppm);

        let out = correction.freq_ppm * ((1 << 16) as f64);

        let mut adj = shared.ptp_device.clock().get_adjustments()?;
        adj.set_freq(out.round() as i64);
        shared.ptp_device.clock().set_adjustments(&adj)?;

        Ok(())
    }

    async fn run_leader_all_followers_sync(
        shared: &Arc<Shared>,
        config: &TimeSyncConfig,
        leader_state: &mut LeaderState
    ) -> Result<()> {
        // Get rid of any old follower entries.
        {
            let mut follower_set = HashSet::<Vec<u8>>::new();
            for follower in config.leader().followers() {
                follower_set.insert(follower.serialize()?);
            }

            let mut old_followers = vec![];

            for key in leader_state.followers.keys() {
                if !follower_set.contains(key) {
                    old_followers.push(key.clone());
                }
            }

            for key in old_followers  {
                leader_state.followers.remove(&key);
            }
        }

        // Add new follower entries.

        for follower in config.leader().followers() {
            let follower_key = follower.serialize()?;
            if leader_state.followers.contains_key(&follower_key) {
                continue;
            }

            /*
            let channel = cluster_client::service::create_rpc_channel(
                &follower.rpc_addr(),
                shared.meta_client.clone()
            ).await?;
            */

            let channel = {
                Arc::new(
                    rpc::Http2Channel::create(format!("http://{}", follower.rpc_addr()).as_str())
                        .await?,
                )
            };

            let rpc = Arc::new(TimeSyncStub::new(channel));

            /*
            let ptp = cluster_client::ServiceResolver::create(
                follower.ptp_addr(), shared.meta_client.clone()
            )?;
            */

            leader_state.followers.insert(follower_key, LeaderPeer {
                proto: follower.as_ref().clone(),
                health: LeaderPeerHealth::default(),
                configured: false,
                rpc,
                ptp_addr: follower.ptp_addr().parse()?
            });
        }

        // Sync to all peers.
        // - Individual peer failures shouldn't take down the whole leader.
        // - These can't be parallelized since that would risk increasing network queueing and thus RTT

        for follower in leader_state.followers.values_mut() {
            match Self::run_leader_one_follower_sync(shared, config, follower).await {
                Ok(()) => {
                    follower.health.last_round = Some(Instant::now());
                    follower.health.num_consecutive_failures = 0;
                }
                Err(e) => {
                    eprintln!("Failed to sync to: {}: {}", follower.proto.rpc_addr(), e);
                    follower.health.num_failures += 1;
                    follower.health.num_consecutive_failures += 1;
                    follower.health.last_error = e.to_string();
                }
            }
        }

        let mut peers = leader_state.followers.values()
            .map(|f| (f.proto.clone(), f.health.clone()))
            .collect::<Vec<_>>();
        peers.sort_by(|a, b| a.0.rpc_addr().cmp(b.0.rpc_addr()));

        lock!(state <= shared.state.lock().await?, {
            state.leader_peers = peers;
        });

        Ok(())
    }

    async fn run_leader_one_follower_sync(
        shared: &Arc<Shared>,
        config: &TimeSyncConfig,
        follower: &mut LeaderPeer
    ) -> Result<()> {
        let request_context = rpc::ClientRequestContext::default();

        // NOTE: This is done in the mocap manager so we don't bother doing it.
        /*
        if !follower.configured {
            let mut req = ConfigureRequest::default();
            req.set_config(config.clone());
            req.config_mut().set_role(TimeSyncRole::FOLLOWER);

            let _ = executor::timeout(
                Duration::from_secs_f32(config.leader().sync_timeout_seconds()),
                follower.rpc.Configure(&request_context, &req)
            )
            .await
            .map_err(|_| err_msg("Timed out on Configure RPC"))?
            .result?;

            follower.configured = true;

            println!("Configured!");
        }
        */

        /*
        let ptp_endpoints = follower.ptp.resolve().await?;
        if ptp_endpoints.is_empty() {
            return Err(err_msg("No PTP addresses resolved"));
        }

        if ptp_endpoints.len() != 1 {
            return Err(err_msg("More than 1 PTP address resolved"));
        }

        let ptp_addr = &ptp_endpoints[0].address;
        */
        let ptp_addr = &follower.ptp_addr;

        // println!("Sending to: {:?}", ptp_addr);

        // Clear any old data so that 'Self::get_received_packet' only notices newly
        // received data.
        lock!(state <= shared.state.lock().await?, {
            state.last_tx = None;
            state.last_rx = None;
        });

        let mut tx_id = vec![0u8; ID_SIZE];
        crypto::random::global_rng().generate_bytes(&mut tx_id).await;

        let tx_time = shared.ptp_socket.send_to(&tx_id, ptp_addr).await?;

        let rx_packet = executor::timeout(
            Duration::from_secs_f32(config.leader().ptp_timeout_seconds()),
            Self::get_received_packet(shared.clone())
        )
        .await
        .map_err(|_| err_msg("Timed out while receiving PTP response"))??;

        if rx_packet.peer_addr != *ptp_addr {
            return Err(format_err!("Received PTP response from wrong peer: {:?} vs {:?}",
                rx_packet.peer_addr, ptp_addr));
        }

        let mut point = TimeSyncPoint::default();
        point.set_tx_id(tx_id);
        point.set_tx_time(tx_time);
        point.set_rx_id(rx_packet.rx_id);
        point.set_rx_time(rx_packet.rx_time);


        let _ = executor::timeout(
            Duration::from_secs_f32(config.leader().sync_timeout_seconds()),
            follower.rpc.Sync(&request_context, &point)
        )
        .await
        .map_err(|_| err_msg("Timed out on Sync RPC"))?
        .result?;

        Ok(())
    }

    async fn get_received_packet(shared: Arc<Shared>) -> Result<ReceivedPacket> {
        loop {
            let mut state = shared.state.lock().await?.enter();
            if let Some(pkt) = state.last_rx.take() {
                state.exit();
                return Ok(pkt);
            }

            state.wait().await;
        }
    }

    // Receives PTP packets from peers and stores them in last_rx
    // If we are a follower, this also handles responding to the packets and stores that data in last_tx
    //
    // NOTE: the ptp_socket binds to the ethernet device so all packets sent/received should have
    // timestamps if the local machine is healthy.
    async fn server_thread(shared: Arc<Shared>) -> Result<()> {

        let mut buf = [0u8; 32];
        
        loop {
            // TODO: Don't error out if an invalid packet type is received
            let (n_received, rx_time, peer_addr) = shared.ptp_socket.recv_from(&mut buf).await?;
            if n_received != ID_SIZE {
                continue;
            }
            

            let time = Instant::now();

            let rx_id = buf[0..n_received].to_vec();

            // println!("RX: {:?}: {:?}", peer_addr, rx_id);

            // Locking before the transfer to ensure that the last_rx state is updated before the state
            // is visible to sync().
            lock_async!(state <= shared.state.lock().await?, {
                // Drop extra packets until the current one is acknowledged.
                // NOTE: we do need to allow overwriting old packets since a leader may fail before the sync RPC is called.
                // if state.last_rx.is_some() {
                //     return Ok(());
                // }

                // A follower answers only its own leader (when the manager named one), so a
                // replaced leader that still runs can not take rounds away from the new one.
                if state.config.has_follower()
                    && !ptp_core::is_from_leader(state.config.follower(), &peer_addr)
                {
                    let leader = state.config.follower().leader_ptp_addr().to_string();
                    if let Some(follower_state) = state.follower_state.as_mut() {
                        if follower_state.ignored.record(&peer_addr) {
                            eprintln!(
                                "TimeSyncNode: ignored a PTP packet from {} (leader: {:?}); {} ignored so far",
                                peer_addr.to_string(),
                                leader,
                                follower_state.ignored.count
                            );
                        }
                    }
                    return Result::<_, Error>::Ok(());
                }

                state.last_rx = Some(ReceivedPacket {
                    time,
                    peer_addr: peer_addr.clone(),
                    rx_id,
                    rx_time,
                });
                
                if state.config.has_follower() {
                    let mut tx_id = vec![0u8; ID_SIZE];
                    crypto::random::global_rng().generate_bytes(&mut tx_id).await;

                    let tx_time = shared.ptp_socket.send_to(&tx_id, &peer_addr).await?;

                    state.last_tx = Some(SentPacket {
                        tx_id,
                        tx_time,
                    });
                } else {
                    state.last_tx = None;
                }

                state.notify_all();

                Result::<_, Error>::Ok(())
            })?;
        }
    }
}

/// The follower state after a reconfiguration from 'old' to 'new'.
///
/// A node that stays a follower keeps its PI filter, which holds the frequency it learned:
/// the manager reconfigures every follower when the leader changes, and the new leader was
/// locked to the old one. Only the state that belongs to the old leader's rounds (its path's
/// minimum RTT, its last sync) is dropped.
fn reconfigure_follower(
    old: &TimeSyncConfig,
    new: &TimeSyncConfig,
    state: Option<FollowerState>,
) -> Option<FollowerState> {
    match (new.has_follower(), state) {
        (false, _) => None,
        (true, Some(mut follower_state)) if old.has_follower() => {
            if old.follower() != new.follower() {
                follower_state.rtt_filter = None;
                follower_state.last_sync = None;
                follower_state.pi_filter.hold();
            }
            Some(follower_state)
        }
        (true, _) => Some(FollowerState::default()),
    }
}

/// The basic client state after a reconfiguration: kept while the basic client config is
/// unchanged apart from its server address, since its step-limited servo needs a long time to
/// learn its frequency again and the leader's config changes with every camera that comes or
/// goes. The server address alone changes with every manager start (its time server binds a
/// random port).
///
/// A new state starts from 'phc_freq_ppm', the frequency the PTP clock runs at now: a follower
/// promoted to leader was locked to the old leader, and the followers keep that frequency.
fn reconfigure_basic_client(
    old: &TimeSyncConfig,
    new: &TimeSyncConfig,
    state: Option<FollowerState>,
    phc_freq_ppm: f64,
) -> Option<FollowerState> {
    let without_server = |config: &TimeSyncConfig| {
        let mut basic_client = config.basic_client().clone();
        basic_client.set_server_addr("");
        basic_client
    };

    match (new.has_basic_client(), state) {
        (false, _) => None,
        (true, Some(basic_state))
            if old.has_basic_client() && without_server(old) == without_server(new) =>
        {
            Some(basic_state)
        }
        (true, _) => Some(FollowerState {
            pi_filter: PIFilterState::starting_at(new.basic_client().clock_sync(), phc_freq_ppm),
            ..Default::default()
        }),
    }
}

#[async_trait]
impl TimeSyncService for TimeSyncNode {

    async fn Status(
        &self,
        request: rpc::ServerRequest<StatusRequest>,
        response: &mut rpc::ServerResponse<StatusResponse>
    ) -> Result<()> {
        response.value = self.status().await?;
        Ok(())
    }

    async fn Configure(
        &self,
        request: rpc::ServerRequest<ConfigureRequest>,
        response: &mut rpc::ServerResponse<ConfigureResponse>
    ) -> Result<()> {
        self.configure(request.config()).await?;
        Ok(())
    }

    async fn Sync(
        &self,
        request: rpc::ServerRequest<TimeSyncPoint>,
        response: &mut rpc::ServerResponse<SyncResponse>
    ) -> Result<()> {
        self.sync(&request.value).await?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn follower_of(leader: &str) -> TimeSyncConfig {
        let mut config = TimeSyncConfig::default();
        config.set_follower(ptp_core::default_config_template().follower().clone());
        config.follower_mut().set_leader_ptp_addr(leader);
        config
    }

    fn used_state() -> FollowerState {
        let mut state = FollowerState::default();
        state.num_queued_rounds = 7;
        state.rtt_filter = Some(MinRttFilter::new(64));
        state.last_sync = Some(LastSyncMetadata { time: Instant::now(), rtt: 28e-6, error: 1e-8 });
        state
    }

    #[test]
    fn follower_keeps_its_state_when_the_leader_changes() {
        let old = follower_of("10.0.0.1:319");
        let new = follower_of("10.0.0.2:319");

        let state = reconfigure_follower(&old, &new, Some(used_state())).unwrap();
        // The same state (its counter survived), without the old leader's path.
        assert_eq!(state.num_queued_rounds, 7);
        assert!(state.rtt_filter.is_none());
        assert!(state.last_sync.is_none());

        // An unchanged config keeps everything.
        let state = reconfigure_follower(&old, &old, Some(used_state())).unwrap();
        assert!(state.rtt_filter.is_some());
        assert!(state.last_sync.is_some());

        // A node that becomes a follower starts fresh; one that stops has none.
        let leader = TimeSyncConfig::default();
        assert_eq!(reconfigure_follower(&leader, &new, Some(used_state())).unwrap().num_queued_rounds, 0);
        assert!(reconfigure_follower(&new, &leader, Some(used_state())).is_none());
    }

    #[test]
    fn follower_does_not_integrate_over_a_leader_change() {
        let old = follower_of("10.0.0.1:319");
        let new = follower_of("10.0.0.2:319");
        let config = new.follower().leader_clock_sync().clone();
        let error = 1e-6;
        let start = Instant::now();

        let mut state = FollowerState::default();
        state.pi_filter.update(&config, error, start);
        let mut state = reconfigure_follower(&old, &new, Some(state)).unwrap();
        let c = state.pi_filter.update(&config, error, start + Duration::from_secs(100));

        // The first round from the new leader adds only the proportional term. Integrating the
        // error over the 100 s gap would add 5 ppm.
        let expected = config.p_scale() * error * 1e6;
        assert!((c.freq_ppm - expected).abs() < 1e-9, "{} ppm instead of {} ppm", c.freq_ppm, expected);
    }

    #[test]
    fn ignored_rounds_are_counted_and_logged_sparingly() {
        let a: SocketAddr = "10.0.0.5:319".parse().unwrap();
        let b: SocketAddr = "10.0.0.6:319".parse().unwrap();
        let mut ignored = IgnoredRounds::default();

        // The first packet from a peer is logged, the next ones only every 100th.
        let logged = (0..250).filter(|_| ignored.record(&a)).count();
        assert_eq!(logged, 3, "logged {} of 250 packets from one peer", logged);
        assert_eq!(ignored.count, 250);

        // A packet from another peer is logged at once.
        assert!(ignored.record(&b));
        assert_eq!(ignored.count, 251);
        assert_eq!(ignored.last_peer, Some(b));
    }

    #[test]
    fn basic_client_keeps_its_state_while_its_config_is_unchanged() {
        let mut old = TimeSyncConfig::default();
        old.set_basic_client(ptp_core::default_config_template().basic_client().clone());
        old.basic_client_mut().set_server_addr("10.0.0.9:400");

        // The leader's follower list changed, the basic client did not.
        let mut new = old.clone();
        new.leader_mut().new_followers().set_ptp_addr("10.0.0.3:319");
        assert_eq!(reconfigure_basic_client(&old, &new, Some(used_state()), 0.0).unwrap().num_queued_rounds, 7);

        // A restarted manager: same PC, its time server on a new random port.
        new.basic_client_mut().set_server_addr("10.0.0.9:4123");
        assert_eq!(reconfigure_basic_client(&old, &new, Some(used_state()), 0.0).unwrap().num_queued_rounds, 7);

        // Other settings changed: a new state, which starts at the PTP clock's frequency.
        new.basic_client_mut().set_sync_interval_seconds(1.0);
        let mut state = reconfigure_basic_client(&old, &new, Some(used_state()), 15.0).unwrap();
        assert_eq!(state.num_queued_rounds, 0);

        let config = new.basic_client().clock_sync();
        let c = state.pi_filter.update(config, 0.0, Instant::now());
        assert!(
            (c.freq_ppm - 15.0).abs() <= config.max_frequency_step_ppm() + 1e-9,
            "the new basic client set the clock to {} ppm instead of keeping 15 ppm",
            c.freq_ppm
        );
    }
}
