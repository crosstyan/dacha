# PTP (Precision Time Protocol) Support Library


## Upgrading: cameras before the manager

The manager builds every camera's TimeSyncConfig from its own copy of
`ptp_core::default_config_template()`, so the follower settings a camera runs
with come from the manager, while the code that reads them is the camera's.

Upgrade the cameras first:

- A new camera with an old manager behaves like an old camera: the old
  template has no `max_rtt_above_min`, `rtt_min_window` or `leader_ptp_addr`, so
  the minimum-RTT filter and the leader check are off, and the old 10 us
  `max_network_rtt` applies (which rejects every round on 100 Mbit/s links).
- An old camera with a new manager is not safe. The new template sets
  `max_network_rtt` to 1 ms, a ceiling meant for faults only, and relies on the
  minimum-RTT filter to drop rounds that queued at a switch. An old camera has
  no such filter, so it would use rounds up to 1 ms long, whose error can be up
  to about 0.5 ms. It ignores `leader_ptp_addr` and lists no followers in its
  status, which is harmless: the manager then judges such a leader by its
  Status RPC only.

## One IPv4 address per camera

A follower takes rounds only from the address in `leader_ptp_addr`, which the
manager builds from the leader's mDNS endpoint and port 319. The leader's PTP
socket is bound to 0.0.0.0:319, so the kernel picks the source address from the
route to each follower. With one IPv4 address per camera (the link-local setup
in `pkg/vision/mocap/doc/networking.md`) the two are the same. A camera with a
second address (a static or DHCP one beside the link-local one) can send from
the other one; its followers then ignore every round. `ptp_status` shows this as
a growing `ignored_rounds` with the address in `last_ignored_peer`, and the
follower logs the first ignored packet and every 100th.

`leader_ptp_addr` does not support IPv6: an IPv6 endpoint gives "ip:319"
without brackets, which does not parse, and a follower with an address that
does not parse ignores every round.


## Unsorted Notes


BCM54210PE
- IEEE 1588v2 compliant
- The ethernet PHY on the CM5 is a BCM54210PE which is attached to a 25Mhz crystal.
- Driver only supports 1PPS
    - https://github.com/raspberrypi/linux/blob/rpi-6.12.y/drivers/net/phy/bcm-phy-ptp.c#L594


'ptp_sys_offset_extended' is the magic for sys to realtime syncing. 

TODO: PTP packet prioritization in the switch

TODO: Ideally coordinate higher level master election.

TODO: PTP currently has no security aside from physical access restrictions.

Ports:
- udp/319, udp/320
- So need to propagate "cap_net_bind_service"




```
# Verifying hardware support
ethtool -T eth0
# => Should should also print "PTP Hardware Clock: 0" which means that corresponds to /dev/ptp0

sudo apt install linuxptp

timedatectl status
# Look for "System clock synchronized: yes"

# -s CLOCK_REALTIME : Source is the Linux system clock (NTP)
# -c eth0           : Destination is the PTP Hardware Clock on eth0
# -O 0              : Offset correction (0 seconds)
sudo phc2sys -s CLOCK_REALTIME -c eth0 -O 0 -m

# -i eth0 : Interface
# -m      : Print messages to stdout
# (No -s flag means it defaults to Master mode if no better clock is found)
sudo ptp4l -i eth0 -m -H --tx_timestamp_timeout 200

## On slave devices

sudo systemctl stop systemd-timesyncd

# -s : Slave-only mode (won't try to become master)
# -H : Hardware timestamping
sudo ptp4l -i eth0 -s -m -H

# -s eth0           : Source is the PTP Hardware Clock
# -c CLOCK_REALTIME : Destination is the System Clock
sudo phc2sys -s eth0 -c CLOCK_REALTIME -O 0 -m


wget https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/plain/tools/testing/selftests/ptp/testptp.c
gcc -Wall -lrt testptp.c -o testptp

sudo ./testptp -d /dev/ptp0 -L0,2
    PTP_PIN_SETFUNC
    - index: 0
    - func: PTP_PF_PEROUT

sudo ./testptp -d /dev/ptp0 -p 1000000000
    PTP_PEROUT_REQUEST2


```


Issues:

```

timed out while polling for tx timestamp

wget https://github.com/jclark/rpi-cm4-ptp-guide/raw/refs/heads/main/files/linuxptp/linuxptp_3.1.1-4jclark1_arm64.deb

sudo dpkg -i linuxptp_3.1.1-4jclark1_arm64.deb

Also need  --tx_timestamp_timeout 200

https://github.com/jclark/rpi-cm4-ptp-guide/issues/40

```