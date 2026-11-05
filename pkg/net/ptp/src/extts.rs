use std::io::Read;
use std::os::fd::AsRawFd;
use std::time::Duration;

use common::errors::*;

use sys::bindings::{ptp_extts_event, ptp_extts_request, PTP_ENABLE_FEATURE, PTP_RISING_EDGE};

use crate::ioctl::*;

const EVENT_SIZE: usize = std::mem::size_of::<ptp_extts_event>();

/// One external timestamp input (EXTTS) of a PTP clock: the clock's time at
/// each edge of a signal wired to the MAC's timestamp pin.
///
/// The channel is enabled while this exists. Since Linux 6.7 every open file
/// of the clock gets its own copy of the events, so this opens its own.
pub struct ExttsChannel {
    file: std::fs::File,
    index: u32,
}

impl ExttsChannel {
    /// Opens the clock at 'path' (e.g. "/dev/ptp0") and enables input 'index'
    /// for rising edges, the only kind stmmac timestamps.
    pub fn open(path: &str, index: u32) -> Result<Self> {
        let file = std::fs::OpenOptions::new().read(true).write(true).open(path)?;

        let inst = Self { file, index };
        // PTP_EXTTS_REQUEST2 refuses PTP_ENABLE_FEATURE without an edge.
        inst.request(PTP_ENABLE_FEATURE | PTP_RISING_EDGE)?;
        Ok(inst)
    }

    fn request(&self, flags: u32) -> Result<()> {
        let mut req = ptp_extts_request::default();
        req.index = self.index;
        req.flags = flags;
        unsafe { ptp_extts_request2(self.file.as_raw_fd(), &req) }
            .map_err(|e| format_err!("While setting EXTTS {} flags {:#x}: {}", self.index, flags, e))?;
        Ok(())
    }

    /// Waits up to 'timeout' for events and returns the clock times of the
    /// edges seen on this channel, oldest first. Empty on a timeout.
    pub fn wait_events(&mut self, timeout: Duration) -> Result<Vec<Duration>> {
        let mut fds = [sys::bindings::pollfd {
            fd: self.file.as_raw_fd(),
            events: sys::POLLIN,
            revents: 0,
        }];

        if sys::poll(&mut fds, Some(timeout))? == 0 {
            return Ok(vec![]);
        }

        let mut buf = [0u8; EVENT_SIZE * 16];
        let n = self.file.read(&mut buf)?;

        Ok(parse_events(&buf[..n], self.index))
    }
}

impl Drop for ExttsChannel {
    fn drop(&mut self) {
        if let Err(e) = self.request(0) {
            eprintln!("[ExttsChannel] {}", e);
        }
    }
}

/// Times of the events in 'data' (whole ptp_extts_event records) that are for
/// channel 'index'.
fn parse_events(data: &[u8], index: u32) -> Vec<Duration> {
    let mut out = vec![];
    for record in data.chunks_exact(EVENT_SIZE) {
        let sec = i64::from_ne_bytes(record[0..8].try_into().unwrap());
        let nsec = u32::from_ne_bytes(record[8..12].try_into().unwrap());
        let event_index = u32::from_ne_bytes(record[16..20].try_into().unwrap());

        if event_index != index || sec < 0 {
            continue;
        }

        out.push(Duration::new(sec as u64, nsec));
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(sec: i64, nsec: u32, index: u32) -> Vec<u8> {
        let mut v = vec![];
        v.extend_from_slice(&sec.to_ne_bytes());
        v.extend_from_slice(&nsec.to_ne_bytes());
        v.extend_from_slice(&0u32.to_ne_bytes());
        v.extend_from_slice(&index.to_ne_bytes());
        v.extend_from_slice(&[0u8; 12]);
        v
    }

    #[test]
    fn parse_events_keeps_this_channels_records_in_order() {
        assert_eq!(EVENT_SIZE, 32);

        let mut data = record(5, 100, 0);
        data.extend(record(6, 200, 1));
        data.extend(record(7, 300, 0));
        data.extend_from_slice(&[1, 2, 3]); // A partial record is ignored.

        assert_eq!(
            parse_events(&data, 0),
            vec![Duration::new(5, 100), Duration::new(7, 300)]
        );
        assert_eq!(parse_events(&data, 1), vec![Duration::new(6, 200)]);
    }
}
