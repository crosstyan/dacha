use std::time::Duration;

use common::errors::*;
use file::{LocalFile, LocalPath};

use sys::bindings::{ptp_pin_function, ptp_perout_request, ptp_pin_desc, ptp_clock_time};

use crate::ioctl::*;

pub struct PTPDevice {
    fd: LocalFile
}


impl PTPDevice {
    pub fn open_default() -> Result<Self> {
        let mut fd = file::LocalFile::open_with_options(
            "/dev/ptp0",
            file::LocalFileOpenOptions::new().write(true),
        )?;

        Ok(Self {
            fd
        })
    }

    /// Configures the first pin tied to the PTP device to output a 1Hz
    /// pulse.
    ///
    /// This is the equivalent of running the following commands:
    ///     sudo ./testptp -d /dev/ptp0 -L0,2
    ///     sudo ./testptp -d /dev/ptp0 -p 1000000000
    pub fn configure_pps_output(&self) -> Result<()> {
        let mut desc = ptp_pin_desc::default();
        desc.index = 0;
        desc.chan = 0;
        desc.func = ptp_pin_function::PTP_PF_PEROUT as u32;
        unsafe { ptp_pin_setfunc(self.fd.as_raw_fd(), &desc) }?;

        let mut req = ptp_perout_request::default();
        req.period.sec = 1;
        req.period.nsec = 0;
        unsafe { ptp_perout_request2(self.fd.as_raw_fd(), &req) }?;

        Ok(())
    }

    pub fn get_time(&self) -> Result<sys::kernel::timespec> {
        Ok(sys::ClockId::from_fd(unsafe { self.fd.as_raw_fd() }).get_time()?)
    }

    pub fn clock(&self) -> sys::ClockId {
        sys::ClockId::from_fd(unsafe { self.fd.as_raw_fd() })
    }

    #[cfg(all(target_arch = "aarch64"))]
    pub fn offset_to(&self, other_clock_id: sys::ClockId) -> Result<PTPClockOffset> {

        let mut data = sys::bindings::ptp_sys_offset_extended::default();
        data.n_samples = 3;
        data.clockid = other_clock_id.0;

        match unsafe { crate::ioctl::ptp_sys_offset_extended2(self.fd.as_raw_fd(), &mut data) } {
            Ok(_) => {}
            // Drivers without gettimex64 do not support this ioctl, among them stmmac (the
            // RV1126B's ethernet). Read the clocks in turn instead.
            Err(sys::Errno::EOPNOTSUPP) => return self.offset_by_reads(other_clock_id),
            Err(e) => return Err(e.into()),
        }

        let samples = (0..3)
            .map(|i| (to_dur(&data.ts[i][0]), to_dur(&data.ts[i][1]), to_dur(&data.ts[i][2])))
            .collect::<Vec<_>>();

        tightest_offset(&samples)
            .ok_or_else(|| err_msg("The other clock went backwards in every read of the PTP clock"))
    }

    /// For older kernels that don't have 'ptp_sys_offset_extended2'
    #[cfg(not(target_arch = "aarch64"))]
    pub fn offset_to(&self, other_clock_id: sys::ClockId) -> Result<PTPClockOffset> {
        self.offset_by_reads(other_clock_id)
    }

    /// Reads the other clock before and after the PTP clock, three times, and keeps the
    /// tightest of the three. Works with every driver, but the reads are separate syscalls, so
    /// the window is wider than with 'ptp_sys_offset_extended2'. A sample in which the other
    /// clock went backwards (it was stepped between the reads) is skipped.
    pub fn offset_by_reads(&self, other_clock_id: sys::ClockId) -> Result<PTPClockOffset> {
        let mut samples = vec![];
        for _ in 0..3 {
            let other_time: Duration = other_clock_id.get_time()?.into();
            let ptp_time: Duration = self.get_time()?.into();
            let other_time2: Duration = other_clock_id.get_time()?.into();
            samples.push((other_time, ptp_time, other_time2));
        }

        tightest_offset(&samples)
            .ok_or_else(|| err_msg("The other clock went backwards in every read of the PTP clock"))
    }
}

/// Of samples (other clock, PTP clock, other clock again), the one with the shortest window,
/// as an offset taken at the window's midpoint. Samples in which the other clock went
/// backwards are skipped; None if all did.
fn tightest_offset(samples: &[(Duration, Duration, Duration)]) -> Option<PTPClockOffset> {
    let mut best_offset: Option<PTPClockOffset> = None;

    for (other_time, ptp_time, other_time2) in samples.iter().cloned() {
        let rtt = match other_time2.checked_sub(other_time) {
            Some(v) => v,
            None => continue,
        };

        if best_offset.as_ref().map(|b| rtt < b.rtt).unwrap_or(true) {
            best_offset = Some(PTPClockOffset {
                ptp_time,
                other_time: other_time + (rtt / 2),
                rtt
            });
        }
    }

    best_offset
}

fn to_dur(t: &ptp_clock_time) -> Duration {
    Duration::from_secs(t.sec as u64) + Duration::from_nanos(t.nsec as u64)
}

#[derive(Clone, Debug)]
pub struct PTPClockOffset {
    pub ptp_time: Duration,
    
    /// NOTE: This already includes compensation for half the RTT.
    pub other_time: Duration,

    pub rtt: Duration,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn us(v: u64) -> Duration {
        Duration::from_micros(v)
    }

    #[test]
    fn tightest_offset_skips_samples_where_the_clock_went_back() {
        let samples = [
            (us(1000), us(5000), us(1008)),
            (us(2000), us(6000), us(1500)), // stepped back between the reads
            (us(3000), us(7000), us(3004)),
        ];

        let best = tightest_offset(&samples).unwrap();
        assert_eq!(best.rtt, us(4));
        assert_eq!(best.other_time, us(3002));
        assert_eq!(best.ptp_time, us(7000));

        assert!(tightest_offset(&[(us(2000), us(6000), us(1500))]).is_none());
    }
}
