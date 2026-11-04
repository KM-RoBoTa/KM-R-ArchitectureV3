//! Footprint and host facts from `/proc/self`, read after `run()` (research R11, R12).
//!
//! A chunked line scanner over a fixed buffer, so nothing lands on the heap.

use std::fs::File;
use std::io::{ErrorKind, Read};

/// Scanner buffer size; a longer line is cut to it.
const CHUNK: usize = 1024;

/// Peak resident set, `VmHWM` of /proc/self/status, in kB.
pub fn vm_hwm_kb() -> Option<u64> {
    File::open("/proc/self/status")
        .ok()
        .and_then(vm_hwm_kb_from)
}

/// Stack high-water mark to the page, the `Rss` of the `[stack]` mapping in /proc/self/smaps, in kB.
pub fn stack_kb() -> Option<u64> {
    File::open("/proc/self/smaps").ok().and_then(stack_kb_from)
}

/// The main thread's timer slack, /proc/self/timerslack_ns; the loop runs on that thread.
///
/// Call it on the main thread: from any other one the kernel asks for `CAP_SYS_NICE`.
pub fn timer_slack_ns() -> Option<u64> {
    File::open("/proc/self/timerslack_ns")
        .ok()
        .and_then(first_int_from)
}

/// `VmHWM` from status text.
pub fn vm_hwm_kb_from(r: impl Read) -> Option<u64> {
    let mut found = None;
    scan_lines::<CHUNK>(r, |line| {
        found = line.strip_prefix(b"VmHWM:").and_then(int_in);
        found.is_none()
    });
    found
}

/// `Rss` of the `[stack]` mapping from smaps text.
pub fn stack_kb_from(r: impl Read) -> Option<u64> {
    let mut in_stack = false;
    let mut found = None;
    scan_lines::<CHUNK>(r, |line| {
        if line
            .first()
            .is_some_and(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(b))
        {
            in_stack = line.trim_ascii_end().ends_with(b"[stack]");
        } else if in_stack {
            found = line.strip_prefix(b"Rss:").and_then(int_in);
        }
        found.is_none()
    });
    found
}

/// The integer on the first line.
pub fn first_int_from(r: impl Read) -> Option<u64> {
    let mut found = None;
    scan_lines::<CHUNK>(r, |line| {
        found = int_in(line);
        false
    });
    found
}

/// Calls `each` on every line of `r`, without its `\n`, until it returns `false`.
///
/// A line longer than `CAP` is handed on cut to `CAP` bytes; a read error ends the scan.
pub fn scan_lines<const CAP: usize>(mut r: impl Read, mut each: impl FnMut(&[u8]) -> bool) {
    let mut buf = [0u8; CAP];
    let mut held = 0;
    let mut skipping = false;
    loop {
        let Some(room) = buf.get_mut(held..) else {
            return;
        };
        let read = match r.read(room) {
            Ok(n) => n,
            Err(e) if e.kind() == ErrorKind::Interrupted => continue,
            Err(_) => return,
        };
        if read == 0 {
            if held > 0 && !skipping {
                each(buf.get(..held).unwrap_or_default());
            }
            return;
        }
        let filled = held + read;
        let mut start = 0;
        while let Some(nl) = buf
            .get(start..filled)
            .and_then(|rest| rest.iter().position(|&b| b == b'\n'))
        {
            if !skipping && !each(buf.get(start..start + nl).unwrap_or_default()) {
                return;
            }
            skipping = false;
            start += nl + 1;
        }
        if start == 0 && filled == CAP {
            if !skipping && !each(&buf) {
                return;
            }
            skipping = true;
            held = 0;
        } else {
            buf.copy_within(start..filled, 0);
            held = filled - start;
        }
    }
}

/// The first run of digits in `text`.
fn int_in(text: &[u8]) -> Option<u64> {
    let from = text.iter().position(u8::is_ascii_digit)?;
    let digits = text.get(from..)?;
    let len = digits
        .iter()
        .position(|b| !b.is_ascii_digit())
        .unwrap_or(digits.len());
    digits.get(..len)?.iter().try_fold(0u64, |acc, &d| {
        acc.checked_mul(10)?.checked_add(u64::from(d - b'0'))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const STATUS: &[u8] =
        b"Name:\ttick\nVmPeak:\t   10000 kB\nVmHWM:\t    2924 kB\nVmRSS:\t    2900 kB\n";

    const SMAPS: &[u8] = b"\
55d1c0000000-55d1c0001000 r--p 00000000 103:02 1234                     /usr/bin/tick
Size:                  4 kB
Rss:                   4 kB
VmFlags: rd mr mw me
7ffd5c000000-7ffd5c021000 rw-p 00000000 00:00 0                          [stack]
Size:                132 kB
KernelPageSize:        4 kB
Rss:                  16 kB
Pss:                  16 kB
VmFlags: rd wr mr mw me gd ac
ffffffffff600000-ffffffffff601000 --xp 00000000 00:00 0                  [vsyscall]
Rss:                   0 kB
";

    /// A reader that hands out at most `step` bytes per call.
    struct Trickle<'a> {
        bytes: &'a [u8],
        step: usize,
    }

    impl Read for Trickle<'_> {
        fn read(&mut self, out: &mut [u8]) -> std::io::Result<usize> {
            let n = self.step.min(out.len()).min(self.bytes.len());
            out[..n].copy_from_slice(&self.bytes[..n]);
            self.bytes = &self.bytes[n..];
            Ok(n)
        }
    }

    #[test]
    fn vm_hwm_is_read_from_status() {
        assert_eq!(vm_hwm_kb_from(STATUS), Some(2924));
        assert_eq!(vm_hwm_kb_from(&b"Name:\ttick\n"[..]), None);
    }

    #[test]
    fn stack_rss_is_the_one_of_the_stack_mapping() {
        assert_eq!(stack_kb_from(SMAPS), Some(16));
        assert_eq!(stack_kb_from(&SMAPS[..200]), None);
    }

    #[test]
    fn timer_slack_is_the_first_integer() {
        assert_eq!(first_int_from(&b"50000\n"[..]), Some(50_000));
        assert_eq!(first_int_from(&b"\n"[..]), None);
    }

    #[test]
    fn a_line_split_across_two_chunks_is_read_whole() {
        let mut lines = 0;
        let mut hwm = None;
        // A 32-byte buffer and 5-byte reads cut "VmHWM:\t    2924 kB" across chunks.
        scan_lines::<32>(
            Trickle {
                bytes: STATUS,
                step: 5,
            },
            |line| {
                lines += 1;
                if let Some(rest) = line.strip_prefix(b"VmHWM:") {
                    hwm = int_in(rest);
                }
                true
            },
        );
        assert_eq!((lines, hwm), (4, Some(2924)));
        assert_eq!(
            stack_kb_from(Trickle {
                bytes: SMAPS,
                step: 7
            }),
            Some(16)
        );
    }

    #[test]
    fn a_line_longer_than_the_buffer_is_cut_and_the_next_one_kept() {
        let mut seen = [[0u8; 8]; 3];
        let mut n = 0;
        scan_lines::<8>(&b"0123456789abcdef\nRss: 4\nlast"[..], |line| {
            seen[n][..line.len()].copy_from_slice(line);
            n += 1;
            true
        });
        assert_eq!(n, 3);
        assert_eq!(&seen[0], b"01234567");
        assert_eq!(&seen[1][..6], b"Rss: 4");
        assert_eq!(&seen[2][..4], b"last");
    }

    #[test]
    fn this_process_reads_its_own_figures() {
        assert!(vm_hwm_kb().is_some_and(|kb| kb > 0));
        assert!(stack_kb().is_some_and(|kb| kb > 0));
    }
}
