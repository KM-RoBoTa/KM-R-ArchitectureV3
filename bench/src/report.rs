//! Writer of a report's `[run]` and `[figures]` blocks (contracts/results-schema.md, schema 1).
//!
//! The binary writes them to the `KMR_BENCH_OUT` file, never to stdout
//! (research R3); the run script adds `[meta]`. Every line is formatted into
//! a fixed buffer.

use std::fmt::{self, Write as _};
use std::fs::File;
use std::io;
use std::path::Path;

use crate::buf::Buf;

/// Most keys in one file.
pub const MAX_KEYS: usize = 256;
/// Longest key, in bytes.
pub const MAX_KEY: usize = 128;
/// Longest text value, in bytes.
pub const MAX_TEXT: usize = 512;
/// Largest file, in bytes.
pub const MAX_FILE: usize = 64 * 1024;
/// The value of a figure every report carries but this run could not take.
pub const NOT_MEASURED: &str = "not measured";

/// How a run ended, with the reason when it did not complete.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome<'a> {
    /// Every planned tick ran.
    Complete,
    /// `run()` returned `Ok` early: Ctrl-C, SIGTERM, emergency stop.
    Incomplete(&'a str),
    /// A bad plan, missing metadata, or a `run()` error.
    Failed(&'a str),
}

impl<'a> Outcome<'a> {
    /// The `status` value.
    pub fn status(self) -> &'static str {
        match self {
            Self::Complete => "complete",
            Self::Incomplete(_) => "incomplete",
            Self::Failed(_) => "failed",
        }
    }

    /// The `reason` value, `None` for a complete run.
    pub fn reason(self) -> Option<&'a str> {
        match self {
            Self::Complete => None,
            Self::Incomplete(reason) | Self::Failed(reason) => Some(reason),
        }
    }

    /// The outcome that wins: `failed` over `incomplete` over `complete`; on a tie, `self`.
    pub fn worst(self, other: Self) -> Self {
        let rank = |o: Self| match o {
            Self::Complete => 0,
            Self::Incomplete(_) => 1,
            Self::Failed(_) => 2,
        };
        if rank(other) > rank(self) {
            other
        } else {
            self
        }
    }
}

/// A set of run flags.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Flags(u8);

impl Flags {
    /// No bus plugin, or kmr said the hardware is not driven: no I/O figure.
    pub const SIMULATED: Self = Self(1);
    /// The host is not the reference bench of SRS NFR-1.
    pub const NOT_REFERENCE: Self = Self(1 << 1);
    /// A throttle flag was set at the start or the end.
    pub const THROTTLED: Self = Self(1 << 2);
    /// Two counts that must agree did not.
    pub const INCONSISTENT: Self = Self(1 << 3);
    /// The calibration drifted more than 2 %.
    pub const UNCALIBRATED: Self = Self(1 << 4);
    /// The loop thread allocated inside the window.
    pub const ALLOC_SEEN: Self = Self(1 << 5);

    /// Every flag with its name, in report order.
    const NAMES: [(Self, &'static str); 6] = [
        (Self::SIMULATED, "simulated"),
        (Self::NOT_REFERENCE, "not_reference"),
        (Self::THROTTLED, "throttled"),
        (Self::INCONSISTENT, "inconsistent"),
        (Self::UNCALIBRATED, "uncalibrated"),
        (Self::ALLOC_SEEN, "alloc_seen"),
    ];

    /// No flag.
    pub const fn empty() -> Self {
        Self(0)
    }

    /// This set plus `flag` when `on`.
    pub const fn with_if(self, flag: Self, on: bool) -> Self {
        if on { Self(self.0 | flag.0) } else { self }
    }

    /// Whether every flag of `flag` is set.
    pub const fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 == flag.0
    }
}

/// Calibration of one run (data-model.md "Calibration").
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Calibration {
    /// ps per workload iteration before `run()`.
    pub ps_per_iter_before: u64,
    /// ps per workload iteration after `run()`.
    pub ps_per_iter_after: u64,
    /// Their drift, `None` when `before` is zero.
    pub drift_permille: Option<u64>,
    /// What the probe adds per tick.
    pub probe_cost_ns: u64,
}

/// The `[run]` block of one run.
pub struct RunInfo<'a> {
    /// Measurement id.
    pub measurement: &'a str,
    /// Cell id.
    pub cell: &'a str,
    /// Tick period, as planned.
    pub dt_ns: u64,
    /// Planned ticks.
    pub ticks: u64,
    /// Warm-up ticks.
    pub warmup: u64,
    /// Workload per tick.
    pub work_ns: u64,
    /// `none` or `all`.
    pub write: &'a str,
    /// How the run ended.
    pub outcome: Outcome<'a>,
    /// Flags the binary knows; the script adds its own.
    pub flags: Flags,
    /// Ticks that finished.
    pub ticks_done: u64,
    /// `absent` before the engine's Time getters are read, `engine` after.
    pub time_surface: &'a str,
    /// `None` when the run failed before calibrating.
    pub calibration: Option<Calibration>,
    /// Size of the bench statics.
    pub bench_storage_bytes: u64,
}

/// Why a report could not be written.
#[derive(Debug)]
pub enum ReportError {
    /// A limit of the schema, named.
    Limit(&'static str),
    /// The sink refused a write.
    Io(io::Error),
}

impl From<io::Error> for ReportError {
    fn from(e: io::Error) -> Self {
        Self::Io(e)
    }
}

/// A report being written to `W`, one line at a time.
pub struct Report<W: io::Write> {
    out: W,
    keys: usize,
    bytes: usize,
}

impl<W: io::Write> Report<W> {
    /// A report on `out`, nothing written yet.
    pub fn new(out: W) -> Self {
        Self {
            out,
            keys: 0,
            bytes: 0,
        }
    }

    /// Writes the `[run]` block.
    ///
    /// # Errors
    ///
    /// A schema limit or the sink.
    pub fn run_block(&mut self, run: &RunInfo<'_>) -> Result<(), ReportError> {
        self.block("run")?;
        self.text("measurement", run.measurement)?;
        self.text("cell", run.cell)?;
        self.int("dt_ns", run.dt_ns)?;
        self.int("ticks", run.ticks)?;
        self.int("warmup", run.warmup)?;
        self.int("work_ns", run.work_ns)?;
        self.text("write", run.write)?;
        self.text("status", run.outcome.status())?;
        let mut flags = Buf::<MAX_TEXT>::new();
        for (flag, name) in Flags::NAMES {
            if run.flags.contains(flag) {
                if !flags.is_empty() {
                    flags.push(b",");
                }
                flags.push(name.as_bytes());
            }
        }
        self.text("flags", flags.as_str())?;
        if let Some(reason) = run.outcome.reason() {
            self.text("reason", reason)?;
        }
        self.int("ticks_done", run.ticks_done)?;
        self.text("anchor", "tick0_start")?;
        self.text("time_surface", run.time_surface)?;
        if let Some(c) = run.calibration {
            self.int("ps_per_iter_before", c.ps_per_iter_before)?;
            self.int("ps_per_iter_after", c.ps_per_iter_after)?;
            self.int_opt("drift_permille", c.drift_permille)?;
            self.int("probe_cost_ns", c.probe_cost_ns)?;
        }
        self.int("bench_storage_bytes", run.bench_storage_bytes)
    }

    /// Writes the `[run]` block of a run whose plan was bad: `failed`, the reason, no tick done.
    ///
    /// # Errors
    ///
    /// A schema limit or the sink.
    pub fn bad_plan(&mut self, reason: &str) -> Result<(), ReportError> {
        self.block("run")?;
        self.text("status", Outcome::Failed(reason).status())?;
        self.text("flags", "")?;
        self.text("reason", reason)?;
        self.int("ticks_done", 0)
    }

    /// Opens `[figures]` when the run is complete, with `loop_thread_allocs` and `locks`.
    ///
    /// `loop_thread_allocs` is `None` for a binary without the counting window: the key then
    /// says `"not measured"`, as `locks` does (FR-012 (SRS NFR-1)). Returns `None` for a failed
    /// or incomplete run: it holds no figure (FR-024 (SRS NFR-2)).
    ///
    /// # Errors
    ///
    /// A schema limit or the sink.
    pub fn figures(
        &mut self,
        outcome: Outcome<'_>,
        loop_thread_allocs: Option<u64>,
    ) -> Result<Option<Figures<'_, W>>, ReportError> {
        if outcome != Outcome::Complete {
            return Ok(None);
        }
        self.block("figures")?;
        match loop_thread_allocs {
            Some(count) => self.int("loop_thread_allocs", count)?,
            None => self.text("loop_thread_allocs", NOT_MEASURED)?,
        }
        self.text("locks", NOT_MEASURED)?;
        Ok(Some(Figures(self)))
    }

    /// Writes a `[name]` line.
    ///
    /// # Errors
    ///
    /// A schema limit or the sink.
    pub fn block(&mut self, name: &str) -> Result<(), ReportError> {
        let mut line = Buf::<{ MAX_KEY + 4 }>::new();
        writeln!(line, "[{name}]").map_err(|_| ReportError::Limit("key"))?;
        self.line(line.as_bytes())
    }

    /// Writes `key = <integer>`.
    ///
    /// # Errors
    ///
    /// A schema limit or the sink.
    pub fn int(&mut self, key: &str, value: u64) -> Result<(), ReportError> {
        check_key(key)?;
        let mut line = Buf::<{ MAX_KEY + 32 }>::new();
        writeln!(line, "{key} = {value}").map_err(|_| ReportError::Limit("key"))?;
        self.keyed(line.as_bytes())
    }

    /// Writes `key = <integer>` when there is a value: a missing figure is an absent key.
    ///
    /// # Errors
    ///
    /// A schema limit or the sink.
    pub fn int_opt(&mut self, key: &str, value: Option<u64>) -> Result<(), ReportError> {
        value.map_or(Ok(()), |v| self.int(key, v))
    }

    /// Writes `key = "<text>"`, `"` turned to `'` and control characters to spaces, cut to 512 bytes.
    ///
    /// # Errors
    ///
    /// A schema limit or the sink.
    pub fn text(&mut self, key: &str, value: &str) -> Result<(), ReportError> {
        check_key(key)?;
        let mut line = Buf::<{ MAX_KEY + MAX_TEXT + 8 }>::new();
        write!(line, "{key} = \"").map_err(|_| ReportError::Limit("key"))?;
        let mut text = Buf::<MAX_TEXT>::new();
        for c in value.chars() {
            let c = match c {
                '"' => '\'',
                c if c.is_control() => ' ',
                c => c,
            };
            if text.write_char(c).is_err() {
                break;
            }
        }
        line.push(text.as_bytes());
        line.push(b"\"\n");
        self.keyed(line.as_bytes())
    }

    /// Flushes and hands back the sink.
    ///
    /// # Errors
    ///
    /// The sink.
    pub fn finish(mut self) -> Result<W, ReportError> {
        self.out.flush()?;
        Ok(self.out)
    }

    /// Writes one key line, counting it against [`MAX_KEYS`].
    fn keyed(&mut self, line: &[u8]) -> Result<(), ReportError> {
        if self.keys >= MAX_KEYS {
            return Err(ReportError::Limit("keys"));
        }
        self.line(line)?;
        self.keys += 1;
        Ok(())
    }

    /// Writes one line, counting it against [`MAX_FILE`].
    fn line(&mut self, line: &[u8]) -> Result<(), ReportError> {
        if self.bytes.saturating_add(line.len()) > MAX_FILE {
            return Err(ReportError::Limit("file"));
        }
        self.out.write_all(line)?;
        self.bytes += line.len();
        Ok(())
    }
}

/// The open `[figures]` block of a complete run.
pub struct Figures<'r, W: io::Write>(&'r mut Report<W>);

impl<W: io::Write> Figures<'_, W> {
    /// Writes `key = <integer>`.
    ///
    /// # Errors
    ///
    /// A schema limit or the sink.
    pub fn int(&mut self, key: &str, value: u64) -> Result<(), ReportError> {
        self.0.int(key, value)
    }

    /// Writes `key = <integer>` when there is a value.
    ///
    /// # Errors
    ///
    /// A schema limit or the sink.
    pub fn int_opt(&mut self, key: &str, value: Option<u64>) -> Result<(), ReportError> {
        self.0.int_opt(key, value)
    }

    /// Writes `key = "<text>"`.
    ///
    /// # Errors
    ///
    /// A schema limit or the sink.
    pub fn text(&mut self, key: &str, value: &str) -> Result<(), ReportError> {
        self.0.text(key, value)
    }
}

/// Creates the report file at `path` and writes it with `body`.
///
/// # Errors
///
/// The file cannot be created, or `body` failed.
pub fn write_file(
    path: &Path,
    body: impl FnOnce(&mut Report<File>) -> Result<(), ReportError>,
) -> Result<(), ReportError> {
    let mut report = Report::new(File::create(path)?);
    body(&mut report)?;
    report.finish().map(drop)
}

/// `value` as report text, for a `reason`; a text over 512 bytes is replaced by a note.
pub fn text_of(value: impl fmt::Display) -> Buf<MAX_TEXT> {
    let mut text = Buf::new();
    if write!(text, "{value}").is_err() {
        text.clear();
        text.push(b"reason over 512 bytes");
    }
    text
}

/// A key of 1 to 128 bytes of `[a-z0-9_]`.
fn check_key(key: &str) -> Result<(), ReportError> {
    let ok = (1..=MAX_KEY).contains(&key.len())
        && key
            .bytes()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'_');
    if ok {
        Ok(())
    } else {
        Err(ReportError::Limit("key"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn info(outcome: Outcome<'_>, flags: Flags) -> RunInfo<'_> {
        RunInfo {
            measurement: "jitter",
            cell: "adhoc",
            dt_ns: 1_000_000,
            ticks: 6_000,
            warmup: 1_000,
            work_ns: 0,
            write: "none",
            outcome,
            flags,
            ticks_done: 6_000,
            time_surface: "absent",
            calibration: Some(Calibration {
                ps_per_iter_before: 600,
                ps_per_iter_after: 612,
                drift_permille: Some(20),
                probe_cost_ns: 41,
            }),
            bench_storage_bytes: 240_120,
        }
    }

    /// Writes `[run]` and, when complete, one figure; returns the text and whether `[figures]` opened.
    fn write(outcome: Outcome<'_>, store: &mut [u8]) -> (usize, bool) {
        let mut report = Report::new(Cursor::new(store));
        let flags = Flags::empty()
            .with_if(Flags::SIMULATED, true)
            .with_if(Flags::ALLOC_SEEN, true);
        report.run_block(&info(outcome, flags)).expect("run block");
        let opened = match report.figures(outcome, Some(0)).expect("figures") {
            Some(mut f) => {
                f.int("jitter_p50_ns", 55_900).expect("figure");
                f.int_opt("jitter_max_ns", None).expect("figure");
                true
            }
            None => false,
        };
        let cursor = report.finish().expect("finish");
        (usize::try_from(cursor.position()).unwrap_or(0), opened)
    }

    #[test]
    fn a_complete_run_writes_both_blocks() {
        let mut store = [0u8; 4096];
        let (len, opened) = write(Outcome::Complete, &mut store);
        let text = std::str::from_utf8(&store[..len]).expect("utf-8");
        assert!(opened);
        assert!(
            text.starts_with(
                "[run]\nmeasurement = \"jitter\"\ncell = \"adhoc\"\ndt_ns = 1000000\n"
            )
        );
        assert!(text.contains(
            "status = \"complete\"\nflags = \"simulated,alloc_seen\"\nticks_done = 6000\n"
        ));
        assert!(!text.contains("reason"));
        assert!(text.contains("anchor = \"tick0_start\"\ntime_surface = \"absent\"\n"));
        assert!(
            text.contains(
                "drift_permille = 20\nprobe_cost_ns = 41\nbench_storage_bytes = 240120\n"
            )
        );
        assert!(text.ends_with(
            "[figures]\nloop_thread_allocs = 0\nlocks = \"not measured\"\njitter_p50_ns = 55900\n"
        ));
    }

    #[test]
    fn a_run_without_the_counter_says_allocs_are_not_measured() {
        let mut store = [0u8; 512];
        let mut report = Report::new(Cursor::new(&mut store[..]));
        report.figures(Outcome::Complete, None).expect("figures");
        let len = usize::try_from(report.finish().expect("finish").position()).unwrap_or(0);
        assert_eq!(
            std::str::from_utf8(&store[..len]).expect("utf-8"),
            "[figures]\nloop_thread_allocs = \"not measured\"\nlocks = \"not measured\"\n"
        );
    }

    #[test]
    fn failed_and_incomplete_runs_hold_no_figures() {
        for outcome in [
            Outcome::Failed("bad plan: KMR_BENCH_TICKS"),
            Outcome::Incomplete("Ctrl-C"),
        ] {
            let mut store = [0u8; 4096];
            let (len, opened) = write(outcome, &mut store);
            let text = std::str::from_utf8(&store[..len]).expect("utf-8");
            assert!(!opened);
            assert!(!text.contains("[figures]"));
            assert!(text.contains("ticks_done = 6000"));
            let reason = outcome.reason().expect("a reason");
            assert!(text.contains(reason), "{text}");
        }
    }

    #[test]
    fn a_bad_plan_report_is_failed_with_its_reason() {
        let mut store = [0u8; 512];
        let mut report = Report::new(Cursor::new(&mut store[..]));
        report.bad_plan("bad plan: KMR_BENCH_TICKS").expect("block");
        let len = usize::try_from(report.finish().expect("finish").position()).unwrap_or(0);
        assert_eq!(
            std::str::from_utf8(&store[..len]).expect("utf-8"),
            "[run]\nstatus = \"failed\"\nflags = \"\"\nreason = \"bad plan: KMR_BENCH_TICKS\"\nticks_done = 0\n"
        );
    }

    #[test]
    fn status_precedence_is_failed_then_incomplete_then_complete() {
        let (f, i, c) = (
            Outcome::Failed("f"),
            Outcome::Incomplete("i"),
            Outcome::Complete,
        );
        assert_eq!(c.worst(i), i);
        assert_eq!(i.worst(f), f);
        assert_eq!(f.worst(i), f);
        assert_eq!(c.worst(c), c);
        assert_eq!(f.worst(Outcome::Failed("g")), f);
    }

    #[test]
    fn text_is_cleaned_and_cut_to_512_bytes() {
        let mut store = [0u8; 2048];
        let mut report = Report::new(Cursor::new(&mut store[..]));
        report.text("reason", "port \"FT8\"\nbusy").expect("text");
        let long = [b'x'; 600];
        report
            .text("reason2", std::str::from_utf8(&long).expect("ascii"))
            .expect("text");
        let len = usize::try_from(report.finish().expect("finish").position()).unwrap_or(0);
        let text = std::str::from_utf8(&store[..len]).expect("utf-8");
        assert!(text.starts_with("reason = \"port 'FT8' busy\"\n"));
        let second = text.lines().nth(1).expect("second line");
        assert_eq!(second.len(), "reason2 = \"\"".len() + MAX_TEXT);
    }

    #[test]
    fn the_key_limits_hold() {
        let mut store = [0u8; MAX_FILE];
        let mut report = Report::new(Cursor::new(&mut store[..]));
        assert!(matches!(report.int("", 1), Err(ReportError::Limit("key"))));
        assert!(matches!(
            report.int("Bad-Key", 1),
            Err(ReportError::Limit("key"))
        ));
        let long = [b'k'; MAX_KEY + 1];
        let long = std::str::from_utf8(&long).expect("ascii");
        assert!(matches!(
            report.int(long, 1),
            Err(ReportError::Limit("key"))
        ));
        assert!(report.int(&long[..MAX_KEY], 1).is_ok());
        for _ in 1..MAX_KEYS {
            report.int("k", 1).expect("within the key limit");
        }
        assert!(matches!(
            report.int("k", 1),
            Err(ReportError::Limit("keys"))
        ));
    }

    #[test]
    fn the_file_limit_holds() {
        let mut store = [0u8; MAX_FILE + 1024];
        let mut report = Report::new(Cursor::new(&mut store[..]));
        let text = [b't'; MAX_TEXT];
        let text = std::str::from_utf8(&text).expect("ascii");
        // Each line is 519 bytes: the 127th crosses 64 KiB.
        for _ in 0..126 {
            report.text("k", text).expect("within the file limit");
        }
        assert!(matches!(
            report.text("k", text),
            Err(ReportError::Limit("file"))
        ));
    }
}
