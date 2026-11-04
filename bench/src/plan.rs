//! The run plan: what one process measures, read once from the environment before `run()`.
//!
//! Variables and defaults: contracts/commands.md. A bad variable fails the
//! run with `bad plan: <var>`, before `run()` (FR-024 (SRS NFR-2)).

use std::ffi::OsStr;
use std::fmt;
use std::os::unix::ffi::OsStrExt;
use std::path::Path;

use crate::buf::Buf;

/// Longest report path accepted, in bytes.
pub const OUT_CAP: usize = 4096;

/// Longest `measurement` or `cell` name accepted, in bytes.
pub const NAME_CAP: usize = 64;

/// What the `EachTick` probe writes each tick (research R15, R16).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WriteMode {
    /// Nothing.
    None,
    /// Every actuator.
    All,
}

impl WriteMode {
    /// The value as the report and the environment spell it.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::All => "all",
        }
    }
}

/// One process's run plan (data-model.md "RunPlan").
pub struct RunPlan {
    /// Report file the binary writes.
    pub out: Buf<OUT_CAP>,
    /// Tick period, handed to `run()` as given: the bench never range-checks it.
    pub dt_ns: u64,
    /// Planned ticks, warm-up included.
    pub ticks: u64,
    /// Ticks left out of every figure.
    pub warmup: u64,
    /// Calibrated workload per tick, in ns.
    pub work_ns: u64,
    /// What the `EachTick` probe writes.
    pub write: WriteMode,
    /// Measurement id, for the report.
    pub measurement: Buf<NAME_CAP>,
    /// Cell id, for the report.
    pub cell: Buf<NAME_CAP>,
    /// Test-only: the tick at which `work` allocates once (research R21 P4). Debug builds only,
    /// so the release binaries the scripts run cannot inject.
    #[cfg(debug_assertions)]
    pub inject_at: Option<u64>,
}

impl RunPlan {
    /// The report path.
    pub fn out_path(&self) -> &Path {
        as_path(&self.out)
    }

    /// Reads the plan from the process environment.
    ///
    /// # Errors
    ///
    /// [`BadPlan`] naming the first bad variable.
    pub fn from_env() -> Result<Self, BadPlan> {
        Self::parse(std::env::var_os)
    }

    /// Reads the plan through `get`, which returns a variable's value or `None` when unset.
    ///
    /// # Errors
    ///
    /// [`BadPlan`] naming the first bad variable, then a broken rule.
    pub fn parse<V: AsRef<OsStr>>(
        get: impl Fn(&'static str) -> Option<V>,
    ) -> Result<Self, BadPlan> {
        let mut out = Buf::new();
        match get(OUT) {
            Some(v) if !v.as_ref().is_empty() && out.push(v.as_ref().as_bytes()) => {}
            _ => return Err(BadPlan(OUT)),
        }
        let plan = Self {
            out,
            dt_ns: int(&get, DT_NS, 1_000_000)?,
            ticks: int(&get, TICKS, 601_000)?,
            warmup: int(&get, WARMUP, 1_000)?,
            work_ns: int(&get, WORK_NS, 0)?,
            write: match get(WRITE) {
                None => WriteMode::None,
                Some(v) if v.as_ref() == "none" => WriteMode::None,
                Some(v) if v.as_ref() == "all" => WriteMode::All,
                Some(_) => return Err(BadPlan(WRITE)),
            },
            measurement: name(&get, MEASUREMENT, "jitter")?,
            cell: name(&get, CELL, "adhoc")?,
            #[cfg(debug_assertions)]
            inject_at: match get(INJECT_AT) {
                None => None,
                Some(_) => Some(int(&get, INJECT_AT, 0)?),
            },
        };
        if plan.ticks <= plan.warmup.saturating_add(2) {
            return Err(BadPlan(TICKS));
        }
        if plan.work_ns >= plan.dt_ns {
            return Err(BadPlan(WORK_NS));
        }
        Ok(plan)
    }
}

/// `KMR_BENCH_OUT` alone, `None` when unset, empty or too long: where a bad plan's report goes.
pub fn out_from_env() -> Option<Buf<OUT_CAP>> {
    let value = std::env::var_os(OUT)?;
    let mut out = Buf::new();
    (!value.is_empty() && out.push(value.as_bytes())).then_some(out)
}

/// A report path as a [`Path`].
pub fn as_path(out: &Buf<OUT_CAP>) -> &Path {
    Path::new(OsStr::from_bytes(out.as_bytes()))
}

/// A plan variable that is missing, malformed or breaks a rule.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BadPlan(pub &'static str);

impl fmt::Display for BadPlan {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "bad plan: {}", self.0)
    }
}

/// Report path, required.
const OUT: &str = "KMR_BENCH_OUT";
/// Tick period in ns.
const DT_NS: &str = "KMR_BENCH_DT_NS";
/// Planned ticks.
const TICKS: &str = "KMR_BENCH_TICKS";
/// Warm-up ticks.
const WARMUP: &str = "KMR_BENCH_WARMUP";
/// Workload per tick in ns.
const WORK_NS: &str = "KMR_BENCH_WORK_NS";
/// `none` or `all`.
const WRITE: &str = "KMR_BENCH_WRITE";
/// Measurement id.
const MEASUREMENT: &str = "KMR_BENCH_MEASUREMENT";
/// Cell id.
const CELL: &str = "KMR_BENCH_CELL";
/// Test-only allocation tick, debug builds only.
#[cfg(debug_assertions)]
const INJECT_AT: &str = "KMR_BENCH_INJECT_AT";

/// An unsigned base-10 integer variable, `default` when unset.
fn int<V: AsRef<OsStr>>(
    get: &impl Fn(&'static str) -> Option<V>,
    var: &'static str,
    default: u64,
) -> Result<u64, BadPlan> {
    let Some(value) = get(var) else {
        return Ok(default);
    };
    let digits = value.as_ref().as_bytes();
    if digits.is_empty() {
        return Err(BadPlan(var));
    }
    digits.iter().try_fold(0u64, |acc, &d| {
        if !d.is_ascii_digit() {
            return Err(BadPlan(var));
        }
        acc.checked_mul(10)
            .and_then(|acc| acc.checked_add(u64::from(d - b'0')))
            .ok_or(BadPlan(var))
    })
}

/// A name of `[A-Za-z0-9._-]`, safe in a report value and a file name; `default` when unset.
fn name<V: AsRef<OsStr>>(
    get: &impl Fn(&'static str) -> Option<V>,
    var: &'static str,
    default: &str,
) -> Result<Buf<NAME_CAP>, BadPlan> {
    let mut buf = Buf::new();
    let ok = match get(var) {
        None => buf.push(default.as_bytes()),
        Some(v) => {
            let bytes = v.as_ref().as_bytes();
            !bytes.is_empty()
                && bytes
                    .iter()
                    .all(|b| b.is_ascii_alphanumeric() || b"._-".contains(b))
                && buf.push(bytes)
        }
    };
    if ok { Ok(buf) } else { Err(BadPlan(var)) }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A lookup over fixed `(name, value)` pairs.
    fn env<const K: usize>(
        vars: [(&'static str, &'static str); K],
    ) -> impl Fn(&str) -> Option<&'static str> {
        move |name| vars.iter().find(|(k, _)| *k == name).map(|(_, v)| *v)
    }

    fn bad<const K: usize>(vars: [(&'static str, &'static str); K]) -> Option<&'static str> {
        RunPlan::parse(env(vars)).err().map(|BadPlan(var)| var)
    }

    #[test]
    fn defaults_hold_when_only_the_report_path_is_set() {
        let plan = RunPlan::parse(env([(OUT, "run.toml")])).expect("good plan");
        assert_eq!(plan.out_path(), Path::new("run.toml"));
        assert_eq!(
            (
                plan.dt_ns,
                plan.ticks,
                plan.warmup,
                plan.work_ns,
                plan.write
            ),
            (1_000_000, 601_000, 1_000, 0, WriteMode::None)
        );
        assert_eq!(
            (plan.measurement.as_str(), plan.cell.as_str()),
            ("jitter", "adhoc")
        );
    }

    #[test]
    fn every_variable_is_read() {
        let plan = RunPlan::parse(env([
            (OUT, "/tmp/x.toml"),
            (DT_NS, "10000"),
            (TICKS, "6000"),
            (WARMUP, "10"),
            (WORK_NS, "9999"),
            (WRITE, "all"),
            (MEASUREMENT, "budget"),
            (CELL, "rt-w9999"),
        ]))
        .expect("good plan");
        assert_eq!(
            (
                plan.dt_ns,
                plan.ticks,
                plan.warmup,
                plan.work_ns,
                plan.write
            ),
            (10_000, 6_000, 10, 9_999, WriteMode::All)
        );
        assert_eq!(
            (plan.measurement.as_str(), plan.cell.as_str()),
            ("budget", "rt-w9999")
        );
    }

    #[test]
    fn a_missing_or_empty_report_path_is_a_bad_plan() {
        assert_eq!(bad([]), Some(OUT));
        assert_eq!(bad([(OUT, "")]), Some(OUT));
    }

    #[test]
    fn each_malformed_variable_is_named() {
        for var in [DT_NS, TICKS, WARMUP, WORK_NS] {
            for value in ["abc", "", "-1", "1e6", " 5", "18446744073709551616"] {
                assert_eq!(
                    bad([(OUT, "r"), (var, value)]),
                    Some(var),
                    "{var}={value:?}"
                );
            }
        }
        assert_eq!(bad([(OUT, "r"), (WRITE, "some")]), Some(WRITE));
        assert_eq!(bad([(OUT, "r"), (MEASUREMENT, "a b")]), Some(MEASUREMENT));
        assert_eq!(bad([(OUT, "r"), (CELL, "../x")]), Some(CELL));
        assert_eq!(bad([(OUT, "r"), (CELL, "")]), Some(CELL));
    }

    #[test]
    fn ticks_must_exceed_warmup_plus_two() {
        assert_eq!(
            bad([(OUT, "r"), (TICKS, "1002"), (WARMUP, "1000")]),
            Some(TICKS)
        );
        assert_eq!(bad([(OUT, "r"), (TICKS, "1003"), (WARMUP, "1000")]), None);
    }

    #[test]
    fn work_must_stay_below_dt() {
        assert_eq!(
            bad([(OUT, "r"), (DT_NS, "1000"), (WORK_NS, "1000")]),
            Some(WORK_NS)
        );
        assert_eq!(bad([(OUT, "r"), (DT_NS, "1000"), (WORK_NS, "999")]), None);
    }

    #[cfg(debug_assertions)]
    #[test]
    fn the_injection_tick_is_read_in_debug_builds() {
        let plan = RunPlan::parse(env([(OUT, "r")])).expect("good plan");
        assert_eq!(plan.inject_at, None);
        let plan = RunPlan::parse(env([(OUT, "r"), (INJECT_AT, "1010")])).expect("good plan");
        assert_eq!(plan.inject_at, Some(1010));
        assert_eq!(bad([(OUT, "r"), (INJECT_AT, "x")]), Some(INJECT_AT));
    }

    #[test]
    fn dt_is_never_range_checked() {
        let plan = RunPlan::parse(env([(OUT, "r"), (DT_NS, "5000")])).expect("good plan");
        assert_eq!(plan.dt_ns, 5_000);
    }

    #[test]
    fn the_reason_names_the_variable() {
        let mut text = Buf::<64>::new();
        assert!(std::fmt::Write::write_fmt(&mut text, format_args!("{}", BadPlan(TICKS))).is_ok());
        assert_eq!(text.as_str(), "bad plan: KMR_BENCH_TICKS");
    }
}
