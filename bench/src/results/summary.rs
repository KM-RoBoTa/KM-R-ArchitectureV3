//! `results summary <dir>`: one set from one commit and host, every cell side by side.
//!
//! The 1 kHz cells first (non-RT and PREEMPT_RT, idle and loaded), then the
//! ceiling, the budget and the footprint. It names what is missing from the
//! set and never gives a verdict (spec US1 scenario 2; FR-023 (SRS NFR-11); SC-004).

use std::fmt::Write as _;
use std::io;

use super::parse::Report;
use super::{
    Cell, Exit, FileInfo, Group, Groups, Guard, MAX_FILES, MAX_GROUPS, Refusal, Set, Table, load,
    padded,
};
use crate::buf::Buf;

/// The refusal keys, in order: the contract's three, then a mix of reference and not.
const KEYS: [&str; 4] = ["schema", "host_id", "robot", "reference"];

/// The `[meta]` keys of FR-002 (SRS NFR-1, NFR-8) every report must hold.
const META: [&str; 31] = [
    "schema",
    "host_id",
    "board_model",
    "board_revision",
    "distro",
    "kernel",
    "kernel_rt",
    "governor",
    "affinity",
    "sched",
    "sched_rt_runtime_us",
    "timer_slack_ns",
    "rust_log",
    "supply_mv",
    "supply_meter",
    "ambient_mc",
    "ambient_meter",
    "throttled_start",
    "throttled_end",
    "soc_mc_start",
    "soc_mc_end",
    "commit",
    "dirty",
    "profile",
    "features",
    "robot",
    "robot_n",
    "buses",
    "controllers",
    "load_command",
    "reference",
];

/// The `[meta]` keys a bench binary gives: a compile-time report has none of them.
const FROM_BINARY: [&str; 5] = ["timer_slack_ns", "robot", "robot_n", "buses", "controllers"];

/// Columns per printed table.
const COLS: usize = 4;

/// Width of one printed cell.
const PART: usize = 160;

/// Summarizes `set` on `out`.
///
/// # Errors
///
/// The sink.
pub fn summary(set: &impl Set, out: &mut impl io::Write) -> io::Result<Exit> {
    let mut report = Report::new();
    let mut guard = Guard::new(KEYS);
    let mut refusal = Refusal::none();
    let mut groups = Groups::new();
    let mut files = [FileInfo::EMPTY; MAX_FILES];
    let mut missing = [0u64; MAX_FILES];

    for (i, (info, gaps)) in files
        .iter_mut()
        .zip(missing.iter_mut())
        .enumerate()
        .take(set.len())
    {
        if !load(set, i, &mut report, out)? {
            return Ok(Exit::Unreadable);
        }
        refusal.keep(guard.differs(&report, refusal.below()));
        guard.note(&report);
        let Some(g) = groups.of(&report) else {
            writeln!(out, "{}: over 64 measurement and cell pairs", set.label())?;
            return Ok(Exit::Unreadable);
        };
        *info = FileInfo {
            group: g,
            excluded: super::exclusion(&report),
        };
        if info.excluded.is_none()
            && let Some(r) = groups.get_mut(g).map(|g| &mut g.repeats[0])
        {
            *r += 1;
        }
        let compile = report.run_text("measurement") == "compile";
        for (bit, key) in META.iter().enumerate() {
            if report.meta(key).is_none() && !(compile && FROM_BINARY.contains(key)) {
                *gaps |= 1 << bit;
            }
        }
    }
    if refusal.print(&guard, out)? {
        return Ok(Exit::Refused);
    }

    let mut table = Table::<COLS>::new();
    for section in Section::ALL {
        let mut cols = [0usize; MAX_GROUPS];
        let mut n = 0;
        let mut hidden = 0;
        for (g, group) in groups.all().iter().enumerate() {
            if !section.holds(group.measurement.as_str()) {
                continue;
            }
            if section.candidate(group.cell.as_str()) {
                hidden += group.repeats[0];
            } else if group.repeats[0] > 0
                && let Some(slot) = cols.get_mut(n)
            {
                *slot = g;
                n += 1;
            }
        }
        if n == 0 && hidden == 0 {
            continue;
        }
        let all = groups.all();
        let cols = cols.get_mut(..n).unwrap_or_default();
        cols.sort_unstable_by_key(|&g| {
            all.get(g).map(|g| {
                (
                    cell_rank(g.cell.as_str()),
                    g.measurement.as_bytes(),
                    g.cell.as_bytes(),
                )
            })
        });
        writeln!(out, "== {} ==", section.title())?;
        if hidden > 0 {
            writeln!(out, "({hidden} candidate reports not shown)")?;
        }
        for chunk in cols.chunks(COLS) {
            table.clear();
            for (col, &g) in chunk.iter().enumerate() {
                for (i, info) in files.iter().enumerate().take(set.len()) {
                    if info.group == g && info.excluded.is_none() {
                        if !load(set, i, &mut report, out)? {
                            return Ok(Exit::Unreadable);
                        }
                        table.add(col, &report);
                    }
                }
            }
            print_table(&table, all, chunk, out)?;
        }
        writeln!(out)?;
    }

    for kernel in ["nonrt", "rt"] {
        for (item, measurement) in [
            ("(a)", "ceiling"),
            ("(b)", "jitter"),
            ("(c)", "load"),
            ("(d)", "budget"),
        ] {
            let held = groups.all().iter().any(|g| {
                g.repeats[0] > 0
                    && g.measurement.as_str() == measurement
                    && expected(measurement, kernel, g.cell.as_str())
            });
            if !held {
                writeln!(out, "missing: {item} {measurement} on {kernel}")?;
            }
        }
    }
    if !groups
        .all()
        .iter()
        .any(|g| g.repeats[0] > 0 && g.measurement.as_str() == "footprint")
    {
        writeln!(out, "missing: (e) footprint")?;
    }
    for (i, gaps) in missing.iter().enumerate().take(set.len()) {
        if *gaps == 0 {
            continue;
        }
        write!(out, "missing in {}/{}: [meta]", set.label(), set.name(i))?;
        let mut first = true;
        for (bit, key) in META.iter().enumerate() {
            if gaps & (1 << bit) != 0 {
                write!(out, "{} {key}", if first { "" } else { "," })?;
                first = false;
            }
        }
        writeln!(out)?;
    }
    for (i, info) in files.iter().enumerate().take(set.len()) {
        if let Some(why) = info.excluded {
            writeln!(out, "excluded: {}/{}: {why}", set.label(), set.name(i))?;
        }
    }
    Ok(Exit::Printed)
}

/// One printed part of the summary.
#[derive(Clone, Copy)]
enum Section {
    /// `jitter` and `load`: the four cells of (b) and (c).
    Cells,
    /// (a), summary reports only.
    Ceiling,
    /// (d), summary reports only.
    Budget,
    /// (e).
    Footprint,
    /// Every other measurement.
    Other,
}

impl Section {
    /// In print order.
    const ALL: [Self; 5] = [
        Self::Cells,
        Self::Ceiling,
        Self::Budget,
        Self::Footprint,
        Self::Other,
    ];

    /// Whether reports of `measurement` print here.
    fn holds(self, measurement: &str) -> bool {
        match self {
            Self::Cells => matches!(measurement, "jitter" | "load"),
            Self::Ceiling => measurement == "ceiling",
            Self::Budget => measurement == "budget",
            Self::Footprint => measurement == "footprint",
            Self::Other => !matches!(
                measurement,
                "jitter" | "load" | "ceiling" | "budget" | "footprint"
            ),
        }
    }

    /// Whether `cell` is one candidate of a search, not its summary: `<k>-dt<n>`, `<k>-w<n>`.
    fn candidate(self, cell: &str) -> bool {
        let tag = match self {
            Self::Ceiling => "-dt",
            Self::Budget => "-w",
            _ => return false,
        };
        cell.rfind(tag).is_some_and(|at| {
            let digits = cell.get(at + tag.len()..).unwrap_or_default();
            !digits.is_empty() && digits.bytes().all(|b| b.is_ascii_digit())
        })
    }

    /// The heading.
    fn title(self) -> &'static str {
        match self {
            Self::Cells => "1 kHz cells",
            Self::Ceiling => "(a) ceiling",
            Self::Budget => "(d) budget",
            Self::Footprint => "(e) footprint",
            Self::Other => "other measurements",
        }
    }
}

/// Whether `cell` is the summary of `measurement` on `kernel`.
fn expected(measurement: &str, kernel: &str, cell: &str) -> bool {
    let Some(rest) = cell.strip_prefix(kernel).and_then(|c| c.strip_prefix('-')) else {
        return false;
    };
    match measurement {
        "ceiling" | "jitter" => rest == "idle",
        "load" => rest == "load",
        _ => !Section::Budget.candidate(cell),
    }
}

/// Column order: the four 1 kHz cells first, in the SRS order, then by name.
fn cell_rank(cell: &str) -> u8 {
    match cell {
        "nonrt-idle" => 0,
        "nonrt-load" => 1,
        "rt-idle" => 2,
        "rt-load" => 3,
        _ => 4,
    }
}

/// Prints one table: a header of cells, then one line per figure.
fn print_table(
    table: &Table<COLS>,
    groups: &[Group],
    cols: &[usize],
    out: &mut impl io::Write,
) -> io::Result<()> {
    let mut widths = [0usize; COLS + 1];
    widths[0] = "figure".len();
    for (w, &g) in widths.iter_mut().skip(1).zip(cols) {
        *w = header(groups, g).as_str().chars().count();
    }
    for (key, cells) in table.rows() {
        widths[0] = widths[0].max(key.chars().count());
        for (w, c) in widths.iter_mut().skip(1).zip(cells.iter().take(cols.len())) {
            *w = (*w).max(part(key, c).as_str().chars().count());
        }
    }
    let last = cols.len();
    padded(out, "figure", widths[0])?;
    for (i, (w, &g)) in widths.iter().skip(1).zip(cols).enumerate() {
        out.write_all(b"  ")?;
        padded(
            out,
            header(groups, g).as_str(),
            if i + 1 == last { 0 } else { *w },
        )?;
    }
    writeln!(out)?;
    for (key, cells) in table.rows() {
        padded(out, key, widths[0])?;
        for (i, (w, c)) in widths
            .iter()
            .skip(1)
            .zip(cells.iter().take(last))
            .enumerate()
        {
            out.write_all(b"  ")?;
            padded(
                out,
                part(key, c).as_str(),
                if i + 1 == last { 0 } else { *w },
            )?;
        }
        writeln!(out)?;
    }
    Ok(())
}

/// `<measurement> <cell> (<repeats>)`.
fn header(groups: &[Group], g: usize) -> Buf<PART> {
    let mut h = Buf::new();
    if let Some(g) = groups.get(g) {
        let _ = write!(
            h,
            "{} {} ({})",
            g.measurement.as_str(),
            g.cell.as_str(),
            g.repeats[0]
        );
    }
    h
}

/// `<median>`, `<median> (±<spread>)` over repeats, the text, or `-`.
fn part(key: &str, cell: &Cell) -> Buf<PART> {
    let mut p = Buf::new();
    let _ = match (cell.stat(), cell.text()) {
        (Some(s), _) if s.n == 1 => write!(p, "{}", s.median),
        (Some(s), _) => write!(p, "{} (±{})", s.median, s.spread),
        (None, Some("yes")) if key == "floor_held" => {
            write!(p, "ceiling at least 100 kHz (floor held)")
        }
        (None, Some(t)) => write!(p, "{t}"),
        (None, None) => write!(p, "-"),
    };
    p
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::results::fixture::{Fixture, Out, report};

    /// Every FR-002 key, so a report misses none.
    const FULL: [(&str, &str); 25] = [
        ("board_model", "\"Raspberry Pi 4 Model B Rev 1.4\""),
        ("board_revision", "\"c03114\""),
        ("distro", "\"Arch Linux ARM\""),
        ("kernel", "\"6.14.3\""),
        ("governor", "\"performance\""),
        ("affinity", "\"3\""),
        ("sched", "\"SCHED_FIFO 80\""),
        ("sched_rt_runtime_us", "950000"),
        ("timer_slack_ns", "50000"),
        ("rust_log", "\"unset\""),
        ("supply_mv", "5120"),
        ("supply_meter", "\"m\""),
        ("ambient_mc", "21500"),
        ("ambient_meter", "\"t\""),
        ("throttled_start", "\"0x0\""),
        ("throttled_end", "\"0x0\""),
        ("soc_mc_start", "48300"),
        ("soc_mc_end", "49000"),
        ("commit", "\"76bd454\""),
        ("dirty", "\"no\""),
        ("profile", "\"release\""),
        ("features", "\"\""),
        ("buses", "\"none\""),
        ("controllers", "\"probe+noop/first,each,last\""),
        ("load_command", "\"none\""),
    ];

    /// Runs `summary` on one set of report texts.
    fn run(files: &[(&str, &str)]) -> (Exit, Out) {
        let mut out = Out(Buf::new());
        let exit = summary(&Fixture { label: "S", files }, &mut out).expect("sink");
        (exit, out)
    }

    /// A complete report of `measurement` and `cell` with every FR-002 key.
    fn full(measurement: &str, cell: &str, figures: &str) -> Buf<2048> {
        report(
            &FULL,
            (measurement, cell, "complete", "simulated,not_reference"),
            figures,
        )
    }

    #[test]
    fn refuses_a_differing_schema_host_or_robot_and_a_reference_mix() {
        let base = full("jitter", "nonrt-idle", "jitter_p50_ns = 1\n");
        for (over, line) in [
            (&[("schema", "2")][..], "refused: schema differs: 1 vs 2\n"),
            (
                &[("host_id", "\"pi\"")][..],
                "refused: host_id differs: dev vs pi\n",
            ),
            (
                &[("robot", "\"Arm6\"")][..],
                "refused: robot differs: XmBench vs Arm6\n",
            ),
            (
                &[("reference", "\"yes\"")][..],
                "refused: reference differs: no vs yes\n",
            ),
        ] {
            let other = report(
                over,
                ("jitter", "rt-idle", "complete", ""),
                "jitter_p50_ns = 1\n",
            );
            let (exit, out) = run(&[("1", base.as_str()), ("2", other.as_str())]);
            assert_eq!((exit, out.0.as_str()), (Exit::Refused, line));
        }
        let kernel = report(
            &[("robot_n", "5"), ("kernel_rt", "\"yes\"")],
            ("jitter", "rt-idle", "complete", ""),
            "jitter_p50_ns = 1\n",
        );
        let (exit, _) = run(&[("1", base.as_str()), ("2", kernel.as_str())]);
        assert_eq!(exit, Exit::Printed);
    }

    #[test]
    fn a_set_prints_side_by_side_and_names_what_is_missing() {
        let idle1 = full("jitter", "nonrt-idle", "jitter_p50_ns = 50000\n");
        let idle2 = full("jitter", "nonrt-idle", "jitter_p50_ns = 52000\n");
        let load = full("load", "nonrt-load", "jitter_p50_ns = 61000\n");
        let candidate = full("ceiling", "nonrt-dt10000", "bench_overruns = 0\n");
        let ceiling = full(
            "ceiling",
            "nonrt-idle",
            "ceiling_dt_ns = 10000\nfloor_held = \"yes\"\n",
        );
        let budget = full(
            "budget",
            "nonrt-none",
            "budget_ns = 990000\noverhead_ns = 10000\n",
        );
        let compile = report(
            &[("load_command", "\"none\"")],
            ("compile", "dev-x86", "complete", ""),
            "kmr_core_dev_clean_median_ms = 900\n",
        );
        let throttled = report(
            &FULL,
            ("jitter", "nonrt-idle", "complete", "throttled"),
            "jitter_p50_ns = 99\n",
        );
        let (exit, out) = run(&[
            ("budget-nonrt-none-1.toml", budget.as_str()),
            ("ceiling-nonrt-dt10000-1.toml", candidate.as_str()),
            ("ceiling-nonrt-idle-1.toml", ceiling.as_str()),
            ("compile-dev-x86-1.toml", compile.as_str()),
            ("jitter-nonrt-idle-1.toml", idle1.as_str()),
            ("jitter-nonrt-idle-2.toml", idle2.as_str()),
            ("jitter-nonrt-idle-3.toml", throttled.as_str()),
            ("load-nonrt-load-1.toml", load.as_str()),
        ]);
        assert_eq!(exit, Exit::Printed);
        let text = out.0.as_str();
        assert!(
            text.starts_with(
                "== 1 kHz cells ==\n\
                 figure         jitter nonrt-idle (2)  load nonrt-load (1)\n\
                 jitter_p50_ns  50000 (±2000)          61000\n\n\
                 == (a) ceiling ==\n\
                 (1 candidate reports not shown)\n"
            ),
            "{text}"
        );
        assert!(text.contains("floor_held     ceiling at least 100 kHz (floor held)"));
        assert!(text.contains("== (d) budget ==\n"));
        assert!(text.contains("== other measurements ==\n"));
        assert!(!text.contains("missing: (a) ceiling on nonrt"));
        assert!(!text.contains("missing: (d) budget on nonrt"));
        for line in [
            "missing: (a) ceiling on rt\n",
            "missing: (b) jitter on rt\n",
            "missing: (c) load on rt\n",
            "missing: (d) budget on rt\n",
            "missing: (e) footprint\n",
            "missing in S/compile-dev-x86-1.toml: [meta] board_model, board_revision,",
            "excluded: S/jitter-nonrt-idle-3.toml: throttled\n",
        ] {
            assert!(text.contains(line), "{line} in {text}");
        }
        assert!(!text.contains("missing in S/jitter-nonrt-idle-1.toml"));
        let compile_line = text
            .lines()
            .find(|l| l.starts_with("missing in S/compile"))
            .unwrap_or_default();
        assert!(!compile_line.contains("robot_n"), "{compile_line}");
    }
}
