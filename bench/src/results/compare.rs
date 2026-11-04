//! `results compare <dir A> <dir B>`: every figure of two builds side by side.
//!
//! Rules: data-model.md "Comparison". Refuses across hosts, robots or kernels,
//! and across a reference and a non-reference set (spec US2 scenario 2); each
//! measurement and cell needs 1 to 16 counted repeats on each side; never gives
//! a verdict (FR-022, FR-023 (SRS NFR-11, NFR-8, NFR-1); SC-009).

use std::fmt::Write as _;
use std::io;

use super::parse::Report;
use super::{
    Cell, Exit, FileInfo, Groups, Guard, MAX_FILES, MAX_REPEATS, Refusal, Set, Table, load, padded,
};
use crate::buf::Buf;

/// The refusal keys, in order: the contract's five, then `reference`.
const KEYS: [&str; 6] = [
    "schema",
    "host_id",
    "robot",
    "robot_n",
    "kernel_rt",
    "reference",
];

/// Width of one printed column part.
const PART: usize = 160;

/// Compares set `a` with set `b` and prints the result on `out`.
///
/// # Errors
///
/// The sink.
pub fn compare<S: Set>(a: &S, b: &S, out: &mut impl io::Write) -> io::Result<Exit> {
    let sets = [a, b];
    let mut report = Report::new();
    let mut guards = [Guard::new(KEYS), Guard::new(KEYS)];
    let mut groups = Groups::new();
    let mut files = [[FileInfo::EMPTY; MAX_FILES]; 2];

    for set in sets {
        if set.is_empty() {
            writeln!(out, "{}: no report", set.label())?;
            return Ok(Exit::Unreadable);
        }
    }
    for ((side, set), (guard, infos)) in sets
        .iter()
        .enumerate()
        .zip(guards.iter_mut().zip(files.iter_mut()))
    {
        if !scan(*set, side, &mut report, guard, &mut groups, infos, out)? {
            return Ok(Exit::Unreadable);
        }
    }

    let mut refusal = Refusal::none();
    for i in 0..a.len() {
        if !load(a, i, &mut report, out)? {
            return Ok(Exit::Unreadable);
        }
        let found = guards[1].differs(&report, refusal.below());
        refusal.keep(found.map(|(k, kept_b, theirs_a)| (k, theirs_a, kept_b)));
    }
    for i in 0..b.len() {
        if !load(b, i, &mut report, out)? {
            return Ok(Exit::Unreadable);
        }
        refusal.keep(guards[0].differs(&report, refusal.below()));
    }
    if refusal.print(&guards[0], out)? {
        return Ok(Exit::Refused);
    }
    // After the refusal: sets from two kernels also differ in their cell names.
    for (g, group) in groups.all().iter().enumerate() {
        for ((set, infos), repeats) in sets.iter().zip(files.iter()).zip(group.repeats) {
            if (1..=MAX_REPEATS).contains(&repeats) {
                continue;
            }
            let (m, c) = (group.measurement.as_str(), group.cell.as_str());
            if repeats == 0 {
                writeln!(out, "{}: no repeat of {m} {c} counts", set.label())?;
            } else {
                writeln!(
                    out,
                    "{}: over {MAX_REPEATS} repeats of {m} {c}",
                    set.label()
                )?;
            }
            print_excluded(*set, infos, Some(g), out)?;
            return Ok(Exit::Unreadable);
        }
    }

    let mut table = Table::<2>::new();
    let (order, count) = groups.sorted();
    for &g in order.get(..count).unwrap_or_default() {
        table.clear();
        for (side, (set, infos)) in sets.iter().zip(files.iter()).enumerate() {
            for (i, info) in infos.iter().enumerate().take(set.len()) {
                if info.group != g || info.excluded.is_some() {
                    continue;
                }
                if !load(*set, i, &mut report, out)? {
                    return Ok(Exit::Unreadable);
                }
                table.add(side, &report);
            }
        }
        let Some(group) = groups.all().get(g) else {
            continue;
        };
        writeln!(
            out,
            "{} {}  (A {}, B {})",
            group.measurement.as_str(),
            group.cell.as_str(),
            group.repeats[0],
            group.repeats[1]
        )?;
        print_rows(&table, out)?;
        writeln!(out)?;
    }

    for (set, infos) in sets.iter().zip(files.iter()) {
        print_excluded(*set, infos, None, out)?;
    }
    Ok(Exit::Printed)
}

/// Prints `excluded: <dir>/<file>: <why>` for the excluded reports of `set`, of group `only`
/// when given.
fn print_excluded(
    set: &impl Set,
    infos: &[FileInfo; MAX_FILES],
    only: Option<usize>,
    out: &mut impl io::Write,
) -> io::Result<()> {
    for (i, info) in infos.iter().enumerate().take(set.len()) {
        if let Some(why) = info.excluded
            && only.is_none_or(|g| g == info.group)
        {
            writeln!(out, "excluded: {}/{}: {why}", set.label(), set.name(i))?;
        }
    }
    Ok(())
}

/// Reads every report of `set` once: refusal values, groups, exclusions.
fn scan(
    set: &impl Set,
    side: usize,
    report: &mut Report,
    guard: &mut Guard<6>,
    groups: &mut Groups,
    files: &mut [FileInfo; MAX_FILES],
    out: &mut impl io::Write,
) -> io::Result<bool> {
    for (i, info) in files.iter_mut().enumerate().take(set.len()) {
        if !load(set, i, report, out)? {
            return Ok(false);
        }
        guard.note(report);
        let Some(g) = groups.of(report) else {
            writeln!(out, "{}: over 64 measurement and cell pairs", set.label())?;
            return Ok(false);
        };
        *info = FileInfo {
            group: g,
            excluded: super::exclusion(report),
        };
        if info.excluded.is_none()
            && let Some(r) = groups.get_mut(g).and_then(|g| g.repeats.get_mut(side))
        {
            *r += 1;
        }
    }
    Ok(true)
}

/// Prints one aligned line per figure of a two-column table.
fn print_rows(table: &Table<2>, out: &mut impl io::Write) -> io::Result<()> {
    let mut widths = [0usize; 3];
    for (key, cells) in table.rows() {
        let [a, b] = cells;
        for (w, len) in widths.iter_mut().zip([
            key.chars().count(),
            side_part('A', a).as_str().chars().count(),
            side_part('B', b).as_str().chars().count(),
        ]) {
            *w = (*w).max(len);
        }
    }
    for (key, cells) in table.rows() {
        let [a, b] = cells;
        padded(out, key, widths[0])?;
        out.write_all(b"  ")?;
        padded(out, side_part('A', a).as_str(), widths[1])?;
        out.write_all(b"  ")?;
        let delta = delta_part(a, b);
        if delta.is_empty() {
            writeln!(out, "{}", side_part('B', b).as_str().trim_end())?;
        } else {
            padded(out, side_part('B', b).as_str(), widths[2])?;
            writeln!(out, "  {}", delta.as_str())?;
        }
    }
    Ok(())
}

/// `A <median> (±<spread>)`, `A <text>` or `A -`.
fn side_part(name: char, cell: &Cell) -> Buf<PART> {
    let mut part = Buf::new();
    let _ = match (cell.stat(), cell.text()) {
        (Some(s), _) => write!(part, "{name} {} (±{})", s.median, s.spread),
        (None, Some(t)) => write!(part, "{name} {t}"),
        (None, None) => write!(part, "{name} -"),
    };
    part
}

/// `Δ <B−A>  Δ/spread <permille>‰`; `-` where a side has no value or both spreads are 0;
/// empty for a text figure.
fn delta_part(a: &Cell, b: &Cell) -> Buf<PART> {
    let mut part = Buf::new();
    if a.stat().is_none() && b.stat().is_none() {
        return part;
    }
    let _ = match (a.stat(), b.stat()) {
        (Some(a), Some(b)) => {
            let delta = i128::from(b.median) - i128::from(a.median);
            let spread = i128::from(a.spread.max(b.spread));
            if spread == 0 {
                write!(part, "Δ {delta}  Δ/spread -")
            } else {
                write!(part, "Δ {delta}  Δ/spread {}‰", delta * 1000 / spread)
            }
        }
        _ => write!(part, "Δ -  Δ/spread -"),
    };
    part
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::results::fixture::{Fixture, Out, report};

    /// A complete jitter report with one integer and one text figure.
    fn jitter(meta: &[(&str, &str)], p50: u64) -> Buf<2048> {
        let mut figures = Buf::<256>::new();
        let _ = write!(figures, "jitter_p50_ns = {p50}\nlocks = \"not measured\"\n");
        report(
            meta,
            ("jitter", "nonrt-idle", "complete", "simulated"),
            figures.as_str(),
        )
    }

    /// The output with every run of padding cut to two spaces.
    fn squeezed(out: &Out) -> Buf<32768> {
        let mut s = Buf::new();
        let mut spaces = 0;
        for c in out.0.as_str().chars() {
            spaces = if c == ' ' { spaces + 1 } else { 0 };
            if spaces <= 2 {
                let _ = s.write_char(c);
            }
        }
        s
    }

    /// Runs `compare` on two sets of report texts.
    fn run(a: &[(&str, &str)], b: &[(&str, &str)]) -> (Exit, Out) {
        let mut out = Out(Buf::new());
        let exit = compare(
            &Fixture {
                label: "A",
                files: a,
            },
            &Fixture {
                label: "B",
                files: b,
            },
            &mut out,
        )
        .expect("sink");
        (exit, out)
    }

    #[test]
    fn refusal_names_the_first_differing_key_in_order() {
        let base = jitter(&[], 100);
        for (over, line) in [
            (&[("schema", "2")][..], "refused: schema differs: 1 vs 2\n"),
            (
                &[("host_id", "\"pi\""), ("schema", "2")][..],
                "refused: schema differs: 1 vs 2\n",
            ),
            (
                &[("host_id", "\"pi\""), ("robot_n", "5")][..],
                "refused: host_id differs: dev vs pi\n",
            ),
            (
                &[("robot", "\"Arm6\""), ("kernel_rt", "\"yes\"")][..],
                "refused: robot differs: XmBench vs Arm6\n",
            ),
            (
                &[("robot_n", "5"), ("kernel_rt", "\"yes\"")][..],
                "refused: robot_n differs: 4 vs 5\n",
            ),
            (
                &[("kernel_rt", "\"yes\""), ("reference", "\"yes\"")][..],
                "refused: kernel_rt differs: no vs yes\n",
            ),
            (
                &[("reference", "\"yes\"")][..],
                "refused: reference differs: no vs yes\n",
            ),
        ] {
            let other = jitter(over, 100);
            let (exit, out) = run(&[("a.toml", base.as_str())], &[("b.toml", other.as_str())]);
            assert_eq!(exit, Exit::Refused);
            assert_eq!(exit.code(), 2);
            assert_eq!(out.0.as_str(), line);
        }
    }

    #[test]
    fn a_difference_inside_one_side_is_still_refused() {
        let (a1, a2, b1) = (
            jitter(&[], 1),
            jitter(&[("robot_n", "5")], 1),
            jitter(&[], 1),
        );
        let (exit, out) = run(
            &[("1.toml", a1.as_str()), ("2.toml", a2.as_str())],
            &[("1.toml", b1.as_str())],
        );
        assert_eq!(exit, Exit::Refused);
        assert_eq!(out.0.as_str(), "refused: robot_n differs: 5 vs 4\n");
    }

    #[test]
    fn excluded_reports_are_listed_and_left_out() {
        let a = jitter(&[], 100);
        let b = jitter(&[], 130);
        let incomplete = report(&[], ("jitter", "nonrt-idle", "incomplete", ""), "");
        let throttled = report(
            &[],
            ("jitter", "nonrt-idle", "complete", "simulated,throttled"),
            "jitter_p50_ns = 9999\n",
        );
        let inconsistent = report(
            &[],
            ("jitter", "nonrt-idle", "complete", "inconsistent"),
            "jitter_p50_ns = 9999\n",
        );
        let (exit, out) = run(
            &[("a.toml", a.as_str())],
            &[
                ("1.toml", b.as_str()),
                ("2.toml", incomplete.as_str()),
                ("3.toml", throttled.as_str()),
                ("4.toml", inconsistent.as_str()),
            ],
        );
        assert_eq!(exit, Exit::Printed);
        assert_eq!(
            out.0.as_str(),
            "jitter nonrt-idle  (A 1, B 1)\n\
             jitter_p50_ns  A 100 (±0)      B 130 (±0)      Δ 30  Δ/spread -\n\
             locks          A not measured  B not measured\n\
             \n\
             excluded: B/2.toml: status incomplete\n\
             excluded: B/3.toml: throttled\n\
             excluded: B/4.toml: inconsistent\n"
        );
    }

    #[test]
    fn median_spread_and_delta_on_1_2_3_and_16_repeats() {
        let texts: [Buf<2048>; 16] = core::array::from_fn(|i| jitter(&[], 200 + 10 * i as u64));
        let names: [&str; 16] = [
            "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14",
            "15", "16",
        ];
        let files: [(&str, &str); 16] = core::array::from_fn(|i| (names[i], texts[i].as_str()));
        let one = jitter(&[], 100);
        for (b, line) in [
            (&files[..1], "A 100 (±0)  B 200 (±0)  Δ 100  Δ/spread -"),
            (
                &files[..2],
                "A 100 (±0)  B 200 (±10)  Δ 100  Δ/spread 10000‰",
            ),
            (
                &files[..3],
                "A 100 (±0)  B 210 (±20)  Δ 110  Δ/spread 5500‰",
            ),
            (
                &files[..],
                "A 100 (±0)  B 270 (±150)  Δ 170  Δ/spread 1133‰",
            ),
        ] {
            let (exit, out) = run(&[("a", one.as_str())], b);
            assert_eq!(exit, Exit::Printed);
            assert!(squeezed(&out).as_str().contains(line), "{}", out.0.as_str());
        }
        let (_, out) = run(&files[..3], &[("b", one.as_str())]);
        assert!(
            squeezed(&out)
                .as_str()
                .contains("A 210 (±20)  B 100 (±0)  Δ -110  Δ/spread -5500‰"),
            "{}",
            out.0.as_str()
        );
    }

    #[test]
    fn more_than_16_repeats_names_the_directory() {
        let text = jitter(&[], 1);
        let names: [&str; 17] = [
            "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14",
            "15", "16", "17",
        ];
        let files: [(&str, &str); 17] = core::array::from_fn(|i| (names[i], text.as_str()));
        let (exit, out) = run(&files[..1], &files);
        assert_eq!(exit, Exit::Unreadable);
        assert_eq!(exit.code(), 1);
        assert_eq!(out.0.as_str(), "B: over 16 repeats of jitter nonrt-idle\n");
    }

    #[test]
    fn a_side_without_a_counted_repeat_is_refused_as_input() {
        let a = jitter(&[], 100);
        let other_cell = report(
            &[],
            ("jitter", "nonrt-load", "complete", "simulated"),
            "jitter_p50_ns = 1\n",
        );
        let incomplete = report(&[], ("jitter", "nonrt-idle", "incomplete", ""), "");
        let throttled = report(
            &[],
            ("jitter", "nonrt-idle", "complete", "throttled"),
            "jitter_p50_ns = 1\n",
        );
        for (b, text) in [
            (&[][..], "B: no report\n"),
            (
                &[("1.toml", other_cell.as_str())][..],
                "B: no repeat of jitter nonrt-idle counts\n",
            ),
            (
                &[
                    ("1.toml", incomplete.as_str()),
                    ("2.toml", throttled.as_str()),
                ][..],
                "B: no repeat of jitter nonrt-idle counts\n\
                 excluded: B/1.toml: status incomplete\n\
                 excluded: B/2.toml: throttled\n",
            ),
        ] {
            let (exit, out) = run(&[("a.toml", a.as_str())], b);
            assert_eq!(exit, Exit::Unreadable);
            assert_eq!(exit.code(), 1);
            assert_eq!(out.0.as_str(), text);
        }
        let (exit, out) = run(&[], &[("b.toml", a.as_str())]);
        assert_eq!(exit, Exit::Unreadable);
        assert_eq!(out.0.as_str(), "A: no report\n");
    }

    #[test]
    fn a_refusal_wins_over_a_missing_cell() {
        let a = jitter(&[], 100);
        let rt = report(
            &[("kernel_rt", "\"yes\"")],
            ("jitter", "rt-idle", "complete", "simulated"),
            "jitter_p50_ns = 1\n",
        );
        let (exit, out) = run(&[("a.toml", a.as_str())], &[("b.toml", rt.as_str())]);
        assert_eq!(exit, Exit::Refused);
        assert_eq!(out.0.as_str(), "refused: kernel_rt differs: no vs yes\n");
    }

    #[test]
    fn a_bad_line_names_the_file_and_line() {
        let good = jitter(&[], 1);
        let (exit, out) = run(
            &[("a.toml", good.as_str())],
            &[("b.toml", "[meta]\nschema = x\n")],
        );
        assert_eq!(exit, Exit::Unreadable);
        assert_eq!(out.0.as_str(), "B/b.toml: line 2: bad value\n");
    }
}
