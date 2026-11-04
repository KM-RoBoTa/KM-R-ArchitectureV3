//! The `results` tool: reads sets of reports, compares two, summarizes one.
//!
//! Contract: contracts/results-schema.md "Tools". Integers only, and never a
//! verdict (FR-023 (SRS NFR-11, NFR-1)). Everything is held in fixed arrays.

pub mod compare;
pub mod parse;
pub mod summary;

use std::ffi::OsStr;
use std::fmt::Write as _;
use std::io;
use std::os::unix::ffi::OsStrExt;
use std::path::Path;

use crate::buf::Buf;
use crate::report::{MAX_KEY, MAX_KEYS};
use parse::{Block, LoadError, Report, Value};

/// Most report files in one directory.
pub const MAX_FILES: usize = 256;
/// Longest report file name, in bytes.
pub const NAME_CAP: usize = 128;
/// Most repeats of one measurement and cell (data-model.md "Comparison").
pub const MAX_REPEATS: usize = 16;
/// Most measurement and cell pairs in one directory.
pub const MAX_GROUPS: usize = 64;
/// Longest refusal value or text figure kept, in bytes.
pub const SHORT: usize = 64;
/// Longest path built from a directory and a name.
const PATH_CAP: usize = 4096;

/// How the tool ends (contracts/results-schema.md "Tools").
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Exit {
    /// Printed, whatever the differences.
    Printed,
    /// An input could not be read; the file and line were printed.
    Unreadable,
    /// The sets must not be compared; the differing key was printed.
    Refused,
}

impl Exit {
    /// The process exit code: 0, 1 or 2.
    pub fn code(self) -> u8 {
        match self {
            Self::Printed => 0,
            Self::Unreadable => 1,
            Self::Refused => 2,
        }
    }
}

/// The `.toml` files of one directory, sorted by name.
pub struct Listing {
    names: [Buf<NAME_CAP>; MAX_FILES],
    count: usize,
}

impl Listing {
    /// Lists `dir`.
    ///
    /// # Errors
    ///
    /// The directory cannot be read, holds a name over 128 bytes, or over 256 reports.
    pub fn read(dir: &Path) -> Result<Self, &'static str> {
        let mut listing = Self {
            names: [const { Buf::new() }; MAX_FILES],
            count: 0,
        };
        let entries = std::fs::read_dir(dir).map_err(|_| "cannot read the directory")?;
        for entry in entries {
            let entry = entry.map_err(|_| "cannot read the directory")?;
            let name = entry.file_name();
            let name = name.as_bytes();
            if !name.ends_with(b".toml") {
                continue;
            }
            let slot = listing
                .names
                .get_mut(listing.count)
                .ok_or("over 256 reports")?;
            if !slot.push(name) {
                return Err("a file name over 128 bytes");
            }
            listing.count += 1;
        }
        if let Some(names) = listing.names.get_mut(..listing.count) {
            names.sort_unstable_by(|a, b| a.as_bytes().cmp(b.as_bytes()));
        }
        Ok(listing)
    }

    /// The name of file `i`.
    pub fn name(&self, i: usize) -> &str {
        self.names.get(i).map_or("", Buf::as_str)
    }

    /// Number of files.
    pub fn len(&self) -> usize {
        self.count
    }

    /// Whether the directory holds no report.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
}

/// A set of reports: a directory, or fixtures in the tests.
pub trait Set {
    /// Where the set comes from, for messages.
    fn label(&self) -> &str;
    /// Number of reports.
    fn len(&self) -> usize;
    /// Whether the set holds no report.
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    /// Name of report `i`.
    fn name(&self, i: usize) -> &str;
    /// Loads report `i` into `report`.
    ///
    /// # Errors
    ///
    /// The report cannot be read or parsed.
    fn load(&self, i: usize, report: &mut Report) -> Result<(), LoadError>;
}

/// The reports of one directory.
pub struct Dir<'a> {
    dir: &'a str,
    listing: Listing,
}

impl<'a> Dir<'a> {
    /// Lists the reports of `dir`.
    ///
    /// # Errors
    ///
    /// See [`Listing::read`].
    pub fn open(dir: &'a str) -> Result<Self, &'static str> {
        Ok(Self {
            dir,
            listing: Listing::read(Path::new(dir))?,
        })
    }
}

impl Set for Dir<'_> {
    fn label(&self) -> &str {
        self.dir
    }

    fn len(&self) -> usize {
        self.listing.len()
    }

    fn name(&self, i: usize) -> &str {
        self.listing.name(i)
    }

    fn load(&self, i: usize, report: &mut Report) -> Result<(), LoadError> {
        let mut path = Buf::<PATH_CAP>::new();
        let fits =
            path.push(self.dir.as_bytes()) && path.push(b"/") && path.push(self.name(i).as_bytes());
        if !fits {
            return Err(LoadError::Io(io::Error::other("path too long")));
        }
        report.load(Path::new(OsStr::from_bytes(path.as_bytes())))
    }
}

/// Loads report `i` of `set`; on failure prints `<dir>/<file>: <fault>` and returns `false`.
///
/// # Errors
///
/// The sink.
pub fn load(
    set: &impl Set,
    i: usize,
    report: &mut Report,
    out: &mut impl io::Write,
) -> io::Result<bool> {
    match set.load(i, report) {
        Ok(()) => Ok(true),
        Err(e) => {
            writeln!(out, "{}/{}: {e}", set.label(), set.name(i))?;
            Ok(false)
        }
    }
}

/// Why a report is kept out of every figure, `None` when it counts.
pub fn exclusion(report: &Report) -> Option<&'static str> {
    match report.run("status").and_then(Value::text) {
        Some("complete") => {}
        Some("incomplete") => return Some("status incomplete"),
        Some("failed") => return Some("status failed"),
        Some(_) => return Some("status unknown"),
        None => return Some("no status"),
    }
    if report.has_flag("throttled") {
        Some("throttled")
    } else if report.has_flag("inconsistent") {
        Some("inconsistent")
    } else {
        None
    }
}

/// A short copy of a value, for refusals and text figures.
pub type Short = Buf<SHORT>;

/// A short copy of `value`, cut to [`SHORT`] bytes on a character boundary.
pub fn short(value: Value<'_>) -> Short {
    let mut s = Short::new();
    match value {
        Value::Int(v) => {
            let _ = write!(s, "{v}");
        }
        Value::Text(t) => {
            for c in t.chars() {
                if s.write_char(c).is_err() {
                    break;
                }
            }
        }
    }
    s
}

/// The values of one `[meta]` key that tell two sets apart, first value kept.
pub struct Guard<const K: usize> {
    keys: [&'static str; K],
    first: [Option<Short>; K],
}

impl<const K: usize> Guard<K> {
    /// A guard on `keys`, in refusal order.
    pub const fn new(keys: [&'static str; K]) -> Self {
        Self {
            keys,
            first: [const { None }; K],
        }
    }

    /// Keeps the values of `report` for every key not seen yet.
    pub fn note(&mut self, report: &Report) {
        for (key, first) in self.keys.iter().zip(self.first.iter_mut()) {
            if first.is_none() {
                *first = report.meta(key).map(short);
            }
        }
    }

    /// The first key, in order, on which `report` differs from the kept values, from index
    /// `below` down: `(index, kept, report's)`. A key absent from either side never differs.
    pub fn differs(&self, report: &Report, below: usize) -> Option<(usize, Short, Short)> {
        self.keys
            .iter()
            .zip(self.first.iter())
            .enumerate()
            .take(below)
            .find_map(|(i, (key, first))| {
                let first = first.as_ref()?;
                let theirs = short(report.meta(key)?);
                (theirs.as_bytes() != first.as_bytes()).then(|| (i, clone(first), theirs))
            })
    }

    /// Key `i`.
    pub fn key(&self, i: usize) -> &'static str {
        self.keys.get(i).copied().unwrap_or_default()
    }
}

/// A copy of a short value.
fn clone(s: &Short) -> Short {
    let mut c = Short::new();
    c.push(s.as_bytes());
    c
}

/// The first refusal found so far: key index and the two values.
pub struct Refusal(Option<(usize, Short, Short)>);

impl Refusal {
    /// None yet.
    pub const fn none() -> Self {
        Self(None)
    }

    /// Records `found` when its key comes before the one held.
    pub fn keep(&mut self, found: Option<(usize, Short, Short)>) {
        if let Some(f) = found
            && self.0.as_ref().is_none_or(|held| f.0 < held.0)
        {
            self.0 = Some(f);
        }
    }

    /// Key index up to which a new difference would still win.
    pub fn below(&self) -> usize {
        self.0.as_ref().map_or(usize::MAX, |held| held.0)
    }

    /// Prints `refused: <key> differs: <A> vs <B>` when a difference was found.
    ///
    /// # Errors
    ///
    /// The sink.
    pub fn print<const K: usize>(
        &self,
        guard: &Guard<K>,
        out: &mut impl io::Write,
    ) -> io::Result<bool> {
        match &self.0 {
            Some((i, a, b)) => {
                writeln!(
                    out,
                    "refused: {} differs: {} vs {}",
                    guard.key(*i),
                    a.as_str(),
                    b.as_str()
                )?;
                Ok(true)
            }
            None => Ok(false),
        }
    }
}

/// A measurement and cell pair, the unit repeats are counted in.
pub struct Group {
    /// `[run]` `measurement`.
    pub measurement: Short,
    /// `[run]` `cell`.
    pub cell: Short,
    /// Reports that count, per side.
    pub repeats: [usize; 2],
}

/// The groups of one or two directories.
pub struct Groups {
    groups: [Group; MAX_GROUPS],
    count: usize,
}

impl Groups {
    /// No group yet.
    pub const fn new() -> Self {
        Self {
            groups: [const {
                Group {
                    measurement: Buf::new(),
                    cell: Buf::new(),
                    repeats: [0; 2],
                }
            }; MAX_GROUPS],
            count: 0,
        }
    }

    /// The index of the group of `report`, added when new; `None` when the table is full.
    pub fn of(&mut self, report: &Report) -> Option<usize> {
        let m = short(Value::Text(report.run_text("measurement")));
        let c = short(Value::Text(report.run_text("cell")));
        let found = self.all().iter().position(|g| {
            g.measurement.as_bytes() == m.as_bytes() && g.cell.as_bytes() == c.as_bytes()
        });
        if found.is_some() {
            return found;
        }
        let slot = self.groups.get_mut(self.count)?;
        *slot = Group {
            measurement: m,
            cell: c,
            repeats: [0; 2],
        };
        self.count += 1;
        Some(self.count - 1)
    }

    /// Group `i`.
    pub fn get_mut(&mut self, i: usize) -> Option<&mut Group> {
        self.groups.get_mut(..self.count)?.get_mut(i)
    }

    /// Every group, in the order first seen.
    pub fn all(&self) -> &[Group] {
        self.groups.get(..self.count).unwrap_or_default()
    }

    /// Group indices sorted by measurement, then cell.
    pub fn sorted(&self) -> ([usize; MAX_GROUPS], usize) {
        let mut order = [0; MAX_GROUPS];
        for (i, o) in order.iter_mut().enumerate() {
            *o = i;
        }
        if let Some(order) = order.get_mut(..self.count) {
            order.sort_unstable_by(|&a, &b| {
                let (a, b) = (&self.groups[a], &self.groups[b]);
                (a.measurement.as_bytes(), a.cell.as_bytes())
                    .cmp(&(b.measurement.as_bytes(), b.cell.as_bytes()))
            });
        }
        (order, self.count)
    }
}

impl Default for Groups {
    fn default() -> Self {
        Self::new()
    }
}

/// Where one report file belongs.
#[derive(Clone, Copy)]
pub struct FileInfo {
    /// Its group.
    pub group: usize,
    /// Why it does not count, `None` when it does.
    pub excluded: Option<&'static str>,
}

impl FileInfo {
    /// A placeholder for the unused slots.
    pub const EMPTY: Self = Self {
        group: 0,
        excluded: None,
    };
}

/// Up to 16 repeats of one figure in one column.
pub struct Cell {
    values: [u64; MAX_REPEATS],
    n: usize,
    text: Option<Short>,
}

impl Cell {
    /// An empty cell.
    const EMPTY: Self = Self {
        values: [0; MAX_REPEATS],
        n: 0,
        text: None,
    };

    /// Median, spread and count of the integer values, `None` when there is none.
    pub fn stat(&self) -> Option<Stat> {
        let mut sorted = self.values;
        let values = sorted.get_mut(..self.n)?;
        values.sort_unstable();
        Some(Stat {
            median: *values.get((self.n.checked_sub(1)?) / 2)?,
            spread: values.last()? - values.first()?,
            n: self.n,
        })
    }

    /// The text of the first report that gave one.
    pub fn text(&self) -> Option<&str> {
        self.text.as_ref().map(Buf::as_str)
    }
}

/// One column's view of one figure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stat {
    /// Median; for an even count the lower middle value.
    pub median: u64,
    /// Max − min.
    pub spread: u64,
    /// Values it was taken over.
    pub n: usize,
}

/// Figures of several columns side by side, one row per key, rows in the order first seen.
pub struct Table<const COLS: usize> {
    keys: [Buf<MAX_KEY>; MAX_KEYS],
    cells: [[Cell; COLS]; MAX_KEYS],
    rows: usize,
}

impl<const COLS: usize> Table<COLS> {
    /// An empty table.
    pub const fn new() -> Self {
        Self {
            keys: [const { Buf::new() }; MAX_KEYS],
            cells: [const { [Cell::EMPTY; COLS] }; MAX_KEYS],
            rows: 0,
        }
    }

    /// Empties the table.
    pub fn clear(&mut self) {
        self.rows = 0;
        for row in &mut self.cells {
            for cell in row {
                cell.n = 0;
                cell.text = None;
            }
        }
    }

    /// Adds the `[figures]` of `report` to column `col`; beyond 256 rows or 16 repeats a value
    /// is dropped and `false` returned.
    pub fn add(&mut self, col: usize, report: &Report) -> bool {
        let mut kept = true;
        for (key, value) in report.entries(Block::Figures) {
            let row = match self.row(key) {
                Some(row) => row,
                None => {
                    kept = false;
                    continue;
                }
            };
            let Some(cell) = self.cells.get_mut(row).and_then(|r| r.get_mut(col)) else {
                kept = false;
                continue;
            };
            match value {
                Value::Int(v) => match cell.values.get_mut(cell.n) {
                    Some(slot) => {
                        *slot = v;
                        cell.n += 1;
                    }
                    None => kept = false,
                },
                Value::Text(_) => {
                    if cell.text.is_none() {
                        cell.text = Some(short(value));
                    }
                }
            }
        }
        kept
    }

    /// The rows: key and the cells of every column.
    pub fn rows(&self) -> impl Iterator<Item = (&str, &[Cell; COLS])> {
        self.keys
            .iter()
            .zip(self.cells.iter())
            .take(self.rows)
            .map(|(k, c)| (k.as_str(), c))
    }

    /// The row of `key`, added when new.
    fn row(&mut self, key: &str) -> Option<usize> {
        if let Some(i) = self
            .keys
            .get(..self.rows)?
            .iter()
            .position(|k| k.as_str() == key)
        {
            return Some(i);
        }
        let slot = self.keys.get_mut(self.rows)?;
        slot.clear();
        if !slot.push(key.as_bytes()) {
            return None;
        }
        self.rows += 1;
        Some(self.rows - 1)
    }
}

impl<const COLS: usize> Default for Table<COLS> {
    fn default() -> Self {
        Self::new()
    }
}

/// Space-pads `text` to `width` characters on `out`.
fn padded(out: &mut impl io::Write, text: &str, width: usize) -> io::Result<()> {
    write!(out, "{text}")?;
    for _ in text.chars().count()..width {
        out.write_all(b" ")?;
    }
    Ok(())
}

/// Reports held in memory, and a sink that keeps what was printed: for the tests.
#[cfg(test)]
pub(crate) mod fixture {
    use super::parse::{LoadError, Report};
    use crate::buf::Buf;
    use std::fmt::Write as _;
    use std::io;

    /// A set of `(name, text)` reports.
    pub(crate) struct Fixture<'a> {
        /// The set's label.
        pub label: &'a str,
        /// Its reports.
        pub files: &'a [(&'a str, &'a str)],
    }

    impl super::Set for Fixture<'_> {
        fn label(&self) -> &str {
            self.label
        }

        fn len(&self) -> usize {
            self.files.len()
        }

        fn name(&self, i: usize) -> &str {
            self.files.get(i).map_or("", |f| f.0)
        }

        fn load(&self, i: usize, report: &mut Report) -> Result<(), LoadError> {
            report
                .parse(self.files.get(i).map_or("", |f| f.1).as_bytes())
                .map_err(LoadError::Parse)
        }
    }

    /// What the tool printed.
    pub(crate) struct Out(pub Buf<32768>);

    impl io::Write for Out {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            if self.0.push(bytes) {
                Ok(bytes.len())
            } else {
                Err(io::Error::other("full"))
            }
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    /// The `[meta]` defaults of [`report`].
    const META: [(&str, &str); 6] = [
        ("schema", "1"),
        ("host_id", "\"dev\""),
        ("robot", "\"XmBench\""),
        ("robot_n", "4"),
        ("kernel_rt", "\"no\""),
        ("reference", "\"no\""),
    ];

    /// A report: `[meta]` defaults with `meta` replacing or adding keys, `[run]` of
    /// `measurement`, `cell`, `status`, `flags`, then `figures` lines when complete.
    pub(crate) fn report(
        meta: &[(&str, &str)],
        run: (&str, &str, &str, &str),
        figures: &str,
    ) -> Buf<2048> {
        let mut t = Buf::new();
        let _ = writeln!(t, "[meta]");
        for (key, value) in META {
            let value = meta.iter().find(|m| m.0 == key).map_or(value, |m| m.1);
            let _ = writeln!(t, "{key} = {value}");
        }
        for (key, value) in meta {
            if !META.iter().any(|m| m.0 == *key) {
                let _ = writeln!(t, "{key} = {value}");
            }
        }
        let (measurement, cell, status, flags) = run;
        let _ = write!(
            t,
            "[run]\nmeasurement = \"{measurement}\"\ncell = \"{cell}\"\nstatus = \"{status}\"\nflags = \"{flags}\"\n"
        );
        if status == "complete" {
            let _ = write!(t, "[figures]\n{figures}");
        }
        t
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(values: &[u64]) -> Cell {
        let mut c = Cell::EMPTY;
        for (slot, v) in c.values.iter_mut().zip(values) {
            *slot = *v;
        }
        c.n = values.len();
        c
    }

    #[test]
    fn median_and_spread_on_1_2_3_and_16_repeats() {
        assert_eq!(
            cell(&[7]).stat(),
            Some(Stat {
                median: 7,
                spread: 0,
                n: 1
            })
        );
        assert_eq!(
            cell(&[9, 4]).stat(),
            Some(Stat {
                median: 4,
                spread: 5,
                n: 2
            })
        );
        assert_eq!(
            cell(&[30, 10, 20]).stat(),
            Some(Stat {
                median: 20,
                spread: 20,
                n: 3
            })
        );
        let sixteen: [u64; 16] = core::array::from_fn(|i| 100 - i as u64);
        assert_eq!(
            cell(&sixteen).stat(),
            Some(Stat {
                median: 92,
                spread: 15,
                n: 16
            })
        );
        assert_eq!(cell(&[]).stat(), None);
    }

    #[test]
    fn exclusion_follows_status_then_flags() {
        let mut r = Report::new();
        for (run, why) in [
            ("status = \"complete\"\nflags = \"simulated\"", None),
            (
                "status = \"incomplete\"\nflags = \"\"",
                Some("status incomplete"),
            ),
            (
                "status = \"failed\"\nflags = \"throttled\"",
                Some("status failed"),
            ),
            (
                "status = \"complete\"\nflags = \"simulated,throttled\"",
                Some("throttled"),
            ),
            (
                "status = \"complete\"\nflags = \"inconsistent\"",
                Some("inconsistent"),
            ),
            ("flags = \"\"", Some("no status")),
        ] {
            let mut text = Buf::<256>::new();
            let _ = write!(text, "[meta]\nschema = 1\n[run]\n{run}\n");
            r.parse(text.as_bytes()).expect("fixture");
            assert_eq!(exclusion(&r), why, "{run}");
        }
    }
}
