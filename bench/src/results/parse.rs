//! Reader of one report into fixed arrays (contracts/results-schema.md, schema 1).
//!
//! The file is copied into a 64 KiB buffer; each key keeps byte spans into it.
//! A key the reader does not know is kept: adding a key is compatible.

use std::fmt;
use std::fs::File;
use std::io::{self, Read};
use std::path::Path;

use crate::report::{MAX_FILE, MAX_KEY, MAX_KEYS, MAX_TEXT};

/// The three blocks of a report.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Block {
    /// Host and build, written by `meta.sh`.
    Meta,
    /// Plan and status, written by the binary.
    Run,
    /// Measured figures of a complete run.
    Figures,
}

impl Block {
    /// The block named by a `[name]` line.
    fn named(name: &[u8]) -> Option<Self> {
        match name {
            b"meta" => Some(Self::Meta),
            b"run" => Some(Self::Run),
            b"figures" => Some(Self::Figures),
            _ => None,
        }
    }
}

/// One value: `<integer>` or `"<text>"`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Value<'a> {
    /// Unsigned base-10 integer.
    Int(u64),
    /// Text, quotes removed.
    Text(&'a str),
}

impl<'a> Value<'a> {
    /// The integer, `None` for text.
    pub fn int(self) -> Option<u64> {
        match self {
            Self::Int(v) => Some(v),
            Self::Text(_) => None,
        }
    }

    /// The text, `None` for an integer.
    pub fn text(self) -> Option<&'a str> {
        match self {
            Self::Int(_) => None,
            Self::Text(t) => Some(t),
        }
    }
}

impl fmt::Display for Value<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Int(v) => write!(f, "{v}"),
            Self::Text(t) => f.write_str(t),
        }
    }
}

/// Why a report could not be read: the line (0 for the whole file) and what was wrong.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ParseError {
    /// 1-based line, 0 when the fault is the file's.
    pub line: usize,
    /// What was wrong.
    pub what: &'static str,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.line == 0 {
            f.write_str(self.what)
        } else {
            write!(f, "line {}: {}", self.line, self.what)
        }
    }
}

/// Why a report file could not be loaded.
#[derive(Debug)]
pub enum LoadError {
    /// Opening or reading the file.
    Io(io::Error),
    /// Its content.
    Parse(ParseError),
}

impl fmt::Display for LoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(e) => write!(f, "{e}"),
            Self::Parse(e) => write!(f, "{e}"),
        }
    }
}

/// A key's value as stored: an integer, or a span of the text.
#[derive(Clone, Copy)]
enum Raw {
    /// Integer value.
    Int(u64),
    /// Text value, start and end in the file.
    Text(u32, u32),
}

/// One `key = value` line.
#[derive(Clone, Copy)]
struct Entry {
    /// Its block.
    block: Block,
    /// Key span in the file.
    key: (u32, u32),
    /// Its value.
    value: Raw,
}

impl Entry {
    /// A placeholder for the unused slots.
    const EMPTY: Self = Self {
        block: Block::Meta,
        key: (0, 0),
        value: Raw::Int(0),
    };
}

/// One parsed report.
pub struct Report {
    bytes: [u8; MAX_FILE],
    len: usize,
    entries: [Entry; MAX_KEYS],
    count: usize,
}

impl Report {
    /// An empty report; about 68 KiB.
    pub const fn new() -> Self {
        Self {
            bytes: [0; MAX_FILE],
            len: 0,
            entries: [Entry::EMPTY; MAX_KEYS],
            count: 0,
        }
    }

    /// Reads and parses the file at `path`, replacing what was held.
    ///
    /// # Errors
    ///
    /// The file cannot be read, is over 64 KiB, or a line breaks the format.
    pub fn load(&mut self, path: &Path) -> Result<(), LoadError> {
        self.clear();
        let mut file = File::open(path).map_err(LoadError::Io)?;
        let mut len = 0;
        while let Some(room) = self.bytes.get_mut(len..) {
            if room.is_empty() {
                let mut probe = [0u8; 1];
                if file.read(&mut probe).map_err(LoadError::Io)? > 0 {
                    return Err(LoadError::Parse(TOO_LARGE));
                }
                break;
            }
            match file.read(room) {
                Ok(0) => break,
                Ok(n) => len += n,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) => return Err(LoadError::Io(e)),
            }
        }
        self.len = len;
        self.index().map_err(LoadError::Parse)
    }

    /// Parses `text`, replacing what was held.
    ///
    /// # Errors
    ///
    /// The text is over 64 KiB or a line breaks the format.
    pub fn parse(&mut self, text: &[u8]) -> Result<(), ParseError> {
        self.clear();
        let room = self.bytes.get_mut(..text.len()).ok_or(TOO_LARGE)?;
        room.copy_from_slice(text);
        self.len = text.len();
        self.index()
    }

    /// The value of `key` in `block`.
    pub fn get(&self, block: Block, key: &str) -> Option<Value<'_>> {
        self.entries(block)
            .find(|(k, _)| *k == key)
            .map(|(_, value)| value)
    }

    /// The value of a `[meta]` key.
    pub fn meta(&self, key: &str) -> Option<Value<'_>> {
        self.get(Block::Meta, key)
    }

    /// The value of a `[run]` key.
    pub fn run(&self, key: &str) -> Option<Value<'_>> {
        self.get(Block::Run, key)
    }

    /// The text of a `[run]` key, empty when absent or an integer.
    pub fn run_text(&self, key: &str) -> &str {
        self.run(key).and_then(Value::text).unwrap_or_default()
    }

    /// Every key of `block`, in file order.
    pub fn entries(&self, block: Block) -> impl Iterator<Item = (&str, Value<'_>)> {
        self.entries
            .get(..self.count)
            .unwrap_or_default()
            .iter()
            .filter(move |e| e.block == block)
            .map(|e| (self.str_at(e.key), self.value(e.value)))
    }

    /// Whether `[run]` `flags` holds `flag`.
    pub fn has_flag(&self, flag: &str) -> bool {
        self.run_text("flags").split(',').any(|f| f == flag)
    }

    /// Drops what was held.
    fn clear(&mut self) {
        self.len = 0;
        self.count = 0;
    }

    /// The stored value as a [`Value`].
    fn value(&self, raw: Raw) -> Value<'_> {
        match raw {
            Raw::Int(v) => Value::Int(v),
            Raw::Text(start, end) => Value::Text(self.str_at((start, end))),
        }
    }

    /// The text of a span; spans are made on UTF-8 boundaries by [`Report::index`].
    fn str_at(&self, (start, end): (u32, u32)) -> &str {
        let (Ok(start), Ok(end)) = (usize::try_from(start), usize::try_from(end)) else {
            return "";
        };
        self.bytes
            .get(start..end)
            .and_then(|b| std::str::from_utf8(b).ok())
            .unwrap_or_default()
    }

    /// Splits the held bytes into entries.
    fn index(&mut self) -> Result<(), ParseError> {
        let text = self.bytes.get(..self.len).unwrap_or_default();
        if let Err(e) = std::str::from_utf8(text) {
            let line = text
                .get(..e.valid_up_to())
                .unwrap_or_default()
                .iter()
                .filter(|&&b| b == b'\n')
                .count();
            return Err(ParseError {
                line: line + 1,
                what: "not UTF-8",
            });
        }
        let mut block = None;
        let mut seen = [false; 3];
        let mut offset = 0;
        for (n, line) in text.split(|&b| b == b'\n').enumerate() {
            let at = |what| ParseError { line: n + 1, what };
            let start = offset;
            offset += line.len() + 1;
            let trimmed = line.trim_ascii();
            if trimmed.is_empty() || trimmed.starts_with(b"#") {
                continue;
            }
            if let Some(name) = trimmed
                .strip_prefix(b"[")
                .and_then(|rest| rest.strip_suffix(b"]"))
            {
                let b = Block::named(name).ok_or(at("unknown block"))?;
                let slot = seen.get_mut(b as usize).ok_or(at("unknown block"))?;
                if *slot {
                    return Err(at("block repeated"));
                }
                *slot = true;
                block = Some(b);
                continue;
            }
            let Some(eq) = line.iter().position(|&b| b == b'=') else {
                return Err(at("not a key line"));
            };
            let key = line.get(..eq).unwrap_or_default();
            let lead = key.len() - key.trim_ascii_start().len();
            let key = key.trim_ascii();
            if key.is_empty()
                || key.len() > MAX_KEY
                || !key
                    .iter()
                    .all(|&b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'_')
            {
                return Err(at("bad key"));
            }
            let block = block.ok_or(at("key outside a block"))?;
            let raw = line.get(eq + 1..).unwrap_or_default();
            let vlead = raw.len() - raw.trim_ascii_start().len();
            let raw = raw.trim_ascii();
            let value = if let Some(inner) = raw
                .strip_prefix(b"\"")
                .and_then(|rest| rest.strip_suffix(b"\""))
            {
                if inner.contains(&b'"') || inner.len() > MAX_TEXT {
                    return Err(at("bad text"));
                }
                let s = start + eq + 1 + vlead + 1;
                Raw::Text(span(s), span(s + inner.len()))
            } else {
                Raw::Int(integer(raw).ok_or(at("bad value"))?)
            };
            let key = (span(start + lead), span(start + lead + key.len()));
            let dup = self
                .entries
                .get(..self.count)
                .unwrap_or_default()
                .iter()
                .any(|e| e.block == block && bytes_at(text, e.key) == bytes_at(text, key));
            if dup {
                return Err(at("key repeated"));
            }
            let slot = self
                .entries
                .get_mut(self.count)
                .ok_or(at("over 256 keys"))?;
            *slot = Entry { block, key, value };
            self.count += 1;
        }
        if !seen[Block::Meta as usize] {
            return Err(ParseError {
                line: 0,
                what: "no [meta] block",
            });
        }
        Ok(())
    }
}

impl Default for Report {
    fn default() -> Self {
        Self::new()
    }
}

/// The fault of a file over the size limit.
const TOO_LARGE: ParseError = ParseError {
    line: 0,
    what: "over 64 KiB",
};

/// An offset as stored; a file is at most 64 KiB, so it always fits.
fn span(offset: usize) -> u32 {
    u32::try_from(offset).unwrap_or(u32::MAX)
}

/// The bytes of a span.
fn bytes_at(text: &[u8], (start, end): (u32, u32)) -> &[u8] {
    let (Ok(start), Ok(end)) = (usize::try_from(start), usize::try_from(end)) else {
        return &[];
    };
    text.get(start..end).unwrap_or_default()
}

/// An unsigned base-10 integer that fits `u64`.
fn integer(digits: &[u8]) -> Option<u64> {
    if digits.is_empty() {
        return None;
    }
    digits.iter().try_fold(0u64, |acc, &b| {
        if b.is_ascii_digit() {
            acc.checked_mul(10)?.checked_add(u64::from(b - b'0'))
        } else {
            None
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::buf::Buf;
    use std::fmt::Write as _;

    const GOOD: &str = "\
[meta]
schema = 1
host_id = \"rpi4-c03114\"
# a comment

[run]
measurement = \"jitter\"
status = \"complete\"
flags = \"simulated,not_reference\"
[figures]
jitter_p50_ns = 55900
future_key = \"kept\"
";

    #[test]
    fn a_good_file_reads_every_block() {
        let mut r = Report::new();
        r.parse(GOOD.as_bytes()).expect("good file");
        assert_eq!(r.meta("schema"), Some(Value::Int(1)));
        assert_eq!(r.meta("host_id"), Some(Value::Text("rpi4-c03114")));
        assert_eq!(r.run_text("status"), "complete");
        assert!(r.has_flag("not_reference"));
        assert!(!r.has_flag("throttled"));
        assert_eq!(
            r.get(Block::Figures, "jitter_p50_ns"),
            Some(Value::Int(55_900))
        );
        assert_eq!(
            r.get(Block::Figures, "future_key"),
            Some(Value::Text("kept"))
        );
        assert_eq!(r.get(Block::Run, "jitter_p50_ns"), None);
        assert_eq!(r.entries(Block::Run).count(), 3);
    }

    #[test]
    fn a_missing_meta_block_is_refused() {
        let mut r = Report::new();
        let e = r
            .parse(b"[run]\nstatus = \"failed\"\n")
            .expect_err("no meta");
        assert_eq!(e.to_string(), "no [meta] block");
    }

    #[test]
    fn a_bad_line_names_its_line() {
        for (text, line, what) in [
            ("[meta]\nschema = -1\n", 2, "bad value"),
            ("[meta]\nschema = 18446744073709551616\n", 2, "bad value"),
            ("[meta]\n\nhost_id = \"a\"b\"\n", 3, "bad text"),
            ("[meta]\nhost id\n", 2, "not a key line"),
            ("[meta]\nHost = 1\n", 2, "bad key"),
            ("schema = 1\n[meta]\n", 1, "key outside a block"),
            ("[meta]\n[extra]\n", 2, "unknown block"),
            ("[meta]\n[meta]\n", 2, "block repeated"),
            ("[meta]\na = 1\na = 2\n", 3, "key repeated"),
        ] {
            let mut r = Report::new();
            let e = r.parse(text.as_bytes()).expect_err(text);
            assert_eq!((e.line, e.what), (line, what), "{text}");
        }
        let mut r = Report::new();
        let e = r.parse(b"[meta]\na = \"\xff\"\n").expect_err("not utf-8");
        assert_eq!(e.to_string(), "line 2: not UTF-8");
    }

    #[test]
    fn the_limits_hold() {
        let mut r = Report::new();
        let mut text = Buf::<{ 2 * MAX_FILE }>::new();
        let _ = writeln!(text, "[meta]");
        for i in 0..MAX_KEYS {
            let _ = writeln!(text, "k{i} = {i}");
        }
        r.parse(text.as_bytes()).expect("256 keys");
        let _ = writeln!(text, "one_more = 1");
        let e = r.parse(text.as_bytes()).expect_err("257 keys");
        assert_eq!((e.line, e.what), (MAX_KEYS + 2, "over 256 keys"));

        let filler = [b'k'; MAX_TEXT + 1];
        for (head, n, tail, ok) in [
            (&b"[meta]\n"[..], MAX_KEY, &b" = 1\n"[..], true),
            (b"[meta]\n", MAX_KEY + 1, b" = 1\n", false),
            (b"[meta]\nt = \"", MAX_TEXT, b"\"\n", true),
            (b"[meta]\nt = \"", MAX_TEXT + 1, b"\"\n", false),
        ] {
            text.clear();
            text.push(head);
            text.push(filler.get(..n).unwrap_or_default());
            text.push(tail);
            assert_eq!(r.parse(text.as_bytes()).is_ok(), ok, "{n}");
        }

        text.clear();
        let _ = writeln!(text, "[meta]");
        while text.len() <= MAX_FILE {
            let _ = writeln!(text, "# padding to go over the file limit");
        }
        assert_eq!(r.parse(text.as_bytes()), Err(TOO_LARGE));
    }
}
