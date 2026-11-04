//! `results`: compares two report sets or summarizes one (contracts/results-schema.md "Tools").
//!
//! Exit 0 when printed, whatever the differences; 1 for an unreadable input; 2 for a refusal.

use std::io::{self, Write as _};
use std::process::ExitCode;

use kmr_bench::results::compare::compare;
use kmr_bench::results::summary::summary;
use kmr_bench::results::{Dir, Exit};

/// Printed on a bad command line.
const USAGE: &str = "usage: results compare <dir A> <dir B> | results summary <dir>";

fn main() -> ExitCode {
    let args: [Option<std::ffi::OsString>; 4] = {
        let mut it = std::env::args_os().skip(1);
        core::array::from_fn(|_| it.next())
    };
    let arg = |i: usize| {
        args.get(i)
            .and_then(|a| a.as_ref())
            .and_then(|a| a.to_str())
    };
    let mut out = io::stdout().lock();
    let exit = match (
        arg(0),
        arg(1),
        arg(2),
        args.get(3).is_some_and(Option::is_some),
    ) {
        (Some("compare"), Some(a), Some(b), false) => match (open(a), open(b)) {
            (Some(a), Some(b)) => compare(&a, &b, &mut out),
            _ => Ok(Exit::Unreadable),
        },
        (Some("summary"), Some(dir), None, false) => match open(dir) {
            Some(dir) => summary(&dir, &mut out),
            None => Ok(Exit::Unreadable),
        },
        _ => {
            eprintln!("{USAGE}");
            Ok(Exit::Unreadable)
        }
    };
    let exit = exit.and_then(|e| out.flush().map(|()| e));
    ExitCode::from(exit.map_or(Exit::Unreadable, |e| e).code())
}

/// Lists the reports of `dir`; on failure prints `<dir>: <fault>`.
fn open(dir: &str) -> Option<Dir<'_>> {
    Dir::open(dir)
        .inspect_err(|what| println!("{dir}: {what}"))
        .ok()
}
