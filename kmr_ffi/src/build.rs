//! The build-script helper (feature `build`): headers, ABI file and the C/C++ compile.
//!
//! Run from the user's `build.rs` (D12 A, decision 0009): [`robot`] or [`model`] names the
//! robot again, and kmr_model_core gives the same N as `robot!`. Each [`Build::compile`] call is
//! one group: it writes `kmr.h` (C11) and `kmr.hpp` (C++17) for one N and one set of engine
//! [`Sizes`] into a folder of its own under `OUT_DIR`, compiles the user's sources against them
//! with `cc`, and lists the functions the group defines. `kmr_ffi_abi.rs` in `OUT_DIR` maps every
//! function of every group to the N and sizes it was built for; `kmr_api::c!` looks its function
//! up there, so a function built for another N fails the build.
//!
//! The headers expose the controller API over the snapshot and nothing else: no builder, no
//! loop, no phase, no signal bus internals, no HAL (FR-002 (SRS FR-10.1)).
// Build-script code, never on the control path (FR-019 (SRS FR-10.3)).
#![allow(clippy::disallowed_types)]

use std::ffi::OsString;
use std::path::{Component, Path, PathBuf};
use std::process::Command;
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

/// What can go wrong in the build-script helper.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// No robot preset of that name; `robot!`'s message, listing every preset.
    #[error("{0}")]
    UnknownPreset(String),
    /// The robot descriptor could not be read or is invalid; `robot!`'s message.
    #[error("{0}")]
    Descriptor(String),
    /// N is outside `1..=64`, the actuators one [`Mask`](crate::Mask) holds.
    #[error("kmr_ffi: N = {0} is outside 1..=64, one mask bit per actuator")]
    ActuatorCount(usize),
    /// A source is neither C (`.c`) nor C++ (`.cc`, `.cpp`, `.cxx`).
    #[error("kmr_ffi: `{0}` is neither C nor C++")]
    UnknownSource(String),
    /// `OUT_DIR` is unset: the helper runs outside a build script.
    #[error("kmr_ffi: OUT_DIR is unset; call this from build.rs")]
    NoOutDir,
    /// A header or the ABI file could not be written.
    #[error("kmr_ffi: cannot write {}", .0.display())]
    Write(PathBuf, #[source] std::io::Error),
    /// The C/C++ compiler failed.
    #[error("kmr_ffi: {0}")]
    Compile(String),
    /// No `nm` could list the functions of a compiled library.
    #[error(
        "kmr_ffi: no nm could list the functions of the compiled C/C++ (tried {}): {detail}; \
         install binutils or LLVM, or set NM to the nm of the target's toolchain",
        .tried.join(", ")
    )]
    NoSymbolTool {
        /// The tools tried, in order.
        tried: Vec<String>,
        /// Why the last one failed.
        detail: String,
    },
    /// Two `compile` calls define the same function: the linker would keep one for both.
    #[error(
        "kmr_ffi: `{symbol}` is defined by two compile calls, one for [{}] and one for [{}]; the \
         linker would keep one of them for both robots. Give each function its own name, and \
         compile a source shared by both robots in one call only",
        .first.join(", "),
        .second.join(", ")
    )]
    DuplicateSymbol {
        /// The function's symbol.
        symbol: String,
        /// The sources of the earlier call.
        first: Vec<String>,
        /// The sources of the later call.
        second: Vec<String>,
    },
}

/// The engine's fixed sizes the headers are written for; kmr_api fills them from the core.
// DECISION: passed in, not looked up: kmr_ffi keeps no engine dependency (research R9, decision
// 0009), and the core names each of them once.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sizes {
    /// History slots, the core's `HISTORY_DEPTH`.
    pub history_depth: usize,
    /// IMU slots, the core's `MAX_IMUS`.
    pub max_imus: usize,
    /// Microphone slots, the core's `MAX_MICS`.
    pub max_mics: usize,
    /// Samples a microphone slot holds per tick, the core's `MIC_WINDOW`.
    pub mic_window: usize,
}

/// Where a build's N comes from, read when [`Build::compile`] runs.
enum Robot {
    /// A preset shipped in kmr_model_core, by name.
    Preset(String),
    /// A descriptor file, relative to the calling crate's `Cargo.toml`.
    Model(PathBuf),
    /// N given as is.
    #[cfg(test)]
    Count(usize),
}

/// A build of the user's C and C++ sources for one robot and one set of engine sizes.
pub struct Build {
    /// The robot whose actuator count is `robot!`'s N.
    robot: Robot,
    /// The engine's sizes.
    sizes: Sizes,
}

/// A build for the preset named `name`, the robot `robot!(name)` gives.
pub fn robot(name: &str, sizes: Sizes) -> Build {
    Build {
        robot: Robot::Preset(name.to_owned()),
        sizes,
    }
}

/// A build for the descriptor at `path`, the robot `robot!(model = path)` gives.
///
/// A relative `path` is relative to the calling crate's `Cargo.toml`, as in `robot!`.
pub fn model(path: impl AsRef<Path>, sizes: Sizes) -> Build {
    Build {
        robot: Robot::Model(path.as_ref().to_path_buf()),
        sizes,
    }
}

impl Build {
    /// A build for `n` actuators.
    #[cfg(test)]
    pub(crate) fn new(n: usize, sizes: Sizes) -> Self {
        Self {
            robot: Robot::Count(n),
            sizes,
        }
    }

    /// N, from kmr_model_core: the same reader and checks `robot!` runs.
    fn n(&self) -> Result<usize, Error> {
        match &self.robot {
            Robot::Preset(name) => kmr_model_core::preset(name).map(|r| r.n()).map_err(|msg| {
                if kmr_model_core::preset_names().any(|p| p == name) {
                    Error::Descriptor(msg)
                } else {
                    Error::UnknownPreset(msg)
                }
            }),
            Robot::Model(path) => {
                let path = match std::env::var_os("CARGO_MANIFEST_DIR") {
                    Some(dir) => Path::new(&dir).join(path),
                    None if path.is_absolute() => path.clone(),
                    None => {
                        return Err(Error::Descriptor(
                            "kmr: CARGO_MANIFEST_DIR is not set; build with cargo".into(),
                        ));
                    }
                };
                println!("cargo:rerun-if-changed={}", path.display());
                kmr_model_core::descriptor(&path)
                    .map(|r| r.n())
                    .map_err(Error::Descriptor)
            }
            #[cfg(test)]
            Robot::Count(n) => Ok(*n),
        }
    }

    /// Compiles `sources` for this robot, as one group of its own.
    ///
    /// Call it once per robot. `.c` files are compiled as C11; `.cc`, `.cpp` and `.cxx` files as
    /// C++17, each against the `kmr.h` and `kmr.hpp` of this call. Cargo reruns the build when a
    /// file in a source's folder changes, so headers next to the sources count; a header
    /// elsewhere needs its own `cargo:rerun-if-changed`. Two calls defining the same function is
    /// [`Error::DuplicateSymbol`].
    pub fn compile(&self, sources: &[&str]) -> Result<(), Error> {
        let n = self.n()?;
        if !(1..=crate::Mask::BITS as usize).contains(&n) {
            return Err(Error::ActuatorCount(n));
        }
        let (c, cpp) = split(sources)?;
        let out = std::env::var_os("OUT_DIR").ok_or(Error::NoOutDir)?;
        let out = Path::new(&out);
        let k = NEXT_GROUP.fetch_add(1, Ordering::Relaxed);
        let include = out.join("kmr").join(k.to_string());
        std::fs::create_dir_all(&include).map_err(|e| Error::Write(include.clone(), e))?;
        write(&include.join("kmr.h"), &c_header(n, self.sizes))?;
        write(&include.join("kmr.hpp"), &cpp_header())?;
        for path in watched(sources) {
            println!("cargo:rerun-if-changed={}", path.display());
        }
        println!("cargo:rerun-if-env-changed=NM");
        // DECISION: every controller has the four parameters of `kmr.h`; one it does not use
        // is not worth a warning. `OUT_DIR` stays an include dir after the group's own, so a
        // header the build script writes there (the embedded URDF of the Pinocchio example) is
        // still found; the group's `kmr.h` comes first.
        let mut libs = Vec::new();
        for (lang, files) in [("c", &c), ("cpp", &cpp)] {
            if files.is_empty() {
                continue;
            }
            let mut build = cc::Build::new();
            if lang == "cpp" {
                build.cpp(true).std("c++17");
            } else {
                build.std("c11");
            }
            build
                .include(&include)
                .include(out)
                .flag_if_supported("-Wno-unused-parameter")
                .files(files.iter());
            let name = format!("kmr_ffi_{lang}_{k}");
            build
                .try_compile(&name)
                .map_err(|e| Error::Compile(e.to_string()))?;
            let archiver = build
                .try_get_archiver()
                .ok()
                .map(|cmd| PathBuf::from(cmd.get_program()));
            libs.push((library(out, &name), archiver));
        }
        let mut symbols = Vec::new();
        for (lib, archiver) in &libs {
            let tools = nm_tools(std::env::var_os("NM"), archiver.as_deref());
            symbols.extend(list_symbols(&tools, lib)?);
        }
        let apple = std::env::var("CARGO_CFG_TARGET_VENDOR").is_ok_and(|v| v == "apple");
        if apple {
            for symbol in &mut symbols {
                if let Some(bare) = symbol.strip_prefix('_') {
                    *symbol = bare.to_owned();
                }
            }
        }
        symbols.sort();
        symbols.dedup();
        let group = Group {
            sources: sources.iter().map(|s| (*s).to_owned()).collect(),
            abi: abi(n, self.sizes),
            symbols,
        };
        let mut groups = GROUPS.lock().unwrap_or_else(|e| e.into_inner());
        check_duplicates(&groups, &group)?;
        groups.push(group);
        write(&out.join("kmr_ffi_abi.rs"), &abi_table(&groups))
    }
}

/// The number of the next [`Build::compile`] call of this build script.
static NEXT_GROUP: AtomicUsize = AtomicUsize::new(0);

/// Every group this build script compiled so far, in call order.
// DECISION: process-wide: a build script is one process, and its `compile` calls are the groups.
static GROUPS: Mutex<Vec<Group>> = Mutex::new(Vec::new());

/// One [`Build::compile`] call: its sources, the N and sizes it was built for, and the
/// functions its libraries define.
struct Group {
    /// The sources as given.
    sources: Vec<String>,
    /// N, History depth, IMU slots, microphone slots and microphone window.
    abi: [usize; 5],
    /// The strong global functions it defines, sorted.
    symbols: Vec<String>,
}

/// N and the sizes, in the order of `kmr_ffi_abi.rs`.
fn abi(n: usize, sizes: Sizes) -> [usize; 5] {
    [
        n,
        sizes.history_depth,
        sizes.max_imus,
        sizes.max_mics,
        sizes.mic_window,
    ]
}

/// The static library `cc` left in `out` for `name`.
fn library(out: &Path, name: &str) -> PathBuf {
    let unix = out.join(format!("lib{name}.a"));
    if unix.exists() {
        return unix;
    }
    out.join(format!("{name}.lib"))
}

/// The `nm` beside `archiver`: its last `ar` word swapped for `nm`, its folder kept.
///
/// `aarch64-linux-gnu-ar` gives `aarch64-linux-gnu-nm`, `llvm-ar-18` gives `llvm-nm-18`; a name
/// without an `ar` word (`emar`, `lib.exe`) gives none.
fn nm_beside(archiver: &Path) -> Option<PathBuf> {
    let name = archiver.file_name()?.to_str()?;
    let (stem, exe) = match name.strip_suffix(".exe") {
        Some(stem) => (stem, ".exe"),
        None => (name, ""),
    };
    let mut words: Vec<&str> = stem.split('-').collect();
    let ar = words.iter().rposition(|w| *w == "ar")?;
    if let Some(word) = words.get_mut(ar) {
        *word = "nm";
    }
    Some(archiver.with_file_name(format!("{}{exe}", words.join("-"))))
}

/// The `nm` tools to try, in order: `NM` alone when set, else the one beside the archiver,
/// `llvm-nm`, then `nm`.
// DECISION: a set `NM` is the only tool tried: an override that silently falls back would hide
// a wrong path.
fn nm_tools(env: Option<OsString>, archiver: Option<&Path>) -> Vec<PathBuf> {
    if let Some(nm) = env.filter(|nm| !nm.is_empty()) {
        return vec![PathBuf::from(nm)];
    }
    let mut tools = Vec::new();
    for tool in [
        archiver.and_then(nm_beside),
        Some("llvm-nm".into()),
        Some("nm".into()),
    ]
    .into_iter()
    .flatten()
    {
        if !tools.contains(&tool) {
            tools.push(tool);
        }
    }
    tools
}

/// The strong global functions `lib` defines, by the first of `tools` that can read it.
fn list_symbols(tools: &[PathBuf], lib: &Path) -> Result<Vec<String>, Error> {
    let mut detail = String::from("none found");
    for tool in tools {
        let out = match Command::new(tool)
            .args(["-P", "-g", "--defined-only"])
            .arg(lib)
            .output()
        {
            Ok(out) => out,
            Err(e) => {
                detail = format!("{}: {e}", tool.display());
                continue;
            }
        };
        if out.status.success() {
            // DECISION: the tool is named on the build script's own output (target/…/output),
            // which cargo keeps and does not print: enough to tell which nm ran.
            println!("kmr_ffi: {} listed by {}", lib.display(), tool.display());
            return Ok(strong_functions(&String::from_utf8_lossy(&out.stdout)));
        }
        let why = String::from_utf8_lossy(&out.stderr);
        detail = format!(
            "{}: {}",
            tool.display(),
            why.lines().next().unwrap_or("failed")
        );
    }
    Err(Error::NoSymbolTool {
        tried: tools.iter().map(|t| t.display().to_string()).collect(),
        detail,
    })
}

/// The strong global functions of `nm -P` output: GNU's `lib.a[member.o]:` headers and
/// llvm-nm's `member.o:` headers are skipped.
// DECISION: only `T`: a controller is a function, and the weak (`W`, `V`), unique (`u`) and
// data symbols C++ headers emit in every object (inline functions, templates, COMDAT) repeat
// across groups by design. Strong data is left out too: a duplicate global variable is the
// user's C, not a controller built for another robot.
fn strong_functions(nm: &str) -> Vec<String> {
    nm.lines()
        .filter_map(|line| {
            let mut words = line.split_whitespace();
            match (words.next(), words.next()) {
                (Some(name), Some("T")) => Some(name.to_owned()),
                _ => None,
            }
        })
        .collect()
}

/// [`Error::DuplicateSymbol`] for the first function of `group` an earlier group defines.
fn check_duplicates(groups: &[Group], group: &Group) -> Result<(), Error> {
    for earlier in groups {
        if let Some(symbol) = group
            .symbols
            .iter()
            .find(|s| earlier.symbols.binary_search(s).is_ok())
        {
            return Err(Error::DuplicateSymbol {
                symbol: symbol.clone(),
                first: earlier.sources.clone(),
                second: group.sources.clone(),
            });
        }
    }
    Ok(())
}

/// Whether `symbol` can be what `c!(name)` names: a plain identifier, not a C++ mangled name.
// DECISION: the table keeps only these: `c!` takes an identifier, and an Itanium-mangled `_Z…`
// name is a C++ function, never an `extern "C"` one. The duplicate check still sees every `T`.
fn nameable(symbol: &str) -> bool {
    let mut chars = symbol.chars();
    chars
        .next()
        .is_some_and(|c| c.is_ascii_alphabetic() || c == '_')
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
        && !symbol.starts_with("_Z")
}

/// `kmr_ffi_abi.rs`: every function of `groups` with the N and sizes it was built for, sorted
/// by name, read by `kmr_api::c!`.
fn abi_table(groups: &[Group]) -> String {
    let mut rows: Vec<(&str, [usize; 5])> = groups
        .iter()
        .flat_map(|g| {
            g.symbols
                .iter()
                .filter(|s| nameable(s))
                .map(|s| (s.as_str(), g.abi))
        })
        .collect();
    rows.sort();
    let mut text = String::from(
        "// Every C and C++ function build.rs compiled, with the N, History depth, IMU slots,\n\
         // microphone slots and microphone window it was compiled for.\n&[\n",
    );
    for (name, [n, d, i, m, w]) in rows {
        text.push_str(&format!("    ({name:?}, [{n}, {d}, {i}, {m}, {w}]),\n"));
    }
    text.push_str("]\n");
    text
}

/// Splits `sources` into C and C++ files by extension.
fn split<'a>(sources: &[&'a str]) -> Result<(Vec<&'a str>, Vec<&'a str>), Error> {
    let mut c = Vec::new();
    let mut cpp = Vec::new();
    for src in sources {
        match Path::new(src).extension().and_then(|e| e.to_str()) {
            Some("c") => c.push(*src),
            Some("cc" | "cpp" | "cxx") => cpp.push(*src),
            _ => return Err(Error::UnknownSource((*src).into())),
        }
    }
    Ok((c, cpp))
}

/// The paths Cargo watches for `sources`: each source's folder, or the file itself at the root.
///
/// The crate root is never watched whole: it holds `target/`, which every build changes. An
/// absolute path to a file at the root would watch it whole; sources are given relative.
fn watched<'a>(sources: &[&'a str]) -> Vec<&'a Path> {
    let mut paths: Vec<&Path> = Vec::new();
    for src in sources {
        let src = Path::new(*src);
        let path = match src.parent() {
            Some(dir) if !dir.components().all(|c| c == Component::CurDir) => dir,
            _ => src,
        };
        if !paths.contains(&path) {
            paths.push(path);
        }
    }
    paths
}

/// Writes `text` to `path`.
fn write(path: &Path, text: &str) -> Result<(), Error> {
    std::fs::write(path, text).map_err(|e| Error::Write(path.to_path_buf(), e))
}

/// One field as the generators see it: its name and its typed-value name.
type Field = (&'static str, &'static str);

/// The field list as `(name, Type)` pairs.
macro_rules! field_names {
    ($($field:ident: $ty:ident),*) => {
        &[$((stringify!($field), stringify!($ty))),*]
    };
}

/// The flag list as names.
macro_rules! flag_names {
    ($($flag:ident),*) => {
        &[$(stringify!($flag)),*]
    };
}

#[cfg(test)]
pub(crate) use field_names;

/// Every field, from [`for_each_field!`](crate::for_each_field).
const FIELDS: &[Field] = crate::for_each_field!(field_names);

/// Every per-actuator flag, from [`for_each_flag!`](crate::for_each_flag).
pub(crate) const FLAGS: &[&str] = crate::for_each_flag!(flag_names);

/// The getter aliases of kmr_api's `State` and `GroupView`, `(alias, field)`.
const ALIASES: &[(&str, &str)] = &[
    ("position", "q"),
    ("velocity", "qd"),
    ("torque", "tau"),
    ("effort", "tau"),
];

/// The telemetry readings, `(name, C type, C++ type)`: `kmr_<name>_at` and `has_<name>`.
const TELEMETRY: &[(&str, &str, &str)] = &[
    ("temperature", "float", "float"),
    ("voltage", "float", "float"),
    ("hardware_error", "uint8_t", "std::uint8_t"),
    ("telemetry_age", "uint64_t", "std::uint64_t"),
];

/// `template` with each placeholder replaced by its value.
fn fill(template: &str, values: &[(&str, &str)]) -> String {
    values.iter().fold(template.to_owned(), |text, (from, to)| {
        text.replace(from, to)
    })
}

/// `template` filled once per field, `@f` its name.
fn per_field(fields: &[Field], template: &str) -> String {
    fields
        .iter()
        .map(|(field, _)| fill(template, &[("@f", field)]))
        .collect()
}

/// `template` filled once per alias whose field is in `fields`, `@a` the alias, `@f` the field.
fn per_alias(fields: &[Field], template: &str) -> String {
    ALIASES
        .iter()
        .filter(|(_, field)| fields.iter().any(|(f, _)| f == field))
        .map(|(alias, field)| fill(template, &[("@a", alias), ("@f", field)]))
        .collect()
}

/// `template` filled once per flag, `@g` its name.
fn per_flag(flags: &[&str], template: &str) -> String {
    flags.iter().map(|g| fill(template, &[("@g", g)])).collect()
}

/// The note at the top of both headers (FR-025 (SRS NFR-2, FR-5.10)).
const NOTE: &str = "\
\x20* Generated by kmr_ffi for one build. Do not edit: the next build overwrites it.
 *
 * Read and write through the kmr_* functions: they check every index against KMR_N and every
 * depth against the History. A raw access to the structs is not checked; past an array it is
 * memory unsafety in your code, which kmr cannot catch.
 *
 * A crash or abort in this code leaves torque as it was. No software guard can prevent it:
 * test the controller with the no-hardware adapter first, as SRS FR-5.10 asks.
";

/// `kmr.h` for `n` actuators and the engine's `sizes`.
pub(crate) fn c_header(n: usize, sizes: Sizes) -> String {
    c_header_from(n, sizes, FIELDS, FLAGS)
}

/// `kmr.hpp`; it takes N and the sizes from `kmr.h`.
pub(crate) fn cpp_header() -> String {
    cpp_header_from(FIELDS, FLAGS)
}

/// The mask of actuators `0..n`, as a C literal.
fn all_literal(n: usize) -> String {
    let mask = if n >= 64 { u64::MAX } else { (1u64 << n) - 1 };
    format!("UINT64_C({mask:#x})")
}

/// The status codes, Rust's `Result` and `Option` as C reads them.
// DECISION: the codes follow kmr_api's StateErrorKind in order; `KMR_NONE` is Rust's `Ok(None)`
// of the `Result<Option<_>>` readers, so `== KMR_OK` alone reads "a value"; a plain `Option` is a
// `bool`, a reading `NULL`.
const C_STATUS: &str = r#"/* What a call returns: KMR_OK for Rust's Ok, a code per kmr_api StateErrorKind otherwise. */
typedef enum {
    KMR_OK = 0,
    KMR_OUT_OF_RANGE = 1,         /* an actuator index past KMR_N */
    KMR_HISTORY_OUT_OF_RANGE = 2, /* a depth not recorded yet, or past KMR_HISTORY_DEPTH */
    KMR_VALUE_MISSING = 3,        /* the value has not been recorded yet */
    KMR_NO_SAMPLE = 4,            /* the actuator had not been read yet at that depth */
    KMR_NONE = 5                  /* not an error: no value yet, Rust's Ok(None); *out untouched */
} kmr_status;

"#;

/// The clock and the fixed sensor types.
// DECISION: the reading types are `kmr_tick_faults`, `kmr_imu_reading` and `kmr_mic_reading`, so
// the API functions keep the Rust names `kmr_faults`, `kmr_imu` and `kmr_mic` (one C namespace).
const C_TIME_TYPES: &str = r#"/* A duration; the kmr_* time functions give it in seconds. */
typedef struct {
    uint64_t secs;
    uint32_t nanos;
    uint32_t reserved;
} kmr_duration;

/* The clock of the tick in progress: a controller's first argument. */
typedef struct {
    uint64_t tick;
    uint64_t overrun_count;
    kmr_duration elapsed;
    kmr_duration dt;
    kmr_duration last_overrun;
    kmr_duration accumulated_overrun;
    uint32_t late; /* 1: the previous tick finished late, by last_overrun */
    uint32_t reserved;
} kmr_time;

"#;

/// The state struct after the per-field types.
const C_STATE: &str = r#"/* The state: a controller's second argument. A snapshot of History refilled before each
 * call; what the call stages is applied after it returns, through the same setters as Rust. */
typedef struct {
    kmr_slot slot[KMR_HISTORY_DEPTH]; /* slot[k]: k ticks back */
    kmr_write_buf write;
    uint64_t telemetry_age[KMR_N];
    uint64_t has_age;
    uint64_t has_temperature;
    uint64_t has_voltage;
    uint64_t has_hardware_error;
    uint64_t has_telemetry_age;
    float temperature[KMR_N];
    float voltage[KMR_N];
    uint32_t age[KMR_N];
    uint32_t read_every[KMR_N];
    uint32_t filled;
    uint32_t faults_filled;
    uint32_t sample_age;
    uint32_t has_sample_age;
    uint32_t signal; /* 0 none, 1 shutdown, 2 emergency stop */
    uint8_t hardware_error[KMR_N];
    char reason[KMR_REASON_LEN];
} kmr_state;

/* Three axes: an acceleration in m/s^2 or an angular rate in rad/s. */
typedef struct {
    float x;
    float y;
    float z;
} kmr_axes;

/* One IMU's newest reading. */
typedef struct {
    uint64_t age;
    uint64_t dropped;
    kmr_axes accel;
    kmr_axes gyro;
    uint32_t present;
    uint32_t has_accel;
    uint32_t has_gyro;
    uint32_t reserved;
} kmr_imu_reading;

/* The samples one microphone delivered this tick. */
typedef struct {
    uint64_t dropped;
    uint32_t present;
    uint32_t rate_hz;
    uint32_t len;
    uint32_t reserved;
    int16_t samples[KMR_MIC_WINDOW];
} kmr_mic_reading;

/* The sensors: a controller's third argument. */
typedef struct {
    kmr_imu_reading imu[KMR_MAX_IMUS];
    kmr_mic_reading mic[KMR_MAX_MICS];
} kmr_sensors;

/* The signature of a controller; ctx is the context of kmr_api::c!(name, with = ctx), or NULL. */
typedef void (*kmr_controller)(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors,
                               void *ctx);

"#;

/// The helpers every function below builds on.
const C_INTERNAL: &str = r#"/* kmr_internal_*: the header's own helpers, not part of the API. */
static inline bool kmr_internal_bit(uint64_t mask, size_t i) {
    return i < 64 && ((mask >> i) & 1u) != 0;
}

static inline kmr_status kmr_internal_at(const kmr_state *s, size_t i, size_t depth) {
    if (depth >= s->filled || depth >= KMR_HISTORY_DEPTH) {
        return KMR_HISTORY_OUT_OF_RANGE;
    }
    if (i >= KMR_N) {
        return KMR_OUT_OF_RANGE;
    }
    if (!kmr_internal_bit(s->slot[depth].sampled, i)) {
        return KMR_NO_SAMPLE;
    }
    return KMR_OK;
}

static inline kmr_status kmr_internal_row(const kmr_state *s, size_t depth) {
    if (depth >= s->filled || depth >= KMR_HISTORY_DEPTH) {
        return KMR_HISTORY_OUT_OF_RANGE;
    }
    if (s->slot[depth].sampled != KMR_INTERNAL_ALL) {
        return KMR_NO_SAMPLE;
    }
    return KMR_OK;
}

static inline double kmr_internal_secs(kmr_duration d) {
    return (double)d.secs + (double)d.nanos / 1e9;
}

static inline void kmr_internal_request(kmr_state *s, uint32_t level, const char *a,
                                        const char *b) {
    size_t n = 0;
    const char *p;
    if (level <= s->signal) {
        return;
    }
    s->signal = level;
    for (p = a; p != NULL && *p != '\0' && n + 1 < KMR_REASON_LEN; ++p) {
        s->reason[n++] = *p;
    }
    for (p = b; p != NULL && *p != '\0' && n + 1 < KMR_REASON_LEN; ++p) {
        s->reason[n++] = *p;
    }
    s->reason[n] = '\0';
}

"#;

/// The getters and setters of one field, `@f`.
// DECISION: indices are `size_t` (Rust's usize) and values bare `float`s in engine units: FFI is
// the one place bare scalars cross (SRS FR-2.2). A group is its index array and length, no handle.
const C_FIELD: &str = r#"/* @f of actuator i, depth ticks back (0 = newest). */
static inline kmr_status kmr_prev_@f_at(const kmr_state *s, size_t i, size_t depth, float *out) {
    kmr_status st = kmr_internal_at(s, i, depth);
    if (st == KMR_OK) {
        *out = s->slot[depth].@f[i];
    }
    return st;
}

/* @f of every actuator, depth ticks back, into out. */
static inline kmr_status kmr_prev_@f(const kmr_state *s, size_t depth, float out[KMR_N]) {
    size_t i;
    kmr_status st = kmr_internal_row(s, depth);
    if (st == KMR_OK) {
        for (i = 0; i < KMR_N; ++i) {
            out[i] = s->slot[depth].@f[i];
        }
    }
    return st;
}

/* Newest @f of actuator i. */
static inline kmr_status kmr_@f_at(const kmr_state *s, size_t i, float *out) {
    return kmr_prev_@f_at(s, i, 0, out);
}

/* Newest @f of every actuator into out; false until each has a sample (Rust's None). */
static inline bool kmr_@f(const kmr_state *s, float out[KMR_N]) {
    return kmr_prev_@f(s, 0, out) == KMR_OK;
}

/* Stages @f for every actuator. */
static inline void kmr_set_all_@f(kmr_state *s, const float values[KMR_N]) {
    size_t i;
    for (i = 0; i < KMR_N; ++i) {
        s->write.@f[i] = values[i];
    }
    s->write.written.@f = KMR_INTERNAL_ALL;
}

/* Stages value as @f of actuator i. */
static inline kmr_status kmr_set_@f_at(kmr_state *s, float value, size_t i) {
    if (i >= KMR_N) {
        return KMR_OUT_OF_RANGE;
    }
    s->write.@f[i] = value;
    s->write.written.@f |= (uint64_t)1 << i;
    return KMR_OK;
}

/* Newest @f of each of the k group actuators into out; false if one has none or is past KMR_N. */
static inline bool kmr_group_@f(const kmr_state *s, const size_t *indices, size_t k, float *out) {
    size_t j;
    for (j = 0; j < k; ++j) {
        if (kmr_internal_at(s, indices[j], 0) != KMR_OK) {
            return false;
        }
    }
    for (j = 0; j < k; ++j) {
        out[j] = s->slot[0].@f[indices[j]];
    }
    return true;
}

/* @f of each group actuator, depth ticks back; on the first error, out is untouched. */
static inline kmr_status kmr_group_prev_@f(const kmr_state *s, const size_t *indices, size_t k,
                                          size_t depth, float *out) {
    size_t j;
    for (j = 0; j < k; ++j) {
        kmr_status st = kmr_internal_at(s, indices[j], depth);
        if (st != KMR_OK) {
            return st;
        }
    }
    for (j = 0; j < k; ++j) {
        out[j] = s->slot[depth].@f[indices[j]];
    }
    return KMR_OK;
}

/* Stages values[j] as @f of group actuator indices[j]; nothing if one is past KMR_N. */
static inline kmr_status kmr_group_set_all_@f(kmr_state *s, const size_t *indices, size_t k,
                                             const float *values) {
    size_t j;
    for (j = 0; j < k; ++j) {
        if (indices[j] >= KMR_N) {
            return KMR_OUT_OF_RANGE;
        }
    }
    for (j = 0; j < k; ++j) {
        (void)kmr_set_@f_at(s, values[j], indices[j]);
    }
    return KMR_OK;
}

"#;

/// One getter alias, `@a` for `@f`.
const C_ALIAS: &str = r#"/* Alias of kmr_@f. */
static inline bool kmr_@a(const kmr_state *s, float out[KMR_N]) {
    return kmr_@f(s, out);
}

/* Alias of kmr_group_@f. */
static inline bool kmr_group_@a(const kmr_state *s, const size_t *indices, size_t k, float *out) {
    return kmr_group_@f(s, indices, k, out);
}

"#;

/// The faults of a depth, before the per-flag accessors.
const C_FAULTS: &str = r#"/* The fault bits and flags of the tick depth back (0 = this tick). */
static inline kmr_status kmr_prev_faults(const kmr_state *s, size_t depth, kmr_tick_faults *out) {
    if (depth >= s->faults_filled || depth >= KMR_HISTORY_DEPTH) {
        return KMR_HISTORY_OUT_OF_RANGE;
    }
    *out = s->slot[depth].faults;
    return KMR_OK;
}

/* This tick's faults; false before the first tick. */
static inline bool kmr_faults(const kmr_state *s, kmr_tick_faults *out) {
    return kmr_prev_faults(s, 0, out) == KMR_OK;
}

/* Whether this tick's read of actuator i failed, its previous value kept. */
static inline bool kmr_stale_at(const kmr_state *s, size_t i) {
    return s->faults_filled > 0 && kmr_internal_bit(s->slot[0].faults.stale, i);
}

"#;

/// One per-actuator flag accessor, `@g`.
const C_FLAG: &str = r#"/* Actuators with @g, bit i = actuator i. */
static inline uint64_t kmr_faults_@g(const kmr_tick_faults *f) {
    return f->@g;
}

"#;

/// One per-tick flag accessor, `@g` with bit `@b`.
const C_TICK_FLAG: &str = r#"static inline bool kmr_faults_@g(const kmr_tick_faults *f) {
    return (f->tick & @bu) != 0;
}

"#;

/// One telemetry reading, `@t` of C type `@T`.
const C_TELEMETRY: &str = r#"/* @t of actuator i; KMR_NONE without a reading. */
static inline kmr_status kmr_@t_at(const kmr_state *s, size_t i, @T *out) {
    if (i >= KMR_N) {
        return KMR_OUT_OF_RANGE;
    }
    if (!kmr_internal_bit(s->has_@t, i)) {
        return KMR_NONE;
    }
    *out = s->@t[i];
    return KMR_OK;
}

"#;

/// Ages, cadence, the other state functions, the clock, the signals and the sensors.
const C_REST: &str = r#"/* Ticks since actuator i was last read; KMR_NONE before its first read. */
static inline kmr_status kmr_age_at(const kmr_state *s, size_t i, uint32_t *out) {
    if (i >= KMR_N) {
        return KMR_OUT_OF_RANGE;
    }
    if (!kmr_internal_bit(s->has_age, i)) {
        return KMR_NONE;
    }
    *out = s->age[i];
    return KMR_OK;
}

/* Age of the oldest sample over every actuator; false while one was never read. */
static inline bool kmr_sample_age(const kmr_state *s, uint32_t *out) {
    if (s->has_sample_age == 0) {
        return false;
    }
    *out = s->sample_age;
    return true;
}

/* How often actuator i is read, in ticks. */
static inline kmr_status kmr_read_every(const kmr_state *s, size_t i, uint32_t *out) {
    if (i >= KMR_N) {
        return KMR_OUT_OF_RANGE;
    }
    *out = s->read_every[i];
    return KMR_OK;
}

/* Temperature of every actuator, deg C; present[i] false (and out[i] 0) without a reading. */
static inline void kmr_temperatures(const kmr_state *s, float out[KMR_N], bool present[KMR_N]) {
    size_t i;
    for (i = 0; i < KMR_N; ++i) {
        out[i] = 0.0f;
        present[i] = kmr_temperature_at(s, i, &out[i]) == KMR_OK;
    }
}

/* Age of the oldest sample over the group; false while one was never read or is past KMR_N. */
static inline bool kmr_group_sample_age(const kmr_state *s, const size_t *indices, size_t k,
                                        uint32_t *out) {
    size_t j;
    uint32_t oldest = 0;
    for (j = 0; j < k; ++j) {
        uint32_t age = 0;
        if (kmr_age_at(s, indices[j], &age) != KMR_OK) {
            return false;
        }
        if (age > oldest) {
            oldest = age;
        }
    }
    *out = oldest;
    return true;
}

/* Whether this tick's read failed on one of the group. */
static inline bool kmr_group_stale(const kmr_state *s, const size_t *indices, size_t k) {
    size_t j;
    for (j = 0; j < k; ++j) {
        if (kmr_stale_at(s, indices[j])) {
            return true;
        }
    }
    return false;
}

/* Temperature of each group actuator; present[j] false (and out[j] 0) without a reading. */
static inline void kmr_group_temperatures(const kmr_state *s, const size_t *indices, size_t k,
                                          float *out, bool *present) {
    size_t j;
    for (j = 0; j < k; ++j) {
        out[j] = 0.0f;
        present[j] = kmr_temperature_at(s, indices[j], &out[j]) == KMR_OK;
    }
}

/* Asks for a graceful shutdown: the tick completes, then torque goes off. */
static inline void kmr_shutdown(kmr_state *s, const char *reason) {
    kmr_internal_request(s, 1u, reason, NULL);
}

/* Asks for an emergency stop: the loop leaves at its next check. */
static inline void kmr_emergency_stop(kmr_state *s, const char *reason) {
    kmr_internal_request(s, 2u, reason, NULL);
}

/* Seconds since the tick grid was armed, stamped at the start of the tick. */
static inline double kmr_elapsed(const kmr_time *t) {
    return kmr_internal_secs(t->elapsed);
}

/* The tick duration, s. */
static inline double kmr_dt(const kmr_time *t) {
    return kmr_internal_secs(t->dt);
}

/* Index of the tick in progress, 0 for the first one. */
static inline uint64_t kmr_tick(const kmr_time *t) {
    return t->tick;
}

/* How many ticks finished after their deadline. */
static inline uint64_t kmr_overrun_count(const kmr_time *t) {
    return t->overrun_count;
}

/* How late the previous tick finished, s; false if it was on time. */
static inline bool kmr_last_overrun(const kmr_time *t, double *out) {
    if (t->late == 0) {
        return false;
    }
    *out = kmr_internal_secs(t->last_overrun);
    return true;
}

/* Sum of every overrun so far, s. */
static inline double kmr_accumulated_overrun(const kmr_time *t) {
    return kmr_internal_secs(t->accumulated_overrun);
}

/* IMU i, in the order the sensors were added; NULL without a reading. */
static inline const kmr_imu_reading *kmr_imu(const kmr_sensors *s, size_t i) {
    return (i < KMR_MAX_IMUS && s->imu[i].present != 0) ? &s->imu[i] : NULL;
}

/* Acceleration, m/s^2; false when that half of the sensor is off. */
static inline bool kmr_imu_accel(const kmr_imu_reading *imu, kmr_axes *out) {
    if (imu->has_accel == 0) {
        return false;
    }
    *out = imu->accel;
    return true;
}

/* Angular rate, rad/s; false when that half of the sensor is off. */
static inline bool kmr_imu_gyro(const kmr_imu_reading *imu, kmr_axes *out) {
    if (imu->has_gyro == 0) {
        return false;
    }
    *out = imu->gyro;
    return true;
}

/* Ticks since the reading arrived. */
static inline uint64_t kmr_imu_age(const kmr_imu_reading *imu) {
    return imu->age;
}

/* Readings lost so far. */
static inline uint64_t kmr_imu_dropped(const kmr_imu_reading *imu) {
    return imu->dropped;
}

/* Microphone i, in the order the sensors were added; NULL without a reading. */
static inline const kmr_mic_reading *kmr_mic(const kmr_sensors *s, size_t i) {
    return (i < KMR_MAX_MICS && s->mic[i].present != 0) ? &s->mic[i] : NULL;
}

/* PCM samples that arrived since the last tick, oldest first; *len of them. */
static inline const int16_t *kmr_mic_samples(const kmr_mic_reading *mic, size_t *len) {
    *len = mic->len;
    return mic->samples;
}

/* Sample rate, Hz. */
static inline uint32_t kmr_mic_rate_hz(const kmr_mic_reading *mic) {
    return mic->rate_hz;
}

/* Samples lost so far. */
static inline uint64_t kmr_mic_dropped(const kmr_mic_reading *mic) {
    return mic->dropped;
}

"#;

/// `kmr.h` from a field and a flag list.
pub(crate) fn c_header_from(n: usize, sizes: Sizes, fields: &[Field], flags: &[&str]) -> String {
    let Sizes {
        history_depth,
        max_imus,
        max_mics,
        mic_window,
    } = sizes;
    let reason = crate::REASON_LEN;
    let all = all_literal(n);
    let mut h = format!(
        "/* kmr.h: the controller API of kmr for C11, over the snapshot kmr refills each tick.\n\
         \x20*\n{NOTE} */\n\
         #ifndef KMR_H\n#define KMR_H\n\n\
         #include <stdbool.h>\n#include <stddef.h>\n#include <stdint.h>\n\n\
         #ifdef __cplusplus\nextern \"C\" {{\n#endif\n\n\
         #define KMR_N {n}\n\
         #define KMR_HISTORY_DEPTH {history_depth}\n\
         #define KMR_MAX_IMUS {max_imus}\n\
         #define KMR_MAX_MICS {max_mics}\n\
         #define KMR_MIC_WINDOW {mic_window}\n\
         #define KMR_REASON_LEN {reason}\n\
         #define KMR_INTERNAL_ALL {all}\n\
         #define KMR_INTERNAL_ABI abi_n{n}_h{history_depth}_i{max_imus}_m{max_mics}_w{mic_window}\n\n"
    );
    h.push_str(C_STATUS);
    h.push_str(C_TIME_TYPES);

    h.push_str("/* The fault bits and flags of one tick: bit i of a mask is actuator i. */\n");
    h.push_str("typedef struct {\n");
    h.push_str(&per_flag(flags, "    uint64_t @g;\n"));
    h.push_str("    uint32_t tick;\n    uint32_t reserved;\n} kmr_tick_faults;\n\n");

    h.push_str("/* One History slot. */\ntypedef struct {\n");
    h.push_str(&per_field(fields, "    float @f[KMR_N];\n"));
    h.push_str("    uint64_t sampled;\n    kmr_tick_faults faults;\n} kmr_slot;\n\n");

    h.push_str("/* Bit i set: entry i was staged. */\ntypedef struct {\n");
    h.push_str(&per_field(fields, "    uint64_t @f;\n"));
    h.push_str("} kmr_written;\n\n");

    h.push_str("/* The values a call staged. */\ntypedef struct {\n");
    h.push_str(&per_field(fields, "    float @f[KMR_N];\n"));
    h.push_str("    kmr_written written;\n} kmr_write_buf;\n\n");

    h.push_str(C_STATE);
    h.push_str(C_INTERNAL);
    h.push_str("static inline void kmr_internal_discard(kmr_state *s) {\n");
    h.push_str(&per_field(fields, "    s->write.written.@f = 0;\n"));
    h.push_str("}\n\n");
    h.push_str(&per_field(fields, C_FIELD));
    h.push_str(&per_alias(fields, C_ALIAS));
    h.push_str(C_FAULTS);
    h.push_str(&per_flag(flags, C_FLAG));
    for (flag, bit) in crate::c::TICK_FLAGS {
        h.push_str(&format!("/* The tick's flag {flag}. */\n"));
        h.push_str(&fill(
            C_TICK_FLAG,
            &[("@g", flag), ("@b", &bit.to_string())],
        ));
    }
    h.push_str("/* Actuators with at least one fault bit set. */\n");
    h.push_str("static inline uint64_t kmr_faults_any_actuator(const kmr_tick_faults *f) {\n");
    h.push_str("    return 0");
    h.push_str(&per_flag(flags, " | f->@g"));
    h.push_str(";\n}\n\n");
    h.push_str("/* Whether no bit and no flag is set. */\n");
    h.push_str("static inline bool kmr_faults_is_clear(const kmr_tick_faults *f) {\n");
    h.push_str("    return kmr_faults_any_actuator(f) == 0 && f->tick == 0;\n}\n\n");
    for (name, c_ty, _) in TELEMETRY {
        h.push_str(&fill(C_TELEMETRY, &[("@t", name), ("@T", c_ty)]));
    }
    h.push_str(C_REST);
    h.push_str("#ifdef __cplusplus\n}\n#endif\n\n#endif /* KMR_H */\n");
    h
}

/// The value types of `kmr.hpp`, before `Group` and `State`.
// DECISION: everything inside an inline namespace named after N and the sizes
// (`KMR_INTERNAL_ABI` of `kmr.h`). The member functions of `kmr::State` are inline, emitted weak in
// every object, and their mangled names hold no N: two groups for two N would define
// `kmr::State::q()` twice, and the linker would keep one for both (ODR). `kmr::State` still names
// the class.
const CPP_TOP: &str = r#"namespace kmr {
/* Named after N and the sizes: two robots' groups linked into one binary never share an
 * inline function of this header. */
inline namespace KMR_INTERNAL_ABI {

/* What a call returns: KMR_OK for Rust's Ok. */
using Status = kmr_status;

/* One value per actuator, engine units. */
using Values = std::array<float, KMR_N>;

/* Rust's Result<T, StateError>: the value when ok(), the code in error() otherwise. */
template <class T>
class Result {
public:
    Result(Status status, T value) noexcept : status_(status), value_(value) {}
    bool ok() const noexcept { return status_ == KMR_OK; }
    explicit operator bool() const noexcept { return ok(); }
    const T &operator*() const noexcept { return value_; }
    const T *operator->() const noexcept { return &value_; }
    T value_or(T fallback) const noexcept { return ok() ? value_ : fallback; }
    Status error() const noexcept { return status_; }

private:
    Status status_;
    T value_;
};

namespace detail {
/* Rust's Result<Option<T>>: KMR_NONE is an ok std::nullopt. */
template <class T>
Result<std::optional<T>> maybe(Status status, T value) noexcept {
    if (status == KMR_OK) {
        return Result<std::optional<T>>(KMR_OK, value);
    }
    return Result<std::optional<T>>(status == KMR_NONE ? KMR_OK : status, std::nullopt);
}
} // namespace detail

/* One bit per actuator: bit i is actuator i. */
struct Mask {
    std::uint64_t bits;
    bool get(std::size_t i) const noexcept { return i < 64 && ((bits >> i) & 1u) != 0; }
    bool is_empty() const noexcept { return bits == 0; }
};

"#;

/// `Time` and the sensors of `kmr.hpp`.
const CPP_TIME_SENSORS: &str = r#"/* The clock of the tick in progress; durations in seconds. */
class Time {
public:
    explicit Time(const kmr_time *time) noexcept : t_(time) {}
    double elapsed() const noexcept { return kmr_elapsed(t_); }
    double dt() const noexcept { return kmr_dt(t_); }
    std::uint64_t tick() const noexcept { return kmr_tick(t_); }
    std::uint64_t overrun_count() const noexcept { return kmr_overrun_count(t_); }
    std::optional<double> last_overrun() const noexcept {
        double v = 0.0;
        if (kmr_last_overrun(t_, &v)) {
            return v;
        }
        return std::nullopt;
    }
    double accumulated_overrun() const noexcept { return kmr_accumulated_overrun(t_); }

private:
    const kmr_time *t_;
};

/* An acceleration (m/s^2) or an angular rate (rad/s): x, y, z. */
using Axes = kmr_axes;

/* One IMU's newest reading. */
class Imu {
public:
    explicit Imu(const kmr_imu_reading *imu) noexcept : r_(imu) {}
    std::optional<Axes> accel() const noexcept {
        Axes a{};
        if (kmr_imu_accel(r_, &a)) {
            return a;
        }
        return std::nullopt;
    }
    std::optional<Axes> gyro() const noexcept {
        Axes g{};
        if (kmr_imu_gyro(r_, &g)) {
            return g;
        }
        return std::nullopt;
    }
    std::uint64_t age() const noexcept { return kmr_imu_age(r_); }
    std::uint64_t dropped() const noexcept { return kmr_imu_dropped(r_); }

private:
    const kmr_imu_reading *r_;
};

/* PCM samples, oldest first; valid for the call. */
class Samples {
public:
    Samples(const std::int16_t *data, std::size_t size) noexcept : data_(data), size_(size) {}
    const std::int16_t *data() const noexcept { return data_; }
    std::size_t size() const noexcept { return size_; }
    bool empty() const noexcept { return size_ == 0; }
    const std::int16_t *begin() const noexcept { return data_; }
    const std::int16_t *end() const noexcept { return data_ + size_; }
    std::int16_t operator[](std::size_t k) const noexcept { return data_[k]; }

private:
    const std::int16_t *data_;
    std::size_t size_;
};

/* The samples one microphone delivered this tick. */
class Mic {
public:
    explicit Mic(const kmr_mic_reading *mic) noexcept : r_(mic) {}
    Samples samples() const noexcept {
        std::size_t len = 0;
        const std::int16_t *data = kmr_mic_samples(r_, &len);
        return Samples(data, len);
    }
    std::uint32_t rate_hz() const noexcept { return kmr_mic_rate_hz(r_); }
    std::uint64_t dropped() const noexcept { return kmr_mic_dropped(r_); }

private:
    const kmr_mic_reading *r_;
};

/* The sensors of the tick. */
class Sensors {
public:
    explicit Sensors(const kmr_sensors *sensors) noexcept : s_(sensors) {}
    std::optional<Imu> imu(std::size_t index) const noexcept {
        const kmr_imu_reading *r = kmr_imu(s_, index);
        if (r == nullptr) {
            return std::nullopt;
        }
        return Imu(r);
    }
    std::optional<Mic> mic(std::size_t index) const noexcept {
        const kmr_mic_reading *r = kmr_mic(s_, index);
        if (r == nullptr) {
            return std::nullopt;
        }
        return Mic(r);
    }

private:
    const kmr_sensors *s_;
};

/* A view of K chosen actuators, from State::group: the field accessors over them, in order. */
template <std::size_t K>
class Group {
public:
    Group(kmr_state *state, const std::array<std::size_t, K> &indices) noexcept
        : s_(state), indices_(indices) {}
"#;

/// One field of `Group`, `@f`.
const CPP_GROUP_FIELD: &str = r#"    std::optional<std::array<float, K>> @f() const noexcept {
        std::array<float, K> v{};
        if (kmr_group_@f(s_, indices_.data(), K, v.data())) {
            return v;
        }
        return std::nullopt;
    }
    Result<std::array<float, K>> prev_@f(std::size_t depth) const noexcept {
        std::array<float, K> v{};
        Status st = kmr_group_prev_@f(s_, indices_.data(), K, depth, v.data());
        return Result<std::array<float, K>>(st, v);
    }
    Status set_all_@f(const std::array<float, K> &values) noexcept {
        return kmr_group_set_all_@f(s_, indices_.data(), K, values.data());
    }
"#;

/// One alias of `Group`.
const CPP_GROUP_ALIAS: &str = r#"    std::optional<std::array<float, K>> @a() const noexcept { return @f(); }
"#;

/// The end of `Group` and the start of `State`.
const CPP_GROUP_END: &str = r#"    std::optional<std::uint32_t> sample_age() const noexcept {
        std::uint32_t v = 0;
        if (kmr_group_sample_age(s_, indices_.data(), K, &v)) {
            return v;
        }
        return std::nullopt;
    }
    bool stale() const noexcept { return kmr_group_stale(s_, indices_.data(), K); }
    std::array<std::optional<float>, K> temperatures() const noexcept {
        std::array<std::optional<float>, K> out{};
        for (std::size_t j = 0; j < K; ++j) {
            float v = 0.0f;
            if (kmr_temperature_at(s_, indices_[j], &v) == KMR_OK) {
                out[j] = v;
            }
        }
        return out;
    }

private:
    kmr_state *s_;
    std::array<std::size_t, K> indices_;
};

/* The state: a snapshot of History; setters stage, kmr applies after the call. */
class State {
public:
    explicit State(kmr_state *state) noexcept : s_(state) {}
"#;

/// One field of `State`, `@f`.
const CPP_STATE_FIELD: &str = r#"    std::optional<Values> @f() const noexcept {
        Values v{};
        if (kmr_@f(s_, v.data())) {
            return v;
        }
        return std::nullopt;
    }
    Result<float> @f_at(std::size_t i) const noexcept {
        float v = 0.0f;
        Status st = kmr_@f_at(s_, i, &v);
        return Result<float>(st, v);
    }
    Result<Values> prev_@f(std::size_t depth) const noexcept {
        Values v{};
        Status st = kmr_prev_@f(s_, depth, v.data());
        return Result<Values>(st, v);
    }
    Result<float> prev_@f_at(std::size_t i, std::size_t depth) const noexcept {
        float v = 0.0f;
        Status st = kmr_prev_@f_at(s_, i, depth, &v);
        return Result<float>(st, v);
    }
    void set_all_@f(const Values &values) noexcept { kmr_set_all_@f(s_, values.data()); }
    Status set_@f_at(float value, std::size_t i) noexcept { return kmr_set_@f_at(s_, value, i); }
"#;

/// One alias of `State`.
const CPP_STATE_ALIAS: &str = r#"    std::optional<Values> @a() const noexcept { return @f(); }
"#;

/// One telemetry reading of `State`, `@t` of C++ type `@T`.
const CPP_STATE_TELEMETRY: &str = r#"    Result<std::optional<@T>> @t_at(std::size_t i) const noexcept {
        @T v{};
        Status st = kmr_@t_at(s_, i, &v);
        return detail::maybe<@T>(st, v);
    }
"#;

/// The rest of `State`, the controller glue and the macro.
// DECISION: an exception leaving a KMR_CONTROLLER body discards the call's writes and asks for a
// shutdown, as a Python exception does; a missing context is a shutdown too, never a null
// dereference. No exception crosses the extern "C" boundary.
const CPP_STATE_END: &str = r#"    std::optional<Faults> faults() const noexcept {
        kmr_tick_faults f{};
        if (kmr_faults(s_, &f)) {
            return Faults(f);
        }
        return std::nullopt;
    }
    Result<Faults> prev_faults(std::size_t depth) const noexcept {
        kmr_tick_faults f{};
        Status st = kmr_prev_faults(s_, depth, &f);
        return Result<Faults>(st, Faults(f));
    }
    bool stale_at(std::size_t i) const noexcept { return kmr_stale_at(s_, i); }
    Result<std::optional<std::uint32_t>> age_at(std::size_t i) const noexcept {
        std::uint32_t v = 0;
        Status st = kmr_age_at(s_, i, &v);
        return detail::maybe<std::uint32_t>(st, v);
    }
    std::optional<std::uint32_t> sample_age() const noexcept {
        std::uint32_t v = 0;
        if (kmr_sample_age(s_, &v)) {
            return v;
        }
        return std::nullopt;
    }
    Result<std::uint32_t> read_every(std::size_t i) const noexcept {
        std::uint32_t v = 1;
        Status st = kmr_read_every(s_, i, &v);
        return Result<std::uint32_t>(st, v);
    }
    std::array<std::optional<float>, KMR_N> temperatures() const noexcept {
        std::array<std::optional<float>, KMR_N> out{};
        for (std::size_t i = 0; i < KMR_N; ++i) {
            float v = 0.0f;
            if (kmr_temperature_at(s_, i, &v) == KMR_OK) {
                out[i] = v;
            }
        }
        return out;
    }
    template <std::size_t K>
    Group<K> group(const std::array<std::size_t, K> &indices) noexcept {
        return Group<K>(s_, indices);
    }
    template <std::size_t K>
    Group<K> group(const std::size_t (&indices)[K]) noexcept {
        std::array<std::size_t, K> a{};
        for (std::size_t j = 0; j < K; ++j) {
            a[j] = indices[j];
        }
        return Group<K>(s_, a);
    }
    /* Asks for a graceful shutdown: the tick completes, then torque goes off. */
    void shutdown(const char *reason) noexcept { kmr_shutdown(s_, reason); }
    /* Asks for an emergency stop: the loop leaves at its next check. */
    void emergency_stop(const char *reason) noexcept { kmr_emergency_stop(s_, reason); }

private:
    kmr_state *s_;
};

namespace detail {
template <class R, class A, class B, class C, class T>
T ctx_type(R (*)(A, B, C, T &));
template <class R, class A, class B, class C, class T>
T ctx_type(R (*)(A, B, C, T &) noexcept);
template <class L, class R, class A, class B, class C, class T>
T ctx_type(R (L::*)(A, B, C, T &) const);
template <class L, class R, class A, class B, class C, class T>
T ctx_type(R (L::*)(A, B, C, T &) const noexcept);

/* The context type a controller takes by reference as its fourth parameter. */
template <class F, class = void>
struct ctx_of {
    using type = decltype(ctx_type(&F::operator()));
};
template <class F>
struct ctx_of<F, std::enable_if_t<std::is_pointer<F>::value>> {
    using type = decltype(ctx_type(std::declval<F>()));
};

/* Wraps the C handles and runs f; an exception discards the call's writes and shuts down. */
template <class F>
void invoke(F &&f, const kmr_time *t, kmr_state *s, const kmr_sensors *se, void *ctx) noexcept {
    Time time(t);
    State state(s);
    Sensors sensors(se);
#if defined(__cpp_exceptions)
    try {
#endif
        if constexpr (std::is_invocable<F &, Time &, State &, Sensors &>::value) {
            (void)ctx;
            f(time, state, sensors);
        } else {
            using Ctx = typename ctx_of<std::decay_t<F>>::type;
            if (ctx == nullptr) {
                state.shutdown("this C++ controller takes a context: kmr_api::c!(name, with = ..)");
                return;
            }
            f(time, state, sensors, *static_cast<Ctx *>(ctx));
        }
#if defined(__cpp_exceptions)
    } catch (const std::exception &e) {
        kmr_internal_discard(s);
        kmr_internal_request(s, 1u, "C++ exception: ", e.what());
    } catch (...) {
        kmr_internal_discard(s);
        kmr_internal_request(s, 1u, "C++ exception", nullptr);
    }
#endif
}
} // namespace detail

} // inline namespace KMR_INTERNAL_ABI
} // namespace kmr

/* Defines the C function `name`, what kmr_api::c!(name) registers, over `function`:
 * void function(const kmr::Time &, kmr::State &, const kmr::Sensors &[, T &ctx]).
 * With a T &ctx, register it as kmr_api::c!(name, with = ..) and mirror T as #[repr(C)]. */
#define KMR_CONTROLLER(name, function)                                                         \
    extern "C" void name(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors,   \
                         void *ctx) noexcept {                                                 \
        ::kmr::detail::invoke(function, time, state, sensors, ctx);                            \
    }

#endif /* KMR_HPP */
"#;

/// `kmr.hpp` from a field and a flag list.
pub(crate) fn cpp_header_from(fields: &[Field], flags: &[&str]) -> String {
    let mut h = format!(
        "/* kmr.hpp: the controller API of kmr for C++17, methods over kmr.h at no cost.\n\
         \x20*\n{NOTE} *\n\
         \x20* Rust's Result is kmr::Result, Rust's Option std::optional. No exception crosses to\n\
         \x20* Rust: KMR_CONTROLLER turns one that leaves the body into a shutdown, nothing applied.\n\
         \x20*/\n\
         #ifndef KMR_HPP\n#define KMR_HPP\n\n\
         #include <array>\n#include <cstddef>\n#include <cstdint>\n#include <optional>\n\
         #include <type_traits>\n#include <utility>\n\
         #if defined(__cpp_exceptions)\n#include <exception>\n#endif\n\n\
         #include \"kmr.h\"\n\n"
    );
    h.push_str(CPP_TOP);
    h.push_str("/* The fault bits and flags of one tick: masks per actuator, flags per tick. */\n");
    h.push_str("class Faults {\npublic:\n");
    h.push_str("    explicit Faults(const kmr_tick_faults &faults) noexcept : f_(faults) {}\n");
    h.push_str(&per_flag(
        flags,
        "    Mask @g() const noexcept { return Mask{kmr_faults_@g(&f_)}; }\n",
    ));
    for (flag, _) in crate::c::TICK_FLAGS {
        h.push_str(&fill(
            "    bool @g() const noexcept { return kmr_faults_@g(&f_); }\n",
            &[("@g", flag)],
        ));
    }
    h.push_str(
        "    Mask any_actuator() const noexcept { return Mask{kmr_faults_any_actuator(&f_)}; }\n\
         \x20   bool is_clear() const noexcept { return kmr_faults_is_clear(&f_); }\n\n\
         private:\n\
         \x20   kmr_tick_faults f_;\n\
         };\n\n",
    );
    h.push_str(CPP_TIME_SENSORS);
    h.push_str(&per_field(fields, CPP_GROUP_FIELD));
    h.push_str(&per_alias(fields, CPP_GROUP_ALIAS));
    h.push_str(CPP_GROUP_END);
    h.push_str(&per_field(fields, CPP_STATE_FIELD));
    h.push_str(&per_alias(fields, CPP_STATE_ALIAS));
    for (name, _, cpp_ty) in TELEMETRY {
        h.push_str(&fill(CPP_STATE_TELEMETRY, &[("@t", name), ("@T", cpp_ty)]));
    }
    h.push_str(CPP_STATE_END);
    h
}

#[cfg(test)]
mod tests;
