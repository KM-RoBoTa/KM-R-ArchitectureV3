//! The generated headers compiled and run by the system compilers, against Rust's layout.

use super::*;
use crate::{
    CDuration, CImu, CMic, CSensors, CSlot, CState, CTickFaults, CTime, WriteBuf, Written,
};
use core::mem::{align_of, offset_of, size_of};
use std::path::PathBuf;
use std::process::Command;

/// The core's sizes today.
const SIZES: Sizes = Sizes {
    history_depth: 4,
    max_imus: 4,
    max_mics: 1,
    mic_window: 1024,
};

/// A fresh directory for one test's files.
fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir()
        .join(format!("kmr_ffi_build_{}", std::process::id()))
        .join(name);
    std::fs::create_dir_all(&dir).expect("scratch dir");
    dir
}

/// Removes a scratch directory and its parent once empty.
fn clean(dir: &Path) {
    let _ = std::fs::remove_dir_all(dir);
    if let Some(parent) = dir.parent() {
        let _ = std::fs::remove_dir(parent);
    }
}

/// Compiles `src` with `compiler` and `args`, warnings as errors; panics with its output.
fn compile_ok(compiler: &str, args: &[&str], dir: &Path, src: &Path) {
    let out = Command::new(compiler)
        .args(args)
        .args(["-Wall", "-Wextra", "-Werror", "-pedantic", "-I"])
        .arg(dir)
        .arg(src)
        .output()
        .expect("a C/C++ compiler on PATH");
    assert!(
        out.status.success(),
        "{compiler} failed on {}:\n{}",
        src.display(),
        String::from_utf8_lossy(&out.stderr)
    );
}

/// Builds `src` into an executable with `compiler` and runs it; panics unless it exits 0.
fn run_ok(compiler: &str, std: &str, dir: &Path, src: &Path) {
    let exe = dir.join(src.file_stem().expect("a file name"));
    let exe_arg = exe.to_str().expect("utf-8 scratch path");
    compile_ok(compiler, &[std, "-O1", "-o", exe_arg], dir, src);
    let run = Command::new(&exe).output().expect("runs the probe");
    assert_eq!(
        run.status.code(),
        Some(0),
        "{} failed at the check on line {:?}",
        src.display(),
        run.status.code()
    );
}

/// Writes both headers for `n` and `sizes` into `dir`.
fn headers(dir: &Path, n: usize, sizes: Sizes) {
    std::fs::write(dir.join("kmr.h"), c_header(n, sizes)).expect("write h");
    std::fs::write(dir.join("kmr.hpp"), cpp_header()).expect("write hpp");
}

/// `_Static_assert`s that C's layout of every crossing type equals Rust's for `N` and the
/// sizes `D`, `I`, `M`, `W`.
fn layout_probe<const N: usize, const D: usize, const I: usize, const M: usize, const W: usize>()
-> String {
    let mut c = String::from("#include <stddef.h>\n#include \"kmr.h\"\n");
    let mut eq = |what: &str, rust: usize| {
        c.push_str(&format!("_Static_assert({what} == {rust}, \"{what}\");\n"));
    };
    eq("KMR_N", N);
    eq("KMR_HISTORY_DEPTH", D);
    eq("KMR_MAX_IMUS", I);
    eq("KMR_MAX_MICS", M);
    eq("KMR_MIC_WINDOW", W);
    eq("KMR_REASON_LEN", crate::REASON_LEN);
    macro_rules! layout {
        ($cty:literal, $rty:ty; $($cf:literal: $rf:ident),*) => {
            eq(concat!("sizeof(", $cty, ")"), size_of::<$rty>());
            eq(concat!("_Alignof(", $cty, ")"), align_of::<$rty>());
            $(eq(concat!("offsetof(", $cty, ", ", $cf, ")"), offset_of!($rty, $rf));)*
        };
    }
    layout!("kmr_duration", CDuration; "secs": secs, "nanos": nanos, "reserved": reserved);
    layout!("kmr_time", CTime; "tick": tick, "overrun_count": overrun_count,
        "elapsed": elapsed, "dt": dt, "last_overrun": last_overrun,
        "accumulated_overrun": accumulated_overrun, "late": late, "reserved": reserved);
    layout!("kmr_tick_faults", CTickFaults; "tick": tick, "reserved": reserved);
    layout!("kmr_slot", CSlot<N>; "sampled": sampled, "faults": faults);
    layout!("kmr_written", Written;);
    layout!("kmr_write_buf", WriteBuf<N>; "written": written);
    layout!("kmr_state", CState<N, D>; "slot": slot, "write": write,
        "telemetry_age": telemetry_age, "has_age": has_age, "has_temperature": has_temperature,
        "has_voltage": has_voltage, "has_hardware_error": has_hardware_error,
        "has_telemetry_age": has_telemetry_age, "temperature": temperature, "voltage": voltage,
        "age": age, "read_every": read_every, "filled": filled, "faults_filled": faults_filled,
        "sample_age": sample_age, "has_sample_age": has_sample_age, "signal": signal,
        "hardware_error": hardware_error, "reason": reason);
    layout!("kmr_axes", crate::CAxes; "x": x, "y": y, "z": z);
    layout!("kmr_imu_reading", CImu; "age": age, "dropped": dropped, "accel": accel,
        "gyro": gyro, "present": present, "has_accel": has_accel, "has_gyro": has_gyro,
        "reserved": reserved);
    layout!("kmr_mic_reading", CMic<W>; "dropped": dropped, "present": present,
        "rate_hz": rate_hz, "len": len, "reserved": reserved, "samples": samples);
    layout!("kmr_sensors", CSensors<I, M, W>; "imu": imu, "mic": mic);
    macro_rules! per_field {
        ($($field:ident: $ty:ident),*) => {
            $(
                eq(concat!("offsetof(kmr_slot, ", stringify!($field), ")"),
                    offset_of!(CSlot<N>, $field));
                eq(concat!("offsetof(kmr_write_buf, ", stringify!($field), ")"),
                    offset_of!(WriteBuf<N>, $field));
                eq(concat!("offsetof(kmr_written, ", stringify!($field), ")"),
                    offset_of!(Written, $field));
            )*
        };
    }
    macro_rules! per_flag {
        ($($flag:ident),*) => {
            $(eq(concat!("offsetof(kmr_tick_faults, ", stringify!($flag), ")"),
                offset_of!(CTickFaults, $flag));)*
        };
    }
    crate::for_each_field!(per_field);
    crate::for_each_flag!(per_flag);
    c
}

/// Checks the C layout probe for `N` and the sizes, and that `kmr.h` alone compiles as C++.
fn layout_matches<
    const N: usize,
    const D: usize,
    const I: usize,
    const M: usize,
    const W: usize,
>() {
    let dir = scratch(&format!("layout_n{N}_d{D}_i{I}_m{M}_w{W}"));
    let sizes = Sizes {
        history_depth: D,
        max_imus: I,
        max_mics: M,
        mic_window: W,
    };
    headers(&dir, N, sizes);
    let c = dir.join("probe.c");
    std::fs::write(&c, layout_probe::<N, D, I, M, W>()).expect("write probe.c");
    compile_ok("cc", &["-std=c11", "-fsyntax-only", "-x", "c"], &dir, &c);
    let cpp = dir.join("plain.cpp");
    std::fs::write(&cpp, "#include \"kmr.h\"\n#include \"kmr.hpp\"\n").expect("write plain");
    compile_ok(
        "c++",
        &["-std=c++17", "-fsyntax-only", "-x", "c++"],
        &dir,
        &cpp,
    );
    clean(&dir);
}

#[test]
fn the_c_layout_equals_rust_n1() {
    layout_matches::<1, 4, 4, 1, 1024>();
}

#[test]
fn the_c_layout_equals_rust_n3() {
    layout_matches::<3, 4, 4, 1, 1024>();
}

#[test]
fn the_c_layout_equals_rust_n22() {
    layout_matches::<22, 4, 4, 1, 1024>();
}

#[test]
fn the_c_layout_equals_rust_n64() {
    layout_matches::<64, 4, 4, 1, 1024>();
}

#[test]
fn the_c_layout_equals_rust_for_other_sizes() {
    layout_matches::<5, 2, 1, 2, 7>();
}

/// A C program that fills a state by hand and checks every kind of answer of the API; it exits
/// with the line of the first failed check.
const C_PROBE: &str = r#"
#include <string.h>
#include "kmr.h"

#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

int main(void) {
    static kmr_state s;
    static kmr_sensors se;
    kmr_time t;
    float v = -1.0f, row[KMR_N];
    uint32_t age = 7;
    double secs = 0.0;
    kmr_tick_faults f;
    const size_t g[2] = {2, 0};
    const size_t bad[2] = {1, 99};
    float gv[2] = {0.5f, 0.25f};
    size_t len = 0;
    kmr_axes a;
    memset(&s, 0, sizeof s);
    memset(&se, 0, sizeof se);
    memset(&t, 0, sizeof t);

    /* No History yet: every read says so, in the core's order. */
    CHECK(!kmr_q(&s, row));
    CHECK(kmr_q_at(&s, 0, &v) == KMR_HISTORY_OUT_OF_RANGE);
    CHECK(kmr_q_at(&s, 99, &v) == KMR_HISTORY_OUT_OF_RANGE && v == -1.0f);
    CHECK(!kmr_faults(&s, &f) && !kmr_stale_at(&s, 0));
    CHECK(!kmr_sample_age(&s, &age) && age == 7);

    /* Two slots; actuator 1 sampled only at depth 0. */
    s.filled = 2;
    s.faults_filled = 2;
    s.slot[0].sampled = KMR_INTERNAL_ALL;
    s.slot[1].sampled = KMR_INTERNAL_ALL & ~(uint64_t)2;
    s.slot[0].q[2] = 1.5f;
    s.slot[1].q[2] = 1.25f;
    s.slot[0].tau[0] = 3.0f;
    s.slot[0].faults.stale = 1u << 1;
    s.slot[1].faults.tick = 1u;
    CHECK(kmr_q_at(&s, 2, &v) == KMR_OK && v == 1.5f);
    CHECK(kmr_prev_q_at(&s, 2, 1, &v) == KMR_OK && v == 1.25f);
    CHECK(kmr_prev_q_at(&s, 1, 1, &v) == KMR_NO_SAMPLE);
    CHECK(kmr_prev_q_at(&s, 99, 1, &v) == KMR_OUT_OF_RANGE);
    CHECK(kmr_prev_q_at(&s, 99, 2, &v) == KMR_HISTORY_OUT_OF_RANGE);
    CHECK(kmr_q(&s, row) && row[2] == 1.5f);
    CHECK(kmr_position(&s, row) && row[2] == 1.5f);
    CHECK(kmr_effort(&s, row) && row[0] == 3.0f);
    CHECK(kmr_prev_q(&s, 1, row) == KMR_NO_SAMPLE);
    CHECK(kmr_group_q(&s, g, 2, gv) && gv[0] == 1.5f);
    CHECK(kmr_group_prev_q(&s, g, 2, 1, gv) == KMR_OK && gv[0] == 1.25f);
    CHECK(kmr_group_prev_q(&s, bad, 2, 1, gv) == KMR_NO_SAMPLE);
    CHECK(!kmr_group_q(&s, bad, 2, gv));

    /* Faults. */
    CHECK(kmr_faults(&s, &f) && kmr_faults_stale(&f) == 2u && !kmr_faults_overrun(&f));
    CHECK(kmr_stale_at(&s, 1) && !kmr_stale_at(&s, 0) && !kmr_stale_at(&s, 99));
    CHECK(kmr_group_stale(&s, g, 2) == false && kmr_group_stale(&s, bad, 2));
    CHECK(kmr_prev_faults(&s, 1, &f) == KMR_OK && kmr_faults_overrun(&f));
    CHECK(kmr_faults_any_actuator(&f) == 0 && !kmr_faults_is_clear(&f));
    CHECK(kmr_prev_faults(&s, 2, &f) == KMR_HISTORY_OUT_OF_RANGE);

    /* Ages and telemetry: Ok(None) is KMR_NONE, out untouched. */
    s.has_age = 1u;
    s.age[0] = 3;
    CHECK(kmr_age_at(&s, 0, &age) == KMR_OK && age == 3);
    CHECK(kmr_age_at(&s, 1, &age) == KMR_NONE && age == 3);
    CHECK(kmr_age_at(&s, KMR_N, &age) == KMR_OUT_OF_RANGE);
    CHECK(!kmr_group_sample_age(&s, g, 2, &age));
    s.has_temperature = 1u << 2;
    s.temperature[2] = 41.0f;
    CHECK(kmr_temperature_at(&s, 2, &v) == KMR_OK && v == 41.0f);
    CHECK(kmr_temperature_at(&s, 0, &v) == KMR_NONE);
    {
        float temps[KMR_N];
        bool present[KMR_N];
        kmr_temperatures(&s, temps, present);
        CHECK(present[2] && temps[2] == 41.0f && !present[0] && temps[0] == 0.0f);
        kmr_group_temperatures(&s, g, 2, temps, present);
        CHECK(present[0] && !present[1]);
    }
    s.read_every[1] = 4;
    CHECK(kmr_read_every(&s, 1, &age) == KMR_OK && age == 4);

    /* Writes: staged with their bit; a group is atomic. */
    CHECK(kmr_set_q_at(&s, 0.5f, 1) == KMR_OK);
    CHECK(kmr_set_q_at(&s, 0.5f, KMR_N) == KMR_OUT_OF_RANGE);
    CHECK(s.write.written.q == 2u && s.write.q[1] == 0.5f);
    CHECK(kmr_group_set_all_kd(&s, bad, 2, gv) == KMR_OUT_OF_RANGE && s.write.written.kd == 0);
    CHECK(kmr_group_set_all_kd(&s, g, 2, gv) == KMR_OK && s.write.written.kd == 5u);
    kmr_set_all_tau(&s, row);
    CHECK(s.write.written.tau == KMR_INTERNAL_ALL);

    /* Requests: the highest wins, its reason kept and cut to fit. */
    kmr_shutdown(&s, "first");
    kmr_shutdown(&s, "second");
    CHECK(s.signal == 1 && strcmp(s.reason, "first") == 0);
    kmr_emergency_stop(&s, "0123456789012345678901234567890123456789012345678901234567890123"
                           "0123456789012345678901234567890123456789012345678901234567890123");
    CHECK(s.signal == 2 && strlen(s.reason) == KMR_REASON_LEN - 1);
    kmr_shutdown(&s, NULL);
    CHECK(s.signal == 2);

    /* Time: seconds as Rust's as_secs_f64. */
    t.elapsed.secs = 2;
    t.elapsed.nanos = 500000000u;
    t.dt.nanos = 1000000u;
    CHECK(kmr_elapsed(&t) == 2.5 && kmr_dt(&t) == 0.001);
    CHECK(!kmr_last_overrun(&t, &secs));
    t.late = 1;
    t.last_overrun.nanos = 250000000u;
    CHECK(kmr_last_overrun(&t, &secs) && secs == 0.25);

    /* Sensors: NULL for None. */
    CHECK(kmr_imu(&se, 0) == NULL && kmr_mic(&se, 0) == NULL && kmr_imu(&se, 99) == NULL);
    se.imu[0].present = 1;
    se.imu[0].has_gyro = 1;
    se.imu[0].gyro.z = 0.5f;
    se.mic[0].present = 1;
    se.mic[0].len = 2;
    se.mic[0].samples[1] = -3;
    CHECK(kmr_imu(&se, 0) != NULL && !kmr_imu_accel(kmr_imu(&se, 0), &a));
    CHECK(kmr_imu_gyro(kmr_imu(&se, 0), &a) && a.z == 0.5f);
    CHECK(kmr_mic_samples(kmr_mic(&se, 0), &len)[1] == -3 && len == 2);
    return 0;
}
"#;

#[test]
fn the_c_api_answers_as_rust_does() {
    for n in [4usize, 64] {
        let dir = scratch(&format!("c_probe_n{n}"));
        headers(&dir, n, SIZES);
        let c = dir.join("api.c");
        std::fs::write(&c, C_PROBE).expect("write api.c");
        run_ok("cc", "-std=c11", &dir, &c);
        clean(&dir);
    }
}

/// A C++ program over `kmr.hpp`: Result and optional, a group, faults, the three controller
/// shapes of `KMR_CONTROLLER`, a missing context and an exception.
const CPP_PROBE: &str = r#"
#include <cstring>
#include <stdexcept>
#include "kmr.hpp"

#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)

struct Count {
    int n;
};

static void plain(const kmr::Time &time, kmr::State &state, const kmr::Sensors &sensors) {
    if (auto q = state.q_at(0)) {
        state.set_q_at(*q + 0.1f, 0);
    }
    if (!sensors.mic(0) && time.tick() > 2) {
        state.shutdown("tick");
    }
}
KMR_CONTROLLER(c_plain, plain)

static void counting(const kmr::Time &, kmr::State &state, const kmr::Sensors &, Count &count) {
    count.n += 1;
    state.group({1, 2}).set_all_qd({0.5f, 0.25f});
}
KMR_CONTROLLER(c_counting, counting)

static void throwing(const kmr::Time &, kmr::State &state, const kmr::Sensors &) {
    state.set_tau_at(1.0f, 0);
    throw std::runtime_error("boom");
}
KMR_CONTROLLER(c_throwing, throwing)

KMR_CONTROLLER(c_lambda, [](kmr::Time, kmr::State &state, kmr::Sensors, int &k) {
    k += 1;
    state.emergency_stop("lambda");
})

int main() {
    static kmr_state s;
    static kmr_sensors se;
    kmr_time t{};
    std::memset(&s, 0, sizeof s);
    s.filled = 1;
    s.faults_filled = 1;
    s.slot[0].sampled = KMR_INTERNAL_ALL;
    s.slot[0].q[0] = 1.0f;
    s.slot[0].faults.offline = 4u;
    s.has_telemetry_age = 1u;
    s.telemetry_age[0] = 9;

    kmr::State state(&s);
    CHECK(state.q().has_value() && state.position().has_value());
    CHECK(state.q_at(KMR_N).error() == KMR_OUT_OF_RANGE);
    CHECK(state.prev_kp(1).error() == KMR_HISTORY_OUT_OF_RANGE);
    CHECK(state.prev_q_at(0, 0).value_or(-1.0f) == 1.0f);
    CHECK(state.faults()->offline().get(2) && !state.faults()->is_clear());
    CHECK(state.age_at(0).ok() && !state.age_at(0)->has_value());
    CHECK(!state.age_at(KMR_N).ok());
    CHECK(state.telemetry_age_at(0) && **state.telemetry_age_at(0) == 9u);
    CHECK(!state.sample_age().has_value());
    CHECK(state.group(std::array<std::size_t, 2>{0, 1}).q().has_value());

    c_plain(&t, &s, &se, nullptr);
    CHECK(s.write.written.q == 1u && s.write.q[0] == 1.0f + 0.1f && s.signal == 0);
    t.tick = 3;
    c_plain(&t, &s, &se, nullptr);
    CHECK(s.signal == 1 && std::strcmp(s.reason, "tick") == 0);

    s.signal = 0;
    Count count{0};
    c_counting(&t, &s, &se, &count);
    CHECK(count.n == 1 && s.write.qd[2] == 0.25f && (s.write.written.qd & 6u) == 6u);
    c_counting(&t, &s, &se, nullptr);
    CHECK(count.n == 1 && s.signal == 1);

    s.signal = 0;
    s.write.written.tau = 0;
    c_throwing(&t, &s, &se, nullptr);
    CHECK(s.write.written.tau == 0 && s.write.written.q == 0 && s.signal == 1);
    CHECK(std::strcmp(s.reason, "C++ exception: boom") == 0);

    s.signal = 0;
    int k = 0;
    c_lambda(&t, &s, &se, &k);
    CHECK(k == 1 && s.signal == 2);
    return 0;
}
"#;

#[test]
fn the_cpp_api_answers_as_rust_does() {
    let dir = scratch("cpp_probe");
    headers(&dir, 4, SIZES);
    let cpp = dir.join("api.cpp");
    std::fs::write(&cpp, CPP_PROBE).expect("write api.cpp");
    run_ok("c++", "-std=c++17", &dir, &cpp);
    clean(&dir);
}

#[test]
fn the_c_header_carries_n_and_the_sizes() {
    for n in [1, 4, 64] {
        let h = c_header(n, SIZES);
        assert!(h.contains(&format!("#define KMR_N {n}\n")));
        assert!(h.contains("#define KMR_HISTORY_DEPTH 4\n"));
        assert!(h.contains("#define KMR_MIC_WINDOW 1024\n"));
        assert!(h.contains("    float q[KMR_N];\n"));
        assert!(h.contains("    uint64_t envelope;\n"));
        assert!(h.contains("static inline kmr_status kmr_set_tau_at(kmr_state *s, float value"));
        assert!(h.contains("static inline bool kmr_faults_unauthorized("));
    }
    assert!(c_header(64, SIZES).contains("UINT64_C(0xffffffffffffffff)"));
    assert!(c_header(3, SIZES).contains("#define KMR_INTERNAL_ALL UINT64_C(0x7)"));
}

/// A group of `sources` built for `n`, defining `symbols`.
fn group(sources: &[&str], n: usize, symbols: &[&str]) -> Group {
    let mut symbols: Vec<String> = symbols.iter().map(|s| (*s).to_owned()).collect();
    symbols.sort();
    Group {
        sources: sources.iter().map(|s| (*s).to_owned()).collect(),
        abi: abi(n, SIZES),
        symbols,
    }
}

/// The two comment lines at the top of `kmr_ffi_abi.rs`.
const TABLE_TOP: &str = "// Every C and C++ function build.rs compiled, with the N, History depth, \
                         IMU slots,\n// microphone slots and microphone window it was compiled \
                         for.\n";

#[test]
fn the_abi_table_maps_each_function_to_its_group_sorted() {
    let groups = [
        group(&["c/clap.c"], 22, &["move_on_clap", "_ZN4clap4stepEv"]),
        group(&["cpp/leg.cpp"], 3, &["walk_cpp", "abc"]),
    ];
    assert_eq!(
        abi_table(&groups),
        format!(
            "{TABLE_TOP}&[\n    (\"abc\", [3, 4, 4, 1, 1024]),\n    (\"move_on_clap\", [22, 4, 4, \
             1, 1024]),\n    (\"walk_cpp\", [3, 4, 4, 1, 1024]),\n]\n"
        )
    );
    assert_eq!(abi_table(&[]), format!("{TABLE_TOP}&[\n]\n"));
}

#[test]
fn only_identifiers_that_are_not_mangled_reach_the_table() {
    for name in ["swing", "_swing", "a1_b2", "Walk"] {
        assert!(nameable(name), "{name}");
    }
    for name in ["", "1a", "_ZN3kmr5State1qEv", "a.b", "a$b", "fn::x", "é"] {
        assert!(!nameable(name), "{name}");
    }
}

#[test]
fn the_nm_beside_the_archiver_swaps_its_ar_word() {
    let beside = |ar: &str| nm_beside(Path::new(ar)).map(|p| p.display().to_string());
    assert_eq!(beside("ar").as_deref(), Some("nm"));
    assert_eq!(
        beside("aarch64-linux-gnu-ar").as_deref(),
        Some("aarch64-linux-gnu-nm")
    );
    assert_eq!(
        beside("/usr/bin/aarch64-linux-gnu-gcc-ar").as_deref(),
        Some("/usr/bin/aarch64-linux-gnu-gcc-nm")
    );
    assert_eq!(beside("llvm-ar").as_deref(), Some("llvm-nm"));
    assert_eq!(beside("llvm-ar-18").as_deref(), Some("llvm-nm-18"));
    assert_eq!(
        beside("x86_64-w64-mingw32-ar.exe").as_deref(),
        Some("x86_64-w64-mingw32-nm.exe")
    );
    assert_eq!(beside("emar"), None);
    assert_eq!(beside("lib.exe"), None);
    assert_eq!(beside("/opt/arm/bin/"), None);
}

#[test]
fn nm_is_the_override_alone_else_beside_then_llvm_then_plain() {
    let tools = nm_tools(Some("/x/my-nm".into()), Some(Path::new("ar")));
    assert_eq!(tools, [PathBuf::from("/x/my-nm")]);
    let tools = nm_tools(Some("".into()), Some(Path::new("aarch64-linux-gnu-ar")));
    assert_eq!(
        tools,
        ["aarch64-linux-gnu-nm", "llvm-nm", "nm"].map(PathBuf::from)
    );
    assert_eq!(
        nm_tools(None, Some(Path::new("ar"))),
        ["nm", "llvm-nm"].map(PathBuf::from)
    );
    assert_eq!(nm_tools(None, None), ["llvm-nm", "nm"].map(PathBuf::from));
}

/// `nm -P -g --defined-only` of one C++ object in an archive, as GNU nm 2.47 prints it.
const GNU_NM: &str = "\
libkmr_ffi_cpp_0.a[a.o]:
ctrl T b 7
g_bss B 0 4
g_data D 0 4
g_ro R 0 4
_Z3inlv W 0 b
_Z3usev T 12 66
_ZNKSt8optionalISt5arrayIfLm3EEEcvbEv W 0 1a
_ZTV4Base V 0 18
_ZZ3usevE1x u 0 4
";

/// Two objects of an archive, as llvm-nm prints them.
const LLVM_NM: &str = "
a.o:
_Z3inlv W 0 b
_Z3usev T 12 66
_ZNKSt8optionalISt5arrayIfLm3EEEcvbEv W 0 1a
ctrl T b 7
g_bss B 0 4
g_data D 0 4

b.o:
walk T 0 4
";

#[test]
fn nm_output_gives_the_strong_functions_only() {
    assert_eq!(strong_functions(GNU_NM), ["ctrl", "_Z3usev"]);
    assert_eq!(strong_functions(LLVM_NM), ["_Z3usev", "ctrl", "walk"]);
    assert!(strong_functions("").is_empty());
    assert!(strong_functions("/my dir/lib.a[x.o]:\n").is_empty());
}

#[test]
fn a_function_two_groups_define_is_a_duplicate() {
    let first = group(&["a.c"], 1, &["a_c", "shared"]);
    let second = group(&["b.cpp", "util.c"], 2, &["b_cpp", "shared"]);
    let err = check_duplicates(std::slice::from_ref(&first), &second).expect_err("duplicate");
    assert!(
        matches!(&err, Error::DuplicateSymbol { symbol, first, second }
            if symbol == "shared" && first == &["a.c"] && second == &["b.cpp", "util.c"]),
        "{err:?}"
    );
    let msg = err.to_string();
    assert!(
        msg.contains("`shared` is defined by two compile calls"),
        "{msg}"
    );
    assert!(msg.contains("[a.c] and one for [b.cpp, util.c]"), "{msg}");
    let third = group(&["c.c"], 2, &["c_c"]);
    assert!(check_duplicates(&[first, second], &third).is_ok());
}

#[test]
fn no_nm_names_the_tools_and_the_fix() {
    let tools = [PathBuf::from("/nonexistent/nm")];
    let err = list_symbols(&tools, Path::new("lib.a")).expect_err("no such tool");
    assert!(
        matches!(&err, Error::NoSymbolTool { tried, .. } if tried == &["/nonexistent/nm"]),
        "{err:?}"
    );
    let msg = err.to_string();
    assert!(msg.contains("tried /nonexistent/nm"), "{msg}");
    assert!(msg.contains("set NM"), "{msg}");
}

#[test]
fn the_cpp_header_lives_in_a_namespace_per_n_and_sizes() {
    assert!(c_header(3, SIZES).contains("#define KMR_INTERNAL_ABI abi_n3_h4_i4_m1_w1024\n"));
    let h = cpp_header();
    assert!(h.contains("\ninline namespace KMR_INTERNAL_ABI {\n"));
    assert!(h.contains("\n} // inline namespace KMR_INTERNAL_ABI\n} // namespace kmr\n"));
}

#[test]
fn no_engine_item_crosses() {
    let banned = [
        "Robot",
        "robot",
        "run",
        "builder",
        "Builder",
        "PreInit",
        "Init",
        "PostInit",
        "First",
        "EachTick",
        "Last",
        "Schedule",
        "Signal",
        "Bus",
        "Hal",
        "hal",
        "HAL",
        "adapter_t",
    ];
    let headers = [c_header(4, SIZES), cpp_header()];
    for h in &headers {
        let words: Vec<&str> = h
            .split(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
            .collect();
        for word in banned {
            assert!(!words.contains(&word), "header names `{word}`");
        }
    }
}

#[test]
fn sources_split_by_language() {
    let (c, cpp) = split(&["a.c", "b.cpp", "c.cc", "d.cxx"]).expect("all known");
    assert_eq!(c, ["a.c"]);
    assert_eq!(cpp, ["b.cpp", "c.cc", "d.cxx"]);
    let err = split(&["a.c", "swing.rs"]).expect_err("not C");
    assert!(matches!(&err, Error::UnknownSource(src) if src == "swing.rs"));
}

#[test]
fn compile_outside_a_build_script_is_an_error() {
    let err = Build::new(4, SIZES)
        .compile(&["swing.c"])
        .expect_err("no OUT_DIR");
    assert!(matches!(err, Error::NoOutDir));
}

#[test]
fn an_n_no_mask_holds_is_an_error() {
    for n in [0, 65] {
        let err = Build::new(n, SIZES)
            .compile(&["swing.c"])
            .expect_err("bad N");
        assert!(matches!(err, Error::ActuatorCount(got) if got == n));
    }
}

#[test]
fn a_header_that_cannot_be_written_is_an_error() {
    let missing = Path::new("/nonexistent/kmr.h");
    let err = write(missing, "").expect_err("no such folder");
    assert!(matches!(&err, Error::Write(path, _) if path == missing));
}

#[test]
fn a_preset_gives_robot_macro_n() {
    assert_eq!(robot("XmBench", SIZES).n().ok(), Some(4));
    assert_eq!(robot("Rx28Chain", SIZES).n().ok(), Some(10));
}

#[test]
fn an_unknown_preset_is_robot_macro_message() {
    let err = robot("Nope", SIZES).n().expect_err("no such preset");
    assert!(matches!(&err, Error::UnknownPreset(_)));
    assert_eq!(
        err.to_string(),
        "kmr: no preset named Nope; presets: XmBench, Rx28Chain"
    );
}

#[test]
fn a_descriptor_path_is_relative_to_the_manifest() {
    let build = model("../kmr_model_core/presets/rx28_chain.urdf", SIZES);
    assert_eq!(build.n().ok(), Some(10));
}

#[test]
fn a_bad_descriptor_is_robot_macro_message() {
    let err = model("no/such/robot.urdf", SIZES).n().expect_err("no file");
    assert!(matches!(&err, Error::Descriptor(_)));
    assert!(
        err.to_string()
            .starts_with("kmr: cannot read robot descriptor "),
        "{err}"
    );
    let err = model("robot.yaml", SIZES).n().expect_err("unknown syntax");
    assert!(err.to_string().ends_with("expected .urdf, .ron or .json"));
}

#[test]
fn a_robot_is_resolved_before_out_dir() {
    let err = robot("Nope", SIZES)
        .compile(&["swing.c"])
        .expect_err("no preset");
    assert!(matches!(err, Error::UnknownPreset(_)));
}

#[test]
fn cargo_watches_each_source_folder_once() {
    let paths = watched(&["src/a.c", "src/b.cpp", "lib/c.c", "root.c", "./dot.c"]);
    let expect = ["src", "lib", "root.c", "./dot.c"].map(Path::new);
    assert_eq!(paths, expect);
}
