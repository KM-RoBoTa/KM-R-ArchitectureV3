#!/usr/bin/env bash
# setup-check.sh: the host against SRS NFR-1 and the procedure (FR-001, FR-024 (SRS NFR-1,
# NFR-2); quickstart §7). Prints one line per check on stdout and stops at the first failure,
# naming its field, with exit 1. Run from the repository root, as root: sudo -E keeps the
# operator fields.

set -euo pipefail

here=$(dirname "${BASH_SOURCE[0]}")
# shellcheck source=profile.env
source "$here/profile.env"
# shellcheck source=meta.sh
source "$here/meta.sh"

ok() { printf 'ok    %s: %s\n' "$1" "$2"; }

fail() {
    printf 'FAIL  %s: %s\n' "$1" "$2"
    exit 1
}

# The revision, from the device tree or /proc/cpuinfo; neither is a stop of its own.
rev=$(board_revision)
if [[ -z $rev ]]; then
    fail board_revision "no revision source (/proc/device-tree/system/linux,revision, Revision in /proc/cpuinfo)"
fi
if [[ $rev =~ 311[45]$ ]]; then
    ok board_revision "$rev"
else
    fail board_revision "$rev (want …3114 or …3115, Raspberry Pi 4 Model B rev 1.4 or 1.5)"
fi

d=$(distro)
[[ $d == "Arch Linux ARM" ]] && ok distro "$d" || fail distro "$d (want Arch Linux ARM)"

release=$(uname -r)
IFS=. read -r major minor _ <<<"$release"
minor=${minor%%[^0-9]*}
if [[ $major == 6 && ${minor:-0} -ge 14 ]]; then
    ok kernel "$release, PREEMPT_RT $(kernel_rt)"
else
    fail kernel "$release (want 6.x with 6.14 <= v < 7.0)"
fi

g=$(governor)
[[ $g == "$GOVERNOR" ]] && ok governor "$g on every core" || fail governor "$g (want $GOVERNOR on every core)"

for var in BENCH_SUPPLY_MV BENCH_SUPPLY_METER BENCH_AMBIENT_MC BENCH_THERMOMETER; do
    [[ -n ${!var:-} ]] || fail "$var" "unset (operator field, set it in the environment)"
done
for var in BENCH_SUPPLY_MV BENCH_AMBIENT_MC; do
    [[ ${!var} =~ ^[0-9]+$ ]] || fail "$var" "${!var} (want an integer, mV or m°C)"
done
ok operator "supply $BENCH_SUPPLY_MV mV ($BENCH_SUPPLY_METER), ambient $BENCH_AMBIENT_MC m°C ($BENCH_THERMOMETER)"

[[ -z ${RUST_LOG+set} ]] && ok rust_log unset || fail rust_log "set to '$RUST_LOG' (want unset)"

for tool in stress-ng chrt taskset size strip; do
    command -v "$tool" >/dev/null || fail "$tool" "not found"
done
ok tools "stress-ng chrt taskset size strip"

[[ $(id -u) == 0 ]] && ok root "uid 0" || fail root "uid $(id -u) (want root: chrt, the governor, sched_rt_runtime_us)"
