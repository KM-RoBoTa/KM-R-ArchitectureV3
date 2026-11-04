#!/usr/bin/env bash
# meta.sh: the [meta] block of a bench report (contracts/results-schema.md; FR-001, FR-002
# (SRS NFR-1, NFR-8); research R12).
#
#   meta.sh start     the [meta] header and every field known before the run
#   meta.sh end       the fields read after the run: throttled_end, soc_mc_end
#   meta.sh host-id   the host id alone, for the report path
#
# Writes to stdout. Run from the repository root. bench_run (profile.env) sets BENCH_AFFINITY,
# BENCH_SCHED, BENCH_PROFILE, BENCH_FEATURES and BENCH_LOAD_COMMAND, the last three from its
# caller; compile-time.sh sets BENCH_PROFILE and BENCH_FEATURES itself; the operator sets
# BENCH_SUPPLY_MV, BENCH_SUPPLY_METER, BENCH_AMBIENT_MC and BENCH_THERMOMETER. A missing
# operator field prints "meta: missing <var>" on stderr and exits 2 before any output.
# Optional: BENCH_HOST_ID replaces the derived host id; BENCH_CHAIN (e.g. u2d2-FT8ISZ6H) is
# appended to it; BENCH_DIRTY replaces the working-tree check, for a build of a clean copy of
# HEAD (compile-time.sh). Sourced, it defines its functions and runs nothing.

set -euo pipefail

# key = "<text>": '"' becomes "'", newlines and tabs become spaces, cut to 512 bytes.
txt() {
    local v=${2//\"/\'}
    v=${v//$'\n'/ }
    v=${v//$'\r'/ }
    v=${v//$'\t'/ }
    printf '%s = "%s"\n' "$1" "${v:0:512}"
}

# key = <integer>
int() { printf '%s = %s\n' "$1" "$2"; }

# The board revision in lower-case hex without leading zeros, empty when no source gives one.
board_revision() {
    local f=/proc/device-tree/system/linux,revision rev=""
    if [[ -r $f ]]; then
        rev=$(od -An -tx1 -N4 "$f" | tr -d ' \n')
    elif [[ -r /proc/cpuinfo ]]; then
        rev=$(sed -n 's/^Revision[[:space:]]*:[[:space:]]*\([0-9a-fA-F]*\).*/\1/p' /proc/cpuinfo | head -n1)
    fi
    rev=${rev,,}
    while [[ $rev == 0* ]]; do rev=${rev#0}; done
    printf '%s' "$rev"
}

board_model() {
    if [[ -r /proc/device-tree/model ]]; then
        tr -d '\0' </proc/device-tree/model
    elif [[ -r /sys/devices/virtual/dmi/id/product_name ]]; then
        cat /sys/devices/virtual/dmi/id/product_name
    else
        printf 'unknown'
    fi
}

distro() {
    local name=""
    if [[ -r /etc/os-release ]]; then
        name=$(sed -n 's/^NAME="\{0,1\}\([^"]*\)"\{0,1\}$/\1/p' /etc/os-release | head -n1)
    fi
    printf '%s' "${name:-unknown}"
}

kernel_rt() {
    if [[ $(cat /sys/kernel/realtime 2>/dev/null || true) == 1 ]] || uname -v | grep -q PREEMPT_RT; then
        printf 'yes'
    else
        printf 'no'
    fi
}

# One governor for every core, "mixed" when they differ, "unknown" without cpufreq.
governor() {
    local g seen=""
    for f in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do
        [[ -r $f ]] || continue
        g=$(<"$f")
        if [[ -z $seen ]]; then
            seen=$g
        elif [[ $g != "$seen" ]]; then
            printf 'mixed'
            return
        fi
    done
    printf '%s' "${seen:-unknown}"
}

# The throttle flags: vcgencmd's hex word, else the raspberrypi-hwmon under-voltage alarm.
throttled() {
    local out
    if command -v vcgencmd >/dev/null && out=$(vcgencmd get_throttled 2>/dev/null); then
        printf '%s' "${out#throttled=}"
        return
    fi
    for d in /sys/class/hwmon/hwmon*; do
        if [[ $(cat "$d/name" 2>/dev/null || true) == raspberrypi-hwmon && -r $d/in0_lcrit_alarm ]]; then
            printf 'hwmon:%s' "$(<"$d/in0_lcrit_alarm")"
            return
        fi
    done
    printf 'unavailable'
}

soc_mc() {
    local f=/sys/class/thermal/thermal_zone0/temp
    [[ -r $f ]] && cat "$f"
}

host_id() {
    local id rev
    if [[ -n ${BENCH_HOST_ID:-} ]]; then
        id=$BENCH_HOST_ID
    elif rev=$(board_revision) && [[ -n $rev ]] && board_model | grep -q 'Raspberry Pi 4'; then
        id=rpi4-$rev
    else
        id=$(cat /proc/sys/kernel/hostname 2>/dev/null || printf 'host')
    fi
    [[ -n ${BENCH_CHAIN:-} ]] && id=$id-$BENCH_CHAIN
    id=${id//[^A-Za-z0-9._-]/-}
    printf '%s' "$id"
}

# "yes" only on the reference bench of SRS NFR-1: revision …3114 or …3115, Arch Linux ARM,
# kernel 6.14 <= v < 7.0.
reference() {
    local rev=$1 major minor
    IFS=. read -r major minor _ <<<"$(uname -r)"
    minor=${minor%%[^0-9]*}
    if [[ $rev =~ 311[45]$ && $(distro) == "Arch Linux ARM" && $major == 6 && ${minor:-0} -ge 14 ]]; then
        printf 'yes'
    else
        printf 'no'
    fi
}

require() {
    local var
    for var in BENCH_SUPPLY_MV BENCH_SUPPLY_METER BENCH_AMBIENT_MC BENCH_THERMOMETER; do
        if [[ -z ${!var:-} ]]; then
            echo "meta: missing $var" >&2
            exit 2
        fi
    done
    for var in BENCH_SUPPLY_MV BENCH_AMBIENT_MC; do
        if [[ ! ${!var} =~ ^[0-9]+$ ]]; then
            echo "meta: missing $var (not an integer: ${!var})" >&2
            exit 2
        fi
    done
}

start() {
    require
    local rev runtime soc commit dirty
    rev=$(board_revision)
    echo "[meta]"
    int schema 1
    txt host_id "$(host_id)"
    txt board_model "$(board_model)"
    txt board_revision "${rev:-none}"
    txt distro "$(distro)"
    txt kernel "$(uname -r) $(uname -v)"
    txt kernel_rt "$(kernel_rt)"
    txt governor "$(governor)"
    txt affinity "${BENCH_AFFINITY:-none}"
    txt sched "${BENCH_SCHED:-unknown}"
    runtime=$(cat /proc/sys/kernel/sched_rt_runtime_us 2>/dev/null || true)
    if [[ $runtime =~ ^[0-9]+$ ]]; then
        int sched_rt_runtime_us "$runtime"
    elif [[ $runtime == -1 ]]; then
        txt sched_rt_runtime_us unlimited
    fi
    txt rust_log "${RUST_LOG-unset}"
    int supply_mv "$BENCH_SUPPLY_MV"
    txt supply_meter "$BENCH_SUPPLY_METER"
    int ambient_mc "$BENCH_AMBIENT_MC"
    txt ambient_meter "$BENCH_THERMOMETER"
    txt throttled_start "$(throttled)"
    if soc=$(soc_mc) && [[ $soc =~ ^[0-9]+$ ]]; then int soc_mc_start "$soc"; fi
    commit=$(git rev-parse HEAD 2>/dev/null || printf 'unknown')
    txt commit "$commit"
    if [[ -n ${BENCH_DIRTY:-} ]]; then
        dirty=$BENCH_DIRTY
    elif [[ -n $(git status --porcelain --untracked-files=no 2>/dev/null) ]]; then
        dirty=yes
    else
        dirty=no
    fi
    txt dirty "$dirty"
    txt profile "${BENCH_PROFILE:-release}"
    txt features "${BENCH_FEATURES:-}"
    txt load_command "${BENCH_LOAD_COMMAND:-none}"
    txt reference "$(reference "$rev")"
}

end() {
    local soc
    txt throttled_end "$(throttled)"
    if soc=$(soc_mc) && [[ $soc =~ ^[0-9]+$ ]]; then int soc_mc_end "$soc"; fi
}

# Sourced (setup-check.sh): the functions only.
[[ ${BASH_SOURCE[0]} == "$0" ]] || return 0

case ${1:-} in
start) start ;;
end) end ;;
host-id) host_id ;;
*)
    echo "usage: meta.sh start|end|host-id" >&2
    exit 64
    ;;
esac
