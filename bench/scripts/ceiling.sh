#!/usr/bin/env bash
# ceiling.sh: the tick-rate ceiling on the running kernel (FR-007, FR-010 second view (SRS NFR-8 a,
# FR-3.1); research R9, R10; SC-008). One `tick` process per candidate dt, held HOLD_S, cell
# <nonrt|rt>-dt<dt_ns>: first CEIL_DT_MIN_NS (10 us), then CEIL_DT_MAX_NS (1 ms), then a bisection
# on a log scale between them until the bracket is within CEIL_STOP_PERMILLE of dt. Never below
# 10 us. Then one summary report, measurement ceiling, cell <nonrt|rt>-idle. Prints every report
# path, the summary last. Run from the repository root.

set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=profile.env
source "$here/profile.env"

cd "$(git rev-parse --show-toplevel)"
tick=$(bench_bin tick)
kernel=$(bench_kernel)
last="" held=""

# Runs candidate $1 (dt in ns); sets `last` to its report; true when it holds: complete, and
# both the bench's and the engine's overrun counts are 0 (research R10).
try() {
    local dt=$1 ticks status
    ticks=$((HOLD_S * 1000000000 / dt + WARMUP))
    last=$(bench_run "$tick" ceiling "$kernel-dt$dt" KMR_BENCH_DT_NS="$dt" \
        KMR_BENCH_TICKS="$ticks" KMR_BENCH_WARMUP="$WARMUP") || true
    echo "$last"
    status=$(bench_key "$last" status run)
    case $status in
    complete) ;;
    *) stop "$status" "$(bench_key "$last" reason run)" ;;
    esac
    [[ $(bench_key "$last" bench_overruns figures) == 0 &&
        $(bench_key "$last" engine_overruns figures) == 0 ]]
}

# Writes the summary of a search that ended early with status $1, reason $2, and exits.
stop() {
    local out
    out=$(bench_next_report "$(dirname "$last")" ceiling "$kernel-idle")
    {
        bench_head "$last" ceiling "$kernel-idle" |
            awk -v s="$1" -v r="candidate $(basename "$last"): $2" '
                /^\[/ { block = $0 }
                block == "[run]" && $1 == "status" && $2 == "=" { print "status = \"" s "\""; print "reason = \"" r "\""; next }
                block == "[run]" && $1 == "reason" && $2 == "=" { next }
                { print }'
    } >"$out"
    echo "$out"
    [[ $1 == failed ]] && exit 1
    exit 0
}

floor_held=no
ceiling=""
if try "$CEIL_DT_MIN_NS"; then
    floor_held=yes ceiling=$CEIL_DT_MIN_NS held=$last
elif try "$CEIL_DT_MAX_NS"; then
    lo=$CEIL_DT_MIN_NS hi=$CEIL_DT_MAX_NS held=$last
    while (((hi - lo) * 1000 > hi * CEIL_STOP_PERMILLE)); do
        mid=$(awk -v a="$lo" -v b="$hi" 'BEGIN { printf "%d", sqrt(a * b) }')
        ((mid > lo && mid < hi)) || break
        if try "$mid"; then
            hi=$mid held=$last
        else
            lo=$mid
        fi
    done
    ceiling=$hi
fi

out=$(bench_next_report "$(dirname "$last")" ceiling "$kernel-idle")
{
    bench_head "$last" ceiling "$kernel-idle"
    echo "[figures]"
    if [[ -n $held ]]; then
        printf 'loop_thread_allocs = %s\n' "$(bench_key "$held" loop_thread_allocs figures)"
    else
        echo 'loop_thread_allocs = "not measured"'
    fi
    echo 'locks = "not measured"'
    if [[ -n $ceiling ]]; then
        printf 'ceiling_dt_ns = %s\n' "$ceiling"
        printf 'ceiling_hz = %s\n' "$((1000000000 / ceiling))"
    fi
    printf 'floor_held = "%s"\n' "$floor_held"
    if [[ -n $held ]]; then
        for key in min p50; do
            value=$(bench_key "$held" "gap_on_time_${key}_ns" figures)
            [[ -n $value ]] && printf 'gap_ceiling_%s_ns = %s\n' "$key" "$value"
        done
    fi
} >"$out"
echo "$out"
