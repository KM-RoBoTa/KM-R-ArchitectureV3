#!/usr/bin/env bash
# budget.sh [--write all]: the controller budget at dt = 1 ms and the engine overhead it leaves
# (FR-010 (SRS NFR-8 d, NFR-11); research R8, R9). Bisects KMR_BENCH_WORK_NS, the calibrated
# workload per tick, for the largest one held HOLD_S with zero overruns, to BUDGET_STEP_NS
# (1 us); one `tick` process per candidate (a failed window pollutes the next one), cell
# <nonrt|rt>-w<work_ns>. Then one `tick` run at dt = 10 us, cell <nonrt|rt>-floor-w0, whose
# gap_after_late_min_ns is the engine's own path between ticks. Then one summary report,
# measurement budget, cell <nonrt|rt>-<none|all>. --write all makes the probe write every
# actuator each tick. Prints every report path, the summary last. Run from the repository root.

set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=profile.env
source "$here/profile.env"

write=none
while (($#)); do
    case $1 in
    --write)
        [[ ${2:-} == all ]] || {
            echo "usage: budget.sh [--write all]" >&2
            exit 64
        }
        write=all
        shift 2
        ;;
    --hw)
        echo "budget.sh: --hw waits on the read/write-only tick split (bench T057, T059)" >&2
        exit 64
        ;;
    *)
        echo "usage: budget.sh [--write all]" >&2
        exit 64
        ;;
    esac
done

cd "$(git rev-parse --show-toplevel)"
tick=$(bench_bin tick)
kernel=$(bench_kernel)
dt=1000000
last="" held=""

# Runs `tick` at dt $1 with workload $2 on cell $3; sets `last`; stops the search on a run that
# is not complete; true when both overrun counts are 0.
try() {
    local status
    last=$(bench_run "$tick" budget "$3" KMR_BENCH_DT_NS="$1" KMR_BENCH_WORK_NS="$2" \
        KMR_BENCH_WRITE="$write" KMR_BENCH_TICKS="$((HOLD_S * 1000000000 / $1 + WARMUP))" \
        KMR_BENCH_WARMUP="$WARMUP") || true
    echo "$last"
    status=$(bench_key "$last" status run)
    [[ $status == complete ]] || stop "$status" "$(bench_key "$last" reason run)"
    [[ $(bench_key "$last" bench_overruns figures) == 0 &&
        $(bench_key "$last" engine_overruns figures) == 0 ]]
}

# Writes the summary of a search that ended early with status $1, reason $2, and exits.
stop() {
    local out
    out=$(bench_next_report "$(dirname "$last")" budget "$kernel-$write")
    bench_head "$last" budget "$kernel-$write" |
        awk -v s="$1" -v r="candidate $(basename "$last"): $2" '
            /^\[/ { block = $0 }
            block == "[run]" && $1 == "status" && $2 == "=" { print "status = \"" s "\""; print "reason = \"" r "\""; next }
            block == "[run]" && $1 == "reason" && $2 == "=" { next }
            { print }' >"$out"
    echo "$out"
    [[ $1 == failed ]] && exit 1
    exit 0
}

budget=""
zero_held=no
try "$dt" 0 "$kernel-w0" && zero_held=yes
first=$last
if [[ $zero_held == yes ]]; then
    lo=0 hi=$dt budget=0 held=$last
    while ((hi - lo > BUDGET_STEP_NS)); do
        mid=$(((lo + hi) / 2 / BUDGET_STEP_NS * BUDGET_STEP_NS))
        ((mid > lo && mid < hi)) || break
        if try "$dt" "$mid" "$kernel-w$mid"; then
            lo=$mid budget=$mid held=$last
        else
            hi=$mid
        fi
    done
fi
try "$CEIL_DT_MIN_NS" 0 "$kernel-floor-w0" || true
floor=$last

# The head comes from the held candidate, else workload 0: never from the 10 us run, whose
# flags (often inconsistent, see probe.rs ORIGIN) are not the budget's.
head=${held:-$first}
out=$(bench_next_report "$(dirname "$floor")" budget "$kernel-$write")
{
    bench_head "$head" budget "$kernel-$write"
    echo "[figures]"
    printf 'loop_thread_allocs = %s\n' "$(bench_key "$head" loop_thread_allocs figures)"
    echo 'locks = "not measured"'
    echo 'method = "budget"'
    if [[ -n $budget ]]; then
        printf 'budget_ns = %s\noverhead_ns = %s\n' "$budget" "$((dt - budget))"
    fi
    gap=$(bench_key "$floor" gap_after_late_min_ns figures)
    [[ -n $gap ]] && printf 'gap_floor_min_ns = %s\n' "$gap"
} >"$out"
echo "$out"
