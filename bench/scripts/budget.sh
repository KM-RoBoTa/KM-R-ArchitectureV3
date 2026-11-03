#!/usr/bin/env bash
# budget.sh [--write all] [--hw]: the controller budget at dt = 1 ms and the engine overhead it
# leaves (FR-010 (SRS NFR-8 d, NFR-11); research R8, R9). Bisects KMR_BENCH_WORK_NS, the
# calibrated workload per tick, for the largest one held HOLD_S with zero overruns, to
# BUDGET_STEP_NS (1 us); one `tick` process per candidate (a failed window pollutes the next
# one), cell <nonrt|rt>-w<work_ns>. Then one `tick` run at dt = 10 us, cell
# <nonrt|rt>-floor-w0, whose gap_after_late_min_ns is the engine's own path between ticks. Then
# one summary report, measurement budget, cell <nonrt|rt>-<none|all>. --write all makes the
# probe write every actuator each tick. Prints every report path, the summary last. Run from the
# repository root.
#
# --hw (research R16; FR-015, FR-017 (SRS FR-5.3, FR-5.10, NFR-11)): needs the reference chain
# (bench/PROCEDURE.md "Reference chain"). First the simulated search with --write all, then the
# same search on `tick_hw` with --write all, cells <nonrt|rt>-hw-w<work_ns>; "held" counts the
# write-only ticks' overruns only, read ticks are reported apart. No 10 us floor run on the
# chain. Summary cell <nonrt|rt>-hw-all: budget_ns and overhead_ns of the chain,
# engine_overhead_ns of the simulated search next to io_write_tick_ns = budget(simulated) -
# budget(hardware) and io_read_ns = read_jitter_p50_ns - write_only_jitter_p50_ns of the held
# candidate. No verdict.

set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=profile.env
source "$here/profile.env"

usage() {
    echo "usage: budget.sh [--write all] [--hw]" >&2
    exit 64
}

write=none hw=no
while (($#)); do
    case $1 in
    --write)
        [[ ${2:-} == all ]] || usage
        write=all
        shift 2
        ;;
    --hw)
        hw=yes
        shift
        ;;
    *) usage ;;
    esac
done

cd "$(git rev-parse --show-toplevel)"
kernel=$(bench_kernel)
dt=1000000
last="" held=""
if [[ $hw == yes ]]; then
    # DECISION: --hw always writes every actuator: the write tick is what the chain's budget
    # measures (research R16), so --hw alone means --write all.
    write=all
    sim_paths=$(mktemp)
    trap 'rm -f "$sim_paths"' EXIT
    "$here/budget.sh" --write all | tee "$sim_paths"
    sim=$(tail -n 1 "$sim_paths")
    tick=$(bench_bin tick_hw)
    tag=$kernel-hw
else
    tick=$(bench_bin tick)
    tag=$kernel
fi
summary_cell=$tag-$write

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
    if [[ $hw == yes ]]; then
        # The engine count holds the read ticks too, which overrun at 1 ms by design (R16).
        [[ $(bench_key "$last" write_only_bench_overruns figures) == 0 &&
            $(bench_key "$last" write_only_ticks figures) -gt 0 ]]
    else
        [[ $(bench_key "$last" bench_overruns figures) == 0 &&
            $(bench_key "$last" engine_overruns figures) == 0 ]]
    fi
}

# Writes the summary of a search that ended early with status $1, reason $2, and exits.
stop() {
    local out
    out=$(bench_next_report "$(dirname "$last")" budget "$summary_cell")
    bench_head "$last" budget "$summary_cell" |
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
try "$dt" 0 "$tag-w0" && zero_held=yes
first=$last
if [[ $zero_held == yes ]]; then
    lo=0 hi=$dt budget=0 held=$last
    while ((hi - lo > BUDGET_STEP_NS)); do
        mid=$(((lo + hi) / 2 / BUDGET_STEP_NS * BUDGET_STEP_NS))
        ((mid > lo && mid < hi)) || break
        if try "$dt" "$mid" "$tag-w$mid"; then
            lo=$mid budget=$mid held=$last
        else
            hi=$mid
        fi
    done
fi
floor=""
if [[ $hw == no ]]; then
    try "$CEIL_DT_MIN_NS" 0 "$kernel-floor-w0" || true
    floor=$last
fi

# The head comes from the held candidate, else workload 0: never from the 10 us run, whose
# flags (often inconsistent, see probe.rs ORIGIN) are not the budget's.
head=${held:-$first}
out=$(bench_next_report "$(dirname "$head")" budget "$summary_cell")
{
    bench_head "$head" budget "$summary_cell"
    echo "[figures]"
    printf 'loop_thread_allocs = %s\n' "$(bench_key "$head" loop_thread_allocs figures)"
    echo 'locks = "not measured"'
    echo 'method = "budget"'
    if [[ -n $budget ]]; then
        printf 'budget_ns = %s\noverhead_ns = %s\n' "$budget" "$((dt - budget))"
    fi
    if [[ -n $floor ]]; then
        gap=$(bench_key "$floor" gap_after_late_min_ns figures)
        [[ -n $gap ]] && printf 'gap_floor_min_ns = %s\n' "$gap"
    fi
    if [[ $hw == yes ]]; then
        sim_budget=$(bench_key "$sim" budget_ns figures)
        [[ -n $sim_budget ]] && printf 'engine_overhead_ns = %s\n' "$((dt - sim_budget))"
        if [[ -n $sim_budget && -n $budget ]]; then
            printf 'io_write_tick_ns = %s\n' "$((sim_budget - budget))"
        fi
        for key in read_ticks write_only_ticks read_bench_overruns; do
            v=$(bench_key "$head" "$key" figures)
            [[ -n $v ]] && printf '%s = %s\n' "$key" "$v"
        done
        read_p50=$(bench_key "$head" read_jitter_p50_ns figures)
        write_p50=$(bench_key "$head" write_only_jitter_p50_ns figures)
        if [[ -n $read_p50 && -n $write_p50 ]]; then
            printf 'io_read_ns = %s\n' "$((read_p50 - write_p50))"
        fi
    fi
} >"$out"
echo "$out"
