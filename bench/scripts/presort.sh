#!/usr/bin/env bash
# presort.sh [--gain]: the write pre-sort against the presort-off build (FR-016 (SRS FR-5.3, §8);
# research R15; assumed, pending human: D11 B, the one place with the bench's presort-off
# feature).
#
# Cost (no argument, simulated): budget.sh --write all on the default build, then on the
# presort-off build; presort_cost_ns is the overhead difference, to BUDGET_STEP_NS. One report,
# presort-<nonrt|rt>-cost-<n>.toml, after the reports of both searches.
#
# --gain (the reference chain, bench/PROCEDURE.md "Reference chain"): budget.sh --hw on both
# builds, each hardware summary copied into presort-gain-<n>/default and
# presort-gain-<n>/presort-off, then `results compare` of the two, its output kept as
# presort-gain-<n>/compare.txt. No verdict.
#
# The presort-off build lands in PRESORT_OFF_BIN and its search reports under
# $REPORT_ROOT/presort-off/: two summaries of one cell in one directory would read as repeats.
# Prints every report path, its own last. Run from the repository root.

set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=profile.env
source "$here/profile.env"

gain=no
case ${1:-} in
"") ;;
--gain) gain=yes ;;
*)
    echo "usage: presort.sh [--gain]" >&2
    exit 64
    ;;
esac

cd "$(git rev-parse --show-toplevel)"
kernel=$(bench_kernel)
off_bin=$PRESORT_OFF_BIN
bins="tick results"
off_build="cargo build -p kmr_bench --release --bins --features presort-off --target-dir ${off_bin%/release}"
if [[ $gain == yes ]]; then
    bins="tick tick_hw results"
    off_build="$off_build; cargo build -p kmr_bench --release --bin tick_hw --features hardware,presort-off --target-dir ${off_bin%/release}"
fi
for bin in $bins; do
    bench_bin "$bin" >/dev/null
    [[ -x $off_bin/$bin ]] || {
        echo "presort: $off_bin/$bin missing; build once: $off_build" >&2
        exit 1
    }
done

paths=$(mktemp)
trap 'rm -f "$paths"' EXIT

# Runs budget.sh with arguments $2.. on the build named by $1 (default or presort-off); prints its
# report paths; sets `summary` to its last one.
search() {
    local build=$1
    shift
    if [[ $build == default ]]; then
        "$here/budget.sh" "$@" | tee "$paths"
    else
        BENCH_BIN=$off_bin REPORT_ROOT=$REPORT_ROOT/presort-off \
            BENCH_FEATURES=${BENCH_FEATURES:+$BENCH_FEATURES,}presort-off \
            "$here/budget.sh" "$@" | tee "$paths"
    fi
    summary=$(tail -n 1 "$paths")
}

if [[ $gain == yes ]]; then
    search default --hw
    on=$summary
    search presort-off --hw
    off=$summary
    n=1
    while [[ -e $(dirname "$on")/presort-gain-$n ]]; do n=$((n + 1)); done
    dir=$(dirname "$on")/presort-gain-$n
    mkdir -p "$dir/default" "$dir/presort-off"
    cp "$on" "$dir/default/"
    cp "$off" "$dir/presort-off/"
    rc=0
    "$BENCH_BIN/results" compare "$dir/default" "$dir/presort-off" | tee "$dir/compare.txt" ||
        rc=$?
    echo "$dir/compare.txt"
    exit "$rc"
fi

search default --write all
on=$summary
search presort-off --write all
off=$summary

out=$(bench_next_report "$(dirname "$on")" presort "$kernel-cost")
over_on=$(bench_key "$on" overhead_ns figures)
over_off=$(bench_key "$off" overhead_ns figures)
{
    bench_head "$on" presort "$kernel-cost"
    if [[ $(bench_key "$on" status run) == complete && $(bench_key "$off" status run) == complete ]]; then
        echo "[figures]"
        echo 'loop_thread_allocs = "not measured"'
        echo 'locks = "not measured"'
        echo 'method = "budget --write all, default build against presort-off build"'
        printf 'presort_cost_resolution_ns = %s\n' "$BUDGET_STEP_NS"
        [[ -n $over_on ]] && printf 'overhead_presort_ns = %s\n' "$over_on"
        [[ -n $over_off ]] && printf 'overhead_presort_off_ns = %s\n' "$over_off"
        if [[ -n $over_on && -n $over_off ]] && ((over_on >= over_off)); then
            printf 'presort_cost_ns = %s\n' "$((over_on - over_off))"
        fi
    fi
} >"$out"
echo "$out"
