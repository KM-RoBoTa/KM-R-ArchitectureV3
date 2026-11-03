#!/usr/bin/env bash
# presort.sh: the per-tick cost of the write pre-sort, simulated (FR-016 (SRS FR-5.3); research
# R15; D11 A fallback). Runs budget.sh, then budget.sh --write all, on the same build;
# presort_cost_ns is the overhead difference, an upper bound: it holds the write mask, the batch
# and its clear, not the sort alone. One report, presort-<nonrt|rt>-cost-<n>.toml, after the
# reports of both searches. Prints every report path, its own last. Run from the repository
# root.

set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=profile.env
source "$here/profile.env"

case ${1:-} in
"") ;;
--gain)
    echo "presort.sh: --gain waits on the presort-off build and the reference chain (bench T061, T063)" >&2
    exit 64
    ;;
*)
    echo "usage: presort.sh" >&2
    exit 64
    ;;
esac

cd "$(git rev-parse --show-toplevel)"
kernel=$(bench_kernel)
"$here/budget.sh" | tee "${TMPDIR:-/tmp}/presort-none.$$"
none=$(tail -n 1 "${TMPDIR:-/tmp}/presort-none.$$")
"$here/budget.sh" --write all | tee "${TMPDIR:-/tmp}/presort-all.$$"
all=$(tail -n 1 "${TMPDIR:-/tmp}/presort-all.$$")
rm -f "${TMPDIR:-/tmp}/presort-none.$$" "${TMPDIR:-/tmp}/presort-all.$$"

out=$(bench_next_report "$(dirname "$none")" presort "$kernel-cost")
over_none=$(bench_key "$none" overhead_ns figures)
over_all=$(bench_key "$all" overhead_ns figures)
{
    bench_head "$none" presort "$kernel-cost"
    if [[ $(bench_key "$none" status run) == complete && $(bench_key "$all" status run) == complete ]]; then
        echo "[figures]"
        echo 'loop_thread_allocs = "not measured"'
        echo 'locks = "not measured"'
        echo 'method = "budget, write all against write none"'
        echo 'presort_cost_bound = "upper: mask, batch and clear included"'
        [[ -n $over_none ]] && printf 'overhead_none_ns = %s\n' "$over_none"
        [[ -n $over_all ]] && printf 'overhead_all_ns = %s\n' "$over_all"
        if [[ -n $over_none && -n $over_all ]] && ((over_all >= over_none)); then
            printf 'presort_cost_ns = %s\n' "$((over_all - over_none))"
        fi
    fi
} >"$out"
echo "$out"
