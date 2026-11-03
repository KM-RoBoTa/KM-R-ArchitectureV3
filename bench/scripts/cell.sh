#!/usr/bin/env bash
# cell.sh <idle|load>: one 1 kHz cell on the running kernel, cell <nonrt|rt>-<idle|load>
# (FR-008, FR-009 (SRS NFR-8 b, c); research R17, R18). Runs `tick` once through bench_run with
# TICKS and WARMUP from profile.env; `load` starts LOAD_CMD (default: profile.env's load_cmd for
# the whole run) LOAD_LEAD_S before the run and stops it after. Prints the report path. Run from
# the repository root.

set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=profile.env
source "$here/profile.env"

case ${1:-} in
idle) measurement=jitter ;;
load) measurement=load ;;
*)
    echo "usage: cell.sh <idle|load>" >&2
    exit 64
    ;;
esac
kind=$1
cd "$(git rev-parse --show-toplevel)"
tick=$(bench_bin tick)
cell=$(bench_kernel)-$kind

if [[ $kind == idle ]]; then
    BENCH_LOAD_COMMAND=none bench_run "$tick" "$measurement" "$cell" \
        KMR_BENCH_TICKS="$TICKS" KMR_BENCH_WARMUP="$WARMUP"
    exit
fi

# The run length in s, rounded up, plus the lead and as much again as a margin.
run_s=$(((TICKS * 1000000 + 999999999) / 1000000000))
: "${LOAD_CMD:=$(load_cmd $((run_s + 2 * LOAD_LEAD_S)))}"
bash -c "exec $LOAD_CMD" >/dev/null 2>&1 &
load_pid=$!
trap 'kill "$load_pid" 2>/dev/null || true; wait "$load_pid" 2>/dev/null || true' EXIT
sleep "$LOAD_LEAD_S"
rc=0
BENCH_LOAD_COMMAND=$LOAD_CMD bench_run "$tick" "$measurement" "$cell" \
    KMR_BENCH_TICKS="$TICKS" KMR_BENCH_WARMUP="$WARMUP" || rc=$?
exit "$rc"
