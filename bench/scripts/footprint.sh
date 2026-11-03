#!/usr/bin/env bash
# footprint.sh: size, sections, peak RSS and stack of the engine alone and of the bench binary
# (FR-011 (SRS NFR-8 e); research R11). Reads the release binaries built once (BENCH_BIN):
# `minimal` (the engine, no-op controllers) gives the unprefixed keys, `tick` the `tick_` ones
# plus its `bench_storage_bytes`, `baseline` the pages touched before main. Each binary runs
# FOOTPRINT_TICKS at 1 ms. Writes one report, footprint-<nonrt|rt>-<n>.toml, and prints its path.
# Run from the repository root.

set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=profile.env
source "$here/profile.env"

cd "$(git rev-parse --show-toplevel)"
minimal=$(bench_bin minimal)
tick=$(bench_bin tick)
baseline=$(bench_bin baseline)
cell=$(bench_kernel)
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

# `<prefix>file_bytes`, `stripped_bytes`, `text`, `data`, `bss` of binary $2, prefix $1.
sizes() {
    local text data bss
    printf '%sfile_bytes = %s\n' "$1" "$(stat -c %s "$2")"
    cp "$2" "$tmp/stripped"
    strip "$tmp/stripped"
    printf '%sstripped_bytes = %s\n' "$1" "$(stat -c %s "$tmp/stripped")"
    read -r text data bss _ < <(size "$2" | tail -n 1)
    printf '%stext = %s\n%sdata = %s\n%sbss = %s\n' "$1" "$text" "$1" "$data" "$1" "$bss"
}

# Runs binary $1 outside bench_run into $tmp/$2.toml, kmr's output appended to the report's .log.
side_run() {
    env -u RUST_LOG KMR_BENCH_OUT="$tmp/$2.toml" KMR_BENCH_MEASUREMENT=footprint \
        KMR_BENCH_CELL="$cell" KMR_BENCH_TICKS="$FOOTPRINT_TICKS" \
        taskset -c "$AFFINITY_CPU" "$1" >>"$log" 2>&1 || true
    if [[ ! -s $tmp/$2.toml ]]; then
        printf '[run]\nstatus = "failed"\nreason = "no report from %s"\n' "$2" >"$tmp/$2.toml"
    fi
}

rc=0
report=$(BENCH_LOAD_COMMAND=none bench_run "$minimal" footprint "$cell" \
    KMR_BENCH_TICKS="$FOOTPRINT_TICKS") || rc=$?
log=${report%.toml}.log
side_run "$tick" tick
side_run "$baseline" baseline

status=$(bench_key "$report" status run)
reason=""
for part in tick baseline; do
    s=$(bench_key "$tmp/$part.toml" status run)
    if [[ $status == complete && $s != complete ]]; then
        status=${s:-failed}
        reason="$part: $(bench_key "$tmp/$part.toml" reason run)"
    fi
done

cp "$report" "$tmp/minimal.toml"
{
    bench_head "$tmp/minimal.toml" footprint "$cell" |
        awk -v s="$status" -v r="$reason" '
            /^\[/ { block = $0 }
            block == "[run]" && r != "" && $1 == "status" && $2 == "=" { print "status = \"" s "\""; print "reason = \"" r "\""; next }
            { print }'
    if [[ $status == complete ]]; then
        echo "[figures]"
        echo 'loop_thread_allocs = "not measured"'
        echo 'locks = "not measured"'
        sizes "" "$minimal"
        for key in vm_hwm_kb stack_kb; do
            printf '%s = %s\n' "$key" "$(bench_key "$tmp/minimal.toml" "$key" figures)"
        done
        sizes tick_ "$tick"
        for key in vm_hwm_kb stack_kb; do
            printf 'tick_%s = %s\n' "$key" "$(bench_key "$tmp/tick.toml" "$key" figures)"
        done
        printf 'tick_bench_storage_bytes = %s\n' "$(bench_key "$tmp/tick.toml" bench_storage_bytes run)"
        printf 'vm_hwm_baseline_kb = %s\n' "$(bench_key "$tmp/baseline.toml" vm_hwm_baseline_kb figures)"
        stack_baseline=$(bench_key "$tmp/baseline.toml" stack_baseline_kb figures)
        printf 'stack_baseline_kb = %s\n' "$stack_baseline"
        if [[ $(bench_key "$tmp/minimal.toml" stack_kb figures) == "$stack_baseline" ]]; then
            echo 'stack_note = "below the first pages"'
        fi
        printf 'sim_warning_count = %s\n' "$(bench_key "$tmp/minimal.toml" sim_warning_count figures)"
    fi
} >"$report"
echo "$report"
[[ $status == failed ]] && exit 1
exit "$rc"
