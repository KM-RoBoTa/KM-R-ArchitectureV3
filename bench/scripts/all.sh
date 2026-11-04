#!/usr/bin/env bash
# all.sh: the reference set (a) to (e) on the running kernel, then the reboot prompt (SC-001
# (SRS NFR-8); research R18). setup-check.sh first, stopping on its failure; then footprint,
# cell idle, cell load, ceiling and budget; then `results summary` of the set. It never reboots
# and never runs compile time. About 40 min per kernel with the profile.env lengths. Run from
# the repository root, as root: sudo -E bench/scripts/all.sh

set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=profile.env
source "$here/profile.env"

cd "$(git rev-parse --show-toplevel)"
results=$(bench_bin results)
for bin in tick minimal baseline; do bench_bin "$bin" >/dev/null; done

if ! "$here/setup-check.sh"; then
    echo "all.sh: setup-check failed; fix the field above, then run again" >&2
    exit 1
fi

kernel=$(bench_kernel)
started=$(date +%s)
failed=0
for step in footprint.sh "cell.sh idle" "cell.sh load" ceiling.sh budget.sh; do
    echo "== $step ($kernel) =="
    # shellcheck disable=SC2086
    "$here"/$step || failed=$((failed + 1))
done

[[ $REPORT_ROOT == /* ]] || REPORT_ROOT=$PWD/$REPORT_ROOT
set_dir=$REPORT_ROOT/$(git rev-parse --short=12 HEAD)/$("$here/meta.sh" host-id)
echo "== results summary $set_dir =="
"$results" summary "$set_dir" || failed=$((failed + 1))

echo "all.sh: $kernel half done in $((($(date +%s) - started) / 60)) min, $failed step(s) failed"
if [[ $kernel == rt ]]; then other="the non-RT kernel"; else other="the PREEMPT_RT kernel"; fi
echo "Next: reboot into $other and run sudo -E bench/scripts/all.sh again (the same binaries)."
echo "When both halves are done, commit bench/reports/ (a human does)."
((failed == 0))
