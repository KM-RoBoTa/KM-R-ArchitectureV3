#!/usr/bin/env bash
# Counts the SRS markers in kmr_core/src and fails on an `unwrap`, `expect`
# or `panic!` outside them (SRS NFR-2, § 5 marker rule). Test code is out of
# scope: the test files, a trailing `#[cfg(test)]` module and comment lines.
set -euo pipefail

root="${1:-kmr_core/src}"

scan() {
    find "$root" -name '*.rs' ! -name 'test.rs' ! -name 'test_util.rs' -print0 |
        sort -z |
        xargs -0 awk '
            FNR == 1 { pending = 0; skip = 0 }
            skip { next }
            /^[[:space:]]*\/\// { next }
            pending && /^[[:space:]]*mod [a-z_]+;/ { pending = 0; next }
            pending && /^[[:space:]]*mod [a-z_]+ \{/ { skip = 1; next }
            { pending = 0 }
            /^[[:space:]]*#\[cfg\(test\)\]/ { pending = 1; next }
            { print FILENAME ":" FNR ":" $0 }
        '
}

code="$(scan)"
markers="$(grep -cE 'unimplemented!\("SRS' <<<"$code" || true)"
echo "SRS markers in $root: $markers"
grep -E 'unimplemented!\("SRS' <<<"$code" || true

banned="$(grep -E '\.unwrap\(\)|\.expect\(|panic!\(' <<<"$code" |
    grep -vE 'unimplemented!\("SRS' || true)"
if [[ -n "$banned" ]]; then
    echo "unwrap, expect or panic! outside an SRS marker:"
    echo "$banned"
    exit 1
fi
