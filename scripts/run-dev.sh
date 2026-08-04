#!/usr/bin/env bash
# Dev build: swaps Cargo.dev.toml (real kmr_core source path dep) into
# Cargo.toml so cargo/rust-analyzer resolve kmr_core, then LEAVES it active —
# default working state after this runs is dev. Run scripts/run-prod.sh
# before packaging/releasing to swap back to the rlib-only manifest.
#
# Usage: scripts/run-dev.sh [cargo subcommand + args]   (default: check)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
API_DIR="$(cd "$SCRIPT_DIR/../kmr_api" && pwd)"
CORE_DIR="$(cd "$SCRIPT_DIR/../kmr_v3" && pwd)"

# rebuild kmr_core rlib so kmr_api's extern path resolves
cargo build -p kmr_core --manifest-path "$CORE_DIR/Cargo.toml"

cd "$API_DIR"

if [[ ! -f Cargo.dev.toml ]]; then
    echo "error: $API_DIR/Cargo.dev.toml not found" >&2
    exit 1
fi

# keep separate lockfiles per manifest so switching back and forth doesn't
# thrash Cargo.lock
[[ -f Cargo.lock && ! -f Cargo.prod.lock ]] && cp Cargo.lock Cargo.prod.lock || true
cp Cargo.dev.toml Cargo.toml
[[ -f Cargo.dev.lock ]] && cp Cargo.dev.lock Cargo.lock || true

# dev builds resolve kmr_core via the path dep above; the prod rustflags
# below force an extra --extern to the closed rlib, which collides with it.
if [[ ! -f .cargo/config.dev.toml ]]; then
    echo "error: $API_DIR/.cargo/config.dev.toml not found" >&2
    exit 1
fi
[[ -f .cargo/config.toml && ! -f .cargo/config.prod.toml ]] && cp .cargo/config.toml .cargo/config.prod.toml || true
cp .cargo/config.dev.toml .cargo/config.toml

if [[ $# -eq 0 ]]; then
    set -- check
fi
cargo "$@"

[[ -f Cargo.lock ]] && cp Cargo.lock Cargo.dev.lock || true

echo "note: Cargo.toml now DEV (real kmr_core source). Run scripts/run-prod.sh before shipping." >&2
