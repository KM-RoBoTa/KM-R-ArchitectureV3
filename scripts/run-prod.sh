#!/usr/bin/env bash
# Prod/customer build: forces Cargo.prod.toml (no kmr_core dependency) into
# Cargo.toml, builds against the closed rlib via .cargo/config.toml's
# rustflags (--extern kmr_core=...rlib), and LEAVES prod active — this is
# the safe state to commit/package from. Run this before any release.
#
# Usage: scripts/run-prod.sh [cargo subcommand + args]   (default: build --release)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
API_DIR="$(cd "$SCRIPT_DIR/../kmr_api" && pwd)"
cd "$API_DIR"

if [[ ! -f Cargo.prod.toml ]]; then
    echo "error: $API_DIR/Cargo.prod.toml not found" >&2
    exit 1
fi

[[ -f Cargo.lock && ! -f Cargo.dev.lock ]] && cp Cargo.lock Cargo.dev.lock || true
cp Cargo.prod.toml Cargo.toml
[[ -f Cargo.prod.lock ]] && cp Cargo.prod.lock Cargo.lock || true

if [[ ! -f .cargo/config.prod.toml ]]; then
    echo "error: $API_DIR/.cargo/config.prod.toml not found" >&2
    exit 1
fi
cp .cargo/config.prod.toml .cargo/config.toml

rlib="$(grep -A2 '"--extern"' .cargo/config.toml | grep -o '/[^"]*\.rlib' | head -1)"
if [[ -n "$rlib" && ! -f "$rlib" ]]; then
    echo "error: rlib referenced by .cargo/config.toml not found: $rlib" >&2
    exit 1
fi

if grep -Eq '^\s*kmr_core\s*=\s*\{' Cargo.toml; then
    echo "error: Cargo.toml still has an active kmr_core path dependency after swap — refusing to build." >&2
    exit 1
fi

if [[ $# -eq 0 ]]; then
    set -- run --example complete_api_example
fi
cargo "$@"

[[ -f Cargo.lock ]] && cp Cargo.lock Cargo.prod.lock || true

echo "note: Cargo.toml now PROD (closed rlib only). Safe to commit/package from this state." >&2
