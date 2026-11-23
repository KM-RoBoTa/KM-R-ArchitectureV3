#!/usr/bin/env bash
# Regenerates compile_commands.json at the workspace root for clangd: the C++ example
# and, when Pinocchio is installed, the GeckoLeg example. Each build runs under bear.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

# Touching build.rs forces cargo to rerun the build script, which is what emits the compiles.
touch kmr_ffi/examples/cpp/build.rs
bear --output "$tmp/cpp.json" -- cargo build -p kmr_ffi_example_cpp

inputs=("$tmp/cpp.json")
if pkg-config --exists pinocchio; then
    touch kmr_ffi/examples/pinocchio/build.rs
    CXXFLAGS="$(pkg-config --cflags pinocchio)" \
        bear --output "$tmp/pinocchio.json" -- \
        cargo build -p kmr_ffi_example_pinocchio --features pinocchio
    inputs+=("$tmp/pinocchio.json")
else
    echo "compile-db: pinocchio not found, GeckoLeg entries skipped" >&2
fi

# One merged file: each source once, so repeated runs do not pile up entries.
jq -s 'add | unique_by(.file)' "${inputs[@]}" > "$tmp/compile_commands.json"
mv "$tmp/compile_commands.json" compile_commands.json
echo "compile-db: wrote $(jq length compile_commands.json) entries to compile_commands.json"
