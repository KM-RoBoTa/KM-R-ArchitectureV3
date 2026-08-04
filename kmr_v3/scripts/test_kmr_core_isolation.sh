#!/usr/bin/env bash
# Proves kmr_core cannot be imported by SDK users.
# Run from workspace root: ./scripts/test_kmr_core_isolation.sh

set -euo pipefail

WORKSPACE=$(cd "$(dirname "$0")/.." && pwd)
RLIB="$WORKSPACE/target/debug/libkmr_api.rlib"
DEPS="$WORKSPACE/target/debug/deps"
PASS=0
FAIL=0

red()   { printf '\033[0;31m%s\033[0m\n' "$*"; }
green() { printf '\033[0;32m%s\033[0m\n' "$*"; }

assert_compile_fail() {
    local name="$1"
    local code="$2"
    if rustc --edition 2024 \
        --extern "kmr_api=$RLIB" \
        -L "$DEPS" - <<< "$code" 2>/dev/null; then
        red "FAIL [$name]: compiled — should have been rejected"
        FAIL=$((FAIL + 1))
    else
        green "PASS [$name]"
        PASS=$((PASS + 1))
    fi
}

echo "Building workspace..."
cargo build -p kmr_api --manifest-path "$WORKSPACE/Cargo.toml" -q

assert_compile_fail "use kmr_core::" \
    'use kmr_core::RobotState; fn main() {}'

assert_compile_fail "extern crate kmr_core" \
    'extern crate kmr_core; fn main() {}'

assert_compile_fail "newtype inner field robot.0" \
    'use kmr_api::{Controller,RobotState};
     struct H;
     impl Controller for H {
         fn main_controller(&self, robot: RobotState) { let _ = robot.0; }
     }
     fn main() {}'

assert_compile_fail "kmr_api::kmr_core path" \
    'fn main() { let _ = kmr_api::kmr_core::RobotState::default(); }'

echo ""
echo "Results: $PASS passed, $FAIL failed"
[[ $FAIL -eq 0 ]]
