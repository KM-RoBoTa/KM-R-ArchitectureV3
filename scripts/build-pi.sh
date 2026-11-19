#!/usr/bin/env bash
# Cross-compile a controllers example for the robot Raspberry Pi 4 and optionally scp it there.
# Never runs anything on the Pi: the user starts the binary (it can move the motors).
# From cubemars_rs's scripts/build-pi.sh, building in this workspace.
#
#   scripts/build-pi.sh [example] [--deploy]
#
#   example    name under controllers/examples/ (default: geckoleg)
#   --deploy   scp the binary to $PI_USER@$PI_HOST:$PI_DIR
#
# Environment overrides:
#   PI_TARGET    rust target   (default aarch64-unknown-linux-gnu; use armv7-unknown-linux-gnueabihf
#                               if the Pi runs a 32-bit OS: `uname -m` on the Pi says armv7l)
#   PI_HOST      required with --deploy (the Pi address)
#   PI_USER      default alarm
#   PI_DIR       default ~/
#   PI_FEATURES  controllers features (default cubemars)
set -euo pipefail

example="geckoleg"
deploy=0
for arg in "$@"; do
    case "$arg" in
        --deploy) deploy=1 ;;
        -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
        *) example="$arg" ;;
    esac
done

target="${PI_TARGET:-aarch64-unknown-linux-gnu}"
user="${PI_USER:-alarm}"
dir="${PI_DIR:-~/}"
features="${PI_FEATURES:-cubemars}"
if [ "$deploy" = 1 ]; then
    host="${PI_HOST:?set PI_HOST to the Pi address}"
fi

case "$target" in
    aarch64-unknown-linux-gnu)
        linker="aarch64-linux-gnu-gcc"
        linker_var="CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER" ;;
    armv7-unknown-linux-gnueabihf)
        linker="arm-linux-gnueabihf-gcc"
        linker_var="CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER" ;;
    *) echo "unsupported PI_TARGET: $target" >&2; exit 1 ;;
esac

command -v "$linker" >/dev/null || { echo "missing $linker (Arch: pacman -S ${linker%-gcc}-gcc)" >&2; exit 1; }
rustup target list --installed | grep -qx "$target" || rustup target add "$target"

cd "$(dirname "$0")/.."
env "$linker_var=$linker" cargo build --release --target "$target" \
    -p controllers --example "$example" --features "$features"

bin="target/$target/release/examples/$example"
file "$bin"

if [ "$deploy" = 1 ]; then
    scp "$bin" "$user@$host:$dir"
    echo "copied. On the Pi, you run: ${dir}${example}"
fi
