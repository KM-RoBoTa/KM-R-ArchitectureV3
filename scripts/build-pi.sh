#!/usr/bin/env bash
# Cross-compile a controllers example for the robot Raspberry Pi 4 and optionally scp it there.
# Never runs anything on the Pi: the user starts the binary (it can move the motors).
# From cubemars_rs's scripts/build-pi.sh, building in this workspace.
#
#   scripts/build-pi.sh [example] [--deploy]
#
#   example    name under controllers/examples/ (default: geckoleg), or geckoleg_pinocchio:
#              the C++ Pinocchio example (kmr_ffi/examples/pinocchio), aarch64 only
#   --deploy   scp the binary to $PI_USER@$PI_HOST:$PI_DIR (geckoleg_pinocchio: the binary
#              and, next to it, geckoleg_pinocchio-libs/lib/, the Pinocchio libraries it loads)
#
# Environment overrides:
#   PI_TARGET    rust target   (default aarch64-unknown-linux-gnu; use armv7-unknown-linux-gnueabihf
#                               if the Pi runs a 32-bit OS: `uname -m` on the Pi says armv7l)
#   PI_HOST      required with --deploy (the Pi address)
#   PI_USER      default alarm
#   PI_DIR       default ~/
#   PI_FEATURES  features (default cubemars; geckoleg_pinocchio: pinocchio,cubemars)
#   PINOCCHIO_PI_PREFIX  geckoleg_pinocchio: an aarch64 Pinocchio prefix (include/, lib/,
#                lib/pkgconfig/) instead of the cache target/pi-pinocchio/, which the first
#                build fills with uv from PyPI (pin 3.7.0 wheels, manylinux aarch64)
set -euo pipefail

example="geckoleg"
deploy=0
for arg in "$@"; do
    case "$arg" in
        --deploy) deploy=1 ;;
        -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
        *) example="$arg" ;;
    esac
done

target="${PI_TARGET:-aarch64-unknown-linux-gnu}"
user="${PI_USER:-alarm}"
dir="${PI_DIR:-~/}"
if [ "$example" = geckoleg_pinocchio ]; then
    features="${PI_FEATURES:-pinocchio,cubemars}"
else
    features="${PI_FEATURES:-cubemars}"
fi
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
if [ "$example" = geckoleg_pinocchio ] && [ "$target" != aarch64-unknown-linux-gnu ]; then
    echo "geckoleg_pinocchio builds for aarch64-unknown-linux-gnu only (Pinocchio's wheels), not $target" >&2
    exit 1
fi

command -v "$linker" >/dev/null || { echo "missing $linker (Arch: pacman -S ${linker%-gcc}-gcc)" >&2; exit 1; }
rustup target list --installed | grep -qx "$target" || rustup target add "$target"

# Resolved before the cd below, so a relative prefix means relative to where the user stands.
if [ -n "${PINOCCHIO_PI_PREFIX:-}" ]; then
    PINOCCHIO_PI_PREFIX="$(cd "$PINOCCHIO_PI_PREFIX" && pwd)"
fi
cd "$(dirname "$0")/.."

if [ "$example" != geckoleg_pinocchio ]; then
    env "$linker_var=$linker" cargo build --release --target "$target" \
        -p controllers --example "$example" --features "$features"

    bin="target/$target/release/examples/$example"
    file "$bin"

    if [ "$deploy" = 1 ]; then
        scp "$bin" "$user@$host:$dir"
        echo "copied. On the Pi, you run: ${dir}${example}"
    fi
    exit 0
fi

# geckoleg_pinocchio: an aarch64 Pinocchio prefix, the cross build against it, its libraries.
readelf="aarch64-linux-gnu-readelf"
command -v "$readelf" >/dev/null || { echo "missing $readelf (Arch: pacman -S aarch64-linux-gnu-binutils)" >&2; exit 1; }

if [ -n "${PINOCCHIO_PI_PREFIX:-}" ]; then
    prefix="$PINOCCHIO_PI_PREFIX"
else
    cache="target/pi-pinocchio"
    prefix="$cache/site/cmeel.prefix"
    if [ ! -f "$prefix/lib/pkgconfig/pinocchio.pc" ]; then
        command -v uv >/dev/null || { echo "missing uv, which fetches Pinocchio's aarch64 wheels (Arch: pacman -S uv), or set PINOCCHIO_PI_PREFIX" >&2; exit 1; }
        echo "fetching Pinocchio 3.7.0 for aarch64 into $cache (PyPI, once)"
        rm -rf "$cache/site.tmp"
        # urdfdom and tinyxml2 pinned: pin 3.7.0 links liburdfdom_*.so.4.0, which needs libtinyxml2.so.10.
        uv pip install --no-config --no-python-downloads --target "$cache/site.tmp" \
            --python-platform aarch64-manylinux_2_28 --python-version 3.13 --only-binary :all: \
            pin==3.7.0 cmeel-urdfdom==4.0.1 cmeel-tinyxml2==10.0.0
        rm -rf "$cache/site"
        mv "$cache/site.tmp" "$cache/site"
    fi
fi
[ -f "$prefix/lib/pkgconfig/pinocchio.pc" ] || { echo "no lib/pkgconfig/pinocchio.pc under $prefix" >&2; exit 1; }
prefix="$(cd "$prefix" && pwd)"

# Eigen is header-only: the prefix's when it ships one, else the host's.
if [ -d "$prefix/include/eigen3" ]; then
    eigen="$prefix/include/eigen3"
elif [ -d /usr/include/eigen3 ]; then
    eigen="/usr/include/eigen3"
else
    echo "no Eigen: neither $prefix/include/eigen3 nor /usr/include/eigen3 (Arch: pacman -S eigen)" >&2
    exit 1
fi
# -Wno-psabi: GCC notes an aarch64 parameter-passing change of GCC 10.1 on every Eigen vector.
cxxflags="-I$prefix/include -I$eigen -Wno-psabi"
[ -d "$prefix/include/urdfdom" ] && cxxflags="$cxxflags -I$prefix/include/urdfdom"

libs="geckoleg_pinocchio-libs"
# The libraries in $libs/lib, not flat in $libs: libcoal's RUNPATH is $ORIGIN/../lib, and the
# binary's rpath cannot stand in for it (the loader skips DT_RPATH for a lib with a DT_RUNPATH).
# Single quotes on purpose: the loader expands $ORIGIN, not the shell.
# shellcheck disable=SC2016
rpath='$ORIGIN/'"$libs/lib"

env "$linker_var=$linker" \
    "CXXFLAGS_aarch64_unknown_linux_gnu=$cxxflags" \
    "PKG_CONFIG_PATH=$prefix/lib/pkgconfig" \
    "PINOCCHIO_RPATH=$rpath" \
    cargo build --release --target "$target" \
    -p kmr_ffi_example_pinocchio --bin kmr_ffi_example_pinocchio --features "$features"

release="target/$target/release"
bin="$release/geckoleg_pinocchio"
cp "$release/kmr_ffi_example_pinocchio" "$bin"
file "$bin"

# The NEEDED closure found in the prefix (lib, or lib64 for zlib); glibc, libstdc++ and libgcc
# come from the Pi.
rm -rf "${release:?}/$libs"
mkdir -p "$release/$libs/lib"
queue=("$bin")
while [ "${#queue[@]}" -gt 0 ]; do
    elf="${queue[0]}"
    queue=("${queue[@]:1}")
    for needed in $("$readelf" -d "$elf" | sed -n 's/.*(NEEDED).*\[\(.*\)\]/\1/p'); do
        [ -e "$release/$libs/lib/$needed" ] && continue
        found=""
        for d in "$prefix/lib" "$prefix/lib64"; do
            [ -e "$d/$needed" ] && { found="$d/$needed"; break; }
        done
        if [ -n "$found" ]; then
            cp -L "$found" "$release/$libs/lib/$needed"
            queue+=("$release/$libs/lib/$needed")
        else
            case "$needed" in
                libc.so.*|libm.so.*|libstdc++.so.*|libgcc_s.so.*|ld-linux-*|libpthread.so.*|libdl.so.*|librt.so.*) ;;
                *) echo "$needed, needed by $(basename "$elf"), is neither in $prefix nor a system library" >&2; exit 1 ;;
            esac
        fi
    done
done
du -ch "$release/$libs/lib"/* | tail -n 1

if [ "$deploy" = 1 ]; then
    scp -r "$bin" "$release/$libs" "$user@$host:$dir"
    echo "copied. On the Pi, you run: ${dir}geckoleg_pinocchio"
fi
