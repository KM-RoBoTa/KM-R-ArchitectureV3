#!/usr/bin/env bash
# compile-time.sh [--host-label <l>]: clean and incremental compile times of kmr_core and
# controllers, dev and release, and the registration sweep (FR-013, FR-014 (SRS NFR-11, §8);
# research R14). Builds a copy of HEAD (git archive) in a temporary directory: the working
# tree is never edited. Writes one report, bench/reports/<commit12>/<label>/compile-<label>-<n>.toml,
# its cargo output next to it as .log, and prints the report path. The label defaults to the
# host id. The operator fields are needed, as for any run (meta.sh).

set -euo pipefail
# The generated code holds `&`: bash 5.2 would expand it in ${var//pattern/text}.
shopt -u patsub_replacement 2>/dev/null || true

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=profile.env
source "$here/profile.env"

label=""
while (($#)); do
    case $1 in
    --host-label)
        label=${2:?--host-label needs a value}
        shift 2
        ;;
    *)
        echo "usage: compile-time.sh [--host-label <l>]" >&2
        exit 64
        ;;
    esac
done

repo=$(git rev-parse --show-toplevel)
cd "$repo"
label=${label:-$("$here/meta.sh" host-id)}
label=${label//[^A-Za-z0-9._-]/-}
commit=$(git rev-parse --short=12 HEAD)
[[ $REPORT_ROOT == /* ]] || REPORT_ROOT=$repo/$REPORT_ROOT
dir=$REPORT_ROOT/$commit/$label
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

# Fails before any build when an operator field is missing.
BENCH_PROFILE="dev,release" BENCH_FEATURES="" "$here/meta.sh" start >"$work/meta.toml"

mkdir -p "$dir"
n=1
while [[ -e $dir/compile-$label-$n.toml ]]; do n=$((n + 1)); done
report=$dir/compile-$label-$n.toml
log=${report%.toml}.log
: >"$log"

ws=$work/ws
mkdir -p "$ws"
git archive HEAD | tar -x -C "$ws"
# Sibling path dependencies (../dxl_rs-agent): linked, read only, so the copy resolves them.
sed -n 's/.*path *= *"\.\.\/\([^"]*\)".*/\1/p' Cargo.toml | while read -r sib; do
    if [[ -e $repo/../$sib ]]; then ln -s "$(cd "$repo/../$sib" && pwd)" "$work/$sib"; fi
done
export CARGO_TARGET_DIR=$work/target
unset RUST_LOG CARGO_BUILD_RUSTFLAGS

figures=$work/figures
: >"$figures"

# Prints the ms the command takes, or fails with it; its output goes to the log.
timed() {
    local t0 t1
    t0=$(date +%s%N)
    "$@" >>"$log" 2>&1 || return 1
    t1=$(date +%s%N)
    echo $(((t1 - t0) / 1000000))
}

# Writes the failed report and exits 1.
failed() {
    {
        cat "$work/meta.toml"
        "$here/meta.sh" end
        printf '[run]\nmeasurement = "compile"\ncell = "%s"\nstatus = "failed"\nflags = ""\n' "$label"
        printf 'reason = "%s"\n' "${1//\"/\'}"
    } >"$report"
    echo "${report#"$repo"/}"
    exit 1
}

# Appends <key>_median_ms, _min_ms, _max_ms of the given ms values; the median of an even
# count is the lower middle value.
stat() {
    local key=$1
    shift
    local -a sorted
    mapfile -t sorted < <(printf '%s\n' "$@" | sort -n)
    {
        printf '%s_median_ms = %s\n' "$key" "${sorted[$(((${#sorted[@]} - 1) / 2))]}"
        printf '%s_min_ms = %s\n' "$key" "${sorted[0]}"
        printf '%s_max_ms = %s\n' "$key" "${sorted[-1]}"
    } >>"$figures"
}

# The crate root of a package of the copy.
root_of() {
    case $1 in
    kmr_core) echo "$ws/kmr_core/src/lib.rs" ;;
    controllers) echo "$ws/controllers/src/main.rs" ;;
    esac
}

# Copies the crate root with one more registration: the last add_controller line, again.
add_registration() {
    awk '
        /\.add_controller\(/ { last = $0 }
        /^[[:space:]]*\.run\(\)/ && last != "" { print last; added = 1 }
        { print }
        END { exit added ? 0 : 1 }' "$1" >"$2"
}

cd "$ws"
for crate in kmr_core controllers; do
    cp "$(root_of "$crate")" "$work/$crate.pristine"
done
nonce=0
for profile in dev release; do
    flag=()
    [[ $profile == release ]] && flag=(--release)
    for crate in kmr_core controllers; do
        rootfile=$(root_of "$crate")
        # Per crate: its own feature set, and the restored root of the crate before it.
        cargo build -p "$crate" "${flag[@]}" >>"$log" 2>&1 || failed "warm-up build of $crate, $profile"
        clean=() body=() reg=()
        for _ in $(seq "$REPEATS"); do
            cp "$work/$crate.pristine" "$rootfile"
            cargo clean -p "$crate" "${flag[@]}" >>"$log" 2>&1 || failed "cargo clean -p $crate"
            t=$(timed cargo build -p "$crate" "${flag[@]}") || failed "clean build of $crate, $profile"
            clean+=("$t")
            nonce=$((nonce + 1))
            printf '\nfn __bench_nonce() -> u32 {\n    %s\n}\n' "$nonce" >>"$rootfile"
            t=$(timed cargo build -p "$crate" "${flag[@]}") || failed "body-edit build of $crate, $profile"
            body+=("$t")
            if [[ $crate == controllers ]]; then
                add_registration "$rootfile" "$work/reg.rs" || failed "no add_controller line before .run() in controllers"
                cp "$work/reg.rs" "$rootfile"
                t=$(timed cargo build -p "$crate" "${flag[@]}") || failed "registration-edit build, $profile"
                reg+=("$t")
            fi
        done
        cp "$work/$crate.pristine" "$rootfile"
        stat "${crate}_${profile}_clean" "${clean[@]}"
        stat "${crate}_${profile}_body" "${body[@]}"
        if [[ $crate == controllers ]]; then stat "${crate}_${profile}_reg" "${reg[@]}"; fi
    done
done

# FR-014: generated crates, one phase, dev, CARGO_INCREMENTAL=0; each build follows an edit.
for count in $REG_COUNTS; do
    gen=$work/gen/reg$count
    mkdir -p "$gen/src"
    cp "$ws/Cargo.lock" "$gen/"
    printf '[package]\nname = "reg%s"\nversion = "0.0.0"\nedition = "2024"\npublish = false\n\n[dependencies]\nkmr_api = { path = "%s/kmr_api" }\n\n[workspace]\n' \
        "$count" "$ws" >"$gen/Cargo.toml"
    fns="" regs=""
    for i in $(seq 0 $((count - 1))); do
        fns+="fn c$i(_: &Time, _: &mut State, _: &Sensors) {}"$'\n'
        regs+="        .add_controller(EachTick, c$i)"$'\n'
    done
    template=$(<"$repo/bench/gen/registrations.rs.in")
    template=${template//@COUNT@/$count}
    template=${template//@FNS@/$fns}
    template=${template//@REGS@/$regs}
    printf '%s\n' "$template" >"$work/reg$count.rs"
    cp "$work/reg$count.rs" "$gen/src/main.rs"
    CARGO_INCREMENTAL=0 cargo build --manifest-path "$gen/Cargo.toml" >>"$log" 2>&1 || failed "generated crate, $count registrations"
    times=()
    for _ in $(seq "$REPEATS"); do
        nonce=$((nonce + 1))
        cp "$work/reg$count.rs" "$gen/src/main.rs"
        printf '\nfn __bench_nonce() -> u32 {\n    %s\n}\n' "$nonce" >>"$gen/src/main.rs"
        t=$(timed env CARGO_INCREMENTAL=0 cargo build --manifest-path "$gen/Cargo.toml") || failed "generated crate, $count registrations"
        times+=("$t")
    done
    stat "reg${count}_dev" "${times[@]}"
done

cd "$repo"
flags=""
[[ $(bench_key "$work/meta.toml" reference meta) == yes ]] || flags=not_reference
{
    cat "$work/meta.toml"
    "$here/meta.sh" end
    printf '[run]\nmeasurement = "compile"\ncell = "%s"\nstatus = "complete"\nflags = "%s"\n' "$label" "$flags"
    printf 'repeats = %s\nreg_counts = "%s"\n' "$REPEATS" "$REG_COUNTS"
    echo "[figures]"
    cat "$figures"
} >"$report"
echo "${report#"$repo"/}"
