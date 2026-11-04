# kmr_core T019: build time of `controllers` before and after M2

FR-061 (SRS NFR-11). A clean `cargo build --timings -p controllers` (dev
profile, default features, so no Dynamixel SDK), each in its own fresh target
directory, on the same machine (32 threads, rustc 1.97.1), 2026-11-03.

| Build | Commit | Total | Units | kmr_core | kmr_adapters | kmr_api | controllers |
|---|---|---|---|---|---|---|---|
| Before T021 | d2479bb | **5.5 s** | 101 | 0.35 s | 0.37 s | 0.20 s | 0.20 s |
| After T030 | abef51d | **5.7 s** | 101 | 0.29 s | 0.38 s | 0.18 s | 0.23 s |

Reports, kept for the PR (not tracked: `target/` is ignored):

- `target/t019-before/cargo-timings/cargo-timing.html`
- `target/t019-after/cargo-timings/cargo-timing.html`

Reading: +0.2 s on the total, one run each, inside what a second run moves
on this machine. As design-const-generic-n §4 expected, the core's own crate
got cheaper (its hardware contract is generic now, so its code is generated
in the user crate) and `controllers` a little dearer. The crates' own units
are a few tenths of a second out of 5.5 s; dependencies dominate a clean
build. Not measured here: the `hardware` feature (the Dynamixel adapter is
generic over N too, its share moves the same way).
