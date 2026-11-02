# Quickstart: validating the reference example

Run from the git root. Expected outcomes are the rows of
[contracts/run-behaviour.md](contracts/run-behaviour.md). The demo's numbers are in
[data-model.md](data-model.md).

## Prerequisites

- Rust 1.97 stable, the workspace builds.
- `main.rs` written per plan C2. That needs `Time::elapsed()` and `Deg` in `kmr_api`
  ([contracts/api-needs.md](contracts/api-needs.md)).
- V4 to V6 need critical path steps 1 to 9. V5 to V7 need XmBench wired and powered.

## V1. Clean reference (SC-003, SC-004, SC-007)

```sh
cargo build --workspace
cargo clippy --workspace --all-targets --message-format short 2>&1 | grep '^controllers/'
cargo fmt --all --check
grep -rnE 'todo!|unimplemented!|compile_error!|unwrap|expect\(|panic!' controllers/src
grep -rnE 'kmr_core|kmr_model|kmr_adapters|dxl' controllers/src controllers/Cargo.toml
command ls controllers controllers/src
```

Expected: build ok; the two `grep` searches and the clippy filter print nothing; `fmt` clean;
`controllers/` holds `Cargo.toml`, `README.md`, `src/main.rs` only, no `build.rs`. The manifest
names one kmr crate, `kmr_api`.

Review by hand (SC-005, SC-006): no lock, heap type, sleep, blocking call or I/O in `swing`;
every comment true against the SRS and the API.

## V2. Simulation run (SC-011, row 1)

```sh
cargo build -p controllers
time ./target/debug/controllers; echo "exit=$?"
```

Expected: exit 0 after about 12 s. Exactly one warning, "no bus plugin: simulating". The last
line is the shutdown with the reason "reference demo: time is up". Repeat 10 times.

Today's prototype prints two warnings here (research R11). V2 passes once kmr_api and kmr_core
fix that.

## V3. Cross-builds without hardware (SC-008)

```sh
cargo build -p controllers --target x86_64-unknown-linux-gnu
CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc \
  cargo build -p controllers --target aarch64-unknown-linux-gnu
CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER=arm-linux-gnueabihf-gcc \
  cargo build -p controllers --target armv7-unknown-linux-gnueabihf
cargo tree -p controllers | grep -c dxl_rs
```

Expected: three builds ok; `dxl_rs` count 0. Without the cross linkers, `cargo check --target`
proves the compile half only.

## V4. Bus plugin, no robot (SC-012, rows 2 and 3)

Add the walkthrough line `.add_plugins(Bus::by_id("<serial>"))`, robot unplugged.

```sh
cargo run -p controllers --release; echo "exit=$?"                     # row 2
cargo run -p controllers --features hardware --release; echo "exit=$?" # row 3
```

Expected: row 2, an error naming the adapter feature, `kmr_api/dynamixel`, which `hardware`
turns on; row 3, a bring-up error naming the port. Both non-zero, no panic. A release build
prints the `Display` text; a debug build prints the derived `Debug`, variant and fields
(kmr_api, 2026-10-29: `UserError` in kmr_api/src/error.rs; research R12).

## V5. Bench run (HUMAN, SC-009, SC-010, row 4)

XmBench wired, the adapter's serial in the bus line.

```sh
cargo run -p controllers --features hardware --release
```

Expected: every actuator swings ±5° around where it stood, period 4 s, then stops at 12 s of
`elapsed()`. Record History; every sample within 5° of the start. Afterwards `Torque_Enable` = 0
on ids 1 to 4 (critical path step 11). Repeat 10 times.

## V6. Signals (HUMAN, SC-010, row 5)

Run V5 and press Ctrl-C mid-swing. Then again with `kill -TERM` and `kill -HUP` on the process.
Expected each time: shutdown, hold, torque off, exit 0.

## V7. Onboarding trial (HUMAN, SC-001, SC-002)

A control engineer new to kmr follows `controllers/README.md` from a fresh checkout to the
first motion. Record a trial row ([data-model.md](data-model.md), Trial record). Then the same
task as the ROS2 baseline node (research R7). Expected: kmr wins on time and on file count;
at most 2 files written and 2 lines edited.
