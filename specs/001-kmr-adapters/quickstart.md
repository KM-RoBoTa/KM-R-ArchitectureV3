# Quickstart: validating kmr_adapters (2026-10-28)

One section per plan step. Each names what it proves. Details live in the contracts:
[hal-conformance](contracts/hal-conformance.md), [hardware-table](contracts/hardware-table.md),
[port-binding](contracts/port-binding.md), [backend](contracts/backend.md).

## Prerequisites

- Rust 1.97 stable; repo root as working directory.
- No hardware and no C SDK for sections 0 to 3.
- Section 4 needs the HUMAN core steps 3 to 8 (design-first-motor) and `impl Hal for &mut A`.
- Section 5 needs `dxl_rs` with `force_torque_off`, pushed (HUMAN), libclang, a C compiler, the
  NFR-1 bench (Raspberry Pi 4, XmBench chain, U2D2), and the udev rule
  `ACTION=="add", SUBSYSTEM=="usb-serial", ATTR{latency_timer}="1"`.

## 0. Baseline (every step)

```sh
cargo fmt --all --check
cargo clippy -p kmr_adapters --all-targets
cargo test -p kmr_adapters --offline
cargo test --workspace --no-fail-fast
```

Expected: all pass. Baseline on 2026-10-28: 86 unit tests and 3 doctests in `kmr_adapters`.

## 1. Torque off everywhere (step A, critical path step 2)

```sh
cargo test -p kmr_adapters torque
cargo test -p kmr_adapters read_only
```

Expected:

- before A3, the new tests of [backend.md](contracts/backend.md) § Torque tests fail: the
  read-only actuator and the ones left on by an earlier run stay powered;
- after A3, they pass: every exit after `open` leaves no id powered; `ClosePorts` is last;
- the fake refuses a plain torque off it never powered, and Protocol 1 health
  (`cargo test -p kmr_adapters scripted`);
- no test aborts with a double panic.

## 2. Preset hardware half and count check (step B; B3, B4 land after step C)

```sh
cargo test -p kmr_adapters --doc
cargo test -p kmr_adapters table
```

Expected:

- `XmBench::HARDWARE` builds; `Preset::N` is 4;
- compile-fail doctests: wrong count (E0308 naming both sizes), repeated id, id 253, unknown
  label in `.baud`, a baud a model rejects, `Velocity` mode (names SRS section 8), `read_every`
  0. Each E0080 message names the actuator index or the label;
- an override chain in a `const` changes only the values it names.

## 3. Plugin port resolution (step C)

```sh
cargo test -p kmr_adapters binding
cargo test -p kmr_adapters open
```

Expected:

- `bind` errors with zero backend calls: unknown label, unbound bus, extra plugin, missing label
  on a two-bus table, two plugins on one label, a plugin baud a model rejects (names the
  actuator index);
- one-bus table with no label binds;
- `ById` resolves the single matching entry of a scratch `by-id` directory; none or two is
  `PortNotFound`;
- `ByDev` on a pty opens; a regular file is `PortAccess`; two buses on one device are
  `SharedPort`, before any flush.

## 4. Through the engine (step D, after the HUMAN core steps)

```sh
cargo test -p kmr_adapters engine
```

Expected, on a pty with the strict fake, through `run_on(&mut adapter)`:

- shutdown: three ticks of goal writes, one hold, `ForceTorqueOff` for ids 1 to 4, `ClosePorts`
  last;
- an estop skips that tick's write and the hold;
- a missing actuator: `run_on` returns `Missing` naming index, bus and id, after torque off and
  close;
- a panicking controller: torque off and close still run, then the panic goes on.

Then, for `kmr_api` (its plan A5, A7; hand-offs T042, T043): `run()` with no plugin opens no
port and warns once (core's warning); with plugins and without the `dynamixel` feature, `run()`
returns an error naming the feature.

## 5. On the bench (HUMAN, critical path step 11)

```sh
cargo run -p controllers --features hardware --release   # hardware = ["kmr_api/dynamixel"]
```

The shipped `controllers/src/main.rs` has `robot!(XmBench)`, a bounded swing from `prev(0)` and
`shutdown!` after a fixed time, and no bus line: as shipped it simulates (assumed, pending
human: D15 A). For this run the bench adds `.add_plugins(Bus::by_id("<U2D2 serial>"))` locally
and does not commit it.

Expected:

- the four actuators follow the swing (SC-001);
- after the shutdown, and again after a run ended by `emergency_stop!`, Dynamixel Wizard reads
  `Torque Enable` = 0 on ids 1 to 4 (SC-005); with one actuator set read-only, it too reads 0;
- ten runs in a row reopen the port cleanly (SC-005);
- write-tick I/O under 1 ms; read ticks logged apart; the result sets `DEFAULT_READ_EVERY`
  (SC-003, D9);
- removing the udev rule fails bring-up with the latency-timer hint, nothing powered (FR-004).
