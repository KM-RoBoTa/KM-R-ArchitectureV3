# Contract: the builder and `run()`

Public surface of `kmr_api::Robot`, as user code calls it through the `robot!` alias. Phase
letters match plan.md. Signatures show today's shape (N = `JOINTS`); after the core's const
generic N, `Robot<H, C>` becomes `Robot<N, H, C>`, then `Robot<N, H, C, E>` with the core's
type-state (C6), and nothing in user code changes.

## Construction

```rust
kmr_api::robot!(XmBench);           // emits N, State, Robot (contracts/robot-hardware.md)

Robot::new()                        // `Robot` is the alias: kmr_api::Robot<__Spec>
```

Bare `kmr_api::Robot::new()` without `robot!` does not compile (E0283, research R5).

## Settings (keep the type-state)

| Call | Phase | Notes | Spec ids |
| --- | --- | --- | --- |
| `.set_dt(Duration)` | exists | range checked by `run()` | FR-015 (SRS FR-3.1) |
| `.add_plugins(Bus::by_id(serial))` | A | preferred | FR-022 (SRS FR-6.3) |
| `.add_plugins(Bus::by_dev(path))` | A | less reliable across reboots | FR-022 |
| `Bus::..(..).baud(u32)` | A | replaces the table's baud for that bus | FR-023 (SRS FR-6.1) |
| `Bus::..(..).bus("label")` | A | optional for one bus (D6) | FR-024 (SRS FR-6.3, FR-6.5) |
| `.register_payload(Payload::medium(kg, "link"))`, `Payload::heavy(..)` | B | no light tier | FR-062, FR-063 (SRS FR-8.1, FR-8.4) |
| `.set_shutdown_hold(Option<Duration>)` | C | default `None` | FR-018 (SRS FR-4.1) |
| `.set_limit_policy(i, Policy)`, `.set_limit_policy_group(&G, Policy)`, `.set_limit_policy_all(Policy)` | C | `Clamp`, `Warn`, `Abort`; no default. Under D2 B (assumed, pending human) `run()` refuses a limited actuator without one; `_all` covers every actuator in one call | FR-019 (SRS FR-4.5) |
| `.set_envelope(i, Envelope)`, `.set_envelope_group(&G, Envelope)` | E | position, velocity, torque bounds | FR-051 (SRS FR-13.5) |
| `.add_plugins(Gateway::new(config))` | Z | feature `zenoh` | FR-043 (SRS FR-7.1) |

`add_plugins` takes any sealed `Plugin` (`Bus`, `Gateway`) from phase Z on; until then it takes
`Bus`.

## Dynamixel declarations (2026-11-05)

`.add_actuator(kmr_api::dynamixel::Rx28::ids(1..=10).on(bus))`: `Ids` knobs before `on`, each
lowered into the adapter's `Declared` at `run()`.

| Call on `dynamixel::Ids` | Notes | Spec ids |
| --- | --- | --- |
| `.mode(Mode)`, `.read_only()`, `.zero(Zero)` | existing | SRS FR-6.1, FR-5.8 |
| `.gains(p: u16, i: u16, d: u16)` | raw register values, written in `configure` torque off and read back; refused at `check` on AX and RX (no register) and above the model's range (254 on the MX-64, 16383 on Protocol 2) | SRS FR-5.2, FR-6.1 |
| `.led(on: bool)` | written and read back in `configure`; refused at `check` on the XW series (no LED) | SRS FR-5.2, FR-6.1 |

## Registrations (grow the type-state)

| Call | Controller | Spec ids |
| --- | --- | --- |
| `.add_controller(phase, f)` | `Fn(&Time, &mut State, &Sensors)` | FR-010 (SRS FR-1.2) |
| `.add_controller_with(phase, ctx, f)` | `Fn(&mut T, &Time, &mut State, &Sensors)` | FR-010, FR-011 |
| `.add_controller_as_thread(phase, ctx, f)` | `Fn(&mut T, &Time)`; stored, not spawned (SRS FR-1.2 open) | FR-010 |
| `.add_controller_or(phase, f, when, fallback)` | `f` and `fallback` inline; `when: Fn(&Faults) -> bool` | FR-049 (SRS FR-13.5) |
| `.add_remote_source(phase, Remote::allow(id).timeout(d), fallback)` | `fallback` inline, runs while the source is timed out | FR-058, FR-059 (SRS FR-13.3) |

Phases: `PreInit`, `Init`, `PostInit`, `First`, `EachTick`, `Last`, re-exported, no other
(FR-012 (SRS FR-1.4)). No hold-policy argument anywhere (FR-050).

A builder with no registration: `run()` does not compile once kmr_core #15 lands (FR-016
(SRS NFR-4)); until then it is `UserError::NoControllers`.

## Ending the chain

```rust
fn run(self) -> Result<(), UserError>                                  // phase A
fn bind(self) -> Result<Bound<H, C>, UserError>                        // phase C, no I/O
impl Bound<H, C> {
    fn adapter(&mut self) -> &mut H::Adapter;                          // before the run
    fn run(self) -> Result<H::Adapter, UserError>;                     // hands it back
}
```

`run()` returns `Ok(())` after a shutdown and after an emergency stop (spec Assumptions, SRS
section 8 open).

## Errors

| Variant | When | Before I/O | Message names |
| --- | --- | --- | --- |
| `DtOutOfRange { dt }` | `dt` zero or outside [10 µs, 1 s] | yes | the value and the range |
| `DuplicateBus { port }` | two plugins, same port string | yes | the port |
| `UnknownLink { link }` | payload mount not a model link | yes | the link |
| `NoHardwareDeclared` | plugins, `robot!(model = ..)` without `hardware` | yes | the fix (`hardware = ..`) |
| `FeatureOff { feature }` | plugins, adapter not compiled in | yes | `dynamixel` |
| `BringUp { cause }` | adapter binding (label, baud) | yes | the adapter's text |
| `BringUp { cause }` | port resolution, ping, health, configure, `read_all`, `torque_on` | no (torque already cut, ports closed) | serial or path, bus label, actuator |
| `NoPolicy { actuator }` | a limited actuator without a policy (D2 B, after core K5.3) | yes | the actuator and the fix (`set_limit_policy_all`) |
| `NoControllers` | empty builder, until core #15 | yes | the fix |

`Debug` prints the `Display` text. Release builds show no SDK type or internal cause
(FR-021 (SRS FR-6.4, NFR-2)).

## Behaviour of `run()`

| Plugins | `dynamixel` feature of kmr_api | Result |
| --- | --- | --- |
| none | any | simulates; the core prints "no bus plugin: simulating" once; kmr_api prints nothing (FR-026) |
| some | off | `FeatureOff` |
| some | on | binds, brings up, drives; ends with torque off on every exit after `open` (kmr_core) |

## Not offered

- No `build()`; no per-machine config file; no tick-level remote driving (FR-047); no light
  payload tier; no hold policy; no builder knob for read cadence (FR-042: the table owns it).
