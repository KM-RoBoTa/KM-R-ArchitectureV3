# Contract: the kmr_api surface the reference example calls

The example compiles against exactly this. `kmr_api`'s plan must keep every row, or this file
and `main.rs` change together. Status is the repo on 2026-10-28.

| Item | Use in `main.rs` | Status | Owner, spec id |
|---|---|---|---|
| `kmr_api::robot!(XmBench)` emitting `N`, `State<'a>`, `Robot` | robot line | exists; `main.rs` adopts it in plan C0, before kmr_api A4 | kmr_api, SRS FR-6.1 |
| `.set_limit_policy_all(Policy)`, `Policy::Clamp` | builder (plan C9) | **missing** | kmr_api C4 over core K5.2, K5.3 (HUMAN); assumed, pending human: D2 B (SRS FR-4.5) |
| `Robot::new()`, `.set_dt(Duration)`, `.add_controller_with(S, T, F)`, `.run()` | builder | exists | kmr_api, SRS section 7 |
| `.add_plugins(Bus::by_id(&'static str))` | walkthrough line | exists; drives nothing yet | kmr_api, critical path step 9 |
| `EachTick`, `Sensors`, `Q`, `UserError` | imports | exist | kmr_api |
| `shutdown!` | end of demo | exists (re-export) | kmr_core, kmr_api |
| `State::prev_q(0) -> Result<[Q; N], StateError>` | start sample | exists | kmr_api |
| `State::set_all_q([Q; N])` | command | exists | kmr_api |
| `Q + Q` | target | exists (`derive_more`) | kmr_core |
| `Time::elapsed() -> Duration` | offset, end check | **missing** | core #14 (HUMAN), then kmr_api FR-038 (SRS FR-3.3) |
| `Time::dt()`, `Time::tick()` | not called; named by FR-016 | **missing** | same |
| `Deg(pub f32)` and `From<Deg> for Q` (degrees to rad) | `AMPLITUDE`, offset | **missing** | kmr_api FR-033 (SRS FR-2.2) |
| `hardware` feature reachable as `kmr_api/dynamixel` | `Cargo.toml` | exists | kmr_api, kmr_adapters R9 |
| `UserError`'s `Debug` prints its `Display` message | readable error from `main` | **missing** | kmr_api FR-021 (SRS FR-6.4) |
| Exactly one simulation warning, the core's | row 1 of run-behaviour | **two today** | kmr_core FR-042, kmr_api FR-026 (SRS FR-6.3) |

Not called, on purpose (FR-012 (SRS FR-1.2, FR-2.7, FR-8.4)): `add_controller_as_thread`,
`initial_state`, `go_home`, `register_payload`, groups, `Bus::by_dev`. (Superseded 2026-11-11: SRS FR-2.7 closed home as a kmr_api preset, `kmr_api::presets::home::Home`.)

Under the const-generic N design, `State` becomes `kmr_api::State<'a, N>` behind the same
alias. `main.rs` does not change.
