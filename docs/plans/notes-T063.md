# T063: specs/002-kmr-core/quickstart.md, every step without hardware

2026-11-03, agent-goal after T062, x86_64 Linux, stable Rust. 0 failed everywhere.

| section | command or check | result |
|---|---|---|
| Always | `cargo fmt --all --check` | clean |
| Always | `cargo clippy --workspace --all-targets` | 0 warnings |
| Always | `cargo test --workspace --no-fail-fast` | 813 passed, 0 failed, 9 ignored |
| M1 | `cargo test -p kmr_core exit_paths` | 35 passed |
| M1 | `cargo test -p kmr_core state::test` | 71 passed |
| M1 | `cargo test -p kmr_core --test termination` | 1 passed |
| M1 | `cargo test -p kmr_core clock` | 16 passed |
| M1 | `an_estop_in_post_init_never_powers` (K1.9) | 1 passed |
| M1 | `a_no_hardware_run_logs_simulating_exactly_once` (K1.6) | 1 passed |
| M1 | `cargo test -p kmr_adapters dynamixel::tests::engine` | 4 passed |
| M1 | Bench | not run: hardware (core T018, docs/deferred.md) |
| M2 | `cargo test -p kmr_api --test robot_macro` (the filter form matches 3 unit tests) | 23 passed |
| M2 | `grep -rn JOINTS kmr_core/src` | 0 hits |
| M2 | 65 actuators: `kmr_api/tests/ui/n_65.stderr` "more than 64 actuators", in `cargo test -p kmr_api --test ui` | 1 passed (the whole ui suite) |
| M3 | `a_dt_out_of_range_is_refused_before_any_io`, `the_bounds_of_the_dt_range_are_accepted` | 2 passed |
| M3 | empty robot: compile-fail doctests, `cargo test -p kmr_core --doc` | 10 passed, 3 ignored (`ignore` examples) |
| M3 | `a_run_without_hardware_has_no_sample_and_no_age` | 1 passed |
| M3 | `an_actuator_read_every_third_tick_ages_between_reads` | 1 passed |
| M3 | `grep -rn 'todo!\|unimplemented!' kmr_core/src` | 0 hits |
| M3 | `cargo test -p kmr_core --features test-util --test no_alloc` (10 000 ticks, 0 allocations, 0 log lines) | 1 passed |
| M3 | `value_types_are_one_f32_and_a_slot_starts_a_cache_line` | in state::test above |
| M4 | faults per slot: `per_slot_flags` and the `fault` tests | 5 and 4 passed |
| M4 | `a_flag_raised_from_another_thread_during_tick_0_reads_in_tick_1` | 1 passed |
| M4 | `a_hold_lasts_its_duration_in_tick_steps`, `an_emergency_stop_during_the_hold_cuts_torque_at_once` | 2 passed |
| M5 | `stages_run_after_read_before_first_and_after_write` | 1 passed |
| M5 | `a_limit_without_a_policy_is_refused_before_any_io`, `a_limited_actuator_without_a_policy_is_refused` | 2 passed |
| M5 | `clamp`, `abort`, `continuous`, `envelope` filters | 3, 2, 1, 1 passed |
| M6 | `cargo build -p kmr_core --no-default-features --target thumbv7em-none-eabihf` | builds |
| M6 | same for `thumbv6m-none-eabi` | fails as expected: no `compare_exchange_weak`, `fetch_or`, `swap` on its atomics (research R13) |
| M6 | `cargo build -p kmr_core --features presort-off`; `presort_off_keeps_declaration_order` | builds; 1 passed |

The quickstart's "191 passed today" dates from the spec; the count has grown with every
milestone.
