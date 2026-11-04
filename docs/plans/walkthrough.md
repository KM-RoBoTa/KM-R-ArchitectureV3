# Walkthrough: hardware binding (D-01)

Scoped step-10 walkthrough for `docs/deferred.md` D-01, not the whole
project. Code is read in the dev working tree
`/home/ovsiankina/Projects/km-robota/projects/Architecture/kmr_v3`
(where the URDF snake test ran), not in this worktree. Rows in call
order from the user's `main.rs`. Outputs: `/issue` lines, ticks, and at
the end one D-01 decision line in `docs/deferred.md`.

- [ ] `controllers/src/main.rs` + `controllers/models/rx28_snake.urdf`: the failing case, `NoHardwareDeclared`
- [ ] `robot!`, both arms: `kmr_api/src/robot_macro.rs`
- [ ] `actuator_count!`: `kmr_model/src/lib.rs`
- [ ] `Spec`, `__bind_none`, `__bind_dynamixel`, `__DxlAdapter`: `kmr_api/src/spec.rs`
- [ ] what a user crate can name: `kmr_api/src/lib.rs` re-exports
- [ ] `Bus` plugin → `Binding`: `kmr_api/src/bus.rs`, `kmr_adapters/src/binding.rs`
- [ ] `Robot::add_plugins`, `Robot::run` (where `bind` is called): `kmr_api/src/robot.rs`
- [ ] `Table`, `Actuator`, `Table::check`: `kmr_adapters/src/dynamixel/config.rs`
- [ ] `Dynamixel::bind`: `kmr_adapters/src/dynamixel.rs`
- [ ] `Preset`, `Rx28Chain`: `kmr_adapters/src/presets.rs`, `kmr_adapters/src/presets/rx28_chain.rs`
- [ ] `Hal`, `NoHardware`, `JOINTS`: `kmr_core/src/hal.rs`, `kmr_core/src/state.rs` (why N is fixed before the user crate builds)
