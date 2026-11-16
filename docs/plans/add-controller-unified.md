# Plan: one `add_controller` (D-28, D-31)

Crate: `kmr_api` only. No `kmr_core` change, no new dependency, no `unsafe`.
Branch: `feature/kmr-api-add-controller`, cut from `dev`.

## Decided pub surface (maintainer, 2026-11-16)

```rust
.add_controller(EachTick, f)                                  // plain Fn(&Time, &mut State<N>, &Sensors)
.add_controller(EachTick, with(ctx, f))                       // was add_controller_with
.add_controller(EachTick, f.or_when(when, fallback))          // was add_controller_or
.add_controller(EachTick, threaded(ctx, f))                   // was add_controller_as_thread
.add_controller(EachTick, (a, b, with(ctx, c)))               // several, inserted left to right
```

- `add_controller<S: Schedule, X: IntoController<N, S, C>>(self, schedule: S, x: X)`,
  body `self.grown(|inner| x.add_to(schedule, inner))` (or equivalent).
- `IntoController` is `pub`, `#[doc(hidden)]`, sealed, with
  `#[diagnostic::on_unimplemented]` starting "kmr: this is not a controller".
- `with(ctx, f) -> With<T, F>` and `threaded(ctx, f) -> InThread<T, F>`: free
  functions, opaque structs with private fields.
- `or_when(self, when, fallback) -> Guarded<F, G, P>`: extension trait on plain
  controllers only. Not on `With` (no guarded `InlineWith` in the core).
- No marker generic unless coherence demands one. No `before` / `after`.
- Closure signatures, node types and run order are unchanged from today.
- `add_controller_with`, `add_controller_or`, `add_controller_as_thread` are
  removed, not deprecated.

## Tasks

- [x] A01 `IntoController` for plain `Fn`, `With`, `Guarded`, `InThread`;
      `with`, `threaded`, `or_when`; `add_controller` takes `impl IntoController`.
      Same on every type that has the old methods (`Robot`, `Bound` if it has them).
- [x] A02 Remove the three old methods; move every call site, example, doctest
      and README (`controllers/`, `kmr_api/examples/`, `kmr_ffi` if any).
- [x] A03 Tuples: `macro_rules!`-generated impls, arity 1 to 8, chaining
      `Output`; a test that proves left-to-right order in one bucket and across
      a mixed tuple (`plain`, `with`, `or_when`).
- [x] A04 Prelude re-exports (`with`, `threaded`, the `or_when` trait); update
      the `complete_api_example` prose that names the old methods.

## Done when

`cargo test --workspace --no-fail-fast`, `cargo clippy --workspace --all-targets`,
`cargo fmt --all --check` and
`cargo build -p kmr_core --no-default-features --target thumbv7em-none-eabihf`
all pass, and `grep -rn "add_controller_\(with\|or\|as_thread\)"` finds nothing
outside `docs/`.
