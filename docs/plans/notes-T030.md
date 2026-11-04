# kmr_api T030: review of every `unsafe` in kmr_api

Review (research R22 #3, constitution IV), done 2026-11-03 on this branch
after T029, T048 and the Zenoh gateway (T042 to T055). For the PR
description.

**Verdict: kmr_api's library code has no `unsafe`.** The slot is the
`triple_buffer` crate (T029), so the hand-written version of research R15
and its `unsafe` blocks never landed. The only `unsafe` are two counting
allocators, each in a test binary, never in a build a user links.

## The `unsafe` blocks

| File:line | What | Safe because |
| --- | --- | --- |
| kmr_api/src/slot/tests.rs:22 | `unsafe impl GlobalAlloc for Counting` (T029, lib unit-test binary only: the module is `#[cfg(test)]`) | Every method forwards its arguments unchanged to `std::alloc::System`, which upholds the `GlobalAlloc` contract. The count reads a `const`-initialized thread-local `Cell<bool>` (no destructor, no lazy init, so it never allocates or re-enters the allocator) and bumps an `AtomicU64`. Neither the layout nor the pointer is touched. `try_with` keeps a thread being torn down from panicking inside the allocator. |
| kmr_api/src/slot/tests.rs:26, 32, 38, 43 | `System.alloc`, `alloc_zeroed`, `realloc`, `dealloc` | The caller's contract is passed on as is; `realloc` and `dealloc` get pointers this allocator handed out, which are `System`'s. |
| kmr_api/tests/zenoh.rs:83 | `unsafe impl GlobalAlloc for Counting` (T048, the `zenoh` integration-test binary only) | Same body as above, same argument. The window is opened and closed by `RealTime::window` on the tick thread only; a positive control (`the_window_counts_an_allocation_of_the_tick_thread`) shows the count is live. |
| kmr_api/tests/zenoh.rs:87, 93, 99, 104 | `System.alloc`, `alloc_zeroed`, `realloc`, `dealloc` | As above. |

Each block has its `// SAFETY:` comment. Both allocators are the RULES
exception (a counting `#[global_allocator]` wrapping `System`); bench's
`kmr_bench::alloc::Counting` is the same shape and was not re-reviewed here.

## What kmr_api depends on that has `unsafe`

Not kmr_api's own, recorded so the reviewer knows where the trust goes:

- `triple_buffer` 9.0.0 (T029, the `Latest<T>` slot): three `UnsafeCell`
  buffers indexed by an atomic back-buffer word, and `unsafe impl Sync` for
  its shared state. Its protocol is the one research R15 proved by hand
  (writer: one `AcqRel` swap per publish; reader: one `Relaxed` load, one
  `AcqRel` swap when new). kmr_api uses only `publish` (through
  `input_buffer_mut` then `publish`), `update` and `output_buffer`; it never
  holds a buffer reference across a publish or an update.
  `kmr_api/src/slot/tests.rs` stresses it: 200 000 puts of a 528-byte
  value against 2 000 000 takes, never torn, strictly increasing.
- `zenoh` 1.10.2 (feature `zenoh`, gateway thread only): its own runtime and
  transports. Nothing of it runs on the tick thread.

## Locks on the tick side (T048, SC-003 "locks: review")

Read for `Mutex`, `RwLock`, `Once`, `try_lock`, a channel or any wait:

- `kmr_api/src/zenoh.rs`, `TickSide::inbound`: `Time` getters, the
  watchdog (one `Acquire` load per remote source, a `Duration` add, one
  `fetch_or` on the core's `ExternalFlags` when silent), one
  `LatestReader::take`, then the masked `RobotState::set_at` calls.
- `TickSide::outbound`: reads of depth 0 and one `LatestWriter::put`.
- `LatestReader::take`: `Output::update` (one load, one swap when new),
  then a copy out of the buffer. `LatestWriter::put`: a copy in, one swap,
  one `fetch_add` only when an untaken value was overwritten.

**Finding: no lock and no wait on the tick side.** The gateway thread
(`kmr_api/src/zenoh/gateway.rs`) sleeps and calls Zenoh, which locks; it
shares nothing with the tick but the two slots, the per-source accepted
counts and the external flags, all atomics.
