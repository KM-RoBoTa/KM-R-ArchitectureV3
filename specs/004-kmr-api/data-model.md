# Data model: kmr_api

Two kinds of data. **Builder data** lives on the heap, outside the tick, and dies at the end of
`run()`'s finalization or with the process. **Tick data** is borrowed or fixed-size: no heap,
no lock, no wait (FR-067 (SRS NFR-1)). `N` below is `JOINTS` until kmr_core takes N as a const
generic, then the `N` that `robot!` emits.

## Builder data

| Entity | Fields | Rules | Spec ids |
| --- | --- | --- | --- |
| `Robot<H, C = Scheduled>`; `Robot<N, H, C>` after B7; `Robot<N, H, C, E = Empty>` after C6 (core `E`: `Empty`, `Filled`) | `inner: kmr_core::Robot<C>` (then `<N, C, E>`), `plugins: Vec<Binding>`, `dt: Option<Duration>`, `payloads: Vec<Payload>`, `envelopes`, `limit_policy`, `shutdown_hold: Option<Duration>`, `gateway: Option<Gateway>` (feature `zenoh`), `PhantomData<fn() -> H>` | `H: Spec` comes from `robot!`. `C` grows by one node per registration. Settings keep `C`. Consumed by `run()` or `bind()` | FR-009, FR-014, FR-062 |
| `Spec` (hidden trait) | `type Drivers`: the list of drivers, one `Hal` member per brand of the hardware table (frunk HList; `HCons<__DxlAdapter<__NB>, HNil>` for XmBench), not one adapter type (2026-10-29, `specs/002-kmr-core/design-hal-set.md`; the first motor may pass its one member alone until core T069, pending human); `fn bind(&[Binding]) -> Result<Drivers, UserError>`; after step 2: `PRISMATIC`, `CONTINUOUS` (`[u64; MASK_WORDS]`), `LOWER`, `UPPER` (`[Q; N]`), `SPEED` (`[Qd; N]`), `EFFORT` (`[Tau; N]`), `LINK_NAMES` as consts. Handed to the core as `[Limit; N]`, `lower`/`upper` `None` where `CONTINUOUS` is set | One impl per binary, emitted by `robot!` on a private `__Spec`. Users never implement it by hand | FR-002, FR-005 |
| `Bus` | wraps kmr_adapters `Binding { port: Port, baud: Option<u32>, bus: Option<&'static str> }` | `by_id(serial)` preferred, `by_dev(path)` allowed; `.baud(..)`, `.bus(label)` (D6) | FR-022 to FR-024 |
| `Payload` | `tier: Medium \| Heavy` (`Tier`), `mass_kg: f32`, `mount: &'static str`; private, read through `const` getters | No light tier. Mass finite and > 0. Mount names a model link, checked at finalization | FR-062 to FR-064 |
| `Envelope` | `q: Option<(Q, Q)>`, `qd: Option<(Qd, Qd)>`, `tau: Option<(Tau, Tau)>` | lower ≤ upper, finite. Stored as per-actuator arrays, handed to the core's pre-write stage | FR-051, FR-052 |
| `LimitPolicy` | `Clamp \| Warn \| Abort` per actuator, `Option` | No default. Assumed, pending human: D2 B: `run()` refuses a limited actuator without one (core K5.3); `set_limit_policy_all` sets all in one call | FR-019 |
| `UserError` | `DuplicateBus { port }`, `FeatureOff { feature }`, `NoHardwareDeclared`, `DtOutOfRange { dt }`, `UnknownLink { link }`, `NoPolicy { actuator }` (D2 B), `BringUp { source: Box<dyn Error + Send + Sync> }` (the adapter's error, typed, reached through `source()`, 2026-10-29); `NoControllers` until core #15 | `Display` names the cause; `Debug` derived in debug builds (variant and fields), prints `Display` in release (SRS NFR-2) | FR-015, FR-021, FR-025, FR-027, FR-064 |
| `Bound<H, C>` | the finalized robot and its bound drivers | `drivers()` before the run (`None` without a plugin: the run simulates); `run()` hands the drivers back, `Option<H::Drivers>`. Never reachable in a tick | FR-041 |

### Finalization, in order (first failure stops; no hardware I/O before step 6)

1. `dt`: zero or outside [10 µs, 1 s] is `DtOutOfRange`; unset is the core's 1 ms warning (R13).
2. Plugins: two with the same port string is `DuplicateBus` (FR-025).
3. Payloads: each mount link in `Spec::LINK_NAMES`, else `UnknownLink` (FR-064).
4. No plugin: the drivers are `hlist![NoHardware]`; skip to 6.
5. Plugins: `Spec::bind(&plugins)`: `FeatureOff`, `NoHardwareDeclared`, or the adapter's
   binding errors (label, baud) as `BringUp` (kmr_adapters port-binding contract).
6. `inner.run_on(drivers)`: the core's own finalization (`dt` range after K3.2, policies after
   K5.3: `RunError::Config`, no I/O), then `check` on every member and the coverage check
   (`specs/002-kmr-core/design-hal-set.md` §4), then bring-up. Port resolution errors
   (`PortNotFound`, `PortAccess`, `SharedPort`) are bring-up errors, returned as `BringUp`
   after the core's guard cut torque and closed on every member.

Remote sources without a fallback cannot be written (R18), so they need no step here.

## Tick data (borrowed per call)

| Entity | Shape | Rules | Spec ids |
| --- | --- | --- | --- |
| `State<'a>` | `{ inner: &'a mut kmr_core::RobotState }` (named field) | Reads History, writes Desired. Plain methods forward to the core's generic ones, no logic. Index out of range: `Err`, nothing written | FR-028 to FR-031, FR-035 |
| `GroupView<'a, K>` | the State borrow + `[usize; K]` | Same accessor set as `State`. Duplicates allowed, last write wins | FR-034 |
| `Time<'a>` | `{ inner: &'a kmr_core::Time }` | Six getters by value; no setter | FR-038 |
| `Sensors<'a>` | `{ inner: &'a kmr_core::Sensors }` | Surface open (SRS FR-5.7) | FR-040 |
| `Faults` (view) | per actuator: stale, bus error, offline, rejected, joint limit hit, envelope (bit sets; envelope under D3 A, assumed); per tick: overrun, sensor missing, remote timeout, unauthorized | Read by depth through `State`. Controllers cannot set or clear | FR-039 |
| Age | `age_at(i) -> Result<Option<u32>, StateError>` ticks, `sample_age() -> Option<u32>` (oldest actuator), `stale_at(i) -> bool` | Forwarded from kmr_core FR-018. `None`: never read (no-hardware run, or before the first read) | FR-031 |
| Units | `Deg`, `Cm`, `M`, `DegPerS`, `CmPerS`, `MPerS` (`pub f32` tuple structs) | `From` into `Q` / `Qd` only: rad, mm, rad/s, mm/s | FR-033 |
| `Guarded<F, G, P>` | controller `F`, fallback `G`, predicate `P: Fn(&Faults) -> bool` | Calls exactly one of `F`, `G` per tick. Named type, stored in an `Inline` node | FR-049 |

## Zenoh data (feature `zenoh`; gateway threads unless marked "tick")

| Entity | Fields | Rules | Spec ids |
| --- | --- | --- | --- |
| `Gateway` | instance key, transport config (TLS or QUIC paths), ACL, sources | A plugin; owns the session and threads | FR-043, FR-054 |
| `DesiredMsg` | `seq: u64`, `sent_ns: u64` (logged, not checked), `q/qd/tau: [f32; N]`, `written: [u64; 3]` (one mask per field) | Decoded and checked on the gateway thread. Only fields whose bit is set are applied | FR-046, FR-057 |
| `SampleMsg` | `tick: u64`, `age: Option<u32>` (sample age), `q/qd/tau: [f32; N]`, `faults` bit sets | Built from depth 0 after the write; the outbound stage gets `&RobotState<N>`, so age and flags are reachable | FR-045 |
| `Latest<T>` (tick side: take or put) | triple buffer of `T: Copy`, one writer, one reader | Writer: one swap. Reader: one load, one swap if new. Untaken overwrite counted | FR-044 |
| `Ring` | last `HISTORY_DEPTH` `SampleMsg` the gateway took | Served on request; gaps visible through `tick` | FR-045 |
| `RemoteSource` | `identity`, `timeout: Duration`, `last_seq: u64`, `last_seen`, the fallback's registration | Built only with a fallback (R18) | FR-055 to FR-059 |
| Counters | `missed_samples`, `superseded_desired`, `unauthorized`, `out_of_order` (atomics, gateway side) | Readable by the user outside the tick | FR-045, FR-056, FR-057 |

### Desired from a remote peer, one message

1. Transport authenticates the peer (mTLS or QUIC). Unknown peers never get a session.
2. Gateway thread: identity not allowed to publish Desired: drop, `unauthorized += 1`, set the
   unauthorized bit through the core's pending word (kmr_core FR-052).
3. `seq <= last_seq`: drop, `out_of_order += 1`.
4. Put into the Desired slot. An untaken one is overwritten and counted as superseded.
5. Tick k, inbound stage (after read, before `First`): take once; write the masked fields
   through `State`. Not held to tick k+1.
6. Local controllers may overwrite the same actuators later in tick k; the last write wins.
7. The core's pre-write stage (envelopes, limits) applies to both.

## Relationships

- Actuator index `i` is the same in `State`, groups, envelopes, the hardware table rows, the
  model arrays and the Zenoh arrays (SRS Definitions "Index space").
- `robot!` emits the `Spec` impl, the aliases and the checks; the model facts come from
  kmr_model's expansion (specs/003-kmr-model/data-model.md, "Emitted facts"), not duplicated
  here.
- The hardware table is kmr_adapters' `dynamixel::Table<NB, N>`
  (specs/001-kmr-adapters/contracts/hardware-table.md); `robot!` holds one as `__TABLE`.
