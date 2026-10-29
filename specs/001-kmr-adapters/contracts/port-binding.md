# Contract: port binding

How bus plugins reach the adapter. FR-029 to FR-031 (SRS FR-6.3, FR-6.4). D6 (spec Q1) is
planned with option B, pending human confirmation, and lives in `match_plugins` only
(research R5).

## Types

```rust
pub mod binding {
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum Port {
        /// A `/dev/serial/by-id` entry holding this serial. Preferred.
        ById(&'static str),
        /// This device path, as given. Less reliable across reboots.
        ByDev(&'static str),
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct Binding {
        pub port: Port,
        pub baud: Option<u32>,
        pub bus: Option<&'static str>,
    }
}
```

`kmr_api::Bus` wraps a `Binding`: `by_id` and `by_dev` set `port`, `.baud(..)` sets `baud`,
`.bus("label")` sets `bus`.

## Entry point

```rust
impl<B: Backend, const NB: usize> Dynamixel<B, NB> {
    /// Binds the plugins to the table's buses. No I/O.
    pub fn bind(backend: B, table: Table<NB, JOINTS>, plugins: &[Binding])
        -> Result<Self, DynamixelError>;
}
```

The public, SDK-backed form is `Dynamixel::<DxlRs, NB>::bind(DxlRs::new(), ..)`, under the
`dynamixel` feature. `kmr_api::run()` calls it, then `run_on(adapter)`. With no plugin, `kmr_api`
never calls it and uses `NoHardware` (FR-031).

## Matching (D6, option B)

`match_plugins(labels: [&str; NB], plugins: &[Binding]) -> Result<[Binding; NB], DynamixelError>`:

| Case | Result |
|---|---|
| one bus, one plugin, no label | bound |
| one bus, one plugin, its label | bound |
| plugin label not in the table | `UnknownBus { label }` |
| a bus with no plugin | `UnboundBus { bus }` |
| more plugins than buses | `ExtraPlugin { port }` |
| several buses, a plugin without a label | `LabelRequired { port }` |
| two plugins with one label | `BusBoundTwice { bus }` |

All before any I/O: `run()` errors. Option A or C replaces this function only.

## Baud

The plugin baud replaces the table's for that bus. Each model on the bus must accept it, else
`BaudRejected { actuator, bus, baud }`, before any I/O. The core's `Overrides` is not used for
the baud (research R6).

## Resolution (Init, in `open`)

| Port | Resolved to | Errors |
|---|---|---|
| `ById(serial)` | the single entry of `/dev/serial/by-id` whose name holds `serial` | `PortNotFound` (none or several) |
| `ByDev(path)` | `path` | `PortNotFound` (missing) |
| both | pre-flight: character device, readable and writable | `PortAccess` |
| two buses, one device | — | `SharedPort { first, bus }` |

These are bring-up errors (SRS FR-6.4), returned by `run_on` after `torque_off` (no-op) and
`close`. Every message names the bus label and the serial or path.
