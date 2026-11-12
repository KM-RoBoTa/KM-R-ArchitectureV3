# High-level design: Sensors (2026-11-12)

Step 3 for two changes: `sensors` (kmr_core, kmr_api) and `xiao-adapter`
(kmr_adapters, kmr_api, and three contract changes in
`../xiao-nrf52840-sensors`). Task lists: `docs/plans/sensors.md`,
`docs/plans/xiao-adapter.md`. Nothing is built yet.

## Decided (maintainer, 2026-11-12)

- The core's `Sensors` is a **closed typed set**, like State: sealed kinds,
  a small fixed number of slots per kind, typed readings with an age. The
  controller signature does not change.
- The XIAO adapter uses `xiao-api`, and `xiao-api` changes to serve a
  real-time loop: a read that never waits, sensors that own their link, a
  public test link.
- Scope: the IMU (accelerometer, gyroscope) **and** the microphone.

## What a controller sees

```rust
fn balance(_: &Time, state: &mut State, sensors: &Sensors) {
    if let Some(imu) = sensors.imu(0) {
        let w: Option<Gyro>  = imu.gyro();    // rad/s, None when that half is off
        let a: Option<Accel> = imu.accel();   // m/s²
        let age: u64 = imu.age();             // ticks since the reading arrived
    }
    if let Some(mic) = sensors.mic(0) {
        let pcm: &[i16] = mic.samples();      // what arrived since the last tick
        let rate: u32 = mic.rate_hz();        // 16_000 on the XIAO
    }
}

robot
    .add_sensor(xiao::Xiao::usb().imu().mic())
    .add_controller(EachTick, balance)
```

`imu(i)` / `mic(i)` are `None` when no sensor was declared at that index or
none has reported yet. Indices follow declaration order, per kind.

## Core (`kmr_core`, `no_std`, no heap)

| piece | file | what |
|---|---|---|
| value types | `sensors.rs` | `Accel([f32; 3])` m/s², `Gyro([f32; 3])` rad/s, with `x()`/`y()`/`z()` |
| storage | `sensors.rs` | `Sensors`: `[ImuSlot; MAX_IMUS]` (4), `[MicSlot; MAX_MICS]` (1). A mic slot is one flat window `[i16; MIC_WINDOW]` (1024 samples = 64 ms at 16 kHz) with a length, emptied at each tick's read |
| reader views | `sensors.rs` | `Imu<'_>`: `accel()`, `gyro()`, `age()`, `dropped()`. `Mic<'_>`: `samples()`, `rate_hz()`, `dropped()` |
| writer view | `sensors/feed.rs` | `Feed<'_>`: what one adapter may write, scoped to its own slots; marks them refreshed so the engine stamps the tick (same idea as `Telemetry::stamp`) |
| contract | `hal/sensor.rs` | `SensorHal: seal::Adapter`: `type Error`, `needs()`, `open()`, `read(Feed)`, `close()`. Each adapter brings its own typed error |
| list | `hal/sensing.rs` | `Sensing`, on `HNil` and `HCons<H: SensorHal, T: Sensing>`, like `Drivers`. Static dispatch, no `dyn` |

A sensor is not a `Hal<N>` member: a member that owns no actuator is refused
(`Coverage::Idle`, `hal/set.rs:131`), and a sensor has no torque to cut. It
gets its own trait and its own list.

**Slots.** At finalization, before any I/O, the engine walks the list, sums
`needs()` and hands each member its slot range. More than `MAX_IMUS` /
`MAX_MICS` is `RunError::Config`.

**When the engine calls it** (SRS FR-5.2):

- Init: `open` on each member, after the actuators' `Open` and `Zero`,
  before the Init controllers. An error ends the run before torque on.
- PostInit: one `read`, right after `read_all` and telemetry.
- Each tick: `read` right after `hal.read`, before `First`
  (`runtime.rs:441`). `read` must never wait.
- Every exit after `open`: `close`, from the `Powered` guard, **after**
  `torque_off` and `close` of the drivers. Actuators first.

**Missing.** `TickFlags::SENSOR_MISSING` (exists, never raised today) is
raised on a tick when a member reports its link down, or when a slot has
been silent for longer than the `max_silence` its member declared. A sensor
slower than the tick is not missing: at 104 Hz on a 1 kHz loop a reading is
simply 0 to 9 ticks old. By default a sensor lost at run time does not stop
the loop: the flag is the signal, the controller decides (FR-13.4). The
user may ask for more with `set_sensor_policy` (`Warn`, `Abort`).

## API (`kmr_api`)

- `Sensors<'a>` forwards `imu(i)` and `mic(i)` as borrowed wrappers;
  `Accel` and `Gyro` are re-exported from the core, not copied.
- `Robot::add_sensor(s)` grows a type-level list, like `add_controller`:
  one more generic on the wrapper, defaulted to `HNil`.
- Feature `xiao = ["kmr_adapters/xiao"]` exposes `kmr_api::xiao`.

## XIAO adapter (`kmr_adapters/src/xiao.rs`, feature `xiao`)

One member per board: `Xiao` holds the link description and which sensors
are asked for. Nothing opens before Init (FR-5.5). In `open` it opens the
link and turns the sensors on; in `read` it drains `try_latest` /
`try_next` into the `Feed`, converting g → m/s² (× 9.80665) and
°/s → rad/s (× π/180); `close` drops the sensors, which powers them off on
the board.

Heap and threads stay inside `xiao-api` (the SDK boundary exemption): the
adapter's own code names no heap type and spawns nothing.

### What changes in `xiao-api`

| change | why |
|---|---|
| `try_latest()` on `Imu`, `Gyro`, `Accel`; `try_next()` on `Mic` | `latest()` waits up to 2 s when nothing is queued |
| sensors own a shared link handle, no `'l` lifetime | an adapter struct must hold link and sensor together; closes the open question in the xiao `docs/deferred.md` |
| a public test link behind a `fake` feature | kmr tests script the board without a second fake of their own (audit finding #7) |

## Proposed SRS wording (maintainer edits `docs/srs/SRS.md`)

> FR-5.7 Sensors are read into a `Sensors` handle each tick. The set of
> sensor kinds is closed and typed (IMU: acceleration in m/s², angular
> rate in rad/s; microphone: PCM samples), with a fixed number of slots
> per kind and no heap. Each sensor adapter implements a sealed sensor
> contract separate from the actuator HAL, opens in Init, is read once in
> PostInit and once per tick before `First`, never blocks the tick, and is
> closed at every exit after the actuators. Controllers read each value
> and its age in ticks. A sensor that fails at bring-up fails `run()` with
> the adapter's own typed error, which says why and what causes it; a
> sensor lost at run time raises `sensor missing` (FR-13.4), and a
> user-set policy decides the rest: flag only (default), warn, or
> emergency stop.

## Settled (maintainer, 2026-11-12)

1. **Run entry.** One new full entry `run_all(hal, sensors, stages)`;
   `run_on` and `run_on_staged` delegate with `HNil`.
2. **Sensor errors are typed per adapter.** `SensorHal` has an associated
   `Error`; the list nests them like `Drivers` does (`MemberError`), and
   `RunError<E, S = Infallible>` gains `Sensor(S)`. The rule: no ambiguous
   error. A refusal always says why and what causes it ("busy" names the
   sensor and the link that holds it; "too slow" gives both speeds).
3. **Slot counts.** `MAX_IMUS = 4`, `MAX_MICS = 1`, `MIC_WINDOW = 1024`
   are the defaults, named once in `sensors.rs`. They must be settable;
   how (const generics with defaults, or a build-time setting) is a data
   layout choice, parked in `docs/deferred.md`.
4. **Run-time loss.** `SENSOR_MISSING` is always raised. What else happens
   is the user's choice: `SensorPolicy::Flag` (default, nothing more),
   `Warn` (logged once per member) or `Abort` (emergency stop that tick),
   set with `Robot::set_sensor_policy`.
