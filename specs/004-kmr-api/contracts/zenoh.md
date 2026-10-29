# Contract: the Zenoh gateway (feature `zenoh`)

Delivery steps 5 (gateway) and 7 (security). Off by default: a build without the feature
compiles no Zenoh crate (FR-043, SC-004 (SRS FR-7.1)). Message fields:
[data-model.md](../data-model.md#zenoh-data-feature-zenoh-gateway-threads-unless-marked-tick).

## Builder

```rust
Robot::new()
    .add_plugins(Gateway::new(GatewayConfig {
        instance: "arm-1",
        identity: Identity::files("/etc/kmr/arm-1.crt", "/etc/kmr/arm-1.key", "/etc/kmr/ca.crt"),
        read_history: &["viz-1", "ctl-2"],     // identities that may read History
    }))
    .add_remote_source(EachTick, Remote::allow("ctl-2").timeout(Duration::from_millis(20)), hold_still)
```

- Paths name files on the machine; their content never lives in source or the descriptor
  (FR-060 (SRS FR-13.2, NFR-12)).
- Only `add_remote_source` lets an identity publish Desired. It requires a fallback, so a source
  without one cannot be written (FR-059).

## Keys

| Key | Direction | Payload | Access |
| --- | --- | --- | --- |
| `kmr/<instance>/history` | out, once per tick taken | `SampleMsg` (depth 0, tick, age, flags) | `read_history` identities |
| `kmr/<instance>/history/ring` | queryable | the gateway's last `HISTORY_DEPTH` `SampleMsg` | `read_history` identities |
| `kmr/<instance>/desired` | in | `DesiredMsg` (values, per-field written mask, seq, sender time) | remote sources only |

Per-group keys: not offered (FR-048, SRS issue 6). Default deny for every other key.

## Timing per tick (stage points of kmr_core FR-053)

| Step | Who | What |
| --- | --- | --- |
| read | core | hardware read |
| inbound stage | kmr_api, tick | take the Desired slot once; write masked fields through `State`; fold pending flags (core) |
| `First`, `EachTick`, `Last` | user | a later local write to the same actuator wins |
| pre-write stage | core | limits and envelopes, on remote and local values alike |
| write | core | bulked write |
| outbound stage | kmr_api, tick | put depth 0, its age and its flags into the History slot |

A Desired taken in tick k is applied in tick k and never in k+1 (FR-046, SC-010).

## Counters (atomics, read outside the tick)

`missed_samples`, `superseded_desired`, `unauthorized`, `out_of_order`.

## Security behaviour (phase F)

| Incoming | Result |
| --- | --- |
| peer without a valid certificate | no session (transport) |
| Desired from an identity that is not a remote source | dropped, `unauthorized += 1`, unauthorized flag set |
| Desired with `seq` ≤ last accepted | dropped, `out_of_order += 1` |
| no Desired from a source for longer than its timeout | remote-timeout flag set; its fallback runs each tick until the source speaks again |

Authentication, decryption, ACL and sequence checks run on the gateway thread. The tick does no
crypto (FR-061 (SRS NFR-12)).
