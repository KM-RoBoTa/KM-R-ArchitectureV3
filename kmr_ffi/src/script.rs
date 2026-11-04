//! What the Python and Lua bridges share: the name lists the script objects are built from,
//! and per-field access to the views in list order.
//!
//! Both come from [`for_each_field!`](crate::for_each_field) and
//! [`for_each_flag!`](crate::for_each_flag), so a new field reaches both bridges from its one
//! line (FR-014 (SRS FR-10.2)).

use crate::view::{ActuatorFlags, Mask, Slot, TimeView, WriteBuf};

/// A field or flag list as names.
macro_rules! names {
    ($($name:ident $(: $ty:ident)?),*) => {
        &[$(stringify!($name)),*]
    };
}
#[cfg(test)]
pub(crate) use names;

/// Every field, the names of `read.<field>` and `write.set_<field>`.
pub(crate) const FIELDS: &[&str] = crate::for_each_field!(names);

/// Every per-actuator flag, the names of `read.<flag>`.
pub(crate) const FLAGS: &[&str] = crate::for_each_flag!(names);

/// The clock's names, `time.<name>`, in [`time_values`] order.
pub(crate) const TIME: &[&str] = &[
    "tick",
    "dt_ns",
    "elapsed_ns",
    "overrun_count",
    "last_overrun_ns",
    "accumulated_overrun_ns",
];

/// The clock's values, in [`TIME`] order.
pub(crate) const fn time_values(t: &TimeView) -> [u64; 6] {
    [
        t.tick,
        t.dt_ns,
        t.elapsed_ns,
        t.overrun_count,
        t.last_overrun_ns,
        t.accumulated_overrun_ns,
    ]
}

/// The mask of actuators `0..n`.
pub(crate) const fn all(n: usize) -> Mask {
    if n >= Mask::BITS as usize {
        Mask::MAX
    } else {
        (1 << n) - 1
    }
}

/// Generates the per-field and per-flag walks over the views.
macro_rules! walks {
    ($($field:ident: $ty:ident),*) => {
        /// Calls `f(name, values)` for every field of `slot`, in [`FIELDS`] order.
        pub(crate) fn slot_fields<const N: usize, E>(
            slot: &Slot<N>,
            mut f: impl FnMut(&'static str, &[f32; N]) -> Result<(), E>,
        ) -> Result<(), E> {
            $(f(stringify!($field), &slot.$field)?;)*
            Ok(())
        }

        /// Calls `f(name, values, written)` for every field of `buf`, in [`FIELDS`] order.
        pub(crate) fn buf_fields<const N: usize, E>(
            buf: &mut WriteBuf<N>,
            mut f: impl FnMut(&'static str, &mut [f32; N], &mut Mask) -> Result<(), E>,
        ) -> Result<(), E> {
            $(f(stringify!($field), &mut buf.$field, &mut buf.written.$field)?;)*
            Ok(())
        }
    };
}
crate::for_each_field!(walks);

/// Generates the per-flag walk.
macro_rules! flag_walk {
    ($($flag:ident),*) => {
        /// Calls `f(name, mask)` for every flag, in [`FLAGS`] order.
        pub(crate) fn flag_masks<E>(
            flags: &ActuatorFlags,
            mut f: impl FnMut(&'static str, Mask) -> Result<(), E>,
        ) -> Result<(), E> {
            $(f(stringify!($flag), flags.$flag)?;)*
            Ok(())
        }
    };
}
crate::for_each_flag!(flag_walk);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn walks_follow_the_lists() {
        let mut seen = Vec::new();
        slot_fields(&Slot::<4>::default(), |name, _| {
            seen.push(name);
            Ok::<(), ()>(())
        })
        .ok();
        assert_eq!(seen, FIELDS);

        seen.clear();
        flag_masks(&ActuatorFlags::default(), |name, _| {
            seen.push(name);
            Ok::<(), ()>(())
        })
        .ok();
        assert_eq!(seen, FLAGS);

        seen.clear();
        let mut buf = WriteBuf::<4>::default();
        buf_fields(&mut buf, |name, values, written| {
            seen.push(name);
            values[0] = 1.0;
            *written = 1;
            Ok::<(), ()>(())
        })
        .ok();
        assert_eq!(seen, FIELDS);
        assert_eq!(buf.written.tau, 1);
    }

    #[test]
    fn the_mask_covers_n_actuators() {
        assert_eq!(all(1), 1);
        assert_eq!(all(4), 0b1111);
        assert_eq!(all(64), u64::MAX);
    }

    #[test]
    fn time_names_match_values() {
        let t = TimeView {
            tick: 1,
            dt_ns: 2,
            elapsed_ns: 3,
            overrun_count: 4,
            last_overrun_ns: 5,
            accumulated_overrun_ns: 6,
        };
        assert_eq!(time_values(&t), [1, 2, 3, 4, 5, 6]);
        assert_eq!(TIME.len(), 6);
    }
}
