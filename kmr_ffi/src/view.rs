//! The types that cross to foreign code: `#[repr(C)]`, `Copy`, no pointers, no lifetimes.
//!
//! Generic over N (actuators) and D (History depth) on the Rust side; the C side gets the same
//! layout from a header generated for one N and D (research R1). Values are in engine units.

/// One bit per actuator, bit i = actuator i. Its width caps N (assumed, pending human: D4 A).
///
/// The `Default` of [`ReadView`], [`Slot`] and [`WriteBuf`] fails the build for an N outside
/// `1..=64`. A struct literal skips that check: build these types through `Default`.
pub type Mask = u64;

/// Holds the compile-time check that N actuators fit a [`Mask`].
pub(crate) struct MaskFits<const N: usize>;

impl<const N: usize> MaskFits<N> {
    /// Fails the build for an N a [`Mask`] cannot hold.
    pub(crate) const CHECK: () = assert!(
        N >= 1 && N <= Mask::BITS as usize,
        "kmr_ffi: N must be in 1..=64, one mask bit per actuator"
    );
}

/// The age of an actuator that was never read, the core's `None` (`KMR_AGE_NONE` in C).
pub const AGE_NONE: u32 = u32::MAX;

/// Generates [`ActuatorFlags`] from a flag list.
macro_rules! define_flags {
    ($($flag:ident),* $(,)?) => {
        /// The per-actuator faults of one History slot, one mask per flag (FR-010 (SRS FR-2.6,
        /// FR-13.4)).
        #[repr(C)]
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
        pub struct ActuatorFlags {
            $(
                #[doc = concat!("Bit i set: `", stringify!($flag), "` on actuator i.")]
                pub $flag: $crate::view::Mask,
            )*
        }
    };
}

/// Generates [`Slot`], [`Written`] and [`WriteBuf`] from a field list.
macro_rules! define_field_types {
    ($($field:ident: $ty:ident),* $(,)?) => {
        /// One History slot: one array per field in engine units, and its faults.
        ///
        /// A stale actuator holds its last good value, never zero. `Default` checks N, as
        /// [`Mask`](crate::Mask) says.
        #[repr(C)]
        #[derive(Clone, Copy, Debug, PartialEq)]
        pub struct Slot<const N: usize> {
            $(
                #[doc = concat!("`", stringify!($ty), "` of each actuator.")]
                pub $field: [f32; N],
            )*
            /// Per-actuator faults.
            pub flags: $crate::view::ActuatorFlags,
            /// Per-tick faults (overrun, remote timeout, unauthorized), one bit each.
            pub tick_flags: u32,
            /// Zero; keeps the size a multiple of 8.
            pub reserved: u32,
        }

        impl<const N: usize> Default for Slot<N> {
            fn default() -> Self {
                let () = $crate::view::MaskFits::<N>::CHECK;
                Self {
                    $($field: [0.0; N],)*
                    flags: $crate::view::ActuatorFlags::default(),
                    tick_flags: 0,
                    reserved: 0,
                }
            }
        }

        /// Which entries of a [`WriteBuf`] foreign code wrote, one mask per field.
        #[repr(C)]
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
        pub struct Written {
            $(
                #[doc = concat!("Bit i set: `", stringify!($field), "[i]` was written.")]
                pub $field: $crate::view::Mask,
            )*
        }

        /// The values foreign code writes in one call; only the entries flagged in `written`
        /// count (FR-011 (SRS FR-2.5)). `Default` checks N, as [`Mask`](crate::Mask)
        /// says.
        #[repr(C)]
        #[derive(Clone, Copy, Debug, PartialEq)]
        pub struct WriteBuf<const N: usize> {
            $(
                #[doc = concat!("`", stringify!($ty), "` to write, per actuator.")]
                pub $field: [f32; N],
            )*
            /// Which entries count.
            pub written: Written,
        }

        impl<const N: usize> Default for WriteBuf<N> {
            fn default() -> Self {
                let () = $crate::view::MaskFits::<N>::CHECK;
                Self {
                    $($field: [0.0; N],)*
                    written: Written::default(),
                }
            }
        }
    };
}

crate::for_each_flag!(define_flags);
crate::for_each_field!(define_field_types);
#[cfg(test)]
pub(crate) use define_field_types;

/// Everything foreign code reads from the state: every History slot and per-actuator ages.
///
/// `slot[k]` is `prev(k)`, the same values as `prev_q(k)` and its siblings (FR-010 (SRS
/// FR-2.3)). N is in `1..=64`, checked at compile time by `Default`, as [`Mask`] says:
///
/// ```
/// let view = kmr_ffi::ReadView::<64, 4>::default();
/// assert_eq!(view.samples, 0);
/// ```
///
/// ```compile_fail
/// let _ = kmr_ffi::ReadView::<65, 4>::default();
/// ```
///
/// ```compile_fail
/// let _ = kmr_ffi::Slot::<0>::default();
/// ```
///
/// ```compile_fail
/// let _ = kmr_ffi::WriteBuf::<65>::default();
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReadView<const N: usize, const D: usize> {
    /// Valid slots, `0..=D`. 0 = no sample: do not read `slot`.
    pub samples: u32,
    /// Zero.
    pub reserved: u32,
    /// Ticks since actuator i was last read, latest slot only; [`AGE_NONE`] = never read.
    pub age: [u32; N],
    /// The History, newest first.
    pub slot: [Slot<N>; D],
}

impl<const N: usize, const D: usize> Default for ReadView<N, D> {
    fn default() -> Self {
        let () = MaskFits::<N>::CHECK;
        Self {
            samples: 0,
            reserved: 0,
            age: [AGE_NONE; N],
            slot: [Slot::default(); D],
        }
    }
}

/// The clock as foreign code reads it, the same values as the `Time` readers (SRS FR-3.3).
///
/// Integers, so ticks and nanoseconds stay exact (assumed, pending human: Q1).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TimeView {
    /// Index of the tick in progress, 0 first.
    pub tick: u64,
    /// The tick period.
    pub dt_ns: u64,
    /// Since start, stamped at tick start.
    pub elapsed_ns: u64,
    /// Late ticks so far.
    pub overrun_count: u64,
    /// The previous tick's lateness; 0 = on time.
    pub last_overrun_ns: u64,
    /// Sum of lateness.
    pub accumulated_overrun_ns: u64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::{align_of, offset_of, size_of};

    /// Rounds `x` up to a multiple of `a`, the C11 padding rule.
    const fn align_up(x: usize, a: usize) -> usize {
        x.div_ceil(a) * a
    }

    /// The number of names in a list.
    macro_rules! count {
        ($($name:ident $(: $ty:ident)?),*) => {
            [$(stringify!($name)),*].len()
        };
    }

    /// The offset of every field array in `$ty<$n>`, in list order.
    macro_rules! field_offsets {
        ($ty:ident, $n:literal; $($field:ident: $fty:ident),*) => {
            [$(offset_of!($ty<$n>, $field)),*]
        };
    }

    /// The offset of every mask in `Written`, in list order.
    macro_rules! written_offsets {
        ($($field:ident: $fty:ident),*) => {
            [$(offset_of!(Written, $field)),*]
        };
    }

    /// The offset of every mask in `ActuatorFlags`, in list order.
    macro_rules! flag_offsets {
        ($($flag:ident),*) => {
            [$(offset_of!(ActuatorFlags, $flag)),*]
        };
    }

    const FIELDS: usize = crate::for_each_field!(count);
    const FLAGS: usize = crate::for_each_flag!(count);
    const D: usize = 4;

    /// Checks size, alignment and every offset of the six types against a hand count by the
    /// C11 rules: fields in order, each at the next multiple of its alignment, the size padded
    /// to the largest alignment.
    fn check<const N: usize>(slot: &[usize], buf: &[usize]) {
        let arr = 4 * N;
        let flags_size = 8 * FLAGS;
        let slot_flags = align_up(arr * FIELDS, 8);
        let slot_size = align_up(slot_flags + flags_size + 8, 8);
        let view_slot = align_up(8 + 4 * N, 8);
        let buf_written = align_up(arr * FIELDS, 8);

        assert_eq!(size_of::<ActuatorFlags>(), flags_size);
        assert_eq!(align_of::<ActuatorFlags>(), 8);
        for (k, off) in crate::for_each_flag!(flag_offsets).iter().enumerate() {
            assert_eq!(*off, 8 * k);
        }

        assert_eq!(size_of::<Slot<N>>(), slot_size);
        assert_eq!(align_of::<Slot<N>>(), 8);
        for (k, off) in slot.iter().enumerate() {
            assert_eq!(*off, arr * k);
        }
        assert_eq!(offset_of!(Slot<N>, flags), slot_flags);
        assert_eq!(offset_of!(Slot<N>, tick_flags), slot_flags + flags_size);
        assert_eq!(offset_of!(Slot<N>, reserved), slot_flags + flags_size + 4);

        assert_eq!(size_of::<ReadView<N, D>>(), view_slot + D * slot_size);
        assert_eq!(align_of::<ReadView<N, D>>(), 8);
        assert_eq!(offset_of!(ReadView<N, D>, samples), 0);
        assert_eq!(offset_of!(ReadView<N, D>, reserved), 4);
        assert_eq!(offset_of!(ReadView<N, D>, age), 8);
        assert_eq!(offset_of!(ReadView<N, D>, slot), view_slot);

        assert_eq!(size_of::<Written>(), 8 * FIELDS);
        assert_eq!(align_of::<Written>(), 8);
        for (k, off) in crate::for_each_field!(written_offsets).iter().enumerate() {
            assert_eq!(*off, 8 * k);
        }

        assert_eq!(size_of::<WriteBuf<N>>(), buf_written + 8 * FIELDS);
        assert_eq!(align_of::<WriteBuf<N>>(), 8);
        for (k, off) in buf.iter().enumerate() {
            assert_eq!(*off, arr * k);
        }
        assert_eq!(offset_of!(WriteBuf<N>, written), buf_written);

        assert_eq!(size_of::<TimeView>(), 48);
        assert_eq!(align_of::<TimeView>(), 8);
        let time = [
            offset_of!(TimeView, tick),
            offset_of!(TimeView, dt_ns),
            offset_of!(TimeView, elapsed_ns),
            offset_of!(TimeView, overrun_count),
            offset_of!(TimeView, last_overrun_ns),
            offset_of!(TimeView, accumulated_overrun_ns),
        ];
        for (k, off) in time.iter().enumerate() {
            assert_eq!(*off, 8 * k);
        }
    }

    /// Research R1's measured table (three fields, six flags): sizes and the offsets it lists.
    fn r1_table<const N: usize>(
        slot: usize,
        slot_flags: usize,
        view: usize,
        view_slot: usize,
        buf: usize,
    ) {
        if FIELDS != 3 || FLAGS != 6 {
            return;
        }
        assert_eq!(size_of::<ActuatorFlags>(), 48);
        assert_eq!(size_of::<Slot<N>>(), slot);
        assert_eq!(offset_of!(Slot<N>, flags), slot_flags);
        assert_eq!(size_of::<ReadView<N, D>>(), view);
        assert_eq!(offset_of!(ReadView<N, D>, age), 8);
        assert_eq!(offset_of!(ReadView<N, D>, slot), view_slot);
        assert_eq!(size_of::<WriteBuf<N>>(), buf);
        assert_eq!(offset_of!(WriteBuf<N>, written), slot_flags);
        assert_eq!(size_of::<TimeView>(), 48);
    }

    #[test]
    fn layout_n1() {
        check::<1>(
            &crate::for_each_field!(field_offsets, Slot, 1),
            &crate::for_each_field!(field_offsets, WriteBuf, 1),
        );
        r1_table::<1>(72, 16, 304, 16, 40);
    }

    #[test]
    fn layout_n4() {
        check::<4>(
            &crate::for_each_field!(field_offsets, Slot, 4),
            &crate::for_each_field!(field_offsets, WriteBuf, 4),
        );
        r1_table::<4>(104, 48, 440, 24, 72);
    }

    #[test]
    fn layout_n64() {
        check::<64>(
            &crate::for_each_field!(field_offsets, Slot, 64),
            &crate::for_each_field!(field_offsets, WriteBuf, 64),
        );
        r1_table::<64>(824, 768, 3560, 264, 792);
    }

    // A definition checks a type for FFI safety by value only, not behind a pointer.
    #[deny(improper_ctypes_definitions)]
    extern "C" fn by_value(
        view: ReadView<4, 4>,
        _buf: WriteBuf<4>,
        _time: TimeView,
        _slot: Slot<4>,
        _flags: ActuatorFlags,
        _written: Written,
    ) -> u32 {
        view.samples
    }

    #[test]
    fn every_type_is_ffi_safe() {
        let n = by_value(
            ReadView::default(),
            WriteBuf::default(),
            TimeView::default(),
            Slot::default(),
            ActuatorFlags::default(),
            Written::default(),
        );
        assert_eq!(n, 0);
    }

    #[test]
    fn a_default_view_has_no_sample_and_no_read() {
        let view = ReadView::<4, 4>::default();
        assert_eq!(view.samples, 0);
        assert!(view.age.iter().all(|a| *a == AGE_NONE));
        assert_eq!(AGE_NONE, u32::MAX);
    }
}
