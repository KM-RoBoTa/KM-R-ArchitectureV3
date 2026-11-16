//! The write buffer a C or C++ controller stages into: `#[repr(C)]`, `Copy`, no pointers, no
//! lifetimes.
//!
//! Generic over N (actuators). [`WriteBuf`] is the staging area of the C snapshot,
//! [`CState`](crate::CState). Values are in engine units.

/// One bit per actuator, bit i = actuator i. Its width caps N (assumed, pending human: D4 A).
///
/// The `Default` of [`WriteBuf`] fails the build for an N outside `1..=64`. A struct literal
/// skips that check: build it through `Default`.
///
/// ```
/// let buf = kmr_ffi::WriteBuf::<64>::default();
/// assert_eq!(buf.written, kmr_ffi::Written::default());
/// ```
///
/// ```compile_fail
/// let _ = kmr_ffi::WriteBuf::<0>::default();
/// ```
///
/// ```compile_fail
/// let _ = kmr_ffi::WriteBuf::<65>::default();
/// ```
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

/// Generates [`Written`] and [`WriteBuf`] from a field list.
macro_rules! define_field_types {
    ($($field:ident: $ty:ident),* $(,)?) => {
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

crate::for_each_field!(define_field_types);
#[cfg(test)]
pub(crate) use define_field_types;

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
        ($($name:ident: $ty:ident),*) => {
            [$(stringify!($name)),*].len()
        };
    }

    /// The offset of every field array in `WriteBuf<$n>`, in list order.
    macro_rules! field_offsets {
        ($n:literal; $($field:ident: $fty:ident),*) => {
            [$(offset_of!(WriteBuf<$n>, $field)),*]
        };
    }

    /// The offset of every mask in `Written`, in list order.
    macro_rules! written_offsets {
        ($($field:ident: $fty:ident),*) => {
            [$(offset_of!(Written, $field)),*]
        };
    }

    const FIELDS: usize = crate::for_each_field!(count);

    /// Checks size, alignment and every offset of both types against a hand count by the C11
    /// rules: fields in order, each at the next multiple of its alignment, the size padded to
    /// the largest alignment.
    fn check<const N: usize>(buf: &[usize]) {
        let arr = 4 * N;
        let buf_written = align_up(arr * FIELDS, 8);

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
    }

    #[test]
    fn layout_n1() {
        check::<1>(&crate::for_each_field!(field_offsets, 1));
    }

    #[test]
    fn layout_n4() {
        check::<4>(&crate::for_each_field!(field_offsets, 4));
    }

    #[test]
    fn layout_n64() {
        check::<64>(&crate::for_each_field!(field_offsets, 64));
    }

    // A definition checks a type for FFI safety by value only, not behind a pointer.
    #[deny(improper_ctypes_definitions)]
    extern "C" fn by_value(buf: WriteBuf<4>, written: Written) -> u64 {
        buf.written.q | written.q
    }

    #[test]
    fn every_type_is_ffi_safe() {
        assert_eq!(by_value(WriteBuf::default(), Written::default()), 0);
    }
}
