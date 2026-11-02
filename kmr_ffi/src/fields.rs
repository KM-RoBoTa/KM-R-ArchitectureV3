//! The one declaration of what crosses: the state fields and the per-actuator flags.
//!
//! Every per-field and per-flag item is generated from these two lists: the Rust types, the
//! C and C++ headers and the bridges (FR-014 (SRS FR-10.2)). A new field is one line here.

/// Calls `$cb!` with every field of the core's sealed set, as `name: Type` pairs.
///
/// `for_each_field!(cb)` expands to `cb! { q: Q, qd: Qd, tau: Tau }`. Tokens after the callback
/// come first, ended by `;`: `for_each_field!(cb, 4)` expands to `cb! { 4; q: Q, .. }`.
#[macro_export]
macro_rules! for_each_field {
    ($cb:ident) => {
        $cb! { q: Q, qd: Qd, tau: Tau }
    };
    ($cb:ident, $($pre:tt)*) => {
        $cb! { $($pre)*; q: Q, qd: Qd, tau: Tau }
    };
}

/// Calls `$cb!` with every per-actuator flag of one History slot (core K4.1, SRS FR-13.4).
///
/// `envelope` is per actuator: assumed, pending human: D3 A. Tokens after the callback come
/// first, ended by `;`, as in [`for_each_field!`].
#[macro_export]
macro_rules! for_each_flag {
    ($cb:ident) => {
        $cb! { stale, bus_error, offline, rejected, joint_limit, envelope }
    };
    ($cb:ident, $($pre:tt)*) => {
        $cb! { $($pre)*; stale, bus_error, offline, rejected, joint_limit, envelope }
    };
}

#[cfg(test)]
mod tests {
    /// The real field list plus `dummy: Dummy`, handed to `$cb!`.
    macro_rules! plus_dummy {
        ($cb:ident; $($field:ident: $ty:ident),*) => {
            $cb! { $($field: $ty,)* dummy: Dummy }
        };
    }

    /// The types of [`crate::view`], generated from the list with `dummy`.
    mod with_dummy {
        use crate::view::define_field_types;
        crate::for_each_field!(plus_dummy, define_field_types);
    }

    #[test]
    fn one_line_adds_a_field_to_the_types() {
        let slot = with_dummy::Slot::<4>::default();
        let buf = with_dummy::WriteBuf::<4>::default();
        let written = with_dummy::Written::default();
        assert_eq!(slot.dummy.len(), 4);
        assert_eq!(buf.dummy.len(), 4);
        assert_eq!(written.dummy, 0);
        assert_eq!(
            core::mem::size_of::<with_dummy::Written>(),
            core::mem::size_of::<crate::Written>() + 8
        );
    }

    #[test]
    fn one_line_adds_a_field_to_the_headers() {
        use crate::build::{FLAGS, c_header_from, cpp_header_from, field_names};
        let fields = crate::for_each_field!(plus_dummy, field_names);
        let c = c_header_from(4, 4, fields, FLAGS);
        assert_eq!(c.matches("    float dummy[KMR_N];\n").count(), 2);
        assert!(c.contains("    uint64_t dummy;\n"));
        assert!(c.contains("static inline void kmr_set_dummy(kmr_write_buf *w"));
        let cpp = cpp_header_from(fields, FLAGS);
        assert!(cpp.contains("struct Dummy {"));
        assert!(cpp.contains("Dummy dummy(std::size_t depth, std::size_t i) const noexcept"));
        assert!(cpp.contains("void set(std::size_t i, Dummy v) noexcept"));
    }
}
