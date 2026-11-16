//! The one declaration of what crosses: the state fields and the per-actuator flags.
//!
//! Every per-field and per-flag item is generated from these two lists: the Rust types, the
//! C and C++ headers and the bridges (FR-014 (SRS FR-10.2)). A new field is one line here.

/// Calls `$cb!` with every field of the core's sealed set, as `name: Type` pairs.
///
/// `for_each_field!(cb)` expands to `cb! { q: Q, qd: Qd, tau: Tau, kp: Kp, kd: Kd }`. Tokens
/// after the callback come first, ended by `;`: `for_each_field!(cb, 4)` expands to
/// `cb! { 4; q: Q, .. }`. Each `Type` is the name of the core's `FieldId` variant, so kmr_api's
/// apply step, an exhaustive `match` generated from this list, stops compiling when the list
/// misses a field.
// DECISION: the gains `kp`, `kd` cross too: they are in the core's sealed set (K3.6), and the
// apply `match` on `FieldId` must cover every variant.
#[macro_export]
macro_rules! for_each_field {
    ($cb:ident) => {
        $cb! { q: Q, qd: Qd, tau: Tau, kp: Kp, kd: Kd }
    };
    ($cb:ident, $($pre:tt)*) => {
        $cb! { $($pre)*; q: Q, qd: Qd, tau: Tau, kp: Kp, kd: Kd }
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
        use crate::c::define_slot;
        use crate::view::define_field_types;
        crate::for_each_field!(plus_dummy, define_field_types);
        crate::for_each_field!(plus_dummy, define_slot);
    }

    #[test]
    fn one_line_adds_a_field_to_the_types() {
        let slot = with_dummy::Slot::<4>::default();
        let c_slot = with_dummy::CSlot::<4>::default();
        let buf = with_dummy::WriteBuf::<4>::default();
        let written = with_dummy::Written::default();
        assert_eq!(slot.dummy.len(), 4);
        assert_eq!(c_slot.dummy.len(), 4);
        assert_eq!(buf.dummy.len(), 4);
        assert_eq!(written.dummy, 0);
        assert_eq!(
            core::mem::size_of::<with_dummy::Written>(),
            core::mem::size_of::<crate::Written>() + 8
        );
    }

    #[test]
    fn one_line_adds_a_field_to_the_headers() {
        use crate::build::{FLAGS, Sizes, c_header_from, cpp_header_from, field_names};
        let fields = crate::for_each_field!(plus_dummy, field_names);
        let sizes = Sizes {
            history_depth: 4,
            max_imus: 4,
            max_mics: 1,
            mic_window: 16,
        };
        let c = c_header_from(4, sizes, fields, FLAGS);
        assert_eq!(c.matches("    float dummy[KMR_N];\n").count(), 2);
        assert!(c.contains("    uint64_t dummy;\n"));
        assert!(c.contains("static inline kmr_status kmr_set_dummy_at(kmr_state *s"));
        assert!(c.contains("static inline kmr_status kmr_group_set_all_dummy(kmr_state *s"));
        assert!(c.contains("    s->write.written.dummy = 0;\n"));
        let cpp = cpp_header_from(fields, FLAGS);
        assert!(cpp.contains("    std::optional<Values> dummy() const noexcept {"));
        assert!(cpp.contains("    Status set_dummy_at(float value, std::size_t i) noexcept"));
        assert!(cpp.contains("    Result<std::array<float, K>> prev_dummy(std::size_t depth)"));
    }
}
