//! The FFI apply step generated from a field list that leaves `Kd` out does not build: the
//! `match` on the core's `FieldId` is exhaustive (kmr_ffi T028, SC-004).

kmr_api::__ffi_apply! { q: Q, qd: Qd, tau: Tau, kp: Kp }

fn main() {}
