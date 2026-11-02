//! A `#![no_std]` caller of the model: the emitted facts need no `std` or
//! `alloc` (FR-019, SRS NFR-1; SC-006, SRS NFR-7).

#![no_std]

kmr_model::model!(kmr_api; model = "tests/fixtures/four.urdf");

#[test]
fn facts_are_read_without_std() {
    assert_eq!(N, 4);
    assert_eq!(model::NAMES[0], "j1");
    assert_eq!(model::LOWER[0], kmr_api::Q(-1.57));
}
