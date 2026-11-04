//! A preset and its descriptor give the same model (US7 AS1; SC-010 (SRS FR-6.1)).

mod by_name {
    kmr_api::robot!(XmBench);
}

mod by_path {
    kmr_api::robot!(model = "../kmr_model_core/presets/xm_bench.urdf");
}

#[test]
fn xm_bench_by_name_equals_its_descriptor() {
    assert_eq!(by_name::N, by_path::N);
    assert_eq!(by_name::model::NAMES, by_path::model::NAMES);
    assert_eq!(by_name::model::LOWER, by_path::model::LOWER);
    assert_eq!(by_name::model::UPPER, by_path::model::UPPER);
    assert_eq!(by_name::model::SPEED, by_path::model::SPEED);
    assert_eq!(by_name::model::EFFORT, by_path::model::EFFORT);
    assert_eq!(by_name::model::LINK_NAMES, by_path::model::LINK_NAMES);
    assert_eq!(by_name::model::LINK_PARENT, by_path::model::LINK_PARENT);
    assert_eq!(by_name::model::LINK_MASS, by_path::model::LINK_MASS);
    assert_eq!(by_name::model::LINK_COM, by_path::model::LINK_COM);
}

/// The declared costs of a preset are reachable through `kmr_api` alone
/// (FR-035 (SRS NFR-11, FR-5.3); plan C7).
#[test]
fn xm_bench_declared_costs_through_kmr_api() {
    use core::time::Duration;
    use kmr_api::presets::XmBench;

    assert_eq!(XmBench::READ_COST, Duration::from_micros(2_260));
    assert_eq!(XmBench::WRITE_COST, Duration::from_micros(340));
}
