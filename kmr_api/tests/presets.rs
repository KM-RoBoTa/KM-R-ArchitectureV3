//! A preset and its descriptor give the same model (US7 AS1; SC-010 (SRS FR-6.1)).

mod by_name {
    kmr_api::robot!(XmBench);
}

mod by_path {
    kmr_api::robot!(model = "../kmr_model/presets/xm_bench.urdf");
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
