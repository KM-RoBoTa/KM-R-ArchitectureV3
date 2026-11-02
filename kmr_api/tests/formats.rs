//! One robot in URDF, `.ron` and JSON gives one model (SC-003 (SRS FR-6.1); quickstart § 4).

mod urdf {
    kmr_api::robot!(model = "tests/fixtures/four.urdf");
}

mod ron {
    kmr_api::robot!(model = "tests/fixtures/four.ron");
}

mod json {
    kmr_api::robot!(model = "tests/fixtures/four.json");
}

/// Every emitted item of module `$a` equals the one of `$b`.
macro_rules! same_model {
    ($a:ident, $b:ident) => {
        assert_eq!($a::N, $b::N);
        assert_eq!($a::model::NAMES, $b::model::NAMES);
        assert_eq!($a::model::LOWER, $b::model::LOWER);
        assert_eq!($a::model::UPPER, $b::model::UPPER);
        assert_eq!($a::model::SPEED, $b::model::SPEED);
        assert_eq!($a::model::EFFORT, $b::model::EFFORT);
        assert_eq!($a::model::LINK_NAMES, $b::model::LINK_NAMES);
        assert_eq!($a::model::LINK_PARENT, $b::model::LINK_PARENT);
        assert_eq!($a::model::LINK_MASS, $b::model::LINK_MASS);
        assert_eq!($a::model::LINK_COM, $b::model::LINK_COM);
    };
}

#[test]
fn ron_gives_the_model_of_the_urdf() {
    same_model!(urdf, ron);
}

#[test]
fn json_gives_the_model_of_the_urdf() {
    same_model!(urdf, json);
}
