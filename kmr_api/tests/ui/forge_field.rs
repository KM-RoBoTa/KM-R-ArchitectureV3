//! The field set is sealed: a user type cannot become a state field.

struct MyType;

impl kmr_api::Field for MyType {}

fn main() {}
