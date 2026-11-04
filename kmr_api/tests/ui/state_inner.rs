//! The core state behind a `State` is private, under its name and as a tuple field.

fn named(state: kmr_api::State<'_, 4>) {
    let _core = state.inner;
}

fn tuple(state: kmr_api::State<'_, 4>) {
    let _core = state.0;
}

fn main() {}
