//! An override naming an actuator past the table does not build: the error names the index.

kmr_api::robot!(XmBench, hardware = |hw| hw.read_only(7));

fn main() {}
