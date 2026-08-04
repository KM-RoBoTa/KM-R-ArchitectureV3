#[derive(Default)]
enum PayloadMode {
    #[default]
    Light,
    Medium,
    Heavy,
}

// #[derive(Default)]
// struct TreePayload {
//     flag: bool,
//     from_joint: u16, // canonical joint id from which the tree payload is extended
// }
//
// impl TreePayload {
//     // Assums that if new() is called.. then flag is true by default.
//     fn new(flag: bool, from_joint: u16) -> Self {
//         Self { flag, from_joint }
//     }
// }

#[derive(Default)]
pub struct Payload {
    // TODO: force the user to specify all the architecture needs
    // tree_payload: TreePayload,
    mode: PayloadMode,
}

impl Payload {
    // fn is_tree(&self) -> bool {
    // self.tree_payload.flag
    // }

    pub fn as_tree(self) -> Self {
        // TODO: The user specifies from where this new payload is registered
        // from.
        // e.g: the user adds a new arm that extends from the left knee of the
        // precompiled model.

        todo!("");
    }

    // payload reference: https://dev.bostondynamics.com/docs/payload/readme
    //
    // These must define which trait will the payload impl
    pub fn as_light_payload(self) -> Self {
        // TODO: The user specified a new mode:
        // Light — Attaching an inert payload to the robot without connecting to the robot’s [hardware interface] or network.
        todo!("");
    }
    pub fn as_medium_payload(self) -> Self {
        // TODO: The user specified a new mode:
        // Medium — The payload connects to a [hardware interface] and uses
        // [the state SoA] provided by the robot.
        todo!("");
    }
    pub fn as_heavy_payload(self) -> Self {
        // TODO: The user specified a new mode:
        // Heavy — A payload that registers and provides standard services that
        // integrate with other components of the robot system, such as the tablet
        // driving interface, or other payloads.
        todo!("");
    }
}
