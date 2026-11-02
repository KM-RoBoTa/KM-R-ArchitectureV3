//! Payloads registered through the builder (SRS FR-8.1, FR-8.4). Tiers follow Boston Dynamics'
//! Spot classification; a light payload is not registered, so there is no light tier.

/// How a payload integrates with the robot (SRS section 2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tier {
    /// Connects to a hardware interface and uses the robot's state.
    Medium,
    /// Also registers services that other parts of the robot system use.
    Heavy,
}

/// A payload: its tier, its mass and the model link it is mounted on.
///
/// `run()` checks the mount against the model's links before any I/O (SRS FR-6.4).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Payload {
    tier: Tier,
    mass_kg: f32,
    mount: &'static str,
}

impl Payload {
    /// A medium payload of `mass_kg`, mounted on the model link `mount`.
    pub const fn medium(mass_kg: f32, mount: &'static str) -> Self {
        Self {
            tier: Tier::Medium,
            mass_kg,
            mount,
        }
    }

    /// A heavy payload of `mass_kg`, mounted on the model link `mount`.
    pub const fn heavy(mass_kg: f32, mount: &'static str) -> Self {
        Self {
            tier: Tier::Heavy,
            mass_kg,
            mount,
        }
    }

    /// Its tier.
    pub const fn tier(&self) -> Tier {
        self.tier
    }

    /// Its mass, in kg.
    pub const fn mass_kg(&self) -> f32 {
        self.mass_kg
    }

    /// The model link it is mounted on.
    pub const fn mount(&self) -> &'static str {
        self.mount
    }
}

/// The robot's center of mass with `payloads` on (SRS FR-8.3, roadmap). Finalization does not
/// call it in this version.
#[expect(
    dead_code,
    reason = "SRS FR-8.3 is roadmap: nothing consumes a center of mass yet"
)]
pub(crate) fn center_of_mass(_payloads: &[Payload]) -> [f32; 3] {
    unimplemented!("SRS FR-8.3: center of mass")
}
