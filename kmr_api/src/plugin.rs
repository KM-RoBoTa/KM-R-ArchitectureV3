//! Plugins: what [`Robot::add_plugins`](crate::Robot::add_plugins) takes (SRS FR-6.3, FR-7.1).

use crate::Bus;

/// What [`Robot::add_plugins`](crate::Robot::add_plugins) takes: a [`Bus`], or with the
/// `zenoh` feature the `Gateway`. Sealed: no other type is a plugin.
pub trait Plugin: seal::Plug {}

/// The seal of [`Plugin`] and where a plugin goes.
pub(crate) mod seal {
    /// Puts a plugin with the builder's others.
    pub trait Plug {
        /// Adds `self` to `plugins`.
        fn plug(self, plugins: &mut super::Plugins);
    }
}

/// The plugins a builder collected, in call order per kind.
#[derive(Default)]
pub struct Plugins {
    /// The bus plugins.
    pub(crate) buses: Vec<Bus>,
    /// The gateways; more than one is refused at finalization.
    #[cfg(feature = "zenoh")]
    pub(crate) gateways: Vec<crate::zenoh::Gateway>,
}

impl seal::Plug for Bus {
    fn plug(self, plugins: &mut Plugins) {
        plugins.buses.push(self);
    }
}

impl Plugin for Bus {}
