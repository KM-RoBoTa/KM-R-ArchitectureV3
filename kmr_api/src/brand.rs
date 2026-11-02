//! The robot's hardware as `robot!` names it: one member per brand, each bound to the bus
//! plugins whose label names one of its buses (SRS FR-5.1, FR-6.3, FR-6.4).

use std::any::type_name;

use kmr_adapters::binding::{Binding, Port};
use kmr_core::hal::{Drivers, Hal, HardwareError, MemberError};
use kmr_core::{HCons, HNil};

use crate::UserError;

/// Why a member's description did not bind.
#[doc(hidden)]
#[derive(Debug)]
pub enum Refused<E> {
    /// Its adapter is not compiled in: kmr_api lacks this feature.
    FeatureOff(&'static str),
    /// Its adapter refused the plugins.
    Member(E),
}

/// One brand's part of the robot's hardware, binding to its member, no I/O.
///
/// `robot!` holds one per brand; users never implement it.
#[doc(hidden)]
pub trait Brand {
    /// The member it binds to. Its error crosses threads inside [`UserError::BringUp`].
    type Member: Hal<Error: Send + Sync>;

    /// Labels of its buses.
    fn labels(&self) -> Vec<&'static str>;

    /// Binds `plugins`, those routed to this brand.
    ///
    /// # Errors
    ///
    /// [`Refused::FeatureOff`], or the member's binding error.
    fn bind(
        self,
        plugins: &[Binding],
    ) -> Result<Self::Member, Refused<<Self::Member as Hal>::Error>>;
}

mod seal {
    /// Implemented for `HNil` and `HCons` only.
    pub trait Brands {}
}

/// The robot's hardware description, `HCons<A, HCons<B, HNil>>` of [`Brand`]s, in the order
/// the members are walked.
#[doc(hidden)]
pub trait Brands: seal::Brands {
    /// The members it binds to, in the same order.
    type Drivers: Drivers;

    /// Brands in the list.
    const LEN: usize;

    /// Labels of every bus, member by member.
    fn labels(&self) -> Vec<&'static str>;

    /// Binds each brand to the plugins `route` hands it.
    ///
    /// # Errors
    ///
    /// The first brand's refusal, wrapped to name its position.
    fn bind(
        self,
        plugins: &[Binding],
        route: &Route,
    ) -> Result<Self::Drivers, Refused<<Self::Drivers as Drivers>::Error>>;
}

impl seal::Brands for HNil {}

impl Brands for HNil {
    type Drivers = HNil;

    const LEN: usize = 0;

    fn labels(&self) -> Vec<&'static str> {
        Vec::new()
    }

    fn bind(
        self,
        _plugins: &[Binding],
        _route: &Route,
    ) -> Result<HNil, Refused<core::convert::Infallible>> {
        Ok(HNil)
    }
}

impl<B: Brand, T: Brands> seal::Brands for HCons<B, T> {}

impl<B: Brand, T: Brands> Brands for HCons<B, T> {
    type Drivers = HCons<B::Member, T::Drivers>;

    const LEN: usize = 1 + T::LEN;

    fn labels(&self) -> Vec<&'static str> {
        let mut labels = self.head.labels();
        labels.extend(self.tail.labels());
        labels
    }

    fn bind(
        self,
        plugins: &[Binding],
        route: &Route,
    ) -> Result<Self::Drivers, Refused<<Self::Drivers as Drivers>::Error>> {
        let labels = self.head.labels();
        let mine: Vec<Binding> = plugins
            .iter()
            .filter(|plugin| route.takes(&labels, plugin))
            .copied()
            .collect();
        let head = self.head.bind(&mine).map_err(|refused| match refused {
            Refused::FeatureOff(feature) => Refused::FeatureOff(feature),
            Refused::Member(error) => Refused::Member(MemberError::Head {
                name: type_name::<B::Member>(),
                error,
            }),
        })?;
        let tail = self
            .tail
            .bind(plugins, route)
            .map_err(|refused| match refused {
                Refused::FeatureOff(feature) => Refused::FeatureOff(feature),
                Refused::Member(error) => Refused::Member(MemberError::Tail(error)),
            })?;
        Ok(HCons { head, tail })
    }
}

/// How plugins reach the brands: by the label of the bus they name.
#[doc(hidden)]
#[derive(Debug)]
pub struct Route {
    /// One brand only: it takes every plugin and refuses the ones it cannot bind itself.
    alone: bool,
    /// The robot has one bus: a plugin without a label binds it.
    one_bus: bool,
}

impl Route {
    /// Whether the brand of buses `labels` takes `plugin`.
    fn takes(&self, labels: &[&'static str], plugin: &Binding) -> bool {
        match plugin.bus {
            _ if self.alone => true,
            Some(label) => labels.contains(&label),
            None => self.one_bus && !labels.is_empty(),
        }
    }
}

/// The serial id or the path, as written.
fn port(plugin: &Binding) -> &'static str {
    match plugin.port {
        Port::ById(port) | Port::ByDev(port) => port,
    }
}

/// Binds `plugins` to the brands of `hardware`, no I/O: each plugin goes to the brand whose
/// bus it names. A robot of one brand hands that brand every plugin, so its own binding
/// errors name a wrong one.
///
/// # Errors
///
/// Over several brands, [`UserError::SharedBusLabel`], [`UserError::UnknownBus`] and
/// [`UserError::LabelRequired`]; then
/// [`UserError::FeatureOff`], or a member's binding error as [`UserError::BringUp`], named by
/// its position and type, as a bring-up error would be.
#[doc(hidden)]
pub fn __bind<H>(hardware: H, plugins: &[Binding]) -> Result<H::Drivers, UserError>
where
    H: Brands,
    <H::Drivers as Drivers>::Error: Send + Sync,
{
    let labels = hardware.labels();
    let route = Route {
        alone: H::LEN == 1,
        one_bus: labels.len() == 1,
    };
    if !route.alone {
        let shared = labels
            .iter()
            .enumerate()
            .find(|&(at, label)| labels[..at].contains(label));
        if let Some((_, &label)) = shared {
            return Err(UserError::SharedBusLabel { label });
        }
        for plugin in plugins {
            match plugin.bus {
                Some(label) if !labels.contains(&label) => {
                    return Err(UserError::UnknownBus {
                        label,
                        port: port(plugin),
                    });
                }
                None if !route.one_bus => {
                    return Err(UserError::LabelRequired { port: port(plugin) });
                }
                _ => {}
            }
        }
    }
    hardware
        .bind(plugins, &route)
        .map_err(|refused| match refused {
            Refused::FeatureOff(feature) => UserError::FeatureOff { feature },
            Refused::Member(error) => UserError::bring_up(HardwareError::<
                <H::Drivers as Drivers>::Error,
            >::Member(error)),
        })
}
