// ===========================================================================
// This module holds ALL the machinery: the config, the heterogeneous list of
// controllers, and the `Robot<L>` builder that owns them. `kmr_api` only wraps
// `Robot<L>` in a newtype, so everything here is the protected IP; the public
// crate adds no logic of its own.
// ===========================================================================

// ===========================================================================
// (A) The HList — provided by `frunk`. Knows NOTHING about robots.
//
//   HNil                        empty list
//   HCons<H, T>                 one element `H` on top of a tail list `T`
//
// We re-export these so `kmr_api` can NAME them in its return types without a
// direct `frunk` dependency. The list SHAPE being public is fine: the IP lives
// in the controller node types (`Plain`/`With`/`AsThread`) below, whose fields
// are `pub(crate)` and whose constructors are `#[doc(hidden)]` — a user can
// hold an `HCons` but can neither forge a node to put in one nor read one out.
// ===========================================================================

pub use frunk::hlist::HList;
pub use frunk::{HCons, HNil};

use crate::config::user_config::RobotConfig;
use crate::controllers::{Inline, InlineWith, Threaded};
use crate::error::ApiError;
use crate::schedule::Schedule;

// ===========================================================================
// (B) The controller node types — the elements stored in the HList.
//
// Each is an opaque, generic container. The core stores the user's schedule
// `S`, optional context `T`, and function `F`, WITHOUT ever naming the api-side
// `Time`/`State`/`Sensors` (that would make `kmr_core` depend on `kmr_api` — a
// cycle). `kmr_api` instantiates each node with a concrete controller `F`.
//
// Fields are `pub(crate)`, construction is `#[doc(hidden)]`. User code only
// ever sees `kmr_api`, so it can neither forge a node nor read one.
// ===========================================================================

// ===========================================================================
// (D) The builder. `Robot` HAS-A list `L` (a field) — it is not the list.
//
// `L` starts as `HNil` and each `add_controller*` prepends a node, growing the
// TYPE by one `HCons` layer. Settings keep `L` unchanged.
// ===========================================================================

pub struct Robot<L: HList = HNil> {
    config: RobotConfig,
    list: L,
}

impl Robot<HNil> {
    pub fn new() -> Self {
        Robot {
            config: RobotConfig::default(),
            list: HNil,
        }
    }
}

impl Default for Robot<HNil> {
    fn default() -> Self {
        Self::new()
    }
}

impl<L: HList> Robot<L> {
    // ---- time: the only `Time` knob exposed to the user. Everything else on
    // `Time` is `pub(crate)` — written/read by the runtime only. ----

    pub fn set_dt_ms(mut self, ms: u32) -> Self {
        self.config.time.set_delta_ms(ms);
        self
    }

    pub fn set_dt_us(mut self, us: u32) -> Self {
        self.config.time.set_delta_us(us);
        self
    }

    // ---- controllers: grow the list by one `HCons` ----
    //
    // Deliberately generic over `S`/`T`/`F` with NO trait bounds that name
    // api types. `kmr_api` applies its own bounds (`FakeSchedule`, `Send`) and
    // pins `F` to a concrete controller signature at its boundary.

    // pub fn config(self) -> &RobotConfig {
    //     &self.config
    // }

    pub fn add_controller<S: Schedule, F>(
        self,
        schedule: S,
        f: F,
    ) -> Robot<HCons<Inline<S, F>, L>> {
        Robot {
            config: self.config,
            list: self.list.prepend(Inline::new(schedule, f)),
        }
    }

    pub fn add_controller_with<S: Schedule, T, F>(
        self,
        schedule: S,
        ctx: T,
        f: F,
    ) -> Robot<HCons<InlineWith<S, T, F>, L>> {
        Robot {
            config: self.config,
            list: self.list.prepend(InlineWith::new(schedule, ctx, f)),
        }
    }

    pub fn add_controller_as_thread<S: Schedule, T, F>(
        self,
        schedule: S,
        ctx: T,
        f: F,
    ) -> Robot<HCons<Threaded<S, T, F>, L>> {
        Robot {
            config: self.config,
            list: self.list.prepend(Threaded::new(schedule, ctx, f)),
        }
    }

    // Runtime check: `Robot<HNil>` has `L::LEN == 0`, panics with a plain
    // message instead of a compile-time trait-bound gate — the compiled rlib
    // otherwise spills internal paths/types (`frunk_core::hlist::HCons`,
    // `kmr_core`'s own file layout) into E0277 diagnostics at the call site.
    pub fn run(self) -> Result<(), ApiError> {
        if L::LEN == 0 {
            return Err(ApiError::NoControllers);
        }

        let Robot { config, list } = self;
        let _ = (config, list);
        // TODO: hand `config` + the `list` (walked via a `Tick` impl over the
        // HList) to `crate::runtime`. The tick-walk touches api types, so it is
        // driven from `kmr_api`'s side or through a core-generic environment —
        // see runtime.rs.
        todo!("drive config + controller list through the runtime");
        Ok(())
    }
}
