// Generic params below reuse the snake_case bucket-field names as type-param
// idents (kept in lockstep by the `schedules!` macro), so allow the lint.
#![allow(non_camel_case_types)]

use frunk::hlist::HList;
use frunk::{HCons, HNil};

use crate::sealed::Sealed;

// ===========================================================================
// Bevy-style fixed pipeline, PRE-SORTED at build time — no per-tick scanning.
//
// Every controller is tagged with a zero-sized label (`add_controller(EachTick,
// f)`). Routing happens ONCE, when the controller is added: `Insert<Label, Node>`
// prepends the node into the ONE bucket that label owns, growing only that
// bucket's HList type. `Schedules` is a struct with one bucket HList per phase.
//
// Per tick the runtime walks each bucket exactly once, in the fixed order the
// fields are declared — every controller visited once, ZERO phase compares,
// ZERO filtering. Startup buckets are walked once, on the first tick only.
//
// Fully static: label is the compile-time routing key, buckets are concrete
// HLists, no `dyn`, no fat pointer. `Phase` below is introspection only (debug /
// letting the runtime tell a startup bucket from a per-tick one) — never touched
// for dispatch.
// ===========================================================================

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Phase {
    // Startup group — walked ONCE, on the first tick only.
    PreInit,  // or Startup | Mostly for build time gen
    Init,     // Mostly for settings
    PostInit, // mostly for post-processing of settings if needed.

    // during ticks

    // Per-tick group — walked EVERY tick, in this order.
    First,           // Initialization that must be done every tick
    EachTick,        // Every tick
    StateTransition, // Not sure if needed. possibly if user want to change with its own
    // tracking system the current goal/mode of the robot like "walking" to "running" to "swimming"
    // RunFixedMainLoop, // never, its dangerous. remove it
    Last, // last thing to do each tick
}

impl Phase {
    pub const fn is_startup(self) -> bool {
        matches!(self, Phase::PreInit | Phase::Init | Phase::PostInit)
    }
}

/// Sealed label trait. Users pick from the labels below; they can neither add a
/// phase nor forge one. Carries only its phase tag (introspection, not dispatch).
pub trait Schedule: Sealed + Copy {
    const PHASE: Phase;
}

/// Routes `Node` into the bucket owned by label `L`, growing only that bucket.
/// The label `L` is the compile-time routing key; `add_controller` names
/// `<Schedules as Insert<L, Node>>::Output` as its return type.
pub trait Insert<L: Schedule, Node> {
    type Output;
    fn insert(self, node: Node) -> Self::Output;
}

// ---------------------------------------------------------------------------
// Generates, kept in lockstep from one ordered `field => Label` list:
//   * the label unit structs + `Schedule` impls,
//   * `Schedules<..>` — one bucket HList field per phase,
//   * one `Insert<Label, Node>` impl per phase (grows only that field).
// ---------------------------------------------------------------------------
macro_rules! schedules {
    ($($field:ident => $Label:ident),+ $(,)?) => {
        $(
            #[derive(Clone, Copy)]
            pub struct $Label;
            impl Sealed for $Label {}
            impl Schedule for $Label {
                const PHASE: Phase = Phase::$Label;
            }
        )+

        /// One bucket per phase. Each type param is that bucket's element HList,
        /// defaulting to `HNil` (empty). Fields declared in execution order.
        pub struct Schedules<$($field = HNil),+> {
            $(pub(crate) $field: $field),+
        }

        impl Schedules {
            pub fn new() -> Self {
                Schedules { $($field: HNil),+ }
            }
        }

        impl Default for Schedules {
            fn default() -> Self {
                Self::new()
            }
        }

        schedules!(@routes [] [$($field => $Label),+]);
    };

    // Walk the ordered list, carrying already-seen phases in the accumulator so
    // each `@one` knows the buckets BEFORE and AFTER its target.
    (@routes [$($seen:tt)*] []) => {};
    (@routes [$($seen:tt)*] [$field:ident => $Label:ident $(, $rf:ident => $rl:ident)*]) => {
        schedules!(@one [$($seen)*] ($field => $Label) [$($rf => $rl),*]);
        schedules!(@routes [$($seen)* $field => $Label ,] [$($rf => $rl),*]);
    };

    (@one
        [$($bf:ident => $bl:ident),* $(,)?]
        ($tf:ident => $TL:ident)
        [$($af:ident => $al:ident),* $(,)?]
    ) => {
        impl<$($bf,)* $tf: HList, $($af,)* Node> Insert<$TL, Node>
            for Schedules<$($bf,)* $tf, $($af,)*>
        {
            // Only the target bucket's type changes: one more `HCons` layer.
            type Output = Schedules<$($bf,)* HCons<Node, $tf>, $($af,)*>;
            fn insert(self, node: Node) -> Self::Output {
                Schedules {
                    $($bf: self.$bf,)*
                    $tf: self.$tf.prepend(node),
                    $($af: self.$af,)*
                }
            }
        }
    };
}

schedules! {
    pre_init         => PreInit,
    init             => Init,
    post_init        => PostInit,
    first            => First,
    each_tick        => EachTick,
    state_transition => StateTransition,
    last             => Last,
}
