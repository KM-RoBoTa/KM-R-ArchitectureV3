// ---------------------------------------------------------------------------
// Generates, kept in lockstep from one ordered `field => Label` list:
//   * the label unit structs + `Schedule` impls,
//   * `Scheduled<..>` — one bucket HList field per phase,
//   * `Drive` + its blanket impl — one method per phase, each walking that
//     bucket; the sole place the full bucket-param list is written out,
//   * one small `Insert<Label, Node>` impl per phase (schedule.rs holds the
//     trait): each names its single bucket, appends to it, and passes the
//     others through. The `@insert` recursion walks the field list, peeling one
//     field at a time as the "target" (with the already-seen fields as `before`
//     and the rest as `after`) so each generated impl can spell out all six
//     fields in declaration order.
// ---------------------------------------------------------------------------
macro_rules! schedules {
($($field:ident => $Label:ident),+ $(,)?) => {
    $(
        #[derive(Clone, Copy)]
        /// Schedule label that runs during [`crate::runtime`]
        pub struct $Label;
        impl Sealed for $Label {}
        impl Schedule for $Label {}
    )+

    /// One bucket per phase. Each type param is that bucket's element HList,
    /// defaulting to `HNil` (empty). Fields declared in execution order.
    pub struct Scheduled<$($field = HNil),+> {
        $(pub(crate) $field: $field),+
    }

    impl Scheduled {
        pub fn new() -> Self {
            Scheduled { $($field: HNil),+ }
        }
    }

    impl Default for Scheduled {
        fn default() -> Self {
            Self::new()
        }
    }

    // Drives the schedule: one method per phase, each walking that phase's
    // bucket via `RunPhase`. This is the bound `runtime` takes. The blanket impl
    // is the ONLY place the full bucket-param list is spelled — the macro writes
    // it, so adding a phase never means hand-writing `impl<A, B, C, …>`.
    // `#[doc(hidden)] pub(crate)`: internal walk machinery, not a user extension
    // point — no reason for it to be nameable or documented outside the crate.
    #[doc(hidden)]
    pub trait Drive {
        $( fn $field(&mut self, env: &mut Env<'_>); )+
    }

    #[doc(hidden)]
    impl<$($field: RunPhase),+> Drive for Scheduled<$($field),+> {
        $(
            fn $field(&mut self, env: &mut Env<'_>) {
                self.$field.run_phase(env);
            }
        )+
    }

    // Kick off the per-phase impl generation with an empty `before` set.
    schedules!(@insert [] [$($field => $Label,)+]);
};

// Recursion: `[before…]` are the fields already given an impl; the head of the
// second list is the current target, its tail is `after`. Emit one impl for the
// target, then move it into `before` and recurse.
(@insert
    [$($bf:ident => $BL:ident,)*]
    [$tf:ident => $TL:ident, $($af:ident => $AL:ident,)*]
) => {
    #[allow(non_camel_case_types)]
    impl<$($bf,)* $tf, $($af,)* Node> Insert<$TL, Node>
        for Scheduled<$($bf,)* $tf, $($af,)*>
    where
        $tf: HAppend<Node>,
    {
        type Output = Scheduled<$($bf,)* <$tf as HAppend<Node>>::Output, $($af,)*>;
        fn insert(self, node: Node) -> Self::Output {
            Scheduled {
                $($bf: self.$bf,)*
                $tf: self.$tf.append(node),
                $($af: self.$af,)*
            }
        }
    }

    schedules!(@insert [$($bf => $BL,)* $tf => $TL,] [$($af => $AL,)*]);
};

// Base case: no fields left to target.
(@insert [$($bf:ident => $BL:ident,)*] []) => {};
}
pub(crate) use schedules;
