//! Joint *groups*: an api-only view over a fixed set of joint indices.
//!
//! A group is **not** a slice and **not** a range — it is a fixed-size index
//! set `[usize; K]` with `K` baked in at the call site. Indices may be
//! non-contiguous and in any order (`[1, 3, 4]` is fine). Every group read or
//! write is therefore a fixed-size array operation: no slices, no allocation,
//! bare-metal friendly.
//!
//! * Reads **gather** — the per-index accessor ([`RobotState::q_at`],
//!   [`RobotState::prev_q_at`], …) is looped over the `K` indices into an owned
//!   `[T; K]`.
//! * Writes **scatter** — the per-index setter ([`RobotState::set_q_at`], …) is
//!   looped over the same indices. A group never calls `set_all` on the core:
//!   that overwrites the *whole* desired buffer and would clobber joints staged
//!   by other groups in the same tick. Per-index writes touch only this group.
//!
//! Groups are additive — code that never calls [`RobotState::group`] sees the
//! whole-robot API unchanged.
//!
//! ## Errors: `Option` vs `Result`
//!
//! The group surface inherits the whole-robot error vocabulary:
//! * Current reads ([`GroupView::q`], …) return `Option`, mirroring
//!   [`RobotState::q`]. `None` means the value is unavailable — normally "no
//!   sample yet" (a recoverable nothing), but **also** if a group index is out
//!   of range: an `Option` cannot carry the cause, so the two collapse to the
//!   same `None`. A stray index in a group const therefore reads as a permanent
//!   `None`; the `Result`-returning paths below distinguish it (see index
//!   hygiene).
//! * History reads ([`GroupView::prev_q`], …) and writes
//!   ([`GroupView::set_all_q`], …) return `Result`: the cause matters — a group
//!   index or history depth can be out of range — so it is propagated, never
//!   swallowed. (This is why a group setter returns `Result` where the
//!   whole-robot [`RobotState::set_all_q`] returns `()`: a group index, unlike
//!   a full `[T; N]`, is not structurally guaranteed to be valid.)
//!
//! ## Index hygiene
//!
//! Group indices are validated **lazily**, per call — there is no startup
//! registration of groups. Two consequences worth knowing:
//!
//! * **Writes are atomic.** Each `set_all_*` validates every index *before*
//!   staging any value, so an out-of-range index returns `Err` with nothing
//!   written — never a half-applied group command. (A whole-robot `set_all_*`
//!   cannot half-apply; the per-index group scatter could, hence the up-front
//!   check.)
//! * **Duplicates are not de-duplicated.** `[1, 3, 3]` gathers joint 3 twice
//!   and, on a write, scatters to it twice (last value wins). Harmless for
//!   reads, merely redundant for writes *within* one group; cross-group overlap
//!   in a single tick is the job of the deferred per-tick write-mask, not this
//!   layer.

use crate::{Q, Qd, State, StateError, Tau};

impl State {
    /// View a fixed set of joint `indices` as a [`GroupView`]. Indices may be
    /// non-contiguous and in any order; reads and writes preserve that order.
    ///
    /// ```ignore
    /// const R_ARM: [usize; 3] = [3, 4, 5];
    /// let mut arm = robot.group(R_ARM);
    /// let desired = arm.q().unwrap().map(|q| q + Q(0.1));
    /// arm.set_all_q(desired).unwrap(); // touches only joints 3, 4, 5
    /// ```
    #[inline]
    pub fn group<const K: usize>(&mut self, indices: [usize; K]) -> GroupView<'_, K> {
        GroupView {
            robot: self,
            indices,
        }
    }
}

/// A view over `K` chosen joints of a [`RobotState`], created by
/// [`RobotState::group`].
///
/// Mirrors the whole-robot accessors, but every array is sized to the group
/// (`K`) instead of the whole robot ([`crate::N`]), and each operation gathers
/// or scatters per index — so a group write never disturbs joints outside the
/// group.
pub struct GroupView<'a, const K: usize> {
    robot: &'a mut State,
    indices: [usize; K],
}

/// Gather `[T; K]` from a per-index reader, short-circuiting on the first index
/// that yields `None`. All-or-nothing, no wasted reads, no allocation.
#[inline]
fn gather<T, const K: usize>(
    indices: [usize; K],
    mut at: impl FnMut(usize) -> Option<T>,
) -> Option<[T; K]> {
    // Stable has no fallible array initializer, so stage into `Option`s and
    // `?`-bail on the first miss; the move-out below then cannot panic.
    let mut staged: [Option<T>; K] = std::array::from_fn(|_| None);
    for (slot, index) in staged.iter_mut().zip(indices) {
        *slot = Some(at(index)?);
    }
    Some(staged.map(|slot| slot.unwrap()))
}

/// Gather `[T; K]` from a fallible per-index reader, short-circuiting on — and
/// returning, by move (never clone) — the first index that errors.
#[inline]
fn try_gather<T, const K: usize>(
    indices: [usize; K],
    mut at: impl FnMut(usize) -> Result<T, StateError>,
) -> Result<[T; K], StateError> {
    let mut staged: [Option<T>; K] = std::array::from_fn(|_| None);
    for (slot, index) in staged.iter_mut().zip(indices) {
        *slot = Some(at(index)?); // `?` moves the first error out — no clone
    }
    Ok(staged.map(|slot| slot.unwrap()))
}

impl<'a, const K: usize> GroupView<'a, K> {
    /// Validate every group index against the joint count ([`crate::N`], which
    /// equals `kmr_core`'s `JOINTS`), returning the first that is out of range.
    ///
    /// This is the *sole* reason the `set_all_*` scatter is atomic — and it is
    /// only sufficient because `kmr_core::RobotState::set_at` fails
    /// **exclusively** on out-of-range (verified: its one `Err` is `OutOfRange`
    /// when `index >= JOINTS`; there is no other failure path). If a future
    /// core change lets `set_at` fail for another reason, this up-front pass no
    /// longer covers it and the "atomic" guarantee on `set_all_*` silently
    /// breaks — revisit here, and the `N == JOINTS` assumption, if the core
    /// gains a runtime joint length.
    #[inline]
    fn check_indices(&self) -> Result<(), StateError> {
        match self.indices.into_iter().find(|&index| index >= crate::N) {
            Some(index) => Err(StateError::OutOfRange {
                index,
                len: crate::N,
            }),
            None => Ok(()),
        }
    }

    // ---- Q (position) ----

    /// Newest sensed `Q` for each group joint. `None` if a value is
    /// unavailable — no sample yet, or an index out of range (an `Option`
    /// cannot carry the cause; see module docs). Mirrors [`RobotState::q`].
    #[inline]
    pub fn q(&self) -> Option<[Q; K]> {
        gather(self.indices, |i| self.robot.q_at(i).ok())
    }

    /// Alias of [`GroupView::q`].
    #[inline]
    pub fn position(&self) -> Option<[Q; K]> {
        self.q()
    }

    /// `Q` for each group joint, `depth` steps back (`0` = newest). Propagates
    /// the first out-of-range index or history depth.
    #[inline]
    pub fn prev_q(&self, depth: usize) -> Result<[Q; K], StateError> {
        try_gather(self.indices, |i| self.robot.prev_q_at(i, depth))
    }

    /// Stage `Q` for each group joint, scattering one value per index. Joints
    /// outside the group are untouched.
    ///
    /// Atomic: all indices are validated before any value is staged, so an
    /// out-of-range index returns `Err` with nothing written.
    #[inline]
    pub fn set_all_q(&mut self, values: [Q; K]) -> Result<(), StateError> {
        self.check_indices()?;
        values
            .into_iter()
            .zip(self.indices)
            .try_for_each(|(value, index)| self.robot.set_q_at(value, index))
    }

    // ---- Qd (velocity) ----

    /// Newest sensed `Qd` for each group joint. `None` if a value is
    /// unavailable — no sample yet, or an index out of range (an `Option`
    /// cannot carry the cause; see module docs). Mirrors [`RobotState::qd`].
    #[inline]
    pub fn qd(&self) -> Option<[Qd; K]> {
        gather(self.indices, |i| self.robot.qd_at(i).ok())
    }

    /// Alias of [`GroupView::qd`].
    #[inline]
    pub fn velocity(&self) -> Option<[Qd; K]> {
        self.qd()
    }

    /// `Qd` for each group joint, `depth` steps back (`0` = newest). Propagates
    /// the first out-of-range index or history depth.
    #[inline]
    pub fn prev_qd(&self, depth: usize) -> Result<[Qd; K], StateError> {
        try_gather(self.indices, |i| self.robot.prev_qd_at(i, depth))
    }

    /// Stage `Qd` for each group joint, scattering one value per index. Joints
    /// outside the group are untouched.
    ///
    /// Atomic: all indices are validated before any value is staged, so an
    /// out-of-range index returns `Err` with nothing written.
    #[inline]
    pub fn set_all_qd(&mut self, values: [Qd; K]) -> Result<(), StateError> {
        self.check_indices()?;
        values
            .into_iter()
            .zip(self.indices)
            .try_for_each(|(value, index)| self.robot.set_qd_at(value, index))
    }

    // ---- Tau (torque / effort) ----

    /// Newest sensed `Tau` for each group joint. `None` if a value is
    /// unavailable — no sample yet, or an index out of range (an `Option`
    /// cannot carry the cause; see module docs). Mirrors [`RobotState::tau`].
    #[inline]
    pub fn tau(&self) -> Option<[Tau; K]> {
        gather(self.indices, |i| self.robot.tau_at(i).ok())
    }

    /// Alias of [`GroupView::tau`].
    #[inline]
    pub fn torque(&self) -> Option<[Tau; K]> {
        self.tau()
    }

    /// Alias of [`GroupView::tau`].
    #[inline]
    pub fn effort(&self) -> Option<[Tau; K]> {
        self.tau()
    }

    /// `Tau` for each group joint, `depth` steps back (`0` = newest). Propagates
    /// the first out-of-range index or history depth.
    #[inline]
    pub fn prev_tau(&self, depth: usize) -> Result<[Tau; K], StateError> {
        try_gather(self.indices, |i| self.robot.prev_tau_at(i, depth))
    }

    /// Stage `Tau` for each group joint, scattering one value per index. Joints
    /// outside the group are untouched.
    ///
    /// Atomic: all indices are validated before any value is staged, so an
    /// out-of-range index returns `Err` with nothing written.
    #[inline]
    pub fn set_all_tau(&mut self, values: [Tau; K]) -> Result<(), StateError> {
        self.check_indices()?;
        values
            .into_iter()
            .zip(self.indices)
            .try_for_each(|(value, index)| self.robot.set_tau_at(value, index))
    }
}

#[cfg(test)]
mod test;
