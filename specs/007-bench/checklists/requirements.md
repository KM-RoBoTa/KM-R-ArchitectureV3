# Specification Quality Checklist: bench, the reference benchmark of kmr v3

**Purpose**: Validate specification completeness and quality before proceeding to planning
**Created**: 2026-10-27
**Updated**: 2026-10-28 (re-specified against the SRS patch of 2026-10-27)
**Feature**: [spec.md](../spec.md)

## Content Quality

- [x] No implementation details (languages, frameworks, APIs)
- [x] Focused on user value and business needs
- [x] Written for non-technical stakeholders
- [x] All mandatory sections completed

## Requirement Completeness

- [ ] No [NEEDS CLARIFICATION] markers remain
- [x] Requirements are testable and unambiguous
- [x] Success criteria are measurable
- [x] Success criteria are technology-agnostic (no implementation details)
- [x] All acceptance scenarios are defined
- [x] Edge cases are identified
- [x] Scope is clearly bounded
- [x] Dependencies and assumptions identified

## Feature Readiness

- [x] All functional requirements have clear acceptance criteria
- [x] User scenarios cover primary flows
- [x] Feature meets measurable outcomes defined in Success Criteria
- [x] No implementation details leak into specification

## Notes

- Coordination 2026-10-28: FR-006 aligned with kmr_core FR-042 (one "no bus plugin: simulating"
  warning) and kmr_api FR-026 ("not driven yet" message until delivery step 1); FR-027 no longer
  freezes the example; User Story 5 and boundary citations fixed. Markers unchanged.
- Two markers remain, both the human's: FR-015 (which actuator chain is the reference; SRS NFR-1
  names only the host) and FR-016 (how a black-box bench shows the pre-sort gain SRS FR-5.3
  sends to it). Run `/speckit-clarify` before `/speckit-plan`.
- Resolved from the SRS since the last version: engine overhead uses the budget method, from
  NFR-11's "remaining budget of each tick" (old FR-007 marker); the bench records only and never
  gates, since NFR-1 defers every figure (old FR-013 marker); reference compile-time figures come
  from the RP4, per section 8 ("hardware always the RP4 setup").
- Updated to the 2026-10-27 SRS patch: the robot is named with `kmr_api::robot!` (FR-6.1); ports
  bind with `.add_plugins(Bus::by_id(..))` (FR-6.3); no plugin means simulate and warn once, which
  the engine-only runs use (FR-006); the read cadence is declared by the adapter, not set by the
  bench. `NoHardware`, the `dynamixel` feature and `Bus::register::by_id!` are gone from the spec.
- Gaps from specs/spec-verification-report.md addressed: run length and hold window, jitter
  definition, load profile, kernel setup, instruments, stack thread (Assumptions); read ticks
  apart (FR-008); other modules' bench expectations listed per module (Module boundaries).
- Names like `kmr_api`, `robot!`, `Bus::by_id`, stress-ng and PREEMPT_RT appear on purpose. They
  come from the SRS itself or name the module boundary. They are not design choices.
- SC-002 and SC-003 (repeatability) and the run lengths are informed defaults, listed under
  Assumptions. They judge the bench, not kmr: no target figure for kmr is set.
