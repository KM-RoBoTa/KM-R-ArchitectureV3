# Specification Quality Checklist: kmr_api, the public user API

**Purpose**: Validate specification completeness and quality before proceeding to planning
**Created**: 2026-10-27
**Revised**: 2026-10-28 (SRS patch of 2026-10-27)
**Feature**: [spec.md](../spec.md)

## Content Quality

- [x] No implementation details (languages, frameworks, APIs) — see note 1
- [x] Focused on user value and business needs
- [x] Written for non-technical stakeholders — see note 1
- [x] All mandatory sections completed

## Requirement Completeness

- [ ] No [NEEDS CLARIFICATION] markers remain — 2 left, for the human (note 2)
- [x] Requirements are testable and unambiguous
- [x] Success criteria are measurable
- [x] Success criteria are technology-agnostic (no implementation details) — see note 1
- [x] All acceptance scenarios are defined
- [x] Edge cases are identified
- [x] Scope is clearly bounded
- [x] Dependencies and assumptions identified

## Feature Readiness

- [x] All functional requirements have clear acceptance criteria
- [x] User scenarios cover primary flows
- [x] Feature meets measurable outcomes defined in Success Criteria
- [x] No implementation details leak into specification — see note 1

## Notes

1. This module is a library API. Its users are control engineers; the names they type
   (`robot!`, `add_plugins`, `Bus::by_id`, `run()`, Zenoh) are the product, fixed by the SRS.
   The spec names them; it does not say how to build them.
2. Open markers: FR-006 / Q1 (hardware data without a preset), FR-052 / Q3 (what an envelope
   breach does). Each has options and a recommendation in "Open questions". Run
   `/speckit-clarify` before `/speckit-plan`. FR-024 / Q2 (which bus a plugin binds) is applied
   as a bus label, recommended, pending human confirmation.
3. Resolved from the SRS, no marker: State is a thin wrapper (section 3 State); running-loop hold
   deferred, so no default hold and a remote source needs a fallback (FR-050, FR-059); light
   payloads not registered; `by_dev` allowed; units rad / mm; Zenoh behind a feature with two
   lock-free slots; FR-10.3 threaded-only rule gone; `kmr_ffi` re-exported behind an `ffi`
   feature (FR-072).
4. Previous draft's stale items removed: `Bus::register::by_*!` top-level macros, the light
   tier, SI metres for prismatic, the user read-cadence override, the "drop FR-5.9" proposal,
   the rule freezing `controllers/src/main.rs`. Since coordination 2026-10-28 the complete API
   example is not frozen either: it may change with the API, and must build and run (FR-069).
5. Every FR and SC carries its SRS id. "Module boundaries" and "SRS coverage" end the spec;
   SRS problems are listed under "SRS issues found".
6. Iteration 1: all other items pass.
