# Specification Quality Checklist: kmr_ffi, foreign-language controller math

**Purpose**: Validate specification completeness and quality before proceeding to planning
**Created**: 2026-10-27
**Updated**: 2026-10-28 (re-specified against the SRS as patched on 2026-10-27)
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

- Two [NEEDS CLARIFICATION] markers remain, on purpose: FR-023 (Q2, signals from foreign code),
  FR-015 (Q3, building C sources against N). They are the human's calls.
- Coordination 2026-10-28: FR-009 (Q1) applied as integers (ticks `u64`, durations ns `u64`, flags
  bitmasks), recommended, pending human confirmation. FR-010 carries the per-actuator age.
- Closed by the 2026-10-27 SRS patch: the old Python/Lua scope question (FR-10.3 now allows every
  form, kmr_ffi may use the heap). The old threaded-only rule and its "inline Python fails the
  build" criterion are removed.
- "No implementation details": the module *is* a language boundary, so C, C++, Python, Lua, `f32`
  and C layout are the requirement, not a design choice (SRS FR-10.1, FR-10.2). No crate, binding
  tool or header generator is named.
- "Technology-agnostic success criteria": same exception. SC-003 cites the NFR-1 bench, which the
  SRS fixes.
- FR-10.4's measurement protocol is not defined by the SRS; SC-008 reuses the Rust path's protocol
  (owned by `bench` / `controllers`).
- Delivery step 8, Could: planning may start, building waits for steps 1 to 7.
