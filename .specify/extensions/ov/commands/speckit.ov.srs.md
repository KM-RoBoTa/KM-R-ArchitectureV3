---
description: "Guide the human through writing docs/srs/SRS.md, then split it into docs/srs/modules/<name>.md"
---

# ov: write the SRS

Drops the SRS template at `docs/srs/SRS.md` and guides the human through it section by
section. After the human confirms, a script splits the SRS into one generated
`docs/srs/modules/<name>.md` per module.

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Rules

- **The human writes the SRS. You never write, rewrite, summarise or "improve" SRS content**,
  not even when asked to draft a section (Principle II, FR-023). You may explain what a
  section is for and point out a problem the script reported.
- `docs/srs/SRS.md` is listed in the default `.aiignore`, so the guard denies your edits to
  it anyway. Never work around that with Bash.
- The generated module files are written only by the script. Never edit them.

## Execution

1. **Drop the template.** Run from the project root:

   - **Bash**: `.specify/extensions/ov/scripts/bash/ov-srs.sh template`

   Relay `created:` or `exists, kept:`. An existing SRS is never overwritten.

2. **Walk the human through the template**, one section at a time, in file order. For each
   section, read its guidance (the text inside its `TODO(…)` placeholder) to the human, then
   wait while they write it in their editor:
   - the title line (the system name);
   - `## Purpose` and `## Scope`: shared sections, copied into every module spec;
   - each `## Module: <name>` section. The name matches `^[a-z0-9][a-z0-9_-]*$` and is
     unique. Its `Paths:` line lists gitignore globs, space-separated, naming the module's
     code (the AI review uses it to map changed files to modules). Then `### Responsibility`,
     `### Inputs`, `### Outputs`, `### Invariants` and `### Acceptance Tests`. The human
     copies the section once per module.

   After the first module, the only level-2 headings allowed are `## Module:` headings.

3. **Split only after the human confirms** the SRS is finished:

   - **Bash**: `.specify/extensions/ov/scripts/bash/ov-srs.sh split`

4. **Exit codes**:
   - `0`: report every `created:`, `updated:` and `unchanged:` file, and every `orphan:`
     line. An orphan is a module file whose section is gone from the SRS; it is left in
     place for the human to delete.
   - `3`: `TODO(` placeholders are left. Show the listed lines, ask the human to replace
     them, and run `split` again when they confirm.
   - `1`: the SRS is invalid or missing. Show the reported lines (duplicate or invalid
     module name, missing `Paths:`, a stray level-2 heading) and ask the human to fix them.
     Nothing was written.

Re-running `split` on an unchanged SRS changes nothing.
