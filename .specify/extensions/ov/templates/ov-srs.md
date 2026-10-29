# TODO(system name): Software Requirements Specification

<!--
Human-written source of truth: agents never write or rewrite this file.
Replace every TODO placeholder with your own text. /speckit-ov-srs refuses to split the
file into module specs while one is left.

Everything before the first "## Module:" heading is shared: it is copied into every
generated module spec. After the first module, only "## Module: <name>" headings may be
level-2 headings.

Module names match ^[a-z0-9][a-z0-9_-]*$ and are unique. Each module needs a "Paths:" line
of gitignore-syntax globs, space-separated, naming the code that belongs to the module.
Copy the whole "## Module:" section once per module.
-->

## Purpose

TODO(why the system exists and who it serves)

## Scope

TODO(what is in scope, and what is explicitly out of scope)

## Module: TODO(module-name)

Paths: TODO(gitignore globs, space-separated, e.g. src/billing/** lib/billing.rs)

### Responsibility

TODO(what this module is responsible for, and what it is not)

### Inputs

TODO(the data, events or calls the module receives)

### Outputs

TODO(the data, events or calls the module produces)

### Invariants

TODO(conditions that always hold)

### Acceptance Tests

TODO(observable checks, one per line: Given ..., when ..., then ...)
