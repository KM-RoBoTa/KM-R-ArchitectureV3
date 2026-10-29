---
name: speckit-ov-init
description: 'Initialize ov: write AGENTS.md and the docs/ skeleton, then verify the spec-kit hook and event'
compatibility: Requires spec-kit project structure with .specify/ directory
metadata:
  author: TODO(author)
  source: ov:commands/speckit.ov.init.md
---

# ov: initialize the project

The **first once-per-project** ov command. It writes `AGENTS.md` and the empty `docs/srs/`,
`docs/srs/modules/` and `docs/c4/` folders (each with an empty `.gitkeep` so git tracks it),
and checks that spec-kit registered the ov `after_implement` hook and the PreToolUse guard.
It never overwrites an existing file, and running it again changes nothing.

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Execution

This command has no human dialogue. Run the script from the project root:

- **Bash**: `.specify/extensions/ov/scripts/bash/ov-init.sh`

Relay every output line (`created:`, `exists, kept:`, `ov: ...`) to the user.

Exit codes:

- `0`: done. Tell the user to run `/speckit-ov-help` next.
- `1`: the hook or the guard event is missing. Show the restore command the script printed
  (`specify extension add <source> --dev --force`, where `<source>` is the `extensions/ov`
  directory of the ov checkout) and stop. Do not try to register anything yourself.

Do not edit `AGENTS.md` afterwards unless the human asks.