---
description: "Internal: PreToolUse guard that denies agent edits to paths listed in .aiignore (run by spec-kit, not by users)"
scripts:
  sh: scripts/bash/ov-guard.sh
---

# ov: internal PreToolUse guard

This command is **internal**. It exists only to carry the `scripts:` entry that
spec-kit's `pre_tool_use` event runs before every Edit, Write, MultiEdit and
NotebookEdit call. It is not part of the ov workflow, and `/speckit-ov-help`
does not list it.

## User Input

```text
$ARGUMENTS
```

## Execution

Do nothing. Tell the user that `/speckit-ov-guard` is internal: spec-kit runs
it automatically as a PreToolUse hook, and invoking it by hand has no effect.
To choose which paths are read-only for agents, run `/speckit-ov-nogo`.
