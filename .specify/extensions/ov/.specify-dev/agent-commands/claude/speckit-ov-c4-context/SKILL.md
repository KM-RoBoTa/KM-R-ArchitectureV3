---
name: speckit-ov-c4-context
description: Build the C4 context and container levels (C1/C2) in docs/workspace.dsl with the human
compatibility: Requires spec-kit project structure with .specify/ directory
metadata:
  author: TODO(author)
  source: ov:commands/speckit.ov.c4-context.md
---

# ov: C4 context and containers

Creates the `docs/workspace.dsl` skeleton (Structurizr DSL), builds the C1 (system context)
and C2 (container) levels with the human, and keeps one `!include c4/<module>.dsl` line per
SRS module.

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Execution

1. **Drop the skeleton.** Run from the project root:

   - **Bash**: `.specify/extensions/ov/scripts/bash/ov-c4.sh skeleton`

   Relay `created:` or `exists, kept:`. An existing workspace is never overwritten.

2. **Build C1 with the human**: the people who use the system and the external software
   systems it talks to. Propose them from `docs/srs/SRS.md` and the code, then let the human
   confirm or correct, **one question at a time**.

3. **Build C2 with the human**: the containers inside the system (applications, services,
   data stores) with their technology, and the relationships between them. Same rule: you
   propose, the human confirms or corrects, one question at a time.

4. **Edit `docs/workspace.dsl`** with what the human confirmed. The file is shared: you may
   edit it directly (it is not in the default `.aiignore`). Use Structurizr DSL with
   `!identifiers hierarchical`; keep the `systemContext` and `container` views valid. Replace
   the skeleton's `TODO(…)` placeholders.
   - **Never touch the lines between `// ov:includes:begin` and `// ov:includes:end`**, and
     never remove the markers. The script owns that region.

5. **Sync the module includes**:

   - **Bash**: `.specify/extensions/ov/scripts/bash/ov-c4.sh includes`

   It adds one `!include c4/<module>.dsl` per module of `docs/srs/SRS.md` and creates a stub
   `docs/c4/<module>.dsl` for each module that has none. Relay its output.

Exit codes: `0` done; `1` refused (no SRS: run `/speckit-ov-srs` first; markers missing:
ask the human to restore them). Show the message and stop.