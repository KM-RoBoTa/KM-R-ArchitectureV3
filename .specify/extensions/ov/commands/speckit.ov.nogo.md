---
description: "Create .aiignore (paths read-only for agents) and install the ov-aiignore CI check"
---

# ov: read-only paths for agents

Creates `.aiignore`, the list of paths agents may read but never edit, installs the
`ov-aiignore` CI workflow, and verifies that the PreToolUse guard is registered.

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty). Paths or patterns
given there count as the human's answer to the question in step 2.

## Rules

- **Never edit `.aiignore` with the Edit, Write, MultiEdit or NotebookEdit tools.** It lists
  itself, so the guard denies those edits. Only `ov-nogo.sh` (step 3) or the human changes it.
- Read is never blocked: listed paths stay readable, they are only read-only for agents.
- The guard does not intercept writes made through Bash. The `ov-aiignore` CI check is the
  backstop: it fails any PR where an agent commit touches a listed path (FR-014).

## Execution

1. **Show the current list.** If `.aiignore` exists at the project root, read it and show the
   human its non-comment lines. Do not change it.

2. **Ask one question**: "Which paths should be read-only for agents? Use gitignore syntax,
   one pattern per line (for example `src/core/` or `migrations/**`)." The default template
   already lists the ov enforcement files and `docs/srs/SRS.md`; the human only names
   project paths. An empty answer is fine: it keeps the defaults only.

3. **Run the script** from the project root with one `--add` per pattern the human gave:

   - **Bash**: `.specify/extensions/ov/scripts/bash/ov-nogo.sh --add '<pattern>' --add '<pattern>'`

   Quote each pattern so the shell does not expand it.

4. **Relay the output**: every `created:`, `exists, kept:`, `added:` and `already listed:`
   line.

5. **Exit codes**:
   - `0`: done.
   - `1`: refused or error. Show the message. When it says the guard is not registered, show
     the restore command it printed
     (`specify extension add <path-to-ov>/extensions/ov --dev --force`) and stop.

6. **Remind the human** that Read is never blocked, that Bash writes are not intercepted and
   CI is the backstop, and that `.aiignore` and `.github/workflows/ov-aiignore.yml` must be
   committed by the human, as a commit without an agent trailer. An agent commit touching
   them fails `ov-aiignore` by design.
