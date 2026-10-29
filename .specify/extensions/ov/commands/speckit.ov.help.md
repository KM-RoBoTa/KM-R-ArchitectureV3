---
description: "Show the ov command order and which once-per-project steps are missing (read-only)"
---

# ov: workflow order and status

Prints the ov command order in two groups and reports which once-per-project steps are
missing. It is **read-only**: it never runs a step and never creates, modifies or deletes a
file. Running it any number of times is safe.

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Execution

1. Run the status script from the project root and parse its JSON:

   - **Bash**: `.specify/extensions/ov/scripts/bash/ov-status.sh`

   It prints `{"steps":[{"order","step","done","missing"}...],"config_errors":[...],"all_done"}`
   and always exits `0`.

2. Print the two groups below, in this exact order, with exactly one sentence per command.
   Copy each group's bold heading and its opening sentence verbatim: that sentence says why
   the split exists (FR-010). Always print both groups in full, every command of each,
   including when steps are missing; the stop in step 4 comes after both groups.
   Mark each once-per-project step `done` or `missing` from the JSON.

   **Once per project.** These steps set up the policy and the architecture skeleton that
   every feature relies on, so each runs once, in this order.

   1. `/speckit-ov-init`: writes `AGENTS.md` and the empty `docs/` skeleton, and checks that
      the ov hook and guard are registered.
   2. `/speckit-ov-srs`: the human writes `docs/srs/SRS.md` from the template, then it is
      split into one generated spec per module.
   3. `/speckit-ov-c4-context`: builds the C4 context and container levels in
      `docs/workspace.dsl` with the human.
   4. `/speckit-ov-nogo`: creates `.aiignore`, the paths agents may read but never edit, and
      installs the `ov-aiignore` CI check.
   5. `/speckit-constitution`: writes the project constitution, which starts with the ov
      policy.
   6. `/speckit-ov-pr`: records the authorship, auto-merge and AI review policy, and writes
      the ruleset template.

   **Once per module or feature.** These steps repeat for each unit of work and trace back to
   the setup above.

   1. `/speckit-specify`: turns the human's feature description into a spec.
   2. `/speckit-clarify`: resolves the spec's open questions with the human.
   3. `/speckit-plan`: designs the implementation against the spec and the constitution.
   4. `/speckit-tasks`: breaks the plan into ordered tasks.
   5. `/speckit-implement`: carries out the tasks.
   6. `/speckit-converge`: checks the code against the spec and adds any missing work as
      tasks.
   7. `/speckit-ov-c4-code`: updates the C4 component and code levels for the module just
      implemented.
   8. PR: open a pull request; the required checks decide whether it can merge.

3. If `config_errors` is not empty, list each entry and say that `/speckit-ov-pr` rewrites
   the config with valid values.

4. If any step has `done: false`, **stop**: name each missing step, the artifacts in its
   `missing` list, and the command that produces it (the step's own command), and say the
   workflow cannot go on until they exist (FR-011). Otherwise say every once-per-project step
   is done.

## Rules

- Never run a step yourself, even when it is missing.
- Never create, modify or delete a file.
- Never list `/speckit-ov-guard`: it is internal.
