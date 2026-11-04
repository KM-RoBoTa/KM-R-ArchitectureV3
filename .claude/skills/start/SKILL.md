---
name: start
description: Open a cycle session. Reads workflows/cycle/ROADMAP.md, says which module is at which step, asks the mode for any module that has none. Use at session start or on /start.
---

# start

1. Read `workflows/cycle/ROADMAP.md`. If it is missing, say so and stop:
   this project has not adopted the workflow yet.
2. If `docs/deferred.md` has untriaged lines (`- [feature]`, `- [bug]`,
   `- [question]` lines without a mark), show them and ask David to
   mark each one `next` / `later` / `drop` with one `AskUserQuestion`.
   Apply the marks: `next` → top of `docs/plans/<module>.md`, or a new
   change row in the ROADMAP at step 4 when no plan fits; `later` →
   keep the line, add the mark; `drop` → delete the line. Then
   continue.
3. Say which module is at which step. Then list only what blocks the
   current step, one line each. Nothing else.
4. For every row whose mode is unset and whose step is 4 or 5, invoke
   the `mode` skill for that module. One module at a time.
5. Before step 5, if no plan file `docs/plans/<module>.md` exists for
   this module and profile isn't `toy`, run `superpowers:writing-plans`
   first; don't ask.
6. Stop. Do not start building. Wait for the user to pick a module.

Rules:

- Rows whose owner is `mine` are reported but never proposed for a loop.
- If the table is empty, propose step 1 and offer
  `superpowers:brainstorming`.
- If the next module is at step 3, offer `superpowers:writing-plans`.
- If the next module is at step 6, offer
  `superpowers:requesting-code-review` with `QA.md` as the checklist.
