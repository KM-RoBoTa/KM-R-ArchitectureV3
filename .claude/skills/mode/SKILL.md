---
name: mode
description: Set how a module gets built. Asks one question with exactly three options (draft, pair, loop) and writes the answer into the module's ROADMAP row. Use on /mode <module>.
---

# mode

Input: a module name. Ask for it if not given.

1. Read the module's row in `workflows/cycle/ROADMAP.md`. If mode is
   already set, show it and ask whether to change it. On no, stop.
2. Call `AskUserQuestion` with header `Mode`, single-select, and exactly
   these three options in this order:
   - **draft**: I correct what you write. The agent proposes the full
     change; the owner edits it before anything runs.
   - **pair**: I drive, you type. The owner says the next step, the
     agent writes it, the oracle runs after each one.
   - **loop**: `/goal` with the oracle, no questions. The agent runs
     until the oracle is green or 3 deferrals, under `CLAUDE.md`.
   No other options. Put `(Recommended)` on `loop` only when the owner
   is `delegated` and the step is 4 or 5; otherwise on `pair`.
3. Write the answer into the `mode` column of that row. Touch nothing
   else in the table.
4. Print the updated row.

Rules:

- If owner is `mine` and the answer is `loop`, warn once: a loop may not
  touch `mine` files without listing them first. Ask again.
- Never set mode for a module that is not in the table. Say so instead.
