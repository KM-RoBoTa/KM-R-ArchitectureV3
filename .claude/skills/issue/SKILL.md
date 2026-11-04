---
name: issue
description: Note a feature, bug or question found mid-build without acting on it. Appends one line to docs/deferred.md, opens a GitHub issue when the repo has a GitHub remote, then says "noted" and resumes. Use on /issue feature|bug|question <text>.
---

# issue

Input: `/issue <kind> <text>`. `kind` is one of `feature`, `bug`,
`question`. Ask for whichever is missing. Never ask anything else.

1. Find the module and step: the module currently being worked on in
   this session, and its step from `workflows/cycle/ROADMAP.md`. If no
   module is in progress, use `none` and step `-`.
2. Pick the id:
   - If `git remote -v` shows a GitHub remote, run
     `gh issue create --title "<text>" --body "<kind>, found during
     <module> step N" --label <kind>` and use the returned number as
     `#<n>`. If `gh` fails (no auth, no label, network), fall through
     to the next bullet and say so in one line.
   - Otherwise id = `D-NN`, where NN is one more than the highest
     `D-` id already in `docs/deferred.md`, starting at `D-01`.
3. Append to `docs/deferred.md`, after the last line:
   `- [<kind>] <id> <text> (found during <module> step N)`
4. Say `noted` and continue what you were doing before `/issue`.

Rules:

- Never start the work. Not a stub, not a test, not a plan. The line
  is the whole deliverable; triage happens at the next `/start` (see
  `PROCEDURE.md`, "Feature requests during a build").
- Never edit any other line of `docs/deferred.md`.
- Never open an issue when the remote is not GitHub.
- If the same text is already in `docs/deferred.md`, say `already
  noted <id>` and skip steps 2 and 3.
