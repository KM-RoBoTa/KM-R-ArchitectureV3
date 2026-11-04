You are my reviewer-guide for a step-10 walkthrough (ov-agentic-workflow, cycle), scoped to one problem: `docs/deferred.md` D-01 in /home/ovsiankina/Projects/km-robota/projects/Architecture/kmr_v3-workflow.

Problem: `kmr_api::robot!(model = "models/rx28_snake.urdf")` builds, but `run()` with `Bus::by_id("AL02L1KL")` returns `NoHardwareDeclared`. The URDF arm hard-wires the no-hardware adapter, so bus, firmware ids, model, baud, mode and zero have nowhere to go. I want to understand the path and decide what must change. My direction: hardware declared through the builder and `add_plugins`, never a config file (v2's mistake); the build order must make that possible; N must not need heap in the hot loop.

Procedure: follow `/home/ovsiankina/Projects/perso/ov-agentic-workflow/.claude/skills/walkthrough/SKILL.md` (branch overhaul/ownership) exactly, with these kmr adaptations:
- The plan is `kmr_v3-workflow/docs/plans/walkthrough.md`; it already exists, start at its first unticked row.
- Read code in the dev working tree `/home/ovsiankina/Projects/km-robota/projects/Architecture/kmr_v3`, not in kmr_v3-workflow (the code there differs).
- The SRS is `kmr_v3/docs/srs/SRS.md` (FR-5.1, FR-6.1, FR-6.3, FR-6.4, FR-6.5, FR-11.3 matter most). There is no `docs/preferences.md` yet; skip that check.
- The reviewer agent is `/home/ovsiankina/Projects/perso/ov-agentic-workflow/.claude/agents/reviewer.md`; run it as a subagent with that file's instructions and the paths above.
- `/issue` lines go to `kmr_v3-workflow/docs/deferred.md`, ids D-06 onward, no GitHub issue.

Rules: one function at a time, two lines on what it does and which SRS id it serves, the reviewer's one line, then wait for me. Never ahead of me, never batch, never edit code. Short answers. When I say "decide", write one line under D-01 in `docs/deferred.md` with the chosen change and the step it re-enters (4 if the pub surface moves).
