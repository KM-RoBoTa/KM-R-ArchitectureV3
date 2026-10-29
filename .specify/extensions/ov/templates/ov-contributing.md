# Contributing

This project uses **ov** to enforce its rules for AI coding agents. `AGENTS.md` says what
agents may and may not do; this file records the pull request and authorship policy.

<!-- ov:pr:begin -->
## PR and authorship policy

This section is written by `/speckit-ov-pr`. Change the answers by running it again; edits
made here by hand are replaced.

- **Agent commits.** A commit is an agent commit only when it carries an agent trailer (`Co-Authored-By: Claude …` or `Assisted-by: claude-code`), whatever its branch. A commit without one is a human commit.
- **Authorship mode: `{{OV_AUTHORSHIP}}`.** {{OV_AUTHORSHIP_NOTE}}
- **Read-only paths.** The `ov-aiignore` check fails a pull request when an agent commit
  touches a path listed in the base branch's `.aiignore`. Human commits may touch them.
- **Auto-merge: `{{OV_AUTO_MERGE}}`.** {{OV_AUTO_MERGE_NOTE}}
- **AI review: `{{OV_AI_REVIEW}}`.** {{OV_AI_REVIEW_NOTE}}
- **Branch `main`.** Changes land through pull requests only: no direct pushes, no force
  pushes, 0 required approvals, and the required checks listed in
  `.github/ov-ruleset-main.json`.
- **Setup commits.** Files written by ov's setup commands (`.aiignore`,
  `.github/workflows/ov-*.yml`, `.github/ov-ruleset-main.json`, `.claude/settings.json`,
  `.specify/extensions/ov/`) are committed by a human, in a commit without an agent trailer.
<!-- ov:pr:end -->
