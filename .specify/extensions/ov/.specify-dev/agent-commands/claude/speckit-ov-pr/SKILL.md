---
name: speckit-ov-pr
description: 'Set the PR and authorship policy: config, CONTRIBUTING.md, ruleset template and optional workflows'
compatibility: Requires spec-kit project structure with .specify/ directory
metadata:
  author: TODO(author)
  source: ov:commands/speckit.ov.pr.md
---

# ov: PR and authorship policy

Asks three questions, then records the answers in the ov config and `CONTRIBUTING.md`,
writes the GitHub ruleset template for `main` and installs the optional workflows. Running it
again with new answers rewrites only the files ov owns.

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty). Answers given there
count as answers to the questions below; ask only the ones still open.

## Execution

1. **Ask the three questions, one at a time**, each with its default. There is no
   branch-prefix question: a commit is an agent commit only when it carries an agent trailer,
   whatever its branch.
   1. **Authorship**: `coauthor` (default: Claude Code's own `Co-Authored-By: Claude
      <noreply@anthropic.com>` trailer), `trailer` (commits end with
      `Assisted-by: claude-code`) or `none` (no trailer and no PR footer: agent commits look
      human, so `ov-aiignore` cannot see them and the hook is the only guard).
   2. **Auto-merge on green for same-repo PRs**: `on` or `off` (default `off`). Fork PRs are
      never auto-merged.
   3. **AI review** against the module SRS, as a required check: `on` (default) or `off`.

2. **Run the script** from the project root with the answers:

   - **Bash**: `.specify/extensions/ov/scripts/bash/ov-pr.sh --authorship <coauthor|trailer|none> --auto-merge <on|off> --ai-review <on|off>`

3. **Relay its output**: every `created:`, `updated:`, `unchanged:`, `removed:` and
   `exists, kept` line. On exit `1`, show the message and stop: nothing was written.

4. **Tell the human what is left to do by hand**:
   - import `.github/ov-ruleset-main.json` in the repository settings (Settings → Rules →
     Rulesets → Import a ruleset);
   - when auto-merge is `on`: enable "Allow auto-merge" in the repository settings;
   - when `ai_review` is `on`: add the `ANTHROPIC_API_KEY` repository secret. Without it the
     required `ov-ai-review` check fails loudly on every same-repo PR;
   - commit the changed files as a **human commit** (no agent trailer). The default
     `.aiignore` lists several of them, so an agent commit touching them fails
     `ov-aiignore`.

## Rules

- Never edit `CONTRIBUTING.md`, `.claude/settings.json`, the ruleset or the workflows with
  your edit tools; only the script writes them.
- The script installs no git hooks and never sets `core.hooksPath`.