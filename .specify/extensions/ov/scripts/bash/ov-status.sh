#!/usr/bin/env bash
# Read-only state report for /speckit-ov-help (FR-011–FR-013, data-model
# "Project setup state"). Prints JSON:
#   {"steps":[{"order":N,"step":"<command>","done":bool,"missing":[...]}...],
#    "config_errors":[...],"all_done":bool}
# Writes nothing, not even temp files. Always exits 0.
set -euo pipefail

# shellcheck source=SCRIPTDIR/ov-common.sh
source "$(dirname "${BASH_SOURCE[0]}")/ov-common.sh"

steps=()
missing=()

# need <condition-description> <test...>: add to missing when the test fails.
need() {
  local what=$1
  shift
  "$@" || missing+=("$what")
}
step() { # step <order> <command>: close the current step
  steps+=("$(jq -cn --argjson order "$1" --arg step "$2" \
    '{order: $order, step: $step, done: ($ARGS.positional | length == 0), missing: $ARGS.positional}' \
    --args "${missing[@]+"${missing[@]}"}")")
  missing=()
}

is_file() { [[ -f $OV_ROOT/$1 ]]; }
is_dir() { [[ -d $OV_ROOT/$1 ]]; }
hook_ok() { ov_hook_registered; }
event_ok() { ov_event_registered; }
aiignore_has_entries() {
  [[ -f $OV_ROOT/.aiignore ]] && grep -qvE '^[[:space:]]*(#|$)' "$OV_ROOT/.aiignore"
}
# Prints the file with every <!-- ... --> comment removed, multi-line ones included.
strip_html_comments() {
  awk '{
    out = ""; line = $0
    while (line != "") {
      if (in_c) {
        i = index(line, "-->"); if (!i) { line = ""; break }
        line = substr(line, i + 3); in_c = 0
      } else {
        i = index(line, "<!--"); if (!i) { out = out line; break }
        out = out substr(line, 1, i - 1); line = substr(line, i + 4); in_c = 1
      }
    }
    print out
  }' "$1"
}
# Placeholders are looked for outside comments only: /speckit-constitution's Sync Impact
# Report comment names the placeholders it replaced.
constitution_ok() {
  local c="$OV_ROOT/.specify/memory/constitution.md"
  [[ -f $c ]] && grep -qF '<!-- ov-policy -->' "$c" &&
    ! strip_html_comments "$c" | grep -Eq '\[[A-Z][A-Z0-9_]{2,}\]'
}
configured() { [[ $(ov_config_get configured 2>/dev/null || true) == true ]]; }

need "AGENTS.md" is_file AGENTS.md
need "docs/srs/" is_dir docs/srs
need "docs/srs/modules/" is_dir docs/srs/modules
need "docs/c4/" is_dir docs/c4
need "after_implement hook speckit.ov.c4-code in .specify/extensions.yml" hook_ok
need "PreToolUse event speckit.ov.guard in .claude/settings.json" event_ok
step 1 /speckit-ov-init

need "docs/srs/SRS.md" is_file docs/srs/SRS.md
step 2 /speckit-ov-srs

need "docs/workspace.dsl" is_file docs/workspace.dsl
step 3 /speckit-ov-c4-context

need ".aiignore with at least one pattern" aiignore_has_entries
need "PreToolUse event speckit.ov.guard in .claude/settings.json" event_ok
need ".github/workflows/ov-aiignore.yml" is_file .github/workflows/ov-aiignore.yml
step 4 /speckit-ov-nogo

need ".specify/memory/constitution.md with the <!-- ov-policy --> marker and no [PLACEHOLDER]" constitution_ok
step 5 /speckit-constitution

need "configured: true in .specify/extensions/ov/ov-config.yml" configured
need "CONTRIBUTING.md" is_file CONTRIBUTING.md
need ".github/ov-ruleset-main.json" is_file .github/ov-ruleset-main.json
step 6 /speckit-ov-pr

errors=()
if [[ -f $OV_CONFIG ]]; then
  for spec in "authorship:coauthor|trailer|none" "auto_merge:on|off" "ai_review:on|off" "configured:true|false"; do
    key=${spec%%:*}
    allowed=${spec#*:}
    value=$(ov_config_get "$key" 2>/dev/null || true)
    if [[ -z $value ]]; then
      errors+=("$key: missing (allowed: ${allowed//|/ | })")
    elif [[ "|$allowed|" != *"|$value|"* ]]; then
      errors+=("$key: $value (allowed: ${allowed//|/ | })")
    fi
  done
else
  errors+=("ov-config.yml not found at .specify/extensions/ov/ov-config.yml")
fi

printf '%s\n' "${steps[@]}" | jq -s --args '{
  steps: .,
  config_errors: $ARGS.positional,
  all_done: (map(.done) | all)
}' "${errors[@]+"${errors[@]}"}"
