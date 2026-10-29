#!/usr/bin/env bash
# /speckit-ov-init: write AGENTS.md and the docs/ skeleton, then verify the
# spec-kit hook and event (FR-006, FR-008, FR-009).
#
# usage: ov-init.sh
# Exit 0 = done, 1 = refused (hook or event missing). Re-running changes
# nothing (SC-006).
set -euo pipefail

# shellcheck source=SCRIPTDIR/ov-common.sh
source "$(dirname "${BASH_SOURCE[0]}")/ov-common.sh"

restore="specify extension add <source> --dev --force"

ov_copy_template ov-agents.md AGENTS.md

for d in docs/srs docs/srs/modules docs/c4; do
  mkdir -p "$OV_ROOT/$d"
  if [[ -e $OV_ROOT/$d/.gitkeep ]]; then
    printf 'exists, kept: %s/.gitkeep\n' "$d"
  else
    : >"$OV_ROOT/$d/.gitkeep"
    printf 'created: %s/.gitkeep\n' "$d"
  fi
done

hook=0
ov_hook_registered || hook=$?
event=0
ov_event_registered || event=$?

if ((hook == 2)); then
  # The ov entry exists but is disabled; disabling also drops the event.
  printf 'ov: the ov extension is disabled; running: specify extension enable ov\n'
  command -v specify >/dev/null 2>&1 ||
    ov_die "specify is not on PATH. Run: specify extension enable ov"
  (cd "$OV_ROOT" && specify extension enable ov) ||
    ov_die "specify extension enable ov failed. Restore with: $restore"
  hook=0
  ov_hook_registered || hook=$?
  event=0
  ov_event_registered || event=$?
fi

missing=()
((hook == 0)) || missing+=("the after_implement hook speckit.ov.c4-code in .specify/extensions.yml")
((event == 0)) || missing+=("the PreToolUse event speckit.ov.guard in .claude/settings.json")
if ((${#missing[@]})); then
  for m in "${missing[@]}"; do
    printf 'ov: missing %s\n' "$m" >&2
  done
  ov_die "restore it with: $restore (<source> is the extensions/ov directory of your ov checkout)"
fi

printf 'ov: the spec-kit hook (after_implement -> speckit.ov.c4-code) and the PreToolUse guard are registered\n'
printf 'ov: next, run /speckit-ov-help for the workflow order\n'
