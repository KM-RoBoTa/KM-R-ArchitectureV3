#!/usr/bin/env bash
# /speckit-ov-nogo: create .aiignore, add read-only patterns, install the
# ov-aiignore workflow and verify the PreToolUse guard (FR-030–FR-033).
#
# usage: ov-nogo.sh [--add <pattern>]...
# Exit 0 = done, 1 = refused or error. Never modifies .claude/settings.json.
set -euo pipefail

# shellcheck source=SCRIPTDIR/ov-common.sh
source "$(dirname "${BASH_SOURCE[0]}")/ov-common.sh"

patterns=()
while (($#)); do
  case $1 in
    --add)
      (($# >= 2)) || ov_die "--add needs a pattern"
      [[ -n $2 ]] || ov_die "--add: empty pattern"
      [[ $2 != *$'\n'* && $2 != *$'\r'* ]] || ov_die "--add: a pattern must be a single line"
      patterns+=("$2")
      shift 2
      ;;
    *) ov_die "unknown argument: $1 (usage: ov-nogo.sh [--add <pattern>]...)" ;;
  esac
done

ov_copy_template ov-aiignore .aiignore

if ((${#patterns[@]})); then
  aiignore="$OV_ROOT/.aiignore"
  tmp=$(ov_tmpfile)
  cp "$aiignore" "$tmp"
  # Make sure the last line ends before appending.
  if [[ -s $tmp && -n $(tail -c 1 "$tmp") ]]; then
    printf '\n' >>"$tmp"
  fi
  for p in "${patterns[@]}"; do
    if grep -qxF -- "$p" "$tmp"; then
      printf 'already listed: %s\n' "$p"
    else
      printf '%s\n' "$p" >>"$tmp"
      printf 'added: %s\n' "$p"
    fi
  done
  ov_write_if_changed "$aiignore" "$tmp"
fi

ov_copy_template ov-aiignore.yml .github/workflows/ov-aiignore.yml

if ! ov_event_registered; then
  ov_die "the .aiignore guard is not registered in .claude/settings.json. Restore it with: specify extension add <path-to-ov>/extensions/ov --dev --force"
fi

printf 'ov: the .aiignore guard is registered. Read is never blocked; Bash writes are not intercepted, and the ov-aiignore CI check is the backstop.\n'
printf 'ov: commit .aiignore and .github/workflows/ov-aiignore.yml as a human commit (no agent trailer).\n'
