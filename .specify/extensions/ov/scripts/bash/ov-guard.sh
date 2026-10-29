#!/usr/bin/env bash
# ov PreToolUse guard (contracts/guard-hook.md). spec-kit's dispatcher runs it
# for Edit|Write|MultiEdit|NotebookEdit with the Claude Code hook JSON on
# stdin. Exit 2 + stderr denies the call; exit 0 allows it.
#
# Reads only <root>/.aiignore, never ov-config.yml, and never writes to the
# project. A warm call is one jq, one hash and one git check-ignore (SC-007).
# Fails open when the payload is unusable or .aiignore is missing; fails
# closed (denies, naming the error) when .aiignore exists but cannot be
# checked.
set -euo pipefail

_ov_guard_dir=${BASH_SOURCE[0]%/*}
[[ $_ov_guard_dir != "${BASH_SOURCE[0]}" ]] || _ov_guard_dir=.
# shellcheck source=SCRIPTDIR/ov-aiignore-match.sh
source "$_ov_guard_dir/ov-aiignore-match.sh"

ov_guard_allow_warn() {
  printf 'ov: guard: %s; allowing the call\n' "$*" >&2
  exit 0
}

# ov_guard_normalize <absolute-path>: lexical . and .. handling, no symlink
# resolution. Sets OV_NORM.
ov_guard_normalize() {
  local seg IFS=/
  local -a segs out=()
  read -r -a segs <<<"$1"
  for seg in "${segs[@]}"; do
    case $seg in
      '' | .) ;;
      ..) ((${#out[@]} == 0)) || unset 'out[${#out[@]}-1]' ;;
      *) out+=("$seg") ;;
    esac
  done
  OV_NORM="/${out[*]}"
}

ov_guard_main() {
  local tool="" path="" cwd="" root base rel err rc=0
  # One jq call: tool name, path and cwd, NUL-separated. Bad JSON → no output.
  {
    IFS= read -r -d '' tool && IFS= read -r -d '' path && IFS= read -r -d '' cwd
  } < <(jq -j '
    def s: if type == "string" then . else "" end;
    (.tool_name | s), "\u0000",
    ((.tool_input.file_path // .tool_input.notebook_path) | s), "\u0000",
    (.cwd | s), "\u0000"
  ' 2>/dev/null) || ov_guard_allow_warn "could not parse the hook payload"
  [[ -n $path ]] || ov_guard_allow_warn "no file path in the ${tool:-tool} payload"

  root=${CLAUDE_PROJECT_DIR:-}
  [[ -n $root ]] || root=$cwd
  [[ -n $root ]] || root=$PWD
  [[ $root == /* ]] || root="$PWD/$root"
  ov_guard_normalize "$root"
  root=$OV_NORM

  base=${cwd:-$root}
  [[ $base == /* ]] || base="$root/$base"
  [[ $path == /* ]] || path="$base/$path"
  ov_guard_normalize "$path"
  path=$OV_NORM

  if [[ $root == / ]]; then
    rel=${path#/}
  elif [[ $path == "$root"/* ]]; then
    rel=${path#"$root"/}
  else
    exit 0 # outside the project
  fi
  [[ -n $rel ]] || exit 0
  # No .aiignore: nothing is protected yet (fail open). One that exists but
  # cannot be read is a matcher error below (fail closed).
  [[ -e $root/.aiignore || -L $root/.aiignore ]] || exit 0

  err=$(ov_aiignore_match "$root/.aiignore" "$rel" 2>&1 >/dev/null) || rc=$?
  case $rc in
    0)
      printf 'ov: %s is read-only for agents (listed in .aiignore). Ask the human to change it.\n' "$rel" >&2
      exit 2
      ;;
    1) exit 0 ;;
    *)
      err=${err//$'\n'/; }
      printf 'ov: cannot check %s against .aiignore, denying the call (fail closed): %s. Ask the human to fix it.\n' \
        "$rel" "${err:-the .aiignore matcher failed}" >&2
      exit 2
      ;;
  esac
}

ov_guard_main
