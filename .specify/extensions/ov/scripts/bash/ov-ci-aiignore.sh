#!/usr/bin/env bash
# ov-aiignore CI check (contracts/ci-checks.md, research R-6).
#
# usage: ov-ci-aiignore.sh <base_sha> <head_sha>
#
# Run from a checkout of the base commit, with the head commits fetched but
# not checked out. For every commit in base..head that carries an agent
# trailer, every path it touches is matched against the base commit's
# .aiignore. Branch names play no role.
#
# Exit 0 = no violation (or no .aiignore on the base), 1 = violations, listed
# as "<sha> <path>", 2 = usage or git error. Never checks out or runs head
# content.
set -euo pipefail

_ov_ci_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=SCRIPTDIR/ov-aiignore-match.sh
source "$_ov_ci_dir/ov-aiignore-match.sh"

ov_ci_die() {
  printf 'ov-aiignore: %s\n' "$*" >&2
  exit 2
}

ov_ci_summary() {
  [[ -n ${GITHUB_STEP_SUMMARY:-} ]] || return 0
  printf '%s\n' "$@" >>"$GITHUB_STEP_SUMMARY"
}

# ov_ci_is_agent <message>: data-model "Agent commit" (FR-031, FR-034).
ov_ci_is_agent() {
  local msg=${1//$'\r'/}
  grep -qiE '^co-authored-by:.*claude' <<<"$msg" || grep -qE '^Assisted-by: claude-code$' <<<"$msg"
}

[[ $# -eq 2 ]] || {
  printf 'usage: ov-ci-aiignore.sh <base_sha> <head_sha>\n' >&2
  exit 2
}
base_ref=$1
head_ref=$2
base=$(git rev-parse -q --verify "${base_ref}^{commit}") || ov_ci_die "base commit not found: $base_ref"
head=$(git rev-parse -q --verify "${head_ref}^{commit}") || ov_ci_die "head commit not found: $head_ref"

if ! git cat-file -e "$base:.aiignore" 2>/dev/null; then
  printf 'ov-aiignore: no .aiignore on the base branch, nothing to check\n'
  ov_ci_summary "ov-aiignore: no .aiignore on the base branch, nothing to check"
  exit 0
fi

work=$(mktemp -d "${TMPDIR:-/tmp}/ov-ci.XXXXXX")
trap 'rm -rf "$work"' EXIT
patterns="$work/aiignore"
git show "$base:.aiignore" >"$patterns"

base_cfg=$(git show "$base:.specify/extensions/ov/ov-config.yml" 2>/dev/null || true)
if grep -qE '^authorship:[[:space:]]*none[[:space:]]*$' <<<"$base_cfg"; then
  printf 'ov-aiignore: authorship is none on the base branch: agent commits carry no trailer and cannot be told apart\n'
  ov_ci_summary "ov-aiignore: authorship is none on the base branch: agent commits carry no trailer and cannot be told apart"
fi

commits=$(git rev-list --reverse "$base..$head")
checked=0
agents=0
violations=()
while IFS= read -r c; do
  [[ -n $c ]] || continue
  checked=$((checked + 1))
  ov_ci_is_agent "$(git log -1 --format=%B "$c")" || continue
  agents=$((agents + 1))
  read -r -a parents <<<"$(git rev-list --parents -n 1 "$c")"
  if ((${#parents[@]} > 2)); then
    # Merge: only paths whose result differs from every parent.
    paths=$(git diff-tree --no-commit-id --cc --name-only "$c")
  else
    paths=$(git diff-tree --no-commit-id --no-renames -r --root --name-only "$c")
  fi
  [[ -n $paths ]] || continue
  rc=0
  matched=$(ov_aiignore_match "$patterns" --stdin <<<"$paths") || rc=$?
  ((rc <= 1)) || ov_ci_die "the .aiignore matcher failed on commit $c"
  ((rc == 0)) || continue
  while IFS= read -r p; do
    [[ -n $p ]] && violations+=("$c $p")
  done <<<"$matched"
done <<<"$commits"

if ((${#violations[@]})); then
  printf 'ov-aiignore: agent commits touch paths listed in the base branch .aiignore:\n'
  printf '%s\n' "${violations[@]}"
  ov_ci_summary "## ov-aiignore: failed" "" \
    "Agent commits touch paths listed in the base branch \`.aiignore\`:" ""
  for v in "${violations[@]}"; do
    ov_ci_summary "- \`${v%% *}\` \`${v#* }\`"
  done
  ov_ci_summary "" "A human must make these changes, or the paths must leave \`.aiignore\` on the base branch first."
  exit 1
fi
printf 'ov-aiignore: %d commit(s) checked, %d agent commit(s), no read-only path touched\n' "$checked" "$agents"
ov_ci_summary "ov-aiignore: $checked commit(s) checked, $agents agent commit(s), no read-only path touched"
