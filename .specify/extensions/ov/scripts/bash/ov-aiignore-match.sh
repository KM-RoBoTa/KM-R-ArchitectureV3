#!/usr/bin/env bash
# ov .aiignore matcher (research R-4): git's own gitignore engine on a
# throwaway bare repo whose info/exclude holds the patterns.
#
# CLI:
#   ov-aiignore-match.sh <patterns-file> <relpath>...
#   ov-aiignore-match.sh <patterns-file> --stdin
# Prints the matching paths. Exit 0 = at least one match, 1 = no match,
# 2 = usage or matcher error.
#
# Sourced: defines ov_aiignore_match with the same arguments and return codes.
#
# The bare repo is cached per patterns content under
# ${TMPDIR:-/tmp}/ov-aiignore-<sha256>, so a warm call costs one hash and one
# `git check-ignore`. Paths must be relative to the project root.

_ov_aiignore_usage() {
  printf 'usage: ov-aiignore-match.sh <patterns-file> <relpath>... | --stdin\n' >&2
}

# _ov_aiignore_git <cache> <args...>: git with no system/global config, run
# inside the empty work tree so relative paths resolve against its root.
_ov_aiignore_git() {
  local cache=$1
  shift
  GIT_CONFIG_NOSYSTEM=1 GIT_CONFIG_GLOBAL=/dev/null \
    git -C "$cache/wt" --git-dir="$cache" --work-tree="$cache/wt" \
    -c core.excludesFile=/dev/null -c core.quotePath=false "$@"
}

# _ov_aiignore_build <dir> <patterns-file>: fill an empty dir with the bare
# repo and the empty work tree.
_ov_aiignore_build() {
  GIT_CONFIG_NOSYSTEM=1 GIT_CONFIG_GLOBAL=/dev/null \
    git init -q --bare --template= "$1" >/dev/null 2>&1 || return 1
  mkdir -p "$1/info" "$1/wt" || return 1
  cp "$2" "$1/info/exclude" || return 1
}

# _ov_aiignore_cache <patterns-file>: sets _OV_AIIGNORE_CACHE (and
# _OV_AIIGNORE_PRIVATE=1 when the dir must be removed after use).
_ov_aiignore_cache() {
  local patterns=$1 base hash tmp
  base=${TMPDIR:-/tmp}
  base=${base%/}
  if command -v sha256sum >/dev/null 2>&1; then
    read -r hash _ < <(sha256sum <"$patterns") || return 1
  else
    read -r hash _ < <(shasum -a 256 <"$patterns") || return 1
  fi
  [[ -n $hash ]] || return 1
  _OV_AIIGNORE_CACHE="$base/ov-aiignore-$hash"
  _OV_AIIGNORE_PRIVATE=0
  if [[ -d $_OV_AIIGNORE_CACHE/wt && -O $_OV_AIIGNORE_CACHE ]]; then
    return 0
  fi
  tmp=$(mktemp -d "$base/ov-aiignore-build.XXXXXX") || return 1
  if ! _ov_aiignore_build "$tmp" "$patterns"; then
    rm -rf "$tmp"
    return 1
  fi
  if [[ -e $_OV_AIIGNORE_CACHE && ! -O $_OV_AIIGNORE_CACHE ]]; then
    # Owned by another user: never trust it, use the private build once.
    _OV_AIIGNORE_CACHE=$tmp
    _OV_AIIGNORE_PRIVATE=1
    return 0
  fi
  if [[ -d $_OV_AIIGNORE_CACHE/wt ]]; then
    # Another process of ours finished first.
    rm -rf "$tmp"
    return 0
  fi
  # A broken leftover of ours (no work tree): replace it.
  [[ -e $_OV_AIIGNORE_CACHE ]] && rm -rf "$_OV_AIIGNORE_CACHE"
  if mv "$tmp" "$_OV_AIIGNORE_CACHE" 2>/dev/null; then
    # Lost a race: mv put our build inside the winner's dir. Drop it.
    rm -rf "${_OV_AIIGNORE_CACHE:?}/${tmp##*/}"
  else
    rm -rf "$tmp"
  fi
  [[ -d $_OV_AIIGNORE_CACHE/wt && -O $_OV_AIIGNORE_CACHE ]]
}

# ov_aiignore_match <patterns-file> (<relpath>... | --stdin)
ov_aiignore_match() {
  local patterns=${1-} rc=0
  if [[ -z $patterns || $# -lt 2 ]] || [[ $2 == --stdin && $# -ne 2 ]]; then
    _ov_aiignore_usage
    return 2
  fi
  shift
  if [[ ! -f $patterns || ! -r $patterns ]]; then
    printf 'ov: patterns file not readable: %s\n' "$patterns" >&2
    return 2
  fi
  if ! _ov_aiignore_cache "$patterns"; then
    printf 'ov: cannot build the .aiignore matcher under %s\n' "${TMPDIR:-/tmp}" >&2
    return 2
  fi
  if [[ $1 == --stdin ]]; then
    _ov_aiignore_git "$_OV_AIIGNORE_CACHE" check-ignore --no-index --stdin || rc=$?
  else
    _ov_aiignore_git "$_OV_AIIGNORE_CACHE" check-ignore --no-index -- "$@" || rc=$?
  fi
  if ((_OV_AIIGNORE_PRIVATE)); then
    rm -rf "$_OV_AIIGNORE_CACHE"
  fi
  case $rc in
    0 | 1) return "$rc" ;;
    *) return 2 ;;
  esac
}

if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  set -euo pipefail
  rc=0
  ov_aiignore_match "$@" || rc=$?
  exit "$rc"
fi
