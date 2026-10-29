#!/usr/bin/env bash
# /speckit-ov-c4-context and /speckit-ov-c4-code: the Structurizr workspace
# skeleton and its per-module !include lines (FR-040–FR-042).
#
# usage: ov-c4.sh skeleton             copy the template to docs/workspace.dsl
#        ov-c4.sh includes [<module>]  one "!include c4/<module>.dsl" per SRS
#                                      module (or just <module>), plus a stub
#                                      docs/c4/<module>.dsl when absent
# Exit 0 = done, 1 = refused. Never overwrites a module file; never touches
# lines outside the ov:includes markers.
set -euo pipefail

# shellcheck source=SCRIPTDIR/ov-common.sh
source "$(dirname "${BASH_SOURCE[0]}")/ov-common.sh"

WS_REL=docs/workspace.dsl
WS="$OV_ROOT/$WS_REL"
BEGIN_RE='//[[:space:]]*ov:includes:begin[[:space:]]*$'
END_RE='//[[:space:]]*ov:includes:end[[:space:]]*$'

cmd_skeleton() {
  ov_copy_template ov-workspace.dsl "$WS_REL"
}

cmd_includes() {
  local want=${1:-} all m mods=() nb ne lb le indent tmp existed
  [[ $# -le 1 ]] || ov_die "usage: ov-c4.sh includes [<module>]"
  all=$("$(dirname "${BASH_SOURCE[0]}")/ov-srs.sh" list) || exit 1
  if [[ -n $want ]]; then
    grep -qxF -- "$want" <<<"$all" ||
      ov_die "module '$want' is not in docs/srs/SRS.md (modules: $(paste -sd' ' - <<<"$all"))"
    mods=("$want")
  else
    while IFS= read -r m; do mods+=("$m"); done <<<"$all"
  fi

  [[ -f $WS ]] || ov_die "$WS_REL not found: run /speckit-ov-c4-context first"
  nb=$(grep -cE "$BEGIN_RE" "$WS" || true)
  ne=$(grep -cE "$END_RE" "$WS" || true)
  ((nb == 1 && ne == 1)) ||
    ov_die "$WS_REL needs exactly one '// ov:includes:begin' and one '// ov:includes:end' marker line; restore them inside the model block"
  lb=$(grep -nE "$BEGIN_RE" "$WS" | cut -d: -f1)
  le=$(grep -nE "$END_RE" "$WS" | cut -d: -f1)
  ((lb < le)) || ov_die "the ov:includes markers in $WS_REL are out of order; restore them"
  indent=$(sed -n "${lb}p" "$WS" | sed -E 's|//.*$||')

  # New region: the lines already there plus the wanted includes, sorted, once.
  tmp=$(ov_tmpfile)
  {
    sed -n "1,${lb}p" "$WS"
    {
      # (a backwards sed range would still print one line)
      ((le - lb < 2)) || sed -n "$((lb + 1)),$((le - 1))p" "$WS" | sed -E 's/^[[:space:]]+//; /^$/d'
      for m in "${mods[@]}"; do printf '!include c4/%s.dsl\n' "$m"; done
    } | LC_ALL=C sort -u | sed "s|^|$indent|"
    sed -n "${le},\$p" "$WS"
  } >"$tmp"
  ov_write_if_changed "$WS" "$tmp"
  if ((OV_CHANGED)); then
    printf 'updated: %s (include lines)\n' "$WS_REL"
  else
    printf 'unchanged: %s\n' "$WS_REL"
  fi

  mkdir -p "$OV_ROOT/docs/c4"
  for m in "${mods[@]}"; do
    existed="$OV_ROOT/docs/c4/$m.dsl"
    if [[ -e $existed ]]; then
      printf 'exists, kept: docs/c4/%s.dsl\n' "$m"
    else
      printf '// C3/C4 for %s: written by /speckit-ov-c4-code\n' "$m" >"$existed"
      printf 'created: docs/c4/%s.dsl\n' "$m"
    fi
  done
}

case ${1:-} in
  skeleton)
    [[ $# -eq 1 ]] || ov_die "usage: ov-c4.sh skeleton"
    cmd_skeleton
    ;;
  includes)
    shift
    cmd_includes "$@"
    ;;
  *) ov_die "usage: ov-c4.sh skeleton | includes [<module>]" ;;
esac
