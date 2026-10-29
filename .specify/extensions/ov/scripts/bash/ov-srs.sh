#!/usr/bin/env bash
# /speckit-ov-srs: the SRS template and its deterministic split into module
# specs (FR-020–FR-023, research R-9, contracts/srs-format.md).
#
# usage: ov-srs.sh template   copy the template to docs/srs/SRS.md (never overwrites)
#        ov-srs.sh split      write docs/srs/modules/<name>.md per module
#        ov-srs.sh list       print the module names in SRS order
#        ov-srs.sh context    changed paths on stdin -> the module specs they
#                             belong to, or the shared SRS sections (FR-057)
# Exit 0 = done, 1 = refused or invalid SRS (lines named), 3 = TODO( left.
set -euo pipefail

# shellcheck source=SCRIPTDIR/ov-common.sh
source "$(dirname "${BASH_SOURCE[0]}")/ov-common.sh"
# shellcheck source=SCRIPTDIR/ov-aiignore-match.sh
source "$(dirname "${BASH_SOURCE[0]}")/ov-aiignore-match.sh"

SRS_REL=docs/srs/SRS.md
SRS="$OV_ROOT/$SRS_REL"
MODS_REL=docs/srs/modules
MODS="$OV_ROOT/$MODS_REL"
HEADER='<!-- GENERATED from SRS.md, do not edit -->'
NAME_RE='^[a-z0-9][a-z0-9_-]*$'

# srs_scan <file>: one tab-separated record per structural line, ignoring
# fenced code blocks. Names and globs drop inline-code backticks and the "\_"
# escape that Markdown exporters write:
#   M <line> <name>     a "## Module:" heading
#   P <line> <globs>    a "Paths:" line inside a module
#   S <line> <heading>  another level-2 heading after the first module
srs_scan() {
  awk '
    function spaces(s,   i) { i = 0; while (substr(s, i + 1, 1) == " ") i++; return i }
    function run(s, c,   i) { i = 0; while (substr(s, i + 1, 1) == c) i++; return i }
    function plain(s) { gsub(/`/, "", s); gsub(/\\_/, "_", s); return s }
    {
      line = $0; sub(/\r$/, "", line)
      ls = spaces(line)
      if (ls <= 3) {
        rest = substr(line, ls + 1); c = substr(rest, 1, 1)
        if (c == "`" || c == "~") {
          n = run(rest, c)
          if (n >= 3) {
            if (fc == "") { fc = c; fn = n; next }
            if (c == fc && n >= fn && substr(rest, n + 1) ~ /^[ \t]*$/) { fc = ""; next }
          }
        }
      }
      if (fc != "") next
      if (line ~ /^## Module:/) {
        v = line; sub(/^## Module:[ \t]*/, "", v); sub(/[ \t]+$/, "", v); v = plain(v)
        printf "M\t%d\t%s\n", NR, v; inmod = 1; next
      }
      if (inmod && line ~ /^##([ \t]|$)/) { printf "S\t%d\t%s\n", NR, line; next }
      if (inmod && line ~ /^Paths:/) {
        v = line; sub(/^Paths:[ \t]*/, "", v); sub(/[ \t]+$/, "", v); v = plain(v)
        printf "P\t%d\t%s\n", NR, v
      }
    }
  ' "$1"
}

# srs_load <file>: fills MOD_NAMES, MOD_LINES, MOD_PATHS (globs per module),
# STRAY ("<line>\t<heading>").
srs_load() {
  local kind ln val i=-1
  MOD_NAMES=()
  MOD_LINES=()
  MOD_PATHS=()
  STRAY=()
  while IFS=$'\t' read -r kind ln val; do
    case $kind in
      M)
        i=$((i + 1))
        MOD_NAMES[i]=$val
        MOD_LINES[i]=$ln
        MOD_PATHS[i]=""
        ;;
      P) ((i < 0)) || MOD_PATHS[i]="${MOD_PATHS[i]:+${MOD_PATHS[i]} }$val" ;;
      S) STRAY+=("$ln"$'\t'"$val") ;;
    esac
  done < <(srs_scan "$1")
}

# srs_validate [names-only]: prints one "line N: ..." error per problem.
srs_validate() {
  local names_only=${1:-} i j s errs=()
  ((${#MOD_NAMES[@]})) || errs+=("no '## Module: <name>' section found")
  for i in "${!MOD_NAMES[@]}"; do
    if [[ ! ${MOD_NAMES[i]} =~ $NAME_RE ]]; then
      errs+=("line ${MOD_LINES[i]}: invalid module name '${MOD_NAMES[i]}' (must match $NAME_RE)")
    fi
    for ((j = 0; j < i; j++)); do
      if [[ ${MOD_NAMES[j]} == "${MOD_NAMES[i]}" ]]; then
        errs+=("line ${MOD_LINES[i]}: duplicate module name '${MOD_NAMES[i]}' (first at line ${MOD_LINES[j]})")
        break
      fi
    done
    if [[ -z $names_only && -z ${MOD_PATHS[i]} ]]; then
      errs+=("line ${MOD_LINES[i]}: module '${MOD_NAMES[i]}' has no non-empty 'Paths:' line")
    fi
  done
  if [[ -z $names_only ]]; then
    for s in "${STRAY[@]+"${STRAY[@]}"}"; do
      errs+=("line ${s%%$'\t'*}: level-2 heading '${s#*$'\t'}' after the first module; only '## Module: <name>' is allowed there")
    done
  fi
  ((${#errs[@]} == 0)) || printf '%s\n' "${errs[@]}"
}

srs_require() {
  [[ -f $SRS ]] || ov_die "$SRS_REL not found: run /speckit-ov-srs to start from the template"
}

srs_fail_invalid() { # srs_fail_invalid <errors>
  printf 'ov: %s is not valid:\n' "$SRS_REL" >&2
  printf '%s\n' "$1" | sed 's/^/  /' >&2
  exit 1
}

cmd_template() {
  ov_copy_template ov-srs.md "$SRS_REL"
}

cmd_list() {
  local errs
  srs_require
  srs_load "$SRS"
  errs=$(srs_validate names-only)
  [[ -z $errs ]] || srs_fail_invalid "$errs"
  printf '%s\n' "${MOD_NAMES[@]}"
}

cmd_split() {
  local todos errs total first i start end name out tmp existed f base
  srs_require
  todos=$(grep -n 'TODO(' "$SRS" || true)
  if [[ -n $todos ]]; then
    printf 'ov: %s still has TODO( placeholders. Replace them, then split again:\n' "$SRS_REL" >&2
    printf '%s\n' "$todos" | sed -E 's/^([0-9]+):/  line \1: /' >&2
    exit 3
  fi
  srs_load "$SRS"
  errs=$(srs_validate)
  [[ -z $errs ]] || srs_fail_invalid "$errs"

  total=$(awk 'END { print NR }' "$SRS")
  first=${MOD_LINES[0]}
  out=$(mktemp -d "${TMPDIR:-/tmp}/ov-srs.XXXXXX")
  # shellcheck disable=SC2064 # expand now: $out is local
  trap "rm -rf '$out'" EXIT
  for i in "${!MOD_NAMES[@]}"; do
    start=${MOD_LINES[i]}
    if ((i + 1 < ${#MOD_NAMES[@]})); then end=$((MOD_LINES[i + 1] - 1)); else end=$total; fi
    {
      printf '%s\n' "$HEADER"
      ((first == 1)) || sed -n "1,$((first - 1))p" "$SRS"
      sed -n "${start},${end}p" "$SRS"
    } >"$out/${MOD_NAMES[i]}.md"
  done

  # Everything is rendered and valid: now move it into place.
  mkdir -p "$MODS"
  for i in "${!MOD_NAMES[@]}"; do
    name=${MOD_NAMES[i]}
    existed=0
    [[ -e $MODS/$name.md ]] && existed=1
    tmp=$(ov_tmpfile)
    cp "$out/$name.md" "$tmp"
    ov_write_if_changed "$MODS/$name.md" "$tmp"
    if ((OV_CHANGED == 0)); then
      printf 'unchanged: %s/%s.md\n' "$MODS_REL" "$name"
    elif ((existed)); then
      printf 'updated: %s/%s.md\n' "$MODS_REL" "$name"
    else
      printf 'created: %s/%s.md\n' "$MODS_REL" "$name"
    fi
  done

  for f in "$MODS"/*.md; do
    [[ -e $f ]] || continue
    base=${f##*/}
    base=${base%.md}
    case " ${MOD_NAMES[*]} " in
      *" $base "*) ;;
      *) printf 'orphan: %s/%s.md has no "## Module: %s" in %s; left in place, delete it if the module is gone\n' \
        "$MODS_REL" "$base" "$base" "$SRS_REL" ;;
    esac
  done
}

# cmd_context: print each docs/srs/modules/<name>.md whose Paths: globs match
# a changed path read from stdin, in sorted order, each after a
# "=== module: <name> ===" line. When none matches, print the shared sections
# of docs/srs/SRS.md instead. Always exits 0.
cmd_context() {
  local changed f name pats matched=0 first rc
  changed=$(cat)
  pats=$(ov_tmpfile)
  # shellcheck disable=SC2064 # expand now: $pats is local
  trap "rm -f '$pats'" EXIT
  if [[ -n ${changed//[$' \t\n']/} ]]; then
    while IFS= read -r f; do
      name=${f##*/}
      name=${name%.md}
      srs_load "$f"
      ((${#MOD_PATHS[@]})) || continue
      # One gitignore glob per line.
      printf '%s\n' "${MOD_PATHS[@]}" | tr -s ' \t' '\n' | sed '/^$/d' >"$pats"
      [[ -s $pats ]] || continue
      rc=0
      ov_aiignore_match "$pats" --stdin <<<"$changed" >/dev/null || rc=$?
      ((rc <= 1)) || ov_die "the Paths: matcher failed on $MODS_REL/$name.md"
      if ((rc == 0)); then
        printf '=== module: %s ===\n' "$name"
        cat "$f"
        matched=1
      fi
    done < <(find "$MODS" -maxdepth 1 -type f -name '*.md' 2>/dev/null | LC_ALL=C sort)
  fi
  ((matched == 0)) || return 0
  if [[ ! -f $SRS ]]; then
    printf 'ov: no module matched and %s is missing; no SRS context\n' "$SRS_REL" >&2
    return 0
  fi
  srs_load "$SRS"
  printf '=== shared SRS sections (%s) ===\n' "$SRS_REL"
  if ((${#MOD_LINES[@]})); then
    first=${MOD_LINES[0]}
    ((first == 1)) || sed -n "1,$((first - 1))p" "$SRS"
  else
    cat "$SRS"
  fi
}

case ${1:-} in
  template) cmd_template ;;
  split) cmd_split ;;
  list) cmd_list ;;
  context) cmd_context ;;
  *) ov_die "usage: ov-srs.sh template | split | list | context" ;;
esac
