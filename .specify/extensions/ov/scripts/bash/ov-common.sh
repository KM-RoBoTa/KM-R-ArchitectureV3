#!/usr/bin/env bash
# ov shared library. Source it; it only defines functions and path variables.
#
#   OV_ROOT       project root (ov_project_root)
#   OV_EXT_DIR    $OV_ROOT/.specify/extensions/ov
#   OV_TEMPLATES  $OV_EXT_DIR/templates
#   OV_CONFIG     $OV_EXT_DIR/ov-config.yml

# ov_project_root: CLAUDE_PROJECT_DIR, else the git top level, else pwd.
ov_project_root() {
  if [[ -n ${CLAUDE_PROJECT_DIR:-} ]]; then
    printf '%s\n' "${CLAUDE_PROJECT_DIR%/}"
  elif git rev-parse --show-toplevel 2>/dev/null; then
    :
  else
    pwd
  fi
}

# ov_init_paths [root]: (re)compute the path variables.
ov_init_paths() {
  OV_ROOT=${1:-$(ov_project_root)}
  OV_EXT_DIR="$OV_ROOT/.specify/extensions/ov"
  OV_TEMPLATES="$OV_EXT_DIR/templates"
  OV_CONFIG="$OV_EXT_DIR/ov-config.yml"
}

# ov_die <msg>: print "ov: <msg>" on stderr and exit 1.
ov_die() {
  printf 'ov: %s\n' "$*" >&2
  exit 1
}

# ov_sha256 <file>: print the file's sha256.
ov_sha256() {
  local h
  if command -v sha256sum >/dev/null 2>&1; then
    read -r h _ < <(sha256sum <"$1")
  else
    read -r h _ < <(shasum -a 256 <"$1")
  fi
  [[ -n $h ]] || return 1
  printf '%s\n' "$h"
}

# ov_tmpfile: print a new temp file path (outside the project).
ov_tmpfile() {
  local base=${TMPDIR:-/tmp}
  mktemp "${base%/}/ov-tmp.XXXXXX"
}

# ov_write_if_changed <dest> <tmpfile>: put tmpfile's content at dest only
# when it differs (SC-006). Keeps dest's mode. Sets OV_CHANGED to 1 or 0.
# The tmpfile is always consumed.
# shellcheck disable=SC2034 # OV_CHANGED is read by callers
ov_write_if_changed() {
  local dest=$1 tmp=$2
  if [[ -f $dest ]] && cmp -s "$tmp" "$dest"; then
    rm -f "$tmp"
    OV_CHANGED=0
    return 0
  fi
  mkdir -p "$(dirname "$dest")"
  cat "$tmp" >"$dest"
  rm -f "$tmp"
  OV_CHANGED=1
}

# ov_copy_template <template-name> <dest-relpath>: copy a template into the
# project, never over an existing file (FR-007).
ov_copy_template() {
  local name=$1 rel=$2 src dest
  src="$OV_TEMPLATES/$name"
  dest="$OV_ROOT/$rel"
  [[ -f $src ]] || ov_die "template missing: $src (reinstall ov)"
  if [[ -e $dest || -L $dest ]]; then
    printf 'exists, kept: %s\n' "$rel"
    return 0
  fi
  mkdir -p "$(dirname "$dest")"
  cp "$src" "$dest"
  printf 'created: %s\n' "$rel"
}

_ov_config_key_ok() {
  [[ $1 =~ ^[a-z_][a-z0-9_]*$ ]]
}

# ov_config_get <key> [file]: print the value of a flat "key: value" line.
# Returns 1 when the file or the key is missing.
ov_config_get() {
  local key=$1 file=${2:-$OV_CONFIG} line value
  _ov_config_key_ok "$key" || return 1
  [[ -f $file ]] || return 1
  line=$(grep -m1 -E "^${key}:" "$file") || return 1
  value=${line#"$key":}
  # Drop a trailing comment, surrounding blanks and quotes.
  value=$(printf '%s' "$value" | sed -E 's/[[:space:]]+#.*$//; s/^[[:space:]]+//; s/[[:space:]]+$//')
  if [[ $value =~ ^\"(.*)\"$ || $value =~ ^\'(.*)\'$ ]]; then
    value=${BASH_REMATCH[1]}
  fi
  printf '%s\n' "$value"
}

# ov_config_set <key> <value> [file]: replace the key's line or append it.
# Writes only when the content changes.
ov_config_set() {
  local key=$1 value=$2 file=${3:-$OV_CONFIG} tmp src
  _ov_config_key_ok "$key" || ov_die "invalid config key: $key"
  [[ $value != *$'\n'* ]] || ov_die "config value for $key must be one line"
  src=$file
  [[ -f $src ]] || src=/dev/null
  tmp=$(ov_tmpfile)
  OV_K=$key OV_V=$value awk '
    BEGIN { k = ENVIRON["OV_K"]; v = ENVIRON["OV_V"]; done = 0 }
    !done && index($0, k ":") == 1 { print k ": " v; done = 1; next }
    { print }
    END { if (!done) print k ": " v }
  ' "$src" >"$tmp"
  ov_write_if_changed "$file" "$tmp"
}

# ov_hook_registered: the ov after_implement hook in .specify/extensions.yml.
# Returns 0 when an enabled entry runs speckit.ov.c4-code, 2 when the entry
# exists but every copy is "enabled: false", 1 when it is missing.
# Indentation-agnostic block-YAML parse (spec-kit writes block style).
ov_hook_registered() {
  local file="$OV_ROOT/.specify/extensions.yml"
  [[ -f $file ]] || return 1
  awk '
    function lead(s) { match(s, /^ */); return RLENGTH }
    function val(s) {
      sub(/^[^:]*:[ \t]*/, "", s); sub(/[ \t]+#.*$/, "", s); sub(/[ \t]+$/, "", s)
      gsub(/^["\047]|["\047]$/, "", s)
      return s
    }
    function flush() {
      if (in_item && cmd == "speckit.ov.c4-code") {
        if (tolower(en) == "false") disabled = 1; else enabled = 1
      }
      in_item = 0; cmd = ""; en = ""
    }
    /^[ \t]*(#.*)?$/ { next }
    {
      ind = lead($0); line = substr($0, ind + 1)
      if (ind == 0) {
        flush(); in_hooks = (line ~ /^hooks:[ \t]*$/); in_event = 0; ev_ind = -1
        next
      }
      if (!in_hooks) next
      if (line !~ /^-/ && (ev_ind < 0 || ind <= ev_ind)) {
        flush(); ev_ind = ind; in_event = (line ~ /^after_implement:[ \t]*$/)
        next
      }
      if (!in_event) next
      if (line ~ /^-/) {
        flush(); in_item = 1
        line = substr(line, 2); sub(/^[ \t]*/, "", line)
        if (line == "") next
      }
      if (!in_item) next
      if (line ~ /^command:/) cmd = val(line)
      else if (line ~ /^enabled:/) en = val(line)
    }
    END {
      flush()
      if (enabled) exit 0
      if (disabled) exit 2
      exit 1
    }
  ' "$file"
}

# ov_event_registered: the ov PreToolUse event in .claude/settings.json.
ov_event_registered() {
  local file="$OV_ROOT/.claude/settings.json"
  [[ -f $file ]] || return 1
  jq -e '[.hooks.PreToolUse[]? | .. | strings | select(contains("speckit.ov.guard"))] | length > 0' \
    "$file" >/dev/null 2>&1 || return 1
}

ov_init_paths
