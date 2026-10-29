#!/usr/bin/env bash
# /speckit-ov-pr: record the PR and authorship policy (FR-050–FR-058).
#
# usage: ov-pr.sh [--authorship coauthor|trailer|none] [--auto-merge on|off] [--ai-review on|off]
# Non-interactive: the command prompt asks the questions first. A flag left out
# keeps the current config value (or its default when the value is invalid).
# Writes the ov config, CONTRIBUTING.md (between its ov markers only), the
# ruleset template, the optional workflows and, in trailer and none modes,
# .claude/settings.json attribution. Installs no git hooks.
# Exit 0 = done, 1 = refused (nothing written).
set -euo pipefail

# shellcheck source=SCRIPTDIR/ov-common.sh
source "$(dirname "${BASH_SOURCE[0]}")/ov-common.sh"

TRAILER_COAUTHOR="Co-Authored-By: Claude <noreply@anthropic.com>"
TRAILER_ASSISTED="Assisted-by: claude-code"
BEGIN='<!-- ov:pr:begin -->'
END='<!-- ov:pr:end -->'

usage="usage: ov-pr.sh [--authorship coauthor|trailer|none] [--auto-merge on|off] [--ai-review on|off]"

# current <key> <allowed> <default>: the config value if valid, else the default.
current() {
  local v
  v=$(ov_config_get "$1" 2>/dev/null || true)
  if [[ "|$2|" == *"|$v|"* && -n $v ]]; then printf '%s\n' "$v"; else printf '%s\n' "$3"; fi
}
authorship=$(current authorship "coauthor|trailer|none" coauthor)
auto_merge=$(current auto_merge "on|off" off)
ai_review=$(current ai_review "on|off" on)

while (($#)); do
  (($# >= 2)) || ov_die "$1 needs a value. $usage"
  case $1 in
    --authorship) authorship=$2 ;;
    --auto-merge) auto_merge=$2 ;;
    --ai-review) ai_review=$2 ;;
    *) ov_die "unknown argument: $1. $usage" ;;
  esac
  shift 2
done
[[ $authorship == coauthor || $authorship == trailer || $authorship == none ]] ||
  ov_die "invalid authorship: $authorship (allowed: coauthor | trailer | none)"
[[ $auto_merge == on || $auto_merge == off ]] ||
  ov_die "invalid auto-merge: $auto_merge (allowed: on | off)"
[[ $ai_review == on || $ai_review == off ]] ||
  ov_die "invalid ai-review: $ai_review (allowed: on | off)"

contrib="$OV_ROOT/CONTRIBUTING.md"
settings="$OV_ROOT/.claude/settings.json"
ruleset_rel=.github/ov-ruleset-main.json
ruleset="$OV_ROOT/$ruleset_rel"

# --- checks before any write ---------------------------------------------------
contrib_mode=new
if [[ -f $contrib ]]; then
  nb=$(grep -cxF "$BEGIN" "$contrib" || true)
  ne=$(grep -cxF "$END" "$contrib" || true)
  if ((nb == 0 && ne == 0)); then
    contrib_mode=append
  elif ((nb == 1 && ne == 1)) &&
    (($(grep -nxF "$BEGIN" "$contrib" | cut -d: -f1) < $(grep -nxF "$END" "$contrib" | cut -d: -f1))); then
    contrib_mode=replace
  else
    ov_die "CONTRIBUTING.md has a broken $BEGIN / $END pair; fix the markers (one of each, in order) and run again"
  fi
fi
if [[ -f $settings ]] && ! jq -e . "$settings" >/dev/null 2>&1; then
  ov_die ".claude/settings.json is not valid JSON; fix it and run again"
fi
for t in ov-contributing.md ov-ruleset-main.json ov-ai-review.yml ov-automerge.yml; do
  [[ -f $OV_TEMPLATES/$t ]] || ov_die "template missing: $OV_TEMPLATES/$t (reinstall ov)"
done

report() { # report <action> <path>
  printf '%s: %s\n' "$1" "$2"
}
report_write() { # report_write <path> <existed>
  if ((OV_CHANGED == 0)); then
    report unchanged "$1"
  elif (($2)); then
    report updated "$1"
  else
    report created "$1"
  fi
}

# --- config ---------------------------------------------------------------------
cfg_existed=0
[[ -e $OV_CONFIG ]] && cfg_existed=1
before=$( [[ -f $OV_CONFIG ]] && ov_sha256 "$OV_CONFIG" || true)
ov_config_set authorship "$authorship"
ov_config_set auto_merge "$auto_merge"
ov_config_set ai_review "$ai_review"
ov_config_set configured true
after=$(ov_sha256 "$OV_CONFIG")
OV_CHANGED=0
[[ $before == "$after" ]] || OV_CHANGED=1
report_write .specify/extensions/ov/ov-config.yml "$cfg_existed"

# --- CONTRIBUTING.md --------------------------------------------------------------
case $authorship in
  coauthor) author_note="Claude Code ends every commit it makes with the trailer \`$TRAILER_COAUTHOR\`." ;;
  trailer) author_note="Claude Code ends every commit it makes with the trailer \`$TRAILER_ASSISTED\`." ;;
  none) author_note="Agent commits carry no trailer and no co-author: they are made under the maintainer's own identity, so CI cannot tell them from human commits and \`ov-aiignore\` sees no agent commit. The Claude Code hook is the only guard on read-only paths; the owner reviews every commit." ;;
esac
if [[ $auto_merge == on ]]; then
  am_note="Same-repo PRs merge automatically once the required checks pass; fork PRs are never auto-merged."
else
  am_note="A human merges every PR."
fi
if [[ $ai_review == on ]]; then
  ai_note="Every same-repo PR is reviewed by Claude Code against the module SRS (\`ov-ai-review\`, a required check that needs the \`ANTHROPIC_API_KEY\` repository secret). Fork PRs pass with a skip notice."
else
  ai_note="No AI review runs, and \`ov-ai-review\` is not a required check."
fi
filled=$(ov_tmpfile)
OV_A=$authorship OV_AN=$author_note OV_M=$auto_merge OV_MN=$am_note OV_R=$ai_review OV_RN=$ai_note awk '
  {
    gsub(/\{\{OV_AUTHORSHIP\}\}/, ENVIRON["OV_A"])
    gsub(/\{\{OV_AUTHORSHIP_NOTE\}\}/, ENVIRON["OV_AN"])
    gsub(/\{\{OV_AUTO_MERGE_NOTE\}\}/, ENVIRON["OV_MN"])
    gsub(/\{\{OV_AUTO_MERGE\}\}/, ENVIRON["OV_M"])
    gsub(/\{\{OV_AI_REVIEW_NOTE\}\}/, ENVIRON["OV_RN"])
    gsub(/\{\{OV_AI_REVIEW\}\}/, ENVIRON["OV_R"])
    print
  }
' "$OV_TEMPLATES/ov-contributing.md" >"$filled"
blockf=$(ov_tmpfile)
awk -v b="$BEGIN" -v e="$END" '$0 == b { on = 1 } on { print } $0 == e { on = 0 }' "$filled" >"$blockf"
out=$(ov_tmpfile)
case $contrib_mode in
  new) cp "$filled" "$out" ;;
  append)
    cp "$contrib" "$out"
    [[ ! -s $out || -z $(tail -c 1 "$out") ]] || printf '\n' >>"$out"
    printf '\n' >>"$out"
    cat "$blockf" >>"$out"
    ;;
  replace)
    OV_BLOCK=$blockf awk -v b="$BEGIN" -v e="$END" '
      $0 == b { while ((getline l < ENVIRON["OV_BLOCK"]) > 0) print l; skip = 1; next }
      $0 == e { skip = 0; next }
      !skip { print }
    ' "$contrib" >"$out"
    ;;
esac
rm -f "$filled" "$blockf"
existed=0
[[ -e $contrib ]] && existed=1
ov_write_if_changed "$contrib" "$out"
report_write CONTRIBUTING.md "$existed"

# --- ruleset template -------------------------------------------------------------
out=$(ov_tmpfile)
jq --arg ai "$ai_review" '
  .rules |= map(
    if .type == "required_status_checks" and $ai == "on"
    then .parameters.required_status_checks += [{"context": "ov-ai-review"}]
    else . end)
' "$OV_TEMPLATES/ov-ruleset-main.json" >"$out"
if [[ -e $ruleset && $(jq -r '.name // empty' "$ruleset" 2>/dev/null || true) != ov-main ]]; then
  rm -f "$out"
  report "exists, kept (not an ov ruleset: .name is not ov-main)" "$ruleset_rel"
else
  existed=0
  [[ -e $ruleset ]] && existed=1
  ov_write_if_changed "$ruleset" "$out"
  report_write "$ruleset_rel" "$existed"
fi

# --- optional workflows -------------------------------------------------------------
workflow() { # workflow <on|off> <template>
  local rel=".github/workflows/$2" dest
  dest="$OV_ROOT/$rel"
  if [[ $1 == on ]]; then
    ov_copy_template "$2" "$rel"
  elif [[ -e $dest ]]; then
    if cmp -s "$dest" "$OV_TEMPLATES/$2"; then
      rm -f "$dest"
      report removed "$rel"
    else
      report "exists, kept (modified, remove it by hand if unwanted)" "$rel"
    fi
  fi
}
workflow "$ai_review" ov-ai-review.yml
workflow "$auto_merge" ov-automerge.yml

# --- .claude/settings.json attribution (trailer and none modes) -------------------------
# ov owns attribution.commit when it is the Assisted-by trailer or "", and attribution.pr
# when it is "": those are removed first, then set again for the current mode.
out=$(ov_tmpfile)
src=$settings
[[ -f $src ]] || src=/dev/null
jq --arg mode "$authorship" --arg t "$TRAILER_ASSISTED" '
  (. // {})
  | if .attribution.commit? == $t or .attribution.commit? == "" then del(.attribution.commit) else . end
  | if .attribution.pr? == "" then del(.attribution.pr) else . end
  | if $mode == "trailer" then .attribution.commit = $t
    elif $mode == "none" then .attribution.commit = "" | .attribution.pr = ""
    else . end
  | if (.attribution? | type) == "object" and (.attribution | length) == 0 then del(.attribution) else . end
' "$src" >"$out"
if [[ -f $settings ]] && [[ $(jq -S . "$settings") == $(jq -S . "$out") ]]; then
  rm -f "$out"
  report unchanged .claude/settings.json
elif [[ ! -f $settings && $(jq -c . "$out") == "{}" ]]; then
  rm -f "$out"
else
  existed=0
  [[ -e $settings ]] && existed=1
  ov_write_if_changed "$settings" "$out"
  report_write .claude/settings.json "$existed"
fi

printf 'ov: authorship %s, auto-merge %s, ai_review %s\n' "$authorship" "$auto_merge" "$ai_review"
printf 'ov: commit these files as a human commit (no agent trailer)\n'
