#!/usr/bin/env bash
# sp-status.sh — one-shot commit/push status across the four Steam Pigeon repos.
#
# Answers "is everything saved?" in one command: for the locator, receiver, app
# and iOS repos it prints the branch, short HEAD, whether the working tree is
# clean, and whether the branch is ahead/behind its upstream (unpushed/unpulled).
#
# It also reports PARITY DRIFT: how many app commits have touched the Android UI
# since steam-pigeon-ios/docs/UI_PARITY.md last changed. See parity_drift() below.
#
# Usage:
#   Scripts/sp-status.sh          # human-readable report
#   Scripts/sp-status.sh --hint   # also print the SESSION_HANDOFF "Git state" lines
#
# Exit code is 0 only when every repo is clean AND in sync with its upstream,
# so it can gate an end-of-session check.

set -u

# Repo paths. Each resolves in this order, so ONE script works on every machine:
#   1. the SP_*_DIR environment variable, if set;
#   2. a sibling of this repo (the macOS layout: all four under ~/Developer/);
#   3. the original Windows absolute path.
#
# This script lives in <locator repo>/Scripts/, so the locator root is always two
# levels up. That is the anchor the sibling guesses hang off -- it is correct on
# both machines, which is why the Windows defaults below can stay untouched.
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
LOCATOR_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
SIBLINGS=$(cd -- "${LOCATOR_ROOT}/.." && pwd)

# first_repo <candidate>... -- echo the first candidate that is a git repo. If
# none is, echo the last non-empty one so the error names a real expectation.
first_repo() {
  local last=""
  for cand in "$@"; do
    [ -z "$cand" ] && continue
    last="$cand"
    if git -C "$cand" rev-parse --git-dir >/dev/null 2>&1; then
      printf '%s' "$cand"
      return
    fi
  done
  printf '%s' "$last"
}

# opt_repo <candidate>... -- like first_repo, but echoes NOTHING when no
# candidate resolves. For repos that may legitimately be absent on a given
# machine: the iOS app is only BUILT on a Mac, so a Windows box may or may not
# have a checkout. Without this, an absent optional repo reports as a hard
# failure.
#
# Optional is not the same as ignorable, and the gap between them cost something
# on 2026-08-30. The iOS repo WAS checked out on the Windows box -- under
# StudioProjects beside the app rather than beside this repo -- and had
# uncommitted work, which this script reported as "not present on this machine".
# A skipped repo prints in the same column as a clean one, so nothing looked
# wrong. Two defences, both below: every optional repo now carries the same
# absolute-path fallbacks its siblings do, and the skip line names the variable
# that would have found it.
opt_repo() {
  for cand in "$@"; do
    [ -z "$cand" ] && continue
    if git -C "$cand" rev-parse --git-dir >/dev/null 2>&1; then
      printf '%s' "$cand"
      return
    fi
  done
  printf ''
}

REPOS=(
  "app|$(first_repo "${SP_APP_DIR:-}" "${SIBLINGS}/rocket-flight-manager" "/c/Users/ftsch/StudioProjects/rocket-flight-manager")"
  "locator|$(first_repo "${SP_LOCATOR_DIR:-}" "${LOCATOR_ROOT}" "/c/STM32_Projects/Locator")"
  "receiver|$(first_repo "${SP_RECEIVER_DIR:-}" "${SIBLINGS}/steam-pigeon-receiver" "/c/STM32_Projects/Receiver")"
  "ios|$(opt_repo "${SP_IOS_DIR:-}" "${SIBLINGS}/steam-pigeon-ios" "/c/Users/ftsch/StudioProjects/steam-pigeon-ios" "${HOME}/Developer/steam-pigeon-ios")"
)

problems=0
handoff_lines=()

for entry in "${REPOS[@]}"; do
  name="${entry%%|*}"
  dir="${entry#*|}"

  if [ -z "$dir" ]; then
    # Name the escape hatch. A skipped repo prints in the same column a clean
    # one does, so the reader has to be told how to tell the two apart.
    envvar="SP_$(printf '%s' "$name" | tr '[:lower:]' '[:upper:]')_DIR"
    printf '%-9s  -- no checkout found (skipped; set %s if there is one)\n' \
      "$name" "$envvar"
    continue
  fi

  if ! git -C "$dir" rev-parse --git-dir >/dev/null 2>&1; then
    printf '%-9s  !! not a git repo: %s\n' "$name" "$dir"
    problems=$((problems + 1))
    continue
  fi

  # Kept for parity_drift(), which needs two repos at once and so cannot run
  # inside this per-repo loop.
  case "$name" in
    app) APP_DIR="$dir" ;;
    ios) IOS_DIR="$dir" ;;
  esac

  branch=$(git -C "$dir" rev-parse --abbrev-ref HEAD)
  head=$(git -C "$dir" rev-parse --short HEAD)
  subject=$(git -C "$dir" log -1 --pretty=%s)

  # Dirty working tree?
  dirty_files=$(git -C "$dir" status --porcelain)
  if [ -z "$dirty_files" ]; then
    tree="clean"
  else
    tree="DIRTY ($(printf '%s\n' "$dirty_files" | grep -c .) file(s))"
    problems=$((problems + 1))
  fi

  # Ahead/behind upstream?
  sync=""
  if upstream=$(git -C "$dir" rev-parse --abbrev-ref --symbolic-full-name @{upstream} 2>/dev/null); then
    counts=$(git -C "$dir" rev-list --left-right --count "@{upstream}...HEAD" 2>/dev/null || echo "0	0")
    behind=$(printf '%s' "$counts" | cut -f1)
    ahead=$(printf '%s' "$counts" | cut -f2)
    [ "$ahead" != "0" ]  && { sync="${sync} ahead ${ahead} (UNPUSHED)"; problems=$((problems + 1)); }
    [ "$behind" != "0" ] && { sync="${sync} behind ${behind}"; problems=$((problems + 1)); }
    [ -z "$sync" ] && sync="in sync with ${upstream}"
  else
    sync="no upstream"
  fi

  printf '%-9s  %-8s %-8s  %-22s %s\n' "$name" "$branch" "$head" "$tree" "$sync"
  [ -n "$dirty_files" ] && printf '%s\n' "$dirty_files" | sed 's/^/            /'

  handoff_lines+=("- **${name}** \`${branch}\` = \`${head}\` (${subject})")
done

# parity_drift -- how far the Android UI has moved since UI_PARITY.md last did.
#
# sp-docs Gate 3 requires that a change making the two apps DIFFER is recorded in
# steam-pigeon-ios/docs/UI_PARITY.md. That gate is prose, so it is only as good as
# remembering to run it; this is the deterministic half. On 2026-08-31 an entire new
# Android screen went unrecorded there while both other gates passed, because neither
# names that file -- it is the one doc outside the centralized Locator/docs/.
#
# CORRELATED BY DATE, because the two repos share no history and there is nothing
# else to join on. That makes this a smell rather than a proof: a large count can be
# ten commits that changed nothing a user sees, and a count of zero only means the
# file was touched recently, not that it was touched CORRECTLY. It answers "when did
# anyone last think about this?", which is the question that went unasked.
#
# Watches ui/ and strings.xml: every user-visible change lands in one of them, and a
# pure wire-format change is already covered by Gate 1 and the WireLayoutTest triad,
# so including data/ would fire constantly for changes parity does not care about.
#
# Deliberately does NOT increment $problems. This script's contract is "is everything
# saved?", and its exit code gates end-of-session checks; drift is neither unsaved nor
# necessarily wrong, and making a clean tree exit 1 would train people to ignore the
# exit code. It prints loudly instead.
parity_drift() {
  local paths="app/src/main/java/com/steampigeon/flightmanager/ui app/src/main/res/values/strings.xml"

  if [ -z "${APP_DIR:-}" ]; then
    return
  fi
  if [ -z "${IOS_DIR:-}" ]; then
    # Same rule the skipped-repo line follows: an unchecked thing must not print
    # like a checked one.
    printf 'parity     -- UI_PARITY.md drift NOT CHECKED (no iOS checkout; set SP_IOS_DIR)\n'
    return
  fi

  local last
  last=$(git -C "$IOS_DIR" log -1 --format=%cI -- docs/UI_PARITY.md 2>/dev/null)
  if [ -z "$last" ]; then
    printf 'parity     !! docs/UI_PARITY.md not found in the iOS repo history\n'
    return
  fi

  local n
  n=$(git -C "$APP_DIR" rev-list --count --since="$last" HEAD -- $paths 2>/dev/null || echo 0)

  if [ "$n" -eq 0 ]; then
    printf 'parity     UI_PARITY.md is current with the app UI (last touched %s)\n' "${last%%T*}"
  else
    printf 'parity     %s app commit(s) touched the UI since UI_PARITY.md last changed (%s)\n' \
      "$n" "${last%%T*}"
    printf '           Not necessarily wrong -- but sp-docs Gate 3 has not been run since.\n'
    git -C "$APP_DIR" log --since="$last" --pretty='             %h %s' -- $paths | head -8
  fi
}

echo
parity_drift

echo
if [ "$problems" -eq 0 ]; then
  echo "ALL CLEAN & PUSHED."
else
  echo "$problems item(s) need attention (dirty tree and/or unpushed commits)."
fi

if [ "${1:-}" = "--hint" ]; then
  echo
  echo "SESSION_HANDOFF 'Git state' lines (paste + annotate):"
  printf '%s\n' "${handoff_lines[@]}"
fi

exit $([ "$problems" -eq 0 ] && echo 0 || echo 1)
