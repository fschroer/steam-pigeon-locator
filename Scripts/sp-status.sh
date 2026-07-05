#!/usr/bin/env bash
# sp-status.sh — one-shot commit/push status across the three Steam Pigeon repos.
#
# Answers "is everything saved?" in one command: for the locator, receiver, and
# app repos it prints the branch, short HEAD, whether the working tree is clean,
# and whether the branch is ahead/behind its upstream (i.e. unpushed / unpulled).
#
# Usage:
#   Scripts/sp-status.sh          # human-readable report
#   Scripts/sp-status.sh --hint   # also print the SESSION_HANDOFF "Git state" lines
#
# Exit code is 0 only when every repo is clean AND in sync with its upstream,
# so it can gate an end-of-session check.

set -u

# Repo paths (override via env if the checkout ever moves).
REPOS=(
  "app|${SP_APP_DIR:-/c/Users/ftsch/StudioProjects/rocket-flight-manager}"
  "locator|${SP_LOCATOR_DIR:-/c/STM32_Projects/Locator}"
  "receiver|${SP_RECEIVER_DIR:-/c/STM32_Projects/Receiver}"
)

problems=0
handoff_lines=()

for entry in "${REPOS[@]}"; do
  name="${entry%%|*}"
  dir="${entry#*|}"

  if ! git -C "$dir" rev-parse --git-dir >/dev/null 2>&1; then
    printf '%-9s  !! not a git repo: %s\n' "$name" "$dir"
    problems=$((problems + 1))
    continue
  fi

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
