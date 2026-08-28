#!/bin/bash

# Resolve project root relative to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT="$(dirname "$SCRIPT_DIR")"

# git describe must run inside the repo, not the caller's working directory
cd "$ROOT" || exit 1

# Refresh the index before asking about --dirty.  This is hygiene, not a fix for a
# demonstrated bug, and the distinction is worth recording so nobody credits it
# with more than it does.
#
# `git describe --dirty` decides via `git diff-index`, which does not refresh the
# index first.  That is the same reason git's own require_clean_work_tree refreshes
# before testing, and it is cheap, so it is done here too.  What it does NOT
# explain is a stamp that says dirty on a tree that is clean: diff-index falls back
# to comparing CONTENT when stat data differs, so a touched-but-unchanged file was
# tried on 2026-08-27 and did not produce a false -dirty.
#
# If a build ever stamps -dirty against a clean tree, look elsewhere first -- most
# likely the binary is older than the commit, since the stamp is fixed at BUILD
# time and reinstalling does not restamp it.
#
# A non-zero exit means genuinely modified files -- the case --dirty exists to
# report -- so the result is deliberately ignored rather than checked.
git update-index -q --refresh || true

DESCRIBE="$(git describe --tags --long --dirty --always)"
VERSION="$(date +%Y.%m.%d)-${DESCRIBE}"

# A dirty tree describes IDENTICALLY for as long as it stays dirty: --dirty says
# only THAT something is uncommitted, never what.  So every build made between
# two commits on the same day carries a byte-identical stamp, and the version a
# device reports stops distinguishing the firmware actually on it.
#
# That is not theoretical.  Three different firmwares all reported
# 2026.08.14-9c24f54-dirty during one bench session, and "which build is on the
# board?" became unanswerable from the board -- which cost a debugging round
# that blamed a locator for what turned out to be a receiver bug.
#
# So a dirty build gets the time of day appended and a clean one does not.  A
# tagged or committed build keeps a stamp that is reproducible and comparable
# between two people; a development build gets uniqueness instead, which is the
# only property a development build actually needs.  ~31 characters against the
# 64-byte version fields in StartupMessage / VersionInfoMessage, so there is
# room for tags to lengthen it later.
case "$DESCRIBE" in
*-dirty) VERSION="${VERSION}.$(date +%H%M%S)" ;;
esac

OUTFILE="$ROOT/Core/Inc/version.h"

CONTENT="#pragma once
static const char GIT_VERSION[] = \"${VERSION}\";"

# version.h is a prerequisite of Communication.o, so rewrite it only when the
# stamp actually changed -- an unconditional write would bump its mtime and
# force a recompile and relink on every single build.
#
# On a CLEAN tree that still holds and nothing is rebuilt.  On a dirty tree the
# stamp now changes every second, so this guard stops catching and each build
# recompiles Communication.cpp and relinks -- a few seconds, deliberately spent:
# a development build you cannot identify is worth less than the time it saves.
if [ -f "$OUTFILE" ] && [ "$(cat "$OUTFILE")" = "$CONTENT" ]; then
	exit 0
fi

printf '%s\n' "$CONTENT" > "$OUTFILE"
