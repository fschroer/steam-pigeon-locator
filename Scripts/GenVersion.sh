#!/bin/bash

# Resolve project root relative to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT="$(dirname "$SCRIPT_DIR")"

# git describe must run inside the repo, not the caller's working directory
cd "$ROOT" || exit 1

VERSION="$(date +%Y.%m.%d)-$(git describe --tags --long --dirty --always)"

OUTFILE="$ROOT/Core/Inc/version.h"

CONTENT="#pragma once
static const char GIT_VERSION[] = \"${VERSION}\";"

# version.h is a prerequisite of Communication.o, so rewrite it only when the
# stamp actually changed -- an unconditional write would bump its mtime and
# force a recompile and relink on every single build.
if [ -f "$OUTFILE" ] && [ "$(cat "$OUTFILE")" = "$CONTENT" ]; then
	exit 0
fi

printf '%s\n' "$CONTENT" > "$OUTFILE"
