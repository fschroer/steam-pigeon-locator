#!/bin/bash

# Resolve project root relative to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT="$(dirname "$SCRIPT_DIR")"

VERSION="$(date +%Y.%m.%d)-$(git describe --tags --long --dirty --always)"

OUTFILE="$ROOT/Core/Inc/version.h"

echo "#pragma once" > "$OUTFILE"
echo "static const char GIT_VERSION[] = \"${VERSION}\";" >> "$OUTFILE"
