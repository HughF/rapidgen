#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Fetch the SDL2 mingw development SDK for the Windows cross-build.
#
# Arch has no mingw SDL2 package and no mingw pkg-config, and the AUR build
# takes far longer than unpacking the upstream tarball, which is already the
# exact thing that would be built. Everything lands under tools/win/ and is
# ignored by git.
set -e

VER="${1:-2.32.10}"
DIR="$(cd "$(dirname "$0")" && pwd)"
TAR="SDL2-devel-$VER-mingw.tar.gz"
URL="https://github.com/libsdl-org/SDL/releases/download/release-$VER/$TAR"

if [ -d "$DIR/SDL2-$VER" ]; then
    echo "already have $DIR/SDL2-$VER"
    exit 0
fi

echo "fetching $URL"
curl -fL --progress-bar -o "$DIR/$TAR" "$URL"
tar xzf "$DIR/$TAR" -C "$DIR"
rm -f "$DIR/$TAR"
echo "unpacked $DIR/SDL2-$VER"
