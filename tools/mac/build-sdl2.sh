#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Builds SDL2 as a UNIVERSAL (arm64 + x86_64) shared library with a macOS 11.0
# deployment target, into a private prefix. Homebrew's copy is single-
# architecture and built for the runner's own macOS version, so a release
# cannot be linked against it.
#
#   tools/mac/build-sdl2.sh [prefix]      (default: build-mac-deps/prefix)
#
# Idempotent: a prefix that already holds the library is left alone, which is
# what lets CI cache it.
set -euo pipefail

SDL2_VER=2.32.10
SDL2_SHA=5f5993c530f084535c65a6879e9b26ad441169b3e25d789d83287040a9ca5165

here="$(cd "$(dirname "$0")/../.." && pwd)"
prefix="${1:-$here/build-mac-deps/prefix}"
mkdir -p "$prefix"
prefix="$(cd "$prefix" && pwd)"
work="$(dirname "$prefix")/src"

if [ -e "$prefix/lib/libSDL2.dylib" ] && [ -x "$prefix/bin/sdl2-config" ]; then
    echo "SDL2 already built in $prefix"
    exit 0
fi

mkdir -p "$work"
tgz="$work/SDL2-$SDL2_VER.tar.gz"
[ -f "$tgz" ] || curl -fsSL -o "$tgz" \
    "https://github.com/libsdl-org/SDL/releases/download/release-$SDL2_VER/SDL2-$SDL2_VER.tar.gz"
echo "$SDL2_SHA  $tgz" | shasum -a 256 -c -
tar -xzf "$tgz" -C "$work"

cmake -S "$work/SDL2-$SDL2_VER" -B "$work/sdl2-build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 \
    -DCMAKE_INSTALL_PREFIX="$prefix" \
    -DCMAKE_IGNORE_PREFIX_PATH="/opt/homebrew;/usr/local" \
    -DSDL_SHARED=ON -DSDL_STATIC=OFF -DSDL_TEST=OFF
cmake --build "$work/sdl2-build" -j "$(sysctl -n hw.ncpu)"
cmake --install "$work/sdl2-build"

ls -l "$prefix/lib" "$prefix/bin"
lipo -info "$prefix/lib/libSDL2.dylib"
