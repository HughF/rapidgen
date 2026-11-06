#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Builds the universal (arm64 + x86_64) executable with the ordinary Makefile:
# one pass per architecture against the private SDL2 from build-sdl2.sh, then
# lipo. (clang cannot write dependency files for a two-architecture compile,
# so a single -arch arm64 -arch x86_64 pass is not an option.)
#
#   tools/mac/build-universal.sh <deps-prefix> <out-dir>
#
# Leaves <out-dir>/rapidgen (universal), plus the two single-architecture builds.
set -euo pipefail

here="$(cd "$(dirname "$0")/../.." && pwd)"
prefix="$(cd "${1:?deps prefix}" && pwd)"
mkdir -p "${2:?out dir}"
out="$(cd "$2" && pwd)"

export MACOSX_DEPLOYMENT_TARGET=11.0
# The Makefile finds SDL2 through sdl2-config: make it the private one.
export PATH="$prefix/bin:$PATH"
[ "$(command -v sdl2-config)" = "$prefix/bin/sdl2-config" ] || {
    echo "sdl2-config is not the one in $prefix/bin" >&2; exit 1; }

# SDL_LIBS is used only at link time. The rpath lets the executable find the
# SDL2 dylib that make-app.sh puts in Contents/Frameworks; the header padding
# leaves install_name_tool room to rewrite load commands afterwards.
sdl_libs="$(sdl2-config --libs) -Wl,-headerpad_max_install_names -Wl,-rpath,@executable_path/../Frameworks"

cd "$here"
for arch in arm64 x86_64; do
    make clean >/dev/null
    make -j"$(sysctl -n hw.ncpu)" CC="cc -arch $arch" SDL_LIBS="$sdl_libs"
    cp rapidgen "$out/rapidgen-$arch"
done
make clean >/dev/null

lipo -create "$out/rapidgen-arm64" "$out/rapidgen-x86_64" -output "$out/rapidgen"
lipo -info "$out/rapidgen"
