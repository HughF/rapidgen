#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Assembles dist/rapidgen-<version>-macos-universal.dmg from the universal
# executable that build-universal.sh made.
#
#   tools/mac/make-app.sh <deps-prefix> <universal-executable>
#
# The .app is ad-hoc signed only (no Developer ID exists): Apple Silicon will
# not run unsigned code at all, so the ad-hoc signature is required, but
# Gatekeeper still quarantines a downloaded copy - the first launch is
# right-click > Open. Every check near the end exits non-zero, so CI cannot
# ship a bundle that still points at the build machine.
set -euo pipefail

here="$(cd "$(dirname "$0")/../.." && pwd)"
prefix="$(cd "${1:?deps prefix}" && pwd)"
exe_in="${2:?universal executable}"
ver="$(sed -n 's/.*RAPIDGEN_VERSION "\([^"]*\)".*/\1/p' "$here/src/rg_version.h")"
[ -n "$ver" ] || { echo "no version found in src/rg_version.h" >&2; exit 1; }

stage="$here/dist/mac-stage"
work="$here/dist/mac-work"   # outside the stage: the stage becomes the .dmg
app="$stage/rapidgen.app"
rm -rf "$stage" "$work"
mkdir -p "$app/Contents/MacOS" "$app/Contents/Resources" "$app/Contents/Frameworks" "$work"

cp "$exe_in" "$app/Contents/MacOS/rapidgen"
exe="$app/Contents/MacOS/rapidgen"

# Data goes in Resources, not MacOS: codesign treats everything under MacOS as
# code and rejects a .ttf there. SDL_GetBasePath() inside a .app is
# Contents/Resources/, which is where the program looks for DejaVuSans.ttf.
cp "$here/tools/linux/font/DejaVuSans.ttf"     "$app/Contents/Resources/"
cp "$here/tools/linux/font/DejaVu-LICENSE.txt" "$app/Contents/Resources/"
cp "$here/LICENSE"                             "$app/Contents/Resources/LICENSE.txt"

sed "s/@VERSION@/$ver/g" "$here/tools/mac/Info.plist.in" > "$app/Contents/Info.plist"

# Icon: the Windows icon's 256 px master, scaled to the sizes an .icns wants.
iconset="$work/rapidgen.iconset"
mkdir -p "$iconset"
for spec in 16:icon_16x16 32:icon_16x16@2x 32:icon_32x32 64:icon_32x32@2x \
            128:icon_128x128 256:icon_128x128@2x 256:icon_256x256; do
    px="${spec%%:*}"
    sips -z "$px" "$px" "$here/tools/win/icon256.png" \
        --out "$iconset/${spec#*:}.png" >/dev/null
done
iconutil -c icns "$iconset" -o "$app/Contents/Resources/rapidgen.icns"

# Bundle SDL2. cp -L: the install name points at a symlink. The file is stored
# under its install name (libSDL2-2.0.0.dylib), which is what the executable
# asks for, not under the versioned file name.
iname="$(otool -D "$prefix/lib/libSDL2.dylib" | tail -1)"
cp -L "$prefix/lib/libSDL2.dylib" "$app/Contents/Frameworks/$(basename "$iname")"

# Drop every build-machine rpath (and the bundle's own, which is put back), so
# nothing left can point outside the .app. SDL2 depends on no sibling library,
# so the dylib needs no rpath of its own.
rpaths() { otool -l "$1" | awk '/LC_RPATH/{f=1} f&&/path /{print $2; f=0}' | sort -u; }
for f in "$exe" "$app"/Contents/Frameworks/*.dylib; do
    for rp in $(rpaths "$f"); do install_name_tool -delete_rpath "$rp" "$f"; done
done
install_name_tool -add_rpath "@executable_path/../Frameworks" "$exe"

# ---- checks -----------------------------------------------------------------
fail=0
for f in "$exe" "$app"/Contents/Frameworks/*.dylib; do
    echo "== $f"
    lipo -info "$f"
    lipo -archs "$f" | grep -qw arm64  || { echo "MISSING arm64 slice"  >&2; fail=1; }
    lipo -archs "$f" | grep -qw x86_64 || { echo "MISSING x86_64 slice" >&2; fail=1; }
    otool -L "$f" | tail -n +2
    # Only system libraries and bundle-relative names may remain.
    if otool -L "$f" | grep -E $'^\t' | awk '{print $1}' \
        | grep -Ev '^(/usr/lib/|/System/Library/|@rpath/|@loader_path/|@executable_path/)'; then
        echo "REFERENCES A BUILD-MACHINE PATH" >&2; fail=1
    fi
    # Deployment target: every slice must be <= 11.0.
    otool -l "$f" | awk '/LC_BUILD_VERSION/{f=1} f&&/minos/{print "   minos", $2; f=0}'
done
[ "$fail" = 0 ] || exit 1

# Ad-hoc sign inside-out (install_name_tool broke the linker's signature),
# then verify.
for lib in "$app"/Contents/Frameworks/*.dylib; do codesign --force --sign - "$lib"; done
codesign --force --sign - "$exe"
codesign --force --sign - "$app"
codesign --verify --deep --strict --verbose=2 "$app"

# DMG with the usual drag-to-Applications shortcut, and the licence beside it.
cp "$here/LICENSE" "$stage/LICENSE.txt"
ln -s /Applications "$stage/Applications"
dmg="$here/dist/rapidgen-$ver-macos-universal.dmg"
rm -f "$dmg"
hdiutil create -volname "rapidgen $ver" -srcfolder "$stage" -ov -format UDZO "$dmg"
rm "$stage/Applications"
echo "built $dmg"
echo "app: $app"
