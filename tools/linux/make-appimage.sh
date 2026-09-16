#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Hugh Frater
#
# make-appimage.sh — build a portable Linux binary.
#
# The problem this solves is glibc, not packaging. A binary built on a
# rolling-release machine binds symbol versions that do not exist anywhere
# else: on the development box it wants cfsetispeed@GLIBC_2.42 and
# sqrtf@GLIBC_2.43, so it refuses to start on every stable distribution there
# is. Symbol versions are chosen by the linker against the host's libc, so no
# amount of source-level care avoids it. The only fix is to build against an
# older glibc.
#
# That is done here without root and without a container runtime: an Ubuntu
# 22.04 base filesystem is unpacked and entered with bubblewrap, which needs
# only unprivileged user namespaces. Ubuntu 22.04 carries glibc 2.35 and
# SDL 2.0.20 — the SDL floor is 2.0.18, below which SDL_RenderGeometryRaw
# does not exist and the vendored Nuklear renderer will not link.
#
#   ./tools/linux/make-appimage.sh
#   -> dist/rapidgen-<version>-x86_64.AppImage
#
# Everything downloaded is cached in tools/linux/.cache, which is ignored by
# git. Delete it to start clean.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
CACHE="$HERE/.cache"
ROOTFS="$CACHE/rootfs"

UBUNTU_URL="https://cdimage.ubuntu.com/ubuntu-base/releases/22.04/release/ubuntu-base-22.04-base-amd64.tar.gz"
TOOL_URL="https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-x86_64.AppImage"
SDL_VER="2.32.10"
SDL_URL="https://github.com/libsdl-org/SDL/releases/download/release-$SDL_VER/SDL2-$SDL_VER.tar.gz"

VERSION="$(sed -n 's/.*RAPIDGEN_VERSION "\([^"]*\)".*/\1/p' "$ROOT/src/rg_version.h")"
OUT="$ROOT/dist/rapidgen-$VERSION-x86_64.AppImage"

command -v bwrap >/dev/null || { echo "need bubblewrap (pacman -S bubblewrap)"; exit 1; }

mkdir -p "$CACHE" "$ROOT/dist"

# ---- the build filesystem ------------------------------------------------

if [ ! -f "$CACHE/ubuntu-base.tar.gz" ]; then
    echo "fetching the Ubuntu 22.04 base filesystem"
    curl -fL --progress-bar -o "$CACHE/ubuntu-base.tar.gz" "$UBUNTU_URL"
fi
if [ ! -f "$CACHE/appimagetool" ]; then
    echo "fetching appimagetool"
    curl -fL --progress-bar -o "$CACHE/appimagetool" "$TOOL_URL"
    chmod +x "$CACHE/appimagetool"
fi

if [ ! -d "$ROOTFS" ]; then
    echo "unpacking the base filesystem"
    mkdir -p "$ROOTFS"
    # --no-same-owner because this runs as an ordinary user; device nodes are
    # skipped for the same reason and bubblewrap binds the host's /dev anyway.
    tar xzf "$CACHE/ubuntu-base.tar.gz" -C "$ROOTFS" --no-same-owner \
        --exclude='dev/*'
    cp /etc/resolv.conf "$ROOTFS/etc/resolv.conf"
fi

# Run a command inside the build filesystem, as its root, with the source
# tree visible at /src.
inbox() {
    bwrap --bind "$ROOTFS" / \
          --dev-bind /dev /dev \
          --proc /proc \
          --tmpfs /tmp \
          --bind "$ROOT" /src \
          --uid 0 --gid 0 \
          --unshare-user --unshare-ipc --unshare-uts \
          --share-net \
          --setenv PATH /usr/sbin:/usr/bin:/sbin:/bin \
          --setenv HOME /root \
          --setenv DEBIAN_FRONTEND noninteractive \
          --chdir /src \
          /bin/bash -euo pipefail -c "$1"
}

# A stamp, not a check for one of the installed files: apt considers a package
# installed whether or not its files are still there, so deleting a binary to
# force this step leaves apt convinced there is nothing to do and the compiler
# missing.
if [ ! -f "$ROOTFS/.deps-installed" ]; then
    echo "installing the build dependencies"
    # APT::Sandbox::User=root because apt drops privileges to _apt, which
    # cannot work inside a user namespace where everything is already mapped
    # to root — it fails with a bare "http returned an error code (112)".
    # ForceIPv4 because the mirror resolves to IPv6 first and a host without
    # an IPv6 route waits for the timeout on every package.
    APT_OPTS='-o APT::Sandbox::User=root -o Acquire::ForceIPv4=true'
    # The X11, Wayland and audio packages are here for their *headers*: SDL is
    # built from source below and opens all of them with dlopen at runtime, so
    # none of them is linked into the result.
    inbox "apt-get $APT_OPTS -qq update &&
           apt-get $APT_OPTS -qq install -y --no-install-recommends \
               gcc make pkg-config file desktop-file-utils ca-certificates \
               libx11-dev libxext-dev libxcursor-dev libxinerama-dev \
               libxi-dev libxfixes-dev libxrandr-dev libxss-dev \
               libxxf86vm-dev libwayland-dev libxkbcommon-dev \
               libegl1-mesa-dev libgl1-mesa-dev libasound2-dev libpulse-dev \
               libudev-dev libdbus-1-dev"
    touch "$ROOTFS/.deps-installed"
fi

# ---- SDL, built the way it is meant to be redistributed ------------------
#
# Not the distribution's libSDL2. Debian and Ubuntu build it with every
# backend linked directly, so their library carries hard NEEDED entries for
# nineteen X11, Wayland, DRM and audio libraries — libdecor-0.so.0 among
# them, which plenty of systems do not have. The loader resolves all of those
# before main() runs, so one missing library on the target machine and the
# program does not start at all, which is exactly the failure an AppImage
# exists to prevent.
#
# Built from source with the *-shared options instead, SDL opens each backend
# with dlopen when it needs it and does without the ones that are absent.
# That is what upstream ships and what makes a bundled SDL safe to carry.

if [ ! -f "$CACHE/sdl2-$SDL_VER.tar.gz" ]; then
    echo "fetching SDL $SDL_VER"
    curl -fL --progress-bar -o "$CACHE/sdl2-$SDL_VER.tar.gz" "$SDL_URL"
fi

if [ ! -f "$ROOTFS/opt/sdl2/lib/libSDL2.so" ]; then
    echo "building SDL $SDL_VER (a few minutes, once)"
    mkdir -p "$ROOTFS/build"
    tar xzf "$CACHE/sdl2-$SDL_VER.tar.gz" -C "$ROOTFS/build"
    inbox "cd /build/SDL2-$SDL_VER &&
           ./configure --prefix=/opt/sdl2 --disable-static \
               --enable-x11-shared --enable-wayland-shared \
               --enable-alsa-shared --enable-pulseaudio-shared \
               --disable-video-vulkan >/dev/null &&
           make -j\"\$(nproc)\" >/dev/null && make install >/dev/null &&
           echo '  SDL built'"
fi

# ---- build ---------------------------------------------------------------

echo "building against glibc $(inbox 'ldd --version | head -1' | grep -o '[0-9]\+\.[0-9]\+$')"
inbox 'make -C /src clean >/dev/null &&
       make -C /src -j"$(nproc)" \
            SDL_CFLAGS="-I/opt/sdl2/include/SDL2 -D_REENTRANT" \
            SDL_LIBS="-L/opt/sdl2/lib -lSDL2 -Wl,-rpath,\$ORIGIN/../lib" \
            >/dev/null && echo built'

# ---- assemble the AppDir -------------------------------------------------

APPDIR="$ROOT/dist/rapidgen.AppDir"
rm -rf "$APPDIR"
mkdir -p "$APPDIR/usr/bin" "$APPDIR/usr/lib" \
         "$APPDIR/usr/share/applications" \
         "$APPDIR/usr/share/icons/hicolor/256x256/apps" \
         "$APPDIR/usr/share/rapidgen" \
         "$APPDIR/usr/share/doc/rapidgen"

cp "$ROOT/rapidgen"                "$APPDIR/usr/bin/"
cp "$HERE/AppRun"                 "$APPDIR/AppRun"
cp "$HERE/rapidgen.desktop"        "$APPDIR/rapidgen.desktop"
cp "$HERE/rapidgen.desktop"        "$APPDIR/usr/share/applications/"
cp "$HERE/rapidgen.png"            "$APPDIR/rapidgen.png"
cp "$HERE/rapidgen.png"            "$APPDIR/usr/share/icons/hicolor/256x256/apps/"
cp "$ROOT/LICENSE"                "$APPDIR/usr/share/doc/rapidgen/"
cp "$ROOT/README.md"              "$APPDIR/usr/share/doc/rapidgen/"
chmod +x "$APPDIR/AppRun"

# A face, so the interface looks the same everywhere.
#
# Without one the program falls back to Nuklear's built-in bitmap font on any
# machine with no fonts installed — which is what a minimal image looks like,
# and exactly the sort of machine an AppImage gets carried onto. rg_ui.c looks
# here first, relative to the executable, because an AppImage is mounted
# somewhere different every time it runs.
cp "$HERE/font/DejaVuSans.ttf"      "$APPDIR/usr/share/rapidgen/"
cp "$HERE/font/DejaVu-LICENSE.txt"  "$APPDIR/usr/share/doc/rapidgen/"

# Bundle SDL2 and nothing else.
#
# SDL2 is the one dependency that is not on every machine, and it is the one
# whose absence stops the program dead. Everything it talks to — the graphics
# driver, X11, Wayland, ALSA, PulseAudio — is loaded by SDL at runtime with
# dlopen and belongs to the host: bundling a graphics stack is how an
# AppImage ends up refusing to start on the machine it was meant to rescue.
# libc, libm and libstdc++ stay unbundled too; they came from the old build
# filesystem, so the host's are newer by construction.
echo "bundling SDL2"
cp -L "$ROOTFS/opt/sdl2/lib/libSDL2-2.0.so.0" "$APPDIR/usr/lib/"
echo "  libSDL2-2.0.so.0 ($(inbox 'objdump -p /opt/sdl2/lib/libSDL2-2.0.so.0 | grep -c NEEDED') direct dependencies, the rest dlopened)"

# ---- package -------------------------------------------------------------

echo "packaging"
rm -f "$OUT"
# --appimage-extract-and-run: appimagetool is itself an AppImage, and mounting
# one needs FUSE, which is not a given on a build machine.
ARCH=x86_64 "$CACHE/appimagetool" --appimage-extract-and-run \
    --no-appstream "$APPDIR" "$OUT" 2>&1 | grep -vi "^$" | tail -3

chmod +x "$OUT"
rm -rf "$APPDIR"

echo
echo "$OUT"
echo "glibc floor: $(inbox 'ldd --version | head -1' | grep -o '[0-9]\+\.[0-9]\+$')  (Ubuntu 22.04, Debian 12, Fedora 36 and newer)"
