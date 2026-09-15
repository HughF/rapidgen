# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Hugh Frater
#
# rapidgen — ABB S4 spray program generator
#
#   make            build ./rapidgen (the editor, and the command line)
#   make test       build and run the unit tests (ASan + UBSan)
#   make debug      sanitised build of the program itself
#   make run        build and open the editor
#   make example    run the examples/ jobs into examples/out/
#   make windows        cross-compile rapidgen.exe with mingw-w64
#   make windows-dist   exe + SDL2.dll + docs, zipped, in dist/
#   make appimage       portable Linux x86_64 AppImage in dist/
#   make release        windows-dist + appimage + dist/SHA256SUMS.txt
#   make clean
#
# Dependencies: SDL2 only. Nuklear is vendored in third_party/.
#   Debian/Ubuntu   sudo apt install libsdl2-dev
#   Arch            sudo pacman -S sdl2
#   macOS           brew install sdl2
#
# The core — everything the tests exercise — needs only a C99 compiler.

CC      ?= cc
UNAME   := $(shell uname -s)

SRC_DIR  = src
TEST_DIR = tests
OBJ_DIR  = build

WARN    = -Wall -Wextra -Wshadow -Wpointer-arith -Wstrict-prototypes \
          -Wno-unused-parameter

CFLAGS  = -std=c99 -O2 $(WARN) -MMD -MP -I$(SRC_DIR) -Ithird_party
LDLIBS  = -lm

# Nuklear's single-header implementation trips these in code we do not own.
NK_CFLAGS = -Wno-unused-function -Wno-sign-compare -Wno-implicit-fallthrough

ifeq ($(UNAME),Darwin)
    CFLAGS += -D_DARWIN_C_SOURCE
else
    CFLAGS += -D_DEFAULT_SOURCE
endif

SDL_CFLAGS := $(shell sdl2-config --cflags 2>/dev/null)
SDL_LIBS   := $(shell sdl2-config --libs 2>/dev/null)

# ---------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------

# The core: no SDL, no Nuklear. The tests and the command line use these.
CORE = rg_text rg_robot rg_dxf rg_shape rg_pattern rg_job rg_plan rg_rapid rg_report

ifeq ($(OS),Windows_NT)
    PLAT = plat_win32
else
    PLAT = plat_posix
endif

UI   = rg_theme rg_config rg_ui rg_ui_draw rg_ui_pages rg_ui_dialogs rg_main

OBJS   = $(addprefix $(OBJ_DIR)/,$(addsuffix .o,$(CORE) $(PLAT) $(UI)))
TARGET = rapidgen
TESTS  = test_robot test_dxf test_shape test_pattern test_job test_plan test_rapid

.PHONY: all clean distclean test debug run example windows windows-dist appimage release

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $@ $(SDL_LIBS) $(LDLIBS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) $(SDL_CFLAGS) $(NK_CFLAGS) -c $< -o $@

$(OBJ_DIR):
	@mkdir -p $(OBJ_DIR)

-include $(OBJS:.o=.d)

# ---------------------------------------------------------------------
# Tests — always sanitised. A test that only passes without ASan has not
# passed.
# ---------------------------------------------------------------------

TEST_CFLAGS = -std=c99 -O1 -g $(WARN) -I$(SRC_DIR) \
              -fsanitize=address,undefined -fno-omit-frame-pointer \
              -fno-sanitize-recover=undefined
ifeq ($(UNAME),Darwin)
    TEST_CFLAGS += -D_DARWIN_C_SOURCE
else
    TEST_CFLAGS += -D_DEFAULT_SOURCE
endif

TEST_SRCS = $(addprefix $(SRC_DIR)/,$(addsuffix .c,$(CORE))) $(SRC_DIR)/plat_posix.c

test:
	@mkdir -p $(OBJ_DIR)/testout
	@fail=0; \
	for t in $(TESTS); do \
	    $(CC) $(TEST_CFLAGS) $(TEST_DIR)/$$t.c $(TEST_SRCS) $(LDLIBS) \
	        -o $(OBJ_DIR)/$$t || exit 1; \
	    $(OBJ_DIR)/$$t || fail=1; \
	done; \
	if [ $$fail -ne 0 ]; then echo "TESTS FAILED"; exit 1; fi; \
	echo "all tests passed"

debug: CFLAGS += -g -O0 -fsanitize=address,undefined
debug: LDLIBS += -fsanitize=address,undefined
debug: clean $(TARGET)

run: $(TARGET)
	./$(TARGET)

example: $(TARGET)
	@mkdir -p examples/out
	./$(TARGET) examples/tank-bands.rgj --out examples/out
	./$(TARGET) examples/tank-drawing.rgj --out examples/out
	./$(TARGET) examples/panel-flat.rgj --out examples/out

# ---------------------------------------------------------------------
# Windows cross-build (mingw-w64)
#
#   sudo pacman -S --needed mingw-w64-gcc          (Arch; apt: mingw-w64)
#   tools/win/get-sdl2.sh                          (downloads and unpacks)
#
# Arch has no mingw pkg-config and no mingw SDL2 package, so the SDK is
# pointed at by path rather than discovered. Windows objects go in their own
# directory: sharing build/ with the native build links host .o files into
# the .exe and fails in confusing ways.
# ---------------------------------------------------------------------

SDL2_VER   ?= 2.32.10
SDL2_MINGW ?= tools/win/SDL2-$(SDL2_VER)/x86_64-w64-mingw32

WIN_CC     = x86_64-w64-mingw32-gcc
WIN_RC     = x86_64-w64-mingw32-windres
WIN_OBJ    = $(OBJ_DIR)/win
WIN_TARGET = rapidgen.exe
WIN_RES    = $(WIN_OBJ)/rapidgen_res.o
WIN_DIST   = dist/rapidgen-$(RG_VERSION)-win64

# The version in the file's Properties tab is read from the same header the
# program prints, so the two cannot disagree. Leading zeros are stripped: the
# resource compiler reads 08 as a malformed octal constant.
RG_VERSION := $(shell sed -n 's/.*RAPIDGEN_VERSION "\([^"]*\)".*/\1/p' \
                      $(SRC_DIR)/rg_version.h)
RG_VER_A   := $(shell echo $(RG_VERSION) | cut -d. -f1 | sed 's/^0*//')
RG_VER_B   := $(shell echo $(RG_VERSION) | cut -d. -f2 | sed 's/^0*//')
RG_VER_C   := $(shell echo $(RG_VERSION) | cut -d. -f3 | sed 's/^0*//')
# A second release on one day is YYYY.MM.DD.N; a plain date gives 0.
RG_VER_D   := $(or $(shell echo $(RG_VERSION) | cut -s -d. -f4 | sed 's/^0*//'),0)

WIN_OBJS = $(addprefix $(WIN_OBJ)/,$(addsuffix .o,$(CORE) plat_win32 $(UI)))

# _USE_MATH_DEFINES: mingw's math.h hides M_PI in strict C99.
# __USE_MINGW_ANSI_STDIO: mingw's own printf, which has %zu.
# Both SDL include paths: the program says <SDL2/SDL.h>, and the vendored
# nuklear_sdl_renderer.h says <SDL.h>.
WIN_CFLAGS  = -std=c99 -O2 $(WARN) -MMD -MP -I$(SRC_DIR) -Ithird_party \
              -D_USE_MATH_DEFINES -D__USE_MINGW_ANSI_STDIO=1 \
              -I$(SDL2_MINGW)/include -I$(SDL2_MINGW)/include/SDL2

# -mwindows: no console window behind the editor; the command line
# reattaches to the console it was started from. -static-libgcc so the only
# DLL to ship is SDL2's.
WIN_LDFLAGS = -mwindows -static-libgcc -L$(SDL2_MINGW)/lib
WIN_LIBS    = -lmingw32 -lSDL2main -lSDL2 -lshell32 -lsetupapi -luuid -lm

windows: $(WIN_TARGET)

$(WIN_TARGET): $(WIN_OBJS) $(WIN_RES)
	@test -f $(SDL2_MINGW)/lib/libSDL2.a || { \
	    echo "No SDL2 mingw SDK at $(SDL2_MINGW)"; \
	    echo "Run tools/win/get-sdl2.sh, or set SDL2_MINGW=<dir>"; \
	    exit 1; }
	$(WIN_CC) $(WIN_OBJS) $(WIN_RES) -o $@ $(WIN_LDFLAGS) $(WIN_LIBS)

$(WIN_OBJ)/%.o: $(SRC_DIR)/%.c | $(WIN_OBJ)
	$(WIN_CC) $(WIN_CFLAGS) $(NK_CFLAGS) -c $< -o $@

$(WIN_RES): tools/win/rapidgen.rc tools/win/rapidgen.manifest tools/win/rapidgen.ico \
            $(SRC_DIR)/rg_version.h | $(WIN_OBJ)
	$(WIN_RC) -I tools/win -I $(SRC_DIR) \
	    -DRG_VER_A=$(RG_VER_A) -DRG_VER_B=$(RG_VER_B) -DRG_VER_C=$(RG_VER_C) \
	    -DRG_VER_D=$(RG_VER_D) \
	    -o $@ tools/win/rapidgen.rc

$(WIN_OBJ):
	@mkdir -p $(WIN_OBJ)

-include $(WIN_OBJS:.o=.d)

windows-dist: $(WIN_TARGET)
	@rm -rf $(WIN_DIST)
	@mkdir -p $(WIN_DIST)
	cp $(WIN_TARGET) $(WIN_DIST)/
	cp $(SDL2_MINGW)/bin/SDL2.dll $(WIN_DIST)/
	cp LICENSE $(WIN_DIST)/LICENSE.txt
	cp README.md $(WIN_DIST)/README.md
	cp -r examples $(WIN_DIST)/examples
	rm -rf $(WIN_DIST)/examples/out
	cd dist && zip -qr $(notdir $(WIN_DIST)).zip $(notdir $(WIN_DIST))
	@echo "packaged dist/$(notdir $(WIN_DIST)).zip"

# ---------------------------------------------------------------------
# Portable Linux binary (AppImage) — built against an older glibc inside
# bubblewrap. See tools/linux/make-appimage.sh.
# ---------------------------------------------------------------------

appimage:
	tools/linux/make-appimage.sh

release: windows-dist appimage
	cd dist && sha256sum *.zip *.AppImage > SHA256SUMS.txt
	@ls -1 dist

# clean leaves dist/ alone: the AppImage build runs `make clean` inside its
# build box, and a release assembles both packages into the same dist/.
clean:
	rm -rf $(OBJ_DIR) $(TARGET) $(WIN_TARGET)

distclean: clean
	rm -rf dist examples/out
