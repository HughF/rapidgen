# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Hugh Frater
#
# rapidgen — ABB S4 spray program generator
#
#   make            build ./rapidgen
#   make test       build and run the unit tests (ASan + UBSan)
#   make debug      sanitised build of the program itself
#   make example    run the examples/ jobs into examples/out/
#   make windows        cross-compile rapidgen.exe with mingw-w64
#   make windows-dist   exe + docs, zipped, in dist/
#   make clean
#
# No dependencies beyond a C99 compiler and libm. The planned viewer will add
# SDL2 and vendored Nuklear, as pipegen and svpview have.

CC      ?= cc
UNAME   := $(shell uname -s)

SRC_DIR  = src
TEST_DIR = tests
OBJ_DIR  = build

WARN    = -Wall -Wextra -Wshadow -Wpointer-arith -Wstrict-prototypes \
          -Wno-unused-parameter

CFLAGS  = -std=c99 -O2 $(WARN) -MMD -MP -I$(SRC_DIR)
LDLIBS  = -lm

ifeq ($(UNAME),Darwin)
    CFLAGS += -D_DARWIN_C_SOURCE
else
    CFLAGS += -D_DEFAULT_SOURCE
endif

# ---------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------

# The core: no I/O beyond whole files, no UI. The tests use exactly these.
CORE = rg_text rg_robot rg_dxf rg_shape rg_job rg_plan rg_rapid rg_report

ifeq ($(OS),Windows_NT)
    PLAT = plat_win32
else
    PLAT = plat_posix
endif

OBJS   = $(addprefix $(OBJ_DIR)/,$(addsuffix .o,$(CORE) $(PLAT) rg_main))
TARGET = rapidgen
TESTS  = test_robot test_dxf test_shape test_job test_plan test_rapid

.PHONY: all clean distclean test debug example windows windows-dist

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDLIBS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

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

example: $(TARGET)
	@mkdir -p examples/out
	./$(TARGET) examples/tank-bands.rgj --out examples/out
	./$(TARGET) examples/tank-drawing.rgj --out examples/out

# ---------------------------------------------------------------------
# Windows cross-build (mingw-w64)
#
#   sudo pacman -S --needed mingw-w64-gcc          (Arch; apt: mingw-w64)
#
# A console program: it is run from a command prompt or a script. Windows
# objects go in their own directory: sharing build/ with the native build
# links host .o files into the .exe and fails in confusing ways.
# ---------------------------------------------------------------------

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

WIN_OBJS = $(addprefix $(WIN_OBJ)/,$(addsuffix .o,$(CORE) plat_win32 rg_main))

# _USE_MATH_DEFINES: mingw's math.h hides M_PI in strict C99.
# __USE_MINGW_ANSI_STDIO: mingw's own printf, which has %zu.
WIN_CFLAGS  = -std=c99 -O2 $(WARN) -MMD -MP -I$(SRC_DIR) \
              -D_USE_MATH_DEFINES -D__USE_MINGW_ANSI_STDIO=1
WIN_LDFLAGS = -static-libgcc
WIN_LIBS    = -lshell32 -lm

windows: $(WIN_TARGET)

$(WIN_TARGET): $(WIN_OBJS) $(WIN_RES)
	$(WIN_CC) $(WIN_OBJS) $(WIN_RES) -o $@ $(WIN_LDFLAGS) $(WIN_LIBS)

$(WIN_OBJ)/%.o: $(SRC_DIR)/%.c | $(WIN_OBJ)
	$(WIN_CC) $(WIN_CFLAGS) -c $< -o $@

$(WIN_RES): tools/win/rapidgen.rc tools/win/rapidgen.manifest $(SRC_DIR)/rg_version.h | $(WIN_OBJ)
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
	cp LICENSE $(WIN_DIST)/LICENSE.txt
	cp README.md $(WIN_DIST)/README.md
	cp -r examples $(WIN_DIST)/examples
	rm -rf $(WIN_DIST)/examples/out
	cd dist && zip -qr $(notdir $(WIN_DIST)).zip $(notdir $(WIN_DIST))
	@echo "packaged dist/$(notdir $(WIN_DIST)).zip"

clean:
	rm -rf $(OBJ_DIR) $(TARGET) $(WIN_TARGET)

distclean: clean
	rm -rf dist examples/out
