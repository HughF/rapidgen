# Vendored third-party code

pipegen itself is GPL-3.0-or-later (see `../LICENSE`). The files in this
directory are **not** ours and keep their own licences, all of which are
GPL-compatible:

| File | Origin | Licence |
|---|---|---|
| `nuklear.h` | [Immediate-Mode-UI/Nuklear](https://github.com/Immediate-Mode-UI/Nuklear) by Micha Mettke | MIT **or** Unlicense (public domain), at your option. Embeds `stb_textedit`, `stb_truetype` and `stb_rect_pack` by Sean Barrett (public domain / MIT) and ProggyClean.ttf by Tristan Grimmer (MIT) |
| `nuklear_sdl_renderer.h` | the same project's SDL2 renderer backend | same as `nuklear.h` |

SDL2 is linked, not vendored, and is under the zlib licence.

`nuklear_sdl_renderer.h` carries **one local modification**, marked in the file:
upstream hard-codes the vertex index size as 2 bytes in its
`SDL_RenderGeometryRaw` call, which is wrong when `NK_UINT_DRAW_INDEX` is
defined — SDL then reads 32-bit indices as 16-bit and draws garbage. Worth
sending upstream. Do not re-vendor this file without re-applying that fix.
