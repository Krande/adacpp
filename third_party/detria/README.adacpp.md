# Vendored detria

Source: https://github.com/Kimbatt/detria @ master (fetched 2026-07-14), single header, 4559 lines.
License: **WTFPL or MIT, at the user's choice — adacpp takes MIT** (see LICENSE.txt; full texts live
at the bottom of detria.hpp).

Constrained Delaunay triangulation for the `cdt` tessellation track
(`TessTrack::Cdt`). Selection rationale, the full candidate comparison, and the
integration design: `dap/plan/v3/spec_cdt_library_selection.md`.

## Why detria and not artem-ogre/CDT

CDT was the obvious pick (web-ifc uses it) and is **disqualified**: it *silently splits constraint
edges* at Steiner points — verified by compiling it — which breaks the invariant the whole watertight
track depends on (`CdtOpts::pin_boundary`: "constraint edges are never split, so a boundary vertex is
always a loop vertex"). A split frontier is a T-junction is a crack. It also has an open unfixed SEGV
at HEAD. detria **refuses** to split and returns an error code instead.

## Two things you must not break

1. **`-DNDEBUG` is MANDATORY in every build that includes this header.** `DETRIA_ASSERT` does return
   `fail(TE_AssertionFailed{})` as documented, but `detriaAssert()` raises `SIGTRAP`/`__debugbreak`
   FIRST in debug builds — reachable from ordinary input (a Steiner point a denormal off a
   constraint). Release => `ok=false`, exit 0. Debug => `Trace/breakpoint trap`, exit 133, and in
   wasm that poisons the whole module (one bad face kills the other 300k). **emcc does NOT define
   NDEBUG** — only CMake's Release config does. See cmake/deps_detria.cmake.
2. **No-copy aliasing contract on BOTH arrays.** `setPoints` does not copy, and `addOutline`/`addHole`
   alias their index spans too. Points *and* index arrays must outlive `triangulate()`.

Never `-ffast-math`: the bundled Shewchuk predicates depend on strict IEEE-754. (Wasm is *safer* than
native x86-32 here — no x87 excess precision — so this is not a wasm-specific concern.)

Do not edit detria.hpp except for documented patches; record any here.
