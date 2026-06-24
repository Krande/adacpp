# Vendored libtess2

Source: https://github.com/memononen/libtess2 @ 8dbd6483e920311a58c9af10a10beb278efebc36
License: SGI Free Software License B (see LICENSE).

Vendored (not FetchContent) so native and wasm builds share one source tree with no
network at build time. Drives adacpp's step2glb-style boundary triangulation path
(see cmake/deps_libtess2.cmake + src/visit/tessellate_libtess2.*, and the design in
dap/plan/v3/notes_libtess2_and_geom_streaming.md). Do not edit the Source/ files except
for documented fail-soft patches.
