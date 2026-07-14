# detria — Kimbatt/detria, single-header constrained Delaunay triangulation.
# Vendored at third_party/detria (WTFPL or MIT; adacpp takes MIT) so the build needs no network and
# the same header compiles for native AND wasm.
#
# Powers the `cdt` tessellation track (TessTrack::Cdt): trim loops become CONSTRAINT EDGES (never
# split) and the surface curvature grid becomes interior Steiner points — one boundary-first path,
# which is what OCC's BRepMesh and truck do. libtess2 is a winding-rule tessellator and takes no
# Steiner points, which is why it needs a separate UV-grid fast path, and that grid path is the only
# remaining source of shared-edge cracks. Selection rationale + the full candidate comparison:
# dap/plan/v3/spec_cdt_library_selection.md.
#
# Header-only: exposes DETRIA_INCLUDE_DIR only, no sources to compile.
set(DETRIA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/detria)
set(DETRIA_INCLUDE_DIR ${DETRIA_DIR})

# NDEBUG IS MANDATORY, NOT AN OPTIMISATION. detria's DETRIA_ASSERT returns a TE_AssertionFailed
# error as documented, but detriaAssert() raises SIGTRAP/__debugbreak FIRST when NDEBUG is unset —
# reachable from ordinary input (a Steiner point a denormal off a constraint). Release => ok=false,
# exit 0; Debug => Trace/breakpoint trap, exit 133. In wasm a trap poisons the whole module, so one
# bad face would kill the other 300k in a conversion. emcc does NOT define NDEBUG (only CMake's
# Release config does), so force it here for every config rather than trusting the build type.
add_compile_definitions(NDEBUG)
