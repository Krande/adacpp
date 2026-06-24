# Manifold (Apache-2.0, github.com/elalish/manifold) — fast, robust mesh-boolean (CSG) kernel,
# the OCC-free CSG engine adopted by OpenSCAD + Blender. Used by the libtess2 path to boolean
# tessellated solid operands (ngeom_boolean.cpp). FetchContent like the wasm OCCT toolkits.
#
# Serial build (MANIFOLD_PAR off -> no TBB) for a dependency-light, wasm-safe library; tests /
# bindings / cross-section (Clipper2) all off — we only need the 3D Boolean on MeshGL.
include(FetchContent)
set(MANIFOLD_PAR OFF CACHE BOOL "" FORCE)
set(MANIFOLD_TEST OFF CACHE BOOL "" FORCE)
set(MANIFOLD_CBIND OFF CACHE BOOL "" FORCE)
set(MANIFOLD_PYBIND OFF CACHE BOOL "" FORCE)
set(MANIFOLD_JSBIND OFF CACHE BOOL "" FORCE)
set(MANIFOLD_EXPORT OFF CACHE BOOL "" FORCE)
set(MANIFOLD_DEBUG OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    manifold
    GIT_REPOSITORY https://github.com/elalish/manifold.git
    GIT_TAG v3.5.1
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(manifold)
